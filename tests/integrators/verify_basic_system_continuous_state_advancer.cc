#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"

#include "system_continuous_state_integration_access.h"

void VerifyBasicSystemHeldTorqueSynchronization();

namespace {

using orvd::integrators::ContinuousStateErrorTolerances;
using orvd::integrators::ContinuousStateNumericalFailure;
using orvd::integrators::NoCallTimeAppliedForces;
using orvd::integrators::SystemContinuousStateAdvancer;
using orvd::integrators::internal::CoordinateIntegrationDiagnostics;
using orvd::integrators::internal::NewmarkConfiguration;
using orvd::integrators::internal::SystemContinuousStateIntegrationAccess;
using orvd::integrators::internal::SystemContinuousStateIntegrationConfiguration;
using orvd::integrators::internal::SystemContinuousStateIntegrationRecipe;
using orvd::integrators::internal::ZhaiConfiguration;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::CompiledSystemPlan;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;
using orvd::system_assembly::SystemRuntimeContext;

using Recipe = SystemContinuousStateIntegrationRecipe;
using Configuration = SystemContinuousStateIntegrationConfiguration;
using Failure = ContinuousStateNumericalFailure;

// The factory borrows both the immutable system and its compiled plan.
template <class SystemArgument, class PlanArgument>
concept CanMakeConfiguredAdvancer = requires(
    Configuration configuration, SystemRuntimeContext& accepted) {
    SystemContinuousStateIntegrationAccess::Make(
        std::move(configuration), std::declval<SystemArgument>(),
        std::declval<PlanArgument>(), accepted, NoCallTimeAppliedForces{});
};
static_assert(CanMakeConfiguredAdvancer<const SystemInstance&, const CompiledSystemPlan&>);
static_assert(CanMakeConfiguredAdvancer<SystemInstance&, CompiledSystemPlan&>);
static_assert(!CanMakeConfiguredAdvancer<SystemInstance&&, const CompiledSystemPlan&>);
static_assert(!CanMakeConfiguredAdvancer<const SystemInstance&&, const CompiledSystemPlan&>);
static_assert(!CanMakeConfiguredAdvancer<const SystemInstance&, CompiledSystemPlan&&>);
static_assert(!CanMakeConfiguredAdvancer<const SystemInstance&, const CompiledSystemPlan&&>);

void Expect(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void Near(double actual, double expected, double tolerance, const std::string& message) {
    Expect(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
           message + ": actual=" + std::to_string(actual) +
               ", expected=" + std::to_string(expected));
}

template <typename Exception = std::invalid_argument, typename Operation>
void Throws(Operation&& operation, const std::string& message) {
    bool caught = false;
    try {
        operation();
    } catch (const Exception&) {
        caught = true;
    }
    Expect(caught, message);
}

RigidBodyInertiaParameters Inertia(double mass = 1.0,
                                  Eigen::Vector3d moments = Eigen::Vector3d::Constant(0.3)) {
    RigidBodyInertiaParameters result;
    result.mass_kilograms = mass;
    result.center_of_mass_in_body_frame.setZero();
    result.unit_inertia_moments = moments;
    result.unit_inertia_products.setZero();
    return result;
}

Eigen::Vector4d QuaternionEntries(const Eigen::Matrix3d& rotation) {
    const Eigen::Quaterniond quaternion(rotation);
    return {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
}

Eigen::Matrix3d QuaternionRotation(const Eigen::Vector4d& q) {
    return Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
}

Eigen::Matrix3d RpyRotation(const Eigen::Vector3d& q) {
    return (Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
}

double AngleError(const Eigen::Matrix3d& actual, const Eigen::Matrix3d& expected) {
    return Eigen::AngleAxisd(expected.transpose() * actual).angle();
}

// These fixtures assemble the real multibody/force/system chain. Their exact
// solutions are used only for initial data and final error measurement.
struct SystemFixture {
    MultibodyModel model;
    std::unique_ptr<orvd::forces::VehicleForcePlan> forces;
    std::unique_ptr<SystemInstance> system;
    std::unique_ptr<CompiledSystemPlan> plan;

    void Finish(orvd::forces::VehicleForceElementCollection elements = {}) {
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();
        forces = std::make_unique<orvd::forces::VehicleForcePlan>(model, std::move(elements));
        const SystemAssemblyDescription description(model, *forces);
        system = std::make_unique<SystemInstance>(description);
        plan = std::make_unique<CompiledSystemPlan>(*system);
    }

    Eigen::VectorXd DefaultPhysicalState() const {
        const auto context = system->CreateDefaultRuntimeContext(0.0);
        Eigen::VectorXd result(system->continuous_state_size());
        system->CopyContinuousState(*context, result);
        return result;
    }
};

struct FreeTopFixture final : SystemFixture {
    RigidBodyHandle body;
    const Eigen::Matrix3d initial_rotation = RpyRotation({0.2, -0.25, 0.3});
    const Eigen::Vector3d translation_velocity{0.2, -0.1, 0.3};
    const Eigen::Vector3d initial_position{0.4, -0.2, 0.6};
    const Eigen::Vector3d initial_body_omega{0.7, 0.0, 0.9};

    FreeTopFixture() {
        body = model.AddRigidBody("free_top", Inertia(2.0, {0.3, 0.3, 0.5}));
        model.DeclareFreeBody(body);
        Finish();
    }

    Eigen::Matrix3d ExactRotation(double t) const {
        const Eigen::Vector3d momentum_per_mass =
            initial_rotation * Eigen::Vector3d(0.3 * 0.7, 0.0, 0.5 * 0.9);
        const double body_precession_rate = (0.5 - 0.3) / 0.3 * 0.9;
        return Eigen::AngleAxisd(momentum_per_mass.norm() / 0.3 * t,
                                  momentum_per_mass.normalized()).toRotationMatrix() *
               initial_rotation *
               Eigen::AngleAxisd(-body_precession_rate * t,
                                  Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

    Eigen::VectorXd ExactPhysicalState(double t) const {
        auto result = DefaultPhysicalState();
        const auto qr = model.GetFreeBodyPositionRange(body);
        const auto vr = model.GetFreeBodyVelocityRange(body);
        const int velocity_start = system->generalized_velocities_state_range().start();
        const Eigen::Matrix3d rotation = ExactRotation(t);
        result.segment<4>(qr.start()) = 1.7 * QuaternionEntries(rotation);
        result.segment<3>(qr.start() + 4) = initial_position + t * translation_velocity;
        const double rate = (0.5 - 0.3) / 0.3 * 0.9;
        const Eigen::Vector3d body_omega(0.7 * std::cos(rate * t),
                                       0.7 * std::sin(rate * t), 0.9);
        result.segment<3>(velocity_start + vr.start()) = rotation * body_omega;
        result.segment<3>(velocity_start + vr.start() + 3) = translation_velocity;
        return result;
    }
};

struct BallRpyFixture final : SystemFixture {
    JointHandle joint;
    const Eigen::Vector3d initial_rpy{0.2, -0.3, 0.4};
    const Eigen::Vector3d world_omega{0.55, -0.35, 0.4};

    BallRpyFixture() {
        const auto body = model.AddRigidBody("ball_body", Inertia());
        joint = model.AddBallRpyJoint("ball", model.world_frame(), model.body_frame(body),
                                      initial_rpy);
        Finish();
    }

    Eigen::Matrix3d ExactRotation(double t) const {
        return Eigen::AngleAxisd(t * world_omega.norm(), world_omega.normalized())
                   .toRotationMatrix() * RpyRotation(initial_rpy);
    }

    Eigen::VectorXd ExactPhysicalState(double t) const {
        auto result = DefaultPhysicalState();
        const auto qr = model.GetJointPositionRange(joint);
        const auto vr = model.GetJointVelocityRange(joint);
        result.segment<3>(qr.start()) = ExactRotation(t).eulerAngles(2, 1, 0).reverse();
        result.segment<3>(system->generalized_velocities_state_range().start() + vr.start()) =
            world_omega;
        return result;
    }
};

orvd::forces::VehicleForceElementCollection SliderElements(
    const MultibodyModel& model, RigidBodyHandle anchor, RigidBodyHandle slider) {
    using namespace orvd::forces;
    VehicleForceElementCollection elements;
    elements.series_spring_viscous_dampers.push_back(SeriesSpringViscousDamper{
        "moving_maxwell", {model.body_frame(anchor)}, {model.body_frame(slider)},
        ForceElementAxis::kLongitudinal, 4.0, 2.0});
    elements.translational_spring_dampers.push_back(TranslationalSpringDamper{
        "nominal_slot", {model.body_frame(anchor)}, {model.body_frame(slider)},
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
    return elements;
}

struct SliderMaxwellFixture final : SystemFixture {
    RigidBodyHandle anchor;
    RigidBodyHandle slider;
    JointHandle joint;

    SliderMaxwellFixture() {
        anchor = model.AddRigidBody("anchor", Inertia());
        slider = model.AddRigidBody("slider", Inertia());
        model.AddWeldJoint("anchor_weld", model.world_frame(), model.body_frame(anchor));
        joint = model.AddPrismaticJoint("slide", model.body_frame(anchor),
                                        model.body_frame(slider), Eigen::Vector3d::UnitX(), 0.0);
        Finish(SliderElements(model, anchor, slider));
    }

    Eigen::VectorXd ExactPhysicalState(double t) const {
        // q'=v, v'=-F, F'=4v-2F, with (q,v,F)(0)=(0.2,0.7,0.4).
        const double omega = std::sqrt(3.0);
        const double a = 0.7;
        const double b = (0.7 - 0.4) / omega;
        const double c = std::cos(omega * t);
        const double s = std::sin(omega * t);
        const double decay = std::exp(-t);
        const double integral_cos = (decay * (-c + omega * s) + 1.0) / 4.0;
        const double integral_sin = (decay * (-s - omega * c) + omega) / 4.0;
        Eigen::VectorXd result(3);
        result << 0.2 + a * integral_cos + b * integral_sin,
                  decay * (a * c + b * s),
                  decay * ((a - omega * b) * c + (b + omega * a) * s);
        return result;
    }
};

Configuration Settings(Recipe recipe, const SystemFixture& fixture, double h,
                       std::size_t budget = 1000000) {
    if (recipe == Recipe::kZhai) return {ZhaiConfiguration{h}, budget};
    Expect(recipe == Recipe::kNewmark, "test requires a basic mechanical recipe");
    const int nq = fixture.model.num_generalized_positions();
    const int nz = fixture.system->series_spring_damper_force_state_range().size();
    NewmarkConfiguration configuration;
    configuration.step_size_seconds = h;
    auto& solver = configuration.nonlinear_solver;
    // These absolute scales use the fixture's metre/radian/second/Newton units.
    // They are solver scales, not public physical-state ODE error tolerances.
    solver.position_correction_scales = Eigen::VectorXd::Constant(nq, 1e-11);
    solver.velocity_correction_scales = Eigen::VectorXd::Constant(nq, 1e-11);
    solver.acceleration_residual_scales = Eigen::VectorXd::Constant(nq, 1e-11);
    solver.internal_state_correction_scales = Eigen::VectorXd::Constant(nz, 1e-11);
    solver.internal_state_residual_scales = Eigen::VectorXd::Constant(nz, 1e-11);
    solver.unknown_reference_scales = Eigen::VectorXd::Ones(nq + nz);
    return {std::move(configuration), budget};
}

std::unique_ptr<SystemContinuousStateAdvancer> Make(
    const SystemFixture& fixture, SystemRuntimeContext& accepted,
    Configuration configuration) {
    return SystemContinuousStateIntegrationAccess::Make(
        std::move(configuration), *fixture.system, *fixture.plan, accepted,
        NoCallTimeAppliedForces{});
}

Eigen::VectorXd Physical(const SystemFixture& fixture,
                         const SystemRuntimeContext& accepted) {
    Eigen::VectorXd result(fixture.system->continuous_state_size());
    fixture.system->CopyContinuousState(accepted, result);
    return result;
}

void Unchanged(const SystemFixture& fixture, const SystemRuntimeContext& accepted,
               double time, const Eigen::VectorXd& state, const std::string& message) {
    Expect(accepted.time_seconds() == time && Physical(fixture, accepted) == state,
           message);
}

CoordinateIntegrationDiagnostics Diagnostics(const SystemContinuousStateAdvancer& advancer) {
    const auto result = SystemContinuousStateIntegrationAccess::CoordinateDiagnostics(advancer);
    Expect(result.has_value(), "basic factory backend exposes coordinate diagnostics");
    return *result;
}

void CheckResetStatistics(const SystemContinuousStateAdvancer& advancer) {
    const auto statistics = advancer.integration_statistics();
    const auto diagnostics = Diagnostics(advancer);
    Expect(statistics.successful_internal_step_count == 0 &&
               statistics.right_hand_side_evaluation_count == 1 &&
               statistics.linear_solver_right_hand_side_evaluation_count == 0 &&
               statistics.nonlinear_solver_iteration_count == 0 &&
               statistics.nonlinear_solver_convergence_failure_count == 0 &&
               statistics.jacobian_evaluation_count == 0 &&
               statistics.linear_solver_setup_count == 0 &&
               diagnostics.startup_step_count == 0 &&
               diagnostics.endpoint_projection_evaluation_count == 0 &&
               diagnostics.endpoint_projection_change_count == 0,
           "successful synchronization resets work/history and counts its initial RHS");
}

void CheckIdentityAndStepHistory(Recipe recipe) {
    SliderMaxwellFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    const auto initial = fixture.ExactPhysicalState(0.0);
    fixture.system->SetContinuousState(*accepted, initial);
    auto advancer = Make(fixture, *accepted, Settings(recipe, fixture, 0.125));
    Expect(SystemContinuousStateIntegrationAccess::ConfiguredRecipe(*advancer) == recipe,
           "factory retains the requested basic method identity");
    CheckResetStatistics(*advancer);
    Unchanged(fixture, *accepted, 0.0, initial, "factory construction preserves accepted state");

    advancer->AdvanceTo(0.125);
    advancer->AdvanceTo(0.25);
    const auto after_two = advancer->integration_statistics();
    const auto first_diagnostics = Diagnostics(*advancer);
    Expect(after_two.successful_internal_step_count == 2 &&
               first_diagnostics.endpoint_projection_evaluation_count == 2 &&
               first_diagnostics.endpoint_projection_change_count == 0,
           "two public calls preserve actual successful-step/projection counts");
    Expect(first_diagnostics.startup_step_count == (recipe == Recipe::kZhai ? 1 : 0),
           "a public-call boundary must not restart an equal-length Zhai step");
    if (recipe == Recipe::kZhai) {
        Expect(after_two.right_hand_side_evaluation_count == 3 &&
                   after_two.linear_solver_right_hand_side_evaluation_count == 0 &&
                   after_two.jacobian_evaluation_count == 0 &&
                   after_two.linear_solver_setup_count == 0 &&
                   after_two.nonlinear_solver_iteration_count == 0 &&
                   after_two.requested_dense_finite_difference_jacobian_worker_count == 0,
               "Zhai owns no Newton resources and evaluates one accepted endpoint RHS per step");
    } else {
        Expect(after_two.jacobian_evaluation_count > 0 &&
                   after_two.jacobian_evaluation_count == after_two.linear_solver_setup_count &&
                   after_two.jacobian_evaluation_count == after_two.nonlinear_solver_iteration_count &&
                   after_two.linear_solver_right_hand_side_evaluation_count ==
                       2 * after_two.jacobian_evaluation_count &&
                   after_two.requested_dense_finite_difference_jacobian_worker_count == 1,
               "Newmark exposes serial full two-unknown Jacobian costs separately from ordinary RHS");
    }

    advancer->AdvanceTo(0.3125);  // A genuine H/2 stop.
    advancer->AdvanceTo(0.4375);  // Return to H starts once more.
    advancer->AdvanceTo(0.5625);  // This second H is a normal recurrence.
    const auto diagnostics = Diagnostics(*advancer);
    Expect(advancer->integration_statistics().successful_internal_step_count == 5 &&
               diagnostics.endpoint_projection_evaluation_count == 5 &&
               diagnostics.startup_step_count == (recipe == Recipe::kZhai ? 3 : 0),
           "actual stop lengths, not public calls, determine Zhai restart history");
}

void CheckDenseFactoryTransaction(Recipe recipe) {
    SliderMaxwellFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    auto plain = fixture.system->CreateDefaultRuntimeContext(0.0);
    const auto initial = fixture.ExactPhysicalState(0.0);
    fixture.system->SetContinuousState(*accepted, initial);
    fixture.system->SetContinuousState(*plain, initial);
    auto advancer = Make(fixture, *accepted, Settings(recipe, fixture, 0.125));
    auto plain_advancer = Make(fixture, *plain, Settings(recipe, fixture, 0.125));
    const std::array times{0.0, 0.03125, 0.125, 0.1875, 0.25};
    const Eigen::MatrixXd samples = advancer->AdvanceToWithDenseStateSamples(0.25, times);
    plain_advancer->AdvanceTo(0.25);
    Expect(samples.rows() == 3 && samples.cols() == 5 &&
               samples.col(0) == initial && samples.col(4) == Physical(fixture, *accepted) &&
               Physical(fixture, *accepted) == Physical(fixture, *plain) &&
               accepted->time_seconds() == 0.25,
           "dense matrix endpoints and accepted physical state form one transaction");
    Near((samples.col(1) - (0.75 * initial + 0.25 * samples.col(2))).norm(), 0.0, 2e-15,
         "dense interior uses the enclosing physical endpoint interval");
    Near((samples.col(3) - 0.5 * (samples.col(2) + samples.col(4))).norm(), 0.0, 2e-15,
         "dense samples straddling numerical intervals use local linear interpolation");
    Expect(advancer->integration_statistics().successful_internal_step_count == 2 &&
               Diagnostics(*advancer).startup_step_count ==
                   Diagnostics(*plain_advancer).startup_step_count,
           "sample times do not become numerical stops or change Zhai history");
}

void CheckInvalidCalls(Recipe recipe) {
    SliderMaxwellFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    const auto initial = fixture.ExactPhysicalState(0.0);
    fixture.system->SetContinuousState(*accepted, initial);
    for (double h : {0.0, -0.125, std::numeric_limits<double>::infinity()}) {
        Throws([&] { static_cast<void>(Make(fixture, *accepted, Settings(recipe, fixture, h))); },
               "factory must validate the explicit mechanical nominal step");
    }
    Throws([&] { static_cast<void>(Make(fixture, *accepted, Settings(recipe, fixture, 0.125, 0))); },
           "zero public execution budget is invalid configuration");
    Throws([&] {
        static_cast<void>(SystemContinuousStateIntegrationAccess::Make(
            recipe, *fixture.system, *fixture.plan, *accepted,
            ContinuousStateErrorTolerances(1e-8, Eigen::VectorXd::Constant(3, 1e-10)),
            NoCallTimeAppliedForces{}));
    }, "old ODE factory cannot invent a mechanical step or solver scales");
    if (recipe == Recipe::kNewmark) {
        auto invalid = Settings(recipe, fixture, 0.125);
        std::get<NewmarkConfiguration>(invalid.method).nonlinear_solver
            .position_correction_scales.resize(2);
        Throws([&] { static_cast<void>(Make(fixture, *accepted, invalid)); },
               "Newmark factory validates coordinate-scale dimensions");
    }
    Unchanged(fixture, *accepted, 0.0, initial, "invalid factory configuration cannot alter accepted state");
    auto advancer = Make(fixture, *accepted, Settings(recipe, fixture, 0.125));
    Throws([&] { advancer->AdvanceTo(-0.1); }, "past target is invalid");
    Throws([&] { advancer->AdvanceTo(std::numeric_limits<double>::quiet_NaN()); },
           "non-finite target is invalid");
    Throws([&] {
        const std::array times{0.0, 0.0625, 0.0625, 0.125};
        static_cast<void>(advancer->AdvanceToWithDenseStateSamples(0.125, times));
    }, "duplicate dense time is invalid before backend entry");
    advancer->AdvanceTo(0.0);
    const std::array same_time{0.0};
    const auto sample = advancer->AdvanceToWithDenseStateSamples(0.0, same_time);
    Expect(sample.cols() == 1 && sample.col(0) == initial, "same-time dense request preserves exact state");
    CheckResetStatistics(*advancer);
    advancer->AdvanceTo(0.125);
    Expect(accepted->time_seconds() == 0.125,
           "invalid public arguments leave the existing backend usable");
}

void CheckBudgetFailure(Recipe recipe, bool dense) {
    SliderMaxwellFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    const auto initial = fixture.ExactPhysicalState(0.0);
    fixture.system->SetContinuousState(*accepted, initial);
    auto advancer = Make(fixture, *accepted, Settings(recipe, fixture, 0.125, 2));
    bool exhausted = false;
    try {
        if (dense) {
            const std::array times{0.0, 0.0625, 0.25, 0.5};
            static_cast<void>(advancer->AdvanceToWithDenseStateSamples(0.5, times));
        } else {
            advancer->AdvanceTo(0.5);
        }
    } catch (const Failure& error) {
        exhausted = error.reason() == Failure::Reason::kAdvanceWorkBudgetExhausted;
    }
    Expect(exhausted, "bounded public advance exposes the numerical work-budget failure reason");
    Unchanged(fixture, *accepted, 0.0, initial,
              "public failure cannot commit any previously successful internal endpoint");
    const auto failed_statistics = advancer->integration_statistics();
    Expect(failed_statistics.successful_internal_step_count == 2 &&
               failed_statistics.right_hand_side_evaluation_count >= 3 &&
               Diagnostics(*advancer).endpoint_projection_evaluation_count == 2,
           "failed public transaction retains all work performed by successful private steps");
    Throws<std::logic_error>([&] { advancer->AdvanceTo(0.0); },
                             "same-time advance is blocked after numerical failure");
    Throws<std::logic_error>([&] { advancer->AdvanceTo(0.125); },
                             "positive advance is blocked until synchronization");
    Expect(advancer->integration_statistics().successful_internal_step_count == 2,
           "blocked calls must not enter the backend");
    advancer->SynchronizeAfterAcceptedContextChange();
    CheckResetStatistics(*advancer);
    Unchanged(fixture, *accepted, 0.0, initial, "synchronization preserves the rollback endpoint");
    advancer->AdvanceTo(0.25);  // Exactly the allowed two steps must succeed.
    Expect(accepted->time_seconds() == 0.25 &&
               advancer->integration_statistics().successful_internal_step_count == 2 &&
               Diagnostics(*advancer).startup_step_count == (recipe == Recipe::kZhai ? 1 : 0),
           "synchronization restores the accepted state and resets numerical history");
}

Eigen::Vector3d OneStep(Recipe recipe, const Eigen::Vector3d& state,
                        double h, double damping, double nominal_force) {
    Eigen::Matrix3d matrix;
    matrix << 0.0, 1.0, 0.0, 0.0, -damping, -1.0, 0.0, 4.0, -2.0;
    const Eigen::Vector3d input(0.0, -nominal_force, 0.0);
    if (recipe == Recipe::kNewmark) {
        return (Eigen::Matrix3d::Identity() - 0.5 * h * matrix).fullPivLu().solve(
            (Eigen::Matrix3d::Identity() + 0.5 * h * matrix) * state + h * input);
    }
    const Eigen::Vector3d derivative = matrix * state + input;
    Eigen::Vector3d result = state + h * derivative;
    result[0] += 0.5 * h * h * derivative[1];
    return result;
}

void CheckContextLocalSynchronization(Recipe recipe) {
    SliderMaxwellFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(1.0);
    auto unrelated = fixture.system->CreateDefaultRuntimeContext(1.0);
    const Eigen::Vector3d initial = fixture.ExactPhysicalState(0.0);
    fixture.system->SetContinuousState(*accepted, initial);
    fixture.system->SetContinuousState(*unrelated, initial);
    const auto component = fixture.system->GetMultibodyComponentView(
        *accepted, fixture.system->multibody_component());
    const auto nominal = fixture.system->GetTranslationalSpringDamperIndexByName("nominal_slot");
    fixture.model.SetPrismaticJointDampingCoefficient(&component.context(), fixture.joint, 0.6);
    fixture.system->SetNominalForce(*accepted, nominal, {1.2, 0.0, 0.0});
    auto advancer = Make(fixture, *accepted, Settings(recipe, fixture, 0.125));
    // Later accepted-context edits must remain invisible to the private trial
    // until explicit synchronization, including the first RHS cached at construction.
    fixture.model.SetPrismaticJointDampingCoefficient(&component.context(), fixture.joint, 1.4);
    fixture.system->SetNominalForce(*accepted, nominal, {-0.7, 0.0, 0.0});
    advancer->AdvanceTo(1.125);
    Near((Physical(fixture, *accepted) - OneStep(recipe, initial, 0.125, 0.6, 1.2)).norm(),
         0.0, 3e-11, "factory imports construction-time damping and nominal force only");

    fixture.system->SetTimeAndContinuousState(*accepted, 1.125, initial);
    advancer->SynchronizeAfterAcceptedContextChange();
    Unchanged(fixture, *accepted, 1.125, initial,
              "explicit input synchronization does not advance or rewrite accepted state");
    CheckResetStatistics(*advancer);
    advancer->AdvanceTo(1.25);
    Near((Physical(fixture, *accepted) - OneStep(recipe, initial, 0.125, 1.4, -0.7)).norm(),
         0.0, 3e-11, "explicit synchronization refreshes state, damping and nominal held force");
    Expect(Diagnostics(*advancer).startup_step_count == (recipe == Recipe::kZhai ? 1 : 0),
           "explicit input synchronization restarts Zhai from the replaced accepted state");
    Unchanged(fixture, *unrelated, 1.0, initial,
              "factory trials and synchronization leave another runtime context untouched");
    Expect(unrelated->nominal_forces().isZero(), "input synchronization is runtime-context local");
}

void CheckRpyFailureTransaction(Recipe recipe) {
    BallRpyFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    Eigen::VectorXd initial(6);
    initial << 0.0, std::numbers::pi / 2.0 - 0.02, 0.0, 0.0, 1.0, 0.0;
    fixture.system->SetContinuousState(*accepted, initial);
    auto advancer = Make(fixture, *accepted, Settings(recipe, fixture, 0.02));
    std::string diagnostic;
    try {
        advancer->AdvanceTo(0.02);
    } catch (const Failure&) {
        throw std::runtime_error("a real RPY domain callback exception was replaced by a numerical reason");
    } catch (const std::exception& error) {
        diagnostic = error.what();
    }
    Expect(diagnostic.find("singular") != std::string::npos,
           "the true coordinate callback refuses an endpoint at RPY gimbal lock");
    Unchanged(fixture, *accepted, 0.0, initial, "RPY endpoint failure leaves accepted q and physical v intact");
    Expect(advancer->integration_statistics().successful_internal_step_count == 0 &&
               advancer->integration_statistics().right_hand_side_evaluation_count ==
                   (recipe == Recipe::kNewmark ? 2 : 1) &&
               Diagnostics(*advancer).endpoint_projection_evaluation_count ==
                   (recipe == Recipe::kZhai ? 1 : 0),
           "RPY failure records the attempted Newmark RHS or Zhai pre-RHS projection without accepting a step");
    Throws<std::logic_error>([&] { advancer->AdvanceTo(0.01); },
                             "real callback failure blocks retry until explicit synchronization");
    const auto regular = fixture.ExactPhysicalState(0.0);
    fixture.system->SetContinuousState(*accepted, regular);
    advancer->SynchronizeAfterAcceptedContextChange();
    CheckResetStatistics(*advancer);
    advancer->AdvanceTo(0.01);
    Expect(accepted->time_seconds() == 0.01, "synchronization recovers from the RPY callback failure");
}

void CheckNewmarkFailureClassification() {
    SliderMaxwellFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    const auto initial = fixture.ExactPhysicalState(0.0);
    fixture.system->SetContinuousState(*accepted, initial);
    auto settings = Settings(Recipe::kNewmark, fixture, 0.125);
    std::get<NewmarkConfiguration>(settings.method).nonlinear_solver.maximum_iterations = 1;
    auto advancer = Make(fixture, *accepted, settings);
    bool exhausted = false;
    try {
        advancer->AdvanceTo(0.125);
    } catch (const Failure& error) {
        exhausted = error.reason() == Failure::Reason::kNonlinearConvergenceFailure;
    }
    Expect(exhausted, "single full-Newton exhaustion has the precise non-repeated public failure reason");
    Unchanged(fixture, *accepted, 0.0, initial, "Newmark solve failure cannot commit a candidate endpoint");
    const auto statistics = advancer->integration_statistics();
    Expect(statistics.successful_internal_step_count == 0 &&
               statistics.nonlinear_solver_iteration_count == 1 &&
               statistics.nonlinear_solver_convergence_failure_count == 1 &&
               statistics.linear_solver_right_hand_side_evaluation_count == 2,
           "one unsuccessful full Newton round remains observable in factory statistics");
    Throws<std::logic_error>([&] { advancer->AdvanceTo(0.01); },
                             "Newton failure blocks the system advancer");
}

void CheckNonFiniteStateClassification(Recipe recipe) {
    SliderMaxwellFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    // The initial RHS is finite (g=4v); the requested step overflows the
    // candidate q/z arithmetic before an endpoint can be accepted.
    const Eigen::Vector3d initial(0.0, std::numeric_limits<double>::max() / 16.0, 0.0);
    fixture.system->SetContinuousState(*accepted, initial);
    auto advancer = Make(fixture, *accepted, Settings(recipe, fixture, 32.0));
    bool refused = false;
    try {
        advancer->AdvanceTo(32.0);
    } catch (const Failure& error) {
        refused = error.reason() == Failure::Reason::kNonFiniteState;
    }
    Expect(refused, "non-finite coordinate candidate has the dedicated public numerical reason");
    Unchanged(fixture, *accepted, 0.0, initial, "non-finite candidates do not leak into accepted state");
    Expect(advancer->integration_statistics().successful_internal_step_count == 0,
           "overflowing trial cannot count as a successful internal step");
}

template <class Fixture>
Eigen::VectorXd Integrate(Recipe recipe, Fixture& fixture, double end, int steps) {
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    fixture.system->SetContinuousState(*accepted, fixture.ExactPhysicalState(0.0));
    auto advancer = Make(fixture, *accepted, Settings(recipe, fixture, end / steps));
    advancer->AdvanceTo(end);
    Expect(accepted->time_seconds() == end &&
               advancer->integration_statistics().successful_internal_step_count ==
                   static_cast<std::uint64_t>(steps),
           "factory stop schedule must retain the requested fixed refinement grid");
    Expect(Diagnostics(*advancer).startup_step_count == (recipe == Recipe::kZhai ? 1 : 0),
           "smooth fixed-grid factory runs have only their original startup");
    return Physical(fixture, *accepted);
}

void CheckOrders(const std::vector<double>& errors, const std::string& label) {
    Expect(errors.size() == 4, "four fixed refinement grids are required");
    for (std::size_t level = 2; level < errors.size(); ++level) {
        const double order = std::log2(errors[level - 1] / errors[level]);
        Expect(std::isfinite(order) && order >= 1.8 && order <= 2.2,
               label + ": observed order=" + std::to_string(order));
    }
}

void CheckRealFactoryConvergence(Recipe recipe, const std::string& name) {
    constexpr double end = 1.0;
    FreeTopFixture top;
    BallRpyFixture ball;
    SliderMaxwellFixture slider;
    std::vector<double> top_angles, top_velocities, ball_angles, ball_velocities;
    std::array<std::vector<double>, 3> maxwell_errors;
    for (const int steps : {32, 64, 128, 256}) {
        const auto top_result = Integrate(recipe, top, end, steps);
        const auto top_exact = top.ExactPhysicalState(end);
        const auto qr = top.model.GetFreeBodyPositionRange(top.body);
        const auto vr = top.system->generalized_velocities_state_range();
        top_angles.push_back(AngleError(QuaternionRotation(top_result.segment<4>(qr.start())),
                                       top.ExactRotation(end)));
        top_velocities.push_back((top_result.segment(vr.start(), vr.size()) -
                                  top_exact.segment(vr.start(), vr.size())).norm());
        Near(top_result.segment<4>(qr.start()).norm(), 1.7, 3e-14,
             name + " free rigid body retains non-unit initial quaternion norm");
        Near((top_result.segment<3>(qr.start() + 4) - top_exact.segment<3>(qr.start() + 4)).norm(),
             0.0, 2e-12, name + " exact free-body translation");

        const auto ball_result = Integrate(recipe, ball, end, steps);
        ball_angles.push_back(AngleError(RpyRotation(ball_result.head<3>()), ball.ExactRotation(end)));
        ball_velocities.push_back((ball_result.tail<3>() - ball.world_omega).norm());

        const auto maxwell_result = Integrate(recipe, slider, end, steps);
        const auto maxwell_exact = slider.ExactPhysicalState(end);
        for (int component = 0; component < 3; ++component) {
            maxwell_errors[component].push_back(std::abs(maxwell_result[component] -
                                                       maxwell_exact[component]));
        }
        Expect(std::abs(maxwell_result[0] - 0.2) > 0.1,
               "Maxwell factory qualification includes real mechanical displacement");
    }
    CheckOrders(top_angles, name + " factory free-top attitude");
    CheckOrders(top_velocities, name + " factory free-top physical velocity");
    CheckOrders(ball_angles, name + " factory Ball-RPY attitude");
    CheckOrders(ball_velocities, name + " factory Ball-RPY physical angular velocity");
    CheckOrders(maxwell_errors[0], name + " factory moving Maxwell position");
    CheckOrders(maxwell_errors[1], name + " factory moving Maxwell velocity");
    CheckOrders(maxwell_errors[2], name + " factory moving Maxwell force");
}

void CheckReplacedQuaternionReference(Recipe recipe) {
    FreeTopFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    fixture.system->SetContinuousState(*accepted, fixture.ExactPhysicalState(0.0));
    auto advancer = Make(fixture, *accepted, Settings(recipe, fixture, 0.125));
    advancer->AdvanceTo(0.125);
    auto changed = fixture.ExactPhysicalState(0.5);
    const auto qr = fixture.model.GetFreeBodyPositionRange(fixture.body);
    changed.segment<4>(qr.start()) *= -2.3 / 1.7;
    fixture.system->SetTimeAndContinuousState(*accepted, 0.5, changed);
    advancer->SynchronizeAfterAcceptedContextChange();
    Unchanged(fixture, *accepted, 0.5, changed,
              "synchronization imports a different quaternion norm and sign without rewriting accepted data");
    CheckResetStatistics(*advancer);
    advancer->AdvanceTo(0.625);
    Near(Physical(fixture, *accepted).segment<4>(qr.start()).norm(), 2.3, 3e-14,
         "new successful synchronization replaces the core quaternion projection reference");
}

}  // namespace

int main() {
    try {
        for (const auto recipe : {Recipe::kNewmark, Recipe::kZhai}) {
            const std::string name = recipe == Recipe::kNewmark ? "Newmark" : "Zhai";
            CheckIdentityAndStepHistory(recipe);
            CheckDenseFactoryTransaction(recipe);
            CheckInvalidCalls(recipe);
            CheckBudgetFailure(recipe, false);
            CheckBudgetFailure(recipe, true);
            CheckContextLocalSynchronization(recipe);
            CheckRpyFailureTransaction(recipe);
            CheckNonFiniteStateClassification(recipe);
            CheckRealFactoryConvergence(recipe, name);
            CheckReplacedQuaternionReference(recipe);
        }
        CheckNewmarkFailureClassification();
        VerifyBasicSystemHeldTorqueSynchronization();
        std::cout << "basic system continuous-state advancer checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "basic system continuous-state advancer: " << error.what() << '\n';
        return 1;
    }
}
