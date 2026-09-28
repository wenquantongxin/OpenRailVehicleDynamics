#include "system_coordinate_problem.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "newmark_core.h"
#include "zhai_core.h"
#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/integrators/system_rhs_bridge.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"

void VerifySystemCoordinateProblemContactIsolation();

namespace {

using orvd::integrators::NoCallTimeAppliedForces;
using orvd::integrators::SystemRhsBridge;
using orvd::integrators::internal::CoordinateState;
using orvd::integrators::internal::NewmarkConfiguration;
using orvd::integrators::internal::NewmarkCore;
using orvd::integrators::internal::SystemCoordinateProblem;
using orvd::integrators::internal::ZhaiConfiguration;
using orvd::integrators::internal::ZhaiCore;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::CompiledSystemPlan;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;
using orvd::system_assembly::SystemRuntimeContext;

static_assert(!std::is_copy_constructible_v<SystemCoordinateProblem>);
static_assert(!std::is_move_constructible_v<SystemCoordinateProblem>);
static_assert(!std::is_constructible_v<SystemCoordinateProblem, SystemInstance&&,
              const CompiledSystemPlan&, SystemRuntimeContext&, NoCallTimeAppliedForces>);
static_assert(!std::is_constructible_v<SystemCoordinateProblem, const SystemInstance&,
              CompiledSystemPlan&&, SystemRuntimeContext&, NoCallTimeAppliedForces>);

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

struct MixedFixture final : SystemFixture {
    std::array<RigidBodyHandle, 2> free_bodies;
    JointHandle ball_joint;
    JointHandle revolute_joint;
    JointHandle slider_joint;

    MixedFixture() {
        free_bodies[0] = model.AddRigidBody("free_first", Inertia(2.0, {0.3, 0.4, 0.5}));
        free_bodies[1] = model.AddRigidBody("free_second", Inertia(1.5));
        for (const auto body : free_bodies) model.DeclareFreeBody(body);
        const auto anchor = model.AddRigidBody("anchor", Inertia());
        const auto slider = model.AddRigidBody("slider", Inertia());
        const auto rotor = model.AddRigidBody("rotor", Inertia());
        const auto ball = model.AddRigidBody("ball_body", Inertia());
        model.AddWeldJoint("fixed_anchor", model.world_frame(), model.body_frame(anchor));
        slider_joint = model.AddPrismaticJoint("slide", model.body_frame(anchor),
                                               model.body_frame(slider), Eigen::Vector3d::UnitX(), 0.0);
        revolute_joint = model.AddRevoluteJoint("revolve", model.world_frame(),
                                                model.body_frame(rotor), Eigen::Vector3d::UnitZ(), 0.0);
        ball_joint = model.AddBallRpyJoint("ball", model.world_frame(),
                                            model.body_frame(ball), {0.1, -0.2, 0.3});
        Finish(SliderElements(model, anchor, slider));
    }

    Eigen::VectorXd InitialPhysicalState() const {
        auto physical = DefaultPhysicalState();
        const auto vr = system->generalized_velocities_state_range();
        physical.segment(vr.start(), vr.size()) = Eigen::VectorXd::LinSpaced(vr.size(), -0.6, 0.9);
        for (int i = 0; i != 2; ++i) {
            const auto qr = model.GetFreeBodyPositionRange(free_bodies[i]);
            const double signed_norm = i == 0 ? 2.3 : -0.8;
            physical.segment<4>(qr.start()) = signed_norm *
                QuaternionEntries(RpyRotation({0.2 + 0.1 * i, -0.3, 0.15}));
            physical.segment<3>(qr.start() + 4) << 0.7 * i, -0.2, 0.5;
        }
        physical[model.GetJointPositionRange(slider_joint).start()] = 0.25;
        physical[model.GetJointPositionRange(revolute_joint).start()] = -0.35;
        physical[system->series_spring_damper_force_state_range().start()] = 0.4;
        return physical;
    }
};

NewmarkConfiguration NewmarkSettings(const SystemCoordinateProblem& problem, double h) {
    NewmarkConfiguration result;
    result.step_size_seconds = h;
    auto& solver = result.nonlinear_solver;
    solver.position_correction_scales = Eigen::VectorXd::Constant(problem.coordinate_size(), 1e-11);
    solver.velocity_correction_scales = solver.position_correction_scales;
    solver.acceleration_residual_scales = solver.position_correction_scales;
    solver.internal_state_correction_scales =
        Eigen::VectorXd::Constant(problem.internal_state_size(), 1e-11);
    solver.internal_state_residual_scales = solver.internal_state_correction_scales;
    solver.unknown_reference_scales =
        Eigen::VectorXd::Ones(problem.coordinate_size() + problem.internal_state_size());
    return result;
}

template <typename Core, typename Fixture>
Eigen::VectorXd Integrate(Fixture& fixture, double end_time, int steps) {
    auto trial = fixture.system->CreateDefaultRuntimeContext(0.0);
    SystemCoordinateProblem problem(*fixture.system, *fixture.plan, *trial,
                                    NoCallTimeAppliedForces{});
    const auto initial = problem.MakeCoordinateState(0.0, fixture.ExactPhysicalState(0.0));
    const double h = end_time / steps;
    const auto settings = [&] {
        if constexpr (std::is_same_v<Core, NewmarkCore>) return NewmarkSettings(problem, h);
        else return ZhaiConfiguration{h};
    }();
    Core core(problem, settings, initial);
    for (int step = 0; step < steps; ++step) core.AdvanceOneStep();
    CoordinateState final;
    final.time_seconds = core.current_time_seconds();
    final.q.resize(problem.coordinate_size());
    final.s.resize(problem.coordinate_size());
    final.z.resize(problem.internal_state_size());
    core.CopyCurrentState(final.q, final.s, final.z);
    Eigen::VectorXd result(fixture.system->continuous_state_size());
    problem.CopyPhysicalState(final, result);
    Near(final.time_seconds, end_time, 2e-14, "core endpoint time");
    Expect(core.integration_statistics().successful_internal_step_count ==
               static_cast<std::uint64_t>(steps), "each real bridge step must be counted");
    return result;
}

void CheckOrders(const std::vector<double>& errors, const std::string& label) {
    Expect(errors.size() == 4, "four fixed grids are required");
    for (std::size_t level = 2; level < errors.size(); ++level) {
        const double order = std::log2(errors[level - 1] / errors[level]);
        Expect(std::isfinite(order) && order >= 1.8 && order <= 2.2,
               label + ": final observed order=" + std::to_string(order));
    }
}

template <typename Core>
void CheckRealSystemConvergence(const std::string& method) {
    constexpr double end = 0.8;
    FreeTopFixture top;
    BallRpyFixture ball;
    SliderMaxwellFixture slider;
    std::vector<double> top_angles, top_velocities, ball_angles, ball_velocities;
    std::array<std::vector<double>, 3> maxwell_errors;
    for (const int steps : {20, 40, 80, 160}) {
        const Eigen::VectorXd top_result = Integrate<Core>(top, end, steps);
        const auto top_exact = top.ExactPhysicalState(end);
        const auto tqr = top.model.GetFreeBodyPositionRange(top.body);
        const auto tvr = top.system->generalized_velocities_state_range();
        top_angles.push_back(AngleError(QuaternionRotation(top_result.segment<4>(tqr.start())),
                                       top.ExactRotation(end)));
        top_velocities.push_back((top_result.segment(tvr.start(), tvr.size()) -
                                  top_exact.segment(tvr.start(), tvr.size())).norm());
        Near(top_result.segment<4>(tqr.start()).norm(), 1.7, 3e-14,
             method + " free-top quaternion norm");
        Near((top_result.segment<3>(tqr.start() + 4) -
              top_exact.segment<3>(tqr.start() + 4)).norm(), 0.0, 2e-12,
             method + " free-top translational motion");

        const Eigen::VectorXd ball_result = Integrate<Core>(ball, end, steps);
        ball_angles.push_back(AngleError(RpyRotation(ball_result.head<3>()), ball.ExactRotation(end)));
        ball_velocities.push_back((ball_result.tail<3>() - ball.world_omega).norm());

        const Eigen::VectorXd maxwell_result = Integrate<Core>(slider, end, steps);
        const auto maxwell_exact = slider.ExactPhysicalState(end);
        for (int component = 0; component != 3; ++component) {
            maxwell_errors[component].push_back(std::abs(maxwell_result[component] -
                                                       maxwell_exact[component]));
        }
        Expect(std::abs(maxwell_result[0] - 0.2) > 0.1,
               "Maxwell qualification must include actual slider displacement");
    }
    CheckOrders(top_angles, method + " free-top attitude");
    CheckOrders(top_velocities, method + " free-top physical velocity");
    CheckOrders(ball_angles, method + " Ball-RPY attitude");
    CheckOrders(ball_velocities, method + " Ball-RPY physical angular velocity");
    CheckOrders(maxwell_errors[0], method + " moving Maxwell position");
    CheckOrders(maxwell_errors[1], method + " moving Maxwell velocity");
    CheckOrders(maxwell_errors[2], method + " moving Maxwell force");
}

void VerifyRhsAndRoundTrip() {
    SliderMaxwellFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    auto trial = fixture.system->CreateDefaultRuntimeContext(0.0);
    auto direct = fixture.system->CreateDefaultRuntimeContext(0.0);
    const auto initial = fixture.ExactPhysicalState(0.0);
    fixture.system->SetTimeAndContinuousState(*accepted, 0.0, initial);
    SystemCoordinateProblem bridge(*fixture.system, *fixture.plan, *trial, NoCallTimeAppliedForces{});
    SystemRhsBridge rhs(*fixture.system, *fixture.plan, *direct, NoCallTimeAppliedForces{});
    const auto physical = fixture.ExactPhysicalState(0.31);
    const auto coordinates = bridge.MakeCoordinateState(0.31, physical);
    Eigen::VectorXd round_trip(physical.size());
    bridge.CopyPhysicalState(coordinates, round_trip);
    Expect((round_trip.array() == physical.array()).all(), "Euclidean physical round trip");
    Eigen::VectorXd full_rhs(physical.size()), b(1), g(1);
    rhs.CalcTimeDerivatives(0.31, physical, full_rhs);
    bridge.Evaluate(0.31, coordinates.q, coordinates.s, coordinates.z, b, g);
    Near(b[0], full_rhs[1], 0.0, "real full RHS acceleration equivalence");
    Near(g[0], full_rhs[2], 0.0, "real full RHS Maxwell derivative equivalence");
    Near(b[0], -physical[2], 2e-14, "moving Maxwell force accelerates the slider");
    Near(g[0], 4.0 * physical[1] - 2.0 * physical[2], 2e-14,
         "Maxwell drive uses the real moving endpoint velocity");
    Eigen::VectorXd accepted_after(initial.size());
    fixture.system->CopyContinuousState(*accepted, accepted_after);
    Expect((accepted_after.array() == initial.array()).all() && accepted->time_seconds() == 0.0,
           "coordinate trials cannot modify accepted physical state");
}

void VerifyMixedGeometryAndRawTrials() {
    MixedFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(-0.2);
    auto trial = fixture.system->CreateDefaultRuntimeContext(0.0);
    auto direct = fixture.system->CreateDefaultRuntimeContext(0.0);
    const auto initial = fixture.InitialPhysicalState();
    fixture.system->SetTimeAndContinuousState(*accepted, -0.2, initial);
    SystemCoordinateProblem bridge(*fixture.system, *fixture.plan, *trial, NoCallTimeAppliedForces{});
    SystemRhsBridge rhs(*fixture.system, *fixture.plan, *direct, NoCallTimeAppliedForces{});
    const int nq = bridge.coordinate_size();
    const int nz = bridge.internal_state_size();
    const auto vr = fixture.system->generalized_velocities_state_range();
    const auto zr = fixture.system->series_spring_damper_force_state_range();
    Eigen::VectorXd trial_before(initial.size());
    fixture.system->CopyContinuousState(*trial, trial_before);
    auto coordinates = bridge.MakeCoordinateState(0.31, initial);
    bridge.ValidateInitialState(0.31, coordinates.q, coordinates.s, coordinates.z);
    Eigen::VectorXd physical(initial.size());
    bridge.CopyPhysicalState(coordinates, physical);
    Near((physical - initial).norm(), 0.0, 3e-15,
         "mixed nonunit-quaternion, RPY and scalar physical round trip");
    Expect((physical.head(nq).array() == initial.head(nq).array()).all() &&
               physical[zr.start()] == initial[zr.start()],
           "physical import/export preserves q and z exactly");
    Eigen::VectorXd trial_after(initial.size());
    fixture.system->CopyContinuousState(*trial, trial_after);
    Expect((trial_after.array() == trial_before.array()).all() && trial->time_seconds() == 0.0,
           "geometry-only validation and conversion cannot mutate physics trial state");

    const auto tangent = coordinates;
    for (int i = 0; i != 2; ++i) {
        const auto qr = fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[i]);
        coordinates.s.segment<4>(qr.start()) += (0.2 + 0.1 * i) * coordinates.q.segment<4>(qr.start());
    }
    Throws([&] { bridge.ValidateInitialState(0.31, coordinates.q, coordinates.s, coordinates.z); },
           "initial radial rates are rejected rather than repaired");
    bridge.CopyPhysicalState(coordinates, physical);
    Eigen::VectorXd derivatives(initial.size()), expected_b(nq), actual_b(nq), actual_g(nz);
    rhs.CalcTimeDerivatives(0.31, physical, derivatives);
    const Eigen::VectorXd acceleration = derivatives.segment(vr.start(), vr.size());
    const auto direct_component = fixture.system->GetMultibodyComponentView(
        *direct, fixture.plan->derivative_component());
    fixture.model.MapGeneralizedVelocityDerivativesToPositionSecondDerivatives(
        direct_component.context(), acceleration, &expected_b);
    bridge.Evaluate(0.31, coordinates.q, coordinates.s, coordinates.z, actual_b, actual_g);
    Near((actual_b - expected_b).norm(), 0.0, 2e-13,
         "raw radial-rate trial uses the complete physical RHS and nq acceleration map");
    Near((actual_g - derivatives.segment(zr.start(), zr.size())).norm(), 0.0, 1e-14,
         "raw trial retains the same full-RHS internal derivative");
    for (const auto body : fixture.free_bodies) {
        const auto qr = fixture.model.GetFreeBodyPositionRange(body);
        Expect(coordinates.q.segment<4>(qr.start()).dot(actual_b.segment<4>(qr.start())) < -1e-3,
               "full nq bridge result must include nonzero radial acceleration");
    }

    // Change both raw quaternion scales and add radial rates. Neither the
    // floating translations nor the RPY/revolute/prismatic blocks may change.
    auto candidate = tangent;
    std::vector<bool> quaternion_entry(static_cast<std::size_t>(nq), false);
    for (int i = 0; i != 2; ++i) {
        const auto qr = fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[i]);
        candidate.q.segment<4>(qr.start()) *= i == 0 ? 1.2 : 0.7;
        candidate.s.segment<4>(qr.start()) += 0.3 * candidate.q.segment<4>(qr.start());
        for (int j = 0; j != 4; ++j) quaternion_entry[static_cast<std::size_t>(qr.start() + j)] = true;
    }
    const auto raw_candidate = candidate;
    Eigen::VectorXd physical_before(initial.size()), physical_after(initial.size());
    bridge.CopyPhysicalState(candidate, physical_before);
    fixture.system->CopyContinuousState(*trial, trial_before);
    const double trial_time = trial->time_seconds();
    Expect(bridge.ProjectEndpoint(tangent.q, candidate.q, candidate.s),
           "two changed quaternion blocks report projection work");
    bridge.CopyPhysicalState(candidate, physical_after);
    Near((physical_after.segment(vr.start(), vr.size()) -
          physical_before.segment(vr.start(), vr.size())).norm(), 0.0, 4e-15,
         "paired two-body projection preserves every physical velocity");
    for (const auto body : fixture.free_bodies) {
        const auto qr = fixture.model.GetFreeBodyPositionRange(body);
        Near(candidate.q.segment<4>(qr.start()).norm(), tangent.q.segment<4>(qr.start()).norm(),
             2e-15, "each quaternion recovers its own recorded norm");
        Near(candidate.q.segment<4>(qr.start()).dot(candidate.s.segment<4>(qr.start())),
             0.0, 2e-15, "each projected rate is tangent");
        Expect(candidate.q.segment<4>(qr.start()).dot(raw_candidate.q.segment<4>(qr.start())) > 0.0,
               "quaternion projection preserves sign");
    }
    for (int i = 0; i < nq; ++i) {
        if (!quaternion_entry[static_cast<std::size_t>(i)]) {
            Expect(candidate.q[i] == raw_candidate.q[i] && candidate.s[i] == raw_candidate.s[i],
                   "projection leaves every non-quaternion q/s entry bitwise unchanged");
        }
    }
    Expect((candidate.z.array() == raw_candidate.z.array()).all(),
           "coordinate projection cannot change force states");
    fixture.system->CopyContinuousState(*trial, trial_after);
    Expect((trial_after.array() == trial_before.array()).all() && trial->time_seconds() == trial_time,
           "projection and physical export cannot alter physics trial state");
    Eigen::VectorXd accepted_after(initial.size());
    fixture.system->CopyContinuousState(*accepted, accepted_after);
    Expect((accepted_after.array() == initial.array()).all() && accepted->time_seconds() == -0.2,
           "mixed raw trials and projection preserve the accepted context");

    auto bad_reference = tangent.q;
    bad_reference.segment<4>(fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[1]).start()).setZero();
    const auto before_bad_projection = candidate;
    Throws([&] { bridge.ProjectEndpoint(bad_reference, candidate.q, candidate.s); },
           "one invalid reference quaternion rejects the complete paired projection");
    Expect((candidate.q.array() == before_bad_projection.q.array()).all() &&
               (candidate.s.array() == before_bad_projection.s.array()).all(),
           "a failed second quaternion block cannot partially project the first");
}

bool SameBits(const Eigen::VectorXd& first, const Eigen::VectorXd& second) {
    return first.size() == second.size() &&
           std::memcmp(first.data(), second.data(),
                       static_cast<std::size_t>(first.size()) * sizeof(double)) == 0;
}

void VerifyPhysicalInterpolation() {
    MixedFixture fixture;
    auto trial = fixture.system->CreateDefaultRuntimeContext(-0.4);
    SystemCoordinateProblem bridge(*fixture.system, *fixture.plan, *trial,
                                   NoCallTimeAppliedForces{});
    const int nq = bridge.coordinate_size();
    const int physical_size = bridge.physical_state_size();
    Expect(physical_size == fixture.system->continuous_state_size(),
           "physical observation size follows the frozen system layout");
    Eigen::VectorXd trial_before(physical_size);
    fixture.system->CopyContinuousState(*trial, trial_before);
    Eigen::VectorXd first = fixture.InitialPhysicalState();
    Eigen::VectorXd last = first.array() + 0.4;
    Eigen::VectorXd reference = first.head(nq);
    std::vector<bool> quaternion_entry(static_cast<std::size_t>(physical_size), false);
    for (int i = 0; i != 2; ++i) {
        const auto qr = fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[i]);
        // Different endpoint signs and norms must not weight the rotation path.
        last.segment<4>(qr.start()) = (i == 0 ? -0.3 : 4.7) *
            QuaternionEntries(RpyRotation({0.6, -0.1, 0.4 + 0.2 * i}));
        reference.segment<4>(qr.start()) *= i == 0 ? 1.2 : 0.5;
        for (int j = 0; j != 4; ++j) {
            quaternion_entry[static_cast<std::size_t>(qr.start() + j)] = true;
        }
    }
    const auto first_free = fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[0]);
    first[first_free.start() + 4] = -0.0;
    last[physical_size - 1] = -0.0;
    Eigen::VectorXd sample(physical_size);
    for (const double fraction : {0.0, 1.0}) {
        bridge.CopyLinearlyInterpolatedPhysicalState(reference, first, last, fraction, sample);
        Expect(SameBits(sample, fraction == 0.0 ? first : last),
               "exact dense endpoints preserve every stored bit and quaternion sign/norm");
    }
    constexpr double fraction = 0.37;
    bridge.CopyLinearlyInterpolatedPhysicalState(reference, first, last, fraction, sample);
    const Eigen::VectorXd expected = sample;
    for (int i = 0; i != physical_size; ++i) {
        if (!quaternion_entry[static_cast<std::size_t>(i)]) {
            Expect(sample[i] == std::lerp(first[i], last[i], fraction),
                   "ordinary q, physical v and internal z use the same linear interpolation");
        }
    }
    for (const auto body : fixture.free_bodies) {
        const auto qr = fixture.model.GetFreeBodyPositionRange(body);
        const Eigen::Vector4d a = first.segment<4>(qr.start()).normalized();
        Eigen::Vector4d b = last.segment<4>(qr.start()).normalized();
        if (a.dot(b) < 0.0) b = -b;
        const Eigen::Vector4d direction = ((1.0 - fraction) * a + fraction * b).normalized();
        Near((sample.segment<4>(qr.start()) -
              direction * reference.segment<4>(qr.start()).norm()).norm(),
             0.0, 8e-16, "shortest-sign nlerp is independent of endpoint quaternion scale");
        Near(sample.segment<4>(qr.start()).norm(), reference.segment<4>(qr.start()).norm(),
             8e-16, "each observed quaternion keeps its own initialization norm");
    }

    // Exact antipodes represent the same attitude and may not cancel to zero.
    Eigen::VectorXd antipodal = first;
    for (const auto body : fixture.free_bodies) {
        const auto qr = fixture.model.GetFreeBodyPositionRange(body);
        antipodal.segment<4>(qr.start()) *= -3.0;
    }
    bridge.CopyLinearlyInterpolatedPhysicalState(reference, first, antipodal, 0.5, sample);
    for (const auto body : fixture.free_bodies) {
        const auto qr = fixture.model.GetFreeBodyPositionRange(body);
        Near(AngleError(QuaternionRotation(sample.segment<4>(qr.start())),
                        QuaternionRotation(first.segment<4>(qr.start()))),
             0.0, 1e-15, "antipodal quaternion storage leaves physical attitude unchanged");
    }

    Eigen::VectorXd aliased = first;
    bridge.CopyLinearlyInterpolatedPhysicalState(reference, aliased, last, fraction, aliased);
    Expect(SameBits(aliased, expected), "an interpolation input may also be its output");
    Eigen::VectorXd overlap(physical_size + 1), overlap_expected(physical_size);
    overlap.head(physical_size) = first;
    bridge.CopyLinearlyInterpolatedPhysicalState(first.head(nq), first, last, fraction,
                                                overlap_expected);
    bridge.CopyLinearlyInterpolatedPhysicalState(overlap.head(nq), overlap.head(physical_size),
                                                last, fraction, overlap.tail(physical_size));
    Expect(SameBits(overlap.tail(physical_size), overlap_expected),
           "partially overlapping reference, input and output are staged atomically");

    // A direct difference last-first would overflow despite a finite result.
    Eigen::VectorXd extreme_first = first, extreme_last = last;
    const int velocity_start = fixture.system->generalized_velocities_state_range().start();
    for (const int entry : {first_free.start() + 4, velocity_start, physical_size - 1}) {
        extreme_first[entry] = std::numeric_limits<double>::max();
        extreme_last[entry] = -std::numeric_limits<double>::max();
    }
    bridge.CopyLinearlyInterpolatedPhysicalState(reference, extreme_first, extreme_last,
                                                0.5, sample);
    Expect(sample.allFinite() && sample[first_free.start() + 4] == 0.0 &&
               sample[velocity_start] == 0.0 && sample[physical_size - 1] == 0.0,
           "physical linear interpolation remains finite for opposite finite extremes");

    const Eigen::VectorXd sentinel = Eigen::VectorXd::Constant(physical_size, 91.0);
    const auto expect_refusal = [&](const Eigen::VectorXd& ref,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& end, double at) {
        sample = sentinel;
        Throws([&] { bridge.CopyLinearlyInterpolatedPhysicalState(ref, start, end, at, sample); },
               "invalid dense interpolation input is refused");
        Expect(SameBits(sample, sentinel), "failed dense interpolation cannot partially write output");
    };
    for (const double bad_fraction : {-0.1, 1.1, std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::quiet_NaN()}) {
        expect_refusal(reference, first, last, bad_fraction);
    }
    expect_refusal(reference.head(nq - 1), first, last, 0.5);
    expect_refusal(reference, first.head(physical_size - 1), last, 0.5);
    expect_refusal(reference, first, last.head(physical_size - 1), 0.5);
    Eigen::VectorXd wrong_output = Eigen::VectorXd::Constant(physical_size - 1, 92.0);
    Throws([&] { bridge.CopyLinearlyInterpolatedPhysicalState(reference, first, last,
                                                             0.5, wrong_output); },
           "dense output size is checked before writing");
    Expect((wrong_output.array() == 92.0).all(), "wrong-size dense output remains untouched");
    const auto second_free = fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[1]);
    for (const double invalid_scale : {0.0, 1e-200, 1e200}) {
        Eigen::VectorXd invalid_reference = reference;
        invalid_reference.segment<4>(second_free.start()) *= invalid_scale;
        expect_refusal(invalid_reference, first, last, 0.5);
        Eigen::VectorXd invalid_end = last;
        invalid_end.segment<4>(second_free.start()) *= invalid_scale;
        expect_refusal(reference, first, invalid_end, 0.5);
    }
    Eigen::VectorXd nonfinite = last;
    nonfinite[physical_size - 1] = std::numeric_limits<double>::quiet_NaN();
    expect_refusal(reference, first, nonfinite, 0.5);

    // Sampling physical storage does not invoke the singular RPY rate map.
    const auto ball = fixture.model.GetJointPositionRange(fixture.ball_joint);
    Eigen::VectorXd rpy_start = first, rpy_end = last;
    const double half_pi = std::acos(-1.0) / 2.0;
    rpy_start[ball.start() + 1] = half_pi - 0.2;
    rpy_end[ball.start() + 1] = half_pi + 0.2;
    bridge.CopyLinearlyInterpolatedPhysicalState(reference, rpy_start, rpy_end, 0.5, sample);
    Expect(sample.allFinite() && sample[ball.start() + 1] == half_pi,
           "finite physical dense samples may cross an RPY rate-map singularity");
    Throws<std::runtime_error>([&] { (void)bridge.MakeCoordinateState(0.5, sample); },
                               "singular RPY sample really lies outside the coordinate import domain");

    Eigen::VectorXd trial_after(physical_size);
    fixture.system->CopyContinuousState(*trial, trial_after);
    Expect(SameBits(trial_after, trial_before) && trial->time_seconds() == -0.4,
           "all successful and rejected observation work leaves physics trial state/time unchanged");
}

void VerifyValidationAndForeignIdentity() {
    SliderMaxwellFixture fixture;
    const SystemAssemblyDescription other_description(fixture.model, *fixture.forces);
    const SystemInstance other_system(other_description);
    const CompiledSystemPlan other_plan(other_system);
    auto trial = fixture.system->CreateDefaultRuntimeContext(0.0);
    auto foreign = other_system.CreateDefaultRuntimeContext(0.0);
    Throws([&] { SystemCoordinateProblem bad(*fixture.system, other_plan, *trial,
                                              NoCallTimeAppliedForces{}); },
           "same-model foreign plan identity is rejected");
    Throws([&] { SystemCoordinateProblem bad(*fixture.system, *fixture.plan, *foreign,
                                              NoCallTimeAppliedForces{}); },
           "same-model foreign trial identity is rejected");
    SystemCoordinateProblem bridge(*fixture.system, *fixture.plan, *trial, NoCallTimeAppliedForces{});
    Throws([&] { bridge.SynchronizeContextLocalDataFrom(*foreign); },
           "foreign context-local synchronization is rejected");
    const auto initial = fixture.ExactPhysicalState(0.0);
    auto coordinate = bridge.MakeCoordinateState(0.0, initial);
    Eigen::VectorXd b = Eigen::VectorXd::Constant(1, 12.0);
    Eigen::VectorXd g = Eigen::VectorXd::Constant(1, 13.0);
    auto bad = coordinate;
    bad.z[0] = std::numeric_limits<double>::quiet_NaN();
    Throws([&] { bridge.Evaluate(0.0, bad.q, bad.s, bad.z, b, g); },
           "non-finite internal state is rejected");
    Expect(b[0] == 12.0 && g[0] == 13.0,
           "invalid coordinate input leaves both derivative outputs unchanged");
    Eigen::VectorXd wrong_b = Eigen::VectorXd::Constant(2, 14.0);
    Throws([&] { bridge.Evaluate(0.0, coordinate.q, coordinate.s, coordinate.z, wrong_b, g); },
           "wrong mechanical output size is rejected");
    Expect((wrong_b.array() == 14.0).all() && g[0] == 13.0,
           "all output dimensions are checked before any write");
    Eigen::VectorXd overlap = Eigen::VectorXd::Constant(1, 15.0);
    Throws([&] { bridge.Evaluate(0.0, coordinate.q, coordinate.s, coordinate.z, overlap, overlap); },
           "overlapping acceleration and internal derivative outputs are rejected");
    Expect(overlap[0] == 15.0, "overlapping outputs remain unchanged");
    Eigen::VectorXd physical = Eigen::VectorXd::Constant(3, 16.0);
    Throws([&] { bridge.CopyPhysicalState(bad, physical); }, "invalid physical export is rejected");
    Expect((physical.array() == 16.0).all(), "failed physical export is output atomic");
    Throws([&] { (void)bridge.MakeCoordinateState(std::numeric_limits<double>::infinity(), initial); },
           "non-finite physical import time is rejected");

    BallRpyFixture ball;
    auto ball_trial = ball.system->CreateDefaultRuntimeContext(0.0);
    SystemCoordinateProblem ball_bridge(*ball.system, *ball.plan, *ball_trial, NoCallTimeAppliedForces{});
    const auto regular = ball_bridge.MakeCoordinateState(0.0, ball.ExactPhysicalState(0.0));
    auto singular = regular;
    singular.q[1] = std::acos(-1.0) / 2.0;
    Throws<std::runtime_error>([&] {
        ball_bridge.ValidateInitialState(0.0, singular.q, singular.s, singular.z);
    }, "Ball-RPY initial rate-map singularity is rejected");
    Eigen::VectorXd ball_physical = ball.ExactPhysicalState(0.0);
    ball_physical[1] = singular.q[1];
    Throws<std::runtime_error>([&] { (void)ball_bridge.MakeCoordinateState(0.0, ball_physical); },
                               "physical initial import checks the Ball-RPY forward rate-map domain");
    Eigen::VectorXd ball_b = Eigen::VectorXd::Constant(3, 17.0), empty_g(0);
    Throws<std::runtime_error>([&] {
        ball_bridge.Evaluate(0.1, singular.q, singular.s, singular.z, ball_b, empty_g);
    }, "a singular RPY trial is rejected");
    Expect((ball_b.array() == 17.0).all(), "failed geometric RHS leaves full nq output unchanged");

    // No manufactured future layout is needed: an all-weld system really has
    // nq=0 and is outside the nonempty mechanical-core contract.
    SystemFixture welded;
    const auto body = welded.model.AddRigidBody("fixed", Inertia());
    welded.model.AddWeldJoint("weld", welded.model.world_frame(), welded.model.body_frame(body));
    welded.Finish();
    auto welded_trial = welded.system->CreateDefaultRuntimeContext(0.0);
    Throws([&] { SystemCoordinateProblem invalid(*welded.system, *welded.plan, *welded_trial,
                                                  NoCallTimeAppliedForces{}); },
           "zero-coordinate systems are refused at bridge construction");
}

void VerifyExplicitContextLocalSynchronization() {
    SliderMaxwellFixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(3.0);
    auto trial = fixture.system->CreateDefaultRuntimeContext(0.0);
    const auto state = fixture.ExactPhysicalState(0.0);
    fixture.system->SetTimeAndContinuousState(*accepted, 3.0, state);
    const auto component = fixture.system->GetMultibodyComponentView(
        *accepted, fixture.system->multibody_component());
    fixture.model.SetPrismaticJointDampingCoefficient(&component.context(), fixture.joint, 0.6);
    const auto nominal = fixture.system->GetTranslationalSpringDamperIndexByName("nominal_slot");
    fixture.system->SetNominalForce(*accepted, nominal, {1.2, 0.0, 0.0});
    SystemCoordinateProblem bridge(*fixture.system, *fixture.plan, *trial, NoCallTimeAppliedForces{});
    const auto coordinate = bridge.MakeCoordinateState(0.0, state);
    Eigen::VectorXd b(1), g(1);
    const auto evaluate = [&] {
        bridge.Evaluate(0.0, coordinate.q, coordinate.s, coordinate.z, b, g);
    };
    evaluate();
    Near(b[0], -state[2], 1e-14, "bridge does not silently import context-local data");
    Eigen::VectorXd before(state.size()), after(state.size());
    fixture.system->CopyContinuousState(*trial, before);
    bridge.SynchronizeContextLocalDataFrom(*accepted);
    fixture.system->CopyContinuousState(*trial, after);
    Expect((before.array() == after.array()).all() && trial->time_seconds() == 0.0,
           "input synchronization does not copy physical state or time");
    evaluate();
    Near(b[0], -state[2] - 0.6 * state[1] - 1.2, 2e-14,
         "joint damping and nominal force are both synchronized into real dynamics");
    Near(g[0], 4.0 * state[1] - 2.0 * state[2], 2e-14,
         "context-local input synchronization preserves the Maxwell law");
    fixture.model.SetPrismaticJointDampingCoefficient(&component.context(), fixture.joint, 1.4);
    fixture.system->SetNominalForce(*accepted, nominal, {-0.7, 0.0, 0.0});
    evaluate();
    Near(b[0], -state[2] - 0.6 * state[1] - 1.2, 2e-14,
         "accepted-context changes remain invisible until explicitly synchronized");
    bridge.SynchronizeContextLocalDataFrom(*accepted);
    evaluate();
    Near(b[0], -state[2] - 1.4 * state[1] + 0.7, 2e-14,
         "second synchronization replaces the complete admitted input values");
    fixture.system->CopyContinuousState(*accepted, after);
    Expect((after.array() == state.array()).all() && accepted->time_seconds() == 3.0,
           "holding-input synchronization and trials leave accepted continuous state intact");
}

void VerifyActualDynamicsFailureIsolation() {
    SystemFixture fixture;
    const auto anchor = fixture.model.AddRigidBody("anchor", Inertia());
    const auto slider = fixture.model.AddRigidBody("slider", Inertia());
    const auto massless = fixture.model.AddRigidBody("massless", Inertia(0.0, Eigen::Vector3d::Zero()));
    fixture.model.AddWeldJoint("anchor_weld", fixture.model.world_frame(),
                               fixture.model.body_frame(anchor));
    const auto sliding_joint = fixture.model.AddPrismaticJoint(
        "slide", fixture.model.body_frame(anchor), fixture.model.body_frame(slider),
        Eigen::Vector3d::UnitX(), 0.0);
    fixture.model.AddRevoluteJoint("singular", fixture.model.world_frame(),
                                    fixture.model.body_frame(massless), Eigen::Vector3d::UnitZ(), 0.0);
    fixture.Finish(SliderElements(fixture.model, anchor, slider));
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    auto trial = fixture.system->CreateDefaultRuntimeContext(0.0);
    auto physical = fixture.DefaultPhysicalState();
    physical[fixture.model.GetJointPositionRange(sliding_joint).start()] = 0.2;
    physical[fixture.system->generalized_velocities_state_range().start() +
             fixture.model.GetJointVelocityRange(sliding_joint).start()] = 0.7;
    physical[fixture.system->series_spring_damper_force_state_range().start()] = 0.4;
    fixture.system->SetTimeAndContinuousState(*accepted, 0.0, physical);
    SystemCoordinateProblem bridge(*fixture.system, *fixture.plan, *trial, NoCallTimeAppliedForces{});
    auto candidate = bridge.MakeCoordinateState(0.4, physical);
    candidate.q.array() += 0.1;
    candidate.s.array() += 0.05;
    candidate.z.array() += 0.2;
    Eigen::VectorXd b = Eigen::VectorXd::Constant(bridge.coordinate_size(), 21.0);
    Eigen::VectorXd g = Eigen::VectorXd::Constant(bridge.internal_state_size(), 22.0);
    bool failed_in_dynamics = false;
    try {
        bridge.Evaluate(candidate.time_seconds, candidate.q, candidate.s, candidate.z, b, g);
    } catch (const std::exception& error) {
        failed_in_dynamics = std::string(error.what()).find("positive-definite") != std::string::npos;
    }
    Expect(failed_in_dynamics, "singular articulated inertia must fail inside the real dynamics chain");
    Expect((b.array() == 21.0).all() && (g.array() == 22.0).all(),
           "dynamics failure cannot expose either mechanical or already-computed Maxwell derivatives");
    Eigen::VectorXd after(physical.size());
    fixture.system->CopyContinuousState(*accepted, after);
    Expect((after.array() == physical.array()).all() && accepted->time_seconds() == 0.0,
           "a real forward-dynamics failure cannot publish trial state or time");
}

}  // namespace

int main() {
    try {
        VerifyRhsAndRoundTrip();
        VerifyMixedGeometryAndRawTrials();
        VerifyPhysicalInterpolation();
        VerifyValidationAndForeignIdentity();
        VerifyExplicitContextLocalSynchronization();
        VerifyActualDynamicsFailureIsolation();
        VerifySystemCoordinateProblemContactIsolation();
        CheckRealSystemConvergence<NewmarkCore>("Newmark");
        CheckRealSystemConvergence<ZhaiCore>("Zhai");
        std::cout << "system coordinate problem verification passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "system coordinate problem verification failed: " << error.what() << '\n';
        return 1;
    }
}
