#include "newmark_coordinate_layout.h"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <functional>

#include "newmark_continuous_state_advancer.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"
#include "system_coordinate_problem.h"

namespace {
using namespace orvd::integrators;
using namespace orvd::integrators::internal;
using namespace orvd::multibody_model;

void Expect(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template <class Exception = std::invalid_argument, class Operation>
void Reject(Operation operation, const char* message) {
    try { operation(); } catch (const Exception&) { return; }
    throw std::runtime_error(message);
}

orvd::multibody_runtime::RigidBodyInertiaParameters Inertia() {
    orvd::multibody_runtime::RigidBodyInertiaParameters value;
    value.mass_kilograms = 1.0;
    value.center_of_mass_in_body_frame.setZero();
    value.unit_inertia_moments.setConstant(0.3);
    value.unit_inertia_products.setZero();
    return value;
}

struct Fixture {
    MultibodyModel model;
    std::array<RigidBodyHandle, 2> free_bodies;
    JointHandle slider, revolute, ball, weld;
    std::unique_ptr<orvd::forces::VehicleForcePlan> forces;
    std::unique_ptr<orvd::system_assembly::SystemInstance> system;
    std::unique_ptr<orvd::system_assembly::CompiledSystemPlan> plan;

    explicit Fixture(bool with_force = true, bool singular = false) {
        free_bodies[0] = model.AddRigidBody("first", Inertia());
        free_bodies[1] = model.AddRigidBody("second", Inertia());
        for (auto body : free_bodies) model.DeclareFreeBody(body);
        const auto anchor = model.AddRigidBody("anchor", Inertia());
        const auto moving = model.AddRigidBody("moving", Inertia());
        auto rotor_inertia = Inertia();
        if (singular) {
            rotor_inertia.mass_kilograms = 0.0;
            rotor_inertia.unit_inertia_moments.setZero();
        }
        const auto rotor = model.AddRigidBody("rotor", rotor_inertia);
        const auto spherical = model.AddRigidBody("spherical", Inertia());
        weld = model.AddWeldJoint("weld", model.world_frame(), model.body_frame(anchor));
        slider = model.AddPrismaticJoint("slider", model.body_frame(anchor), model.body_frame(moving),
                                         Eigen::Vector3d::UnitX(), 0.0);
        revolute = model.AddRevoluteJoint("rotor", model.world_frame(), model.body_frame(rotor),
                                          Eigen::Vector3d::UnitZ(), 0.0);
        ball = model.AddBallRpyJoint("ball", model.world_frame(), model.body_frame(spherical),
                                    {0.1, -0.2, 0.3});
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();
        orvd::forces::VehicleForceElementCollection elements;
        if (with_force) {
            elements.series_spring_viscous_dampers.push_back(orvd::forces::SeriesSpringViscousDamper{
                "maxwell", {model.body_frame(anchor)}, {model.body_frame(moving)},
                orvd::forces::ForceElementAxis::kLongitudinal, 4.0, 2.0});
            elements.translational_spring_dampers.push_back(orvd::forces::TranslationalSpringDamper{
                "slot", {model.body_frame(anchor)}, {model.body_frame(moving)},
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
        }
        forces = std::make_unique<orvd::forces::VehicleForcePlan>(model, std::move(elements));
        const orvd::system_assembly::SystemAssemblyDescription description(model, *forces);
        system = std::make_unique<orvd::system_assembly::SystemInstance>(description);
        plan = std::make_unique<orvd::system_assembly::CompiledSystemPlan>(*system);
    }

    Eigen::VectorXd State() const {
        const auto context = system->CreateDefaultRuntimeContext(0.0);
        Eigen::VectorXd state(system->continuous_state_size());
        system->CopyContinuousState(*context, state);
        for (int i = 0; i < 2; ++i) {
            const auto range = model.GetFreeBodyPositionRange(free_bodies[i]);
            state.segment<4>(range.start()).setZero();
            state[range.start()] = i == 0 ? 2.0 : -3.0;
        }
        return state;
    }
};

NewmarkConfiguration Configuration() {
    NewmarkConfiguration result;
    result.nominal_step_size_seconds = 0.001;
    result.nonlinear_solver.translation = {1e-8, 2e-8, 3e-8, 4.0};
    result.nonlinear_solver.angle = {5e-8, 6e-8, 7e-8, 8.0};
    result.nonlinear_solver.quaternion = {9e-8, 10e-8, 11e-8, 12.0};
    result.nonlinear_solver.force = {13e-8, 14e-8, 15.0};
    return result;
}

void CheckBlock(const NewmarkCoreConfiguration& expanded, int start, int size,
                const CoordinateNewtonScales& values, double factor) {
    const auto& solver = expanded.nonlinear_solver;
    for (int i = start; i < start + size; ++i) {
        Expect(solver.position_correction_scales[i] == factor * values.position_correction,
               "position scale family/reference mismatch");
        Expect(solver.velocity_correction_scales[i] == factor * values.velocity_correction,
               "velocity scale family/reference mismatch");
        Expect(solver.acceleration_residual_scales[i] == factor * values.acceleration_residual,
               "acceleration residual family/reference mismatch");
        Expect(solver.unknown_reference_scales[i] == factor * values.acceleration_reference,
               "acceleration difference reference family/reference mismatch");
    }
}

void VerifyLayoutAndTypes() {
    Fixture fixture;
    const auto input = Configuration();
    const int nq = fixture.model.num_generalized_positions();
    const auto physical = fixture.State();
    NewmarkCoordinateLayout layout(fixture.model, 1);
    const auto expanded = layout.Expand(input, physical.head(nq));
    Expect(expanded.nonlinear_solver.position_correction_scales.size() == nq,
           "all coordinate ranges covered");
    for (int i = 0; i < 2; ++i) {
        const auto range = fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[i]);
        CheckBlock(expanded, range.start(), 4, input.nonlinear_solver.quaternion, i == 0 ? 2.0 : 3.0);
        CheckBlock(expanded, range.start() + 4, 3, input.nonlinear_solver.translation, 1.0);
    }
    const auto check_joint = [&](JointHandle joint, JointType expected_type,
                                 const CoordinateNewtonScales& scales) {
        Expect(fixture.model.GetJointType(joint) == expected_type, "joint metadata type mismatch");
        const auto range = fixture.model.GetJointPositionRange(joint);
        CheckBlock(expanded, range.start(), range.size(), scales, 1.0);
    };
    check_joint(fixture.slider, JointType::kPrismatic, input.nonlinear_solver.translation);
    check_joint(fixture.revolute, JointType::kRevolute, input.nonlinear_solver.angle);
    check_joint(fixture.ball, JointType::kBallRpy, input.nonlinear_solver.angle);
    check_joint(fixture.weld, JointType::kWeld, input.nonlinear_solver.angle);
    Expect(expanded.nonlinear_solver.internal_state_correction_scales[0] == input.nonlinear_solver.force.correction &&
           expanded.nonlinear_solver.internal_state_residual_scales[0] == input.nonlinear_solver.force.residual &&
           expanded.nonlinear_solver.unknown_reference_scales[nq] == input.nonlinear_solver.force.reference,
           "force scales do not fill all three internal-state blocks");

    Fixture foreign;
    Reject([&] { (void)fixture.model.GetJointType(foreign.slider); }, "foreign joint handle accepted");
    Reject([&] { (void)fixture.model.GetJointType(JointHandle{}); }, "invalid joint handle accepted");
    MultibodyModel unfinished;
    Reject<std::logic_error>([&] { (void)unfinished.GetJointType(JointHandle{}); }, "unfinalized type query accepted");
    Reject([&] { (void)layout.Expand(input, Eigen::VectorXd::Zero(nq - 1)); }, "invalid reference size accepted");
    auto invalid_reference = physical.head(nq).eval();
    invalid_reference.segment<4>(fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[0]).start()).setZero();
    Reject([&] { (void)layout.Expand(input, invalid_reference); }, "zero reference quaternion accepted");
    auto overflow = input;
    overflow.nonlinear_solver.quaternion.acceleration_reference = std::numeric_limits<double>::max();
    Reject([&] { (void)layout.Expand(overflow, physical.head(nq)); }, "expanded reference overflow accepted");
    auto small_reference = physical.head(nq).eval();
    small_reference[fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[0]).start()] = 0.25;
    auto underflow = input;
    underflow.nonlinear_solver.quaternion.velocity_correction = std::numeric_limits<double>::denorm_min();
    Reject([&] { (void)layout.Expand(underflow, small_reference); }, "expanded scale underflow accepted");

    Fixture empty(false);
    const auto empty_state = empty.State();
    const auto empty_expanded = NewmarkCoordinateLayout(empty.model, 0).Expand(input, empty_state.head(nq));
    Expect(empty_expanded.nonlinear_solver.internal_state_residual_scales.size() == 0 &&
           empty_expanded.nonlinear_solver.unknown_reference_scales.size() == nq,
           "empty internal-state layout rejected");
    auto unused_invalid = input;
    unused_invalid.nonlinear_solver.force.correction = 0.0;
    Reject([&] { (void)NewmarkCoordinateLayout(empty.model, 0).Expand(unused_invalid, empty_state.head(nq)); },
           "unused force family allowed zero");
    unused_invalid = input;
    unused_invalid.nonlinear_solver.force.reference = std::numeric_limits<double>::infinity();
    Reject([&] { (void)NewmarkCoordinateLayout(empty.model, 0).Expand(unused_invalid, empty_state.head(nq)); },
           "unused force family allowed non-finite scale");

    // All geometry families are validated even when a scalar oscillator uses
    // only a revolute coordinate.
    MultibodyModel scalar;
    const auto rotor = scalar.AddRigidBody("rotor", Inertia());
    scalar.AddRevoluteJoint("joint", scalar.world_frame(), scalar.body_frame(rotor), Eigen::Vector3d::UnitZ(), 0.0);
    scalar.Finalize();
    for (auto family : {&NewmarkNewtonConfiguration::translation, &NewmarkNewtonConfiguration::quaternion}) {
        auto invalid = input;
        (invalid.nonlinear_solver.*family).position_correction = 0.0;
        Reject([&] { (void)NewmarkCoordinateLayout(scalar, 0).Expand(invalid, Eigen::VectorXd::Zero(1)); },
               "unused coordinate family allowed zero");
    }
}

void VerifyReferenceScaleTransaction() {
    Fixture fixture;
    auto trial = fixture.system->CreateDefaultRuntimeContext(0.0);
    SystemCoordinateProblem problem(*fixture.system, *fixture.plan, *trial, NoCallTimeAppliedForces{});
    auto state = fixture.State();
    const auto config = Configuration();
    NewmarkContinuousStateAdvancer advancer(problem, 0.0, state, config);
    const auto first_range = fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[0]);
    const auto second_range = fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[1]);
    const int first = first_range.start();
    const int second = second_range.start();
    auto changed = state;
    changed[first] = -4.0;
    changed[second] = 5.0;
    advancer.ReinitializeAfterExternalChange(0.2, changed);
    CheckBlock(advancer.expanded_configuration(), first, 4, config.nonlinear_solver.quaternion, 4.0);
    CheckBlock(advancer.expanded_configuration(), second, 4, config.nonlinear_solver.quaternion, 5.0);
    Eigen::VectorXd endpoint(state.size());
    (void)advancer.AdvanceOneInternalStepToward(0.201, endpoint);
    Expect(endpoint.segment<4>(first).norm() == 4.0 && endpoint.segment<4>(second).norm() == 5.0,
           "successful synchronization failed to replace projection reference");
    const auto accepted = endpoint;
    const auto statistics = advancer.integration_statistics();
    auto failing = changed;
    failing[first] = 7.0;
    failing[fixture.system->series_spring_damper_force_state_range().start()] = 1e308;
    bool failed = false;
    try { advancer.ReinitializeAfterExternalChange(0.4, failing); }
    catch (const std::exception&) { failed = true; }
    Expect(failed, "overflowing physical initialization unexpectedly succeeded");
    advancer.CopyCurrentState(endpoint);
    Expect((endpoint.array() == accepted.array()).all() && advancer.current_time_seconds() == 0.201,
           "failed synchronization published a new endpoint");
    CheckBlock(advancer.expanded_configuration(), first, 4, config.nonlinear_solver.quaternion, 4.0);
    CheckBlock(advancer.expanded_configuration(), second, 4, config.nonlinear_solver.quaternion, 5.0);
    Expect(advancer.integration_statistics().right_hand_side_evaluation_count ==
               statistics.right_hand_side_evaluation_count + 1,
           "failed synchronization hid performed evaluation");
    Reject<std::logic_error>([&] { (void)advancer.AdvanceOneInternalStepToward(0.202, endpoint); },
                             "failed synchronization allowed further stepping");
    advancer.ReinitializeAfterExternalChange(0.2, changed);
    (void)advancer.AdvanceOneInternalStepToward(0.201, endpoint);
    Expect(endpoint.segment<4>(first).norm() == 4.0 && endpoint.segment<4>(second).norm() == 5.0,
           "reference did not recover with successful reinitialization");
}

void VerifyPublicBoundary() {
    Fixture fixture;
    auto accepted = fixture.system->CreateDefaultRuntimeContext(0.0);
    const auto state = fixture.State();
    auto overflowing = state;
    overflowing[fixture.system->series_spring_damper_force_state_range().start()] = 1e308;
    fixture.system->SetTimeAndContinuousState(*accepted, 0.0, overflowing);
    auto construct = [&](SystemIntegrationConfiguration configuration) {
        return std::make_unique<SystemContinuousStateAdvancer>(
            *fixture.system, *fixture.plan, *accepted, std::move(configuration),
            NoCallTimeAppliedForces{});
    };
    // This physical state fails on its first RHS. Configuration refusals below
    // must therefore occur before any attempted initialization evaluation.
    auto invalid = Configuration();
    invalid.nonlinear_solver.translation.position_correction = 0.0;
    Reject([&] { (void)construct(SystemIntegrationConfiguration(invalid)); },
           "public invalid scale was not rejected before RHS");
    invalid = Configuration();
    invalid.nominal_step_size_seconds = std::numeric_limits<double>::infinity();
    Reject([&] { (void)construct(SystemIntegrationConfiguration(invalid)); },
           "public invalid Newmark step was accepted");
    invalid = Configuration();
    invalid.nonlinear_solver.maximum_iterations = 0;
    Reject([&] { (void)construct(SystemIntegrationConfiguration(invalid)); },
           "public invalid iteration limit was accepted");
    invalid = Configuration();
    invalid.nonlinear_solver.quaternion.acceleration_reference = std::numeric_limits<double>::max();
    Reject([&] { (void)construct(SystemIntegrationConfiguration(invalid)); },
           "public expansion overflow was not rejected before RHS");
    Reject([&] { (void)construct(SystemIntegrationConfiguration(Configuration(), 0)); },
           "public zero budget was accepted");
    Reject([&] { (void)construct(SystemIntegrationConfiguration(ZhaiConfiguration{0.0})); },
           "public zero Zhai step was accepted");
    Reject([&] {
        (void)construct(SystemIntegrationConfiguration(CvodeBdf2Configuration{
            ContinuousStateErrorTolerances(1e-6, Eigen::VectorXd::Constant(state.size() - 1, 1e-8))}));
    }, "public ODE tolerance dimensions were accepted");
    bool numeric = false;
    try { (void)construct(SystemIntegrationConfiguration(Configuration())); }
    catch (const ContinuousStateNumericalFailure& failure) {
        numeric = failure.reason() == ContinuousStateNumericalFailure::Reason::kNonFiniteRightHandSide;
    }
    Expect(numeric, "initial coordinate numeric failure escaped the public classification");

    fixture.system->SetTimeAndContinuousState(*accepted, 0.0, state);
    auto advancer = construct(SystemIntegrationConfiguration(Configuration()));
    Expect(advancer->integration_statistics().right_hand_side_evaluation_count == 1,
           "public Newmark construction hid its initialization RHS");
    auto new_state = state;
    const int first = fixture.model.GetFreeBodyPositionRange(fixture.free_bodies[0]).start();
    new_state[first] = 4.0;
    auto failed_state = new_state;
    failed_state[fixture.system->series_spring_damper_force_state_range().start()] = 1e308;
    fixture.system->SetTimeAndContinuousState(*accepted, 0.2, failed_state);
    numeric = false;
    try { advancer->SynchronizeAfterAcceptedContextChange(); }
    catch (const ContinuousStateNumericalFailure& failure) {
        numeric = failure.reason() == ContinuousStateNumericalFailure::Reason::kNonFiniteRightHandSide;
    }
    Expect(numeric, "synchronization coordinate numeric failure escaped public classification");
    Expect(advancer->integration_statistics().right_hand_side_evaluation_count == 2,
           "failed public synchronization lost performed work");
    Eigen::VectorXd actual(state.size());
    fixture.system->CopyContinuousState(*accepted, actual);
    Expect((actual.array() == failed_state.array()).all() && accepted->time_seconds() == 0.2,
           "failed public synchronization changed the caller's accepted state");
    Reject<std::logic_error>([&] { advancer->AdvanceTo(0.201); },
                             "failed public synchronization allowed advance");
    fixture.system->SetTimeAndContinuousState(*accepted, 0.2, new_state);
    advancer->SynchronizeAfterAcceptedContextChange();
    advancer->AdvanceTo(0.201);
    fixture.system->CopyContinuousState(*accepted, actual);
    Expect(actual.segment<4>(first).norm() == 4.0,
           "public synchronization failed to install the new norm reference");

    Fixture empty(false);
    auto empty_context = empty.system->CreateDefaultRuntimeContext(0.0);
    invalid = Configuration();
    invalid.nonlinear_solver.force.reference = 0.0;
    Reject([&] {
        SystemContinuousStateAdvancer rejected(*empty.system, *empty.plan, *empty_context,
            SystemIntegrationConfiguration(invalid), NoCallTimeAppliedForces{});
    }, "public unused force family was not validated");

    // Compare a real ABA exception with the one crossing the system boundary:
    // neither its dynamic type nor original message may be recategorized.
    Fixture singular(false, true);
    auto singular_context = singular.system->CreateDefaultRuntimeContext(0.0);
    auto trial = singular.system->CreateDefaultRuntimeContext(0.0);
    SystemCoordinateProblem problem(*singular.system, *singular.plan, *trial, NoCallTimeAppliedForces{});
    const auto singular_state = singular.State();
    const auto coordinates = problem.MakeCoordinateState(0.0, singular_state);
    Eigen::VectorXd b(problem.coordinate_size()), g(problem.internal_state_size());
    std::type_index expected_type(typeid(void));
    std::string expected_message;
    try { problem.Evaluate(0.0, coordinates.q, coordinates.s, coordinates.z, b, g); }
    catch (const std::exception& error) {
        expected_type = typeid(error);
        expected_message = error.what();
    }
    Expect(expected_type != std::type_index(typeid(void)) &&
           expected_message.find("positive-definite") != std::string::npos,
           "singular fixture did not fail inside physical dynamics");
    bool unchanged = false;
    try {
        SystemContinuousStateAdvancer rejected(*singular.system, *singular.plan, *singular_context,
            SystemIntegrationConfiguration(Configuration()), NoCallTimeAppliedForces{});
    } catch (const std::exception& error) {
        unchanged = std::type_index(typeid(error)) == expected_type && error.what() == expected_message;
    }
    Expect(unchanged, "public construction changed the physical exception type or reason");
}
}  // namespace

int main() {
    try {
        VerifyLayoutAndTypes();
        VerifyReferenceScaleTransaction();
        VerifyPublicBoundary();
        std::cout << "Newmark coordinate layout verification passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Newmark coordinate layout verification failed: " << error.what() << '\n';
        return 1;
    }
}
