#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_model.h"
#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"
#include "orvd/system_assembly/system_instance.h"

namespace {

using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::MultibodyComponentView;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;
using orvd::system_assembly::SystemRuntimeContext;

static_assert(!std::is_copy_constructible_v<SystemInstance>);
static_assert(!std::is_move_constructible_v<SystemInstance>);
static_assert(!std::is_copy_constructible_v<SystemRuntimeContext>);
static_assert(!std::is_move_constructible_v<SystemRuntimeContext>);
static_assert(std::is_copy_constructible_v<MultibodyComponentView>);

int failure_count = 0;

void Expect(bool condition, const char* description) {
    if (!condition) {
        std::printf("FAIL %s\n", description);
        ++failure_count;
    }
}

void ExpectNear(double actual, double expected, const char* description) {
    const double tolerance = 128.0 * std::numeric_limits<double>::epsilon() *
                             std::max(1.0, std::abs(expected));
    Expect(std::abs(actual - expected) <= tolerance, description);
}

template <typename Function>
void ExpectInvalidArgument(Function&& function, const char* description) {
    bool refused = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused, description);
}

RigidBodyInertiaParameters MakeInertia(double mass) {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass;
    inertia.center_of_mass_in_body_frame = Eigen::Vector3d(0.1, 0.0, 0.0);
    inertia.unit_inertia_moments = Eigen::Vector3d(0.3, 0.4, 0.5);
    inertia.unit_inertia_products.setZero();
    return inertia;
}

struct Fixture {
    MultibodyModel model;
    orvd::multibody_model::JointHandle revolute;
    orvd::multibody_model::JointHandle prismatic;

    Fixture() {
        const auto rotor = model.AddRigidBody("rotor", MakeInertia(2.0));
        const auto slider = model.AddRigidBody("slider", MakeInertia(1.0));
        revolute = model.AddRevoluteJoint(
            "revolute", model.world_frame(), model.body_frame(rotor),
            Eigen::Vector3d::UnitZ(), 0.5);
        prismatic = model.AddPrismaticJoint(
            "prismatic", model.body_frame(rotor), model.body_frame(slider),
            Eigen::Vector3d::UnitX(), 0.25);
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();
    }
};

void CheckFrozenLayoutAndSingleOwnership() {
    Fixture fixture;
    const SystemAssemblyDescription description(fixture.model);
    const SystemInstance system(description);
    const auto q_range = system.generalized_positions_state_range();
    const auto v_range = system.generalized_velocities_state_range();
    Expect(q_range.start() == 0 && q_range.size() == 2,
           "q is the first frozen system state block");
    Expect(v_range.start() == 2 && v_range.size() == 2,
           "v follows q in the frozen system state layout");
    Expect(system.continuous_state_size() == 4,
           "the frozen continuous-state size covers q and v once");

    auto context = system.CreateDefaultRuntimeContext(0.0);
    const auto first = system.GetMultibodyComponentView(
        *context, system.multibody_component());
    const auto second = system.GetMultibodyComponentView(
        *context, system.multibody_component());
    Expect(context->generalized_positions().data() ==
               first.context().generalized_positions().data(),
           "the root and component view observe the same q storage");
    Expect(context->generalized_velocities().data() ==
               first.context().generalized_velocities().data(),
           "the root and component view observe the same v storage");
    Expect(&first.forward_dynamics_workspace() ==
               &second.forward_dynamics_workspace(),
           "the model-bound hot workspace is allocated once per context");

    const SystemAssemblyDescription other_description(fixture.model);
    const SystemInstance other_system(other_description);
    ExpectInvalidArgument(
        [&] {
            (void)other_system.GetMultibodyComponentView(
                *context, other_system.multibody_component());
        },
        "a runtime context cannot be rebound to another system");
    ExpectInvalidArgument(
        [&] {
            (void)system.GetMultibodyComponentView(
                *context, other_system.multibody_component());
        },
        "a component index cannot be used in another system");
}

void CheckContextLocalDampingAndIsolation() {
    Fixture fixture;
    const SystemAssemblyDescription description(fixture.model);
    const SystemInstance system(description);
    auto first_context = system.CreateDefaultRuntimeContext(0.0);
    auto second_context = system.CreateDefaultRuntimeContext(0.0);
    const auto first = system.GetMultibodyComponentView(
        *first_context, system.multibody_component());
    const auto second = system.GetMultibodyComponentView(
        *second_context, system.multibody_component());

    Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
    positions[fixture.model.GetJointPositionRange(fixture.revolute).start()] =
        0.4;
    fixture.model.SetGeneralizedPositions(&first.context(), positions);
    Eigen::VectorXd velocities = Eigen::VectorXd::Zero(2);
    velocities[fixture.model.GetJointVelocityRange(fixture.revolute).start()] =
        2.0;
    velocities[fixture.model.GetJointVelocityRange(fixture.prismatic).start()] =
        -3.0;
    fixture.model.SetGeneralizedVelocities(&first.context(), velocities);
    fixture.model.SetGeneralizedVelocities(&second.context(), velocities);

    Expect(second_context->generalized_positions().isZero(0.0),
           "writing one system context does not change another context's q");
    Eigen::VectorXd first_force(2);
    Eigen::VectorXd second_force(2);
    fixture.model.CalcJointDampingAppliedGeneralizedForces(first.context(),
                                                           first_force);
    fixture.model.CalcJointDampingAppliedGeneralizedForces(second.context(),
                                                           second_force);
    ExpectNear(first_force[fixture.model.GetJointVelocityRange(fixture.revolute)
                               .start()],
               -1.0, "the default revolute damping is read from the context");
    ExpectNear(first_force[fixture.model.GetJointVelocityRange(fixture.prismatic)
                               .start()],
               0.75, "the default prismatic damping is read from the context");

    fixture.model.SetRevoluteJointDampingCoefficient(
        &first.context(), fixture.revolute, 1.5);
    fixture.model.SetPrismaticJointDampingCoefficient(
        &first.context(), fixture.prismatic, 2.0);
    fixture.model.CalcJointDampingAppliedGeneralizedForces(first.context(),
                                                           first_force);
    fixture.model.CalcJointDampingAppliedGeneralizedForces(second.context(),
                                                           second_force);
    ExpectNear(first_force[fixture.model.GetJointVelocityRange(fixture.revolute)
                               .start()],
               -3.0, "a context-local revolute damping write is immediate");
    ExpectNear(first_force[fixture.model.GetJointVelocityRange(fixture.prismatic)
                               .start()],
               6.0, "a context-local prismatic damping write is immediate");
    ExpectNear(second_force[fixture.model.GetJointVelocityRange(fixture.revolute)
                                .start()],
               -1.0, "the other context keeps its revolute damping");
    ExpectNear(second_force[fixture.model.GetJointVelocityRange(fixture.prismatic)
                                .start()],
               0.75, "the other context keeps its prismatic damping");

    ExpectInvalidArgument(
        [&] {
            fixture.model.SetRevoluteJointDampingCoefficient(
                &first.context(), fixture.revolute, -1.0);
        },
        "negative context-local damping is refused");
    ExpectInvalidArgument(
        [&] {
            fixture.model.SetPrismaticJointDampingCoefficient(
                &first.context(), fixture.prismatic,
                std::numeric_limits<double>::infinity());
        },
        "non-finite context-local damping is refused");
    ExpectInvalidArgument(
        [&] {
            fixture.model.SetRevoluteJointDampingCoefficient(
                &first.context(), fixture.prismatic, 0.4);
        },
        "a prismatic joint cannot receive a revolute damping unit");
    Fixture foreign_fixture;
    auto foreign_context = foreign_fixture.model.CreateDefaultContext();
    ExpectInvalidArgument(
        [&] {
            fixture.model.SetRevoluteJointDampingCoefficient(
                foreign_context.get(), fixture.revolute, 0.4);
        },
        "a foreign context cannot receive a damping write");
    ExpectInvalidArgument(
        [&] {
            fixture.model.SetRevoluteJointDampingCoefficient(
                &first.context(), foreign_fixture.revolute, 0.4);
        },
        "a foreign joint cannot receive a damping write");
    ExpectInvalidArgument(
        [&] {
            fixture.model.SetRevoluteJointDampingCoefficient(
                nullptr, fixture.revolute, 0.4);
        },
        "a null context cannot receive a damping write");
    fixture.model.CalcJointDampingAppliedGeneralizedForces(first.context(),
                                                           first_force);
    ExpectNear(first_force[fixture.model.GetJointVelocityRange(fixture.revolute)
                               .start()],
               -3.0, "a refused write leaves revolute damping unchanged");
    ExpectNear(first_force[fixture.model.GetJointVelocityRange(fixture.prismatic)
                               .start()],
               6.0, "a refused write leaves prismatic damping unchanged");
}

// ---------------------------------------------------------------------------
// Gate 2 — one transaction over every state block
// ---------------------------------------------------------------------------

// A fixture whose system carries a force state block, so that the transaction
// has more than one block to be atomic over.
orvd::forces::VehicleForceElementCollection MakeSeriesFixtureForceElements(
    const Fixture& mechanism) {
    orvd::forces::VehicleForceElementCollection elements;
    elements.translational_spring_dampers = {
        orvd::forces::TranslationalSpringDamper{
            "test_translation",
            orvd::forces::ForceElementEnd{
                mechanism.model.GetFrameByName("rotor")},
            orvd::forces::ForceElementEnd{
                mechanism.model.GetFrameByName("slider")},
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    elements.series_spring_viscous_dampers = {
        orvd::forces::SeriesSpringViscousDamper{
            "test_series",
            orvd::forces::ForceElementEnd{
                mechanism.model.GetFrameByName("rotor")},
            orvd::forces::ForceElementEnd{
                mechanism.model.GetFrameByName("slider")},
            orvd::forces::ForceElementAxis::kLateral, 4.0e5, 9.0e3}};
    return elements;
}

struct SeriesFixture {
    Fixture mechanism;
    orvd::forces::VehicleForcePlan plan;

    SeriesFixture()
        : plan(mechanism.model,
               MakeSeriesFixtureForceElements(mechanism)) {}
};

void CheckOneTransactionOverEveryBlock() {
    const SeriesFixture fixture;
    const SystemAssemblyDescription description(fixture.mechanism.model,
                                                fixture.plan);
    const SystemInstance system(description);
    Expect(system.continuous_state_size() == 5,
           "the state is q, v and the one series force");
    Expect(system.series_spring_damper_force_state_range().start() == 4 &&
               system.series_spring_damper_force_state_range().size() == 1,
           "the force block sits after q and v");

    auto context = system.CreateDefaultRuntimeContext(3.5);
    Eigen::VectorXd accepted(5);
    accepted << 0.4, 1.2, -0.7, 0.3, 250.0;
    system.SetTimeAndContinuousState(*context, 3.5, accepted);

    Eigen::VectorXd observed(5);
    system.CopyContinuousState(*context, observed);
    Expect(observed == accepted,
           "a successful transaction writes every block, including the force "
           "state");

    // A state that is legal in q and v and illegal only in its force component.
    // The force block is checked before the model is asked to write anything,
    // so this must leave q, v, the force and the time all untouched.
    Eigen::VectorXd bad_force(5);
    bad_force << 9.9, 9.9, 9.9, 9.9,
        std::numeric_limits<double>::quiet_NaN();
    ExpectInvalidArgument([&] { system.SetTimeAndContinuousState(*context, 7.0, bad_force); },
                  "a state whose only fault is its force component is refused");
    system.CopyContinuousState(*context, observed);
    Expect(observed == accepted && context->time_seconds() == 3.5,
           "and that refusal leaves the time and all three blocks exactly as "
           "they were: q and v are not written before the force is checked");

    // The mirror case: a state whose fault is in q, which the model refuses on
    // its own account. The force must not have been written either.
    Eigen::VectorXd bad_position(5);
    bad_position << std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0, 1.0,
        4242.0;
    ExpectInvalidArgument(
        [&] { system.SetTimeAndContinuousState(*context, 9.0, bad_position); },
        "a state whose fault is in q is refused");
    system.CopyContinuousState(*context, observed);
    Expect(observed == accepted && context->time_seconds() == 3.5,
           "and that refusal leaves the force state untouched too, so neither "
           "half of the write can land without the other");

    // A wrong length is refused before anything is examined.
    Eigen::VectorXd wrong_length(4);
    wrong_length.setZero();
    ExpectInvalidArgument(
        [&] { system.SetTimeAndContinuousState(*context, 1.0, wrong_length); },
        "a state of the wrong length is refused");
    system.CopyContinuousState(*context, observed);
    Expect(observed == accepted && context->time_seconds() == 3.5,
           "and leaves everything unchanged");

    // Two contexts of one system hold their own force state.
    auto other = system.CreateDefaultRuntimeContext(0.0);
    Eigen::VectorXd other_state(5);
    other_state << 0.0, 0.0, 0.0, 0.0, -880.0;
    system.SetTimeAndContinuousState(*other, 0.0, other_state);
    system.CopyContinuousState(*context, observed);
    Expect(observed[4] == 250.0,
           "one context's force state is not the other's");
}

void CheckNamedForceSlotsAndContextParameterCopy() {
    const Fixture fixture;
    const auto rotor = fixture.model.GetFrameByName("rotor");
    const auto slider = fixture.model.GetFrameByName("slider");
    orvd::forces::VehicleForceElementCollection elements;
    elements.translational_spring_dampers = {
        orvd::forces::TranslationalSpringDamper{
            "first_translation", orvd::forces::ForceElementEnd{rotor},
            orvd::forces::ForceElementEnd{slider}, Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()},
        orvd::forces::TranslationalSpringDamper{
            "named_translation", orvd::forces::ForceElementEnd{rotor},
            orvd::forces::ForceElementEnd{slider}, Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()}};
    elements.series_spring_viscous_dampers = {
        orvd::forces::SeriesSpringViscousDamper{
            "first_series", orvd::forces::ForceElementEnd{rotor},
            orvd::forces::ForceElementEnd{slider},
            orvd::forces::ForceElementAxis::kLateral, 3.0e5, 8.0e3},
        orvd::forces::SeriesSpringViscousDamper{
            "named_series", orvd::forces::ForceElementEnd{rotor},
            orvd::forces::ForceElementEnd{slider},
            orvd::forces::ForceElementAxis::kLateral, 4.0e5, 9.0e3}};
    const orvd::forces::VehicleForcePlan plan(fixture.model,
                                               std::move(elements));
    const SystemAssemblyDescription description(fixture.model, plan);
    const SystemInstance system(description);
    const auto nominal_slot =
        system.GetTranslationalSpringDamperIndexByName("named_translation");
    const auto series_slot =
        system.GetSeriesSpringViscousDamperIndexByName("named_series");
    const auto series_range =
        system.series_spring_damper_force_state_range(series_slot);
    Expect(series_range.start() == 5 && series_range.size() == 1,
           "a named series slot resolves past an earlier declaration to its "
           "own force-state component");
    ExpectInvalidArgument(
        [&] {
            (void)system.GetTranslationalSpringDamperIndexByName("absent");
        },
        "an absent translational element name is refused at setup");
    ExpectInvalidArgument(
        [&] { (void)system.GetSeriesSpringViscousDamperIndexByName("absent"); },
        "an absent series element name is refused at setup");

    auto source = system.CreateDefaultRuntimeContext(1.0);
    auto destination = system.CreateDefaultRuntimeContext(2.0);
    Eigen::VectorXd source_state(6);
    source_state << 0.1, 0.2, 0.3, 0.4, 51.0, -62.0;
    Eigen::VectorXd destination_state(6);
    destination_state << -0.7, 0.8, -0.9, 1.0, -73.0, 84.0;
    system.SetContinuousState(*source, source_state);
    system.SetContinuousState(*destination, destination_state);
    const Eigen::Vector3d nominal(17.0, -23.0, 31.0);
    system.SetNominalForce(*source, nominal_slot, nominal);
    system.CopyContextLocalParameters(*source, *destination);
    Expect(destination->nominal_forces().segment<3>(0).isZero() &&
               destination->nominal_forces().segment<3>(3) == nominal,
           "name resolution reaches the named declaration rather than a raw "
           "default index, and context-parameter copy preserves that slot");
    Eigen::VectorXd observed_destination(6);
    system.CopyContinuousState(*destination, observed_destination);
    Expect(destination->time_seconds() == 2.0 &&
               observed_destination == destination_state,
           "context-parameter copy leaves a distinct time and every q, v and "
           "series-force component untouched");

    const SystemInstance other_system(description);
    const auto foreign_nominal_slot =
        other_system.GetTranslationalSpringDamperIndexByName(
            "named_translation");
    const auto foreign_series_slot =
        other_system.GetSeriesSpringViscousDamperIndexByName("named_series");
    auto foreign_context = other_system.CreateDefaultRuntimeContext(3.0);
    other_system.SetNominalForce(
        *foreign_context, foreign_nominal_slot,
        Eigen::Vector3d(-101.0, 102.0, -103.0));
    const Eigen::VectorXd before = destination->nominal_forces();
    ExpectInvalidArgument(
        [&] {
            system.SetNominalForce(*destination, foreign_nominal_slot,
                                   Eigen::Vector3d::Ones());
        },
        "a named nominal-force index cannot cross system instances");
    ExpectInvalidArgument(
        [&] {
            (void)system.series_spring_damper_force_state_range(
                foreign_series_slot);
        },
        "a named series-force index cannot cross system instances");
    Expect(destination->nominal_forces() == before,
           "foreign named-slot refusals leave the destination unchanged");

    const Eigen::VectorXd foreign_before = foreign_context->nominal_forces();
    ExpectInvalidArgument(
        [&] {
            system.CopyContextLocalParameters(*source, *foreign_context);
        },
        "context-parameter copy checks a foreign destination before writing");
    Expect(foreign_context->nominal_forces() == foreign_before,
           "a foreign-destination refusal leaves its parameters unchanged");
    ExpectInvalidArgument(
        [&] {
            system.CopyContextLocalParameters(*foreign_context, *destination);
        },
        "context-parameter copy checks a foreign source before writing");
    Expect(destination->nominal_forces() == before,
           "a foreign-source refusal leaves the destination unchanged");
}

}  // namespace

int main() {
    CheckFrozenLayoutAndSingleOwnership();
    CheckContextLocalDampingAndIsolation();
    CheckOneTransactionOverEveryBlock();
    CheckNamedForceSlotsAndContextParameterCopy();
    if (failure_count != 0) {
        std::printf("%d system instance checks failed\n", failure_count);
        return 1;
    }
    std::printf("system instance ownership and damping contract verified\n");
    return 0;
}
