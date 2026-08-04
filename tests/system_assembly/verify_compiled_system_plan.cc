#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"

namespace {

using orvd::multibody_model::AppliedBodyWrench;
using orvd::multibody_model::AppliedPrismaticJointForce;
using orvd::multibody_model::AppliedRevoluteJointTorque;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::CompiledSystemPlan;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;

static_assert(!std::is_copy_constructible_v<CompiledSystemPlan>);
static_assert(!std::is_move_constructible_v<CompiledSystemPlan>);
static_assert(std::is_constructible_v<CompiledSystemPlan, SystemInstance&>);
static_assert(!std::is_constructible_v<CompiledSystemPlan, SystemInstance&&>);
static_assert(
    !std::is_constructible_v<CompiledSystemPlan, const SystemInstance&&>);

int failure_count = 0;

void Expect(bool condition, const char* description) {
    if (!condition) {
        std::printf("FAIL %s\n", description);
        ++failure_count;
    }
}

void ExpectNear(double actual, double expected, const char* description) {
    const double tolerance = 256.0 * std::numeric_limits<double>::epsilon() *
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

RigidBodyInertiaParameters MakeInertia() {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 2.0;
    inertia.center_of_mass_in_body_frame.setZero();
    inertia.unit_inertia_moments = Eigen::Vector3d(0.4, 0.45, 0.5);
    inertia.unit_inertia_products.setZero();
    return inertia;
}

struct Fixture {
    MultibodyModel model;
    RigidBodyHandle body;
    orvd::multibody_model::JointHandle joint;

    Fixture() {
        body = model.AddRigidBody("body", MakeInertia());
        joint = model.AddRevoluteJoint(
            "joint", model.world_frame(), model.body_frame(body),
            Eigen::Vector3d::UnitZ(), 0.3);
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();
    }
};

void CheckStaticEvaluationAndStatePurity() {
    Fixture fixture;
    const SystemAssemblyDescription description(fixture.model);
    const SystemInstance system(description);
    const CompiledSystemPlan plan(system);
    Expect(plan.derivative_component() == system.multibody_component(),
           "the frozen order ends in the admitted multibody component");

    auto context = system.CreateDefaultRuntimeContext(0.0);
    const auto component = system.GetMultibodyComponentView(
        *context, system.multibody_component());
    Eigen::VectorXd positions(1);
    positions << 0.4;
    Eigen::VectorXd velocities(1);
    velocities << 2.0;
    fixture.model.SetGeneralizedPositions(&component.context(), positions);
    fixture.model.SetGeneralizedVelocities(&component.context(), velocities);
    const Eigen::VectorXd accepted_positions =
        context->generalized_positions();
    const Eigen::VectorXd accepted_velocities =
        context->generalized_velocities();

    Eigen::VectorXd derivatives(2);
    plan.CalcStateTimeDerivatives(*context, derivatives);
    ExpectNear(derivatives[0], 2.0,
               "the static plan evaluates qdot = N(q)v first");
    // Izz = mass * unit Izz = 1.0; damping contributes -0.3 * 2. The system
    // carries no call-time applied force, so the joint torque that used to be
    // passed here is gone and only damping remains.
    ExpectNear(derivatives[1], -0.6,
               "the static plan assembles forces and evaluates vdot once");
    Expect(context->generalized_positions() == accepted_positions,
           "a successful derivative trial does not write accepted q");
    Expect(context->generalized_velocities() == accepted_velocities,
           "a successful derivative trial does not write accepted v");

    fixture.model.SetRevoluteJointDampingCoefficient(
        &component.context(), fixture.joint, 0.5);
    Eigen::VectorXd accepted_damping_force(1);
    fixture.model.CalcJointDampingAppliedGeneralizedForces(
        component.context(), accepted_damping_force);
    plan.CalcStateTimeDerivatives(*context, derivatives);
    ExpectNear(derivatives[1], -1.0,
               "the next static evaluation reads context-local damping");
    Eigen::VectorXd observed_damping_force(1);
    fixture.model.CalcJointDampingAppliedGeneralizedForces(
        component.context(), observed_damping_force);
    Expect(observed_damping_force == accepted_damping_force,
           "a successful evaluation does not write physical parameters");

    // The call-time applied-force entry point lives one layer below: the system
    // has no such input, and a caller who wants an arbitrary wrench for research
    // or for a unit test reaches the facade directly. These three refusals are
    // therefore stated against the facade, which still owns them.
    Eigen::VectorXd facade_derivatives(2);
    const std::array invalid_torque{AppliedRevoluteJointTorque{
        fixture.joint, std::numeric_limits<double>::quiet_NaN()}};
    ExpectInvalidArgument(
        [&] {
            fixture.model.CalcStateTimeDerivatives(
                component.context(), {}, invalid_torque, {},
                component.forward_dynamics_workspace(), facade_derivatives);
        },
        "an invalid revolute torque trial fails explicitly");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const std::array invalid_body_wrench{AppliedBodyWrench{
        fixture.body, Eigen::Vector3d::Zero(), fixture.model.world_frame(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d(nan, 0.0, 0.0)}};
    ExpectInvalidArgument(
        [&] {
            fixture.model.CalcStateTimeDerivatives(
                component.context(), invalid_body_wrench, {}, {},
                component.forward_dynamics_workspace(), facade_derivatives);
        },
        "the facade rejects an invalid body wrench");
    const std::array wrong_kind_prismatic_force{
        AppliedPrismaticJointForce{fixture.joint, 1.0}};
    ExpectInvalidArgument(
        [&] {
            fixture.model.CalcStateTimeDerivatives(
                component.context(), {}, {}, wrong_kind_prismatic_force,
                component.forward_dynamics_workspace(), facade_derivatives);
        },
        "the facade rejects a wrong-kind prismatic force");

    // And the plan itself refuses an output of the wrong length before it
    // evaluates anything.
    Eigen::VectorXd wrong_size(3);
    wrong_size.setConstant(11.0);
    ExpectInvalidArgument(
        [&] { plan.CalcStateTimeDerivatives(*context, wrong_size); },
        "the plan refuses a derivative output of the wrong size");
    Expect(wrong_size == Eigen::VectorXd::Constant(3, 11.0),
           "that refusal precedes any write to the output");

    Expect(context->generalized_positions() == accepted_positions,
           "rejected derivative trials do not write accepted q");
    Expect(context->generalized_velocities() == accepted_velocities,
           "rejected derivative trials do not write accepted v");
    fixture.model.CalcJointDampingAppliedGeneralizedForces(
        component.context(), observed_damping_force);
    Expect(observed_damping_force == accepted_damping_force,
           "rejected derivative trials do not write physical parameters");

    auto other_context = system.CreateDefaultRuntimeContext(0.0);
    plan.CalcStateTimeDerivatives(*other_context, derivatives);
    ExpectNear(derivatives[0], 0.0,
               "each accepted context is evaluated through its own state");
}

void CheckForeignContextRefusal() {
    Fixture fixture;
    const SystemAssemblyDescription first_description(fixture.model);
    const SystemAssemblyDescription second_description(fixture.model);
    const SystemInstance first(first_description);
    const SystemInstance second(second_description);
    const CompiledSystemPlan plan(first);
    auto foreign_context = second.CreateDefaultRuntimeContext(0.0);
    Eigen::VectorXd derivatives(2);
    derivatives.setConstant(17.0);
    bool refused = false;
    try {
        plan.CalcStateTimeDerivatives(*foreign_context, derivatives);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused,
           "a compiled plan cannot evaluate another system's root context");
    Expect(derivatives == Eigen::VectorXd::Constant(2, 17.0),
           "a foreign root context is rejected before output evaluation");
}

}  // namespace

int main() {
    CheckStaticEvaluationAndStatePurity();
    CheckForeignContextRefusal();
    if (failure_count != 0) {
        std::printf("%d compiled system plan checks failed\n", failure_count);
        return 1;
    }
    std::printf("compiled system evaluation plan verified\n");
    return 0;
}
