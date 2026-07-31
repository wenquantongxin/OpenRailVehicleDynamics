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

using orvd::multibody_model::AppliedRevoluteJointTorque;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::CompiledSystemPlan;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;

static_assert(!std::is_copy_constructible_v<CompiledSystemPlan>);
static_assert(!std::is_move_constructible_v<CompiledSystemPlan>);
static_assert(std::is_constructible_v<CompiledSystemPlan, SystemInstance&>);
static_assert(!std::is_constructible_v<CompiledSystemPlan, SystemInstance&&>);

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
    orvd::multibody_model::JointHandle joint;

    Fixture() {
        const auto body = model.AddRigidBody("body", MakeInertia());
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

    auto context = system.CreateDefaultRuntimeContext();
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

    const std::array torque{
        AppliedRevoluteJointTorque{fixture.joint, 1.25}};
    Eigen::VectorXd derivatives(2);
    plan.CalcStateTimeDerivatives(*context, {}, torque, {}, derivatives);
    ExpectNear(derivatives[0], 2.0,
               "the static plan evaluates qdot = N(q)v first");
    // Izz = mass * unit Izz = 1.0; damping contributes -0.3 * 2.
    ExpectNear(derivatives[1], 0.65,
               "the static plan assembles forces and evaluates vdot once");
    Expect(context->generalized_positions() == accepted_positions,
           "a successful derivative trial does not write accepted q");
    Expect(context->generalized_velocities() == accepted_velocities,
           "a successful derivative trial does not write accepted v");

    const std::array invalid_torque{AppliedRevoluteJointTorque{
        fixture.joint, std::numeric_limits<double>::quiet_NaN()}};
    bool refused = false;
    try {
        plan.CalcStateTimeDerivatives(*context, {}, invalid_torque, {},
                                      derivatives);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused, "an invalid force trial fails explicitly");
    Expect(context->generalized_positions() == accepted_positions,
           "a failed derivative trial does not write accepted q");
    Expect(context->generalized_velocities() == accepted_velocities,
           "a failed derivative trial does not write accepted v");

    auto other_context = system.CreateDefaultRuntimeContext();
    plan.CalcStateTimeDerivatives(*other_context, {}, {}, {}, derivatives);
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
    auto foreign_context = second.CreateDefaultRuntimeContext();
    Eigen::VectorXd derivatives(2);
    bool refused = false;
    try {
        plan.CalcStateTimeDerivatives(*foreign_context, {}, {}, {},
                                      derivatives);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused,
           "a compiled plan cannot evaluate another system's root context");
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
