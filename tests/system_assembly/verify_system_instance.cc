#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_model.h"
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

}  // namespace

int main() {
    CheckFrozenLayoutAndSingleOwnership();
    CheckContextLocalDampingAndIsolation();
    if (failure_count != 0) {
        std::printf("%d system instance checks failed\n", failure_count);
        return 1;
    }
    std::printf("system instance ownership and damping contract verified\n");
    return 0;
}
