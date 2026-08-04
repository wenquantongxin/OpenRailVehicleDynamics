#include <cstdio>
#include <stdexcept>
#include <type_traits>

#include <Eigen/Dense>

#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/system_assembly_description.h"

namespace {

using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::forces::VehicleForcePlan;
using orvd::system_assembly::SystemAssemblyDescription;

static_assert(!std::is_copy_constructible_v<SystemAssemblyDescription>);
static_assert(!std::is_copy_assignable_v<SystemAssemblyDescription>);
static_assert(!std::is_move_constructible_v<SystemAssemblyDescription>);
static_assert(!std::is_move_assignable_v<SystemAssemblyDescription>);
static_assert(
    std::is_constructible_v<SystemAssemblyDescription, MultibodyModel&>);
static_assert(
    !std::is_constructible_v<SystemAssemblyDescription, MultibodyModel&&>);
static_assert(!std::is_constructible_v<SystemAssemblyDescription,
                                       const MultibodyModel&&>);
static_assert(!std::is_constructible_v<SystemAssemblyDescription,
                                       MultibodyModel&&, VehicleForcePlan&>);
static_assert(!std::is_constructible_v<SystemAssemblyDescription,
                                       const MultibodyModel&, VehicleForcePlan&&>);

int failure_count = 0;

void Expect(bool condition, const char* description) {
    if (!condition) {
        std::printf("FAIL %s\n", description);
        ++failure_count;
    }
}

RigidBodyInertiaParameters MakeInertia() {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 2.0;
    inertia.center_of_mass_in_body_frame = Eigen::Vector3d(0.1, 0.0, 0.0);
    inertia.unit_inertia_moments = Eigen::Vector3d(0.3, 0.4, 0.5);
    inertia.unit_inertia_products.setZero();
    return inertia;
}

void CheckFinalizationBoundary() {
    MultibodyModel model;
    bool refused = false;
    try {
        const SystemAssemblyDescription description(model);
        (void)description;
    } catch (const std::logic_error&) {
        refused = true;
    }
    Expect(refused, "an unfinalized model is refused at description time");
}

void CheckFixedDescription() {
    MultibodyModel model;
    const auto body = model.AddRigidBody("body", MakeInertia());
    model.AddRevoluteJoint("joint", model.world_frame(),
                           model.body_frame(body), Eigen::Vector3d::UnitZ(),
                           0.2);
    model.Finalize();

    const SystemAssemblyDescription description(model);
    Expect(&description.multibody_model() == &model,
           "the description keeps the stated finalized component identity");
    Expect(description.generalized_position_count() == 1,
           "the q state block has the finalized model size");
    Expect(description.generalized_velocity_count() == 1,
           "the v state block has the finalized model size");
    Expect(description.state_time_derivative_size() == 2,
           "the sole output is the complete [qdot; vdot] vector");
}

}  // namespace

int main() {
    CheckFinalizationBoundary();
    CheckFixedDescription();
    if (failure_count != 0) {
        std::printf("%d system assembly description checks failed\n",
                    failure_count);
        return 1;
    }
    std::printf("system assembly description contract verified\n");
    return 0;
}
