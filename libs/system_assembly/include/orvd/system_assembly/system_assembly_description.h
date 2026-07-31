#pragma once

/// @file
/// The immutable, model-neutral description of the first executable system.
///
/// The first consumer is one finalized multibody component.  Its continuous
/// state consists of q and v, its physical parameters remain the component's
/// typed context-local records, its call-time force inputs use the three
/// existing physical records below, and its sole output is [qdot; vdot].
///
/// This is deliberately not a port registry.  There is no string lookup,
/// append operation, generic parameter bag, event collection or runtime value
/// here.  G41 turns this description into an executable instance and is the
/// first place that may create a context or a hot-path workspace.

#include "orvd/multibody_model/multibody_applied_forces.h"

namespace orvd::multibody_model {
class MultibodyModel;
}

namespace orvd::system_assembly {

class SystemAssemblyDescription {
   public:
    using BodyWrenchInput = multibody_model::AppliedBodyWrench;
    using RevoluteJointTorqueInput =
        multibody_model::AppliedRevoluteJointTorque;
    using PrismaticJointForceInput =
        multibody_model::AppliedPrismaticJointForce;

    /// Describes `model` without taking ownership or creating runtime state.
    /// The model must outlive this description and every system compiled from
    /// it.
    ///
    /// @throws std::logic_error if the model has not been finalized.
    explicit SystemAssemblyDescription(
        const multibody_model::MultibodyModel& model);
    SystemAssemblyDescription(multibody_model::MultibodyModel&&) = delete;

    SystemAssemblyDescription(const SystemAssemblyDescription&) = delete;
    SystemAssemblyDescription& operator=(const SystemAssemblyDescription&) =
        delete;
    SystemAssemblyDescription(SystemAssemblyDescription&&) = delete;
    SystemAssemblyDescription& operator=(SystemAssemblyDescription&&) = delete;

    /// The sole component in the first assembly contract.
    [[nodiscard]] const multibody_model::MultibodyModel& multibody_model() const;

    /// Sizes of the two distinct continuous-state blocks.
    [[nodiscard]] int generalized_position_count() const;
    [[nodiscard]] int generalized_velocity_count() const;

    /// Size of the sole output [qdot; vdot].
    [[nodiscard]] int state_time_derivative_size() const;

   private:
    const multibody_model::MultibodyModel* multibody_model_;
    int generalized_position_count_{};
    int generalized_velocity_count_{};
};

}  // namespace orvd::system_assembly
