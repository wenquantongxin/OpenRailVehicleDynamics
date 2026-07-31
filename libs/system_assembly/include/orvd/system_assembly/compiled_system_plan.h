#pragma once

/// @file
/// The compiled evaluation order of the first executable system.
///
/// The admitted graph currently contains one multibody component and no
/// event-producing component.  Its static order is therefore one operation:
/// assemble the three typed force inputs and evaluate [qdot; vdot] once.  The
/// component's existing versioned caches and its preallocated forward-dynamics
/// workspace remain the only cache/workspace mechanisms; this plan does not
/// build a second dependency graph.
///
/// Initialization updates, periodic updates and periodic publishes are empty by
/// construction in this first graph.  No generic event, witness, per-step,
/// monitor, forced-event or discrete-update type is exposed.  Timing and commit
/// semantics for the first real event-producing consumer are fixed in
/// ADR-0003; an API appears only when that consumer does.

#include <span>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_applied_forces.h"
#include "orvd/system_assembly/system_instance.h"

namespace orvd::system_assembly {

class CompiledSystemPlan {
   public:
    /// Compiles `system` once.  The non-movable system must outlive the plan.
    explicit CompiledSystemPlan(const SystemInstance& system);
    CompiledSystemPlan(SystemInstance&&) = delete;
    CompiledSystemPlan(const SystemInstance&&) = delete;

    CompiledSystemPlan(const CompiledSystemPlan&) = delete;
    CompiledSystemPlan& operator=(const CompiledSystemPlan&) = delete;
    CompiledSystemPlan(CompiledSystemPlan&&) = delete;
    CompiledSystemPlan& operator=(CompiledSystemPlan&&) = delete;

    /// The last and only derivative-producing operation in the current static
    /// order.
    [[nodiscard]] MultibodyComponentIndex derivative_component() const {
        return derivative_component_;
    }

    /// Evaluates the current accepted state without writing q, v or any
    /// context-local physical parameter.  Logical caches and the model-bound
    /// call workspace may be updated and reused.
    ///
    /// The caller owns the already-sized output.  Validation and failure
    /// atomicity are the multibody facade's existing contract.
    void CalcStateTimeDerivatives(
        SystemRuntimeContext& context,
        std::span<const multibody_model::AppliedBodyWrench> body_wrenches,
        std::span<const multibody_model::AppliedRevoluteJointTorque>
            revolute_joint_torques,
        std::span<const multibody_model::AppliedPrismaticJointForce>
            prismatic_joint_forces,
        Eigen::VectorXd& state_time_derivatives) const;

   private:
    const SystemInstance* system_;
    MultibodyComponentIndex derivative_component_;
};

}  // namespace orvd::system_assembly
