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

#include "orvd/system_assembly/system_instance.h"

namespace orvd::forces {
class VehicleForcePlan;
}

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

    /// Evaluates the current accepted state without writing q, v, z or any
    /// context-local physical parameter.  Logical caches, the model-bound call
    /// workspace and the context's own scratch may be updated and reused.
    ///
    /// There is no call-time applied force. A system's forces are the vehicle's
    /// own force elements; a caller who wants to apply an arbitrary wrench for
    /// research or for a unit test calls the multibody facade directly, which
    /// keeps that entry point and is one layer below this one. Carrying an
    /// always-empty span through this level would only make the system look as
    /// though it had a second force source that nothing supplies.
    ///
    /// The caller owns the already-sized output, which is [qdot; vdot; zdot].
    /// Validation and failure atomicity are the multibody facade's existing
    /// contract.
    void CalcStateTimeDerivatives(
        SystemRuntimeContext& context,
        Eigen::VectorXd& state_time_derivatives) const;

   private:
    const SystemInstance* system_;
    MultibodyComponentIndex derivative_component_;
    const forces::VehicleForcePlan* force_plan_;
};

}  // namespace orvd::system_assembly
