#pragma once

/// @file
/// Accepted/trial transaction boundary for continuous system advancement.

#include <memory>

#include "orvd/integrators/continuous_state_advancer.h"
#include "orvd/integrators/system_rhs_bridge.h"

namespace orvd::system_assembly {
class CompiledSystemPlan;
class SystemInstance;
class SystemRuntimeContext;
}  // namespace orvd::system_assembly

namespace orvd::integrators {

/// Advances one compiled system while keeping trial state out of its accepted
/// runtime context.
///
/// `system`, `plan` and `accepted_context` are borrowed and must outlive this
/// object.  The dedicated RHS context and CVODE backend are private.  A
/// successful public advance accepts time and the complete continuous state
/// exactly once; a failed advance requires explicit synchronization from the
/// still-valid accepted context before another attempt.
class SystemContinuousStateAdvancer final {
   public:
    SystemContinuousStateAdvancer(
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& accepted_context,
        ContinuousStateErrorTolerances tolerances,
        NoCallTimeAppliedForces);
    ~SystemContinuousStateAdvancer();

    SystemContinuousStateAdvancer(
        system_assembly::SystemInstance&&,
        const system_assembly::CompiledSystemPlan&,
        system_assembly::SystemRuntimeContext&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateAdvancer(
        const system_assembly::SystemInstance&&,
        const system_assembly::CompiledSystemPlan&,
        system_assembly::SystemRuntimeContext&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateAdvancer(
        const system_assembly::SystemInstance&,
        system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateAdvancer(
        const system_assembly::SystemInstance&,
        const system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;

    SystemContinuousStateAdvancer(const SystemContinuousStateAdvancer&) =
        delete;
    SystemContinuousStateAdvancer& operator=(
        const SystemContinuousStateAdvancer&) = delete;
    SystemContinuousStateAdvancer(SystemContinuousStateAdvancer&&) = delete;
    SystemContinuousStateAdvancer& operator=(SystemContinuousStateAdvancer&&) =
        delete;

    /// Advances to a finite time no earlier than the accepted time.
    ///
    /// A same-time request is a no-op.  Any failure after entering the backend
    /// leaves the accepted context unchanged and blocks another advance until
    /// `SynchronizeAfterAcceptedContextChange()` succeeds.
    void AdvanceTo(double target_time_seconds);

    /// Copies the accepted state and admitted context-local parameters into the
    /// trial/backend configuration, then reinitializes numerical history.
    ///
    /// Call this after an external accepted-state or joint-damping change, and
    /// after a failed advance.  It is deliberately explicit and is not part of
    /// the RHS hot path.
    void SynchronizeAfterAcceptedContextChange();

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators
