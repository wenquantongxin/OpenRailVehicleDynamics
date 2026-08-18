#pragma once

/// @file
/// Accepted/trial transaction boundary for continuous system advancement.

#include <memory>
#include <span>

#include <Eigen/Dense>

#include "orvd/integrators/continuous_state_advancer.h"
#include "orvd/integrators/system_rhs_bridge.h"

namespace orvd::system_assembly {
class CompiledSystemPlan;
class SystemInstance;
class SystemRuntimeContext;
}  // namespace orvd::system_assembly

namespace orvd::integrators {

namespace internal {
class BdfIntegrationAccess;
enum class MaximumBdfOrder : int;
}

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
    /// `SynchronizeAfterAcceptedContextChange()` succeeds. One public advance
    /// may accept at most 1,000,000 internal steps.
    void AdvanceTo(double target_time_seconds);

    /// Advances once while returning selected states from the successful
    /// internal-step dense intervals.
    ///
    /// `sample_times_seconds` must be non-empty, finite and strictly
    /// increasing. Its first entry must equal the accepted time and its last
    /// entry must equal `target_time_seconds`. The first and last columns equal
    /// the entry and accepted endpoint states, respectively. Intermediate
    /// sample times do not become CVODE stops.
    ///
    /// The returned matrix and the accepted endpoint are one transaction. Any
    /// failure after entering the backend returns no matrix, leaves the
    /// accepted context unchanged, and requires explicit synchronization. A
    /// same-time request is admitted only as one sample and does not enter the
    /// backend.
    [[nodiscard]] Eigen::MatrixXd AdvanceToWithDenseStateSamples(
        double target_time_seconds,
        std::span<const double> sample_times_seconds);

    /// Returns cumulative backend work without entering the RHS hot path.
    /// The counters restart after a successful explicit synchronization,
    /// because synchronization reinitializes the numerical backend.
    [[nodiscard]] ContinuousStateIntegrationStatistics
    integration_statistics() const;

    /// Copies the accepted state, admitted context-local data and latest
    /// wheel-rail projection branches into the trial/backend configuration,
    /// then reinitializes numerical history.
    ///
    /// Call this after an external accepted-state or admitted context-local
    /// physical-parameter change, and after a failed advance. A caller that
    /// relocates a vehicle to an unrelated track branch must construct a newly
    /// resolved start-up context rather than asking this local history to find
    /// a global root. It is deliberately explicit and is not part of the RHS
    /// hot path.
    void SynchronizeAfterAcceptedContextChange();

   private:
    friend class internal::BdfIntegrationAccess;

    SystemContinuousStateAdvancer(
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& accepted_context,
        ContinuousStateErrorTolerances tolerances,
        NoCallTimeAppliedForces no_call_time_applied_forces,
        internal::MaximumBdfOrder maximum_bdf_order);

    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators
