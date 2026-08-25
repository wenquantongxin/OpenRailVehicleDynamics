#pragma once

/// @file
/// Source-private owner of one concrete system integration backend.

#include <cstdint>
#include <memory>

#include <Eigen/Dense>

#include "orvd/integrators/continuous_state_advancer.h"
#include "orvd/integrators/system_rhs_bridge.h"

#include "system_continuous_state_integration_recipe.h"

namespace orvd::system_assembly {
class CompiledSystemPlan;
class SystemInstance;
class SystemRuntimeContext;
}  // namespace orvd::system_assembly

namespace orvd::integrators::internal {

class SystemContinuousStateBackend final {
   public:
    SystemContinuousStateBackend(
        SystemContinuousStateIntegrationRecipe recipe,
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& candidate_context,
        const system_assembly::SystemRuntimeContext& accepted_context,
        const Eigen::VectorXd& initial_continuous_state,
        ContinuousStateErrorTolerances tolerances,
        NoCallTimeAppliedForces no_call_time_applied_forces);
    SystemContinuousStateBackend(
        SystemContinuousStateIntegrationRecipe,
        system_assembly::SystemInstance&&,
        const system_assembly::CompiledSystemPlan&,
        system_assembly::SystemRuntimeContext&,
        const system_assembly::SystemRuntimeContext&, const Eigen::VectorXd&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateBackend(
        SystemContinuousStateIntegrationRecipe,
        const system_assembly::SystemInstance&&,
        const system_assembly::CompiledSystemPlan&,
        system_assembly::SystemRuntimeContext&,
        const system_assembly::SystemRuntimeContext&, const Eigen::VectorXd&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateBackend(
        SystemContinuousStateIntegrationRecipe,
        const system_assembly::SystemInstance&,
        system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&,
        const system_assembly::SystemRuntimeContext&, const Eigen::VectorXd&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateBackend(
        SystemContinuousStateIntegrationRecipe,
        const system_assembly::SystemInstance&,
        const system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&,
        const system_assembly::SystemRuntimeContext&, const Eigen::VectorXd&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateBackend(
        SystemContinuousStateIntegrationRecipe,
        system_assembly::SystemInstance&&,
        system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&,
        const system_assembly::SystemRuntimeContext&, const Eigen::VectorXd&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateBackend(
        SystemContinuousStateIntegrationRecipe,
        system_assembly::SystemInstance&&,
        const system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&,
        const system_assembly::SystemRuntimeContext&, const Eigen::VectorXd&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateBackend(
        SystemContinuousStateIntegrationRecipe,
        const system_assembly::SystemInstance&&,
        system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&,
        const system_assembly::SystemRuntimeContext&, const Eigen::VectorXd&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    SystemContinuousStateBackend(
        SystemContinuousStateIntegrationRecipe,
        const system_assembly::SystemInstance&&,
        const system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&,
        const system_assembly::SystemRuntimeContext&, const Eigen::VectorXd&,
        ContinuousStateErrorTolerances, NoCallTimeAppliedForces) = delete;
    ~SystemContinuousStateBackend();

    SystemContinuousStateBackend(const SystemContinuousStateBackend&) =
        delete;
    SystemContinuousStateBackend& operator=(
        const SystemContinuousStateBackend&) = delete;
    SystemContinuousStateBackend(SystemContinuousStateBackend&&) = delete;
    SystemContinuousStateBackend& operator=(
        SystemContinuousStateBackend&&) = delete;

    [[nodiscard]] ContinuousStateAdvancer& advancer();
    [[nodiscard]] const ContinuousStateAdvancer& advancer() const;
    [[nodiscard]] SystemContinuousStateIntegrationRecipe configured_recipe()
        const noexcept;
    void SynchronizeContextLocalDataFrom(
        const system_assembly::SystemRuntimeContext& accepted_context);

    /// Notifies the selected backend that accepted wheel--rail projection
    /// history changed outside the continuous state.  Backends that retain a
    /// linearization may invalidate it without resetting accepted numerical
    /// history.
    void NotifyAcceptedProjectionHistoryChange();

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators::internal
