#pragma once

/// @file
/// Single source-tree-only construction seam for system integration recipes.

#include <cstdint>
#include <memory>

#include "orvd/integrators/system_continuous_state_advancer.h"

#include "system_continuous_state_integration_recipe.h"

namespace orvd::integrators::internal {

class SystemContinuousStateIntegrationAccess final {
   public:
    [[nodiscard]] static std::unique_ptr<SystemContinuousStateAdvancer> Make(
        SystemContinuousStateIntegrationRecipe recipe,
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& accepted_context,
        ContinuousStateErrorTolerances tolerances,
        NoCallTimeAppliedForces no_call_time_applied_forces);
    [[nodiscard]] static std::unique_ptr<SystemContinuousStateAdvancer> Make(
        SystemContinuousStateIntegrationRecipe,
        system_assembly::SystemInstance&&,
        const system_assembly::CompiledSystemPlan&,
        system_assembly::SystemRuntimeContext&, ContinuousStateErrorTolerances,
        NoCallTimeAppliedForces) = delete;
    [[nodiscard]] static std::unique_ptr<SystemContinuousStateAdvancer> Make(
        SystemContinuousStateIntegrationRecipe,
        const system_assembly::SystemInstance&&,
        const system_assembly::CompiledSystemPlan&,
        system_assembly::SystemRuntimeContext&, ContinuousStateErrorTolerances,
        NoCallTimeAppliedForces) = delete;
    [[nodiscard]] static std::unique_ptr<SystemContinuousStateAdvancer> Make(
        SystemContinuousStateIntegrationRecipe,
        const system_assembly::SystemInstance&,
        system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&, ContinuousStateErrorTolerances,
        NoCallTimeAppliedForces) = delete;
    [[nodiscard]] static std::unique_ptr<SystemContinuousStateAdvancer> Make(
        SystemContinuousStateIntegrationRecipe,
        const system_assembly::SystemInstance&,
        const system_assembly::CompiledSystemPlan&&,
        system_assembly::SystemRuntimeContext&, ContinuousStateErrorTolerances,
        NoCallTimeAppliedForces) = delete;

    [[nodiscard]] static SystemContinuousStateIntegrationRecipe
    ConfiguredRecipe(const SystemContinuousStateAdvancer& advancer);

    // Source-tree qualification diagnostic. This is deliberately not part of
    // the installed advancer or generic numerical-statistics contracts.
    [[nodiscard]] static std::uint64_t
    RecoverableProjectionWindowMissClassificationCount(
        const SystemContinuousStateAdvancer& advancer);
};

}  // namespace orvd::integrators::internal
