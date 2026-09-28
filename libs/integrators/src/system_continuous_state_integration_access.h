#pragma once

/// @file
/// Single source-tree-only construction seam for system integration recipes.

#include <memory>
#include <optional>
#include <type_traits>

#include "orvd/integrators/system_continuous_state_advancer.h"

#include "system_continuous_state_integration_recipe.h"
#include "system_continuous_state_integration_configuration.h"
#include "coordinate_second_order_problem.h"

namespace orvd::integrators::internal {

class SystemContinuousStateIntegrationAccess final {
   public:
    [[nodiscard]] static std::unique_ptr<SystemContinuousStateAdvancer> Make(
        SystemContinuousStateIntegrationConfiguration configuration,
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& accepted_context,
        NoCallTimeAppliedForces no_call_time_applied_forces);
    template <class System, class Plan>
        requires (!std::is_lvalue_reference_v<System&&> ||
                  !std::is_lvalue_reference_v<Plan&&>)
    static std::unique_ptr<SystemContinuousStateAdvancer> Make(
        SystemContinuousStateIntegrationConfiguration, System&&, Plan&&,
        system_assembly::SystemRuntimeContext&, NoCallTimeAppliedForces) = delete;

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

    [[nodiscard]] static std::optional<CoordinateIntegrationDiagnostics>
    CoordinateDiagnostics(const SystemContinuousStateAdvancer& advancer);

};

}  // namespace orvd::integrators::internal
