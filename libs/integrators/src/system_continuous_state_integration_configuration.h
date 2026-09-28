#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <variant>

#include "orvd/integrators/continuous_state_advancer.h"

#include "basic_mechanical_integration_configuration.h"
#include "integrator_limits.h"
#include "system_continuous_state_integration_recipe.h"

namespace orvd::integrators::internal {

struct CvodeBdf2Configuration final {
    ContinuousStateErrorTolerances tolerances;
};
struct CvodeBdf5Configuration final {
    ContinuousStateErrorTolerances tolerances;
};
struct Radau5Configuration final {
    ContinuousStateErrorTolerances tolerances;
};

// The alternative carries its own method identity and all required parameters.
// A mechanical method never receives an unused ODE tolerance vector.
using SystemIntegrationMethodConfiguration =
    std::variant<CvodeBdf2Configuration, CvodeBdf5Configuration,
                 Radau5Configuration, NewmarkConfiguration, ZhaiConfiguration>;

struct SystemContinuousStateIntegrationConfiguration final {
    SystemIntegrationMethodConfiguration method;
    std::size_t maximum_internal_steps_per_public_advance{
        kMaximumInternalStepsPerPublicAdvance};
};

// Compatibility seam for the existing source-tree ODE consumers only.
[[nodiscard]] inline SystemIntegrationMethodConfiguration
MakeOdeMethodConfiguration(SystemContinuousStateIntegrationRecipe recipe,
                           ContinuousStateErrorTolerances tolerances) {
    switch (recipe) {
        case SystemContinuousStateIntegrationRecipe::kCvodeBdf2:
            return CvodeBdf2Configuration{std::move(tolerances)};
        case SystemContinuousStateIntegrationRecipe::kCvodeBdf5:
            return CvodeBdf5Configuration{std::move(tolerances)};
        case SystemContinuousStateIntegrationRecipe::kRadau5:
            return Radau5Configuration{std::move(tolerances)};
        case SystemContinuousStateIntegrationRecipe::kNewmark:
        case SystemContinuousStateIntegrationRecipe::kZhai:
            throw std::invalid_argument(
                "system integration: mechanical methods require their explicit "
                "step and method configuration");
    }
    throw std::invalid_argument("system integration: unsupported recipe");
}

}  // namespace orvd::integrators::internal
