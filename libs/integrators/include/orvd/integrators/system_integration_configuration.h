#pragma once

#include <cstddef>
#include <utility>
#include <variant>

#include "orvd/integrators/continuous_state_advancer.h"
#include "orvd/integrators/mechanical_integration_configuration.h"

namespace orvd::integrators {

inline constexpr std::size_t kDefaultMaximumInternalStepsPerAdvance = 1'000'000;

struct CvodeBdf2Configuration final {
    ContinuousStateErrorTolerances tolerances;
};
struct CvodeBdf5Configuration final {
    ContinuousStateErrorTolerances tolerances;
};
struct Radau5Configuration final {
    ContinuousStateErrorTolerances tolerances;
};

using SystemIntegrationMethodConfiguration =
    std::variant<CvodeBdf2Configuration, CvodeBdf5Configuration,
                 Radau5Configuration, NewmarkConfiguration, ZhaiConfiguration>;

/// Owns a complete method recipe. Applications explicitly choose the method;
/// an observation clock never supplies a mechanical method's step size.
struct SystemIntegrationConfiguration final {
    explicit SystemIntegrationConfiguration(
        SystemIntegrationMethodConfiguration selected_method,
        std::size_t step_budget = kDefaultMaximumInternalStepsPerAdvance)
        : method(std::move(selected_method)),
          maximum_internal_steps_per_advance(step_budget) {}

    SystemIntegrationMethodConfiguration method;
    /// Maximum successful internal steps in one public advance; must be positive.
    std::size_t maximum_internal_steps_per_advance;
};

}  // namespace orvd::integrators
