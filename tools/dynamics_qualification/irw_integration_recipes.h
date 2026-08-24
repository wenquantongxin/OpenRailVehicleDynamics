#pragma once

#include "system_continuous_state_integration_recipe.h"

namespace orvd::dynamics_qualification::internal {

// Closed numerical identities for the IRW qualification scenarios.
// These recipes are private to the uninstalled qualification tools; they are
// not an integrator policy API.
struct IrwIntegrationRecipe final {
    constexpr IrwIntegrationRecipe(
        double relative_tolerance_value,
        double position_absolute_tolerance_value,
        double velocity_absolute_tolerance_value,
        double series_force_absolute_tolerance_newtons_value,
        integrators::internal::SystemContinuousStateIntegrationRecipe
            default_integration_recipe_value)
        : relative_tolerance(relative_tolerance_value),
          position_absolute_tolerance(position_absolute_tolerance_value),
          velocity_absolute_tolerance(velocity_absolute_tolerance_value),
          series_force_absolute_tolerance_newtons(
              series_force_absolute_tolerance_newtons_value),
          default_integration_recipe(default_integration_recipe_value) {}
    IrwIntegrationRecipe() = delete;

    double relative_tolerance{};
    double position_absolute_tolerance{};
    double velocity_absolute_tolerance{};
    double series_force_absolute_tolerance_newtons{};
    integrators::internal::SystemContinuousStateIntegrationRecipe
        default_integration_recipe;
};

inline constexpr IrwIntegrationRecipe
    kIrwR300NoIrregularityV60PassiveIntegrationRecipe{
        1.0e-6,
        1.0e-6,
        1.0e-5,
        1.0e-6,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf2,
    };

inline constexpr IrwIntegrationRecipe
    kIrwR300Aar5V60PassiveIntegrationRecipe{
    1.0e-8,
    1.0e-8,
    1.0e-7,
    1.0e-6,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf5,
    };

inline constexpr IrwIntegrationRecipe
    kIrwR300Aar5V60At100HzFullStateGuidanceIntegrationRecipe{
        1.0e-6,
        1.0e-6,
        1.0e-5,
        1.0e-6,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf2,
    };

// These higher-speed passive identities deliberately own separate recipes.
// Their decade-valued settings currently match, but one physical scenario
// cannot silently change another's numerical contract.
inline constexpr IrwIntegrationRecipe
    kIrwStraightAar5V80PassiveIntegrationRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf5,
    };

inline constexpr IrwIntegrationRecipe
    kIrwR600Aar5V80PassiveIntegrationRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf5,
    };

inline constexpr IrwIntegrationRecipe
    kIrwR800Aar5V100PassiveIntegrationRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf5,
    };

inline constexpr IrwIntegrationRecipe
    kIrwStraightAar6V120PassiveIntegrationRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf5,
    };

inline constexpr IrwIntegrationRecipe
    kIrwR1000Aar6V120PassiveIntegrationRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf5,
    };

inline constexpr IrwIntegrationRecipe
    kIrwStraightAar6V160PassiveIntegrationRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf5,
    };

inline constexpr IrwIntegrationRecipe
    kIrwStraightErriLowV200PassiveIntegrationRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::SystemContinuousStateIntegrationRecipe::
            kCvodeBdf5,
    };

}  // namespace orvd::dynamics_qualification::internal
