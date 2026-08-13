#pragma once

namespace orvd::dynamics_qualification::internal {

// Closed numerical identities for the three IRW qualification scenarios.
// These recipes are private to the uninstalled qualification tools; they are
// not an integrator policy API.
struct IrwIntegrationRecipe final {
    double relative_tolerance{};
    double position_absolute_tolerance{};
    double velocity_absolute_tolerance{};
    double series_force_absolute_tolerance_newtons{};
};

inline constexpr IrwIntegrationRecipe
    kIrwNoIrregularityPassiveIntegrationRecipe{
        1.0e-6,
        1.0e-6,
        1.0e-5,
        1.0e-6,
    };

inline constexpr IrwIntegrationRecipe kIrwR300Aar5PassiveIntegrationRecipe{
    1.0e-7,
    1.0e-7,
    1.0e-6,
    1.0e-5,
};

inline constexpr IrwIntegrationRecipe kIrwP179ControlledIntegrationRecipe{
    1.0e-6,
    1.0e-6,
    1.0e-5,
    1.0e-6,
};

}  // namespace orvd::dynamics_qualification::internal
