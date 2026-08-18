#pragma once

#include "bdf_integration_access.h"

namespace orvd::dynamics_qualification::internal {

// Closed numerical identities for the three IRW qualification scenarios.
// These recipes are private to the uninstalled qualification tools; they are
// not an integrator policy API.
struct IrwIntegrationRecipe final {
    double relative_tolerance{};
    double position_absolute_tolerance{};
    double velocity_absolute_tolerance{};
    double series_force_absolute_tolerance_newtons{};
    integrators::internal::MaximumBdfOrder maximum_bdf_order{};
};

inline constexpr IrwIntegrationRecipe
    kIrwNoIrregularityPassiveIntegrationRecipe{
        1.0e-6,
        1.0e-6,
        1.0e-5,
        1.0e-6,
        integrators::internal::MaximumBdfOrder::kSecond,
    };

inline constexpr IrwIntegrationRecipe kIrwR300Aar5PassiveIntegrationRecipe{
    1.0e-8,
    1.0e-8,
    1.0e-7,
    1.0e-6,
    integrators::internal::MaximumBdfOrder::kFifth,
};

inline constexpr IrwIntegrationRecipe kIrwP179ControlledIntegrationRecipe{
    1.0e-6,
    1.0e-6,
    1.0e-5,
    1.0e-6,
    integrators::internal::MaximumBdfOrder::kSecond,
};

}  // namespace orvd::dynamics_qualification::internal
