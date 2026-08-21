#pragma once

#include "bdf_integration_access.h"

namespace orvd::dynamics_qualification::internal {

// Closed numerical identities for the IRW qualification scenarios.
// These recipes are private to the uninstalled qualification tools; they are
// not an integrator policy API.
struct IrwBdfToleranceRecipe final {
    double relative_tolerance{};
    double position_absolute_tolerance{};
    double velocity_absolute_tolerance{};
    double series_force_absolute_tolerance_newtons{};
    integrators::internal::MaximumBdfOrder maximum_bdf_order{};
};

inline constexpr IrwBdfToleranceRecipe
    kIrwR300NoIrregularityV60PassiveBdfToleranceRecipe{
        1.0e-6,
        1.0e-6,
        1.0e-5,
        1.0e-6,
        integrators::internal::MaximumBdfOrder::kSecond,
    };

inline constexpr IrwBdfToleranceRecipe kIrwR300Aar5V60PassiveBdfToleranceRecipe{
    1.0e-8,
    1.0e-8,
    1.0e-7,
    1.0e-6,
    integrators::internal::MaximumBdfOrder::kFifth,
};

inline constexpr IrwBdfToleranceRecipe
    kIrwR300Aar5V60At100HzFullStateGuidanceBdfToleranceRecipe{
        1.0e-6,
        1.0e-6,
        1.0e-5,
        1.0e-6,
        integrators::internal::MaximumBdfOrder::kSecond,
    };

// These four passive identities deliberately own separate recipes. Their
// decade-valued settings currently match, but one physical scenario cannot
// silently change another's numerical contract.
inline constexpr IrwBdfToleranceRecipe
    kIrwStraightAar5V80PassiveBdfToleranceRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::MaximumBdfOrder::kFifth,
    };

inline constexpr IrwBdfToleranceRecipe
    kIrwR600Aar5V80PassiveBdfToleranceRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::MaximumBdfOrder::kFifth,
    };

inline constexpr IrwBdfToleranceRecipe
    kIrwStraightAar6V120PassiveBdfToleranceRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::MaximumBdfOrder::kFifth,
    };

inline constexpr IrwBdfToleranceRecipe
    kIrwR1000Aar6V120PassiveBdfToleranceRecipe{
        1.0e-9,
        1.0e-9,
        1.0e-8,
        1.0e-7,
        integrators::internal::MaximumBdfOrder::kFifth,
    };

}  // namespace orvd::dynamics_qualification::internal
