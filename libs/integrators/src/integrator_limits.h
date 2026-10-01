#pragma once

#include "orvd/integrators/system_integration_configuration.h"

namespace orvd::integrators::internal {

// CVODE mxsteps bounds a single CVode call; the system also enforces the
// configured per-advance budget across its CV_ONE_STEP loop.
inline constexpr auto kMaximumInternalStepsPerPublicAdvance =
    kDefaultMaximumInternalStepsPerAdvance;

}  // namespace orvd::integrators::internal
