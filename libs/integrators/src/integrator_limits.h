#pragma once

#include <cstddef>

namespace orvd::integrators::internal {

// G44's per-public-advance termination bound.  CVODE's own mxsteps applies to
// one CVode() call, so the system layer must also enforce this across its
// CV_ONE_STEP loop.
inline constexpr std::size_t kMaximumInternalStepsPerPublicAdvance =
    1'000'000;

}  // namespace orvd::integrators::internal
