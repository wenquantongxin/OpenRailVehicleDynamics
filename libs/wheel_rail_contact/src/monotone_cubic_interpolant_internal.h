#pragma once

#include <span>

namespace orvd::wheel_rail_contact::internal {

// Applies the same shape-preserving slope rule as the public checked entry
// point to storage whose construction already proves all preconditions.
//
// The knots and values have equal length of at least two, the knots are finite
// and strictly increasing, the values are finite, `slopes_out` has one entry
// per knot, and both scratch spans have one entry per interval. This function
// is deliberately confined to `src/`: configuration and asset input must keep
// using the checked public API.
void ComputeShapePreservingNodalSlopesForTrustedNodes(
    std::span<const double> knots, std::span<const double> values,
    std::span<double> slopes_out, std::span<double> spacings,
    std::span<double> secants);

}  // namespace orvd::wheel_rail_contact::internal
