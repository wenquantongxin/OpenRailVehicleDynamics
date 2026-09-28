#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace orvd::integrators::internal {

// Finite spacing on either side, including zero and the largest finite value.
inline double CoordinateTimeUlp(double time) {
    if (!std::isfinite(time)) return std::numeric_limits<double>::infinity();
    double spacing = 0.0;
    for (const double direction : {-std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::infinity()}) {
        const double neighbor = std::nextafter(time, direction);
        if (std::isfinite(neighbor)) spacing = std::max(spacing, std::abs(neighbor - time));
    }
    return spacing;
}

// The formula interval h remains authoritative for method coefficients/history.
// An explicit endpoint may only remove clock-rounding differences, never turn
// h into a variable-step coefficient or manufacture representable progress.
inline bool CoordinateEndpointTimeIsCompatible(double start, double h, double end) {
    if (!std::isfinite(start) || !std::isfinite(h) || h <= 0.0 ||
        !std::isfinite(end) || !(end > start)) return false;
    const double arithmetic_end = start + h;
    if (!std::isfinite(arithmetic_end) || !(arithmetic_end > start)) return false;
    const double spacing = std::max({CoordinateTimeUlp(start),
                                     CoordinateTimeUlp(arithmetic_end),
                                     CoordinateTimeUlp(end)});
    return std::abs(end - arithmetic_end) <= std::min(h / 16.0, 16.0 * spacing);
}

inline bool CoordinateStopCanUseNominalStep(double start, double nominal_h,
                                           double grid_end, double stop) {
    if (!std::isfinite(grid_end) || !(grid_end > start)) return false;
    const double spacing = std::max({CoordinateTimeUlp(start),
                                     CoordinateTimeUlp(grid_end),
                                     CoordinateTimeUlp(stop)});
    const double window = std::min(nominal_h / 16.0, 8.0 * spacing);
    return std::abs(stop - grid_end) <= window &&
           CoordinateEndpointTimeIsCompatible(start, nominal_h, stop);
}

}  // namespace orvd::integrators::internal
