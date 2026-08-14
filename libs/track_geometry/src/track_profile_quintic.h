#pragma once

#include <array>

namespace orvd::track_geometry::internal {

struct ProfileValueDerivatives {
    double value{0.0};
    double first_derivative_per_meter{0.0};
    double second_derivative_per_meter_squared{0.0};
};

// Returns ascending coefficients in the physical local coordinate on a window
// of the stated width.  This is the single quintic boundary kernel shared by
// scalar profiles and the dedicated vertical profile.
[[nodiscard]] std::array<double, 6> BuildQuinticHermiteCoefficients(
    double window_length_meters, const ProfileValueDerivatives& start,
    const ProfileValueDerivatives& end);

}  // namespace orvd::track_geometry::internal
