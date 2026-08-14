#include "track_profile_quintic.h"

#include <cstddef>

namespace orvd::track_geometry::internal {
namespace {

// Coefficients in the normalised window coordinate.  Rows correspond to value,
// first derivative and second derivative at the start, then at the end.
constexpr double kQuinticBasis[6][6] = {
    {1.0, 0.0, 0.0, -10.0, 15.0, -6.0},
    {0.0, 1.0, 0.0, -6.0, 8.0, -3.0},
    {0.0, 0.0, 0.5, -1.5, 1.5, -0.5},
    {0.0, 0.0, 0.0, 10.0, -15.0, 6.0},
    {0.0, 0.0, 0.0, -4.0, 7.0, -3.0},
    {0.0, 0.0, 0.0, 0.5, -1.0, 0.5},
};

}  // namespace

std::array<double, 6> BuildQuinticHermiteCoefficients(
    double window_length_meters, const ProfileValueDerivatives& start,
    const ProfileValueDerivatives& end) {
    const double width_squared =
        window_length_meters * window_length_meters;
    const double boundary[6] = {
        start.value,
        window_length_meters * start.first_derivative_per_meter,
        width_squared * start.second_derivative_per_meter_squared,
        end.value,
        window_length_meters * end.first_derivative_per_meter,
        width_squared * end.second_derivative_per_meter_squared,
    };

    std::array<double, 6> normalised{};
    for (std::size_t row = 0; row < normalised.size(); ++row) {
        for (std::size_t order = 0; order < normalised.size(); ++order) {
            normalised[order] += boundary[row] * kQuinticBasis[row][order];
        }
    }

    std::array<double, 6> coefficients{};
    double scale = 1.0;
    for (std::size_t order = 0; order < coefficients.size(); ++order) {
        coefficients[order] = normalised[order] / scale;
        scale *= window_length_meters;
    }
    return coefficients;
}

}  // namespace orvd::track_geometry::internal
