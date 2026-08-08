// The two-channel station field, checked with distinct non-uniform affine
// series so the four public quantities have independent analytic answers.

#include <array>
#include <cmath>
#include <cstdio>
#include <string_view>

#include "orvd/wheel_rail_contact/track_irregularity_field.h"

namespace {

using orvd::wheel_rail_contact::TrackIrregularityField;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "track irregularity field: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireNear(double actual, double expected, double tolerance,
                 std::string_view what) {
    Require(std::abs(actual - expected) <= tolerance, what);
}

}  // namespace

int main() {
    // The grids, sample counts and domains are intentionally different.  The
    // affine fields are reproduced exactly by a natural cubic apart from
    // floating-point roundoff, giving independent expected values and slopes.
    constexpr std::array<double, 4> lateral_stations{-2.0, -0.25, 1.5, 4.0};
    constexpr std::array<double, 4> lateral_displacements{
        -0.005, -0.0015, 0.002, 0.007};  // 0.002*s - 0.001.
    constexpr std::array<double, 5> vertical_stations{-1.0, 0.25, 2.0, 3.5,
                                                       7.0};
    constexpr std::array<double, 5> vertical_displacements{
        0.013, 0.00925, 0.004, -0.0005, -0.011};  // 0.01 - 0.003*s.

    const TrackIrregularityField field(
        lateral_stations, lateral_displacements, vertical_stations,
        vertical_displacements);

    for (const double station : std::array<double, 5>{-0.75, 0.0, 1.0, 2.75,
                                                       3.75}) {
        RequireNear(field.LateralDisplacementMeters(station),
                    0.002 * station - 0.001, 2.0e-17,
                    "the lateral displacement does not follow its series");
        RequireNear(field.VerticalDisplacementMeters(station),
                    0.01 - 0.003 * station, 2.0e-17,
                    "the vertical displacement does not follow its series");
        RequireNear(field.LateralSlopeMetersPerMeter(station), 0.002, 2.0e-17,
                    "the lateral slope does not follow its series");
        RequireNear(field.VerticalSlopeMetersPerMeter(station), -0.003,
                    2.0e-17,
                    "the vertical slope does not follow its series");
    }

    // At -1.5 m the vertical channel is outside its shorter domain while the
    // lateral channel is still live.  At 6 m the converse holds.  This proves
    // that a channel is evaluated on its own grid rather than on a shared one.
    RequireNear(field.LateralDisplacementMeters(-1.5), -0.004, 2.0e-17,
                "the lateral channel did not remain live below the vertical "
                "domain");
    Require(field.VerticalDisplacementMeters(-1.5) ==
                    vertical_displacements.front() &&
                field.VerticalSlopeMetersPerMeter(-1.5) == 0.0,
            "the vertical channel was not held flat below its domain");
    Require(field.LateralDisplacementMeters(6.0) ==
                    lateral_displacements.back() &&
                field.LateralSlopeMetersPerMeter(6.0) == 0.0,
            "the lateral channel was not held flat above its domain");
    RequireNear(field.VerticalDisplacementMeters(6.0), -0.008, 2.0e-17,
                "the vertical channel did not remain live above the lateral "
                "domain");

    return failures == 0 ? 0 : 1;
}
