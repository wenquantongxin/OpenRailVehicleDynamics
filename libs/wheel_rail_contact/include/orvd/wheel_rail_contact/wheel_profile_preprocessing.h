#pragma once

#include <cstddef>
#include <vector>

#include "orvd/wheel_rail_contact/profile_points.h"

// Turns an authored wheel profile into the outline the contact geometry
// samples. A zero step preserves the authored nodes. A positive step replaces
// them with nodes laid at constant arc length along a first natural cubic
// spline, then builds a second natural cubic spline through the replacement
// nodes.
//
// Where the profile turns sharply the equal-arc nodes crowd together, so the
// outline resolves the flange root at the same curve-length density as the
// tread instead of at the density the asset author happened to write. The
// rescan always runs on the physical right-hand ordering and the mirror is
// applied to its result; running it on an already-mirrored list would move the
// remainder interval, and therefore the phase of the whole grid, to the other
// end of the left profile.

namespace orvd::wheel_rail_contact {

struct WheelProfilePreprocessingConfiguration {
    // A positive step selects the equal arc-length rescan; zero preserves the
    // authored nodes.
    double equal_arc_length_rescan_step_meters{0.0};
};

// The outline the contact geometry consumes: a fixed number of stations
// spanning the side-resolved profile, with the surface height and its slope at
// each.
struct WheelProfileOutline {
    std::vector<double> station_meters;
    std::vector<double> height_meters;
    std::vector<double> height_slope;
};

// The replacement nodes laid down by an equal-arc rescan. Empty when the
// authored nodes are preserved.
struct WheelProfileControlNodes {
    std::vector<double> lateral_meters;
    std::vector<double> vertical_meters;
    // The curve length the rescan measured, in metres. Zero when the authored
    // nodes are preserved.
    double total_arc_length_meters{0.0};
};

class WheelProfilePreprocessing {
   public:
    // Throws std::invalid_argument when the step is not finite or is negative.
    explicit WheelProfilePreprocessing(
        WheelProfilePreprocessingConfiguration configuration);

    [[nodiscard]] double step_meters() const { return step_meters_; }

    // Lays the control nodes this preparation calls for.
    //
    // Throws std::invalid_argument when the profile is not a wheel profile, and
    // std::runtime_error when the authored points do not admit the spline the
    // rescan needs.
    [[nodiscard]] WheelProfileControlNodes LayControlNodes(
        const ProfilePoints& authored, WheelSide side) const;

    // Samples the visible outline at `sample_count` stations spanning the
    // side-resolved profile.
    //
    // Throws std::invalid_argument when the profile is not a wheel profile or
    // when fewer than two stations are asked for.
    [[nodiscard]] WheelProfileOutline SampleVisibleOutline(
        const ProfilePoints& authored, WheelSide side,
        std::size_t sample_count) const;

   private:
    double step_meters_{0.0};
};

}  // namespace orvd::wheel_rail_contact
