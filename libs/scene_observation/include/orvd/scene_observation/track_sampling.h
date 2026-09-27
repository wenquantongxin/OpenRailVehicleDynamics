#pragma once

/// @file
/// A sampled table of the line for display: centerline, track-frame
/// orientation, curvature, superelevation and the two rail datum lines.

#include <array>
#include <span>
#include <vector>

#include "orvd/track_geometry/track_geometry.h"

namespace orvd::scene_observation {

/// Where one rail's profile datum sits in the track frame: lateral to the
/// right positive, vertical downward positive. Both values come from the
/// contact plan's pose constants of that side.
struct RailDatumPlacement {
    double lateral_meters{0.0};
    double vertical_meters{0.0};
};

/// One row per requested station. Positions are in the track inertial frame;
/// the quaternion is w,x,y,z and maps track-frame coordinates to inertial
/// coordinates.
struct TrackSampleTable {
    std::vector<double> stations_meters;
    std::vector<std::array<double, 3>> centerline_in_inertial_meters;
    std::vector<std::array<double, 4>> rotation_inertial_from_track_wxyz;
    std::vector<double> curvature_radians_per_meter;
    std::vector<double> superelevation_meters;
    std::vector<std::array<double, 3>> left_rail_datum_in_inertial_meters;
    std::vector<std::array<double, 3>> right_rail_datum_in_inertial_meters;
};

/// Samples the line at the given stations. The stations must be non-empty and
/// finite; the line continues as a straight tangent beyond its definition
/// interval, so the caller chooses the displayed range explicitly.
void SampleTrackGeometry(const track_geometry::TrackGeometry& line,
                         std::span<const double> stations_meters,
                         RailDatumPlacement left_rail,
                         RailDatumPlacement right_rail,
                         TrackSampleTable& output);

}  // namespace orvd::scene_observation
