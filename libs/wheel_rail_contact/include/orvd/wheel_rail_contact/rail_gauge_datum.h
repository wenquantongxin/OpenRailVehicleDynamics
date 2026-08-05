#pragma once

#include "orvd/wheel_rail_contact/profile_points.h"

// Where a rail sits across the track, derived from the rail's own shape.
//
// A track gauge is measured between the two gauge faces at a stated depth below
// the rail crowns, not between the crowns themselves. So the lateral position
// of a rail's profile origin is not the half-gauge: it is the half-gauge plus
// the distance from the gauge face to that origin, and that distance is a
// property of the profile, read off the profile, at the cant the rail is laid
// at.
//
// This is the reason the datum is computed here instead of being carried as a
// number. The profiles this project ships are loaded at runtime and may be
// replaced; a frozen datum would then describe the previous rail. Changing the
// gauge, the measuring depth, the cant or the rail profile must all move it,
// and each of those is an input a researcher is entitled to change.
//
// The construction is deliberately piecewise linear on the profile polyline
// rather than on any interpolant built over it. The gauge face is a
// measurement convention applied to measured points, and interpolating it would
// make the answer depend on which interpolant happened to be selected.

namespace orvd::wheel_rail_contact {

struct RailGaugeDatum {
    // The distance from the gauge face to the rail profile's own origin, always
    // positive. It is the amount by which the rail sits further out than the
    // half-gauge.
    double gauge_face_offset_meters{0.0};
    // The lateral position of the rail profile's origin in the track frame,
    // signed: positive for the right rail, negative for the left.
    double lateral_datum_meters{0.0};
    // The rail's roll about the track's forward axis, signed for the side. The
    // right rail leans toward the track centre under a positive cant, which is
    // a negative roll about the forward axis with the lateral axis pointing
    // right and the vertical axis pointing down.
    double roll_radians{0.0};
};

// Derives the datum for one rail of a pair.
//
// `rail_cant_radians` is the magnitude of the laying cant; the sign each rail
// carries is derived here so that the two rails cannot be given inconsistent
// signs by two different callers.
//
// Throws std::invalid_argument when the profile is not a rail profile, when the
// gauge or the measuring depth is not finite and positive, or when the profile
// never reaches the measuring depth — a profile too shallow to have a gauge
// face cannot be laid to a gauge.
[[nodiscard]] RailGaugeDatum ComputeRailGaugeDatum(
    const ProfilePoints& rail_profile, double track_gauge_meters,
    double gauge_measuring_depth_meters, double rail_cant_radians,
    WheelSide side);

}  // namespace orvd::wheel_rail_contact
