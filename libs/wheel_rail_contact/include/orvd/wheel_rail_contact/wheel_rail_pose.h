#pragma once

#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"
#include "orvd/wheel_rail_contact/rail_gauge_datum.h"

// Where one wheel sits relative to one rail, reduced to the four numbers the
// contact geometry needs.
//
// The contact between a wheel and a rail is solved in the plane across the
// track, and this is where the three-dimensional wheelset state becomes that
// plane. What comes out is a roll, a yaw and two offsets, all measured between
// the wheel's own profile datum and the rail's — not between body origins, and
// not in the track frame. The reduction is of the *pose*; nothing about the
// line or the vehicle's motion becomes two-dimensional by it.
//
// Two frames are involved and they are deliberately different. The offsets are
// resolved in the rail's own axes, which are the track axes rolled by the
// laying cant. The pair roll is measured against a third frame — the local
// tangential rail frame, which is the cant frame further yawed and pitched by
// the alignment and level irregularity slopes. A single frame for all three
// would be tidier and would misplace the wheel.
//
// The vertical offset comes out positive upward, against this project's
// downward vertical convention everywhere else. That is not an oversight: it is
// how far the wheel is raised above the rail, and a penetration is then a
// negative raise. The sign is stated in the field's name so nobody has to
// remember it.
//
// ## Irregularity is an input here, not an assumption
//
// The rail does not run exactly where the line says it runs, and the reference
// model carries the difference as four numbers per wheel station: a lateral and
// a vertical displacement of the rail, and their two longitudinal slopes. All
// four are inputs to this construction, and they are what `TrackIrregularity`
// below is. This project has no field to sample them from yet, so its callers
// pass zeros — but they pass them, and the construction they flow through is
// the general one, not a smooth-track shortcut.
//
// With all four zero the pair yaw becomes the wheelset yaw, the pair roll
// becomes the wheelset roll minus the rail's laying cant, and the nominal
// rolling radius loses its pitch shortening. That is a limit in exact real
// arithmetic, not a bit-level identity: the general path still forms a rotation
// product and takes an `atan2`, and at a general attitude the two can differ in
// the last place. Do not write a test that demands they agree bit-for-bit; do
// not replace the general path with the limit.
//
// What this module does NOT carry, and what a future irregularity wiring must
// add alongside it: the rail profile's own rigid transform (its origin and
// orientation at the sampled station, which the irregularity displaces and
// rotates), the effective station the sample is taken at, and the relative
// velocity of the wheel material point against the rail's. Those belong to the
// contact model's inputs, and the model must keep taking them rather than
// re-deriving them from the four scalars here.

namespace orvd::wheel_rail_contact {

// The fixed geometry one wheel/rail pair carries. Every one of these is either
// a property of the vehicle or derived from the rail asset; none is state.
struct WheelRailPoseConstants {
    // Where the rail profile's origin sits across and along the track's
    // vertical, and how the rail is rolled. The first and third come from the
    // gauge datum; the second is the resolved vertical offset the start-up
    // state carries.
    double rail_lateral_datum_meters{0.0};
    double rail_vertical_datum_meters{0.0};
    double rail_roll_radians{0.0};
    // The wheel profile's datum on the wheelset: how far out along the axle,
    // signed for the side, and how far down from the axle centre.
    double wheel_lateral_datum_meters{0.0};
    double nominal_rolling_radius_meters{0.0};
    // Whether a longitudinal grade in the vertical irregularity shortens the
    // nominal rolling radius by its cosine. Without it a pure grade manufactures
    // a spurious wheel/rail separation of order R0·(dz/ds)²/2. It is gated on
    // its own and not on whether the irregularity rates are used elsewhere.
    bool apply_pitch_correction{true};
};

// The wheelset's placement in the track frame at the station being evaluated.
// The roll and yaw are the first two of the roll-yaw-pitch resolution of the
// wheelset's attitude; the spin is not used and is not asked for.
struct WheelsetPlacement {
    double lateral_meters{0.0};
    double vertical_meters{0.0};
    double roll_radians{0.0};
    double yaw_radians{0.0};
};

// How far the rail departs from the line, at the station this wheel is over,
// and how fast that departure is changing as the wheel runs along.
//
// The two rates are per unit time, not per unit length: each is the spatial
// slope multiplied by the arc rate. They are stored that way because that is
// how they are consumed — as the numerator of a ratio whose denominator is the
// arc rate — and forming the ratio from two rates keeps the sign of travel in
// one place instead of two.
struct TrackIrregularity {
    double lateral_meters{0.0};
    double vertical_meters{0.0};
    double lateral_rate_meters_per_second{0.0};
    double vertical_rate_meters_per_second{0.0};
};

// Everything the reduction reads that changes from step to step.
struct WheelRailPoseInput {
    WheelsetPlacement placement;
    // The rate the carrier advances along the line. Its sign is the direction
    // of travel and it is load-bearing: see the note on reversal below.
    double arc_rate_meters_per_second{0.0};
    TrackIrregularity irregularity;
};

// The four numbers the contact geometry solves against.
struct ContactPoseScalars {
    double roll_radians{0.0};
    double yaw_radians{0.0};
    double lateral_offset_meters{0.0};
    // Positive upward: how far the wheel datum is raised above the rail datum.
    double vertical_raise_meters{0.0};
};

// The guarded slope ratio every angle in this module is formed through.
//
// Below the threshold on the denominator it returns exactly zero rather than
// letting `atan2` resolve a ratio that carries no information. Exposed because
// the station selection upstream must form its correction the same way, and a
// second copy of a threshold is a second thing to get wrong.
//
// The guard is on the denominator's magnitude only, deliberately. Running
// backwards — an arc rate below the negative threshold — with a numerator of
// exactly +0.0 gives `atan2(+0.0, negative) = +pi`, not zero. That is an IEEE
// fact, not an edge case in this code, and it is why a reversing caller cannot
// simply hold the irregularity at zero and expect the smooth-track limit.
[[nodiscard]] double SafeAtan2Ratio(double numerator, double denominator);

// Reduces the placement to the four scalars.
//
// `transport` is the profile/track roll correction from the layer above; under
// a suppressed policy it is three zeros and the reduction is unaffected, which
// is checked by comparison rather than asserted. It enters here and at the
// rail-side contact angle, and nowhere else.
//
// Throws nothing and allocates nothing.
[[nodiscard]] ContactPoseScalars BuildContactPoseScalars(
    const WheelRailPoseConstants& constants, const WheelRailPoseInput& input,
    const ProfileTrackRollTransport& transport);

}  // namespace orvd::wheel_rail_contact
