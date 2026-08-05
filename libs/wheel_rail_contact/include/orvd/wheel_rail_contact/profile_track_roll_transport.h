#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

// How a rail profile taken at one station is carried to a wheel that is being
// evaluated at another.
//
// A wheel's contact section is not taken at the station its carrier body sits
// at. On a curve the two differ by a few centimetres of arclength, and the
// track frame turns and lifts across that distance, so the section the wheel
// meets is rolled and displaced relative to the frame the rest of the pipeline
// is working in. This computes that difference and states it as three numbers:
// a roll angle and a lateral and vertical displacement.
//
// It is pure geometry. It takes two track-frame poses and a body position and
// returns three scalars; it has no notion of a line, a station, a profile or a
// wheel, and it makes no decision about whether the correction should be
// applied at all. That decision belongs to the layer that has the line and the
// trial state, and this project keeps it there rather than parking a flag here
// for someone else to read.
//
// The three numbers are consumed asymmetrically on purpose, and a consumer must
// not tidy that up: the two displacements are added to the lateral and vertical
// body coordinates, while the roll is subtracted from the roll attitude.
//
// A caution about the attitude extraction underneath. It resolves a rotation
// into roll about x, then yaw about z, then pitch about y, and that sequence
// has no unique answer when the yaw reaches a quarter turn. Nothing here guards
// against it, because reaching it would mean two track frames a few centimetres
// apart differed by a quarter turn of yaw, and a guard for that would be
// answering a question the geometry never asks. What it costs is worth stating
// plainly: exactly at that configuration the reported roll collapses to zero or
// jumps by half a turn depending on the sign of a zero, rather than degrading
// gradually.

namespace orvd::wheel_rail_contact {

// The correction, in the shared track frame's units. All three vanish when the
// two poses coincide and the stated body coordinates agree with the body
// position: exactly when the shared attitude is the identity, and to rounding
// otherwise, because rotating a vector out and back is not the identity in
// binary arithmetic. A consumer wanting to know whether the correction is
// switched off should ask the layer that decided, not compare these against
// zero.
struct ProfileTrackRollTransport {
    double roll_offset_radians{0.0};
    double lateral_offset_meters{0.0};
    double vertical_offset_meters{0.0};
};

// Computes the correction.
//
// `shared_*` describe the track frame at the station the pipeline is working
// in: the origin's position in the inertial frame, the rotation taking track
// components to inertial components, and the carrier body's position relative
// to that origin expressed in that track frame. That one position is the sole
// authority for all three body coordinates.
//
// `profile_*` describe the track frame at the station the profile section is
// taken at, in the same two parts.
//
// Throws nothing and allocates nothing.
[[nodiscard]] ProfileTrackRollTransport ComputeProfileTrackRollTransport(
    const Eigen::Vector3d& shared_track_origin_in_inertial_meters,
    const Eigen::Matrix3d& shared_rotation_inertial_from_track,
    const Eigen::Vector3d& shared_body_position_in_track_meters,
    const Eigen::Vector3d& profile_track_origin_in_inertial_meters,
    const Eigen::Matrix3d& profile_rotation_inertial_from_track);

// The roll-yaw-pitch triple of a rotation, resolved in that order about x, then
// z, then y. Exposed because the transport above is one consumer of it and the
// attitude of a body in track coordinates is another, and two copies of an
// Euler convention drift.
//
// The yaw is returned in the closed quarter-turn range, which is what makes the
// resolution unique away from the degenerate configuration described above.
struct RollYawPitchAngles {
    double roll_radians{0.0};
    double yaw_radians{0.0};
    double pitch_radians{0.0};
};

[[nodiscard]] RollYawPitchAngles ResolveRollYawPitch(
    const Eigen::Matrix3d& rotation);

// Whether a vehicle type carries the correction at all.
//
// Two reference vehicles were qualified with two different answers, and
// reproducing either one means reproducing its answer. This is therefore a
// property of the vehicle, chosen once when its contact model is built and
// fixed for that model's life — not a runtime switch, and not a boolean a
// caller may flip between evaluations. When one answer is qualified for both
// vehicles, the losing one is deleted together with this enumeration and with
// the selection it exists to express.
enum class ProfileTrackRollTransportPolicy {
    kSuppressed,
    kApplied,
};

// The policy bound to the mathematics.
//
// The suppressed branch returns three exact zeros rather than a correction
// nobody applies, so a consumer reads one shape of answer under both policies
// and keeps no branch of its own. The two are named states of one object rather
// than two call sites, because a caller that had to remember which to call
// would eventually not.
class ProfileTrackRollTransportStrategy {
   public:
    explicit ProfileTrackRollTransportStrategy(
        ProfileTrackRollTransportPolicy policy)
        : policy_(policy) {}

    [[nodiscard]] ProfileTrackRollTransportPolicy policy() const {
        return policy_;
    }

    // Under the suppressed policy this ignores its arguments and returns zeros;
    // under the applied one it is `ComputeProfileTrackRollTransport`.
    [[nodiscard]] ProfileTrackRollTransport Compute(
        const Eigen::Vector3d& shared_track_origin_in_inertial_meters,
        const Eigen::Matrix3d& shared_rotation_inertial_from_track,
        const Eigen::Vector3d& shared_body_position_in_track_meters,
        const Eigen::Vector3d& profile_track_origin_in_inertial_meters,
        const Eigen::Matrix3d& profile_rotation_inertial_from_track) const;

   private:
    ProfileTrackRollTransportPolicy policy_{
        ProfileTrackRollTransportPolicy::kSuppressed};
};

}  // namespace orvd::wheel_rail_contact
