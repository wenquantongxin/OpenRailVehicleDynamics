// The profile/track roll transport, checked against the closed forms it must
// satisfy rather than against remembered outputs.

#include <cmath>
#include <cstdio>
#include <string_view>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"

namespace {

using orvd::wheel_rail_contact::ComputeProfileTrackRollTransport;
using orvd::wheel_rail_contact::ProfileTrackRollTransport;
using orvd::wheel_rail_contact::ProfileTrackRollTransportPolicy;
using orvd::wheel_rail_contact::ProfileTrackRollTransportStrategy;
using orvd::wheel_rail_contact::ResolveRollYawPitch;
using orvd::wheel_rail_contact::ResolveRollYawPitchRates;
using orvd::wheel_rail_contact::RollYawPitchAngles;
using orvd::wheel_rail_contact::RollYawPitchRates;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "profile track roll transport: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

Eigen::Matrix3d RollAbout(double angle) {
    return Eigen::Matrix3d(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));
}

Eigen::Matrix3d ArbitraryAttitude(double roll, double yaw, double pitch) {
    return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

}  // namespace

int main() {
    {
        // The resolution must invert its own composition. Building a rotation
        // from a roll, then a yaw, then a pitch and resolving it must return
        // the three angles that built it.
        const double roll = 0.31;
        const double yaw = -0.22;
        const double pitch = 0.17;
        const Eigen::Matrix3d rotation =
            RollAbout(roll) *
            Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
            Eigen::Matrix3d(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
        const RollYawPitchAngles resolved = ResolveRollYawPitch(rotation);
        Require(std::abs(resolved.roll_radians - roll) < 1.0e-14 &&
                    std::abs(resolved.yaw_radians - yaw) < 1.0e-14 &&
                    std::abs(resolved.pitch_radians - pitch) < 1.0e-14,
                "the attitude resolution does not invert its own composition");
        Require(std::abs(ResolveRollYawPitch(Eigen::Matrix3d::Identity())
                             .roll_radians) == 0.0,
                "the identity rotation does not resolve to no roll");
    }

    {
        // Independently invert the X-Z-Y rate map at a non-degenerate
        // attitude.  Zero roll or yaw would hide a swapped yaw/pitch rate and
        // the sign of the coupling terms that G53 uses to strip accumulated
        // wheel spin from contact geometry.
        const double roll = 0.23;
        const double yaw = -0.19;
        const double roll_rate = 0.41;
        const double yaw_rate = -0.37;
        const double pitch_rate = 17.3;
        const double temporary = pitch_rate * std::cos(yaw);
        const Eigen::Vector3d relative_angular_velocity(
            roll_rate - pitch_rate * std::sin(yaw),
            temporary * std::cos(roll) - yaw_rate * std::sin(roll),
            temporary * std::sin(roll) + yaw_rate * std::cos(roll));
        const RollYawPitchRates resolved = ResolveRollYawPitchRates(
            relative_angular_velocity, roll, yaw);
        Require(std::abs(resolved.roll_rate_radians_per_second - roll_rate) <
                        1.0e-14 &&
                    std::abs(resolved.yaw_rate_radians_per_second - yaw_rate) <
                        1.0e-14 &&
                    std::abs(resolved.pitch_rate_radians_per_second -
                             pitch_rate) < 1.0e-14,
                "the X-Z-Y angular-rate resolution does not invert its "
                "independent construction");
    }

    {
        // The disabled case: the same station on both sides. With an identity
        // attitude nothing is rotated and the three offsets are exactly zero.
        const Eigen::Vector3d origin(12.0, -4.0, 3.0);
        const Eigen::Vector3d body(0.7, 0.004, -0.6);
        const ProfileTrackRollTransport upright = ComputeProfileTrackRollTransport(
            origin, Eigen::Matrix3d::Identity(), body, origin,
            Eigen::Matrix3d::Identity());
        Require(upright.roll_offset_radians == 0.0 &&
                    upright.lateral_offset_meters == 0.0 &&
                    upright.vertical_offset_meters == 0.0,
                "coincident poses with no attitude did not produce zero offsets");

        // With a general attitude the same station still produces no
        // correction, but only to rounding: carrying the point out to the
        // inertial frame and back is not the identity in binary arithmetic.
        const Eigen::Matrix3d attitude = ArbitraryAttitude(0.13, 0.4, -0.09);
        const ProfileTrackRollTransport tilted = ComputeProfileTrackRollTransport(
            origin, attitude, body, origin, attitude);
        Require(std::abs(tilted.roll_offset_radians) < 1.0e-15 &&
                    std::abs(tilted.lateral_offset_meters) < 1.0e-15 &&
                    std::abs(tilted.vertical_offset_meters) < 1.0e-15,
                "coincident poses produced a correction larger than rounding");

        // The same cancellation check at a realistic long-line coordinate.
        // Forming two absolute body positions first loses the micrometre-scale
        // correction even though the two origins are exactly the same.
        const Eigen::Vector3d distant_origin(10000.0, -10000.0, 10000.0);
        const ProfileTrackRollTransport distant =
            ComputeProfileTrackRollTransport(distant_origin, attitude, body,
                                             distant_origin, attitude);
        Require(std::abs(distant.roll_offset_radians) < 1.0e-15 &&
                    std::abs(distant.lateral_offset_meters) < 1.0e-15 &&
                    std::abs(distant.vertical_offset_meters) < 1.0e-15,
                "coincident poses at a long-line coordinate lost the local "
                "transport correction to cancellation");
    }

    {
        // A pure relative roll must be reported exactly, with the sign that
        // says the profile frame is rolled by that much relative to the shared
        // one.
        const Eigen::Vector3d origin(0.0, 0.0, 0.0);
        const Eigen::Matrix3d shared = ArbitraryAttitude(0.05, 0.21, -0.13);
        const double relative_roll = 0.037;
        const Eigen::Matrix3d profile = shared * RollAbout(relative_roll);
        const Eigen::Vector3d body(0.0, 0.004, -0.6);
        const ProfileTrackRollTransport transport = ComputeProfileTrackRollTransport(
            origin, shared, body, origin, profile);
        Require(std::abs(transport.roll_offset_radians - relative_roll) < 1.0e-14,
                "a pure relative roll was not reported as itself");
    }

    {
        // A pure translation of the profile frame must produce no roll and
        // displacements that are the translation seen in that frame.
        const Eigen::Vector3d origin(5.0, 1.0, -2.0);
        const Eigen::Matrix3d attitude = ArbitraryAttitude(0.0, 0.0, 0.0);
        const Eigen::Vector3d body(0.0, 0.004, -0.6);
        const Eigen::Vector3d shift(0.0, 0.011, -0.007);
        const ProfileTrackRollTransport transport = ComputeProfileTrackRollTransport(
            origin, attitude, body, origin + shift, attitude);
        Require(transport.roll_offset_radians == 0.0,
                "a pure translation produced a roll");
        Require(std::abs(transport.lateral_offset_meters + shift.y()) < 1.0e-15 &&
                    std::abs(transport.vertical_offset_meters + shift.z()) < 1.0e-15,
                "a pure translation was not reported as the displacement it is");
    }

    {
        // The answer is a relative quantity, so moving both track poses through
        // the same rigid transform must not change it. This is the property
        // that catches a frame used in the wrong direction: a transpose the
        // wrong way round is invisible on an identity attitude and shows up
        // here.
        const Eigen::Vector3d shared_origin(10.0, -3.0, 2.0);
        const Eigen::Matrix3d shared_attitude = ArbitraryAttitude(-0.25, 0.4, 0.15);
        const Eigen::Vector3d profile_origin(10.25, -2.95, 2.03);
        const Eigen::Matrix3d profile_attitude = ArbitraryAttitude(-0.19, 0.43, 0.12);
        const Eigen::Vector3d body(0.0, 0.004, -0.6);

        const ProfileTrackRollTransport direct = ComputeProfileTrackRollTransport(
            shared_origin, shared_attitude, body, profile_origin,
            profile_attitude);
        Require(std::abs(direct.roll_offset_radians) > 1.0e-4 &&
                    std::abs(direct.lateral_offset_meters) > 1.0e-4 &&
                    std::abs(direct.vertical_offset_meters) > 1.0e-4,
                "the fixture is degenerate: it produces no correction to be "
                "invariant about");

        const Eigen::Matrix3d global_rotation = ArbitraryAttitude(0.51, -0.77, 0.0);
        const Eigen::Vector3d global_translation(-120.0, 33.0, 7.5);
        const ProfileTrackRollTransport moved = ComputeProfileTrackRollTransport(
            global_translation + global_rotation * shared_origin,
            global_rotation * shared_attitude, body,
            global_translation + global_rotation * profile_origin,
            global_rotation * profile_attitude);
        Require(std::abs(moved.roll_offset_radians -
                         direct.roll_offset_radians) < 1.0e-14 &&
                    std::abs(moved.lateral_offset_meters -
                             direct.lateral_offset_meters) < 1.0e-14 &&
                    std::abs(moved.vertical_offset_meters -
                             direct.vertical_offset_meters) < 1.0e-14,
                "the correction changed when both track poses were moved "
                "together");
    }

    {
        // The longitudinal component of the body position is used. Dropping it
        // is a plausible simplification and it is wrong: the two track frames
        // differ along the track as well as across it, so the point would be
        // lifted at the wrong place.
        const Eigen::Vector3d shared_origin(0.0, 0.0, 0.0);
        const Eigen::Matrix3d shared_attitude = Eigen::Matrix3d::Identity();
        const Eigen::Vector3d profile_origin(0.0, 0.0, 0.0);
        const Eigen::Matrix3d profile_attitude = ArbitraryAttitude(0.0, 0.08, 0.0);
        const Eigen::Vector3d with_reach(0.9, 0.004, -0.6);
        const Eigen::Vector3d without_reach(0.0, 0.004, -0.6);
        const ProfileTrackRollTransport reaching = ComputeProfileTrackRollTransport(
            shared_origin, shared_attitude, with_reach, profile_origin,
            profile_attitude);
        const ProfileTrackRollTransport flat = ComputeProfileTrackRollTransport(
            shared_origin, shared_attitude, without_reach, profile_origin,
            profile_attitude);
        Require(std::abs(reaching.lateral_offset_meters -
                         flat.lateral_offset_meters) > 1.0e-3,
                "the longitudinal component of the body position does not reach "
                "the answer");
    }

    {
        // The two named policies, each on the same non-degenerate geometry.
        // Both branches have to be reachable and both have to be distinguishable
        // from each other, or the selection layer proves nothing.
        const Eigen::Vector3d shared_origin(10.0, -3.0, 2.0);
        const Eigen::Matrix3d shared_attitude = ArbitraryAttitude(-0.25, 0.4, 0.15);
        const Eigen::Vector3d profile_origin(10.25, -2.95, 2.03);
        const Eigen::Matrix3d profile_attitude = ArbitraryAttitude(-0.19, 0.43, 0.12);
        const Eigen::Vector3d body(0.0, 0.004, -0.6);

        const ProfileTrackRollTransportStrategy applying{
            ProfileTrackRollTransportPolicy::kApplied};
        const ProfileTrackRollTransportStrategy suppressing{
            ProfileTrackRollTransportPolicy::kSuppressed};
        Require(applying.policy() == ProfileTrackRollTransportPolicy::kApplied &&
                    suppressing.policy() ==
                        ProfileTrackRollTransportPolicy::kSuppressed,
                "a strategy did not keep the policy it was built with");

        const ProfileTrackRollTransport applied =
            applying.Compute(shared_origin, shared_attitude, body, profile_origin,
                             profile_attitude);
        const ProfileTrackRollTransport bare = ComputeProfileTrackRollTransport(
            shared_origin, shared_attitude, body, profile_origin,
            profile_attitude);
        Require(applied.roll_offset_radians == bare.roll_offset_radians &&
                    applied.lateral_offset_meters == bare.lateral_offset_meters &&
                    applied.vertical_offset_meters == bare.vertical_offset_meters,
                "the applying policy is not the mathematics it selects");

        const ProfileTrackRollTransport suppressed =
            suppressing.Compute(shared_origin, shared_attitude, body,
                                profile_origin, profile_attitude);
        Require(suppressed.roll_offset_radians == 0.0 &&
                    suppressed.lateral_offset_meters == 0.0 &&
                    suppressed.vertical_offset_meters == 0.0,
                "the suppressing policy produced a correction");
        Require(std::abs(applied.roll_offset_radians) > 1.0e-4,
                "the fixture cannot tell the two policies apart");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("profile track roll transport verified");
    return 0;
}
