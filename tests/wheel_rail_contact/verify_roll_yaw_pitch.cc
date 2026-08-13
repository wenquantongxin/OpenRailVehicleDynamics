// The shared X-Z-Y attitude and angular-rate resolution, checked against
// independent constructions rather than remembered outputs.

#include <cmath>
#include <cstdio>
#include <string_view>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/wheel_rail_contact/roll_yaw_pitch.h"

namespace {

using orvd::wheel_rail_contact::ResolveRollYawPitch;
using orvd::wheel_rail_contact::ResolveRollYawPitchRates;
using orvd::wheel_rail_contact::RollYawPitchAngles;
using orvd::wheel_rail_contact::RollYawPitchRates;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "roll yaw pitch: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

Eigen::Matrix3d RollAbout(double angle) {
    return Eigen::Matrix3d(
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));
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
            Eigen::Matrix3d(
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
            Eigen::Matrix3d(
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
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
        // attitude. Zero roll or yaw would hide a swapped yaw/pitch rate and
        // the signs of the coupling terms.
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
                "the angular-rate resolution does not invert its independent "
                "construction");
    }

    if (failures == 0) {
        std::puts("roll-yaw-pitch resolution verified");
    }
    return failures == 0 ? 0 : 1;
}
