#pragma once

#include <Eigen/Core>

// The shared X-Z-Y attitude and angular-rate resolution used by vehicle/track
// kinematics. Keeping the convention in one module prevents independent
// consumers from drifting in order or sign.

namespace orvd::wheel_rail_contact {

// The roll-yaw-pitch triple of a rotation, resolved in that order about x, then
// z, then y. The yaw is returned in the closed quarter-turn range, which makes
// the resolution unique away from the singular configuration.
struct RollYawPitchAngles {
    double roll_radians{0.0};
    double yaw_radians{0.0};
    double pitch_radians{0.0};
};

[[nodiscard]] RollYawPitchAngles ResolveRollYawPitch(
    const Eigen::Matrix3d& rotation);

// Rates of the same X-Z-Y angles, from the moving body's angular velocity
// relative to the reference frame and expressed in that reference frame.
//
// The mapping is singular at a quarter turn of yaw. This function reports the
// direct formula; the vehicle/track layer that knows its operating geometry
// decides whether a non-finite result is admissible.
struct RollYawPitchRates {
    double roll_rate_radians_per_second{0.0};
    double yaw_rate_radians_per_second{0.0};
    double pitch_rate_radians_per_second{0.0};
};

[[nodiscard]] RollYawPitchRates ResolveRollYawPitchRates(
    const Eigen::Vector3d&
        relative_angular_velocity_in_reference_radians_per_second,
    double roll_radians, double yaw_radians);

}  // namespace orvd::wheel_rail_contact
