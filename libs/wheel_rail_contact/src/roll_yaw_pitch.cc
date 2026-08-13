#include "orvd/wheel_rail_contact/roll_yaw_pitch.h"

#include <cmath>

namespace orvd::wheel_rail_contact {

RollYawPitchAngles ResolveRollYawPitch(const Eigen::Matrix3d& rotation) {
    // Five entries of the nine carry the whole answer. The yaw's second
    // argument is a non-negative square root, which confines the result to the
    // quarter-turn range and makes the resolution single-valued.
    RollYawPitchAngles angles;
    angles.roll_radians = std::atan2(rotation(2, 1), rotation(1, 1));
    angles.yaw_radians = std::atan2(
        -rotation(0, 1), std::sqrt(rotation(0, 0) * rotation(0, 0) +
                                   rotation(0, 2) * rotation(0, 2)));
    angles.pitch_radians = std::atan2(rotation(0, 2), rotation(0, 0));
    return angles;
}

RollYawPitchRates ResolveRollYawPitchRates(
    const Eigen::Vector3d&
        relative_angular_velocity_in_reference_radians_per_second,
    double roll_radians, double yaw_radians) {
    const double sine_roll = std::sin(roll_radians);
    const double cosine_roll = std::cos(roll_radians);
    const double transverse =
        relative_angular_velocity_in_reference_radians_per_second.y() *
            cosine_roll +
        relative_angular_velocity_in_reference_radians_per_second.z() *
            sine_roll;

    RollYawPitchRates rates;
    rates.yaw_rate_radians_per_second =
        -relative_angular_velocity_in_reference_radians_per_second.y() *
            sine_roll +
        relative_angular_velocity_in_reference_radians_per_second.z() *
            cosine_roll;
    rates.pitch_rate_radians_per_second =
        transverse / std::cos(yaw_radians);
    rates.roll_rate_radians_per_second =
        relative_angular_velocity_in_reference_radians_per_second.x() +
        transverse * std::tan(yaw_radians);
    return rates;
}

}  // namespace orvd::wheel_rail_contact
