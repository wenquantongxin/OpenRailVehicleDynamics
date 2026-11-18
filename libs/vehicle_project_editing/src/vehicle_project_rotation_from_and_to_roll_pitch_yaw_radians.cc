#include "orvd/vehicle_project_editing/vehicle_project_rotation_from_and_to_roll_pitch_yaw_radians.h"

#include <cmath>
#include <numbers>

namespace orvd::vehicle_project_editing {
namespace {

Eigen::Matrix3d RotationAboutX(double angle_radians) {
    const double cosine = std::cos(angle_radians);
    const double sine = std::sin(angle_radians);
    Eigen::Matrix3d rotation;
    rotation << 1.0, 0.0, 0.0, 0.0, cosine, -sine, 0.0, sine, cosine;
    return rotation;
}

Eigen::Matrix3d RotationAboutY(double angle_radians) {
    const double cosine = std::cos(angle_radians);
    const double sine = std::sin(angle_radians);
    Eigen::Matrix3d rotation;
    rotation << cosine, 0.0, sine, 0.0, 1.0, 0.0, -sine, 0.0, cosine;
    return rotation;
}

Eigen::Matrix3d RotationAboutZ(double angle_radians) {
    const double cosine = std::cos(angle_radians);
    const double sine = std::sin(angle_radians);
    Eigen::Matrix3d rotation;
    rotation << cosine, -sine, 0.0, sine, cosine, 0.0, 0.0, 0.0, 1.0;
    return rotation;
}

// An angle as read, from one atan2 gives in [-pi, pi]: in (-pi, pi], -pi
// turned into pi, and 0 rather than -0, which would read as if turned.
double AsRead(double angle_radians) {
    return angle_radians == -std::numbers::pi ? std::numbers::pi : angle_radians + 0.0;
}

}  // namespace

Eigen::Matrix3d RotationFromRollPitchYawRadians(const Eigen::Vector3d& roll_pitch_yaw_radians) {
    return RotationAboutZ(roll_pitch_yaw_radians.z()) * RotationAboutY(roll_pitch_yaw_radians.y()) *
           RotationAboutX(roll_pitch_yaw_radians.x());
}

// With R = Rz(yaw) Ry(pitch) Rx(roll), R20 = -sin(pitch), the first column's
// other two entries are cos(pitch) times the cosine and sine of yaw, and the
// last row's other two cos(pitch) times the sine and cosine of roll. Where
// cos(pitch) is below one half, roll is taken instead from entries not scaled
// by it,
//     R01 - R12 = (1 + sin(pitch)) sin(roll - yaw),
//     R11 + R02 = (1 + sin(pitch)) cos(roll - yaw),
//     -(R01 + R12) = (1 - sin(pitch)) sin(roll + yaw),
//     R11 - R02 = (1 - sin(pitch)) cos(roll + yaw),
// the first two where pitch is positive and the last two where it is
// negative, each pair scaled by more than one, and turned by yaw.
Eigen::Vector3d RollPitchYawRadiansOfRotation(const Eigen::Matrix3d& rotation) {
    const double cosine_of_pitch = std::hypot(rotation(0, 0), rotation(1, 0));
    const double pitch = std::atan2(-rotation(2, 0), cosine_of_pitch);
    const double yaw = std::abs(pitch) == std::numbers::pi / 2.0
                           ? 0.0
                           : std::atan2(rotation(1, 0), rotation(0, 0));
    if (cosine_of_pitch >= 0.5) {
        return {AsRead(std::atan2(rotation(2, 1), rotation(2, 2))), AsRead(pitch), AsRead(yaw)};
    }
    const double cosine_of_yaw = std::cos(yaw);
    const double sine_of_yaw = std::sin(yaw);
    double roll = 0.0;
    if (pitch > 0.0) {
        const double scaled_sine_of_roll_minus_yaw = rotation(0, 1) - rotation(1, 2);
        const double scaled_cosine_of_roll_minus_yaw = rotation(1, 1) + rotation(0, 2);
        roll = std::atan2(sine_of_yaw * scaled_cosine_of_roll_minus_yaw +
                              cosine_of_yaw * scaled_sine_of_roll_minus_yaw,
                          cosine_of_yaw * scaled_cosine_of_roll_minus_yaw -
                              sine_of_yaw * scaled_sine_of_roll_minus_yaw);
    } else {
        const double scaled_sine_of_roll_plus_yaw = -(rotation(0, 1) + rotation(1, 2));
        const double scaled_cosine_of_roll_plus_yaw = rotation(1, 1) - rotation(0, 2);
        roll = std::atan2(cosine_of_yaw * scaled_sine_of_roll_plus_yaw -
                              sine_of_yaw * scaled_cosine_of_roll_plus_yaw,
                          cosine_of_yaw * scaled_cosine_of_roll_plus_yaw +
                              sine_of_yaw * scaled_sine_of_roll_plus_yaw);
    }
    return {AsRead(roll), AsRead(pitch), AsRead(yaw)};
}

}  // namespace orvd::vehicle_project_editing
