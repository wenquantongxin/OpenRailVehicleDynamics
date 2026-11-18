#include "orvd/vehicle_project_editing/vehicle_project_rotation_from_and_to_roll_pitch_yaw_radians.h"

#include <type_traits>
#include <utility>

static_assert(
    std::is_same_v<decltype(orvd::vehicle_project_editing::RotationFromRollPitchYawRadians(
                       std::declval<const Eigen::Vector3d&>())),
                   Eigen::Matrix3d>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::RollPitchYawRadiansOfRotation(
                                 std::declval<const Eigen::Matrix3d&>())),
                             Eigen::Vector3d>);
