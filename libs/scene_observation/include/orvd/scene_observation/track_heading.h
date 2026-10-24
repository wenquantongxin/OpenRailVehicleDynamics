#pragma once

#include <Eigen/Core>

namespace orvd::scene_observation {

/// Heading of a forward direction projected onto Track-T's x-y plane,
/// radians in [-pi, pi], positive towards +y (right).
/// Directions must be finite and have horizontal length at least 1e-6 of
/// their total length; otherwise throws std::invalid_argument.
[[nodiscard]] double ForwardHeadingRadians(
    const Eigen::Vector3d& forward_in_track);

/// Heading perpendicular to a wheel axle projected onto Track-T's x-y
/// plane, radians in (-pi/2, pi/2], positive to the right. Independent of
/// axle sign and rotation about the axle. Same input domain as above.
[[nodiscard]] double AxleHeadingRadians(const Eigen::Vector3d& axle_in_track);

}  // namespace orvd::scene_observation
