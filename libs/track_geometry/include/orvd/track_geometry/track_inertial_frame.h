#pragma once

#include <Eigen/Core>

// The track inertial frame I, and the two track-attached frames built on it.
//
// I is fixed to the line, not to the earth: its origin is the start of the
// line and
//
//   +x  points along the direction of increasing track station at that origin,
//   +y  points to the right of an observer facing increasing track station,
//   +z  points downward.
//
// The frame is right-handed with z down, so gravity is +z and a positive
// rotation about +z turns the heading from +x toward +y, that is, to the right.
// A right-hand curve therefore has positive curvature.
//
// Two frames ride along the line. The roll-free tangent frame has its x axis
// along the unit tangent of the three-dimensional centerline, its y axis
// horizontal and pointing to the track's right, and its z axis completing the
// right-handed triad. The track frame T is the roll-free frame rolled about its
// own x axis by the track roll angle that the superelevation implies; T is what
// the rails and the vehicle see.
//
// The multibody base stays coordinate-neutral. Nothing here sets gravity on a
// model: a vehicle assembler does that explicitly, and this header only states
// which direction the line layer means by "down".

namespace orvd::track_geometry {

// The downward unit vector of I. Named rather than spelled out at each use so
// that the sign convention has one definition instead of one per caller.
[[nodiscard]] inline Eigen::Vector3d DownwardUnitVectorInInertial() {
    return Eigen::Vector3d(0.0, 0.0, 1.0);
}

// The gravitational acceleration a vehicle assembler must install when it
// builds a model against this line. The magnitude is the caller's to choose;
// the direction is not.
//
// Throws std::invalid_argument when the magnitude is not finite and positive. A
// negative magnitude is refused rather than honoured: it would be a way to flip
// the direction past the one definition this header exists to hold.
[[nodiscard]] Eigen::Vector3d GravitationalAccelerationInInertial(
    double magnitude_meters_per_second_squared);

}  // namespace orvd::track_geometry
