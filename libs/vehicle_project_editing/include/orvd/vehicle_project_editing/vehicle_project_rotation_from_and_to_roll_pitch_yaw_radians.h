#pragma once

#include <Eigen/Core>

// An orientation stated by three angles in radians: roll about x, pitch about
// y and yaw about z, the axes being those of the frame the orientation is
// stated in, fixed, applied in that order:
//
//     R = Rz(yaw) Ry(pitch) Rx(roll).
//
// This is the kernel's convention and unit for its Ball-RPY joint
// (orvd/multibody_model/multibody_model.h), which states it and composes it
// but offers no function for it; here it is stated once for every orientation
// a project's properties hold. No record holds these angles: they are the form
// an orientation is read and edited in.

namespace orvd::vehicle_project_editing {

// The rotation three finite angles give.
[[nodiscard]] Eigen::Matrix3d RotationFromRollPitchYawRadians(
    const Eigen::Vector3d& roll_pitch_yaw_radians);

// Three angles giving `rotation`, a rotation matrix: roll and yaw in (-pi, pi],
// pitch in [-pi/2, pi/2]. Where pitch is pi/2 or -pi/2 the angles are not
// unique: yaw is then 0 and roll carries the rest. Near those two pitches,
// roll is taken from the entries cos(pitch) does not scale, so that the
// angles give the rotation again there as well.
[[nodiscard]] Eigen::Vector3d RollPitchYawRadiansOfRotation(const Eigen::Matrix3d& rotation);

}  // namespace orvd::vehicle_project_editing
