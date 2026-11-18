#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <expected>
#include <string>
#include <string_view>

#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

// What a rotation property reads from the record holding it, and the rotation
// an edit of one gives (PropertyOrientation). Whether a record holds a
// rotation is judged by the rule the check applies to that record, and by no
// other: a marker's matrix by the rule the multibody layer applies to a fixed
// frame's pose when the check assembles the vehicle, which it exports
// (orvd/multibody_runtime/multibody_physical_parameter_validation.h); a
// carrier's wheel-profile basis by the contact declaration's rule, and a
// placement's quaternion by the project's invariants, which the
// configuration and vehicle project libraries keep to themselves
// (vehicle_contact_topology.cc and vehicle_project_invariants.cc there) and
// which are stated here as they state them.

namespace orvd::vehicle_project_editing::internal {

// The orientation a marker's matrix holds: the matrix as it is, and the
// angles giving it where it is a rotation.
[[nodiscard]] PropertyOrientation ReadOrientationOfMarker(const Eigen::Matrix3d& matrix);

// The orientation a carrier's wheel-profile basis holds: the matrix as it is,
// and the angles giving it where it is a rotation.
[[nodiscard]] PropertyOrientation ReadOrientationOfCarrierBasis(const Eigen::Matrix3d& matrix);

// The orientation a placement's quaternion holds: its matrix and the angles
// giving it where it is of unit length, and neither where it is not.
[[nodiscard]] PropertyOrientation ReadOrientationOfPlacement(const Eigen::Quaterniond& quaternion);

// The rotation an edit of the rotation property named `property_name` gives:
// the one its three angles give, or why the value gives none.
[[nodiscard]] std::expected<Eigen::Matrix3d, std::string> RotationGivenByEdit(
    std::string_view property_name, const PropertyOrientation& value);

}  // namespace orvd::vehicle_project_editing::internal
