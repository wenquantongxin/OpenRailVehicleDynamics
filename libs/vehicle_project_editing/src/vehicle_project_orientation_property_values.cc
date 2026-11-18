#include "vehicle_project_orientation_property_values.h"

#include <cmath>
#include <stdexcept>

#include "orvd/multibody_runtime/multibody_physical_parameter_validation.h"
#include "orvd/multibody_runtime/multibody_physical_parameters.h"
#include "orvd/vehicle_project_editing/vehicle_project_rotation_from_and_to_roll_pitch_yaw_radians.h"
#include "vehicle_project_property_edit_value_checks.h"

namespace orvd::vehicle_project_editing::internal {
namespace {

// The tolerance of the contact declaration's rule on a carrier's basis and of
// the project's invariant on a placement's quaternion: a few rounding steps,
// not a repair budget.
constexpr double kRotationTolerance = 1.0e-12;

bool IsRotationAsMarker(const Eigen::Matrix3d& matrix) {
    try {
        multibody_runtime::ThrowIfNotAValidFixedFramePose({.R_PF = matrix}, "a marker");
    } catch (const std::invalid_argument&) {
        return false;
    }
    return true;
}

bool IsRotationAsCarrierBasis(const Eigen::Matrix3d& matrix) {
    return matrix.allFinite() &&
           (matrix.transpose() * matrix - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <=
               kRotationTolerance &&
           std::abs(matrix.determinant() - 1.0) <= kRotationTolerance;
}

bool IsOfUnitLengthAsPlacement(const Eigen::Quaterniond& quaternion) {
    return std::abs(quaternion.norm() - 1.0) <= kRotationTolerance;
}

PropertyOrientation ReadOrientationOfMatrix(const Eigen::Matrix3d& matrix, bool is_rotation) {
    if (!is_rotation) {
        return {.matrix = matrix};
    }
    return {.matrix = matrix, .roll_pitch_yaw_radians = RollPitchYawRadiansOfRotation(matrix)};
}

}  // namespace

PropertyOrientation ReadOrientationOfMarker(const Eigen::Matrix3d& matrix) {
    return ReadOrientationOfMatrix(matrix, IsRotationAsMarker(matrix));
}

PropertyOrientation ReadOrientationOfCarrierBasis(const Eigen::Matrix3d& matrix) {
    return ReadOrientationOfMatrix(matrix, IsRotationAsCarrierBasis(matrix));
}

PropertyOrientation ReadOrientationOfPlacement(const Eigen::Quaterniond& quaternion) {
    if (!IsOfUnitLengthAsPlacement(quaternion)) {
        return {};
    }
    return ReadOrientationOfMatrix(quaternion.toRotationMatrix(), true);
}

std::expected<Eigen::Matrix3d, std::string> RotationGivenByEdit(std::string_view property_name,
                                                                const PropertyOrientation& value) {
    if (value.matrix.has_value() || !value.roll_pitch_yaw_radians.has_value()) {
        return std::unexpected(QuotePropertyName(property_name) +
                               " is set by its roll, pitch and yaw in radians alone");
    }
    if (!value.roll_pitch_yaw_radians->allFinite()) {
        return std::unexpected(QuotePropertyName(property_name) + " takes three finite angles");
    }
    return RotationFromRollPitchYawRadians(*value.roll_pitch_yaw_radians);
}

}  // namespace orvd::vehicle_project_editing::internal
