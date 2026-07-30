#include "comparison/observation_comparator.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <Eigen/Dense>

namespace orvd_comparison {
namespace {

using orvd_contract::ObservationKind;

// Below these magnitudes a difference carries no engineering meaning for this
// harness, so the relative branch would only amplify noise. They are engineering
// floors per dimension, not a roundoff model.
constexpr double kForceAbsoluteFloorNewtons = 1e-9;
constexpr double kTorqueAbsoluteFloorNewtonMetres = 1e-9;
constexpr double kTranslationAbsoluteFloorMeters = 1e-12;
constexpr double kAngleAbsoluteFloorRadians = 1e-12;
constexpr double kUnitQuaternionComponentAbsoluteFloor = 1e-12;

double AbsoluteFloorForKind(ObservationKind kind) {
    switch (kind) {
        case ObservationKind::kForceNewtons:            return kForceAbsoluteFloorNewtons;
        case ObservationKind::kTorqueNewtonMetres:      return kTorqueAbsoluteFloorNewtonMetres;
        case ObservationKind::kTranslationMeters:       return kTranslationAbsoluteFloorMeters;
        case ObservationKind::kAngleRadians:            return kAngleAbsoluteFloorRadians;
        case ObservationKind::kUnitQuaternionComponent: return kUnitQuaternionComponentAbsoluteFloor;
    }
    throw std::logic_error("Observation kind has no absolute-error floor");
}

// Elements are named `<group>[row,col]`. A group missing any of its nine
// elements is not a rotation and is reported as missing rather than measured.
bool ExtractRotation(const orvd_contract::ObservationStream& stream,
                     const std::string& group_name, Eigen::Matrix3d* rotation,
                     std::string* unusable_element,
                     std::string* invalid_reason) {
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            const std::string element_name = group_name + "[" + std::to_string(row) +
                                             "," + std::to_string(column) + "]";
            const auto* observation = stream.FindObservation(element_name);
            if (observation == nullptr) {
                *unusable_element = element_name;
                const auto* unusable =
                    stream.FindUnusableObservation(element_name);
                *invalid_reason = unusable == nullptr ? "" : unusable->reason;
                return false;
            }
            (*rotation)(row, column) = observation->value;
        }
    }
    return true;
}

// The angle of the relative rotation. The atan2 form keeps first-order
// information at small angles, where an arccos of the trace loses all resolution
// below roughly the square root of machine epsilon.
double GeodesicAngleBetweenRotations(const Eigen::Matrix3d& reference,
                                     const Eigen::Matrix3d& candidate) {
    const Eigen::Matrix3d relative = reference.transpose() * candidate;
    const Eigen::Vector3d skew_vector(relative(2, 1) - relative(1, 2),
                                      relative(0, 2) - relative(2, 0),
                                      relative(1, 0) - relative(0, 1));
    return std::atan2(0.5 * skew_vector.norm(), 0.5 * (relative.trace() - 1.0));
}

constexpr double kRotationOrthonormalitySlack = 1e-9;

bool IsRotation(const Eigen::Matrix3d& matrix) {
    return (matrix.transpose() * matrix - Eigen::Matrix3d::Identity())
                   .cwiseAbs().maxCoeff() <= kRotationOrthonormalitySlack &&
           std::abs(matrix.determinant() - 1.0) <= kRotationOrthonormalitySlack;
}

std::string FormatFloatingPoint(double value) {
    std::ostringstream text;
    text << std::scientific << std::setprecision(17) << value;
    return text.str();
}

}  // namespace

ToleranceBranch SelectToleranceBranch(double reference_value, ObservationKind kind) {
    return kRelativeErrorLimit * std::abs(reference_value) >= AbsoluteFloorForKind(kind)
               ? ToleranceBranch::kRelativeError
               : ToleranceBranch::kNearZeroAbsoluteError;
}

ComparisonReport CompareObservationStreams(
    const ComparisonRequirements& requirements,
    const orvd_contract::ObservationStream& reference,
    const orvd_contract::ObservationStream& candidate) {
    ComparisonReport report;

    // Sizes and counts are discrete facts; "close" has no meaning for them.
    for (const std::string& fact_name : requirements.topology_fact_names) {
        const auto* reference_fact = reference.FindTopologyFact(fact_name);
        const auto* candidate_fact = candidate.FindTopologyFact(fact_name);
        if (reference_fact == nullptr || candidate_fact == nullptr) {
            report.outcome = ComparisonOutcome::kRequiredObservationMissing;
            report.detail = "required topology fact absent: " + fact_name;
            return report;
        }
        if (reference_fact->value != candidate_fact->value) {
            report.outcome = ComparisonOutcome::kTopologyMismatch;
            report.detail = fact_name + ": " + std::to_string(reference_fact->value) +
                            " vs " + std::to_string(candidate_fact->value);
            return report;
        }
    }

    for (const RequiredRotation& required : requirements.rotations) {
        Eigen::Matrix3d reference_rotation, candidate_rotation;
        std::string unusable_element;
        std::string invalid_reason;
        if (!ExtractRotation(reference, required.group_name, &reference_rotation,
                             &unusable_element, &invalid_reason) ||
            !ExtractRotation(candidate, required.group_name, &candidate_rotation,
                             &unusable_element, &invalid_reason)) {
            const bool element_is_invalid = !invalid_reason.empty();
            report.outcome = element_is_invalid
                                 ? ComparisonOutcome::kRequiredObservationInvalid
                                 : ComparisonOutcome::kRequiredObservationMissing;
            report.detail = "rotation element " +
                            std::string(element_is_invalid ? "invalid: " : "absent: ") +
                            unusable_element;
            if (element_is_invalid) report.detail += " (" + invalid_reason + ")";
            return report;
        }
        // Taking an angle between things that are not rotations produces a
        // number, and that number can look excellent. Check first.
        if (!IsRotation(reference_rotation) || !IsRotation(candidate_rotation)) {
            report.outcome = ComparisonOutcome::kRotationNotOrthonormal;
            report.detail = required.group_name + " is not a rotation";
            return report;
        }
        const double angle =
            GeodesicAngleBetweenRotations(reference_rotation, candidate_rotation);
        if (angle > kRotationAngleErrorLimitRadians) {
            report.outcome = ComparisonOutcome::kToleranceExceeded;
            report.detail =
                required.group_name + ": geodesic angle " +
                FormatFloatingPoint(angle) + " rad exceeds " +
                FormatFloatingPoint(kRotationAngleErrorLimitRadians);
            return report;
        }
    }

    for (const RequiredObservation& required : requirements.scalars) {
        const auto* reference_observation = reference.FindObservation(required.name);
        const auto* candidate_observation = candidate.FindObservation(required.name);
        if (reference_observation == nullptr || candidate_observation == nullptr) {
            const auto* invalid_reference =
                reference.FindUnusableObservation(required.name);
            const auto* invalid_candidate =
                candidate.FindUnusableObservation(required.name);
            const bool invalid =
                invalid_reference != nullptr || invalid_candidate != nullptr;
            report.outcome = invalid ? ComparisonOutcome::kRequiredObservationInvalid
                                     : ComparisonOutcome::kRequiredObservationMissing;
            report.detail = "required observation " +
                            std::string(invalid ? "invalid: " : "absent: ") +
                            required.name;
            if (invalid) {
                const auto* invalid_observation =
                    invalid_reference != nullptr ? invalid_reference : invalid_candidate;
                report.detail += " (" + invalid_observation->reason + ")";
            }
            return report;
        }
        if (!std::isfinite(reference_observation->value) ||
            !std::isfinite(candidate_observation->value)) {
            report.outcome = ComparisonOutcome::kRequiredObservationInvalid;
            report.detail = "required observation is not finite: " + required.name;
            return report;
        }
        const double error =
            std::abs(candidate_observation->value - reference_observation->value);
        const double relative_allowance =
            kRelativeErrorLimit * std::abs(reference_observation->value);
        const double absolute_floor = AbsoluteFloorForKind(required.kind);
        const bool relative_branch = relative_allowance >= absolute_floor;
        const double allowance = relative_branch ? relative_allowance : absolute_floor;
        if (relative_branch) ++report.relative_branch_count;

        if (error > allowance) {
            report.outcome = ComparisonOutcome::kToleranceExceeded;
            report.detail = required.name + " [" +
                            (relative_branch ? "relative" : "near-zero absolute") +
                            "]: error " + FormatFloatingPoint(error) +
                            " exceeds allowance " + FormatFloatingPoint(allowance);
            return report;
        }
    }
    return report;
}

}  // namespace orvd_comparison
