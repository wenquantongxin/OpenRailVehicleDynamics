// Judges two observation streams against the comparison requirements.
//
// Acceptance, declared once here and nowhere else:
//
//   |b - a| <= max(absolute_floor_for_kind, 1e-3 * |a|)
//
// The branch taken is reported, because a gate that never leaves the near-zero
// branch has not tested proportional accuracy at all. Rotations are judged by
// the SO(3) geodesic angle between them, against a flat 1e-3 rad.
//
// Bit equality is deliberately not the rule: two implementations legitimately
// differ in summation order and vectorisation, and demanding identical bit
// patterns would reject correct work while proving nothing of engineering
// interest.
#pragma once

#include <string>

#include "comparison/required_observations.h"
#include "contract/observation_stream.h"

namespace orvd_comparison {

inline constexpr double kRelativeErrorLimit = 1e-3;
inline constexpr double kRotationAngleErrorLimitRadians = 1e-3;

enum class ComparisonOutcome {
    kAccepted,
    kTopologyMismatch,
    kRequiredObservationMissing,
    kRequiredObservationInvalid,
    kRotationNotOrthonormal,
    kToleranceExceeded,
};

enum class ToleranceBranch { kRelativeError, kNearZeroAbsoluteError };

struct ComparisonReport {
    ComparisonOutcome outcome{ComparisonOutcome::kAccepted};
    std::string detail;
    // How many required scalars were judged in the relative branch. A run where
    // this is zero proves nothing about proportional accuracy.
    int relative_branch_count{};
};

[[nodiscard]] ComparisonReport CompareObservationStreams(
    const ComparisonRequirements& requirements,
    const orvd_contract::ObservationStream& reference,
    const orvd_contract::ObservationStream& candidate);

// Exposed so a self-check can assert which branch a given magnitude lands in
// before perturbing it.
[[nodiscard]] ToleranceBranch SelectToleranceBranch(
    double reference_value, orvd_contract::ObservationKind kind);

}  // namespace orvd_comparison
