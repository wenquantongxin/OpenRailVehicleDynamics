#include <algorithm>
#include <cstdio>
#include <limits>
#include <string>
#include <string_view>

#include <Eigen/Dense>

#include "comparison/observation_comparator.h"
#include "contract/observation_stream.h"

namespace {

int failure_count = 0;

void Expect(bool condition, std::string_view description) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %.*s\n",
                     static_cast<int>(description.size()), description.data());
        ++failure_count;
    }
}

orvd_contract::ObservationStream MakeStream(double force_newtons) {
    orvd_contract::ObservationStream stream;
    stream.topology_facts = {{"num_velocities", 1}};
    stream.observations = {{"generalized_force[0]", force_newtons}};
    const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            stream.observations.push_back(
                {"body_rotation[" + std::to_string(row) + "," +
                     std::to_string(column) + "]",
                 rotation(row, column)});
        }
    }
    return stream;
}

orvd_comparison::ComparisonRequirements MakeRequirements() {
    orvd_comparison::ComparisonRequirements requirements;
    requirements.topology_fact_names = {"num_velocities"};
    requirements.scalars = {
        {"generalized_force[0]", orvd_contract::ObservationKind::kForceNewtons}};
    requirements.rotations = {{"body_rotation"}};
    return requirements;
}

void CheckWhichBranchJudgesWhichMagnitude() {
    // The branch is chosen by comparing the relative allowance against the
    // dimension's own floor, so tightening the relative limit moves the
    // crossover up. These are the magnitudes the rail-vehicle quantities
    // actually sit at, pinned so that a later change to either constant has to
    // be a change somebody decided on rather than one that happened.
    using orvd_comparison::SelectToleranceBranch;
    using orvd_comparison::ToleranceBranch;
    using orvd_contract::ObservationKind;

    Expect(SelectToleranceBranch(1.0, ObservationKind::kTranslationMeters) ==
               ToleranceBranch::kRelativeError,
           "a translation of a metre is judged proportionally");
    Expect(SelectToleranceBranch(1e-5, ObservationKind::kTranslationMeters) ==
               ToleranceBranch::kNearZeroAbsoluteError,
           "a contact-scale translation of ten micrometres is judged against "
           "the absolute floor, because a proportional allowance there would be "
           "smaller than the floor itself");
    Expect(SelectToleranceBranch(0.37, ObservationKind::kAngleRadians) ==
               ToleranceBranch::kRelativeError,
           "a joint angle of a third of a radian is judged proportionally");
    Expect(SelectToleranceBranch(1e-6, ObservationKind::kAngleRadians) ==
               ToleranceBranch::kNearZeroAbsoluteError,
           "a microradian is judged against the absolute floor");
    Expect(SelectToleranceBranch(1.0e3, ObservationKind::kForceNewtons) ==
               ToleranceBranch::kRelativeError,
           "a kilonewton is judged proportionally");
}

void CheckRelativeAndNearZeroLimits() {
    const auto requirements = MakeRequirements();
    const auto reference = MakeStream(100.0);
    constexpr double reference_force = 100.0;
    constexpr double inside_relative_limit =
        0.5 * orvd_comparison::kRelativeErrorLimit;
    constexpr double outside_relative_limit =
        5.0 * orvd_comparison::kRelativeErrorLimit;

    Expect(orvd_comparison::CompareObservationStreams(
               requirements, reference,
               MakeStream(reference_force *
                          (1.0 + inside_relative_limit))).outcome ==
               orvd_comparison::ComparisonOutcome::kAccepted,
           "a relative error inside the declared limit must pass");
    Expect(orvd_comparison::CompareObservationStreams(
               requirements, reference,
               MakeStream(reference_force *
                          (1.0 + outside_relative_limit))).outcome ==
               orvd_comparison::ComparisonOutcome::kToleranceExceeded,
           "a relative error outside the declared limit must fail");

    const auto near_zero_reference = MakeStream(0.0);
    Expect(orvd_comparison::CompareObservationStreams(
               requirements, near_zero_reference, MakeStream(0.5e-9)).outcome ==
               orvd_comparison::ComparisonOutcome::kAccepted,
           "a near-zero force inside its absolute limit must pass");
    const auto near_zero_failure = orvd_comparison::CompareObservationStreams(
        requirements, near_zero_reference, MakeStream(1.5e-9));
    Expect(near_zero_failure.outcome ==
               orvd_comparison::ComparisonOutcome::kToleranceExceeded,
           "a near-zero force outside its absolute limit must fail");
    Expect(near_zero_failure.detail.find("e-") != std::string::npos,
           "a near-zero failure must retain its scientific magnitude");
}

void CheckRotationAndRequiredObservationFailures() {
    const auto requirements = MakeRequirements();
    const auto reference = MakeStream(1.0);

    auto different_topology = reference;
    different_topology.topology_facts.front().value += 1;
    Expect(orvd_comparison::CompareObservationStreams(
               requirements, reference, different_topology).outcome ==
               orvd_comparison::ComparisonOutcome::kTopologyMismatch,
           "a different required topology value must fail");

    auto non_rotation = reference;
    for (orvd_contract::Observation& observation : non_rotation.observations) {
        if (observation.name.starts_with("body_rotation[")) observation.value *= 2.0;
    }
    Expect(orvd_comparison::CompareObservationStreams(
               requirements, reference, non_rotation).outcome ==
               orvd_comparison::ComparisonOutcome::kRotationNotOrthonormal,
           "a scaled rotation matrix must be rejected");

    auto reflection = reference;
    for (orvd_contract::Observation& observation : reflection.observations) {
        if (observation.name == "body_rotation[0,0]") observation.value = -1.0;
    }
    Expect(orvd_comparison::CompareObservationStreams(
               requirements, reference, reflection).outcome ==
               orvd_comparison::ComparisonOutcome::kRotationNotOrthonormal,
           "an orthonormal matrix with determinant -1 must be rejected");

    auto missing = reference;
    std::erase_if(missing.observations, [](const orvd_contract::Observation& observation) {
        return observation.name == "generalized_force[0]";
    });
    Expect(orvd_comparison::CompareObservationStreams(
               requirements, reference, missing).outcome ==
               orvd_comparison::ComparisonOutcome::kRequiredObservationMissing,
           "a missing required observation must fail");

    auto non_finite = reference;
    for (orvd_contract::Observation& observation : non_finite.observations) {
        if (observation.name == "generalized_force[0]") {
            observation.value = std::numeric_limits<double>::quiet_NaN();
        }
    }
    Expect(orvd_comparison::CompareObservationStreams(
               requirements, reference, non_finite).outcome ==
               orvd_comparison::ComparisonOutcome::kRequiredObservationInvalid,
           "a non-finite required scalar must fail");
}

void CheckLooseParsingAtThePointOfUse() {
    const auto requirements = MakeRequirements();
    const auto reference = MakeStream(1.0);

    const std::string valid_text =
        orvd_contract::FormatObservationStream(reference);
    const auto with_unused_malformed_record =
        orvd_contract::ParseObservationStream(
            valid_text + "@observation unused_extension opaque_payload\n");
    Expect(orvd_comparison::CompareObservationStreams(
               requirements, reference, with_unused_malformed_record).outcome ==
               orvd_comparison::ComparisonOutcome::kAccepted,
           "an unused malformed observation must not affect comparison");

    const auto with_required_malformed_record =
        orvd_contract::ParseObservationStream(
            valid_text + "@observation generalized_force[0] opaque_payload\n");
    Expect(orvd_comparison::CompareObservationStreams(
               requirements, reference, with_required_malformed_record).outcome ==
               orvd_comparison::ComparisonOutcome::kRequiredObservationInvalid,
           "a malformed required observation must fail");

    orvd_contract::ObservationStream replacement;
    replacement.observations = {{"generalized_force[0]", 2.0}};
    const auto repeated = orvd_contract::ParseObservationStream(
        valid_text + orvd_contract::FormatObservationStream(replacement));
    const auto* repeated_force = repeated.FindObservation("generalized_force[0]");
    Expect(repeated_force != nullptr && repeated_force->value == 2.0,
           "a repeated observation must keep its last value");
}

}  // namespace

int main() {
    CheckWhichBranchJudgesWhichMagnitude();
    CheckRelativeAndNearZeroLimits();
    CheckRotationAndRequiredObservationFailures();
    CheckLooseParsingAtThePointOfUse();

    if (failure_count > 0) {
        std::fprintf(stderr, "%d comparator check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("observation comparator verified\n");
    return 0;
}
