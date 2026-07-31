// Launches the reference and the candidate as separate processes, compares what
// they observed through pipes, and proves the comparison can actually fail.
//
// Two things happen here and they are not the same thing. Running the reference
// twice and comparing it with itself establishes that the harness works — the
// pipe, the parser and the judge — and nothing about any candidate. Running the
// reference against ORVD's own emitter is the equivalence claim, and it is made
// only over the capability ORVD has actually landed: the judge's
// position-kinematics requirement set says which observations that is, and it
// refuses a missing one rather than skipping it.
//
// Separate processes are not tidiness. libdrake exports the same drake:: symbols
// a vendored copy keeps, so co-linking the two would be an ODR violation whose
// likely symptom is a comparison that appears to pass.
//
// Nothing is written to disk: the streams exist only in the pipe and in memory.
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <Eigen/Dense>

#include "comparison/observation_comparator.h"
#include "comparison/required_observations.h"
#include "contract/observation_stream.h"
#include "contract/scenario_definition.h"

namespace {

int failure_count = 0;

void Expect(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failure_count;
    }
}

// Starts the emitter directly and returns what it wrote. The child's
// exit status is checked: a process that failed to start produces no
// observations, and reporting that as "a required observation is missing" would
// name the symptom and hide the cause.
bool CaptureReferenceEmitterOutput(const std::string& emitter_path,
                                   std::string_view excitation,
                                   std::string* output) {
    int standard_output_pipe[2];
    if (pipe(standard_output_pipe) != 0) {
        std::perror("pipe");
        return false;
    }

    const pid_t child_process_id = fork();
    if (child_process_id < 0) {
        std::perror("fork");
        close(standard_output_pipe[0]);
        close(standard_output_pipe[1]);
        return false;
    }
    if (child_process_id == 0) {
        close(standard_output_pipe[0]);
        if (dup2(standard_output_pipe[1], STDOUT_FILENO) < 0) _exit(126);
        close(standard_output_pipe[1]);
        const std::string excitation_argument(excitation);
        execl(emitter_path.c_str(), emitter_path.c_str(),
              excitation_argument.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }

    close(standard_output_pipe[1]);
    output->clear();
    char buffer[4096];
    ssize_t bytes_read = 0;
    while ((bytes_read = read(standard_output_pipe[0], buffer, sizeof(buffer))) > 0)
        output->append(buffer, static_cast<std::size_t>(bytes_read));
    const int read_error = errno;
    close(standard_output_pipe[0]);

    int child_status = 0;
    while (waitpid(child_process_id, &child_status, 0) < 0) {
        if (errno == EINTR) continue;
        std::perror("waitpid");
        return false;
    }
    if (bytes_read < 0) {
        errno = read_error;
        std::perror("read reference emitter output");
        return false;
    }
    if (!WIFEXITED(child_status) || WEXITSTATUS(child_status) != 0) {
        if (WIFEXITED(child_status)) {
            std::fprintf(stderr, "reference emitter exited with status %d\n",
                         WEXITSTATUS(child_status));
        } else {
            std::fprintf(stderr, "reference emitter terminated abnormally\n");
        }
        return false;
    }
    return true;
}

// Rebuilds one observation stream with a single value perturbed, so the
// perturbation is a legal stream rather than a corrupted one.
orvd_contract::ObservationStream WithScalarPerturbed(
    const orvd_contract::ObservationStream& original, const std::string& name,
    double relative_change) {
    orvd_contract::ObservationStream perturbed = original;
    for (auto& observation : perturbed.observations)
        if (observation.name == name)
            observation.value *= (1.0 + relative_change);
    return perturbed;
}

// Rotates one pose by a legal rotation of the requested angle, so what the
// comparator sees is still orthonormal and the angle gate, not the
// orthonormality gate, is what rejects it.
orvd_contract::ObservationStream WithRotationPerturbed(
    const orvd_contract::ObservationStream& original, const std::string& group_name,
    double angle_radians) {
    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            rotation(row, column) =
                original
                    .FindObservation(group_name + "[" + std::to_string(row) + "," +
                                     std::to_string(column) + "]")
                    ->value;
    const Eigen::Matrix3d rotated =
        rotation * Eigen::AngleAxisd(angle_radians, Eigen::Vector3d::UnitZ())
                       .toRotationMatrix();

    orvd_contract::ObservationStream perturbed = original;
    for (auto& observation : perturbed.observations)
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                if (observation.name == group_name + "[" + std::to_string(row) + "," +
                                            std::to_string(column) + "]")
                    observation.value = rotated(row, column);
    return perturbed;
}

}  // namespace

/// Checks that the requirement set really asks for the capability it is named
/// for.
///
/// The judge refuses a missing observation, so a requirement set that forgot to
/// ask for something produces a passing comparison over less than it claims.
/// That is the one failure this whole file cannot notice by running, which is
/// why the set is inspected rather than trusted.
void ExpectRequirementsCoverPositionKinematics(
    const orvd_comparison::ComparisonRequirements& requirements,
    const orvd_contract::ScenarioDefinition& scenario) {
    const auto has_scalar = [&requirements](const std::string& name) {
        for (const auto& required : requirements.scalars)
            if (required.name == name) return true;
        return false;
    };
    const auto has_rotation = [&requirements](const std::string& group) {
        for (const auto& required : requirements.rotations)
            if (required.group_name == group) return true;
        return false;
    };
    const auto has_fact = [&requirements](const std::string& name) {
        for (const auto& fact_name : requirements.topology_fact_names)
            if (fact_name == name) return true;
        return false;
    };

    for (const auto& link : scenario.links) {
        Expect(has_rotation("pose_" + link.name),
               "every link's orientation must be required: " + link.name);
        for (int axis_index = 0; axis_index < 3; ++axis_index)
            Expect(has_scalar("pose_" + link.name + "_translation[" +
                              std::to_string(axis_index) + "]"),
                   "every component of every link's position must be required: " +
                       link.name);
    }
    for (std::size_t position_index = 0;
         position_index < scenario.generalized_positions.size(); ++position_index)
        Expect(has_scalar("state_readback_position[" +
                          std::to_string(position_index) + "]"),
               "every generalized position must be read back: index " +
                   std::to_string(position_index));
    for (const auto& joint : scenario.revolute_joints)
        for (const char* fact :
             {"joint_position_range_start", "joint_position_range_size",
              "joint_velocity_range_start", "joint_velocity_range_size"})
            Expect(has_fact(std::string(fact) + "[" + joint.name + "]"),
                   "every joint's coordinate range must be a required fact: " +
                       joint.name);
    for (const auto& free_body_name : scenario.free_body_names)
        for (const char* fact : {"free_body_position_range_start",
                                 "free_body_position_range_size",
                                 "free_body_velocity_range_start",
                                 "free_body_velocity_range_size"})
            Expect(has_fact(std::string(fact) + "[" + free_body_name + "]"),
                   "every free body's coordinate range must be a required "
                   "fact: " + free_body_name);

    std::vector<std::string> every_name = requirements.topology_fact_names;
    for (const auto& required : requirements.scalars)
        every_name.push_back(required.name);
    for (const auto& required : requirements.rotations)
        every_name.push_back(required.group_name);
    std::sort(every_name.begin(), every_name.end());
    Expect(std::adjacent_find(every_name.begin(), every_name.end()) ==
               every_name.end(),
           "no requirement may be named twice: a duplicate is a second chance "
           "to pass on the same evidence");
}

/// Drops one named observation, so the missing-observation path is exercised on
/// something this comparison actually requires.
orvd_contract::ObservationStream WithObservationRemoved(
    orvd_contract::ObservationStream stream, const std::string& name) {
    for (auto it = stream.observations.begin(); it != stream.observations.end();
         ++it) {
        if (it->name == name) {
            stream.observations.erase(it);
            return stream;
        }
    }
    Expect(false, "the stream was expected to contain " + name);
    return stream;
}

/// Drops one named topology fact, for the same reason.
orvd_contract::ObservationStream WithTopologyFactRemoved(
    orvd_contract::ObservationStream stream, const std::string& name) {
    for (auto it = stream.topology_facts.begin();
         it != stream.topology_facts.end(); ++it) {
        if (it->name == name) {
            stream.topology_facts.erase(it);
            return stream;
        }
    }
    Expect(false, "the stream was expected to contain the fact " + name);
    return stream;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage: %s <reference_emitter_path> "
                     "<candidate_emitter_path>\n",
                     argv[0]);
        return 2;
    }
    const std::string emitter_path = argv[1];
    const std::string candidate_path = argv[2];

    for (const std::string_view excitation :
         {"near_zero_cancellation", "dynamic_excitation"}) {
        const orvd_contract::ScenarioDefinition scenario =
            orvd_contract::MakeRevoluteChainWithFloatingBodyScenario(excitation);
        const orvd_comparison::ComparisonRequirements requirements =
            orvd_comparison::MakeComparisonRequirements(scenario);

        // Two independent processes of the same reference implementation.
        std::string first_text, second_text;
        if (!CaptureReferenceEmitterOutput(emitter_path, excitation, &first_text) ||
            !CaptureReferenceEmitterOutput(emitter_path, excitation, &second_text)) {
            std::fprintf(stderr, "reference emitter failed to run\n");
            return 1;
        }
        const orvd_contract::ObservationStream first =
            orvd_contract::ParseObservationStream(first_text);
        const orvd_contract::ObservationStream second =
            orvd_contract::ParseObservationStream(second_text);

        const auto control = orvd_comparison::CompareObservationStreams(
            requirements, first, second);
        Expect(control.outcome == orvd_comparison::ComparisonOutcome::kAccepted,
               "two runs of the same reference must agree: " + control.detail);

        if (excitation != "dynamic_excitation") continue;

        // A perturbation only proves anything if it lands where the relative
        // gate governs. Pick a required scalar that is genuinely away from zero,
        // assert which branch it is in, and only then perturb it.
        const orvd_comparison::RequiredObservation* relative_branch_target = nullptr;
        for (const auto& required : requirements.scalars) {
            const auto* observation = first.FindObservation(required.name);
            if (observation == nullptr) continue;
            if (orvd_comparison::SelectToleranceBranch(observation->value,
                                                       required.kind) ==
                orvd_comparison::ToleranceBranch::kRelativeError) {
                relative_branch_target = &required;
                break;
            }
        }
        Expect(relative_branch_target != nullptr,
               "the scenario must produce at least one scalar in the relative branch");
        if (relative_branch_target == nullptr) break;

        const auto tolerated = orvd_comparison::CompareObservationStreams(
            requirements, first,
            WithScalarPerturbed(
                second, relative_branch_target->name,
                0.5 * orvd_comparison::kRelativeErrorLimit));
        Expect(tolerated.outcome == orvd_comparison::ComparisonOutcome::kAccepted,
               "a perturbation inside the relative limit must be accepted");

        const auto rejected = orvd_comparison::CompareObservationStreams(
            requirements, first,
            WithScalarPerturbed(
                second, relative_branch_target->name,
                5.0 * orvd_comparison::kRelativeErrorLimit));
        Expect(rejected.outcome == orvd_comparison::ComparisonOutcome::kToleranceExceeded,
               "a perturbation beyond the relative limit must be rejected");

        Expect(control.relative_branch_count > 0,
               "the relative gate must actually engage on this scenario");

        // Rotation is judged separately, so it gets its own perturbation.
        const std::string rotation_group = requirements.rotations.front().group_name;
        const auto rotation_tolerated = orvd_comparison::CompareObservationStreams(
            requirements, first,
            WithRotationPerturbed(
                second, rotation_group,
                0.5 * orvd_comparison::kRotationAngleErrorLimitRadians));
        Expect(rotation_tolerated.outcome ==
                   orvd_comparison::ComparisonOutcome::kAccepted,
               "a rotation perturbation inside the angle limit must be accepted");

        const auto rotation_rejected = orvd_comparison::CompareObservationStreams(
            requirements, first,
            WithRotationPerturbed(
                second, rotation_group,
                5.0 * orvd_comparison::kRotationAngleErrorLimitRadians));
        Expect(rotation_rejected.outcome ==
                   orvd_comparison::ComparisonOutcome::kToleranceExceeded,
               "a rotation perturbation beyond the angle limit must be rejected");

        // A required observation that is absent must be reported as missing
        // rather than quietly skipped. Removed by name, and by a name this
        // comparison genuinely requires: dropping whichever observation happens
        // to be first would pass even if that one were not required at all.
        for (const std::string& removed :
             {std::string("pose_") + scenario.links.front().name + "[0,0]",
              std::string("pose_") + scenario.links.front().name +
                  "_translation[0]",
              std::string("state_readback_position[0]")}) {
            const auto missing = orvd_comparison::CompareObservationStreams(
                requirements, first, WithObservationRemoved(second, removed));
            Expect(missing.outcome ==
                       orvd_comparison::ComparisonOutcome::
                           kRequiredObservationMissing,
                   "a missing required observation must be reported: " + removed);
        }
        const auto missing_fact = orvd_comparison::CompareObservationStreams(
            requirements, first,
            WithTopologyFactRemoved(
                second, "free_body_position_range_start[" +
                            scenario.free_body_names.front() + "]"));
        Expect(missing_fact.outcome ==
                   orvd_comparison::ComparisonOutcome::kRequiredObservationMissing,
               "a missing required topology fact must be reported");
    }

    // The equivalence claim. Everything above proves the judge can fail; this is
    // the part that says something about ORVD, and it is deliberately confined
    // to the capability ORVD has landed. A wider claim would have to come from a
    // wider requirement set, which is a thing a later goal builds rather than a
    // thing this one can assert.
    for (const std::string_view excitation :
         {"near_zero_cancellation", "dynamic_excitation"}) {
        const orvd_contract::ScenarioDefinition scenario =
            orvd_contract::MakeRevoluteChainWithFloatingBodyScenario(excitation);
        const orvd_comparison::ComparisonRequirements requirements =
            orvd_comparison::MakePositionKinematicsComparisonRequirements(
                scenario);
        ExpectRequirementsCoverPositionKinematics(requirements, scenario);

        std::string reference_text, candidate_text;
        if (!CaptureReferenceEmitterOutput(emitter_path, excitation,
                                           &reference_text) ||
            !CaptureReferenceEmitterOutput(candidate_path, excitation,
                                           &candidate_text)) {
            std::fprintf(stderr, "an emitter failed to run\n");
            return 1;
        }
        const orvd_contract::ObservationStream reference =
            orvd_contract::ParseObservationStream(reference_text);
        const orvd_contract::ObservationStream candidate =
            orvd_contract::ParseObservationStream(candidate_text);

        // The scenario states q, v, v̇ and the generalized-force kinds by global
        // index, so those indices mean something only if both sides lay their
        // coordinates out the way this fixture assumes. Comparing the two sides
        // is not enough: a reordering that happened on both at once would agree
        // with itself. So the layout this fixture depends on is pinned here.
        const auto* free_body_velocity_start = reference.FindTopologyFact(
            "free_body_velocity_range_start[" + scenario.free_body_names.front() +
            "]");
        Expect(free_body_velocity_start != nullptr,
               "the reference must report the free body's velocity range");
        if (free_body_velocity_start != nullptr) {
            Expect(free_body_velocity_start->value == 2,
                   "this fixture's free body starts at velocity 2, after the "
                   "two revolute coordinates");
            // The scenario calls the last three generalized forces translational
            // because they belong to the free body's linear velocities, which
            // begin three past its angular ones. If the layout moved, that
            // labelling would be describing a different set of coordinates.
            std::size_t first_translational = 0;
            while (first_translational <
                       scenario.generalized_force_component_kinds.size() &&
                   scenario.generalized_force_component_kinds[first_translational] !=
                       orvd_contract::GeneralizedForceComponentKind::kForceNewtons)
                ++first_translational;
            Expect(static_cast<int>(first_translational) ==
                       free_body_velocity_start->value + 3,
                   "the scenario's first translational generalized force must "
                   "be the free body's first linear velocity");
        }

        const auto verdict = orvd_comparison::CompareObservationStreams(
            requirements, reference, candidate);
        Expect(verdict.outcome == orvd_comparison::ComparisonOutcome::kAccepted,
               std::string("the ORVD candidate must agree with the Drake "
                           "reference on position kinematics (") +
                   std::string(excitation) + "): " + verdict.detail);
        if (excitation == "dynamic_excitation") {
            Expect(verdict.relative_branch_count > 0,
                   "and must do so with the relative gate actually engaged, "
                   "not entirely in the near-zero branch");
        }
    }

    if (failure_count > 0) {
        std::fprintf(stderr, "%d comparison harness check(s) failed\n", failure_count);
        return 1;
    }
    std::printf(
        "the harness fails when it should, and the ORVD candidate agrees with "
        "the Drake reference on position kinematics\n");
    return 0;
}
