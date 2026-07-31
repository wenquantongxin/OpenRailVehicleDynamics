// Launches the reference and the candidate as separate processes, compares what
// they observed through pipes, and proves the comparison can actually fail.
//
// Two things happen here and they are not the same thing. Running the reference
// twice and comparing it with itself establishes that the harness works — the
// pipe, the parser and the judge — and nothing about any candidate. Running the
// reference against ORVD's own emitter is the equivalence claim, and it is made
// only over the capability ORVD has actually landed: the judge's differential-
// kinematics requirement set says which observations that is, and it refuses a
// missing one rather than skipping it.
//
// Separate processes are not tidiness. libdrake exports the same drake:: symbols
// a vendored copy keeps, so co-linking the two would be an ODR violation whose
// likely symptom is a comparison that appears to pass.
//
// Nothing is written to disk: the streams exist only in the pipe and in memory.
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>
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
    for (const char* fact : {"num_positions", "num_velocities",
                             "num_rigid_bodies_excluding_world"})
        Expect(has_fact(fact),
               std::string("the coordinate counts must be required facts: ") +
                   fact);
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

void ExpectRequirementsCoverSpatialKinematics(
    const orvd_comparison::ComparisonRequirements& requirements,
    const orvd_contract::ScenarioDefinition& scenario) {
    ExpectRequirementsCoverPositionKinematics(requirements, scenario);
    const auto has_scalar =
        [&requirements](const std::string& name,
                        orvd_contract::ObservationKind expected_kind) {
        for (const auto& required : requirements.scalars)
            if (required.name == name && required.kind == expected_kind)
                return true;
        return false;
    };

    for (const auto& link : scenario.links) {
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            Expect(has_scalar("velocity_" + link.name + "_angular[" +
                                  std::to_string(axis_index) + "]",
                              orvd_contract::ObservationKind::
                                  kAngularVelocityRadiansPerSecond),
                   "every angular component of every link velocity must be "
                   "required with angular-velocity units: " +
                       link.name);
            Expect(has_scalar("velocity_" + link.name +
                                  "_translational_at_body_origin[" +
                                  std::to_string(axis_index) + "]",
                              orvd_contract::ObservationKind::
                                  kTranslationalVelocityMetersPerSecond),
                   "every body-origin translational component must be "
                   "required with translational-velocity units: " +
                       link.name);
        }
    }
    for (const auto& relative :
         scenario.relative_spatial_velocity_observations) {
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            Expect(has_scalar("relative_velocity_" + relative.name +
                                  "_angular[" +
                                  std::to_string(axis_index) + "]",
                              orvd_contract::ObservationKind::
                                  kAngularVelocityRadiansPerSecond),
                   "every angular component of a named relative velocity must "
                   "be required with angular-velocity units: " +
                       relative.name);
            Expect(
                has_scalar("relative_velocity_" + relative.name +
                               "_translational_at_moving_origin[" +
                               std::to_string(axis_index) + "]",
                           orvd_contract::ObservationKind::
                               kTranslationalVelocityMetersPerSecond),
                "every moving-origin translational component of a named "
                "relative velocity must be required with "
                "translational-velocity units: " +
                    relative.name);
        }
    }
    for (std::size_t velocity_index = 0;
         velocity_index < scenario.generalized_velocities.size();
         ++velocity_index) {
        Expect(has_scalar("state_readback_velocity[" +
                              std::to_string(velocity_index) + "]",
                          scenario.generalized_velocity_observation_kinds
                              [velocity_index]),
               "every generalized velocity must be read back with its "
               "declared physical kind: index " +
                   std::to_string(velocity_index));
    }
}

void ExpectRequirementsCoverDifferentialKinematics(
    const orvd_comparison::ComparisonRequirements& requirements,
    const orvd_contract::ScenarioDefinition& scenario) {
    ExpectRequirementsCoverSpatialKinematics(requirements, scenario);
    const auto has_scalar =
        [&requirements](const std::string& name,
                        orvd_contract::ObservationKind expected_kind) {
            for (const auto& required : requirements.scalars) {
                if (required.name == name && required.kind == expected_kind)
                    return true;
            }
            return false;
        };

    for (std::size_t position_index = 0;
         position_index < scenario.generalized_positions.size();
         ++position_index) {
        Expect(has_scalar(
                   "mapped_position_derivative[" +
                       std::to_string(position_index) + "]",
                   scenario.generalized_position_derivative_observation_kinds
                       [position_index]),
               "every mapped position derivative must be required with its "
               "physical kind: index " + std::to_string(position_index));
    }
    for (std::size_t velocity_index = 0;
         velocity_index < scenario.generalized_velocities.size();
         ++velocity_index) {
        Expect(has_scalar(
                   "mapped_back_generalized_velocity[" +
                       std::to_string(velocity_index) + "]",
                   scenario.generalized_velocity_observation_kinds
                       [velocity_index]),
               "every inverse-mapped velocity must be required: index " +
                   std::to_string(velocity_index));
    }
    for (const auto& link : scenario.links) {
        for (int axis_index = 0; axis_index < 3; ++axis_index) {
            Expect(
                has_scalar(
                    "acceleration_" + link.name + "_angular[" +
                        std::to_string(axis_index) + "]",
                    orvd_contract::ObservationKind::
                        kAngularAccelerationRadiansPerSecondSquared),
                "every body angular acceleration must be required: " +
                    link.name);
            Expect(
                has_scalar(
                    "acceleration_" + link.name +
                        "_translational_at_body_origin[" +
                        std::to_string(axis_index) + "]",
                    orvd_contract::ObservationKind::
                        kTranslationalAccelerationMetersPerSecondSquared),
                "every body-origin translational acceleration must be "
                "required: " + link.name);
            Expect(has_scalar(
                       "jacobian_probe_response_" + link.name + "_angular[" +
                           std::to_string(axis_index) + "]",
                       orvd_contract::ObservationKind::
                           kAngularVelocityRadiansPerSecond),
                   "every body Jacobian angular response must be required: " +
                       link.name);
            Expect(
                has_scalar(
                    "jacobian_probe_response_" + link.name +
                        "_translational_at_center_of_mass[" +
                        std::to_string(axis_index) + "]",
                    orvd_contract::ObservationKind::
                        kTranslationalVelocityMetersPerSecond),
                "every body Jacobian point response must be required: " +
                    link.name);
        }
    }
}

/// Moves one named observation by an absolute amount.
///
/// Distinct from the proportional perturbation helper: here the caller has
/// already worked out the step it wants, because the point is to choose one the
/// two allowances disagree about.
orvd_contract::ObservationStream WithScalarValueOffset(
    orvd_contract::ObservationStream stream, const std::string& name,
    double offset) {
    for (auto& observation : stream.observations) {
        if (observation.name == name) {
            observation.value += offset;
            return stream;
        }
    }
    Expect(false, "the stream was expected to contain " + name);
    return stream;
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
    std::error_code equivalence_error;
    const bool same_emitter =
        emitter_path == candidate_path ||
        std::filesystem::equivalent(emitter_path, candidate_path,
                                    equivalence_error);
    if (same_emitter) {
        std::fprintf(
            stderr,
            "reference and candidate emitters must be different executables; "
            "'%s' and '%s' resolve to the same executable\n",
            emitter_path.c_str(), candidate_path.c_str());
        return 2;
    }

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
              std::string("state_readback_position[0]"),
              std::string("velocity_upper_link_angular[2]"),
              std::string(
                  "velocity_floating_link_translational_at_body_origin[0]"),
              std::string("relative_velocity_") +
                  scenario.relative_spatial_velocity_observations.front().name +
                  "_angular[0]",
              std::string("state_readback_velocity[0]"),
              std::string("mapped_position_derivative[3]"),
              std::string("mapped_back_generalized_velocity[3]"),
              std::string("acceleration_lower_link_angular[0]"),
              std::string(
                  "jacobian_probe_response_reverse_branch_link_"
                  "translational_at_center_of_mass[0]")}) {
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
            orvd_comparison::MakeDifferentialKinematicsComparisonRequirements(
                scenario);
        ExpectRequirementsCoverDifferentialKinematics(requirements, scenario);

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
        const auto expect_fact = [&reference](const std::string& name,
                                              int expected) {
            const auto* fact = reference.FindTopologyFact(name);
            Expect(fact != nullptr, "the reference must report " + name);
            if (fact != nullptr)
                Expect(fact->value == expected,
                       name + " is " + std::to_string(fact->value) +
                           ", but this fixture's scenario reads its state as "
                           "though it were " + std::to_string(expected));
        };
        // The whole layout, element by element. Pinning only where the free
        // body starts would let the two revolute coordinates swap places on
        // both sides at once and still pass.
        expect_fact("joint_position_range_start[shoulder_joint]", 0);
        expect_fact("joint_position_range_size[shoulder_joint]", 1);
        expect_fact("joint_velocity_range_start[shoulder_joint]", 0);
        expect_fact("joint_velocity_range_size[shoulder_joint]", 1);
        expect_fact("joint_position_range_start[elbow_joint]", 1);
        expect_fact("joint_position_range_size[elbow_joint]", 1);
        expect_fact("joint_velocity_range_start[elbow_joint]", 1);
        expect_fact("joint_velocity_range_size[elbow_joint]", 1);
        expect_fact("joint_position_range_start[reverse_branch_joint]", 2);
        expect_fact("joint_position_range_size[reverse_branch_joint]", 1);
        expect_fact("joint_velocity_range_start[reverse_branch_joint]", 2);
        expect_fact("joint_velocity_range_size[reverse_branch_joint]", 1);
        expect_fact("free_body_position_range_start[floating_link]", 3);
        expect_fact("free_body_position_range_size[floating_link]", 7);
        expect_fact("free_body_velocity_range_start[floating_link]", 3);
        expect_fact("free_body_velocity_range_size[floating_link]", 6);

        const auto* free_body_velocity_start = reference.FindTopologyFact(
            "free_body_velocity_range_start[" + scenario.free_body_names.front() +
            "]");
        Expect(free_body_velocity_start != nullptr,
               "the reference must report the free body's velocity range");
        if (free_body_velocity_start != nullptr) {
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

        // Which branch judges which quantity is part of the claim. A run that
        // never left the near-zero branch would have compared nothing
        // proportionally, and "the tolerance is 1e-8" would be a statement
        // about a number nobody applied. Checked per dimension, because the
        // crossover sits at a different magnitude for each.
        if (excitation == "dynamic_excitation") {
            const auto expect_relative = [&reference](
                                             const std::string& name,
                                             orvd_contract::ObservationKind kind) {
                const auto* observation = reference.FindObservation(name);
                Expect(observation != nullptr,
                       "the reference must report " + name);
                if (observation == nullptr) return;
                Expect(orvd_comparison::SelectToleranceBranch(observation->value,
                                                              kind) ==
                           orvd_comparison::ToleranceBranch::kRelativeError,
                       name + " must be judged proportionally, not against its "
                              "dimension's near-zero floor");
            };
            expect_relative("pose_lower_link_translation[1]",
                            orvd_contract::ObservationKind::kTranslationMeters);
            expect_relative("state_readback_position[0]",
                            orvd_contract::ObservationKind::kAngleRadians);

            // And the judge really does apply the proportional allowance to
            // them. Asking which branch a helper would choose says nothing
            // about which one the comparison used, so each named field is
            // perturbed by an amount that the two allowances answer
            // differently: comfortably inside the proportional one, and far
            // outside the dimension's near-zero floor.
            for (const std::string& named :
                 {std::string("pose_lower_link_translation[1]"),
                  std::string("state_readback_position[0]")}) {
                const auto* observation = reference.FindObservation(named);
                if (observation == nullptr) continue;
                const double proportional_step =
                    0.5 * orvd_comparison::kRelativeErrorLimit *
                    std::abs(observation->value);
                const auto judged = orvd_comparison::CompareObservationStreams(
                    requirements, reference,
                    WithScalarValueOffset(candidate, named, proportional_step));
                Expect(judged.outcome ==
                           orvd_comparison::ComparisonOutcome::kAccepted,
                       named + " must be accepted at half its proportional "
                               "allowance, which is far outside its dimension's "
                               "near-zero floor: " + judged.detail);
            }

            // Every differential-kinematics dimension has its own near-zero
            // floor. Prove that each one actually enters and uses the 1e-8
            // proportional branch in this dynamic scenario.
            for (const auto kind :
                 {orvd_contract::ObservationKind::
                      kAngularVelocityRadiansPerSecond,
                  orvd_contract::ObservationKind::
                      kTranslationalVelocityMetersPerSecond,
                  orvd_contract::ObservationKind::
                      kQuaternionDerivativePerSecond,
                  orvd_contract::ObservationKind::
                      kAngularAccelerationRadiansPerSecondSquared,
                  orvd_contract::ObservationKind::
                      kTranslationalAccelerationMetersPerSecondSquared}) {
                const orvd_comparison::RequiredObservation* target = nullptr;
                for (const auto& required : requirements.scalars) {
                    if (required.kind != kind) continue;
                    const auto* observation =
                        reference.FindObservation(required.name);
                    if (observation != nullptr &&
                        orvd_comparison::SelectToleranceBranch(
                            observation->value, kind) ==
                            orvd_comparison::ToleranceBranch::kRelativeError) {
                        target = &required;
                        break;
                    }
                }
                Expect(target != nullptr,
                       "each differential-kinematics dimension must exercise the "
                       "relative tolerance branch");
                if (target == nullptr) continue;
                const auto* observation =
                    reference.FindObservation(target->name);
                const double inside_step =
                    0.5 * orvd_comparison::kRelativeErrorLimit *
                    std::abs(observation->value);
                const double outside_step =
                    5.0 * orvd_comparison::kRelativeErrorLimit *
                    std::abs(observation->value);
                const auto inside =
                    orvd_comparison::CompareObservationStreams(
                        requirements, reference,
                        WithScalarValueOffset(candidate, target->name,
                                              inside_step));
                Expect(inside.outcome ==
                           orvd_comparison::ComparisonOutcome::kAccepted,
                       target->name +
                           " must accept half its proportional allowance: " +
                           inside.detail);
                const auto outside =
                    orvd_comparison::CompareObservationStreams(
                        requirements, reference,
                        WithScalarValueOffset(candidate, target->name,
                                              outside_step));
                Expect(
                    outside.outcome ==
                        orvd_comparison::ComparisonOutcome::kToleranceExceeded,
                    target->name +
                        " must reject five times its proportional allowance");
            }
        }

        const auto verdict = orvd_comparison::CompareObservationStreams(
            requirements, reference, candidate);
        Expect(verdict.outcome == orvd_comparison::ComparisonOutcome::kAccepted,
               std::string("the ORVD candidate must agree with the Drake "
                           "reference on differential kinematics (") +
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
        "the Drake reference on differential kinematics\n");
    return 0;
}
