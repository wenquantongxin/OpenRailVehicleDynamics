// Launches two independent reference processes, compares their observations
// through pipes, and proves the comparison can actually fail.
//
// What this establishes today is that the harness works: the same reference
// implementation, run twice in separate processes, agrees with itself, and a
// deliberate perturbation is caught. It says nothing about any ORVD candidate,
// because there is no candidate yet — the first one arrives with the vendored
// topology. Claiming otherwise here would be claiming a result we have not
// measured.
//
// Nothing is written to disk: the streams exist only in the pipe and in memory.
#include <cerrno>
#include <cstdio>
#include <string>
#include <string_view>

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

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <reference_emitter_path>\n", argv[0]);
        return 2;
    }
    const std::string emitter_path = argv[1];

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
        // rather than quietly skipped.
        orvd_contract::ObservationStream truncated = second;
        truncated.observations.erase(truncated.observations.begin());
        const auto missing = orvd_comparison::CompareObservationStreams(
            requirements, first, truncated);
        Expect(missing.outcome ==
                   orvd_comparison::ComparisonOutcome::kRequiredObservationMissing,
               "a missing required observation must be reported");
    }

    if (failure_count > 0) {
        std::fprintf(stderr, "%d comparison harness check(s) failed\n", failure_count);
        return 1;
    }
    std::printf("cross-process observation comparison harness verified\n");
    return 0;
}
