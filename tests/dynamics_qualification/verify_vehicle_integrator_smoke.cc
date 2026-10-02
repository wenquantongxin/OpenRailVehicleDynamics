#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <omp.h>

#include "gz18_qualification_runner.h"
#include "irw_passive_scenario_runs.h"
#include "irw_r300_aar5_v60_100hz_full_state_guidance_run.h"
#include "qualification_run_accounting.h"

namespace {
namespace dq = orvd::dynamics_qualification;
namespace fs = std::filesystem;
using Json = nlohmann::json;
constexpr std::int64_t kStepNanoseconds = 12'500;

void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
Json ReadJson(const fs::path& path) {
    std::ifstream input(path);
    Require(static_cast<bool>(input), "cannot read " + path.string());
    return Json::parse(input);
}
void WriteJson(const fs::path& path, const Json& value) {
    std::ofstream output(path);
    output << value.dump(2) << '\n';
    output.close();
    Require(static_cast<bool>(output), "cannot write " + path.string());
}
struct Scenario { const char* name; bool irw; bool controlled; };
constexpr std::array<Scenario, 3> kScenarios{{
    {"gz18_passive", false, false}, {"irw_passive", true, false},
    {"irw_controlled", true, true}}};
struct Result { Eigen::VectorXd terminal; Json work; };

template <class Summary>
Result Capture(const Summary& summary, const std::string& method,
               std::int64_t duration, bool controlled) {
    Require(summary.integrator_recipe_identifier == method,
            "runner selected a different method");
    Require(summary.terminal_continuous_state.size() > 0 &&
                summary.terminal_continuous_state.allFinite(), "invalid terminal physical state");
    Require(summary.integration_work.statistics_complete(), "incomplete work ledger");
    const auto work = summary.integration_work.ToJson();
    const auto& statistics = summary.integration_work.total_statistics();
    const std::uint64_t epochs = controlled ? 2 : 1;
    const auto steps = static_cast<std::uint64_t>(duration / kStepNanoseconds);
    Require(work.at("epoch_count") == epochs &&
                (method == "newmark" ? statistics.successful_internal_step_count >= steps
                                     : statistics.successful_internal_step_count == steps),
            "step or successful-synchronization accounting changed");
    Require(statistics.requested_dense_finite_difference_jacobian_worker_count ==
                (method == "newmark" ? 1 : 0), "wrong mechanical worker identity");
    if (method == "newmark") {
        Require(statistics.nonlinear_solver_convergence_failure_count == 0,
                "the normal Newmark wiring smoke must not require recovery");
    }
    if (method == "zhai") {
        Require(statistics.right_hand_side_evaluation_count == steps + epochs &&
                    statistics.linear_solver_right_hand_side_evaluation_count == 0 &&
                    statistics.jacobian_evaluation_count == 0,
                "Zhai work must count each epoch initialization and endpoint once");
    }
    return {summary.terminal_continuous_state, work};
}

Result Run(const fs::path& source, const fs::path& artifacts,
           const Scenario& scenario, const std::string& method,
           const std::string& suffix = "", std::int64_t sample = 500'000,
           std::uint64_t budget = 1'000'000) {
    const std::string name = std::string(scenario.name) + "_" + method + suffix;
    const auto output = artifacts / name;
    const auto config_path = artifacts / (name + ".json");
    auto configuration = ReadJson(source / "tools/dynamics_qualification/integration_configurations" /
                                  (method + "_explicit_trial.json"));
    configuration["step_size_nanoseconds"] = kStepNanoseconds;
    configuration["maximum_internal_steps_per_advance"] = budget;
    WriteJson(config_path, configuration);
    const auto vehicle = source / "vehicle_library" / (scenario.irw ? "irw" : "gz18");
    const auto geometry = source / "track_library/geometries" /
        (scenario.irw ? "r300_centerline_superelevation_1100m.json" : "straight_level_1100m.json");
    const std::int64_t duration = scenario.controlled ? 20'000'000 : 1'000'000;
    const auto expected_samples = static_cast<std::size_t>(duration / sample + 1);
    Result result;
    if (scenario.controlled) {
        dq::IrwR300Aar5V60At100HzFullStateGuidanceRunConfiguration run;
        run.vehicle_definition_path = vehicle / "vehicle_definition.json";
        run.resolved_startup_state_path = vehicle / "startup_states/moving_startup_60kmh.json";
        run.track_geometry_path = geometry;
        run.orvd_data_root = source;
        run.controller_configuration_path = source / "controller_library/irw/irw_r300_v60_full_state_wheel_speed_guidance_controller.json";
        run.torque_conditioner_configuration_path = vehicle / "drive_torque_conditioners/irw_reference_wheel_drive_torque_conditioner.json";
        run.output_directory = output;
        run.duration_nanoseconds = duration;
        run.integration_config_path = config_path;
        const auto summary = dq::RunIrwR300Aar5V60At100HzFullStateGuidance(run);
        Require(summary.observation_count == 41 && summary.positive_hold_interval_count == 2 &&
                    summary.backend_synchronization_count == 1,
                "controlled smoke must traverse two hold intervals and one synchronization");
        result = Capture(summary, method, duration, true);
    } else if (scenario.irw) {
        dq::IrwPassiveScenarioRunConfiguration run;
        run.scenario_identifier = dq::kIrwR300Aar5V60PassiveScenarioIdentifier;
        run.vehicle_definition_path = vehicle / "vehicle_definition.json";
        run.resolved_startup_state_path = vehicle / "startup_states/moving_startup_60kmh.json";
        run.track_geometry_path = geometry;
        run.orvd_data_root = source;
        run.track_irregularity_identifier = "aar5_irregularity";
        run.output_directory = output;
        run.duration_nanoseconds = duration;
        run.sample_period_nanoseconds = sample;
        run.integration_config_path = config_path;
        const auto summary = dq::RunIrwPassiveScenario(run);
        Require(summary.sample_count == expected_samples, "IRW sample clock changed");
        result = Capture(summary, method, duration, false);
    } else {
        dq::Gz18QualificationRunConfiguration run;
        run.vehicle_definition_path = vehicle / "vehicle_definition.json";
        run.resolved_startup_state_path = vehicle / "startup_states/moving_startup_60kmh.json";
        run.track_geometry_path = geometry;
        run.orvd_data_root = source;
        run.track_irregularity_identifier = "aar6_irregularity";
        run.output_directory = output;
        run.duration_nanoseconds = duration;
        run.sample_period_nanoseconds = sample;
        run.integration_config_path = config_path;
        const auto summary = dq::RunGz18Qualification(run);
        Require(summary.sample_count == expected_samples, "GZ18 sample clock changed");
        result = Capture(summary, method, duration, false);
    }
    for (const char* file : {"COMPLETE", "metadata.json", "performance.json", "continuous_states.tsv",
                             "observations.tsv", "contact_patches.tsv"}) {
        Require(fs::is_regular_file(output / file), "missing published file " + std::string(file));
    }
    Require(!fs::exists(dq::FailureResultPath(output)), "successful run also published failure");
    const auto metadata = ReadJson(output / "metadata.json");
    const auto& contract = metadata.at("numerical_execution_contract");
    Require(contract.at("integrator_recipe_identifier") == method &&
                contract.at("step_size_nanoseconds") == kStepNanoseconds,
            "publication does not describe the requested method and step");
    std::ifstream states(output / "continuous_states.tsv");
    std::string line;
    std::getline(states, line);
    std::size_t rows = 0;
    while (std::getline(states, line)) ++rows;
    Require(rows == expected_samples, "state file lost or duplicated a sample");
    return result;
}

void CheckBudgetFailure(const fs::path& source, const fs::path& artifacts) {
    const auto output = artifacts / "gz18_passive_zhai_budget";
    bool failed = false;
    try { (void)Run(source, artifacts, kScenarios[0], "zhai", "_budget", 500'000, 1); }
    catch (const std::exception&) { failed = true; }
    Require(failed && !fs::exists(output), "budget failure published a success directory");
    const auto failure = ReadJson(dq::FailureResultPath(output));
    Require(failure.at("numerical_failure_reason") == "advance_work_budget_exhausted" &&
                failure.at("last_public_accepted_time_seconds") == 0.0 &&
                failure.at("work").at("availability") == "complete" &&
                failure.at("work").at("integration_statistics").at("successful_internal_step_count") == 1,
            "failure publication lost classification, rollback endpoint or performed work");
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    try {
        omp_set_dynamic(0);
        omp_set_num_threads(1);
        const auto source = fs::absolute(argv[1]);
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        const auto artifacts = fs::absolute(argv[2]) / ("run_" + std::to_string(stamp));
        Require(fs::create_directories(artifacts), "cannot allocate smoke output directory");
        Result sparse;
        for (const auto& scenario : kScenarios) {
            for (const std::string method : {"newmark", "zhai"}) {
                const auto result = Run(source, artifacts, scenario, method);
                if (&scenario == &kScenarios[0] && method == "zhai") sparse = result;
            }
        }
        const auto dense = Run(source, artifacts, kScenarios[0], "zhai", "_dense", 31'250);
        Require(sparse.terminal == dense.terminal && sparse.work == dense.work,
                "additional interior samples altered integration state or work");
        CheckBudgetFailure(source, artifacts);
        std::puts("vehicle integrator wiring smoke passed");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle integrator wiring smoke: %s\n", error.what());
        return 1;
    }
    return 0;
}
