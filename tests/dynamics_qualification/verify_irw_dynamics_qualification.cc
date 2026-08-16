// The closed IRW runner keeps G70/G71 explicitly free of track irregularity
// and accepts the current common AAR5 asset through the closed AAR5 recipe.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <omp.h>

#include "irw_qualification_runner.h"

namespace {

using orvd::dynamics_qualification::IrwQualificationRunConfiguration;
using orvd::dynamics_qualification::
    IrwP179ControlledQualificationRunConfiguration;
using orvd::dynamics_qualification::RunIrwQualification;
using orvd::dynamics_qualification::RunIrwP179ControlledQualification;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "IRW qualification: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

template <class Function>
bool Throws(Function&& function) {
    try {
        function();
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

std::string ReadWholeFile(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open " + path.string());
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

std::vector<std::string> SplitTabs(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t begin = 0;
    while (true) {
        const std::size_t end = line.find('\t', begin);
        fields.push_back(line.substr(begin, end - begin));
        if (end == std::string::npos) {
            return fields;
        }
        begin = end + 1;
    }
}

double ParseFiniteDouble(const std::string& text) {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value)) {
        throw std::runtime_error("invalid finite number in IRW observation");
    }
    return value;
}

bool SameTrajectoryWork(
    const orvd::integrators::ContinuousStateIntegrationStatistics& left,
    const orvd::integrators::ContinuousStateIntegrationStatistics& right) {
    return left.successful_internal_step_count ==
               right.successful_internal_step_count &&
           left.right_hand_side_evaluation_count ==
               right.right_hand_side_evaluation_count &&
           left.linear_solver_right_hand_side_evaluation_count ==
               right.linear_solver_right_hand_side_evaluation_count &&
           left.error_test_failure_count == right.error_test_failure_count &&
           left.nonlinear_solver_iteration_count ==
               right.nonlinear_solver_iteration_count &&
           left.nonlinear_solver_convergence_failure_count ==
               right.nonlinear_solver_convergence_failure_count &&
           left.linear_solver_setup_count == right.linear_solver_setup_count &&
           left.jacobian_evaluation_count == right.jacobian_evaluation_count;
}

template <class Summary>
bool SameTerminalState(const Summary& left, const Summary& right) {
    return left.terminal_continuous_state.size() ==
               right.terminal_continuous_state.size() &&
           (left.terminal_continuous_state.array() ==
            right.terminal_continuous_state.array())
               .all();
}

std::size_t FindColumn(const std::vector<std::string>& header,
                       std::string_view name) {
    for (std::size_t column = 0; column < header.size(); ++column) {
        if (header[column] == name) {
            return column;
        }
    }
    throw std::runtime_error("missing IRW observation column " +
                             std::string(name));
}

void CheckRealIrwRun(char** argv, const std::filesystem::path& root) {
    IrwQualificationRunConfiguration configuration;
    configuration.vehicle_definition_path = std::filesystem::relative(argv[1]);
    configuration.resolved_startup_state_path =
        std::filesystem::relative(argv[2]);
    configuration.track_geometry_path = std::filesystem::relative(argv[3]);
    configuration.orvd_data_root = std::filesystem::relative(argv[4]);
    configuration.output_directory = root / "real-irw";
    configuration.duration_nanoseconds = 10'000'000;
    configuration.sample_period_nanoseconds = 100'000;

    const auto summary = RunIrwQualification(configuration);
    Require(summary.sample_count == 101,
            "the 10 ms / 100 us clock did not publish 101 points");
    Require(summary.integration_statistics.successful_internal_step_count > 0 &&
                summary.integration_statistics.right_hand_side_evaluation_count +
                        summary.integration_statistics
                            .linear_solver_right_hand_side_evaluation_count >
                    0,
            "the real IRW advance did no recorded numerical work");
    Require(summary.endpoint_generalized_force_residual_inf_norm < 1.0e-7,
            "the endpoint generalized-force assembly does not close");
    Require(std::abs(summary.endpoint_virtual_power_residual_watts) < 1.0e-7,
            "the endpoint wrench/generalized-force virtual power does not close");
    Require(summary.endpoint_position_derivative_slice_consistency_inf_norm <
                    1.0e-14 &&
                summary
                        .endpoint_series_force_derivative_slice_consistency_inf_norm <
                    1.0e-14,
            "the endpoint qdot or two-Maxwell derivative slice is inconsistent");
    Require(summary.used_before_track_definition_interval &&
                !summary.used_after_track_definition_interval,
            "the H3 short window did not retain its expected left continuation");

    const std::array<std::string_view, 5> files{
        "COMPLETE", "metadata.json", "observations.tsv",
        "contact_patches.tsv", "performance.json"};
    for (const std::string_view file : files) {
        Require(std::filesystem::is_regular_file(
                    configuration.output_directory / file),
                "the atomic IRW artifact is incomplete");
    }
    Require(!std::filesystem::exists(root / "real-irw.partial"),
            "the successful IRW publication left a partial directory");

    std::ifstream input(configuration.output_directory / "observations.tsv");
    std::string line;
    std::getline(input, line);
    const std::vector<std::string> header = SplitTabs(line);
    std::vector<std::vector<std::string>> rows;
    while (std::getline(input, line)) {
        rows.push_back(SplitTabs(line));
    }
    Require(rows.size() == 101,
            "the IRW observation table does not contain 101 rows");
    if (rows.size() != 101) {
        return;
    }
    for (const auto& row : rows) {
        Require(row.size() == header.size(),
                "an IRW observation row has the wrong fixed width");
        for (const std::string& field : row) {
            (void)ParseFiniteDouble(field);
        }
    }

    const std::size_t time_column = FindColumn(header, "time_seconds");
    for (std::size_t sample = 0; sample < rows.size(); ++sample) {
        Require(ParseFiniteDouble(rows[sample][time_column]) ==
                    static_cast<double>(sample) * 0.0001,
                "the IRW output differs from its integer 100 us clock");
    }
    std::size_t simpack_native_point_count = 0;
    for (std::size_t sample = 0; sample < rows.size(); sample += 5) {
        // Selection is by the shared integer sample ordinal. Recomputing the
        // same time through a second floating multiplication is not an
        // additional physical contract.
        (void)ParseFiniteDouble(rows[sample][time_column]);
        ++simpack_native_point_count;
    }
    Require(simpack_native_point_count == 21,
            "the 10 ms artifact does not contain 21 native SIMPACK indices");

    constexpr std::array<std::string_view, 4> kCarrierNames{
        "axlebridge_ff", "axlebridge_fr", "axlebridge_rf", "axlebridge_rr"};
    constexpr std::array<double, 4> kInitialStations{
        10.000008398803899, 7.5000084046506323,
        -7.4999916011961005, -9.9999915953493677};
    for (std::size_t carrier = 0; carrier < kCarrierNames.size(); ++carrier) {
        const std::size_t station = FindColumn(
            header, std::string(kCarrierNames[carrier]) +
                        ".track_station_meters");
        const std::size_t lateral = FindColumn(
            header, std::string(kCarrierNames[carrier]) + ".lateral_meters");
        const std::size_t yaw = FindColumn(
            header, std::string(kCarrierNames[carrier]) + ".yaw_radians");
        Require(ParseFiniteDouble(rows.front()[station]) ==
                    kInitialStations[carrier] &&
                    ParseFiniteDouble(rows.back()[station]) >
                        ParseFiniteDouble(rows.front()[station]),
                "an H3 axle-bridge station is shifted or does not advance");
        (void)ParseFiniteDouble(rows.back()[lateral]);
        (void)ParseFiniteDouble(rows.back()[yaw]);
    }

    constexpr std::array<std::string_view, 8> kInterfaceNames{
        "wheel_ff_l", "wheel_ff_r", "wheel_fr_l", "wheel_fr_r",
        "wheel_rf_l", "wheel_rf_r", "wheel_rr_l", "wheel_rr_r"};
    for (const std::string_view interface_name : kInterfaceNames) {
        const std::size_t patch_count = FindColumn(
            header, std::string(interface_name) + ".contact_patch_count");
        const std::size_t q = FindColumn(
            header, std::string(interface_name) +
                        ".vertical_support_force_on_wheel_newtons");
        Require(ParseFiniteDouble(rows.front()[patch_count]) >= 1.0 &&
                    ParseFiniteDouble(rows.back()[patch_count]) >= 1.0 &&
                    ParseFiniteDouble(rows.front()[q]) > 0.0 &&
                    ParseFiniteDouble(rows.back()[q]) > 0.0,
                "an independently rotating wheel lost its real short-window contact");
    }
    for (const std::string_view body_name :
         {std::string_view("carbody"), std::string_view("frame_front"),
          std::string_view("frame_rear")}) {
        (void)FindColumn(header,
                         std::string(body_name) + ".track_station_meters");
    }

    const std::string metadata =
        ReadWholeFile(configuration.output_directory / "metadata.json");
    Require(metadata.find("\"qualification_vehicle_recipe\": \"IRW\"") !=
                    std::string::npos &&
                metadata.find("\"track_irregularity_identifier\": null") !=
                    std::string::npos,
            "the IRW recipe or explicit no-irregularity identity is wrong");
    Require(metadata.find("\"relative_tolerance\": 9.9999999999999995e-07") !=
                std::string::npos &&
                metadata.find(
                    "\"generalized_position_absolute_tolerance\": 9.9999999999999995e-07") !=
                    std::string::npos &&
                metadata.find(
                    "\"generalized_velocity_absolute_tolerance\": 1.0000000000000001e-05") !=
                    std::string::npos &&
                metadata.find(
                    "\"series_force_absolute_tolerance_newtons\": 9.9999999999999995e-07") !=
                    std::string::npos,
            "the frozen IRW A-layer tolerance identity is absent");
    Require(metadata.find("\"generalized_position_count\": 81") !=
                    std::string::npos &&
                metadata.find("\"generalized_velocity_count\": 74") !=
                    std::string::npos &&
                metadata.find("\"series_force_state_count\": 2") !=
                    std::string::npos &&
                metadata.find("\"vehicle_body_wrench_count\": 96") !=
                    std::string::npos &&
                metadata.find("\"contact_body_wrench_count\": 8") !=
                    std::string::npos,
            "the passive q81/v74/z2 and 96+8 wrench identity is absent");
    Require(metadata.find(std::filesystem::canonical(argv[1]).string()) !=
                    std::string::npos &&
                metadata.find(std::filesystem::canonical(argv[2]).string()) !=
                    std::string::npos &&
                metadata.find(std::filesystem::canonical(argv[3]).string()) !=
                    std::string::npos &&
                metadata.find(std::filesystem::canonical(argv[4]).string()) !=
                    std::string::npos,
            "the IRW artifact lacks canonical physical input paths");

    const std::string patches =
        ReadWholeFile(configuration.output_directory / "contact_patches.tsv");
    Require(patches.find("wheel_ff_l") != std::string::npos &&
                patches.find("wheel_rr_r") != std::string::npos,
            "the per-patch artifact omits a frozen IRW interface");
    Require(Throws([&] { (void)RunIrwQualification(configuration); }),
            "the IRW runner overwrote an existing successful artifact");

    configuration.output_directory = root / "real-irw-aar5";
    configuration.duration_nanoseconds = 100'000;
    configuration.sample_period_nanoseconds = 100'000;
    configuration.track_irregularity_identifier =
        "aar5_irregularity";
    const auto aar5_summary = RunIrwQualification(configuration);
    Require(aar5_summary.sample_count == 2,
            "the short AAR5 loading run did not publish its two clock points");
    const std::string aar5_metadata =
        ReadWholeFile(configuration.output_directory / "metadata.json");
    Require(aar5_metadata.find(
                "\"track_irregularity_identifier\": "
                "\"aar5_irregularity\"") !=
                std::string::npos,
            "the IRW AAR5 identity did not reach the qualification artifact");
    Require(aar5_metadata.find(
                "\"relative_tolerance\": 9.9999999999999995e-08") !=
                    std::string::npos &&
                aar5_metadata.find(
                    "\"generalized_position_absolute_tolerance\": "
                    "9.9999999999999995e-08") != std::string::npos &&
                aar5_metadata.find(
                    "\"generalized_velocity_absolute_tolerance\": "
                    "9.9999999999999995e-07") != std::string::npos &&
                aar5_metadata.find(
                    "\"series_force_absolute_tolerance_newtons\": "
                    "1.0000000000000001e-05") != std::string::npos &&
                aar5_metadata.find("\"local_sample_refinement\": null") !=
                    std::string::npos,
            "the frozen IRW B-layer tolerance or uniform-clock identity is "
            "absent");
    configuration.output_directory = root / "empty-irregularity";
    configuration.track_irregularity_identifier = "";
    Require(Throws([&] { (void)RunIrwQualification(configuration); }),
            "the required IRW AAR5 recipe accepted an empty identity");
    configuration.output_directory = root / "unsupported-irregularity";
    configuration.track_irregularity_identifier = "aar6_irregularity";
    Require(Throws([&] { (void)RunIrwQualification(configuration); }),
            "the IRW AAR5 recipe accepted a different irregularity field");
}

void CheckPassiveDenseJacobianThreading(
    char** argv, const std::filesystem::path& root) {
    const int original_openmp_dynamic = omp_get_dynamic();
    const int original_openmp_max_threads = omp_get_max_threads();
    omp_set_dynamic(0);

    IrwQualificationRunConfiguration configuration;
    configuration.vehicle_definition_path = std::filesystem::relative(argv[1]);
    configuration.resolved_startup_state_path =
        std::filesystem::relative(argv[2]);
    configuration.track_geometry_path = std::filesystem::relative(argv[3]);
    configuration.orvd_data_root = std::filesystem::relative(argv[4]);
    configuration.track_irregularity_identifier =
        "aar5_irregularity";
    configuration.duration_nanoseconds = 30'000'000;
    configuration.sample_period_nanoseconds = 30'000'000;

    std::optional<orvd::dynamics_qualification::QualificationRunSummary>
        reference;
    std::string reference_observations;
    std::string reference_patches;
    for (const int requested_threads : {1, 4, 8, 16}) {
        omp_set_num_threads(requested_threads);
        configuration.output_directory =
            root / ("irw-aar5-jacobian-t" +
                    std::to_string(requested_threads));
        const auto candidate = RunIrwQualification(configuration);
        Require(candidate.integration_statistics.jacobian_evaluation_count >
                        1 &&
                    candidate.integration_statistics
                            .requested_dense_finite_difference_jacobian_worker_count ==
                        requested_threads &&
                    candidate.terminal_continuous_state.size() == 157,
                "the passive IRW 1/4/8/16-thread run did not exercise the "
                "requested 157-state dense Jacobian");
        const std::string observations = ReadWholeFile(
            configuration.output_directory / "observations.tsv");
        const std::string patches = ReadWholeFile(
            configuration.output_directory / "contact_patches.tsv");
        if (!reference.has_value()) {
            reference = candidate;
            reference_observations = observations;
            reference_patches = patches;
        } else {
            Require(SameTrajectoryWork(reference->integration_statistics,
                                       candidate.integration_statistics) &&
                        SameTerminalState(*reference, candidate) &&
                        observations == reference_observations &&
                        patches == reference_patches,
                    "the passive IRW 1/4/8/16-thread dense Jacobian changed "
                    "the trajectory, numerical work or contact artifacts");
        }
    }
    omp_set_num_threads(original_openmp_max_threads);
    omp_set_dynamic(original_openmp_dynamic);
}

void CheckControlledIrwRun(char** argv, const std::filesystem::path& root) {
    const int original_openmp_dynamic = omp_get_dynamic();
    const int original_openmp_max_threads = omp_get_max_threads();
    omp_set_dynamic(0);
    omp_set_num_threads(8);
    IrwP179ControlledQualificationRunConfiguration configuration;
    configuration.vehicle_definition_path = argv[1];
    configuration.resolved_startup_state_path = argv[2];
    configuration.track_geometry_path = argv[3];
    configuration.orvd_data_root = argv[4];
    configuration.controller_configuration_path = argv[5];
    configuration.torque_conditioner_configuration_path = argv[6];
    configuration.output_directory = root / "controlled-irw";
    configuration.duration_nanoseconds = 20'000'000;

    const auto summary = RunIrwP179ControlledQualification(configuration);
    Require(summary.observation_count == 41 &&
                summary.control_audit_count == 4 &&
                summary.positive_hold_interval_count == 2 &&
                summary.backend_synchronization_count == 1 &&
                summary.integration_statistics.jacobian_evaluation_count > 1 &&
                summary.integration_statistics
                        .requested_dense_finite_difference_jacobian_worker_count ==
                    8 &&
                summary.terminal_continuous_state.size() == 157,
            "the 20 ms control recipe did not produce initialization, "
            "U0..U2, two holds and one nonterminal synchronization");
    Require(summary.integration_statistics.successful_internal_step_count > 0 &&
                summary.integration_statistics
                        .right_hand_side_evaluation_count > 0 &&
                std::isfinite(
                    summary.maximum_generalized_force_residual_inf_norm) &&
                std::isfinite(
                    summary.maximum_absolute_virtual_power_residual_watts),
            "the real controlled IRW window did no finite numerical work");

    constexpr std::array<std::string_view, 7> kFiles{
        "COMPLETE", "metadata.json", "observations.tsv",
        "contact_patches.tsv", "control_events.tsv",
        "endpoint_diagnostics.tsv", "performance.json"};
    for (const std::string_view file : kFiles) {
        Require(std::filesystem::is_regular_file(
                    configuration.output_directory / file),
                "the controlled IRW atomic artifact is incomplete");
    }
    Require(!std::filesystem::exists(root / "controlled-irw.partial"),
            "the controlled IRW run left a partial directory");

    std::ifstream control(configuration.output_directory /
                          "control_events.tsv");
    std::string line;
    std::vector<std::vector<std::string>> control_rows;
    while (std::getline(control, line)) {
        control_rows.push_back(SplitTabs(line));
    }
    Require(control_rows.size() == 5,
            "the control audit is not one header plus four recurrences");
    if (control_rows.size() == 5) {
        const auto& header = control_rows.front();
        const std::size_t kind = FindColumn(header, "event_kind");
        const std::size_t ordinal =
            FindColumn(header, "periodic_event_ordinal");
        const std::size_t time = FindColumn(header, "event_time_seconds");
        const std::size_t actual_torque =
            FindColumn(header, "conditioner.actual_torque.ff_l");
        Require(control_rows[1][kind] == "initialization" &&
                    control_rows[2][kind] == "periodic" &&
                    ParseFiniteDouble(control_rows[1][ordinal]) == 0.0 &&
                    ParseFiniteDouble(control_rows[2][ordinal]) == 0.0 &&
                    ParseFiniteDouble(control_rows[3][ordinal]) == 1.0 &&
                    ParseFiniteDouble(control_rows[4][ordinal]) == 2.0 &&
                    ParseFiniteDouble(control_rows[1][time]) == 0.0 &&
                    ParseFiniteDouble(control_rows[2][time]) == 0.0 &&
                    ParseFiniteDouble(control_rows[3][time]) == 0.01 &&
                    ParseFiniteDouble(control_rows[4][time]) == 0.02,
                "the control audit does not use the startup double update "
                "and integer event grid");
        Require(control_rows[2][actual_torque] !=
                    control_rows[3][actual_torque],
                "U0 and U1 did not change the held wheel torque");
        for (std::size_t row = 1; row < control_rows.size(); ++row) {
            Require(control_rows[row].size() == header.size(),
                    "a control audit row has the wrong fixed width");
        }
    }

    std::ifstream observations(configuration.output_directory /
                               "observations.tsv");
    std::vector<std::vector<std::string>> observation_rows;
    while (std::getline(observations, line)) {
        observation_rows.push_back(SplitTabs(line));
    }
    Require(observation_rows.size() == 42,
            "the controlled observation table is not the 0.5 ms clock plus "
            "one header");
    if (observation_rows.size() == 42) {
        const auto& header = observation_rows.front();
        const std::size_t sample_index = FindColumn(header, "sample_index");
        const std::size_t time_nanoseconds =
            FindColumn(header, "time_nanoseconds");
        const std::size_t time = FindColumn(header, "time_seconds");
        for (std::size_t row = 1; row < observation_rows.size(); ++row) {
            const double expected_index = static_cast<double>(row - 1U);
            const double expected_nanoseconds = expected_index * 500'000.0;
            const double expected_seconds =
                (row - 1U) % 20U == 0U
                    ? static_cast<double>((row - 1U) / 20U) * 0.01
                    : expected_index * 0.0005;
            Require(ParseFiniteDouble(observation_rows[row][sample_index]) ==
                            expected_index &&
                        ParseFiniteDouble(
                            observation_rows[row][time_nanoseconds]) ==
                            expected_nanoseconds &&
                        ParseFiniteDouble(observation_rows[row][time]) ==
                            expected_seconds,
                    "the controlled mechanical observations are not on the "
                    "integer 0.5 ms clock");
        }
    }

    std::ifstream contact_patches(configuration.output_directory /
                                  "contact_patches.tsv");
    std::vector<std::vector<std::string>> patch_rows;
    while (std::getline(contact_patches, line)) {
        patch_rows.push_back(SplitTabs(line));
    }
    Require(patch_rows.size() > 1,
            "the controlled contact-patch table contains no real patch");
    if (patch_rows.size() > 1) {
        const auto& header = patch_rows.front();
        const std::size_t sample_index = FindColumn(header, "sample_index");
        const std::size_t interface_name =
            FindColumn(header, "interface_name");
        const std::size_t normal =
            FindColumn(header, "normal_force_newtons");
        const std::size_t longitudinal = FindColumn(
            header,
            "longitudinal_force_on_wheel_in_contact_frame_newtons");
        const std::size_t lateral = FindColumn(
            header, "lateral_force_on_wheel_in_contact_frame_newtons");
        std::array<bool, 41> observed_samples{};
        for (std::size_t row = 1; row < patch_rows.size(); ++row) {
            const double sample =
                ParseFiniteDouble(patch_rows[row][sample_index]);
            if (sample >= 0.0 && sample <= 40.0 &&
                std::floor(sample) == sample) {
                observed_samples[static_cast<std::size_t>(sample)] = true;
            }
            Require(!patch_rows[row][interface_name].empty() &&
                        std::isfinite(
                            ParseFiniteDouble(patch_rows[row][normal])) &&
                        std::isfinite(ParseFiniteDouble(
                            patch_rows[row][longitudinal])) &&
                        std::isfinite(
                            ParseFiniteDouble(patch_rows[row][lateral])),
                    "a controlled patch row lost its interface or local "
                    "N/Tx/Ty values");
        }
        Require(std::ranges::all_of(observed_samples,
                                    [](bool observed) { return observed; }),
                "the real 20 ms controlled window lacks a patch observation "
                "at one or more 0.5 ms samples");
    }

    std::ifstream endpoint_diagnostics(
        configuration.output_directory / "endpoint_diagnostics.tsv");
    std::vector<std::vector<std::string>> endpoint_rows;
    while (std::getline(endpoint_diagnostics, line)) {
        endpoint_rows.push_back(SplitTabs(line));
    }
    Require(endpoint_rows.size() == 4,
            "the endpoint diagnostic table is not t0 plus two arriving "
            "control boundaries");
    if (endpoint_rows.size() == 4) {
        const auto& header = endpoint_rows.front();
        const std::size_t hold =
            FindColumn(header, "held_torque_event_ordinal");
        const std::size_t time = FindColumn(header, "time_seconds");
        Require(ParseFiniteDouble(endpoint_rows[1][hold]) == 0.0 &&
                    ParseFiniteDouble(endpoint_rows[2][hold]) == 0.0 &&
                    ParseFiniteDouble(endpoint_rows[3][hold]) == 1.0 &&
                    ParseFiniteDouble(endpoint_rows[1][time]) == 0.0 &&
                    ParseFiniteDouble(endpoint_rows[2][time]) == 0.01 &&
                    ParseFiniteDouble(endpoint_rows[3][time]) == 0.02,
                "an endpoint diagnostic was not taken under the preceding "
                "zero-order hold before the new event commit");
    }

    const std::string metadata =
        ReadWholeFile(configuration.output_directory / "metadata.json");
    Require(metadata.find("\"qualification_vehicle_recipe\": "
                          "\"IRW_P179_100HZ_CONTROLLED\"") !=
                    std::string::npos &&
                metadata.find("\"track_irregularity_identifier\": "
                              "\"aar5_irregularity\"") !=
                    std::string::npos &&
                metadata.find("\"numerical_tolerances\": {\"relative\": "
                              "9.9999999999999995e-07, \"q_absolute\": "
                              "9.9999999999999995e-07, \"v_absolute\": "
                              "1.0000000000000001e-05, "
                              "\"z_absolute_newtons\": "
                              "9.9999999999999995e-07}") !=
                    std::string::npos &&
                metadata.find("\"control_audit_count\": 4") !=
                    std::string::npos &&
                metadata.find("\"backend_synchronization_count\": 1") !=
                    std::string::npos &&
                metadata.find(
                    "\"mechanical_observation_period_nanoseconds\": "
                    "500000") != std::string::npos,
            "the controlled artifact lacks its event transaction identity");

    const std::array<std::string_view, 4> physical_files{
        "observations.tsv", "contact_patches.tsv", "control_events.tsv",
        "endpoint_diagnostics.tsv"};
    for (const int requested_threads : {1, 4, 16}) {
        omp_set_num_threads(requested_threads);
        auto comparison = configuration;
        comparison.output_directory =
            root / ("controlled-irw-t" +
                    std::to_string(requested_threads));
        const auto candidate =
            RunIrwP179ControlledQualification(comparison);
        Require(candidate.integration_statistics
                        .requested_dense_finite_difference_jacobian_worker_count ==
                    requested_threads &&
                    SameTrajectoryWork(summary.integration_statistics,
                                       candidate.integration_statistics) &&
                    SameTerminalState(summary, candidate),
                "the controlled IRW 1/4/8/16-thread dense Jacobian changed "
                "the complete state, event history or numerical work");
        for (const std::string_view file : physical_files) {
            Require(ReadWholeFile(configuration.output_directory / file) ==
                        ReadWholeFile(comparison.output_directory / file),
                    "the controlled IRW 1/4/8/16-thread dense Jacobian changed "
                    "a physical or control artifact");
        }
    }
    omp_set_num_threads(original_openmp_max_threads);
    omp_set_dynamic(original_openmp_dynamic);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 8) {
        std::fprintf(stderr,
                     "usage: verify_irw_dynamics_qualification VEHICLE "
                     "STARTUP LINE DATA_ROOT CONTROLLER CONDITIONER "
                     "TEST_ROOT\n");
        return 2;
    }
    const std::filesystem::path root = argv[7];
    if (root.filename() != "irw-dynamics-qualification-fixtures") {
        std::fprintf(stderr, "refusing an unexpected IRW fixture directory\n");
        return 2;
    }
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);

    try {
        CheckRealIrwRun(argv, root);
        CheckPassiveDenseJacobianThreading(argv, root);
        CheckControlledIrwRun(argv, root);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW qualification threw: %s\n", error.what());
        ++failures;
    }
    std::filesystem::remove_all(root);
    if (failures != 0) {
        std::fprintf(stderr, "%d IRW qualification assertion(s) failed\n",
                     failures);
        return 1;
    }
    std::puts("IRW no-irregularity and common-AAR5 recipe wiring verified");
    return 0;
}
