#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <omp.h>

#include "atomic_qualification_directory.h"
#include "gz18_qualification_runner.h"
#include "qualification_sample_clock.h"
#include "station_series_projection.h"

namespace {

using orvd::dynamics_qualification::AtomicQualificationDirectory;
using orvd::dynamics_qualification::Gz18QualificationRunConfiguration;
using orvd::dynamics_qualification::QualificationRunSummary;
using orvd::dynamics_qualification::ProjectMonotoneSeriesToStationGrid;
using orvd::dynamics_qualification::QualificationSampleClock;
using orvd::dynamics_qualification::QualificationSampleRefinement;
using orvd::dynamics_qualification::RunGz18Qualification;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "G58 qualification: %.*s\n",
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

double ParseDouble(const std::string& text) {
    std::size_t consumed = 0;
    const double result = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(result)) {
        throw std::runtime_error("invalid finite number in observation file");
    }
    return result;
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

bool SameTerminalState(const QualificationRunSummary& left,
                       const QualificationRunSummary& right) {
    return left.terminal_continuous_state.size() ==
               right.terminal_continuous_state.size() &&
           (left.terminal_continuous_state.array() ==
            right.terminal_continuous_state.array())
               .all();
}

std::size_t FindColumn(const std::vector<std::string>& header,
                       std::string_view name) {
    for (std::size_t index = 0; index < header.size(); ++index) {
        if (header[index] == name) {
            return index;
        }
    }
    throw std::runtime_error("missing observation column " +
                             std::string(name));
}

void CheckIntegerClock() {
    const QualificationSampleClock ten_seconds(10'000'000'000ULL,
                                                500'000ULL);
    const QualificationSampleClock twenty_seconds(20'000'000'000ULL,
                                                   500'000ULL);
    Require(ten_seconds.sample_count() == 20'001 &&
                twenty_seconds.sample_count() == 40'001,
            "the 0.5 ms integer clock has the wrong 10/20 s sample count");
    const auto times = ten_seconds.MakeSampleTimesSeconds();
    Require(times.front() == 0.0 && times.back() == 10.0,
            "the integer clock does not retain its unique endpoints");
    Require(times[19'999] == static_cast<double>(19'999) * 0.0005,
            "the integer-index clock differs from the WRL multiplication "
            "contract");
    Require(Throws([] {
                (void)QualificationSampleClock(1'000'001ULL, 500'000ULL);
            }),
            "a terminal time not divisible by the sample period was accepted");

    const QualificationSampleClock irw_a_layer(
        30'000'000'000ULL, 500'000ULL,
        QualificationSampleRefinement{
            3'640'000'000ULL, 3'680'000'000ULL, 100'000ULL});
    const auto irw_times = irw_a_layer.MakeSampleTimesSeconds();
    Require(irw_a_layer.sample_count() == 60'321 &&
                irw_times.front() == 0.0 && irw_times.back() == 30.0,
            "the G71 base/refined integer-clock union is not 60321 points");
    Require(irw_a_layer.TargetTimeNanoseconds(7'280) ==
                    3'640'000'000ULL &&
                irw_a_layer.TargetTimeNanoseconds(7'281) ==
                    3'640'100'000ULL &&
                irw_a_layer.TargetTimeNanoseconds(7'680) ==
                    3'680'000'000ULL &&
                irw_a_layer.TargetTimeNanoseconds(7'681) ==
                    3'680'500'000ULL,
            "the G71 refined window is not merged by integer nanosecond "
            "identity");
    Require(Throws([] {
                (void)QualificationSampleClock(
                    30'000'000'000ULL, 500'000ULL,
                    QualificationSampleRefinement{
                        3'640'000'000ULL, 3'680'000'001ULL, 100'000ULL});
            }),
            "a local-refinement interval not divisible by its period was "
            "accepted");
}

void CheckStationProjection() {
    const std::array<double, 4> source_station{0.0, 0.008, 0.017, 0.026};
    const std::array<double, 4> source_value{0.0, 2.0e-6, -1.0e-6,
                                             5.0e-6};
    const std::array<double, 3> target_station{0.004, 0.0125, 0.0215};
    const auto projected = ProjectMonotoneSeriesToStationGrid(
        source_station, source_value, target_station);
    Require(projected.size() == 3 && projected[0] == 1.0e-6 &&
                std::abs(projected[1] - 0.5e-6) <= 1.0e-20 &&
                std::abs(projected[2] - 2.0e-6) <= 1.0e-20,
            "the adjacent-sample station projection is incorrect");
    const std::array<double, 1> uncovered{0.027};
    Require(Throws([&] {
                (void)ProjectMonotoneSeriesToStationGrid(
                    source_station, source_value, uncovered);
            }),
            "station projection silently extrapolated beyond its source");
}

void CheckAtomicDirectory(const std::filesystem::path& root) {
    const std::filesystem::path abandoned = root / "abandoned";
    {
        AtomicQualificationDirectory transaction(abandoned);
        std::ofstream partial(transaction.working_path() / "one-row.tsv");
        partial << "one row\n";
    }
    Require(!std::filesystem::exists(abandoned) &&
                !std::filesystem::exists(root / "abandoned.partial"),
            "an abandoned qualification directory was not cleaned");

    const std::filesystem::path successful = root / "prior-success";
    {
        AtomicQualificationDirectory transaction(successful);
        std::ofstream sentinel(transaction.working_path() / "sentinel.txt");
        sentinel << "prior success\n";
        sentinel.close();
        transaction.Publish();
    }
    Require(Throws([&] {
                (void)AtomicQualificationDirectory(successful);
            }) &&
                ReadWholeFile(successful / "sentinel.txt") ==
                    "prior success\n",
            "a repeated publication did not preserve the prior success");
}

void CheckRealGz18Run(char** argv, const std::filesystem::path& root) {
    const int original_openmp_dynamic = omp_get_dynamic();
    const int original_openmp_max_threads = omp_get_max_threads();
    omp_set_dynamic(0);
    omp_set_num_threads(8);
    Gz18QualificationRunConfiguration configuration;
    configuration.vehicle_definition_path =
        std::filesystem::relative(argv[1]);
    configuration.resolved_startup_state_path =
        std::filesystem::relative(argv[2]);
    configuration.track_geometry_path = std::filesystem::relative(argv[3]);
    configuration.orvd_data_root = std::filesystem::relative(argv[4]);
    configuration.track_irregularity_identifier = argv[5];
    configuration.output_directory = root / "real-gz18";
    configuration.duration_nanoseconds = 30'000'000;
    configuration.sample_period_nanoseconds = 30'000'000;

    const auto summary = RunGz18Qualification(configuration);
    Require(summary.sample_count == 2 &&
                summary.integration_statistics
                        .successful_internal_step_count > 0 &&
                summary.integration_statistics
                            .right_hand_side_evaluation_count +
                        summary.integration_statistics
                            .linear_solver_right_hand_side_evaluation_count >
                    0 &&
                summary.integration_statistics.jacobian_evaluation_count > 1 &&
                summary.integration_statistics
                        .requested_dense_finite_difference_jacobian_worker_count ==
                    8 &&
                summary.used_before_track_definition_interval &&
                !summary.used_after_track_definition_interval,
            "the short real run has the wrong sample, numerical-work or boundary summary");
    Require(std::filesystem::is_regular_file(
                configuration.output_directory / "COMPLETE") &&
                std::filesystem::is_regular_file(
                    configuration.output_directory / "metadata.json") &&
                std::filesystem::is_regular_file(
                    configuration.output_directory / "observations.tsv") &&
                std::filesystem::is_regular_file(
                    configuration.output_directory / "contact_patches.tsv") &&
                std::filesystem::is_regular_file(
                    configuration.output_directory / "performance.json") &&
                !std::filesystem::exists(root / "real-gz18.partial"),
            "the real run did not publish exactly one complete directory");

    std::ifstream input(configuration.output_directory / "observations.tsv");
    std::string line;
    std::getline(input, line);
    const std::vector<std::string> header = SplitTabs(line);
    std::vector<std::vector<std::string>> rows;
    while (std::getline(input, line)) {
        rows.push_back(SplitTabs(line));
    }
    Require(rows.size() == 2, "the real artifact does not contain two rows");
    if (rows.size() != 2) {
        return;
    }
    for (const auto& row : rows) {
        Require(row.size() == header.size(),
                "an observation row has the wrong fixed width");
        for (std::size_t column = 0; column < row.size(); ++column) {
            (void)ParseDouble(row[column]);
        }
    }

    const std::size_t front_leading = FindColumn(
        header, "front_leading_wheelset.track_station_meters");
    const std::size_t front_trailing = FindColumn(
        header, "front_trailing_wheelset.track_station_meters");
    const std::size_t rear_leading = FindColumn(
        header, "rear_leading_wheelset.track_station_meters");
    const std::size_t rear_trailing = FindColumn(
        header, "rear_trailing_wheelset.track_station_meters");
    const std::array<std::size_t, 4> station_columns{
        front_leading, front_trailing, rear_leading, rear_trailing};
    const std::array<double, 4> expected_initial{9.1, 6.6, -6.6, -9.1};
    const std::array<std::string_view, 4> carrier_names{
        "front_leading_wheelset", "front_trailing_wheelset",
        "rear_leading_wheelset", "rear_trailing_wheelset"};
    for (std::size_t carrier = 0; carrier < station_columns.size(); ++carrier) {
        const std::size_t column = station_columns[carrier];
        Require(std::abs(ParseDouble(rows.front()[column]) -
                         expected_initial[carrier]) <= 1.0e-12,
                "the formal s_ref=0 wheelset station was shifted");
        Require(ParseDouble(rows[0][column]) < ParseDouble(rows[1][column]),
                "a wheelset station is not strictly forward across samples");
        const std::size_t right_reference_station = FindColumn(
            header, std::string(carrier_names[carrier]) +
                        ".right.rail_profile_reference_marker_track_station_"
                        "meters");
        const std::size_t left_reference_station = FindColumn(
            header, std::string(carrier_names[carrier]) +
                        ".left.rail_profile_reference_marker_track_station_"
                        "meters");
        const double reference_marker_mean =
            0.5 * (ParseDouble(rows.front()[right_reference_station]) +
                   ParseDouble(rows.front()[left_reference_station]));
        Require(std::abs(reference_marker_mean - expected_initial[carrier]) <=
                    1.0e-12,
                "the initial mean rail-profile reference-marker station does "
                "not reproduce the formal s_ref=0 placement");
    }
    const std::size_t time_nanoseconds_column =
        FindColumn(header, "time_nanoseconds");
    const std::size_t time_column = FindColumn(header, "time_seconds");
    Require(rows[0][time_nanoseconds_column] == "0" &&
                rows[1][time_nanoseconds_column] == "30000000" &&
                ParseDouble(rows[0][time_column]) == 0.0,
            "the artifact does not use the integer-index sample times");

    const std::array<std::string, 8> interface_names{
        "front_leading_wheelset.right", "front_leading_wheelset.left",
        "front_trailing_wheelset.right", "front_trailing_wheelset.left",
        "rear_leading_wheelset.right", "rear_leading_wheelset.left",
        "rear_trailing_wheelset.right", "rear_trailing_wheelset.left"};
    std::ifstream patch_input(configuration.output_directory /
                              "contact_patches.tsv");
    std::getline(patch_input, line);
    const std::vector<std::string> patch_header = SplitTabs(line);
    const std::size_t patch_sample_column =
        FindColumn(patch_header, "sample_index");
    const std::size_t patch_time_column =
        FindColumn(patch_header, "time_seconds");
    const std::size_t patch_interface_column =
        FindColumn(patch_header, "interface_name");
    const std::size_t patch_ordinal_column =
        FindColumn(patch_header, "patch_ordinal");
    const std::size_t patch_normal_column =
        FindColumn(patch_header, "normal_force_newtons");
    const std::size_t patch_longitudinal_column = FindColumn(
        patch_header,
        "longitudinal_force_on_wheel_in_contact_frame_newtons");
    const std::size_t patch_lateral_column = FindColumn(
        patch_header,
        "lateral_force_on_wheel_in_contact_frame_newtons");
    const std::size_t patch_force_z_column = FindColumn(
        patch_header,
        "force_on_wheel_in_carrier_track_frame_z_newtons");
    std::array<std::array<bool, 8>, 2> seen_patch{};
    std::size_t patch_row_count = 0;
    while (std::getline(patch_input, line)) {
        const std::vector<std::string> patch_row = SplitTabs(line);
        Require(patch_row.size() == patch_header.size(),
                "a per-patch row has the wrong fixed width");
        if (patch_row.size() != patch_header.size()) {
            continue;
        }
        const double sample_value = ParseDouble(patch_row[patch_sample_column]);
        const double ordinal_value =
            ParseDouble(patch_row[patch_ordinal_column]);
        Require((sample_value == 0.0 || sample_value == 1.0) &&
                    ordinal_value == 0.0,
                "the single-patch real run has an invalid sample or patch ordinal");
        const std::size_t sample = static_cast<std::size_t>(sample_value);
        const auto interface = std::find(
            interface_names.begin(), interface_names.end(),
            patch_row[patch_interface_column]);
        Require(interface != interface_names.end(),
                "the per-patch table contains an unknown interface name");
        if (interface == interface_names.end()) {
            continue;
        }
        const std::size_t interface_ordinal = static_cast<std::size_t>(
            interface - interface_names.begin());
        Require(!seen_patch[sample][interface_ordinal],
                "the per-patch table duplicates a single-patch interface row");
        seen_patch[sample][interface_ordinal] = true;

        for (std::size_t column = 0; column < patch_row.size(); ++column) {
            if (column != patch_interface_column) {
                (void)ParseDouble(patch_row[column]);
            }
        }
        Require(ParseDouble(patch_row[patch_time_column]) ==
                    ParseDouble(rows[sample][time_column]),
                "a per-patch row is detached from its integer sample time");

        const std::string& prefix = interface_names[interface_ordinal];
        const std::size_t count_column =
            FindColumn(header, prefix + ".contact_patch_count");
        const std::size_t primary_ordinal_column =
            FindColumn(header, prefix + ".primary_patch_ordinal");
        const std::size_t primary_normal_column = FindColumn(
            header, prefix + ".primary_patch_normal_force_newtons");
        const std::size_t primary_longitudinal_column = FindColumn(
            header, prefix + ".longitudinal_force_on_wheel_newtons");
        const std::size_t primary_lateral_column = FindColumn(
            header, prefix + ".lateral_force_on_wheel_newtons");
        const std::size_t support_column = FindColumn(
            header, prefix + ".vertical_support_force_on_wheel_newtons");
        Require(ParseDouble(rows[sample][count_column]) == 1.0 &&
                    ParseDouble(rows[sample][primary_ordinal_column]) == 0.0 &&
                    ParseDouble(rows[sample][primary_normal_column]) ==
                        ParseDouble(patch_row[patch_normal_column]) &&
                    ParseDouble(rows[sample][primary_longitudinal_column]) ==
                        ParseDouble(patch_row[patch_longitudinal_column]) &&
                    ParseDouble(rows[sample][primary_lateral_column]) ==
                        ParseDouble(patch_row[patch_lateral_column]) &&
                    ParseDouble(rows[sample][support_column]) ==
                        -ParseDouble(patch_row[patch_force_z_column]),
                "the wide-table primary patch and long-form patch row disagree");
        ++patch_row_count;
    }
    Require(patch_row_count == 16,
            "the two-sample real run does not contain sixteen patch rows");
    for (const auto& sample : seen_patch) {
        Require(std::all_of(sample.begin(), sample.end(),
                            [](bool present) { return present; }),
                "the per-patch table omits a real wheel interface");
    }

    const std::string metadata =
        ReadWholeFile(configuration.output_directory / "metadata.json");
    Require(metadata.find("\"before_definition_interval\": {") !=
                std::string::npos &&
                metadata.find("\"carrier_name\": \"rear_leading_wheelset\"") !=
                    std::string::npos &&
                metadata.find("\"after_definition_interval\": null") !=
                    std::string::npos,
            "the successful artifact lacks its once-per-side boundary summary");
    Require(metadata.find(
                "\"initial_longitudinal_speed_meters_per_second\": "
                "16.666666666666668") != std::string::npos &&
                metadata.find(
                    "\"vehicle_layout_reference_track_station_meters\": "
                    "0") != std::string::npos &&
                metadata.find(std::filesystem::canonical(argv[1]).string()) !=
                    std::string::npos &&
                metadata.find(std::filesystem::canonical(argv[2]).string()) !=
                    std::string::npos &&
                metadata.find(std::filesystem::canonical(argv[3]).string()) !=
                    std::string::npos &&
                metadata.find(std::filesystem::canonical(argv[4]).string()) !=
                    std::string::npos,
            "the successful artifact lacks its physical input identity");
    Require(metadata.find("\"relative_tolerance\": 1e-08") !=
                std::string::npos &&
                metadata.find("\"openmp_dynamic_teams_enabled\": ") !=
                    std::string::npos &&
                metadata.find("\"contact_batch_worker_cap\": 8") !=
                    std::string::npos &&
                metadata.find(
                    "\"contact_batch_requested_worker_count\": ") !=
                    std::string::npos &&
                metadata.find(
                    "\"rhs_contact_projection_half_width_meters\": 0.01") !=
                    std::string::npos &&
                metadata.find(
                    "\"maximum_carrier_observation_projection_half_width_meters\": ") !=
                    std::string::npos &&
                metadata.find(
                    "\"endpoint_assembly_and_state_slice_diagnostics\": {") !=
                    std::string::npos &&
                metadata.find("\"contact_observation_contract\": {") !=
                    std::string::npos &&
                metadata.find("\"primary_patch_rule\": \"maximum normal "
                              "force; summary convenience only\"") !=
                    std::string::npos,
            "the successful artifact lacks its numerical execution contract");

    const std::string performance =
        ReadWholeFile(configuration.output_directory / "performance.json");
    Require(performance.find("\"integration_statistics\": {") !=
                    std::string::npos &&
                performance.find("\"successful_internal_step_count\": ") !=
                    std::string::npos &&
                performance.find(
                    "\"linear_solver_right_hand_side_evaluation_count\": ") !=
                    std::string::npos &&
                performance.find(
                    "\"requested_dense_finite_difference_jacobian_worker_count\": 8") !=
                    std::string::npos,
            "the successful artifact lacks its lightweight integration statistics");

    Require(Throws([&] { (void)RunGz18Qualification(configuration); }),
            "the runner overwrote an existing successful artifact");

    const std::string parallel_observations = ReadWholeFile(
        configuration.output_directory / "observations.tsv");
    const std::string parallel_patches = ReadWholeFile(
        configuration.output_directory / "contact_patches.tsv");
    for (const int requested_threads : {1, 4}) {
        omp_set_num_threads(requested_threads);
        Gz18QualificationRunConfiguration comparison = configuration;
        comparison.output_directory =
            root / ("real-gz18-t" + std::to_string(requested_threads));
        const QualificationRunSummary candidate =
            RunGz18Qualification(comparison);
        Require(candidate.integration_statistics
                        .requested_dense_finite_difference_jacobian_worker_count ==
                    requested_threads &&
                    SameTrajectoryWork(summary.integration_statistics,
                                       candidate.integration_statistics) &&
                    SameTerminalState(summary, candidate),
                "the 1/4/8-thread dense Jacobian changed the complete GZ18 "
                "trajectory or numerical work");
        Require(ReadWholeFile(comparison.output_directory /
                              "observations.tsv") ==
                        parallel_observations &&
                    ReadWholeFile(comparison.output_directory /
                                  "contact_patches.tsv") == parallel_patches,
                "the 1/4/8-thread dense Jacobian changed a GZ18 physical "
                "artifact");
    }
    omp_set_num_threads(original_openmp_max_threads);
    omp_set_dynamic(original_openmp_dynamic);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::fprintf(stderr,
                     "usage: verify_gz18_dynamics_qualification VEHICLE "
                     "STARTUP LINE DATA_ROOT IRREGULARITY_ID TEST_ROOT\n");
        return 2;
    }
    const std::filesystem::path root = argv[6];
    if (root.filename() != "g58-dynamics-qualification-fixtures") {
        std::fprintf(stderr, "refusing an unexpected G58 fixture directory\n");
        return 2;
    }
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);

    try {
        CheckIntegerClock();
        CheckStationProjection();
        CheckAtomicDirectory(root);
        CheckRealGz18Run(argv, root);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "G58 qualification threw: %s\n", error.what());
        ++failures;
    }
    std::filesystem::remove_all(root);
    if (failures != 0) {
        std::fprintf(stderr, "%d G58 qualification assertion(s) failed\n",
                     failures);
        return 1;
    }
    std::puts("G58 internal long-window qualification runner verified");
    return 0;
}
