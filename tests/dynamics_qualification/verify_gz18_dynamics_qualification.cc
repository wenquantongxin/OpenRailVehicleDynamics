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

#include "atomic_qualification_directory.h"
#include "gz18_qualification_runner.h"
#include "qualification_sample_clock.h"
#include "station_series_projection.h"

namespace {

using orvd::dynamics_qualification::AtomicQualificationDirectory;
using orvd::dynamics_qualification::Gz18QualificationRunConfiguration;
using orvd::dynamics_qualification::ProjectMonotoneSeriesToStationGrid;
using orvd::dynamics_qualification::QualificationSampleClock;
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
    Gz18QualificationRunConfiguration configuration;
    configuration.vehicle_definition_path =
        std::filesystem::relative(argv[1]);
    configuration.resolved_startup_state_path =
        std::filesystem::relative(argv[2]);
    configuration.track_geometry_path = std::filesystem::relative(argv[3]);
    configuration.orvd_data_root = std::filesystem::relative(argv[4]);
    configuration.track_irregularity_identifier = argv[5];
    configuration.output_directory = root / "real-gz18";
    configuration.duration_nanoseconds = 2'000'000;
    configuration.sample_period_nanoseconds = 2'000'000;

    const auto summary = RunGz18Qualification(configuration);
    Require(summary.sample_count == 2 &&
                summary.used_before_track_definition_interval &&
                !summary.used_after_track_definition_interval,
            "the short real run has the wrong sample or boundary summary");
    Require(std::filesystem::is_regular_file(
                configuration.output_directory / "COMPLETE") &&
                std::filesystem::is_regular_file(
                    configuration.output_directory / "metadata.json") &&
                std::filesystem::is_regular_file(
                    configuration.output_directory / "observations.tsv") &&
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
    const std::size_t time_column = FindColumn(header, "time_seconds");
    Require(ParseDouble(rows[0][time_column]) == 0.0 &&
                ParseDouble(rows[1][time_column]) == 0.002,
            "the artifact does not use the integer-index sample times");

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
                    std::string::npos,
            "the successful artifact lacks its numerical execution contract");

    Require(Throws([&] { (void)RunGz18Qualification(configuration); }),
            "the runner overwrote an existing successful artifact");
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
