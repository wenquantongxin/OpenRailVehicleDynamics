// The closed IRW runner keeps G70/G71 explicitly free of track irregularity
// and accepts G72's frozen AAR5 asset through the same physical recipe.

#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "irw_qualification_runner.h"

namespace {

using orvd::dynamics_qualification::IrwQualificationRunConfiguration;
using orvd::dynamics_qualification::RunIrwQualification;

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
    Require(metadata.find("\"relative_tolerance\": 9.9999999999999995e-08") !=
                    std::string::npos &&
                metadata.find(
                    "\"generalized_position_absolute_tolerance\": 1.0000000000000001e-09") !=
                    std::string::npos &&
                metadata.find(
                    "\"generalized_velocity_absolute_tolerance\": 1e-08") !=
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
        "irw_r300_aar5_reference_irregularity";
    const auto aar5_summary = RunIrwQualification(configuration);
    Require(aar5_summary.sample_count == 2,
            "the short AAR5 loading run did not publish its two clock points");
    const std::string aar5_metadata =
        ReadWholeFile(configuration.output_directory / "metadata.json");
    Require(aar5_metadata.find(
                "\"track_irregularity_identifier\": "
                "\"irw_r300_aar5_reference_irregularity\"") !=
                std::string::npos,
            "the IRW AAR5 identity did not reach the qualification artifact");
    configuration.output_directory = root / "empty-irregularity";
    configuration.track_irregularity_identifier = "";
    Require(Throws([&] { (void)RunIrwQualification(configuration); }),
            "the required IRW AAR5 recipe accepted an empty identity");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 6) {
        std::fprintf(stderr,
                     "usage: verify_irw_dynamics_qualification VEHICLE "
                     "STARTUP LINE DATA_ROOT TEST_ROOT\n");
        return 2;
    }
    const std::filesystem::path root = argv[5];
    if (root.filename() != "irw-dynamics-qualification-fixtures") {
        std::fprintf(stderr, "refusing an unexpected IRW fixture directory\n");
        return 2;
    }
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);

    try {
        CheckRealIrwRun(argv, root);
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
    std::puts("IRW no-irregularity and frozen-AAR5 recipes verified");
    return 0;
}
