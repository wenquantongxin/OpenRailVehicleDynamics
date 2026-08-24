// The 80, 100, 120, 160 and 200 km/h passive IRW identities bind line,
// irregularity, a physically scaled eight-wheel start-up, and one private
// numerical recipe.

#include <algorithm>
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

#include <nlohmann/json.hpp>

#include "irw_passive_scenario_runs.h"

namespace {

using orvd::dynamics_qualification::IrwPassiveScenarioRunConfiguration;
using orvd::dynamics_qualification::RunIrwPassiveScenario;

constexpr double kV60MetersPerSecond = 60.0 / 3.6;
constexpr double kV80MetersPerSecond = 80.0 / 3.6;
constexpr double kV100MetersPerSecond = 100.0 / 3.6;
constexpr double kV120MetersPerSecond = 120.0 / 3.6;
constexpr double kV160MetersPerSecond = 160.0 / 3.6;
constexpr double kV200MetersPerSecond = 200.0 / 3.6;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(
            stderr,
            "IRW 80/100/120/160/200 km/h passive scenario check: %.*s\n",
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

void WriteJson(const std::filesystem::path& path,
               const nlohmann::json& document) {
    std::ofstream output(path, std::ios::out | std::ios::binary);
    if (!output) {
        throw std::runtime_error("could not write " + path.string());
    }
    output << document.dump(2) << '\n';
}

std::filesystem::path WriteIrwStartupScaledFromV60(
    const std::filesystem::path& source,
    const std::filesystem::path& destination,
    double target_speed_meters_per_second) {
    nlohmann::json document = nlohmann::json::parse(ReadWholeFile(source));
    const double source_speed =
        document.at("initial_longitudinal_speed_meters_per_second")
            .get<double>();
    if (source_speed != kV60MetersPerSecond ||
        !std::isfinite(target_speed_meters_per_second) ||
        target_speed_meters_per_second <= 0.0) {
        throw std::runtime_error(
            "IRW start-up source is not the finite 60 km/h state");
    }
    const double scale = target_speed_meters_per_second / source_speed;
    document["initial_longitudinal_speed_meters_per_second"] =
        target_speed_meters_per_second;

    constexpr std::array<std::string_view, 8> kWheelJoints{
        "rev_wheel_ff_l", "rev_wheel_ff_r", "rev_wheel_fr_l",
        "rev_wheel_fr_r", "rev_wheel_rf_l", "rev_wheel_rf_r",
        "rev_wheel_rr_l", "rev_wheel_rr_r"};
    std::array<bool, kWheelJoints.size()> seen{};
    auto& states = document.at("revolute_joint_startup_states");
    if (!states.is_array() || states.size() != kWheelJoints.size()) {
        throw std::runtime_error(
            "IRW start-up does not contain the closed eight-wheel set");
    }
    for (auto& state : states) {
        const std::string name = state.at("joint_name").get<std::string>();
        const auto iterator = std::ranges::find(kWheelJoints, name);
        if (iterator == kWheelJoints.end()) {
            throw std::runtime_error("IRW start-up contains an unknown wheel");
        }
        const std::size_t index =
            static_cast<std::size_t>(iterator - kWheelJoints.begin());
        if (seen[index]) {
            throw std::runtime_error("IRW start-up repeats a wheel identity");
        }
        seen[index] = true;
        auto& rate = state.at("rate");
        if (rate.at("kind").get<std::string>() !=
            "explicit_angular_rate") {
            throw std::runtime_error("IRW wheel rate is not explicit");
        }
        const double source_rate =
            rate.at("angular_rate_radians_per_second").get<double>();
        if (!std::isfinite(source_rate)) {
            throw std::runtime_error("IRW wheel rate is not finite");
        }
        rate["angular_rate_radians_per_second"] = source_rate * scale;
    }
    WriteJson(destination, document);
    return destination;
}

std::filesystem::path WriteIrwStartupWithModifiedNonWheelField(
    const std::filesystem::path& source,
    const std::filesystem::path& destination) {
    nlohmann::json document = nlohmann::json::parse(ReadWholeFile(source));
    document["rail_profile_reference_vertical_offset_meters"] =
        document.at("rail_profile_reference_vertical_offset_meters")
                .get<double>() +
        1.0e-6;
    WriteJson(destination, document);
    return destination;
}

std::vector<std::filesystem::path>
WriteIrwStartupsWithOneMismatchedWheelRate(
    const std::filesystem::path& correctly_scaled_source,
    const std::filesystem::path& output_directory,
    std::string_view speed_name) {
    const nlohmann::json source =
        nlohmann::json::parse(ReadWholeFile(correctly_scaled_source));
    const auto& source_states = source.at("revolute_joint_startup_states");
    if (!source_states.is_array() || source_states.size() != 8) {
        throw std::runtime_error(
            "scaled IRW start-up does not contain eight wheel joints");
    }

    std::vector<std::filesystem::path> outputs;
    outputs.reserve(source_states.size());
    for (std::size_t index = 0; index < source_states.size(); ++index) {
        nlohmann::json candidate = source;
        auto& rate = candidate.at("revolute_joint_startup_states")
                         .at(index)
                         .at("rate")
                         .at("angular_rate_radians_per_second");
        rate = rate.get<double>() + 1.0;
        const std::filesystem::path path =
            output_directory /
            ("startup-" + std::string(speed_name) + "-wrong-wheel-" +
             std::to_string(index) + ".json");
        WriteJson(path, candidate);
        outputs.push_back(path);
    }
    return outputs;
}

struct ScenarioExpectation final {
    std::string_view identifier;
    std::string_view metadata_label;
    std::filesystem::path geometry;
    std::filesystem::path different_geometry;
    std::filesystem::path startup;
    std::filesystem::path different_speed_startup;
    std::filesystem::path modified_non_wheel_startup;
    std::vector<std::filesystem::path> mismatched_wheel_rate_startups;
    std::string_view irregularity;
    std::string_view different_irregularity;
    double speed_meters_per_second{};
    std::string_view output_stem;
};

void CheckScenario(const ScenarioExpectation& expected,
                   const std::filesystem::path& vehicle,
                   const std::filesystem::path& data_root,
                   const std::filesystem::path& test_root) {
    IrwPassiveScenarioRunConfiguration configuration;
    configuration.scenario_identifier = expected.identifier;
    configuration.vehicle_definition_path = vehicle;
    configuration.resolved_startup_state_path = expected.startup;
    configuration.track_geometry_path = expected.geometry;
    configuration.orvd_data_root = data_root;
    configuration.track_irregularity_identifier = expected.irregularity;
    configuration.output_directory = test_root / expected.output_stem;
    configuration.duration_nanoseconds = 100'000;
    configuration.sample_period_nanoseconds = 100'000;

    const auto summary = RunIrwPassiveScenario(configuration);
    Require(summary.sample_count == 2 && summary.maximum_bdf_order == 5 &&
                summary.integration_recipe ==
                    orvd::integrators::internal::
                        SystemContinuousStateIntegrationRecipe::kCvodeBdf5,
            "a declared scenario did not run with its fifth-order recipe");

    const nlohmann::json metadata = nlohmann::json::parse(
        ReadWholeFile(configuration.output_directory / "metadata.json"));
    const auto& contract = metadata.at("numerical_execution_contract");
    const auto& input_paths = metadata.at("input_paths");
    Require(metadata.at("qualification_vehicle_recipe") ==
                    expected.metadata_label &&
                metadata.at("initial_longitudinal_speed_meters_per_second") ==
                    expected.speed_meters_per_second &&
                metadata.at("track_irregularity_identifier") ==
                    expected.irregularity &&
                contract.at("integrator_recipe_identifier") ==
                    "cvode_bdf5" &&
                contract.at("maximum_bdf_order") == 5 &&
                contract.at("relative_tolerance") == 1.0e-9 &&
                contract.at("generalized_position_absolute_tolerance") ==
                    1.0e-9 &&
                contract.at("generalized_velocity_absolute_tolerance") ==
                    1.0e-8 &&
                contract.at("series_force_absolute_tolerance_newtons") ==
                    1.0e-7,
            "a declared scenario published the wrong physical or numerical "
            "identity");
    Require(std::filesystem::path(
                input_paths.at("track_geometry").get<std::string>()) ==
                std::filesystem::canonical(expected.geometry) &&
                std::filesystem::path(
                    input_paths.at("resolved_startup_state")
                        .get<std::string>()) ==
                    std::filesystem::canonical(expected.startup),
            "metadata did not retain the resolved line and start-up inputs");

    IrwPassiveScenarioRunConfiguration invalid = configuration;
    invalid.track_geometry_path = expected.different_geometry;
    invalid.output_directory = test_root /
                               (std::string(expected.output_stem) +
                                "-wrong-geometry");
    Require(Throws([&] { (void)RunIrwPassiveScenario(invalid); }),
            "a scenario accepted a different line geometry");

    invalid = configuration;
    const std::filesystem::path same_name_directory =
        test_root /
        (std::string(expected.output_stem) + "-copied-geometry-directory");
    std::filesystem::create_directories(same_name_directory);
    invalid.track_geometry_path =
        same_name_directory / expected.geometry.filename();
    std::filesystem::copy_file(expected.geometry,
                               invalid.track_geometry_path);
    invalid.output_directory =
        test_root / (std::string(expected.output_stem) +
                     "-copied-geometry");
    Require(Throws([&] { (void)RunIrwPassiveScenario(invalid); }),
            "a scenario accepted a copied line with the bundled filename");

    invalid = configuration;
    invalid.resolved_startup_state_path = expected.different_speed_startup;
    invalid.output_directory =
        test_root / (std::string(expected.output_stem) + "-wrong-speed");
    Require(Throws([&] { (void)RunIrwPassiveScenario(invalid); }),
            "a scenario accepted a different longitudinal start-up speed");

    invalid = configuration;
    invalid.track_irregularity_identifier = expected.different_irregularity;
    invalid.output_directory =
        test_root / (std::string(expected.output_stem) + "-wrong-irregularity");
    Require(Throws([&] { (void)RunIrwPassiveScenario(invalid); }),
            "a scenario accepted the other AAR irregularity field");

    invalid = configuration;
    invalid.resolved_startup_state_path = expected.modified_non_wheel_startup;
    invalid.output_directory =
        test_root / (std::string(expected.output_stem) +
                     "-modified-non-wheel-field");
    Require(Throws([&] { (void)RunIrwPassiveScenario(invalid); }),
            "a scenario accepted a change outside speed and wheel rates");

    for (std::size_t index = 0;
         index < expected.mismatched_wheel_rate_startups.size(); ++index) {
        invalid = configuration;
        invalid.resolved_startup_state_path =
            expected.mismatched_wheel_rate_startups[index];
        invalid.output_directory =
            test_root /
            (std::string(expected.output_stem) + "-wrong-wheel-" +
             std::to_string(index));
        Require(Throws([&] { (void)RunIrwPassiveScenario(invalid); }),
                "a scenario accepted one incorrectly scaled wheel rate");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 9) {
        std::fprintf(
            stderr,
            "usage: verify_irw_aar5_v80_v100_aar6_v120_v160_and_erri_low_"
            "v200_passive_scenarios "
            "VEHICLE V60_STARTUP DATA_ROOT STRAIGHT_LINE R600_LINE "
            "R800_LINE R1000_LINE TEST_ROOT\n");
        return 2;
    }
    const std::filesystem::path test_root = argv[8];
    if (test_root.filename() !=
        "irw-aar5-v80-v100-aar6-v120-v160-and-erri-low-v200-passive-"
        "scenario-fixtures") {
        std::fprintf(stderr, "refusing an unexpected IRW fixture directory\n");
        return 2;
    }
    std::filesystem::remove_all(test_root);
    std::filesystem::create_directories(test_root);

    try {
        const std::filesystem::path startup_v80 =
            WriteIrwStartupScaledFromV60(
                argv[2], test_root / "startup-scaled-v60-to-v80.json",
                kV80MetersPerSecond);
        const std::filesystem::path startup_v120 =
            WriteIrwStartupScaledFromV60(
                argv[2], test_root / "startup-scaled-v60-to-v120.json",
                kV120MetersPerSecond);
        const std::filesystem::path startup_v100 =
            WriteIrwStartupScaledFromV60(
                argv[2], test_root / "startup-scaled-v60-to-v100.json",
                kV100MetersPerSecond);
        const std::filesystem::path startup_v160 =
            WriteIrwStartupScaledFromV60(
                argv[2], test_root / "startup-scaled-v60-to-v160.json",
                kV160MetersPerSecond);
        const std::filesystem::path startup_v200 =
            WriteIrwStartupScaledFromV60(
                argv[2], test_root / "startup-scaled-v60-to-v200.json",
                kV200MetersPerSecond);
        const std::filesystem::path modified_non_wheel_v80 =
            WriteIrwStartupWithModifiedNonWheelField(
                startup_v80,
                test_root / "startup-v80-with-modified-rail-offset.json");
        const std::filesystem::path modified_non_wheel_v120 =
            WriteIrwStartupWithModifiedNonWheelField(
                startup_v120,
                test_root / "startup-v120-with-modified-rail-offset.json");
        const std::filesystem::path modified_non_wheel_v100 =
            WriteIrwStartupWithModifiedNonWheelField(
                startup_v100,
                test_root / "startup-v100-with-modified-rail-offset.json");
        const std::filesystem::path modified_non_wheel_v160 =
            WriteIrwStartupWithModifiedNonWheelField(
                startup_v160,
                test_root / "startup-v160-with-modified-rail-offset.json");
        const std::filesystem::path modified_non_wheel_v200 =
            WriteIrwStartupWithModifiedNonWheelField(
                startup_v200,
                test_root / "startup-v200-with-modified-rail-offset.json");
        const std::vector<std::filesystem::path> wrong_wheels_v80 =
            WriteIrwStartupsWithOneMismatchedWheelRate(
                startup_v80, test_root, "v80");
        const std::vector<std::filesystem::path> wrong_wheels_v120 =
            WriteIrwStartupsWithOneMismatchedWheelRate(
                startup_v120, test_root, "v120");
        const std::vector<std::filesystem::path> wrong_wheels_v100 =
            WriteIrwStartupsWithOneMismatchedWheelRate(
                startup_v100, test_root, "v100");
        const std::vector<std::filesystem::path> wrong_wheels_v160 =
            WriteIrwStartupsWithOneMismatchedWheelRate(
                startup_v160, test_root, "v160");
        const std::vector<std::filesystem::path> wrong_wheels_v200 =
            WriteIrwStartupsWithOneMismatchedWheelRate(
                startup_v200, test_root, "v200");

        const std::vector<ScenarioExpectation> scenarios{
            {orvd::dynamics_qualification::
                 kIrwStraightAar5V80PassiveScenarioIdentifier,
             "IRW_STRAIGHT_AAR5_V80_PASSIVE", argv[4], argv[5], startup_v80,
             startup_v120, modified_non_wheel_v80, wrong_wheels_v80,
             "aar5_irregularity", "aar6_irregularity", kV80MetersPerSecond,
             "straight-aar5-v80"},
            {orvd::dynamics_qualification::
                 kIrwR600Aar5V80PassiveScenarioIdentifier,
             "IRW_R600_AAR5_V80_PASSIVE", argv[5], argv[4], startup_v80,
             startup_v120, modified_non_wheel_v80, {}, "aar5_irregularity",
             "aar6_irregularity", kV80MetersPerSecond, "r600-aar5-v80"},
            {orvd::dynamics_qualification::
                 kIrwR800Aar5V100PassiveScenarioIdentifier,
             "IRW_R800_AAR5_V100_PASSIVE", argv[6], argv[4], startup_v100,
             startup_v120, modified_non_wheel_v100, wrong_wheels_v100,
             "aar5_irregularity", "aar6_irregularity", kV100MetersPerSecond,
             "r800-aar5-v100"},
            {orvd::dynamics_qualification::
                 kIrwStraightAar6V120PassiveScenarioIdentifier,
             "IRW_STRAIGHT_AAR6_V120_PASSIVE", argv[4], argv[5], startup_v120,
             startup_v80, modified_non_wheel_v120, wrong_wheels_v120,
             "aar6_irregularity", "aar5_irregularity", kV120MetersPerSecond,
             "straight-aar6-v120"},
            {orvd::dynamics_qualification::
                 kIrwR1000Aar6V120PassiveScenarioIdentifier,
             "IRW_R1000_AAR6_V120_PASSIVE", argv[7], argv[4], startup_v120,
             startup_v80, modified_non_wheel_v120, {}, "aar6_irregularity",
             "aar5_irregularity", kV120MetersPerSecond, "r1000-aar6-v120"},
            {orvd::dynamics_qualification::
                 kIrwStraightAar6V160PassiveScenarioIdentifier,
             "IRW_STRAIGHT_AAR6_V160_PASSIVE", argv[4], argv[5], startup_v160,
             startup_v120, modified_non_wheel_v160, wrong_wheels_v160,
             "aar6_irregularity", "aar5_irregularity", kV160MetersPerSecond,
             "straight-aar6-v160"},
            {orvd::dynamics_qualification::
                 kIrwStraightErriLowV200PassiveScenarioIdentifier,
             "IRW_STRAIGHT_ERRI_LOW_V200_PASSIVE", argv[4], argv[5],
             startup_v200, startup_v160, modified_non_wheel_v200,
             wrong_wheels_v200, "erri_low_irregularity", "aar6_irregularity",
             kV200MetersPerSecond, "straight-erri-low-v200"},
        };
        for (const ScenarioExpectation& scenario : scenarios) {
            CheckScenario(scenario, argv[1], argv[3], test_root);
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW passive scenario check threw: %s\n",
                     error.what());
        ++failures;
    }

    std::filesystem::remove_all(test_root);
    if (failures != 0) {
        std::fprintf(stderr, "%d IRW passive scenario assertion(s) failed\n",
                     failures);
        return 1;
    }
    std::puts(
        "IRW AAR5/V80/V100, AAR6/V120/V160, and ERRI-low/V200 passive "
        "scenarios verified");
    return 0;
}
