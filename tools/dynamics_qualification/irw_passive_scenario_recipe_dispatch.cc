#include "irw_passive_scenario_runs.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string_view>

#include <nlohmann/json.hpp>

#include "irw_bdf_tolerance_recipes.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "vehicle_qualification_runner_internal.h"

namespace orvd::dynamics_qualification {
namespace {

constexpr internal::VehicleQualificationRecipe kNoIrregularityRecipe{
    "IRW_R300_NO_IRREGULARITY_V60_PASSIVE",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwR300NoIrregularityV60PassiveBdfToleranceRecipe
        .relative_tolerance,
    internal::kIrwR300NoIrregularityV60PassiveBdfToleranceRecipe
        .position_absolute_tolerance,
    internal::kIrwR300NoIrregularityV60PassiveBdfToleranceRecipe
        .velocity_absolute_tolerance,
    internal::kIrwR300NoIrregularityV60PassiveBdfToleranceRecipe
        .series_force_absolute_tolerance_newtons,
    internal::kIrwR300NoIrregularityV60PassiveBdfToleranceRecipe
        .maximum_bdf_order,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kForbidden,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kAar5Recipe{
    "IRW_R300_AAR5_V60_PASSIVE",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwR300Aar5V60PassiveBdfToleranceRecipe.relative_tolerance,
    internal::kIrwR300Aar5V60PassiveBdfToleranceRecipe
        .position_absolute_tolerance,
    internal::kIrwR300Aar5V60PassiveBdfToleranceRecipe
        .velocity_absolute_tolerance,
    internal::kIrwR300Aar5V60PassiveBdfToleranceRecipe
        .series_force_absolute_tolerance_newtons,
    internal::kIrwR300Aar5V60PassiveBdfToleranceRecipe.maximum_bdf_order,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kStraightAar5V80Recipe{
    "IRW_STRAIGHT_AAR5_V80_PASSIVE",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwStraightAar5V80PassiveBdfToleranceRecipe
        .relative_tolerance,
    internal::kIrwStraightAar5V80PassiveBdfToleranceRecipe
        .position_absolute_tolerance,
    internal::kIrwStraightAar5V80PassiveBdfToleranceRecipe
        .velocity_absolute_tolerance,
    internal::kIrwStraightAar5V80PassiveBdfToleranceRecipe
        .series_force_absolute_tolerance_newtons,
    internal::kIrwStraightAar5V80PassiveBdfToleranceRecipe
        .maximum_bdf_order,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kR600Aar5V80Recipe{
    "IRW_R600_AAR5_V80_PASSIVE",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwR600Aar5V80PassiveBdfToleranceRecipe
        .relative_tolerance,
    internal::kIrwR600Aar5V80PassiveBdfToleranceRecipe
        .position_absolute_tolerance,
    internal::kIrwR600Aar5V80PassiveBdfToleranceRecipe
        .velocity_absolute_tolerance,
    internal::kIrwR600Aar5V80PassiveBdfToleranceRecipe
        .series_force_absolute_tolerance_newtons,
    internal::kIrwR600Aar5V80PassiveBdfToleranceRecipe
        .maximum_bdf_order,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kR800Aar5V100Recipe{
    "IRW_R800_AAR5_V100_PASSIVE",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwR800Aar5V100PassiveBdfToleranceRecipe
        .relative_tolerance,
    internal::kIrwR800Aar5V100PassiveBdfToleranceRecipe
        .position_absolute_tolerance,
    internal::kIrwR800Aar5V100PassiveBdfToleranceRecipe
        .velocity_absolute_tolerance,
    internal::kIrwR800Aar5V100PassiveBdfToleranceRecipe
        .series_force_absolute_tolerance_newtons,
    internal::kIrwR800Aar5V100PassiveBdfToleranceRecipe
        .maximum_bdf_order,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kStraightAar6V120Recipe{
    "IRW_STRAIGHT_AAR6_V120_PASSIVE",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwStraightAar6V120PassiveBdfToleranceRecipe
        .relative_tolerance,
    internal::kIrwStraightAar6V120PassiveBdfToleranceRecipe
        .position_absolute_tolerance,
    internal::kIrwStraightAar6V120PassiveBdfToleranceRecipe
        .velocity_absolute_tolerance,
    internal::kIrwStraightAar6V120PassiveBdfToleranceRecipe
        .series_force_absolute_tolerance_newtons,
    internal::kIrwStraightAar6V120PassiveBdfToleranceRecipe
        .maximum_bdf_order,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kR1000Aar6V120Recipe{
    "IRW_R1000_AAR6_V120_PASSIVE",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwR1000Aar6V120PassiveBdfToleranceRecipe
        .relative_tolerance,
    internal::kIrwR1000Aar6V120PassiveBdfToleranceRecipe
        .position_absolute_tolerance,
    internal::kIrwR1000Aar6V120PassiveBdfToleranceRecipe
        .velocity_absolute_tolerance,
    internal::kIrwR1000Aar6V120PassiveBdfToleranceRecipe
        .series_force_absolute_tolerance_newtons,
    internal::kIrwR1000Aar6V120PassiveBdfToleranceRecipe
        .maximum_bdf_order,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kStraightAar6V160Recipe{
    "IRW_STRAIGHT_AAR6_V160_PASSIVE",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwStraightAar6V160PassiveBdfToleranceRecipe
        .relative_tolerance,
    internal::kIrwStraightAar6V160PassiveBdfToleranceRecipe
        .position_absolute_tolerance,
    internal::kIrwStraightAar6V160PassiveBdfToleranceRecipe
        .velocity_absolute_tolerance,
    internal::kIrwStraightAar6V160PassiveBdfToleranceRecipe
        .series_force_absolute_tolerance_newtons,
    internal::kIrwStraightAar6V160PassiveBdfToleranceRecipe
        .maximum_bdf_order,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kStraightErriLowV200Recipe{
    "IRW_STRAIGHT_ERRI_LOW_V200_PASSIVE",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwStraightErriLowV200PassiveBdfToleranceRecipe
        .relative_tolerance,
    internal::kIrwStraightErriLowV200PassiveBdfToleranceRecipe
        .position_absolute_tolerance,
    internal::kIrwStraightErriLowV200PassiveBdfToleranceRecipe
        .velocity_absolute_tolerance,
    internal::kIrwStraightErriLowV200PassiveBdfToleranceRecipe
        .series_force_absolute_tolerance_newtons,
    internal::kIrwStraightErriLowV200PassiveBdfToleranceRecipe
        .maximum_bdf_order,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

constexpr std::string_view kAar5IrregularityIdentifier = "aar5_irregularity";
constexpr std::string_view kAar6IrregularityIdentifier = "aar6_irregularity";
constexpr std::string_view kErriLowIrregularityIdentifier =
    "erri_low_irregularity";
constexpr std::string_view kR300GeometryFilename =
    "r300_centerline_superelevation_1100m.json";
constexpr std::string_view kStraightGeometryFilename =
    "straight_level_1100m.json";
constexpr std::string_view kR600GeometryFilename =
    "r600_centerline_superelevation_1100m.json";
constexpr std::string_view kR800GeometryFilename =
    "r800_centerline_superelevation_1100m.json";
constexpr std::string_view kR1000GeometryFilename =
    "r1000_centerline_superelevation_300m.json";
constexpr std::string_view kIrwV60StartupRelativePath =
    "vehicle_library/irw/startup_states/moving_startup_60kmh.json";

void RequireGeometry(const IrwPassiveScenarioRunConfiguration& input,
                     std::string_view expected_filename) {
    const std::filesystem::path expected_path =
        input.orvd_data_root / "track_library" / "geometries" /
        expected_filename;
    std::error_code error;
    const bool same_file = std::filesystem::equivalent(
        input.track_geometry_path, expected_path, error);
    if (error || !same_file) {
        throw std::invalid_argument(
            "IRW passive scenario did not receive its bundled track geometry");
    }
}

nlohmann::json LoadJsonDocument(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open '" + path.string() + "'");
    }
    return nlohmann::json::parse(std::string(
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()));
}

void RequireStartupScaledFromV60(
    const IrwPassiveScenarioRunConfiguration& input,
    double expected_speed_meters_per_second) {
    const configuration::ResolvedStartupState startup =
        configuration::LoadResolvedStartupStateFromJsonFile(
            input.resolved_startup_state_path);
    const configuration::ResolvedStartupState source =
        configuration::LoadResolvedStartupStateFromJsonFile(
            input.orvd_data_root / kIrwV60StartupRelativePath);
    constexpr double kSourceSpeedMetersPerSecond = 60.0 / 3.6;
    if (source.initial_longitudinal_speed_meters_per_second !=
            kSourceSpeedMetersPerSecond ||
        startup.initial_longitudinal_speed_meters_per_second !=
            expected_speed_meters_per_second ||
        source.common_wheel_spin_generation.has_value() ||
        startup.common_wheel_spin_generation.has_value() ||
        source.revolute_joint_startup_states.size() != 8 ||
        startup.revolute_joint_startup_states.size() !=
            source.revolute_joint_startup_states.size()) {
        throw std::invalid_argument(
            "IRW passive scenario did not receive an eight-wheel start-up "
            "derived from the 60 km/h resolved state");
    }

    const double scale =
        expected_speed_meters_per_second / kSourceSpeedMetersPerSecond;
    nlohmann::json expected = LoadJsonDocument(
        input.orvd_data_root / kIrwV60StartupRelativePath);
    expected["initial_longitudinal_speed_meters_per_second"] =
        expected_speed_meters_per_second;
    auto& wheel_states = expected.at("revolute_joint_startup_states");
    if (!wheel_states.is_array() || wheel_states.size() != 8) {
        throw std::invalid_argument(
            "bundled IRW 60 km/h start-up does not contain eight wheel "
            "joints");
    }
    for (auto& wheel_state : wheel_states) {
        auto& rate = wheel_state.at("rate");
        if (rate.at("kind") != "explicit_angular_rate") {
            throw std::invalid_argument(
                "bundled IRW 60 km/h start-up does not state explicit wheel "
                "rates");
        }
        rate["angular_rate_radians_per_second"] =
            rate.at("angular_rate_radians_per_second").get<double>() * scale;
    }
    if (LoadJsonDocument(input.resolved_startup_state_path) != expected) {
        throw std::invalid_argument(
            "IRW passive scenario start-up changed fields other than the "
            "scaled longitudinal speed and eight wheel rates");
    }
}

void RequireIrregularity(const IrwPassiveScenarioRunConfiguration& input,
                         std::string_view expected_identifier) {
    if (!input.track_irregularity_identifier.has_value() ||
        *input.track_irregularity_identifier != expected_identifier) {
        throw std::invalid_argument(
            "IRW passive scenario received a different track irregularity");
    }
}

const internal::VehicleQualificationRecipe& ResolveRecipe(
    const IrwPassiveScenarioRunConfiguration& input) {
    if (input.scenario_identifier ==
        kIrwR300NoIrregularityV60PassiveScenarioIdentifier) {
        RequireGeometry(input, kR300GeometryFilename);
        RequireStartupScaledFromV60(input, 60.0 / 3.6);
        if (input.track_irregularity_identifier.has_value()) {
            throw std::invalid_argument(
                "IRW R300 no-irregularity passive scenario received an "
                "irregularity");
        }
        return kNoIrregularityRecipe;
    }
    if (input.scenario_identifier ==
        kIrwR300Aar5V60PassiveScenarioIdentifier) {
        RequireGeometry(input, kR300GeometryFilename);
        RequireStartupScaledFromV60(input, 60.0 / 3.6);
        RequireIrregularity(input, kAar5IrregularityIdentifier);
        return kAar5Recipe;
    }
    if (input.scenario_identifier ==
        kIrwStraightAar5V80PassiveScenarioIdentifier) {
        RequireGeometry(input, kStraightGeometryFilename);
        RequireStartupScaledFromV60(input, 80.0 / 3.6);
        RequireIrregularity(input, kAar5IrregularityIdentifier);
        return kStraightAar5V80Recipe;
    }
    if (input.scenario_identifier ==
        kIrwR600Aar5V80PassiveScenarioIdentifier) {
        RequireGeometry(input, kR600GeometryFilename);
        RequireStartupScaledFromV60(input, 80.0 / 3.6);
        RequireIrregularity(input, kAar5IrregularityIdentifier);
        return kR600Aar5V80Recipe;
    }
    if (input.scenario_identifier ==
        kIrwR800Aar5V100PassiveScenarioIdentifier) {
        RequireGeometry(input, kR800GeometryFilename);
        RequireStartupScaledFromV60(input, 100.0 / 3.6);
        RequireIrregularity(input, kAar5IrregularityIdentifier);
        return kR800Aar5V100Recipe;
    }
    if (input.scenario_identifier ==
        kIrwStraightAar6V120PassiveScenarioIdentifier) {
        RequireGeometry(input, kStraightGeometryFilename);
        RequireStartupScaledFromV60(input, 120.0 / 3.6);
        RequireIrregularity(input, kAar6IrregularityIdentifier);
        return kStraightAar6V120Recipe;
    }
    if (input.scenario_identifier ==
        kIrwR1000Aar6V120PassiveScenarioIdentifier) {
        RequireGeometry(input, kR1000GeometryFilename);
        RequireStartupScaledFromV60(input, 120.0 / 3.6);
        RequireIrregularity(input, kAar6IrregularityIdentifier);
        return kR1000Aar6V120Recipe;
    }
    if (input.scenario_identifier ==
        kIrwStraightAar6V160PassiveScenarioIdentifier) {
        RequireGeometry(input, kStraightGeometryFilename);
        RequireStartupScaledFromV60(input, 160.0 / 3.6);
        RequireIrregularity(input, kAar6IrregularityIdentifier);
        return kStraightAar6V160Recipe;
    }
    if (input.scenario_identifier ==
        kIrwStraightErriLowV200PassiveScenarioIdentifier) {
        RequireGeometry(input, kStraightGeometryFilename);
        RequireStartupScaledFromV60(input, 200.0 / 3.6);
        RequireIrregularity(input, kErriLowIrregularityIdentifier);
        return kStraightErriLowV200Recipe;
    }
    throw std::invalid_argument("unknown IRW passive scenario identity");
}

}  // namespace

QualificationRunSummary RunIrwPassiveScenario(
    const IrwPassiveScenarioRunConfiguration& input) {
    const internal::VehicleQualificationRecipe& recipe = ResolveRecipe(input);
    return internal::RunVehicleQualification(
        internal::QualificationRunConfiguration{
            input.vehicle_definition_path,
            input.resolved_startup_state_path,
            input.track_geometry_path,
            input.orvd_data_root,
            input.track_irregularity_identifier,
            input.output_directory,
            input.duration_nanoseconds,
            input.sample_period_nanoseconds,
            std::nullopt},
        recipe);
}

}  // namespace orvd::dynamics_qualification
