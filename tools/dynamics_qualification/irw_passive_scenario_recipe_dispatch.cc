#include "irw_passive_scenario_runs.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string_view>

#include <nlohmann/json.hpp>

#include "irw_integration_recipes.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "vehicle_qualification_runner_internal.h"

namespace orvd::dynamics_qualification {
namespace {

constexpr internal::VehicleQualificationRecipe MakeIrwRecipe(
    std::string_view label,
    const internal::IrwIntegrationRecipe& integration,
    internal::TrackIrregularityRequirement irregularity_requirement) {
    return internal::VehicleQualificationRecipe{
        label,
        {"carbody", "frame_front", "frame_rear"},
        integration.relative_tolerance,
        integration.position_absolute_tolerance,
        integration.velocity_absolute_tolerance,
        integration.series_force_absolute_tolerance_newtons,
        integration.default_integration_recipe,
        81,
        74,
        2,
        96,
        irregularity_requirement,
        &configuration::AssembleIrwContactScenario};
}

constexpr internal::VehicleQualificationRecipe kNoIrregularityRecipe =
    MakeIrwRecipe(
        "IRW_R300_NO_IRREGULARITY_V60_PASSIVE",
        internal::kIrwR300NoIrregularityV60PassiveIntegrationRecipe,
        internal::TrackIrregularityRequirement::kForbidden);
constexpr internal::VehicleQualificationRecipe kAar5Recipe = MakeIrwRecipe(
    "IRW_R300_AAR5_V60_PASSIVE",
    internal::kIrwR300Aar5V60PassiveIntegrationRecipe,
    internal::TrackIrregularityRequirement::kRequired);
constexpr internal::VehicleQualificationRecipe kStraightAar5V80Recipe =
    MakeIrwRecipe(
        "IRW_STRAIGHT_AAR5_V80_PASSIVE",
        internal::kIrwStraightAar5V80PassiveIntegrationRecipe,
        internal::TrackIrregularityRequirement::kRequired);
constexpr internal::VehicleQualificationRecipe kR600Aar5V80Recipe =
    MakeIrwRecipe(
        "IRW_R600_AAR5_V80_PASSIVE",
        internal::kIrwR600Aar5V80PassiveIntegrationRecipe,
        internal::TrackIrregularityRequirement::kRequired);
constexpr internal::VehicleQualificationRecipe kR800Aar5V100Recipe =
    MakeIrwRecipe(
        "IRW_R800_AAR5_V100_PASSIVE",
        internal::kIrwR800Aar5V100PassiveIntegrationRecipe,
        internal::TrackIrregularityRequirement::kRequired);
constexpr internal::VehicleQualificationRecipe kStraightAar6V120Recipe =
    MakeIrwRecipe(
        "IRW_STRAIGHT_AAR6_V120_PASSIVE",
        internal::kIrwStraightAar6V120PassiveIntegrationRecipe,
        internal::TrackIrregularityRequirement::kRequired);
constexpr internal::VehicleQualificationRecipe kR1000Aar6V120Recipe =
    MakeIrwRecipe(
        "IRW_R1000_AAR6_V120_PASSIVE",
        internal::kIrwR1000Aar6V120PassiveIntegrationRecipe,
        internal::TrackIrregularityRequirement::kRequired);
constexpr internal::VehicleQualificationRecipe kStraightAar6V160Recipe =
    MakeIrwRecipe(
        "IRW_STRAIGHT_AAR6_V160_PASSIVE",
        internal::kIrwStraightAar6V160PassiveIntegrationRecipe,
        internal::TrackIrregularityRequirement::kRequired);
constexpr internal::VehicleQualificationRecipe kStraightErriLowV200Recipe =
    MakeIrwRecipe(
        "IRW_STRAIGHT_ERRI_LOW_V200_PASSIVE",
        internal::kIrwStraightErriLowV200PassiveIntegrationRecipe,
        internal::TrackIrregularityRequirement::kRequired);

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
            std::nullopt,
            input.time_integrator_qualification_case},
        recipe);
}

}  // namespace orvd::dynamics_qualification
