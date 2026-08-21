#include "irw_passive_scenario_runs.h"

#include <stdexcept>
#include <string_view>

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

constexpr std::string_view kAar5IrregularityIdentifier = "aar5_irregularity";
constexpr std::string_view kR300GeometryFilename =
    "r300_centerline_superelevation_1100m.json";

void RequireR300V60Identity(
    const IrwPassiveScenarioRunConfiguration& input) {
    if (input.track_geometry_path.filename() != kR300GeometryFilename) {
        throw std::invalid_argument(
            "IRW R300/60 km/h passive scenario received a different track "
            "geometry");
    }
    const configuration::ResolvedStartupState startup =
        configuration::LoadResolvedStartupStateFromJsonFile(
            input.resolved_startup_state_path);
    if (startup.initial_longitudinal_speed_meters_per_second != 60.0 / 3.6) {
        throw std::invalid_argument(
            "IRW R300/60 km/h passive scenario received a different "
            "initial longitudinal speed");
    }
}

const internal::VehicleQualificationRecipe& ResolveRecipe(
    const IrwPassiveScenarioRunConfiguration& input) {
    RequireR300V60Identity(input);
    if (input.scenario_identifier ==
        kIrwR300NoIrregularityV60PassiveScenarioIdentifier) {
        if (input.track_irregularity_identifier.has_value()) {
            throw std::invalid_argument(
                "IRW R300 no-irregularity passive scenario received an "
                "irregularity");
        }
        return kNoIrregularityRecipe;
    }
    if (input.scenario_identifier ==
        kIrwR300Aar5V60PassiveScenarioIdentifier) {
        if (!input.track_irregularity_identifier.has_value() ||
            *input.track_irregularity_identifier !=
                kAar5IrregularityIdentifier) {
            throw std::invalid_argument(
                "IRW R300/AAR5/60 km/h passive scenario requires "
                "'aar5_irregularity'");
        }
        return kAar5Recipe;
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
