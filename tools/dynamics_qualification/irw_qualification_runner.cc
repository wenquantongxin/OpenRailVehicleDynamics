#include "irw_qualification_runner.h"

#include <stdexcept>
#include <string_view>

#include "irw_integration_recipe.h"
#include "vehicle_qualification_runner_internal.h"

namespace orvd::dynamics_qualification {
namespace {

constexpr internal::VehicleQualificationRecipe kNoIrregularityRecipe{
    "IRW",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwNoIrregularityPassiveIntegrationRecipe.relative_tolerance,
    internal::kIrwNoIrregularityPassiveIntegrationRecipe
        .position_absolute_tolerance,
    internal::kIrwNoIrregularityPassiveIntegrationRecipe
        .velocity_absolute_tolerance,
    internal::kIrwNoIrregularityPassiveIntegrationRecipe
        .series_force_absolute_tolerance_newtons,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kForbidden,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kAar5Recipe{
    "IRW",
    {"carbody", "frame_front", "frame_rear"},
    internal::kIrwR300Aar5PassiveIntegrationRecipe.relative_tolerance,
    internal::kIrwR300Aar5PassiveIntegrationRecipe
        .position_absolute_tolerance,
    internal::kIrwR300Aar5PassiveIntegrationRecipe
        .velocity_absolute_tolerance,
    internal::kIrwR300Aar5PassiveIntegrationRecipe
        .series_force_absolute_tolerance_newtons,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

constexpr std::string_view kAar5IrregularityIdentifier = "aar5_irregularity";

const internal::VehicleQualificationRecipe& ResolveRecipe(
    const IrwQualificationRunConfiguration& input) {
    if (!input.track_irregularity_identifier.has_value()) {
        return kNoIrregularityRecipe;
    }
    if (*input.track_irregularity_identifier == kAar5IrregularityIdentifier) {
        return kAar5Recipe;
    }
    throw std::invalid_argument(
        "IRW qualification accepts only no track irregularity or "
        "'aar5_irregularity'");
}

}  // namespace

QualificationRunSummary RunIrwQualification(
    const IrwQualificationRunConfiguration& input) {
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
