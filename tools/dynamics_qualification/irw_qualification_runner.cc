#include "irw_qualification_runner.h"

#include "vehicle_qualification_runner_internal.h"

namespace orvd::dynamics_qualification {
namespace {

constexpr internal::VehicleQualificationRecipe kNoIrregularityRecipe{
    "IRW",
    {"carbody", "frame_front", "frame_rear"},
    1.0e-7,
    1.0e-9,
    1.0e-8,
    1.0e-6,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kForbidden,
    &configuration::AssembleIrwContactScenario};

constexpr internal::VehicleQualificationRecipe kAar5Recipe{
    "IRW",
    {"carbody", "frame_front", "frame_rear"},
    1.0e-7,
    1.0e-9,
    1.0e-8,
    1.0e-6,
    81,
    74,
    2,
    96,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleIrwContactScenario};

}  // namespace

QualificationRunSummary RunIrwQualification(
    const IrwQualificationRunConfiguration& input) {
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
            input.local_sample_refinement},
        input.track_irregularity_identifier.has_value() ? kAar5Recipe
                                                        : kNoIrregularityRecipe);
}

}  // namespace orvd::dynamics_qualification
