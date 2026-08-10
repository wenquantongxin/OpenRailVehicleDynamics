#include "gz18_qualification_runner.h"

#include "vehicle_qualification_runner_internal.h"

namespace orvd::dynamics_qualification {
namespace {

constexpr internal::VehicleQualificationRecipe kRecipe{
    "GZ18",
    {"carbody", "front_bogie_frame", "rear_bogie_frame"},
    1.0e-8,
    1.0e-10,
    1.0e-9,
    1.0e-2,
    57,
    50,
    2,
    56,
    internal::TrackIrregularityRequirement::kRequired,
    &configuration::AssembleGz18ContactScenario};

}  // namespace

QualificationRunSummary RunGz18Qualification(
    const Gz18QualificationRunConfiguration& input) {
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
        kRecipe);
}

}  // namespace orvd::dynamics_qualification
