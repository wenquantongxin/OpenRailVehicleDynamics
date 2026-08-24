#include "gz18_qualification_runner.h"

#include "vehicle_qualification_runner_internal.h"

namespace orvd::dynamics_qualification {
namespace {

constexpr internal::VehicleQualificationRecipe kRecipe{
    "GZ18",
    {"carbody", "front_bogie_frame", "rear_bogie_frame"},
    1.0e-6,
    1.0e-7,
    1.0e-6,
    1.0e-1,
    integrators::internal::SystemContinuousStateIntegrationRecipe::
        kCvodeBdf2,
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
            std::nullopt,
            input.time_integrator_qualification_case},
        kRecipe);
}

}  // namespace orvd::dynamics_qualification
