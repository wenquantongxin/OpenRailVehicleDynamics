#pragma once

#include <cstddef>
#include <optional>

#include <Eigen/Core>

#include "orvd/integrators/continuous_state_advancer.h"
#include "qualification_integration_method.h"
#include "time_integrator_qualification_case.h"
#include "qualification_run_accounting.h"

namespace orvd::dynamics_qualification {

// One private migration-run summary. This type is not installed and is not a
// public vehicle-simulation or observation contract.
struct QualificationRunSummary final {
    explicit QualificationRunSummary(
        dynamics_qualification::QualificationIntegrationMethod
            integration_recipe_value)
        : integration_recipe(integration_recipe_value) {}
    QualificationRunSummary() = delete;

    dynamics_qualification::QualificationIntegrationMethod
        integration_recipe;
    std::optional<TimeIntegratorQualificationCase>
        time_integrator_qualification_case;
    std::optional<int> maximum_bdf_order;
    std::size_t sample_count{};
    double advance_wall_seconds{};
    double observation_wall_seconds{};
    // Zero when no scene record was requested. The scene export runs inside
    // the observation replay, so its time is also contained in
    // observation_wall_seconds.
    std::size_t scene_record_frame_count{};
    double scene_record_wall_seconds{};
    double endpoint_diagnostics_wall_seconds{};
    double data_and_metadata_write_wall_seconds{};
    double endpoint_generalized_force_residual_inf_norm{};
    double endpoint_virtual_power_residual_watts{};
    double endpoint_position_derivative_slice_consistency_inf_norm{};
    double endpoint_series_force_derivative_slice_consistency_inf_norm{};
    integrators::ContinuousStateIntegrationStatistics integration_statistics;
    QualificationIntegrationWorkLedger integration_work;
    QualificationRunTimings numerical_timings;
    Eigen::VectorXd terminal_continuous_state;
    bool used_before_track_definition_interval{false};
    bool used_after_track_definition_interval{false};
};

}  // namespace orvd::dynamics_qualification
