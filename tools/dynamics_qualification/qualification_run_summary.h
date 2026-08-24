#pragma once

#include <cstddef>
#include <optional>

#include <Eigen/Core>

#include "orvd/integrators/continuous_state_advancer.h"
#include "system_continuous_state_integration_recipe.h"
#include "time_integrator_qualification_case.h"

namespace orvd::dynamics_qualification {

// One private migration-run summary. This type is not installed and is not a
// public vehicle-simulation or observation contract.
struct QualificationRunSummary final {
    explicit QualificationRunSummary(
        integrators::internal::SystemContinuousStateIntegrationRecipe
            integration_recipe_value)
        : integration_recipe(integration_recipe_value) {}
    QualificationRunSummary() = delete;

    integrators::internal::SystemContinuousStateIntegrationRecipe
        integration_recipe;
    std::optional<TimeIntegratorQualificationCase>
        time_integrator_qualification_case;
    std::optional<int> maximum_bdf_order;
    std::size_t sample_count{};
    double advance_wall_seconds{};
    double observation_wall_seconds{};
    double endpoint_diagnostics_wall_seconds{};
    double data_and_metadata_write_wall_seconds{};
    double endpoint_generalized_force_residual_inf_norm{};
    double endpoint_virtual_power_residual_watts{};
    double endpoint_position_derivative_slice_consistency_inf_norm{};
    double endpoint_series_force_derivative_slice_consistency_inf_norm{};
    integrators::ContinuousStateIntegrationStatistics integration_statistics;
    Eigen::VectorXd terminal_continuous_state;
    bool used_before_track_definition_interval{false};
    bool used_after_track_definition_interval{false};
};

}  // namespace orvd::dynamics_qualification
