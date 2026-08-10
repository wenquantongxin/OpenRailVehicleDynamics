#pragma once

#include <cstddef>

#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::dynamics_qualification {

// One private migration-run summary. This type is not installed and is not a
// public vehicle-simulation or observation contract.
struct QualificationRunSummary final {
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
    bool used_before_track_definition_interval{false};
    bool used_after_track_definition_interval{false};
};

}  // namespace orvd::dynamics_qualification
