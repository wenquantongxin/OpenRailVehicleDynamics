#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

#include <Eigen/Core>

#include "qualification_run_accounting.h"

namespace orvd::dynamics_qualification {

// Private run summary. This type is not installed and is not a
// public vehicle-simulation or observation contract.
struct QualificationRunSummary final {
    explicit QualificationRunSummary(
        std::string_view method_identifier)
        : integrator_recipe_identifier(method_identifier) {}
    QualificationRunSummary() = delete;

    std::string integrator_recipe_identifier;
    std::optional<int> maximum_bdf_order;
    std::size_t sample_count{};
    double advance_wall_seconds{};
    double observation_wall_seconds{};
    // Zero when no scene record was requested. The scene export runs inside
    // the observation replay, so its time is also contained in
    // observation_wall_seconds.
    std::size_t scene_record_frame_count{};
    double scene_record_wall_seconds{};
    double data_and_metadata_write_wall_seconds{};
    QualificationIntegrationWorkLedger integration_work;
    QualificationRunTimings numerical_timings;
    Eigen::VectorXd terminal_continuous_state;
    bool used_before_track_definition_interval{false};
    bool used_after_track_definition_interval{false};
};

}  // namespace orvd::dynamics_qualification
