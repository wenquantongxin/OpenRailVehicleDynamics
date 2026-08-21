#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>

#include <Eigen/Core>

#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::dynamics_qualification {

// One closed R300/AAR5/60 km/h full-state wheel-speed guidance run. The
// 100 Hz event period and IRW topology are fixed by the recipe and loaded
// assets; this surface does not expose an arbitrary controller or event bus.
struct IrwR300Aar5V60At100HzFullStateGuidanceRunConfiguration final {
    std::filesystem::path vehicle_definition_path;
    std::filesystem::path resolved_startup_state_path;
    std::filesystem::path track_geometry_path;
    std::filesystem::path orvd_data_root;
    std::filesystem::path controller_configuration_path;
    std::filesystem::path torque_conditioner_configuration_path;
    std::filesystem::path output_directory;
    std::int64_t duration_nanoseconds{};
};

struct IrwR300Aar5V60At100HzFullStateGuidanceRunSummary final {
    int maximum_bdf_order{};
    std::size_t observation_count{};
    std::size_t control_audit_count{};
    std::size_t positive_hold_interval_count{};
    std::size_t backend_synchronization_count{};
    double advance_and_synchronization_wall_seconds{};
    double control_wall_seconds{};
    double observation_wall_seconds{};
    double data_and_metadata_write_wall_seconds{};
    double maximum_generalized_force_residual_inf_norm{};
    double maximum_absolute_virtual_power_residual_watts{};
    integrators::ContinuousStateIntegrationStatistics integration_statistics;
    Eigen::VectorXd terminal_continuous_state;
};

// Runs the R300/AAR5/60 km/h, 100 Hz full-state wheel-speed guidance and
// control-event personality. Event zero is installed before backend
// construction; nonterminal events explicitly synchronize the backend; the
// committed terminal result is audit-only for future time because it owns no
// positive-duration hold interval.
[[nodiscard]] IrwR300Aar5V60At100HzFullStateGuidanceRunSummary
RunIrwR300Aar5V60At100HzFullStateGuidance(
    const IrwR300Aar5V60At100HzFullStateGuidanceRunConfiguration&
        configuration);

}  // namespace orvd::dynamics_qualification
