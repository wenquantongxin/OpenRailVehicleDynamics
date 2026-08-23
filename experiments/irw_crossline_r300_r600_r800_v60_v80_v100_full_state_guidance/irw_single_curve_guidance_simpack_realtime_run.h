#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string_view>

#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {

struct IrwSingleCurveGuidanceSimpackRealtimeDefinition final {
    std::string_view experiment_name;
    std::string_view active_track_name;
    std::string_view control_profile_identity;
    double base_speed_meters_per_second{};
    double curve_radius_meters{};
    double terminal_minimum_axle_station_meters{};
    std::uint64_t maximum_observation_sample_count{};
    control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig recurrence_config;
    std::function<control::IrwFullStateWheelSpeedGuidanceOperatingPoint(
        const control::IrwGuidanceAxleValues&)>
        operating_point_evaluator;
};

struct IrwSingleCurveGuidanceSimpackRealtimeRunSummary final {
    std::size_t observation_count{};
    std::size_t control_event_count{};
    double simulated_duration_seconds{};
    double final_minimum_axle_station_meters{};
    double solver_advance_wall_seconds{};
};

/// Runs one constant-speed, single-curve experiment-supplied guidance law
/// through the proprietary SIMPACK Realtime direct-call interface.
///
/// `model_path` must already be a temporary model selecting the definition's
/// track and the 1 ms Realtime communication clock. The final output path must
/// not exist. The controller is the same experiment-local recurrence and
/// operating-point schedule used by the native ORVD arm.
[[nodiscard]] IrwSingleCurveGuidanceSimpackRealtimeRunSummary
RunIrwSingleCurveGuidanceSimpackRealtime(
    const IrwSingleCurveGuidanceSimpackRealtimeDefinition& definition,
    const std::filesystem::path& simpack_installation_path,
    const std::filesystem::path& model_path,
    const std::filesystem::path& torque_conditioner_configuration_path,
    const std::filesystem::path& output_directory);

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
