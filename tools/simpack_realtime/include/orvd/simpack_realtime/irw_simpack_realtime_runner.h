#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>

namespace orvd::simpack_realtime {

inline constexpr std::array<std::string_view, 4>
    kIrwSimpackRealtimeAxleIdentifiers{"ff", "fr", "rf", "rr"};
inline constexpr std::array<std::string_view, 8>
    kIrwSimpackRealtimeWheelIdentifiers{
        "ff_l", "ff_r", "fr_l", "fr_r", "rf_l", "rf_r", "rr_l", "rr_r"};

// One accepted 100 Hz mechanical observation mapped to the public identity and
// scalar conventions used by the ORVD IRW full-state control session.
struct IrwSimpackRealtimeControlObservation final {
    std::uint64_t control_grid_ordinal{};
    double time_seconds{};
    std::array<double, 4> axle_track_stations_meters{};
    std::array<double, 4> axle_lateral_displacements_meters{};
    std::array<double, 4> axle_yaw_angles_radians{};
    std::array<double, 8>
        wheel_speeds_in_frozen_scalar_convention_radians_per_second{};
    // Ordered by kIrwSimpackRealtimeWheelIdentifiers. These are the held,
    // post-conditioner wheel torques that acted over the preceding interval,
    // in the accepted ORVD wheel-torque scalar convention. At t=0 there is no
    // preceding interval, so every entry is the exact zero initialization
    // sentinel.
    std::array<double, 8> previous_actual_wheel_torques_newton_metres{};
};

// Four axle differential requests in N m, ordered by
// kIrwSimpackRealtimeAxleIdentifiers. A positive request means left
// common-plus and right common-minus. The runner combines these requests with
// the longitudinal cruise command and applies the one authoritative torque
// conditioner exactly once.
using IrwSimpackRealtimeDifferentialTorqueCallback =
    std::function<std::array<double, 4>(
        const IrwSimpackRealtimeControlObservation&)>;

struct IrwSimpackRealtimeRunConfiguration final {
    std::filesystem::path simpack_installation_path;
    std::filesystem::path model_path;
    std::filesystem::path controller_configuration_path;
    std::filesystem::path torque_conditioner_configuration_path;
    std::filesystem::path output_directory;
    double last_axle_stop_station_meters{};
    double maximum_simulation_time_seconds{};
    std::uint64_t observation_decimation{};
    std::string cpu_assignment;
    double communication_timeout_seconds{1.0};
    int simpack_verbose_level{0};
    // Empty means strict zero differential and preserves the historical cruise
    // runner.  A callback is invoked at t=0 and every later 10 ms control event.
    IrwSimpackRealtimeDifferentialTorqueCallback differential_torque_callback;
};

struct IrwSimpackRealtimeRunSummary final {
    std::size_t observation_count{};
    std::size_t control_event_count{};
    std::size_t contact_patch_row_count{};
    double final_simulation_time_seconds{};
    double final_last_axle_station_meters{};
    double solver_advance_wall_time_seconds{};
    double realtime_loop_wall_time_seconds{};
};

// Runs one closed 100 Hz longitudinal-cruise SIMPACK Realtime session.
//
// The stop station and simulation-time cap are required caller inputs. A
// successful run is published atomically. A failed run deliberately leaves its
// `.partial` directory and a failure record for diagnosis.
[[nodiscard]] IrwSimpackRealtimeRunSummary RunIrwSimpackRealtimeCruise(
    const IrwSimpackRealtimeRunConfiguration& configuration);

}  // namespace orvd::simpack_realtime
