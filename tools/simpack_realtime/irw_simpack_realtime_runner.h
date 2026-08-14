#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace orvd::simpack_realtime {

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
