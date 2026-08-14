#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <string_view>

#include "irw_simpack_realtime_runner.h"

#ifndef ORVD_SIMPACK_ROOT
#error "ORVD_SIMPACK_ROOT must be defined by the optional tool build"
#endif

namespace {

bool ParseFiniteDouble(std::string_view text, double* output) {
    if (output == nullptr || text.empty()) {
        return false;
    }
    double value{};
    const auto [end, error] = std::from_chars(
        text.data(), text.data() + text.size(), value,
        std::chars_format::general);
    if (error != std::errc{} || end != text.data() + text.size() ||
        !std::isfinite(value)) {
        return false;
    }
    *output = value;
    return true;
}

bool ParsePositiveInteger(std::string_view text, std::uint64_t* output) {
    if (output == nullptr || text.empty()) {
        return false;
    }
    std::uint64_t value{};
    const auto [end, error] =
        std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() ||
        value == 0) {
        return false;
    }
    *output = value;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 9 && argc != 10) {
        std::fprintf(
            stderr,
            "usage: orvd_irw_simpack_realtime_cruise MODEL CONTROLLER "
            "CONDITIONER OUTPUT_DIRECTORY LAST_AXLE_STOP_STATION_M "
            "MAXIMUM_SIMULATION_TIME_S OBSERVATION_DECIMATION "
            "COMMUNICATION_TIMEOUT_S "
            "[CPU_ASSIGNMENT]\n");
        return 2;
    }
    orvd::simpack_realtime::IrwSimpackRealtimeRunConfiguration configuration;
    configuration.simpack_installation_path = ORVD_SIMPACK_ROOT;
    configuration.model_path = argv[1];
    configuration.controller_configuration_path = argv[2];
    configuration.torque_conditioner_configuration_path = argv[3];
    configuration.output_directory = argv[4];
    if (!ParseFiniteDouble(argv[5],
                           &configuration.last_axle_stop_station_meters)) {
        std::fprintf(stderr, "last-axle stop station must be finite\n");
        return 2;
    }
    if (!ParseFiniteDouble(argv[6],
                           &configuration.maximum_simulation_time_seconds) ||
        !(configuration.maximum_simulation_time_seconds > 0.0)) {
        std::fprintf(stderr,
                     "maximum simulation time must be finite and positive\n");
        return 2;
    }
    if (!ParsePositiveInteger(argv[7],
                              &configuration.observation_decimation)) {
        std::fprintf(stderr,
                     "observation decimation must be a positive integer\n");
        return 2;
    }
    if (!ParseFiniteDouble(argv[8],
                           &configuration.communication_timeout_seconds) ||
        !(configuration.communication_timeout_seconds > 0.0)) {
        std::fprintf(stderr,
                     "communication timeout must be finite and positive\n");
        return 2;
    }
    if (argc == 10) {
        configuration.cpu_assignment = argv[9];
    }
    try {
        const auto summary =
            orvd::simpack_realtime::RunIrwSimpackRealtimeCruise(configuration);
        std::printf(
            "published %zu observations, %zu control events and %zu contact "
            "patch rows; final time %.6f s, last axle %.6f m, solver "
            "advance %.6f s, realtime loop %.6f s\n",
            summary.observation_count, summary.control_event_count,
            summary.contact_patch_row_count,
            summary.final_simulation_time_seconds,
            summary.final_last_axle_station_meters,
            summary.solver_advance_wall_time_seconds,
            summary.realtime_loop_wall_time_seconds);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "SIMPACK Realtime cruise failed: %s\n",
                     error.what());
        return 1;
    }
}
