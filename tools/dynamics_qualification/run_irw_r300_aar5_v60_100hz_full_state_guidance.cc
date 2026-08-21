#include <charconv>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <string_view>

#include "irw_r300_aar5_v60_100hz_full_state_guidance_run.h"

namespace {

bool ParsePositiveInteger(std::string_view text, std::int64_t* output) {
    if (output == nullptr || text.empty()) {
        return false;
    }
    std::int64_t value{};
    const auto [end, error] =
        std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() ||
        value <= 0) {
        return false;
    }
    *output = value;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 9) {
        std::fprintf(
            stderr,
            "usage: orvd_irw_r300_aar5_v60_100hz_full_state_guidance "
            "VEHICLE STARTUP LINE DATA_ROOT CONTROLLER CONDITIONER "
            "OUTPUT_DIRECTORY DURATION_NS\n");
        return 2;
    }
    orvd::dynamics_qualification::
        IrwR300Aar5V60At100HzFullStateGuidanceRunConfiguration configuration;
    configuration.vehicle_definition_path = argv[1];
    configuration.resolved_startup_state_path = argv[2];
    configuration.track_geometry_path = argv[3];
    configuration.orvd_data_root = argv[4];
    configuration.controller_configuration_path = argv[5];
    configuration.torque_conditioner_configuration_path = argv[6];
    configuration.output_directory = argv[7];
    if (!ParsePositiveInteger(argv[8], &configuration.duration_nanoseconds)) {
        std::fprintf(stderr,
                     "duration must be positive integer nanoseconds\n");
        return 2;
    }
    try {
        const auto summary =
            orvd::dynamics_qualification::
                RunIrwR300Aar5V60At100HzFullStateGuidance(configuration);
        std::printf(
            "published %zu observations and %zu control audits; %zu holds, "
            "%zu backend synchronizations; advance and synchronization "
            "%.6f s\n",
            summary.observation_count, summary.control_audit_count,
            summary.positive_hold_interval_count,
            summary.backend_synchronization_count,
            summary.advance_and_synchronization_wall_seconds);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr,
                     "IRW R300/AAR5/60 km/h 100 Hz full-state guidance run "
                     "failed: %s\n",
                     error.what());
        return 1;
    }
}
