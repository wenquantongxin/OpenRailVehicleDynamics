#include <charconv>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <string_view>

#include "irw_qualification_runner.h"

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
            "usage: orvd_irw_dynamics_qualification VEHICLE STARTUP LINE "
            "DATA_ROOT IRREGULARITY_ID_OR_NONE OUTPUT_DIRECTORY DURATION_NS "
            "SAMPLE_PERIOD_NS\n");
        return 2;
    }

    orvd::dynamics_qualification::IrwQualificationRunConfiguration config;
    config.vehicle_definition_path = argv[1];
    config.resolved_startup_state_path = argv[2];
    config.track_geometry_path = argv[3];
    config.orvd_data_root = argv[4];
    if (std::string_view(argv[5]) != "none") {
        config.track_irregularity_identifier = argv[5];
    }
    config.output_directory = argv[6];
    if (!ParsePositiveInteger(argv[7], &config.duration_nanoseconds) ||
        !ParsePositiveInteger(argv[8], &config.sample_period_nanoseconds)) {
        std::fprintf(stderr,
                     "duration and sample period must be positive integer "
                     "nanoseconds\n");
        return 2;
    }
    try {
        const auto summary =
            orvd::dynamics_qualification::RunIrwQualification(config);
        std::printf(
            "published %zu samples; advance %.6f s, observations %.6f s, "
            "endpoint diagnostics %.6f s, data+metadata write %.6f s\n",
            summary.sample_count, summary.advance_wall_seconds,
            summary.observation_wall_seconds,
            summary.endpoint_diagnostics_wall_seconds,
            summary.data_and_metadata_write_wall_seconds);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW qualification failed: %s\n", error.what());
        return 1;
    }
}
