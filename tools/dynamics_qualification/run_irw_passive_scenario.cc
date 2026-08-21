#include <charconv>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <string_view>

#include "irw_passive_scenario_runs.h"

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
    if (argc != 10) {
        std::fprintf(
            stderr,
            "usage: orvd_irw_passive_scenario SCENARIO VEHICLE STARTUP LINE "
            "DATA_ROOT IRREGULARITY_ID_OR_NONE OUTPUT_DIRECTORY DURATION_NS "
            "SAMPLE_PERIOD_NS\n"
            "SCENARIO: irw_r300_no_irregularity_v60_passive, "
            "irw_r300_aar5_v60_passive, irw_straight_aar5_v80_passive, "
            "irw_r600_aar5_v80_passive, irw_straight_aar6_v120_passive, "
            "or irw_r1000_aar6_v120_passive\n");
        return 2;
    }

    orvd::dynamics_qualification::IrwPassiveScenarioRunConfiguration config;
    config.scenario_identifier = argv[1];
    config.vehicle_definition_path = argv[2];
    config.resolved_startup_state_path = argv[3];
    config.track_geometry_path = argv[4];
    config.orvd_data_root = argv[5];
    if (std::string_view(argv[6]) != "none") {
        config.track_irregularity_identifier = argv[6];
    }
    config.output_directory = argv[7];
    if (!ParsePositiveInteger(argv[8], &config.duration_nanoseconds) ||
        !ParsePositiveInteger(argv[9], &config.sample_period_nanoseconds)) {
        std::fprintf(stderr,
                     "duration and sample period must be positive integer "
                     "nanoseconds\n");
        return 2;
    }
    try {
        const auto summary =
            orvd::dynamics_qualification::RunIrwPassiveScenario(config);
        std::printf(
            "published %zu samples; advance %.6f s, observations %.6f s, "
            "endpoint diagnostics %.6f s, data+metadata write %.6f s\n",
            summary.sample_count, summary.advance_wall_seconds,
            summary.observation_wall_seconds,
            summary.endpoint_diagnostics_wall_seconds,
            summary.data_and_metadata_write_wall_seconds);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW passive scenario run failed: %s\n",
                     error.what());
        return 1;
    }
}
