#include <charconv>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <string_view>

#include "irw_passive_scenario_runs.h"
#include "qualification_cli_options.h"

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
    orvd::dynamics_qualification::internal::QualificationCliOptions options;
    try {
        options = orvd::dynamics_qualification::internal::ParseQualificationCliOptions(
            argc, argv, 9, true);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 2;
    }
    argc = static_cast<int>(options.positional_arguments.size());
    argv = options.positional_arguments.data();
    if (argc != 10 && argc != 11) {
        std::fprintf(
            stderr,
            "usage: orvd_irw_passive_scenario SCENARIO VEHICLE STARTUP LINE "
            "DATA_ROOT IRREGULARITY_ID_OR_NONE OUTPUT_DIRECTORY DURATION_NS "
            "SAMPLE_PERIOD_NS [TIME_INTEGRATOR_QUALIFICATION_CASE | "
            "--integration-config PATH] "
            "[--scene-record]\n"
            "SCENARIO: irw_r300_no_irregularity_v60_passive, "
            "irw_r300_aar5_v60_passive, irw_straight_aar5_v80_passive, "
            "irw_r600_aar5_v80_passive, irw_r800_aar5_v100_passive, "
            "irw_straight_aar6_v120_passive, "
            "irw_r1000_aar6_v120_passive, "
            "irw_straight_aar6_v160_passive, or "
            "irw_straight_erri_low_v200_passive\n");
        return 2;
    }
    orvd::dynamics_qualification::IrwPassiveScenarioRunConfiguration config;
    config.integration_config_path = options.integration_config_path;
    config.scenario_identifier = argv[1];
    config.vehicle_definition_path = argv[2];
    config.resolved_startup_state_path = argv[3];
    config.track_geometry_path = argv[4];
    config.orvd_data_root = argv[5];
    if (std::string_view(argv[6]) != "none") {
        config.track_irregularity_identifier = argv[6];
    }
    config.output_directory = argv[7];
    config.publish_scene_record = options.publish_scene_record;
    if (!ParsePositiveInteger(argv[8], &config.duration_nanoseconds) ||
        !ParsePositiveInteger(argv[9], &config.sample_period_nanoseconds)) {
        std::fprintf(stderr,
                     "duration and sample period must be positive integer "
                     "nanoseconds\n");
        return 2;
    }
    if (argc == 11) {
        config.time_integrator_qualification_case =
            orvd::dynamics_qualification::
                ParseTimeIntegratorQualificationCase(argv[10]);
        if (!config.time_integrator_qualification_case.has_value()) {
            std::fprintf(stderr,
                         "unknown time-integrator qualification case: %s\n",
                         argv[10]);
            return 2;
        }
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
        if (config.publish_scene_record) {
            std::printf("scene record: %zu frames in %.6f s\n",
                        summary.scene_record_frame_count,
                        summary.scene_record_wall_seconds);
        }
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW passive scenario run failed: %s\n",
                     error.what());
        return 1;
    }
}
