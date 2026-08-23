#include <cstdio>
#include <exception>

#include "irw_guidance_experiment_run.h"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(
            stderr,
            "usage: orvd_irw_r600_aar5_v80_normal_differential_wheel_speed "
            "ORVD_DATA_ROOT OUTPUT_DIRECTORY\n");
        return 2;
    }
    try {
        const auto summary =
            orvd::experiments::irw_crossline_full_state_guidance::
                RunIrwR600Aar5V80NormalDifferentialWheelSpeed(argv[1],
                                                               argv[2]);
        std::printf(
            "published %zu observations, %zu contact patches and %zu "
            "control events through %.3f s; final axle stations "
            "%.3f/%.3f/%.3f/%.3f m\n",
            summary.observation_count,
            summary.contact_patch_observation_count,
            summary.control_event_count,
            summary.simulated_duration_seconds,
            summary.final_axle_track_stations_meters[0],
            summary.final_axle_track_stations_meters[1],
            summary.final_axle_track_stations_meters[2],
            summary.final_axle_track_stations_meters[3]);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW R600 normal experiment failed: %s\n",
                     error.what());
        return 1;
    }
}
