#include <cstdio>
#include <exception>

#include "irw_crossline_full_state_guidance_run.h"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(
            stderr,
            "usage: orvd_irw_crossline_r300_r600_r800_v60_v80_v100_"
            "full_state_guidance ORVD_DATA_ROOT OUTPUT_DIRECTORY\n");
        return 2;
    }
    try {
        const auto summary =
            orvd::experiments::irw_crossline_full_state_guidance::
                RunIrwCrosslineR300R600R800V60V80V100FullStateGuidance(
                    argv[1], argv[2]);
        std::printf(
            "published %zu observations, %zu contact patches and %zu "
            "control events; advance and synchronization %.6f s\n",
            summary.observation_count,
            summary.contact_patch_observation_count,
            summary.control_event_count,
            summary.advance_and_synchronization_wall_seconds);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW cross-line experiment failed: %s\n",
                     error.what());
        return 1;
    }
}
