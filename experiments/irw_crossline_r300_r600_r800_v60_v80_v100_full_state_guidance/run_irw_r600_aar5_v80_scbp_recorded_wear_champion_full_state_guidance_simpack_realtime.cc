#include <cstdio>
#include <exception>

#include "irw_single_curve_full_state_guidance_schedule.h"
#include "irw_single_curve_guidance_simpack_realtime_run.h"

#ifndef ORVD_SIMPACK_ROOT
#error "ORVD_SIMPACK_ROOT must be defined for the SIMPACK Realtime experiment"
#endif

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(
            stderr,
            "usage: orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_"
            "simpack_realtime TEMPORARY_MODEL TORQUE_CONDITIONER "
            "OUTPUT_DIRECTORY\n");
        return 2;
    }
    try {
        using namespace
            orvd::experiments::irw_crossline_full_state_guidance;
        const IrwSingleCurveFullStateGuidanceSchedule schedule({
            .base_speed_meters_per_second = 80.0 / 3.6,
            .curve_radius_meters = 600.0,
            .profile = kScbpR600Profile,
        });
        const auto summary = RunIrwSingleCurveGuidanceSimpackRealtime(
            {
                .experiment_name =
                    "IRW R600 V80 SCBP recorded wear champion full-state "
                    "guidance SIMPACK Realtime",
                .active_track_name = "$Trk_Curve_R600m_80kmph",
                .control_profile_identity = schedule.profile().identity,
                .base_speed_meters_per_second = 80.0 / 3.6,
                .curve_radius_meters = 600.0,
                .terminal_minimum_axle_station_meters = 600.0,
                .maximum_observation_sample_count = 45'000,
                .recurrence_config = schedule.MakeRecurrenceConfig(),
                .operating_point_evaluator =
                    [schedule](const auto& stations) {
                        return schedule.EvaluateOperatingPoint(stations);
                    },
            },
            ORVD_SIMPACK_ROOT, argv[1], argv[2], argv[3]);
        std::printf(
            "published %zu observations and %zu control events through "
            "%.3f s; minimum axle station %.3f m; solver advance %.3f s\n",
            summary.observation_count, summary.control_event_count,
            summary.simulated_duration_seconds,
            summary.final_minimum_axle_station_meters,
            summary.solver_advance_wall_seconds);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "SIMPACK R600 SCBP experiment failed: %s\n",
                     error.what());
        return 1;
    }
}
