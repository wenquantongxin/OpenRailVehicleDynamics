#include <cstdio>
#include <exception>

#include "irw_single_curve_guidance_simpack_realtime_run.h"
#include "irw_single_curve_normal_differential_wheel_speed_schedule.h"

#ifndef ORVD_SIMPACK_ROOT
#error "ORVD_SIMPACK_ROOT must be defined for the SIMPACK Realtime experiment"
#endif

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(
            stderr,
            "usage: orvd_irw_r800_aar5_v100_normal_simpack_realtime "
            "TEMPORARY_MODEL TORQUE_CONDITIONER OUTPUT_DIRECTORY\n");
        return 2;
    }
    try {
        using namespace
            orvd::experiments::irw_crossline_full_state_guidance;
        const IrwSingleCurveNormalDifferentialWheelSpeedSchedule schedule({
            .base_speed_meters_per_second = 100.0 / 3.6,
            .curve_radius_meters = 800.0,
        });
        const auto summary = RunIrwSingleCurveGuidanceSimpackRealtime(
            {
                .experiment_name =
                    "IRW R800 V100 normal differential-wheel-speed SIMPACK Realtime",
                .active_track_name = "$Trk_Curve_R800m_100kmph",
                .control_profile_identity =
                    "normal_curvature_differential_wheel_speed",
                .base_speed_meters_per_second = 100.0 / 3.6,
                .curve_radius_meters = 800.0,
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
        std::fprintf(stderr, "SIMPACK R800 normal experiment failed: %s\n",
                     error.what());
        return 1;
    }
}
