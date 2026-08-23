#pragma once

/// @file
/// Shared wheel-speed recurrence constants for the IRW curve experiments.

#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {

inline constexpr double kControlSamplePeriodSeconds = 0.01;
inline constexpr double kRollingRadiusMeters = 0.43;
inline constexpr double kControllerCalibrationLateralHalfSpanMeters = 0.75;

/// Returns the common eight-wheel PI recurrence used by normal, better2 and
/// SCBP.  Route geometry and outer-loop gains remain in their experiment-local
/// operating-point sources.
[[nodiscard]] inline
control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig
MakeCurvatureDifferentialWheelSpeedRecurrenceConfig() {
    return control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig{
        .sample_period_seconds = kControlSamplePeriodSeconds,
        .rolling_radius_meters = kRollingRadiusMeters,
        .guidance_axle_signs = {1.0, 1.0, 1.0, 1.0},
        .guidance_wheel_signs =
            {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
        .lateral_integral_absolute_limit_meter_seconds = 0.5,
        .wheel_speed_pi =
            control::SampledFilteredPiConfig{
                .proportional_gain = 20000.0,
                .integral_time_seconds = 5000.0,
                .output_filter_time_constant_seconds = 0.0001,
                .integral_absolute_limit = 1000000.0,
                .raw_output_absolute_limit = 1000000.0,
            },
        .wheel_speed_pi_wheel_signs =
            {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
    };
}

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
