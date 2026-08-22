#pragma once

/// @file
/// Strongly typed operating-point schedule for the IRW cross-line experiment.

#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {

inline constexpr double kControlSamplePeriodSeconds = 0.01;
inline constexpr double kRollingRadiusMeters = 0.43;
inline constexpr double kControllerCalibrationLateralHalfSpanMeters = 0.75;
inline constexpr double kGuidanceStartTrackStationMeters = 50.0;
inline constexpr double kGuidanceEndTrackStationMeters = 2396.9;

/// Speed and planned curvature evaluated at one axle's track station.
struct IrwCrosslineSpeedAndCurvature {
    double base_speed_meters_per_second{0.0};
    double planned_curvature_radians_per_meter{0.0};
};

/// Stateless owner of the experiment's complete, compile-time schedule.
class IrwCrosslineOperatingPointSchedule {
   public:
    /// Evaluate the speed schedule in v-squared space and the independent
    /// planned curvature schedule at one axle's station.  Values outside the
    /// schedule are clamped to the corresponding endpoint.
    [[nodiscard]] IrwCrosslineSpeedAndCurvature EvaluateSpeedAndCurvature(
        double axle_track_station_meters) const;

    /// Build the line-independent recurrence constants used by this
    /// experiment.
    [[nodiscard]]
    control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig
    MakeRecurrenceConfig() const;

    /// Evaluate one already-resolved operating point.
    ///
    /// Axle-valued schedules are queried at each axle's own station.  The two
    /// scalar outer-loop schedules are queried at the arithmetic mean station
    /// of all four axles, matching the source controller's schedule semantics.
    [[nodiscard]] control::IrwFullStateWheelSpeedGuidanceOperatingPoint
    EvaluateOperatingPoint(
        const control::IrwGuidanceAxleValues& axle_track_stations_meters)
        const;
};

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
