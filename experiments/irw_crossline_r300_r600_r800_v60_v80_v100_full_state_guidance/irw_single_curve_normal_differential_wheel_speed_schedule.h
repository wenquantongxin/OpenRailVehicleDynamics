#pragma once

/// @file
/// Experiment-local normal differential wheel-speed baseline for one curve.

#include "irw_curvature_differential_wheel_speed_control.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {

struct IrwSingleCurveNormalDifferentialWheelSpeedDefinition final {
    double base_speed_meters_per_second{};
    double curve_radius_meters{};
};

/// The historical normal baseline: one constant-speed planned-curvature
/// reference followed by eight independent wheel-speed PI recurrences. It has
/// no full-state outer-loop guidance gains.
class IrwSingleCurveNormalDifferentialWheelSpeedSchedule final {
   public:
    explicit IrwSingleCurveNormalDifferentialWheelSpeedSchedule(
        IrwSingleCurveNormalDifferentialWheelSpeedDefinition definition);

    [[nodiscard]]
    control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig
    MakeRecurrenceConfig() const;

    /// Evaluates the fixed-speed curve reference at each axle's own station.
    /// The planned curvature is zero through 50 m, rises linearly to 1/R over
    /// 50--100 m and remains 1/R thereafter, matching the source reference
    /// generator rather than the smoothed track shape.
    [[nodiscard]] control::IrwFullStateWheelSpeedGuidanceOperatingPoint
    EvaluateOperatingPoint(
        const control::IrwGuidanceAxleValues& axle_track_stations_meters)
        const;

   private:
    IrwSingleCurveNormalDifferentialWheelSpeedDefinition definition_;
};

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
