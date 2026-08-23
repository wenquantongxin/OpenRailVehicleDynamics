#pragma once

/// @file
/// Experiment-local full-state guidance schedule for one isolated curve.

#include "irw_curvature_differential_wheel_speed_control.h"
#include "irw_curve_full_state_guidance_profiles.h"

namespace orvd::experiments::irw_crossline_full_state_guidance {

struct IrwSingleCurveFullStateGuidanceDefinition final {
    double base_speed_meters_per_second{};
    double curve_radius_meters{};
    IrwCurveFullStateGuidanceProfile profile;
};

/// Applies one value-owned curve profile without embedding route identities in
/// the public ORVD controller. Each axle builds its own planned curvature,
/// gains and equilibrium values from zero over the 50--100 m entry transition.
class IrwSingleCurveFullStateGuidanceSchedule final {
   public:
    explicit IrwSingleCurveFullStateGuidanceSchedule(
        IrwSingleCurveFullStateGuidanceDefinition definition);

    [[nodiscard]]
    control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig
    MakeRecurrenceConfig() const;

    [[nodiscard]] control::IrwFullStateWheelSpeedGuidanceOperatingPoint
    EvaluateOperatingPoint(
        const control::IrwGuidanceAxleValues& axle_track_stations_meters)
        const;

    [[nodiscard]] const IrwCurveFullStateGuidanceProfile& profile()
        const noexcept {
        return definition_.profile;
    }

   private:
    IrwSingleCurveFullStateGuidanceDefinition definition_;
};

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
