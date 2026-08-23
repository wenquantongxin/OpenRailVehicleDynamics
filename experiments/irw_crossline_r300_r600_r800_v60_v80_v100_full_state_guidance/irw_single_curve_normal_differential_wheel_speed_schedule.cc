#include "irw_single_curve_normal_differential_wheel_speed_schedule.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace orvd::experiments::irw_crossline_full_state_guidance {
namespace {

constexpr double kReferenceRampStartTrackStationMeters = 50.0;
constexpr double kReferenceRampEndTrackStationMeters = 100.0;
constexpr double kGuidanceStartTrackStationMeters = 50.0;
constexpr double kGuidanceEndTrackStationMeters = 600.0;
constexpr control::IrwGuidanceWheelValues kWheelSideSigns{
    1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0};

void RequireFiniteStation(double station) {
    if (!std::isfinite(station)) {
        throw std::invalid_argument(
            "IRW single-curve normal wheel-speed schedule: axle track station "
            "must be finite");
    }
}

}  // namespace

control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig
IrwSingleCurveNormalDifferentialWheelSpeedSchedule::MakeRecurrenceConfig()
    const {
    return MakeCurvatureDifferentialWheelSpeedRecurrenceConfig();
}

IrwSingleCurveNormalDifferentialWheelSpeedSchedule::
    IrwSingleCurveNormalDifferentialWheelSpeedSchedule(
        IrwSingleCurveNormalDifferentialWheelSpeedDefinition definition)
    : definition_(definition) {
    if (!std::isfinite(definition_.base_speed_meters_per_second) ||
        !(definition_.base_speed_meters_per_second > 0.0) ||
        !std::isfinite(definition_.curve_radius_meters) ||
        !(definition_.curve_radius_meters > 0.0)) {
        throw std::invalid_argument(
            "IRW single-curve normal wheel-speed definition must state a "
            "positive finite speed and radius");
    }
}

control::IrwFullStateWheelSpeedGuidanceOperatingPoint
IrwSingleCurveNormalDifferentialWheelSpeedSchedule::EvaluateOperatingPoint(
    const control::IrwGuidanceAxleValues& axle_track_stations_meters) const {
    control::IrwFullStateWheelSpeedGuidanceOperatingPoint operating_point;
    for (std::size_t axle = 0; axle < control::kIrwGuidanceAxleCount; ++axle) {
        const double station = axle_track_stations_meters[axle];
        RequireFiniteStation(station);
        const double ramp_fraction = std::clamp(
            (station - kReferenceRampStartTrackStationMeters) /
                (kReferenceRampEndTrackStationMeters -
                 kReferenceRampStartTrackStationMeters),
            0.0, 1.0);
        const double planned_curvature =
            ramp_fraction / definition_.curve_radius_meters;
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t wheel = 2 * axle + side;
            operating_point
                .base_wheel_speed_references_meters_per_second[wheel] =
                definition_.base_speed_meters_per_second *
                (1.0 + kWheelSideSigns[wheel] *
                           kControllerCalibrationLateralHalfSpanMeters *
                           planned_curvature);
        }
        operating_point.guidance_active[axle] =
            station >= kGuidanceStartTrackStationMeters &&
            station <= kGuidanceEndTrackStationMeters;
    }

    operating_point.yaw_rate_filter_time_constants_seconds.fill(0.002);
    operating_point.lateral_velocity_filter_time_constants_seconds.fill(
        0.002);
    operating_point
        .wheel_speed_difference_outer_filter_time_constants_seconds.fill(
            0.02);
    operating_point
        .wheel_speed_difference_reference_absolute_limits_radians_per_second
        .fill(1.0);
    return operating_point;
}

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
