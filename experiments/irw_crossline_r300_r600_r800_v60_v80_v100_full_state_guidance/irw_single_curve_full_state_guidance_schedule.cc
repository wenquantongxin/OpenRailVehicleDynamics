#include "irw_single_curve_full_state_guidance_schedule.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace orvd::experiments::irw_crossline_full_state_guidance {
namespace {

constexpr double kEntryStartTrackStationMeters = 50.0;
constexpr double kEntryEndTrackStationMeters = 100.0;
constexpr double kGuidanceEndTrackStationMeters = 600.0;
constexpr double kEntryYawRateFilterTimeConstantSeconds = 0.002;
constexpr double kEntryLateralVelocityFilterTimeConstantSeconds = 0.002;
constexpr double kEntryOuterFilterTimeConstantSeconds = 0.02;
constexpr control::IrwGuidanceWheelValues kWheelSideSigns{
    1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0};

[[nodiscard]] double EntryFraction(double station) {
    return std::clamp((station - kEntryStartTrackStationMeters) /
                          (kEntryEndTrackStationMeters -
                           kEntryStartTrackStationMeters),
                      0.0, 1.0);
}

void RequireFiniteStation(double station) {
    if (!std::isfinite(station)) {
        throw std::invalid_argument(
            "IRW single-curve full-state schedule: axle track station must "
            "be finite");
    }
}

void RequireFiniteAxleValues(const control::IrwGuidanceAxleValues& values,
                             const char* field) {
    if (!std::ranges::all_of(values, [](double value) {
            return std::isfinite(value);
        })) {
        throw std::invalid_argument(
            std::string("IRW single-curve full-state profile has non-finite ") +
            field);
    }
}

}  // namespace

IrwSingleCurveFullStateGuidanceSchedule::
    IrwSingleCurveFullStateGuidanceSchedule(
        IrwSingleCurveFullStateGuidanceDefinition definition)
    : definition_(definition) {
    if (!std::isfinite(definition_.base_speed_meters_per_second) ||
        !(definition_.base_speed_meters_per_second > 0.0) ||
        !std::isfinite(definition_.curve_radius_meters) ||
        !(definition_.curve_radius_meters > 0.0) ||
        definition_.profile.identity.empty() ||
        !std::isfinite(definition_.profile
                           .wheel_speed_difference_outer_filter_time_constant_seconds) ||
        !(definition_.profile
              .wheel_speed_difference_outer_filter_time_constant_seconds >=
          0.0) ||
        !std::isfinite(definition_.profile
                           .wheel_speed_difference_reference_absolute_limit_radians_per_second) ||
        !(definition_.profile
              .wheel_speed_difference_reference_absolute_limit_radians_per_second >=
          0.0)) {
        throw std::invalid_argument(
            "IRW single-curve full-state definition is incomplete or "
            "invalid");
    }
    RequireFiniteAxleValues(definition_.profile.feedforward_gains,
                            "feedforward gains");
    RequireFiniteAxleValues(definition_.profile.yaw_rate_feedback_gains,
                            "yaw-rate gains");
    RequireFiniteAxleValues(definition_.profile.yaw_feedback_gains,
                            "yaw gains");
    RequireFiniteAxleValues(
        definition_.profile.lateral_velocity_feedback_gains,
        "lateral-velocity gains");
    RequireFiniteAxleValues(
        definition_.profile.lateral_displacement_feedback_gains,
        "lateral-displacement gains");
    RequireFiniteAxleValues(
        definition_.profile.lateral_integral_feedback_gains,
        "lateral-integral gains");
    RequireFiniteAxleValues(
        definition_.profile.wheel_speed_difference_feedback_gains,
        "wheel-speed-difference gains");
    RequireFiniteAxleValues(
        definition_.profile.yaw_rate_filter_time_constants_seconds,
        "yaw-rate filter constants");
    RequireFiniteAxleValues(
        definition_.profile.lateral_velocity_filter_time_constants_seconds,
        "lateral-velocity filter constants");
    RequireFiniteAxleValues(definition_.profile.equilibrium_yaw_angles_radians,
                            "equilibrium yaw angles");
    RequireFiniteAxleValues(
        definition_.profile.equilibrium_lateral_displacements_meters,
        "equilibrium lateral displacements");
}

control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig
IrwSingleCurveFullStateGuidanceSchedule::MakeRecurrenceConfig() const {
    return MakeCurvatureDifferentialWheelSpeedRecurrenceConfig();
}

control::IrwFullStateWheelSpeedGuidanceOperatingPoint
IrwSingleCurveFullStateGuidanceSchedule::EvaluateOperatingPoint(
    const control::IrwGuidanceAxleValues& axle_track_stations_meters) const {
    control::IrwFullStateWheelSpeedGuidanceOperatingPoint operating_point;
    double average_station_meters = 0.0;
    for (std::size_t axle = 0; axle < control::kIrwGuidanceAxleCount; ++axle) {
        const double station = axle_track_stations_meters[axle];
        RequireFiniteStation(station);
        average_station_meters +=
            station / static_cast<double>(control::kIrwGuidanceAxleCount);
        const double fraction = EntryFraction(station);
        const double planned_curvature =
            fraction / definition_.curve_radius_meters;
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t wheel = 2 * axle + side;
            operating_point
                .base_wheel_speed_references_meters_per_second[wheel] =
                definition_.base_speed_meters_per_second *
                (1.0 + kWheelSideSigns[wheel] *
                           kControllerCalibrationLateralHalfSpanMeters *
                           planned_curvature);
        }

        operating_point.feedforward_gains[axle] =
            fraction * definition_.profile.feedforward_gains[axle];
        operating_point.yaw_rate_feedback_gains[axle] =
            fraction * definition_.profile.yaw_rate_feedback_gains[axle];
        operating_point.yaw_feedback_gains[axle] =
            fraction * definition_.profile.yaw_feedback_gains[axle];
        operating_point.lateral_velocity_feedback_gains[axle] =
            fraction *
            definition_.profile.lateral_velocity_feedback_gains[axle];
        operating_point.lateral_displacement_feedback_gains[axle] =
            fraction *
            definition_.profile.lateral_displacement_feedback_gains[axle];
        operating_point.lateral_integral_feedback_gains[axle] =
            fraction *
            definition_.profile.lateral_integral_feedback_gains[axle];
        operating_point.wheel_speed_difference_feedback_gains[axle] =
            fraction *
            definition_.profile.wheel_speed_difference_feedback_gains[axle];
        operating_point.yaw_rate_filter_time_constants_seconds[axle] =
            std::lerp(kEntryYawRateFilterTimeConstantSeconds,
                      definition_.profile
                          .yaw_rate_filter_time_constants_seconds[axle],
                      fraction);
        operating_point.lateral_velocity_filter_time_constants_seconds[axle] =
            std::lerp(kEntryLateralVelocityFilterTimeConstantSeconds,
                      definition_.profile
                          .lateral_velocity_filter_time_constants_seconds[axle],
                      fraction);
        operating_point.equilibrium_yaw_angles_radians[axle] =
            fraction *
            definition_.profile.equilibrium_yaw_angles_radians[axle];
        operating_point.equilibrium_lateral_displacements_meters[axle] =
            fraction * definition_.profile
                           .equilibrium_lateral_displacements_meters[axle];
        operating_point.guidance_active[axle] =
            station >= kEntryStartTrackStationMeters &&
            station <= kGuidanceEndTrackStationMeters;
    }

    const double scalar_fraction = EntryFraction(average_station_meters);
    operating_point
        .wheel_speed_difference_outer_filter_time_constants_seconds
        .fill(std::lerp(
            kEntryOuterFilterTimeConstantSeconds,
            definition_.profile
                .wheel_speed_difference_outer_filter_time_constant_seconds,
            scalar_fraction));
    operating_point
        .wheel_speed_difference_reference_absolute_limits_radians_per_second
        .fill(definition_.profile
                  .wheel_speed_difference_reference_absolute_limit_radians_per_second);
    return operating_point;
}

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
