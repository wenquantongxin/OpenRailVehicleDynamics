#include "irw_crossline_operating_point_schedule.h"

#include "irw_curve_full_state_guidance_profiles.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace orvd::experiments::irw_crossline_full_state_guidance {
namespace {

using control::IrwGuidanceAxleValues;
using control::IrwGuidanceWheelValues;

constexpr std::size_t kSpeedScheduleNodeCount = 10;
constexpr std::size_t kGuidanceScheduleNodeCount = 11;
using GuidanceAxleTable =
    std::array<IrwGuidanceAxleValues, kGuidanceScheduleNodeCount>;

constexpr std::array<double, kSpeedScheduleNodeCount>
    kSpeedScheduleTrackStationsMeters{
        0.0, 50.0, 100.0, 600.0, 708.0,
        1208.0, 1346.9, 1546.9, 1596.9, 2396.9};
constexpr std::array<double, kSpeedScheduleNodeCount>
    kBaseSpeedsMetersPerSecond{
        60.0 / 3.6, 60.0 / 3.6, 60.0 / 3.6, 60.0 / 3.6,
        80.0 / 3.6, 80.0 / 3.6, 100.0 / 3.6, 100.0 / 3.6,
        100.0 / 3.6, 100.0 / 3.6};
constexpr std::array<double, kSpeedScheduleNodeCount>
    kPlannedCurvaturesRadiansPerMeter{
        0.0, 0.0, 1.0 / 300.0, 1.0 / 300.0, 1.0 / 600.0,
        1.0 / 600.0, 1.0 / 800.0, 1.0 / 800.0, 0.0, 0.0};

constexpr std::array<double, kGuidanceScheduleNodeCount>
    kGuidanceScheduleTrackStationsMeters{
        0.0, 50.0, 100.0, 600.0, 640.0, 708.0,
        1208.0, 1346.9, 1546.9, 1596.9, 2396.9};

constexpr GuidanceAxleTable kFeedforwardGains{{
    {}, {},
    kBetter2R300Profile.feedforward_gains,
    kBetter2R300Profile.feedforward_gains,
    kBetter2R300Profile.feedforward_gains,
    kBetter2R600Profile.feedforward_gains,
    kBetter2R600Profile.feedforward_gains,
    kBetter2R800Profile.feedforward_gains,
    kBetter2R800Profile.feedforward_gains,
    {}, {},
}};

constexpr GuidanceAxleTable kYawRateFeedbackGains{{
    {}, {},
    kBetter2R300Profile.yaw_rate_feedback_gains,
    kBetter2R300Profile.yaw_rate_feedback_gains,
    kBetter2R300Profile.yaw_rate_feedback_gains,
    kBetter2R600Profile.yaw_rate_feedback_gains,
    kBetter2R600Profile.yaw_rate_feedback_gains,
    kBetter2R800Profile.yaw_rate_feedback_gains,
    kBetter2R800Profile.yaw_rate_feedback_gains,
    {-0.5, -0.5, -0.5, -0.5},
    {-0.5, -0.5, -0.5, -0.5},
}};

constexpr GuidanceAxleTable kYawFeedbackGains{{
    {}, {},
    kBetter2R300Profile.yaw_feedback_gains,
    kBetter2R300Profile.yaw_feedback_gains,
    kBetter2R300Profile.yaw_feedback_gains,
    kBetter2R600Profile.yaw_feedback_gains,
    kBetter2R600Profile.yaw_feedback_gains,
    kBetter2R800Profile.yaw_feedback_gains,
    kBetter2R800Profile.yaw_feedback_gains,
    {-700.0, -300.0, -600.0, -500.0},
    {-700.0, -300.0, -600.0, -500.0},
}};

constexpr GuidanceAxleTable kLateralVelocityFeedbackGains{{
    {}, {},
    kBetter2R300Profile.lateral_velocity_feedback_gains,
    kBetter2R300Profile.lateral_velocity_feedback_gains,
    kBetter2R300Profile.lateral_velocity_feedback_gains,
    kBetter2R600Profile.lateral_velocity_feedback_gains,
    kBetter2R600Profile.lateral_velocity_feedback_gains,
    kBetter2R800Profile.lateral_velocity_feedback_gains,
    kBetter2R800Profile.lateral_velocity_feedback_gains,
    {-1.0, -1.0, -1.0, -1.0},
    {-1.0, -1.0, -1.0, -1.0},
}};

constexpr GuidanceAxleTable kLateralDisplacementFeedbackGains{{
    {}, {},
    kBetter2R300Profile.lateral_displacement_feedback_gains,
    kBetter2R300Profile.lateral_displacement_feedback_gains,
    kBetter2R300Profile.lateral_displacement_feedback_gains,
    kBetter2R600Profile.lateral_displacement_feedback_gains,
    kBetter2R600Profile.lateral_displacement_feedback_gains,
    kBetter2R800Profile.lateral_displacement_feedback_gains,
    kBetter2R800Profile.lateral_displacement_feedback_gains,
    {-200.0, -200.0, -200.0, -200.0},
    {-200.0, -200.0, -200.0, -200.0},
}};

constexpr GuidanceAxleTable kLateralIntegralFeedbackGains{{
    {}, {},
    kBetter2R300Profile.lateral_integral_feedback_gains,
    kBetter2R300Profile.lateral_integral_feedback_gains,
    kBetter2R300Profile.lateral_integral_feedback_gains,
    kBetter2R600Profile.lateral_integral_feedback_gains,
    kBetter2R600Profile.lateral_integral_feedback_gains,
    kBetter2R800Profile.lateral_integral_feedback_gains,
    kBetter2R800Profile.lateral_integral_feedback_gains,
    {}, {},
}};

constexpr GuidanceAxleTable kWheelSpeedDifferenceFeedbackGains{{
    {}, {},
    kBetter2R300Profile.wheel_speed_difference_feedback_gains,
    kBetter2R300Profile.wheel_speed_difference_feedback_gains,
    kBetter2R300Profile.wheel_speed_difference_feedback_gains,
    kBetter2R600Profile.wheel_speed_difference_feedback_gains,
    kBetter2R600Profile.wheel_speed_difference_feedback_gains,
    kBetter2R800Profile.wheel_speed_difference_feedback_gains,
    kBetter2R800Profile.wheel_speed_difference_feedback_gains,
    {}, {},
}};

constexpr GuidanceAxleTable kYawRateFilterTimeConstantsSeconds{{
    kBetter2R300Profile.yaw_rate_filter_time_constants_seconds,
    kBetter2R300Profile.yaw_rate_filter_time_constants_seconds,
    kBetter2R300Profile.yaw_rate_filter_time_constants_seconds,
    kBetter2R300Profile.yaw_rate_filter_time_constants_seconds,
    kBetter2R300Profile.yaw_rate_filter_time_constants_seconds,
    kBetter2R600Profile.yaw_rate_filter_time_constants_seconds,
    kBetter2R600Profile.yaw_rate_filter_time_constants_seconds,
    kBetter2R800Profile.yaw_rate_filter_time_constants_seconds,
    kBetter2R800Profile.yaw_rate_filter_time_constants_seconds,
    {0.01, 0.01, 0.01, 0.01},
    {0.01, 0.01, 0.01, 0.01},
}};

constexpr GuidanceAxleTable kLateralVelocityFilterTimeConstantsSeconds{{
    kBetter2R300Profile.lateral_velocity_filter_time_constants_seconds,
    kBetter2R300Profile.lateral_velocity_filter_time_constants_seconds,
    kBetter2R300Profile.lateral_velocity_filter_time_constants_seconds,
    kBetter2R300Profile.lateral_velocity_filter_time_constants_seconds,
    kBetter2R300Profile.lateral_velocity_filter_time_constants_seconds,
    kBetter2R600Profile.lateral_velocity_filter_time_constants_seconds,
    kBetter2R600Profile.lateral_velocity_filter_time_constants_seconds,
    kBetter2R800Profile.lateral_velocity_filter_time_constants_seconds,
    kBetter2R800Profile.lateral_velocity_filter_time_constants_seconds,
    {0.01, 0.01, 0.01, 0.01},
    {0.01, 0.01, 0.01, 0.01},
}};

constexpr GuidanceAxleTable kEquilibriumYawAnglesRadians{{
    {}, {},
    kBetter2R300Profile.equilibrium_yaw_angles_radians,
    kBetter2R300Profile.equilibrium_yaw_angles_radians,
    kBetter2R300Profile.equilibrium_yaw_angles_radians,
    kBetter2R600Profile.equilibrium_yaw_angles_radians,
    kBetter2R600Profile.equilibrium_yaw_angles_radians,
    kBetter2R800Profile.equilibrium_yaw_angles_radians,
    kBetter2R800Profile.equilibrium_yaw_angles_radians,
    {}, {},
}};

constexpr GuidanceAxleTable kEquilibriumLateralDisplacementsMeters{{
    {}, {},
    kBetter2R300Profile.equilibrium_lateral_displacements_meters,
    kBetter2R300Profile.equilibrium_lateral_displacements_meters,
    kBetter2R300Profile.equilibrium_lateral_displacements_meters,
    kBetter2R600Profile.equilibrium_lateral_displacements_meters,
    kBetter2R600Profile.equilibrium_lateral_displacements_meters,
    kBetter2R800Profile.equilibrium_lateral_displacements_meters,
    kBetter2R800Profile.equilibrium_lateral_displacements_meters,
    {}, {},
}};

constexpr std::array<double, kGuidanceScheduleNodeCount>
    kWheelSpeedDifferenceReferenceAbsoluteLimitsRadiansPerSecond{
        kBetter2R300Profile
            .wheel_speed_difference_reference_absolute_limit_radians_per_second,
        kBetter2R300Profile
            .wheel_speed_difference_reference_absolute_limit_radians_per_second,
        kBetter2R300Profile
            .wheel_speed_difference_reference_absolute_limit_radians_per_second,
        kBetter2R300Profile
            .wheel_speed_difference_reference_absolute_limit_radians_per_second,
        kBetter2R300Profile
            .wheel_speed_difference_reference_absolute_limit_radians_per_second,
        kBetter2R600Profile
            .wheel_speed_difference_reference_absolute_limit_radians_per_second,
        kBetter2R600Profile
            .wheel_speed_difference_reference_absolute_limit_radians_per_second,
        kBetter2R800Profile
            .wheel_speed_difference_reference_absolute_limit_radians_per_second,
        kBetter2R800Profile
            .wheel_speed_difference_reference_absolute_limit_radians_per_second,
        0.5, 0.5};

constexpr std::array<double, kGuidanceScheduleNodeCount>
    kWheelSpeedDifferenceOuterFilterTimeConstantsSeconds{
        0.02, 0.02,
        kBetter2R300Profile
            .wheel_speed_difference_outer_filter_time_constant_seconds,
        kBetter2R300Profile
            .wheel_speed_difference_outer_filter_time_constant_seconds,
        kBetter2R300Profile
            .wheel_speed_difference_outer_filter_time_constant_seconds,
        kBetter2R600Profile
            .wheel_speed_difference_outer_filter_time_constant_seconds,
        kBetter2R600Profile
            .wheel_speed_difference_outer_filter_time_constant_seconds,
        kBetter2R800Profile
            .wheel_speed_difference_outer_filter_time_constant_seconds,
        kBetter2R800Profile
            .wheel_speed_difference_outer_filter_time_constant_seconds,
        0.03, 0.03};

constexpr IrwGuidanceWheelValues kSpeedReferenceWheelSideSigns{
    1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0};
constexpr bool IsFinite(double value) {
    return value >= -std::numeric_limits<double>::max() &&
           value <= std::numeric_limits<double>::max();
}

template <typename Value>
constexpr bool AllFinite(const Value& value) {
    if constexpr (std::is_same_v<Value, double>) {
        return IsFinite(value);
    } else {
        for (const auto& entry : value) {
            if (!AllFinite(entry)) {
                return false;
            }
        }
        return true;
    }
}

template <std::size_t Size>
constexpr bool StrictlyIncreasing(const std::array<double, Size>& values) {
    for (std::size_t index = 1; index < Size; ++index) {
        if (!(values[index] > values[index - 1])) {
            return false;
        }
    }
    return true;
}

template <std::size_t Size>
constexpr bool AllNonnegative(const std::array<double, Size>& values) {
    for (double value : values) {
        if (value < 0.0) {
            return false;
        }
    }
    return true;
}

static_assert(kBaseSpeedsMetersPerSecond.size() ==
              kSpeedScheduleTrackStationsMeters.size());
static_assert(kPlannedCurvaturesRadiansPerMeter.size() ==
              kSpeedScheduleTrackStationsMeters.size());
static_assert(kFeedforwardGains.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kYawRateFeedbackGains.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kYawFeedbackGains.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kLateralVelocityFeedbackGains.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kLateralDisplacementFeedbackGains.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kLateralIntegralFeedbackGains.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kWheelSpeedDifferenceFeedbackGains.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kYawRateFilterTimeConstantsSeconds.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kLateralVelocityFilterTimeConstantsSeconds.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kEquilibriumYawAnglesRadians.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(kEquilibriumLateralDisplacementsMeters.size() ==
              kGuidanceScheduleTrackStationsMeters.size());
static_assert(
    kWheelSpeedDifferenceReferenceAbsoluteLimitsRadiansPerSecond.size() ==
    kGuidanceScheduleTrackStationsMeters.size());
static_assert(
    kWheelSpeedDifferenceOuterFilterTimeConstantsSeconds.size() ==
    kGuidanceScheduleTrackStationsMeters.size());
static_assert(StrictlyIncreasing(kSpeedScheduleTrackStationsMeters));
static_assert(StrictlyIncreasing(kGuidanceScheduleTrackStationsMeters));
static_assert(AllFinite(kSpeedScheduleTrackStationsMeters));
static_assert(AllFinite(kBaseSpeedsMetersPerSecond));
static_assert(AllFinite(kPlannedCurvaturesRadiansPerMeter));
static_assert(AllFinite(kGuidanceScheduleTrackStationsMeters));
static_assert(AllFinite(kFeedforwardGains));
static_assert(AllFinite(kYawRateFeedbackGains));
static_assert(AllFinite(kYawFeedbackGains));
static_assert(AllFinite(kLateralVelocityFeedbackGains));
static_assert(AllFinite(kLateralDisplacementFeedbackGains));
static_assert(AllFinite(kLateralIntegralFeedbackGains));
static_assert(AllFinite(kWheelSpeedDifferenceFeedbackGains));
static_assert(AllFinite(kYawRateFilterTimeConstantsSeconds));
static_assert(AllFinite(kLateralVelocityFilterTimeConstantsSeconds));
static_assert(AllFinite(kEquilibriumYawAnglesRadians));
static_assert(AllFinite(kEquilibriumLateralDisplacementsMeters));
static_assert(AllFinite(
    kWheelSpeedDifferenceReferenceAbsoluteLimitsRadiansPerSecond));
static_assert(AllFinite(
    kWheelSpeedDifferenceOuterFilterTimeConstantsSeconds));
static_assert(AllNonnegative(kBaseSpeedsMetersPerSecond));
static_assert(AllNonnegative(
    kWheelSpeedDifferenceReferenceAbsoluteLimitsRadiansPerSecond));
static_assert(AllNonnegative(
    kWheelSpeedDifferenceOuterFilterTimeConstantsSeconds));

template <std::size_t Size>
std::size_t LowerIntervalIndex(const std::array<double, Size>& stations,
                               double station) {
    if (station <= stations.front()) {
        return 0;
    }
    if (station >= stations.back()) {
        return Size - 2;
    }
    const auto upper = std::upper_bound(stations.begin(), stations.end(),
                                        station);
    return static_cast<std::size_t>(upper - stations.begin() - 1);
}

template <std::size_t Size>
double InterpolateScalar(const std::array<double, Size>& stations,
                         const std::array<double, Size>& values,
                         double station) {
    if (station <= stations.front()) {
        return values.front();
    }
    if (station >= stations.back()) {
        return values.back();
    }
    const std::size_t lower = LowerIntervalIndex(stations, station);
    const double fraction =
        (station - stations[lower]) /
        (stations[lower + 1] - stations[lower]);
    return std::lerp(values[lower], values[lower + 1], fraction);
}

double InterpolateGuidanceAxleValue(const GuidanceAxleTable& values,
                                    std::size_t axle,
                                    double station) {
    if (station <= kGuidanceScheduleTrackStationsMeters.front()) {
        return values.front()[axle];
    }
    if (station >= kGuidanceScheduleTrackStationsMeters.back()) {
        return values.back()[axle];
    }
    const std::size_t lower = LowerIntervalIndex(
        kGuidanceScheduleTrackStationsMeters, station);
    const double fraction =
        (station - kGuidanceScheduleTrackStationsMeters[lower]) /
        (kGuidanceScheduleTrackStationsMeters[lower + 1] -
         kGuidanceScheduleTrackStationsMeters[lower]);
    return std::lerp(values[lower][axle], values[lower + 1][axle],
                     fraction);
}

void RequireFiniteStation(double station) {
    if (!std::isfinite(station)) {
        throw std::invalid_argument(
            "IRW cross-line operating-point schedule: axle track station "
            "must be finite");
    }
}

}  // namespace

IrwCrosslineSpeedAndCurvature
IrwCrosslineOperatingPointSchedule::EvaluateSpeedAndCurvature(
    double axle_track_station_meters) const {
    RequireFiniteStation(axle_track_station_meters);
    std::array<double, kSpeedScheduleNodeCount> squared_speeds{};
    std::transform(kBaseSpeedsMetersPerSecond.begin(),
                   kBaseSpeedsMetersPerSecond.end(), squared_speeds.begin(),
                   [](double speed) { return speed * speed; });
    const double squared_speed = InterpolateScalar(
        kSpeedScheduleTrackStationsMeters, squared_speeds,
        axle_track_station_meters);
    return IrwCrosslineSpeedAndCurvature{
        .base_speed_meters_per_second =
            std::sqrt(std::max(0.0, squared_speed)),
        .planned_curvature_radians_per_meter = InterpolateScalar(
            kSpeedScheduleTrackStationsMeters,
            kPlannedCurvaturesRadiansPerMeter,
            axle_track_station_meters),
    };
}

control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig
IrwCrosslineOperatingPointSchedule::MakeRecurrenceConfig() const {
    return MakeCurvatureDifferentialWheelSpeedRecurrenceConfig();
}

control::IrwFullStateWheelSpeedGuidanceOperatingPoint
IrwCrosslineOperatingPointSchedule::EvaluateOperatingPoint(
    const IrwGuidanceAxleValues& axle_track_stations_meters) const {
    for (double station : axle_track_stations_meters) {
        RequireFiniteStation(station);
    }

    control::IrwFullStateWheelSpeedGuidanceOperatingPoint operating_point;
    double average_station_meters = 0.0;
    for (std::size_t axle = 0; axle < control::kIrwGuidanceAxleCount; ++axle) {
        const double station = axle_track_stations_meters[axle];
        average_station_meters +=
            station / static_cast<double>(control::kIrwGuidanceAxleCount);

        const auto speed_and_curvature =
            EvaluateSpeedAndCurvature(station);
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t wheel = 2 * axle + side;
            operating_point
                .base_wheel_speed_references_meters_per_second[wheel] =
                speed_and_curvature.base_speed_meters_per_second *
                (1.0 + kSpeedReferenceWheelSideSigns[wheel] *
                           kControllerCalibrationLateralHalfSpanMeters *
                           speed_and_curvature
                               .planned_curvature_radians_per_meter);
        }

        operating_point.feedforward_gains[axle] =
            InterpolateGuidanceAxleValue(kFeedforwardGains, axle, station);
        operating_point.yaw_rate_feedback_gains[axle] =
            InterpolateGuidanceAxleValue(kYawRateFeedbackGains, axle, station);
        operating_point.yaw_feedback_gains[axle] =
            InterpolateGuidanceAxleValue(kYawFeedbackGains, axle, station);
        operating_point.lateral_velocity_feedback_gains[axle] =
            InterpolateGuidanceAxleValue(kLateralVelocityFeedbackGains, axle,
                                         station);
        operating_point.lateral_displacement_feedback_gains[axle] =
            InterpolateGuidanceAxleValue(
                kLateralDisplacementFeedbackGains, axle, station);
        operating_point.lateral_integral_feedback_gains[axle] =
            InterpolateGuidanceAxleValue(kLateralIntegralFeedbackGains, axle,
                                         station);
        operating_point.wheel_speed_difference_feedback_gains[axle] =
            InterpolateGuidanceAxleValue(
                kWheelSpeedDifferenceFeedbackGains, axle, station);
        operating_point.yaw_rate_filter_time_constants_seconds[axle] =
            InterpolateGuidanceAxleValue(
                kYawRateFilterTimeConstantsSeconds, axle, station);
        operating_point.lateral_velocity_filter_time_constants_seconds[axle] =
            InterpolateGuidanceAxleValue(
                kLateralVelocityFilterTimeConstantsSeconds, axle, station);
        operating_point.equilibrium_yaw_angles_radians[axle] =
            InterpolateGuidanceAxleValue(kEquilibriumYawAnglesRadians, axle,
                                         station);
        operating_point.equilibrium_lateral_displacements_meters[axle] =
            InterpolateGuidanceAxleValue(
                kEquilibriumLateralDisplacementsMeters, axle, station);
        operating_point.guidance_active[axle] =
            station >= kGuidanceStartTrackStationMeters &&
            station <= kGuidanceEndTrackStationMeters;
    }

    const double outer_filter_time_constant_seconds = InterpolateScalar(
        kGuidanceScheduleTrackStationsMeters,
        kWheelSpeedDifferenceOuterFilterTimeConstantsSeconds,
        average_station_meters);
    const double difference_reference_absolute_limit = InterpolateScalar(
        kGuidanceScheduleTrackStationsMeters,
        kWheelSpeedDifferenceReferenceAbsoluteLimitsRadiansPerSecond,
        average_station_meters);
    operating_point
        .wheel_speed_difference_outer_filter_time_constants_seconds
        .fill(outer_filter_time_constant_seconds);
    operating_point
        .wheel_speed_difference_reference_absolute_limits_radians_per_second
        .fill(difference_reference_absolute_limit);

    return operating_point;
}

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
