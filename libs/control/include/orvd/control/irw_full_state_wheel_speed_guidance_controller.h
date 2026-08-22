#pragma once

/// @file
/// Frozen 100 Hz IRW full-state wheel-speed guidance calculation.

#include <array>
#include <cstddef>
#include <string>

#include "orvd/control/sampled_filtered_pi.h"

namespace orvd::control {

inline constexpr std::size_t kIrwGuidanceAxleCount = 4;
inline constexpr std::size_t kIrwGuidanceWheelCount = 8;

using IrwGuidanceAxleValues = std::array<double, kIrwGuidanceAxleCount>;
using IrwGuidanceWheelValues = std::array<double, kIrwGuidanceWheelCount>;

/// Immutable parameters of the frozen R300/V60 IRW controller personality.
struct IrwFullStateWheelSpeedGuidanceControllerConfig {
    std::string identifier;
    double sample_period_seconds{0.0};

    double base_speed_reference_meters_per_second{0.0};
    double curve_radius_meters{0.0};
    double controller_calibration_lateral_half_span_meters{0.0};
    IrwGuidanceWheelValues speed_reference_wheel_side_signs{};
    double speed_reference_ramp_start_track_station_meters{0.0};
    double speed_reference_ramp_end_track_station_meters{0.0};

    double rolling_radius_meters{0.0};
    IrwGuidanceAxleValues guidance_axle_signs{};
    IrwGuidanceWheelValues guidance_wheel_signs{};
    IrwGuidanceAxleValues feedforward_gains{};
    IrwGuidanceAxleValues yaw_rate_feedback_gains{};
    IrwGuidanceAxleValues yaw_feedback_gains{};
    IrwGuidanceAxleValues lateral_velocity_feedback_gains{};
    IrwGuidanceAxleValues lateral_displacement_feedback_gains{};
    IrwGuidanceAxleValues lateral_integral_feedback_gains{};
    IrwGuidanceAxleValues wheel_speed_difference_feedback_gains{};
    double wheel_speed_difference_reference_absolute_limit_radians_per_second{
        0.0};
    double yaw_rate_filter_time_constant_seconds{0.0};
    double lateral_velocity_filter_time_constant_seconds{0.0};
    double wheel_speed_difference_outer_filter_time_constant_seconds{0.0};
    IrwGuidanceAxleValues equilibrium_yaw_angles_radians{};
    IrwGuidanceAxleValues equilibrium_lateral_displacements_meters{};
    double guidance_start_track_station_meters{0.0};
    double guidance_end_track_station_meters{0.0};
    double lateral_integral_absolute_limit_meter_seconds{0.0};

    SampledFilteredPiConfig wheel_speed_pi;
    IrwGuidanceWheelValues wheel_speed_pi_wheel_signs{};
};

/// Line- and scenario-independent constants of the guidance recurrence.
///
/// A caller that supplies time-varying operating points can construct this
/// recurrence directly, without inventing a frozen controller personality.
struct IrwFullStateWheelSpeedGuidanceRecurrenceConfig {
    double sample_period_seconds{0.0};
    double rolling_radius_meters{0.0};
    IrwGuidanceAxleValues guidance_axle_signs{};
    IrwGuidanceWheelValues guidance_wheel_signs{};
    double lateral_integral_absolute_limit_meter_seconds{0.0};
    SampledFilteredPiConfig wheel_speed_pi;
    IrwGuidanceWheelValues wheel_speed_pi_wheel_signs{};
};

/// One already-evaluated operating point for a single guidance sample.
///
/// This type deliberately has no station, curvature, interpolation or route
/// semantics. Those belong to the caller that computes the operating point.
struct IrwFullStateWheelSpeedGuidanceOperatingPoint {
    IrwGuidanceWheelValues base_wheel_speed_references_meters_per_second{};
    IrwGuidanceAxleValues feedforward_gains{};
    IrwGuidanceAxleValues yaw_rate_feedback_gains{};
    IrwGuidanceAxleValues yaw_feedback_gains{};
    IrwGuidanceAxleValues lateral_velocity_feedback_gains{};
    IrwGuidanceAxleValues lateral_displacement_feedback_gains{};
    IrwGuidanceAxleValues lateral_integral_feedback_gains{};
    IrwGuidanceAxleValues wheel_speed_difference_feedback_gains{};
    IrwGuidanceAxleValues yaw_rate_filter_time_constants_seconds{};
    IrwGuidanceAxleValues lateral_velocity_filter_time_constants_seconds{};
    IrwGuidanceAxleValues
        wheel_speed_difference_outer_filter_time_constants_seconds{};
    IrwGuidanceAxleValues
        wheel_speed_difference_reference_absolute_limits_radians_per_second{};
    IrwGuidanceAxleValues equilibrium_yaw_angles_radians{};
    IrwGuidanceAxleValues equilibrium_lateral_displacements_meters{};
    std::array<bool, kIrwGuidanceAxleCount> guidance_active{};
};

/// Caller-owned controller memory: one initialization flag and 40 numeric
/// values. It is intentionally not packed into a 41-double transport vector.
struct IrwFullStateWheelSpeedGuidanceControllerState {
    bool initialized{false};
    IrwGuidanceAxleValues previous_lateral_displacements_meters{};
    IrwGuidanceAxleValues previous_yaw_angles_radians{};
    IrwGuidanceAxleValues filtered_lateral_velocities_meters_per_second{};
    IrwGuidanceAxleValues filtered_yaw_rates_radians_per_second{};
    IrwGuidanceAxleValues lateral_error_integrals_meter_seconds{};
    IrwGuidanceAxleValues
        filtered_wheel_speed_difference_commands_radians_per_second{};
    IrwGuidanceWheelValues wheel_speed_pi_integrals_meters{};
    IrwGuidanceWheelValues
        wheel_speed_pi_filtered_torques_newton_metres{};
};

/// Mechanical observations consumed by the line-independent recurrence.
struct IrwFullStateWheelSpeedGuidanceMechanicalInput {
    IrwGuidanceAxleValues axle_lateral_displacements_meters{};
    IrwGuidanceAxleValues axle_yaw_angles_radians{};
    IrwGuidanceWheelValues
        wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second{};
};

/// Current mechanical observations in the controller's frozen scalar order.
///
/// The G76 vehicle binding, not this algorithm, maps named wheel joint rates to
/// the wheel-speed convention consumed here.
struct IrwFullStateWheelSpeedGuidanceControllerInput {
    IrwGuidanceAxleValues axle_lateral_displacements_meters{};
    IrwGuidanceAxleValues axle_yaw_angles_radians{};
    IrwGuidanceAxleValues axle_track_stations_meters{};
    IrwGuidanceWheelValues
        wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second{};
};

/// Values exposed by one controller step for qualification and diagnosis.
struct IrwFullStateWheelSpeedGuidanceControllerObservations {
    IrwGuidanceWheelValues base_wheel_speed_references_meters_per_second{};
    IrwGuidanceWheelValues wheel_speed_references_meters_per_second{};
    IrwGuidanceAxleValues
        wheel_speed_difference_references_radians_per_second{};
    IrwGuidanceAxleValues
        measured_wheel_speed_differences_radians_per_second{};
    IrwGuidanceAxleValues
        equilibrium_wheel_speed_differences_radians_per_second{};
    IrwGuidanceAxleValues filtered_lateral_velocities_meters_per_second{};
    IrwGuidanceAxleValues filtered_yaw_rates_radians_per_second{};
    std::array<bool, kIrwGuidanceAxleCount> guidance_active{};
};

/// Requested torques, observations and state produced by one pure step.
struct IrwFullStateWheelSpeedGuidanceControllerResult {
    IrwGuidanceWheelValues requested_wheel_torques_newton_metres{};
    IrwFullStateWheelSpeedGuidanceControllerObservations observations;
    IrwFullStateWheelSpeedGuidanceControllerState next_state;
};

/// Pure state recurrence for one already-evaluated guidance operating point.
class IrwFullStateWheelSpeedGuidanceRecurrence {
   public:
    explicit IrwFullStateWheelSpeedGuidanceRecurrence(
        IrwFullStateWheelSpeedGuidanceRecurrenceConfig config);

    [[nodiscard]] const IrwFullStateWheelSpeedGuidanceRecurrenceConfig& config()
        const noexcept {
        return config_;
    }

    [[nodiscard]] IrwFullStateWheelSpeedGuidanceControllerResult Step(
        const IrwFullStateWheelSpeedGuidanceMechanicalInput& input,
        const IrwFullStateWheelSpeedGuidanceOperatingPoint& operating_point,
        const IrwFullStateWheelSpeedGuidanceControllerState& previous_state)
        const;

   private:
    IrwFullStateWheelSpeedGuidanceRecurrenceConfig config_;
    SampledFilteredPi wheel_speed_pi_;
};

/// Immutable frozen 100 Hz IRW full-state wheel-speed guidance controller.
class IrwFullStateWheelSpeedGuidanceController {
   public:
    explicit IrwFullStateWheelSpeedGuidanceController(
        IrwFullStateWheelSpeedGuidanceControllerConfig config);

    [[nodiscard]] const
    IrwFullStateWheelSpeedGuidanceControllerConfig& config() const noexcept {
        return config_;
    }

    [[nodiscard]] IrwFullStateWheelSpeedGuidanceControllerResult Step(
        const IrwFullStateWheelSpeedGuidanceControllerInput& input,
        const IrwFullStateWheelSpeedGuidanceControllerState& previous_state)
        const;

   private:
    IrwFullStateWheelSpeedGuidanceControllerConfig config_;
    IrwFullStateWheelSpeedGuidanceRecurrence recurrence_;
};

}  // namespace orvd::control
