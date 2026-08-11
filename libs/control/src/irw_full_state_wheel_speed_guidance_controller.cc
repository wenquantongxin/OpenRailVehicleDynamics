#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::control {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument(
        "IRW full-state wheel-speed guidance controller: " + detail);
}

void RequireFinitePositive(const char* name, double value) {
    if (!std::isfinite(value) || !(value > 0.0)) {
        Reject(std::string(name) + " must be finite and positive");
    }
}

void RequireFiniteNonnegative(const char* name, double value) {
    if (!std::isfinite(value) || value < 0.0) {
        Reject(std::string(name) + " must be finite and nonnegative");
    }
}

template <std::size_t Size>
void RequireFiniteArray(const char* name,
                        const std::array<double, Size>& values) {
    for (double value : values) {
        if (!std::isfinite(value)) {
            Reject(std::string(name) + " entries must be finite");
        }
    }
}

template <std::size_t Size>
void RequireSignArray(const char* name,
                      const std::array<double, Size>& values) {
    for (double value : values) {
        if (value != -1.0 && value != 1.0) {
            Reject(std::string(name) + " entries must be -1 or +1");
        }
    }
}

void ValidateConfig(
    const IrwFullStateWheelSpeedGuidanceControllerConfig& config) {
    if (config.identifier.empty()) {
        Reject("the identifier is empty");
    }
    RequireFinitePositive("sample_period_seconds",
                          config.sample_period_seconds);
    if (!std::isfinite(config.base_speed_reference_meters_per_second)) {
        Reject("base_speed_reference_meters_per_second must be finite");
    }
    RequireFinitePositive("curve_radius_meters", config.curve_radius_meters);
    RequireFinitePositive(
        "controller_calibration_lateral_half_span_meters",
        config.controller_calibration_lateral_half_span_meters);
    RequireSignArray("speed_reference_wheel_side_signs",
                     config.speed_reference_wheel_side_signs);
    if (!std::isfinite(
            config.speed_reference_ramp_start_track_station_meters) ||
        !std::isfinite(
            config.speed_reference_ramp_end_track_station_meters) ||
        !(config.speed_reference_ramp_end_track_station_meters >
          config.speed_reference_ramp_start_track_station_meters)) {
        Reject("the speed-reference ramp must have finite, increasing track "
               "stations");
    }

    RequireFinitePositive("rolling_radius_meters",
                          config.rolling_radius_meters);
    RequireSignArray("guidance_axle_signs", config.guidance_axle_signs);
    RequireSignArray("guidance_wheel_signs", config.guidance_wheel_signs);
    RequireFiniteArray("feedforward_gains", config.feedforward_gains);
    RequireFiniteArray("yaw_rate_feedback_gains",
                       config.yaw_rate_feedback_gains);
    RequireFiniteArray("yaw_feedback_gains", config.yaw_feedback_gains);
    RequireFiniteArray("lateral_velocity_feedback_gains",
                       config.lateral_velocity_feedback_gains);
    RequireFiniteArray("lateral_displacement_feedback_gains",
                       config.lateral_displacement_feedback_gains);
    RequireFiniteArray("lateral_integral_feedback_gains",
                       config.lateral_integral_feedback_gains);
    RequireFiniteArray("wheel_speed_difference_feedback_gains",
                       config.wheel_speed_difference_feedback_gains);
    RequireFiniteNonnegative(
        "wheel_speed_difference_reference_absolute_limit_radians_per_second",
        config
            .wheel_speed_difference_reference_absolute_limit_radians_per_second);
    RequireFiniteNonnegative(
        "yaw_rate_filter_time_constant_seconds",
        config.yaw_rate_filter_time_constant_seconds);
    RequireFiniteNonnegative(
        "lateral_velocity_filter_time_constant_seconds",
        config.lateral_velocity_filter_time_constant_seconds);
    RequireFiniteNonnegative(
        "wheel_speed_difference_outer_filter_time_constant_seconds",
        config
            .wheel_speed_difference_outer_filter_time_constant_seconds);
    RequireFiniteArray("equilibrium_yaw_angles_radians",
                       config.equilibrium_yaw_angles_radians);
    RequireFiniteArray("equilibrium_lateral_displacements_meters",
                       config.equilibrium_lateral_displacements_meters);
    if (!std::isfinite(config.guidance_start_track_station_meters) ||
        !std::isfinite(config.guidance_end_track_station_meters) ||
        !(config.guidance_end_track_station_meters >
          config.guidance_start_track_station_meters)) {
        Reject("the guidance interval must have finite, increasing track "
               "stations");
    }
    RequireFiniteNonnegative(
        "lateral_integral_absolute_limit_meter_seconds",
        config.lateral_integral_absolute_limit_meter_seconds);
    RequireSignArray("wheel_speed_pi_wheel_signs",
                     config.wheel_speed_pi_wheel_signs);
}

double FirstOrderAlpha(double sample_period_seconds,
                       double time_constant_seconds) {
    return time_constant_seconds <= 0.0
               ? 1.0
               : sample_period_seconds /
                     (time_constant_seconds + sample_period_seconds);
}

double ClampSymmetric(double value, double absolute_limit) {
    if (absolute_limit <= 0.0) {
        return 0.0;
    }
    return std::clamp(value, -absolute_limit, absolute_limit);
}

}  // namespace

IrwFullStateWheelSpeedGuidanceController::
    IrwFullStateWheelSpeedGuidanceController(
        IrwFullStateWheelSpeedGuidanceControllerConfig config)
    : config_(std::move(config)), wheel_speed_pi_(config_.wheel_speed_pi) {
    ValidateConfig(config_);
}

IrwFullStateWheelSpeedGuidanceControllerResult
IrwFullStateWheelSpeedGuidanceController::Step(
    const IrwFullStateWheelSpeedGuidanceControllerInput& input,
    const IrwFullStateWheelSpeedGuidanceControllerState& previous_state)
    const {
    IrwFullStateWheelSpeedGuidanceControllerResult result;
    auto& observations = result.observations;
    result.next_state = previous_state;

    for (std::size_t axle = 0; axle < kIrwGuidanceAxleCount; ++axle) {
        const double ramp_fraction = std::clamp(
            (input.axle_track_stations_meters[axle] -
             config_.speed_reference_ramp_start_track_station_meters) /
                (config_.speed_reference_ramp_end_track_station_meters -
                 config_.speed_reference_ramp_start_track_station_meters),
            0.0, 1.0);
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t wheel = 2 * axle + side;
            const double straight_reference =
                config_.base_speed_reference_meters_per_second;
            const double curve_reference =
                config_.base_speed_reference_meters_per_second *
                (config_.curve_radius_meters +
                 config_.speed_reference_wheel_side_signs[wheel] *
                     config_
                         .controller_calibration_lateral_half_span_meters) /
                config_.curve_radius_meters;
            observations.base_wheel_speed_references_meters_per_second[wheel] =
                (1.0 - ramp_fraction) * straight_reference +
                ramp_fraction * curve_reference;
        }
    }

    const double lateral_velocity_alpha = FirstOrderAlpha(
        config_.sample_period_seconds,
        config_.lateral_velocity_filter_time_constant_seconds);
    const double yaw_rate_alpha = FirstOrderAlpha(
        config_.sample_period_seconds,
        config_.yaw_rate_filter_time_constant_seconds);
    for (std::size_t axle = 0; axle < kIrwGuidanceAxleCount; ++axle) {
        if (!previous_state.initialized) {
            observations.filtered_lateral_velocities_meters_per_second[axle] =
                0.0;
            observations.filtered_yaw_rates_radians_per_second[axle] = 0.0;
        } else {
            const double raw_lateral_velocity =
                (input.axle_lateral_displacements_meters[axle] -
                 previous_state.previous_lateral_displacements_meters[axle]) /
                config_.sample_period_seconds;
            const double raw_yaw_rate =
                (input.axle_yaw_angles_radians[axle] -
                 previous_state.previous_yaw_angles_radians[axle]) /
                config_.sample_period_seconds;
            observations.filtered_lateral_velocities_meters_per_second[axle] =
                (1.0 - lateral_velocity_alpha) *
                    previous_state
                        .filtered_lateral_velocities_meters_per_second[axle] +
                lateral_velocity_alpha * raw_lateral_velocity;
            observations.filtered_yaw_rates_radians_per_second[axle] =
                (1.0 - yaw_rate_alpha) *
                    previous_state.filtered_yaw_rates_radians_per_second
                        [axle] +
                yaw_rate_alpha * raw_yaw_rate;
        }

        const std::size_t left = 2 * axle;
        const std::size_t right = left + 1;
        observations
            .measured_wheel_speed_differences_radians_per_second[axle] =
            config_.guidance_wheel_signs[left] *
                input
                    .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second
                        [left] -
            config_.guidance_wheel_signs[right] *
                input
                    .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second
                        [right];
        observations
            .equilibrium_wheel_speed_differences_radians_per_second[axle] =
            (observations.base_wheel_speed_references_meters_per_second[left] -
             observations
                 .base_wheel_speed_references_meters_per_second[right]) /
            config_.rolling_radius_meters;
        observations.guidance_active[axle] =
            std::isfinite(input.axle_track_stations_meters[axle]) &&
            input.axle_track_stations_meters[axle] >=
                config_.guidance_start_track_station_meters &&
            input.axle_track_stations_meters[axle] <=
                config_.guidance_end_track_station_meters;
    }

    IrwGuidanceAxleValues wheel_speed_difference_targets{};
    const double outer_filter_alpha = FirstOrderAlpha(
        config_.sample_period_seconds,
        config_
            .wheel_speed_difference_outer_filter_time_constant_seconds);
    bool wheel_speed_difference_feedback_disabled = true;
    for (double gain : config_.wheel_speed_difference_feedback_gains) {
        if (std::abs(gain) > 1.0e-15) {
            wheel_speed_difference_feedback_disabled = false;
        }
    }
    for (std::size_t axle = 0; axle < kIrwGuidanceAxleCount; ++axle) {
        const double lateral_error =
            input.axle_lateral_displacements_meters[axle] -
            config_.equilibrium_lateral_displacements_meters[axle];
        const double yaw_error = input.axle_yaw_angles_radians[axle] -
                                 config_.equilibrium_yaw_angles_radians[axle];
        const double lateral_integral_candidate = ClampSymmetric(
            previous_state.lateral_error_integrals_meter_seconds[axle] +
                lateral_error * config_.sample_period_seconds,
            config_.lateral_integral_absolute_limit_meter_seconds);
        result.next_state.lateral_error_integrals_meter_seconds[axle] =
            observations.guidance_active[axle] ? lateral_integral_candidate
                                               : 0.0;

        const double full_state_feedback =
            config_.yaw_rate_feedback_gains[axle] *
                observations.filtered_yaw_rates_radians_per_second[axle] +
            config_.yaw_feedback_gains[axle] * yaw_error +
            config_.lateral_velocity_feedback_gains[axle] *
                observations
                    .filtered_lateral_velocities_meters_per_second[axle] +
            config_.lateral_displacement_feedback_gains[axle] *
                lateral_error +
            config_.lateral_integral_feedback_gains[axle] *
                result.next_state
                    .lateral_error_integrals_meter_seconds[axle];
        wheel_speed_difference_targets[axle] =
            config_.feedforward_gains[axle] *
                observations
                    .equilibrium_wheel_speed_differences_radians_per_second
                        [axle] +
            config_.guidance_axle_signs[axle] * full_state_feedback;

        if (!observations.guidance_active[axle]) {
            observations
                .wheel_speed_difference_references_radians_per_second[axle] =
                0.0;
            result.next_state
                .filtered_wheel_speed_difference_commands_radians_per_second
                    [axle] = 0.0;
            continue;
        }
        if (wheel_speed_difference_feedback_disabled) {
            observations
                .wheel_speed_difference_references_radians_per_second[axle] =
                wheel_speed_difference_targets[axle];
            result.next_state
                .filtered_wheel_speed_difference_commands_radians_per_second
                    [axle] = wheel_speed_difference_targets[axle];
        } else {
            const double difference_feedback =
                config_.wheel_speed_difference_feedback_gains[axle] *
                (wheel_speed_difference_targets[axle] -
                 observations
                     .measured_wheel_speed_differences_radians_per_second
                         [axle]);
            const double raw_difference_command =
                wheel_speed_difference_targets[axle] + difference_feedback;
            const double filtered_difference_command =
                (1.0 - outer_filter_alpha) *
                    previous_state
                        .filtered_wheel_speed_difference_commands_radians_per_second
                            [axle] +
                outer_filter_alpha * raw_difference_command;
            observations
                .wheel_speed_difference_references_radians_per_second[axle] =
                filtered_difference_command;
            result.next_state
                .filtered_wheel_speed_difference_commands_radians_per_second
                    [axle] = filtered_difference_command;
        }
        observations
            .wheel_speed_difference_references_radians_per_second[axle] =
            ClampSymmetric(
                observations
                    .wheel_speed_difference_references_radians_per_second
                        [axle],
                config_
                    .wheel_speed_difference_reference_absolute_limit_radians_per_second);
    }

    observations.wheel_speed_references_meters_per_second =
        observations.base_wheel_speed_references_meters_per_second;
    for (std::size_t axle = 0; axle < kIrwGuidanceAxleCount; ++axle) {
        const std::size_t left = 2 * axle;
        const std::size_t right = left + 1;
        const double speed_difference =
            0.5 * config_.rolling_radius_meters *
            observations
                .wheel_speed_difference_references_radians_per_second[axle];
        observations.wheel_speed_references_meters_per_second[left] +=
            speed_difference;
        observations.wheel_speed_references_meters_per_second[right] -=
            speed_difference;
    }

    for (std::size_t wheel = 0; wheel < kIrwGuidanceWheelCount; ++wheel) {
        const double wheel_speed_meters_per_second =
            config_.rolling_radius_meters *
            config_.wheel_speed_pi_wheel_signs[wheel] *
            input
                .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second
                    [wheel];
        const double speed_error =
            observations.wheel_speed_references_meters_per_second[wheel] -
            wheel_speed_meters_per_second;
        const auto pi_result = wheel_speed_pi_.Step(
            speed_error, config_.sample_period_seconds,
            SampledFilteredPiState{
                previous_state.wheel_speed_pi_integrals_meters[wheel],
                previous_state
                    .wheel_speed_pi_filtered_torques_newton_metres[wheel]});
        result.requested_wheel_torques_newton_metres[wheel] =
            pi_result.output;
        result.next_state.wheel_speed_pi_integrals_meters[wheel] =
            pi_result.next_state.integral;
        result.next_state
            .wheel_speed_pi_filtered_torques_newton_metres[wheel] =
            pi_result.next_state.filtered_output;
    }

    result.next_state.initialized = true;
    result.next_state.previous_lateral_displacements_meters =
        input.axle_lateral_displacements_meters;
    result.next_state.previous_yaw_angles_radians =
        input.axle_yaw_angles_radians;
    result.next_state.filtered_lateral_velocities_meters_per_second =
        observations.filtered_lateral_velocities_meters_per_second;
    result.next_state.filtered_yaw_rates_radians_per_second =
        observations.filtered_yaw_rates_radians_per_second;
    return result;
}

}  // namespace orvd::control
