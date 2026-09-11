#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::actuation {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("wheel-drive torque command conditioner: " +
                                detail);
}

void RequireFinitePositive(const char* name, double value) {
    if (!std::isfinite(value) || !(value > 0.0)) {
        Reject(std::string(name) + " must be finite and positive");
    }
}

void ValidateDirectionTable(const WheelDriveTorqueDirectionTable& table,
                            const char* name) {
    for (std::size_t index = 0; index < kWheelDriveTorqueSpeedNodeCount;
         ++index) {
        const auto dynamic_limit =
            table.dynamic_limit_drive_side_newton_metres[index];
        if (dynamic_limit.has_value() &&
            (!std::isfinite(*dynamic_limit) || *dynamic_limit < 0.0)) {
            Reject(std::string(name) +
                   " dynamic limits must be absent or finite and "
                   "nonnegative");
        }
        const double increase_rate =
            table.magnitude_increase_rate_drive_side_newton_metres_per_second
                [index];
        const double decrease_rate =
            table.magnitude_decrease_rate_drive_side_newton_metres_per_second
                [index];
        if (!std::isfinite(increase_rate) || increase_rate < 0.0 ||
            !std::isfinite(decrease_rate) || decrease_rate < 0.0) {
            Reject(std::string(name) +
                   " magnitude-change rates must be finite and "
                   "nonnegative");
        }
    }
}

void ValidateConfig(const WheelDriveTorqueCommandConditionerConfig& config) {
    if (config.identifier.empty()) {
        Reject("the identifier is empty");
    }
    RequireFinitePositive("sample_period_seconds",
                          config.sample_period_seconds);
    RequireFinitePositive("drive_ratio", config.drive_ratio);
    RequireFinitePositive("drive_efficiency", config.drive_efficiency);
    RequireFinitePositive("request_deadband_newton_metres",
                          config.request_deadband_newton_metres);
    RequireFinitePositive(
        "wheel_angular_speed_direction_threshold_radians_per_second",
        config.wheel_angular_speed_direction_threshold_radians_per_second);
    if (!std::isfinite(config.forward_wheel_angular_speed_sign) ||
        (config.forward_wheel_angular_speed_sign != -1.0 &&
         config.forward_wheel_angular_speed_sign != 1.0)) {
        Reject("forward_wheel_angular_speed_sign must be -1 or +1");
    }
    for (std::size_t index = 0; index < kWheelDriveTorqueSpeedNodeCount;
         ++index) {
        const double speed =
            config.drive_speed_nodes_revolutions_per_minute[index];
        if (!std::isfinite(speed) || !(speed > 0.0)) {
            Reject("drive-speed nodes must be finite and positive");
        }
        if (index > 0 &&
            !(speed > config.drive_speed_nodes_revolutions_per_minute
                          [index - 1])) {
            Reject("drive-speed nodes must be strictly increasing");
        }
    }
    ValidateDirectionTable(config.traction, "traction");
    ValidateDirectionTable(config.regeneration, "regeneration");
}

int DirectionCode(double requested_wheel_torque,
                  double wheel_angular_speed,
                  const WheelDriveTorqueCommandConditionerConfig& config) {
    if (std::abs(requested_wheel_torque) <=
        config.request_deadband_newton_metres) {
        return 0;
    }
    if (std::abs(wheel_angular_speed) >
        config
            .wheel_angular_speed_direction_threshold_radians_per_second) {
        const double direction_product =
            requested_wheel_torque * wheel_angular_speed;
        if (direction_product > 0.0) {
            return 1;
        }
        if (direction_product < 0.0) {
            return -1;
        }
    }
    return requested_wheel_torque *
                       config.forward_wheel_angular_speed_sign >
                   0.0
               ? 1
               : -1;
}

double EffectiveWheelAngularSpeedSign(
    double wheel_angular_speed,
    const WheelDriveTorqueCommandConditionerConfig& config) {
    if (std::abs(wheel_angular_speed) >
        config
            .wheel_angular_speed_direction_threshold_radians_per_second) {
        return std::signbit(wheel_angular_speed) ? -1.0 : 1.0;
    }
    return config.forward_wheel_angular_speed_sign;
}

double WheelTorqueSign(int direction_code,
                       double effective_wheel_angular_speed_sign) {
    if (direction_code > 0) {
        return effective_wheel_angular_speed_sign;
    }
    if (direction_code < 0) {
        return -effective_wheel_angular_speed_sign;
    }
    return 0.0;
}

double SignedDriveSideToWheelTorque(
    double signed_drive_side_torque,
    double effective_wheel_angular_speed_sign,
    const WheelDriveTorqueCommandConditionerConfig& config) {
    if (std::abs(signed_drive_side_torque) <= 0.0) {
        return 0.0;
    }
    const int direction_code = signed_drive_side_torque > 0.0 ? 1 : -1;
    return WheelTorqueSign(direction_code,
                           effective_wheel_angular_speed_sign) *
           std::abs(signed_drive_side_torque) * config.drive_ratio *
           config.drive_efficiency;
}

struct InterpolatedDirectionRow {
    double dynamic_limit_drive_side_newton_metres{0.0};
    double magnitude_increase_rate_drive_side_newton_metres_per_second{0.0};
    double magnitude_decrease_rate_drive_side_newton_metres_per_second{0.0};
    bool dynamic_advisory{false};
};

struct InterpolatedDriveSpeedSlice {
    InterpolatedDirectionRow traction;
    InterpolatedDirectionRow regeneration;
};

struct SignedTorqueAdvance {
    double signed_drive_side_torque_newton_metres{0.0};
    bool slew_limited{false};
};

double Interpolate(double low, double high, double fraction) {
    return (1.0 - fraction) * low + fraction * high;
}

InterpolatedDirectionRow InterpolateDirectionRow(
    const WheelDriveTorqueDirectionTable& table,
    std::size_t low,
    std::size_t high,
    double fraction) {
    const auto low_dynamic_optional =
        table.dynamic_limit_drive_side_newton_metres[low];
    const auto high_dynamic_optional =
        table.dynamic_limit_drive_side_newton_metres[high];
    const double low_dynamic = low_dynamic_optional.value_or(0.0);
    const double high_dynamic = high_dynamic_optional.value_or(0.0);
    return InterpolatedDirectionRow{
        Interpolate(low_dynamic, high_dynamic, fraction),
        Interpolate(
            table
                .magnitude_increase_rate_drive_side_newton_metres_per_second
                    [low],
            table
                .magnitude_increase_rate_drive_side_newton_metres_per_second
                    [high],
            fraction),
        Interpolate(
            table
                .magnitude_decrease_rate_drive_side_newton_metres_per_second
                    [low],
            table
                .magnitude_decrease_rate_drive_side_newton_metres_per_second
                    [high],
            fraction),
        table.dynamic_advisory[low] || table.dynamic_advisory[high] ||
            !low_dynamic_optional.has_value() ||
            !high_dynamic_optional.has_value()};
}

InterpolatedDriveSpeedSlice InterpolateDriveSpeedSlice(
    const WheelDriveTorqueCommandConditionerConfig& config,
    double drive_speed_query_revolutions_per_minute) {
    const auto& speeds = config.drive_speed_nodes_revolutions_per_minute;
    std::size_t low = 0;
    std::size_t high = 0;
    double fraction = 0.0;
    if (drive_speed_query_revolutions_per_minute <= speeds.front()) {
        low = 0;
        high = 0;
    } else if (drive_speed_query_revolutions_per_minute >= speeds.back()) {
        low = speeds.size() - 1;
        high = speeds.size() - 1;
    } else {
        high = static_cast<std::size_t>(
            std::lower_bound(speeds.begin(), speeds.end(),
                             drive_speed_query_revolutions_per_minute) -
            speeds.begin());
        low = high - 1;
        fraction =
            (drive_speed_query_revolutions_per_minute - speeds[low]) /
            (speeds[high] - speeds[low]);
    }

    return InterpolatedDriveSpeedSlice{
        InterpolateDirectionRow(config.traction, low, high, fraction),
        InterpolateDirectionRow(config.regeneration, low, high, fraction)};
}

int SignCode(double value) {
    if (value > 0.0) {
        return 1;
    }
    if (value < 0.0) {
        return -1;
    }
    return 0;
}

const InterpolatedDirectionRow& DirectionRow(
    const InterpolatedDriveSpeedSlice& slice,
    int nonzero_direction_code) {
    return nonzero_direction_code > 0 ? slice.traction : slice.regeneration;
}

// Positive signed torque is traction and negative signed torque is
// regeneration. Each half-axis uses its own increase/decrease rates, and a
// zero crossing shares one sample period between the two sides.
SignedTorqueAdvance AdvanceSignedDriveSideTorque(
    double previous_signed_torque,
    double target_signed_torque,
    const InterpolatedDriveSpeedSlice& slice,
    double sample_period_seconds) {
    if (previous_signed_torque == target_signed_torque) {
        return SignedTorqueAdvance{
            target_signed_torque == 0.0 ? 0.0 : target_signed_torque, false};
    }

    const int previous_direction = SignCode(previous_signed_torque);
    const int target_direction = SignCode(target_signed_torque);
    const double previous_magnitude = std::abs(previous_signed_torque);
    const double target_magnitude = std::abs(target_signed_torque);

    if (previous_direction == 0) {
        const double rate =
            DirectionRow(slice, target_direction)
                .magnitude_increase_rate_drive_side_newton_metres_per_second;
        const double maximum_magnitude = rate * sample_period_seconds;
        if (target_magnitude <= maximum_magnitude) {
            return SignedTorqueAdvance{target_signed_torque, false};
        }
        if (maximum_magnitude <= 0.0) {
            return SignedTorqueAdvance{0.0, true};
        }
        return SignedTorqueAdvance{
            static_cast<double>(target_direction) * maximum_magnitude, true};
    }

    if (target_direction == 0) {
        const double rate =
            DirectionRow(slice, previous_direction)
                .magnitude_decrease_rate_drive_side_newton_metres_per_second;
        const double maximum_decrease = rate * sample_period_seconds;
        if (previous_magnitude <= maximum_decrease) {
            return SignedTorqueAdvance{0.0, false};
        }
        return SignedTorqueAdvance{
            static_cast<double>(previous_direction) *
                (previous_magnitude - maximum_decrease),
            true};
    }

    if (previous_direction == target_direction) {
        const double magnitude_delta = target_magnitude - previous_magnitude;
        const auto& row = DirectionRow(slice, target_direction);
        const double rate =
            magnitude_delta >= 0.0
                ? row.magnitude_increase_rate_drive_side_newton_metres_per_second
                : row.magnitude_decrease_rate_drive_side_newton_metres_per_second;
        const double maximum_delta = rate * sample_period_seconds;
        if (std::abs(magnitude_delta) <= maximum_delta) {
            return SignedTorqueAdvance{target_signed_torque, false};
        }
        const double limited_delta =
            std::clamp(magnitude_delta, -maximum_delta, maximum_delta);
        return SignedTorqueAdvance{
            static_cast<double>(target_direction) *
                (previous_magnitude + limited_delta),
            true};
    }

    const double old_direction_decrease_rate =
        DirectionRow(slice, previous_direction)
            .magnitude_decrease_rate_drive_side_newton_metres_per_second;
    if (old_direction_decrease_rate <= 0.0) {
        return SignedTorqueAdvance{previous_signed_torque, true};
    }

    const double maximum_old_direction_decrease =
        old_direction_decrease_rate * sample_period_seconds;
    if (previous_magnitude >= maximum_old_direction_decrease) {
        const double remaining_magnitude = std::max(
            0.0, previous_magnitude - maximum_old_direction_decrease);
        return SignedTorqueAdvance{
            remaining_magnitude > 0.0
                ? static_cast<double>(previous_direction) *
                      remaining_magnitude
                : 0.0,
            true};
    }

    const double time_to_zero =
        previous_magnitude / old_direction_decrease_rate;
    const double remaining_time =
        std::max(0.0, sample_period_seconds - time_to_zero);
    const double new_direction_increase_rate =
        DirectionRow(slice, target_direction)
            .magnitude_increase_rate_drive_side_newton_metres_per_second;
    const double maximum_new_magnitude =
        new_direction_increase_rate * remaining_time;
    if (target_magnitude <= maximum_new_magnitude) {
        return SignedTorqueAdvance{target_signed_torque, false};
    }
    if (maximum_new_magnitude <= 0.0) {
        return SignedTorqueAdvance{0.0, true};
    }
    return SignedTorqueAdvance{
        static_cast<double>(target_direction) * maximum_new_magnitude, true};
}

void AddFlag(WheelDriveTorqueLimitFlag* value,
             WheelDriveTorqueLimitFlag flag) {
    *value = *value | flag;
}

}  // namespace

WheelDriveTorqueCommandConditioner::WheelDriveTorqueCommandConditioner(
    WheelDriveTorqueCommandConditionerConfig config)
    : config_(std::move(config)) {
    ValidateConfig(config_);
}

WheelDriveTorqueConditioningResult
WheelDriveTorqueCommandConditioner::Step(
    const WheelDriveTorqueChannelValues&
        requested_wheel_torques_newton_metres,
    const WheelDriveTorqueChannelValues&
        wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
    const WheelDriveTorqueChannelValues&
        previous_drive_side_torque_memory_newton_metres) const {
    for (std::size_t index = 0; index < kWheelDriveTorqueChannelCount;
         ++index) {
        if (!std::isfinite(requested_wheel_torques_newton_metres[index]) ||
            !std::isfinite(
                wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second
                    [index]) ||
            !std::isfinite(
                previous_drive_side_torque_memory_newton_metres[index])) {
            Reject("all request, wheel-speed and previous-memory entries "
                   "must be finite");
        }
    }

    WheelDriveTorqueConditioningResult result;
    const double minimum_drive_speed =
        config_.drive_speed_nodes_revolutions_per_minute.front();
    const double maximum_drive_speed =
        config_.drive_speed_nodes_revolutions_per_minute.back();
    for (std::size_t index = 0; index < kWheelDriveTorqueChannelCount;
         ++index) {
        const double requested_wheel_torque =
            requested_wheel_torques_newton_metres[index];
        const double wheel_angular_speed =
            wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second
                [index];
        const int direction_code = DirectionCode(
            requested_wheel_torque, wheel_angular_speed, config_);
        const double effective_wheel_angular_speed_sign =
            EffectiveWheelAngularSpeedSign(wheel_angular_speed, config_);
        const double drive_speed_revolutions_per_minute =
            std::abs(wheel_angular_speed) * config_.drive_ratio * 60.0 /
            (2.0 * std::numbers::pi);
        result.equivalent_drive_side_speeds_revolutions_per_minute[index] =
            drive_speed_revolutions_per_minute;

        auto& flags = result.limit_flags[index];
        const double drive_speed_query = std::clamp(
            drive_speed_revolutions_per_minute, minimum_drive_speed,
            maximum_drive_speed);
        const InterpolatedDriveSpeedSlice speed_slice =
            InterpolateDriveSpeedSlice(config_, drive_speed_query);

        const double previous_signed_drive_side_torque =
            previous_drive_side_torque_memory_newton_metres[index];
        const bool rate_progression_is_active =
            direction_code != 0 || previous_signed_drive_side_torque != 0.0;
        const bool drive_speed_is_unsupported =
            drive_speed_revolutions_per_minute > maximum_drive_speed;
        if (rate_progression_is_active && drive_speed_is_unsupported) {
            AddFlag(&flags,
                    WheelDriveTorqueLimitFlag::kUnsupportedHighSpeed);
            AddFlag(&flags,
                    WheelDriveTorqueLimitFlag::kDynamicValueAdvisory);
        } else if (rate_progression_is_active &&
                   drive_speed_revolutions_per_minute < minimum_drive_speed) {
            AddFlag(&flags,
                    WheelDriveTorqueLimitFlag::kLowSpeedLookupClamped);
        }

        double target_signed_drive_side_torque = 0.0;
        if (direction_code != 0 && !drive_speed_is_unsupported) {
            const auto& target_row =
                DirectionRow(speed_slice, direction_code);
            if (target_row.dynamic_advisory) {
                AddFlag(&flags,
                        WheelDriveTorqueLimitFlag::kDynamicValueAdvisory);
            }

            const double dynamic_limit =
                target_row.dynamic_limit_drive_side_newton_metres;
            result.wheel_dynamic_torque_limits_newton_metres[index] =
                dynamic_limit * config_.drive_ratio *
                config_.drive_efficiency;

            const double requested_magnitude =
                std::abs(requested_wheel_torque) /
                (config_.drive_ratio * config_.drive_efficiency);
            const double target_magnitude =
                std::min(requested_magnitude, dynamic_limit);
            if (requested_magnitude > dynamic_limit + 1.0e-9) {
                AddFlag(&flags,
                        WheelDriveTorqueLimitFlag::kMagnitudeLimited);
            }
            target_signed_drive_side_torque =
                target_magnitude > 0.0
                    ? static_cast<double>(direction_code) * target_magnitude
                    : 0.0;
        }

        // The terminal rate row can return high-speed memory toward zero, but
        // cannot establish a new nonzero target above the supported range.
        const SignedTorqueAdvance advance = AdvanceSignedDriveSideTorque(
            previous_signed_drive_side_torque, target_signed_drive_side_torque,
            speed_slice, config_.sample_period_seconds);
        if (advance.slew_limited) {
            AddFlag(&flags, WheelDriveTorqueLimitFlag::kSlewLimited);
        }

        const double signed_drive_side_torque =
            advance.signed_drive_side_torque_newton_metres;

        result.next_drive_side_torque_memory_newton_metres[index] =
            signed_drive_side_torque;
        result.actual_wheel_torques_newton_metres[index] =
            SignedDriveSideToWheelTorque(
                signed_drive_side_torque,
                effective_wheel_angular_speed_sign, config_);
    }
    return result;
}

}  // namespace orvd::actuation
