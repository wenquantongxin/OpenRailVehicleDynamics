#pragma once

/// @file
/// Fixed eight-channel wheel-drive torque command conditioning.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace orvd::actuation {

inline constexpr std::size_t kWheelDriveTorqueChannelCount = 8;
inline constexpr std::size_t kWheelDriveTorqueSpeedNodeCount = 10;

using WheelDriveTorqueChannelValues =
    std::array<double, kWheelDriveTorqueChannelCount>;

/// Active observations from one conditioning step.
///
/// The bit values retain the frozen source observation contract. Dormant source
/// features deliberately have no enumerator here.
enum class WheelDriveTorqueLimitFlag : std::uint16_t {
    kNone = 0,
    kMagnitudeLimited = 1,
    kSlewLimited = 2,
    kUnsupportedHighSpeed = 8,
    kLowSpeedLookupClamped = 16,
    kDynamicValueAdvisory = 64,
};

[[nodiscard]] constexpr WheelDriveTorqueLimitFlag operator|(
    WheelDriveTorqueLimitFlag left,
    WheelDriveTorqueLimitFlag right) noexcept {
    return static_cast<WheelDriveTorqueLimitFlag>(
        static_cast<std::uint16_t>(left) |
        static_cast<std::uint16_t>(right));
}

[[nodiscard]] constexpr bool HasWheelDriveTorqueLimitFlag(
    WheelDriveTorqueLimitFlag value,
    WheelDriveTorqueLimitFlag flag) noexcept {
    return (static_cast<std::uint16_t>(value) &
            static_cast<std::uint16_t>(flag)) != 0;
}

/// One source direction table, expressed on the equivalent drive side.
///
/// `std::nullopt` is the only representation of an unavailable nonzero dynamic
/// target. It is never encoded as NaN or as an auxiliary sentinel flag.
struct WheelDriveTorqueDirectionTable {
    std::array<std::optional<double>, kWheelDriveTorqueSpeedNodeCount>
        dynamic_limit_drive_side_newton_metres{};
    std::array<double, kWheelDriveTorqueSpeedNodeCount>
        magnitude_increase_rate_drive_side_newton_metres_per_second{};
    std::array<double, kWheelDriveTorqueSpeedNodeCount>
        magnitude_decrease_rate_drive_side_newton_metres_per_second{};
    std::array<bool, kWheelDriveTorqueSpeedNodeCount> dynamic_advisory{};
};

/// Immutable inputs that define one wheel-drive conditioner personality.
///
/// `traction` and `regeneration` are names of the frozen source direction
/// codes. They do not assert the sign of physical power about the G73 +Y torque
/// axis. The caller supplies wheel speed in that frozen scalar convention.
struct WheelDriveTorqueCommandConditionerConfig {
    std::string identifier;
    double sample_period_seconds{0.0};
    double drive_ratio{0.0};
    double drive_efficiency{0.0};
    double request_deadband_newton_metres{0.0};
    double wheel_angular_speed_direction_threshold_radians_per_second{0.0};
    double forward_wheel_angular_speed_sign{0.0};
    std::array<double, kWheelDriveTorqueSpeedNodeCount>
        drive_speed_nodes_revolutions_per_minute{};
    WheelDriveTorqueDirectionTable traction;
    WheelDriveTorqueDirectionTable regeneration;
};

/// Output of one pure eight-channel conditioning step.
struct WheelDriveTorqueConditioningResult {
    WheelDriveTorqueChannelValues actual_wheel_torques_newton_metres{};
    WheelDriveTorqueChannelValues
        wheel_dynamic_torque_limits_newton_metres{};
    std::array<WheelDriveTorqueLimitFlag, kWheelDriveTorqueChannelCount>
        limit_flags{};
    WheelDriveTorqueChannelValues
        equivalent_drive_side_speeds_revolutions_per_minute{};
    WheelDriveTorqueChannelValues
        next_drive_side_torque_memory_newton_metres{};
};

/// A fixed eight-channel command conditioner with caller-owned memory.
///
/// The object owns only immutable parameters. `Step()` reads the current
/// request, the current wheel--axle relative angular speed expressed in the
/// frozen SIMPACK scalar convention, and the previous accepted-event memory.
/// It neither owns nor commits an event state. In particular, the speed input
/// is not the raw G73 joint rate about the axle bridge's local +Y axis; the IRW
/// event binding is responsible for the documented sign conversion.
class WheelDriveTorqueCommandConditioner {
   public:
    explicit WheelDriveTorqueCommandConditioner(
        WheelDriveTorqueCommandConditionerConfig config);

    [[nodiscard]] const WheelDriveTorqueCommandConditionerConfig& config()
        const noexcept {
        return config_;
    }

    /// Computes one sample without allocation, input mutation or internal
    /// state mutation. The requested input is intentionally not echoed in the
    /// result because it remains the caller's observation authority.
    [[nodiscard]] WheelDriveTorqueConditioningResult Step(
        const WheelDriveTorqueChannelValues&
            requested_wheel_torques_newton_metres,
        const WheelDriveTorqueChannelValues&
            wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        const WheelDriveTorqueChannelValues&
            previous_drive_side_torque_memory_newton_metres) const;

   private:
    WheelDriveTorqueCommandConditionerConfig config_;
};

}  // namespace orvd::actuation
