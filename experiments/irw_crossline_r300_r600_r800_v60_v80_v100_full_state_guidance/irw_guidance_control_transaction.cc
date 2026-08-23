#include "irw_guidance_control_transaction.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace orvd::experiments::irw_crossline_full_state_guidance {
namespace {

constexpr double kCommonModeProbeThresholdNewtonMetres = 1.0e-9;

[[nodiscard]] actuation::WheelDriveTorqueChannelValues
MakeCommonModeProbeRequest(
    const actuation::WheelDriveTorqueChannelValues& raw_requests) {
    auto probe_requests = raw_requests;
    for (std::size_t axle = 0; axle < control::kIrwGuidanceAxleCount;
         ++axle) {
        const std::size_t left = 2 * axle;
        const std::size_t right = left + 1;
        const double common =
            0.5 * (raw_requests[left] + raw_requests[right]);
        if (std::abs(common) > kCommonModeProbeThresholdNewtonMetres) {
            probe_requests[left] = common;
            probe_requests[right] = common;
        }
    }
    return probe_requests;
}

[[nodiscard]] double SanitizeTorqueLimit(double limit) {
    if (!std::isfinite(limit)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, limit);
}

[[nodiscard]] double SignOrOne(double value) {
    return value < 0.0 ? -1.0 : 1.0;
}

[[nodiscard]] actuation::WheelDriveTorqueChannelValues
ApplyLongitudinalCommonModePriority(
    const actuation::WheelDriveTorqueChannelValues& raw_requests,
    const actuation::WheelDriveTorqueChannelValues& dynamic_limits) {
    auto prioritized_requests = raw_requests;
    for (std::size_t axle = 0; axle < control::kIrwGuidanceAxleCount;
         ++axle) {
        const std::size_t left = 2 * axle;
        const std::size_t right = left + 1;
        double common = 0.5 * (raw_requests[left] + raw_requests[right]);
        double differential =
            0.5 * (raw_requests[left] - raw_requests[right]);
        const double left_limit = SanitizeTorqueLimit(dynamic_limits[left]);
        const double right_limit = SanitizeTorqueLimit(dynamic_limits[right]);

        if (!(std::isinf(left_limit) && std::isinf(right_limit))) {
            const double common_limit = std::min(left_limit, right_limit);
            if (std::abs(common) > common_limit) {
                common = SignOrOne(common) * common_limit;
                differential = 0.0;
            } else {
                const double lower_bound =
                    std::max(-left_limit - common, common - right_limit);
                const double upper_bound =
                    std::min(left_limit - common, common + right_limit);
                differential = lower_bound <= upper_bound
                                   ? std::clamp(differential, lower_bound,
                                                upper_bound)
                                   : 0.0;
            }
        }
        prioritized_requests[left] = common + differential;
        prioritized_requests[right] = common - differential;
    }
    return prioritized_requests;
}

}  // namespace

IrwGuidanceControlTransactionResult ComputeIrwGuidanceControlTransaction(
    const control::IrwFullStateWheelSpeedGuidanceRecurrence& recurrence,
    const actuation::WheelDriveTorqueCommandConditioner& conditioner,
    const control::IrwFullStateWheelSpeedGuidanceMechanicalInput&
        mechanical_input,
    const control::IrwFullStateWheelSpeedGuidanceOperatingPoint&
        operating_point,
    const control::IrwFullStateWheelSpeedGuidanceControllerState&
        controller_state,
    const actuation::WheelDriveTorqueChannelValues&
        conditioner_memory_newton_metres) {
    IrwGuidanceControlTransactionResult result;
    result.controller_state_before = controller_state;
    result.conditioner_memory_before_newton_metres =
        conditioner_memory_newton_metres;
    result.controller_result = recurrence.Step(
        mechanical_input, operating_point, controller_state);
    result.common_mode_probe_requests_newton_metres =
        MakeCommonModeProbeRequest(
            result.controller_result.requested_wheel_torques_newton_metres);
    result.common_mode_conditioning_probe = conditioner.Step(
        result.common_mode_probe_requests_newton_metres,
        mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        conditioner_memory_newton_metres);
    result.prioritized_wheel_torque_requests_newton_metres =
        ApplyLongitudinalCommonModePriority(
            result.controller_result.requested_wheel_torques_newton_metres,
            result.common_mode_conditioning_probe
                .wheel_dynamic_torque_limits_newton_metres);
    result.conditioning_result = conditioner.Step(
        result.prioritized_wheel_torque_requests_newton_metres,
        mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        conditioner_memory_newton_metres);
    return result;
}

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
