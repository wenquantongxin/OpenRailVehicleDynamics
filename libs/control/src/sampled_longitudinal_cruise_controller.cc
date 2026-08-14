#include "orvd/control/sampled_longitudinal_cruise_controller.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::control {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("sampled longitudinal cruise controller: " +
                                detail);
}

void ValidateConfig(
    const SampledLongitudinalCruiseControllerConfig& config) {
    if (config.identifier.empty()) {
        Reject("the identifier is empty");
    }
    if (!std::isfinite(config.sample_period_seconds) ||
        !(config.sample_period_seconds > 0.0)) {
        Reject("sample_period_seconds must be finite and positive");
    }
    if (!std::isfinite(config.target_speed_meters_per_second) ||
        !(config.target_speed_meters_per_second > 0.0)) {
        Reject("target_speed_meters_per_second must be finite and positive");
    }
}

}  // namespace

SampledLongitudinalCruiseController::
    SampledLongitudinalCruiseController(
        SampledLongitudinalCruiseControllerConfig config)
    : config_(std::move(config)), speed_pi_(config_.speed_pi) {
    ValidateConfig(config_);
}

SampledLongitudinalCruiseControllerResult
SampledLongitudinalCruiseController::Step(
    double measured_speed_meters_per_second,
    const SampledLongitudinalCruiseControllerState& previous_state) const {
    if (!std::isfinite(measured_speed_meters_per_second)) {
        Reject("measured_speed_meters_per_second must be finite");
    }
    SampledLongitudinalCruiseControllerResult result;
    result.speed_error_meters_per_second =
        config_.target_speed_meters_per_second -
        measured_speed_meters_per_second;
    const SampledFilteredPiResult pi_result = speed_pi_.Step(
        result.speed_error_meters_per_second,
        config_.sample_period_seconds, previous_state.speed_pi);
    result.requested_common_wheel_torque_newton_metres = pi_result.output;
    result.next_state.speed_pi = pi_result.next_state;
    return result;
}

}  // namespace orvd::control
