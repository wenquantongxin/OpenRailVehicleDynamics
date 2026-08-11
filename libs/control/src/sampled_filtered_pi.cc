#include "orvd/control/sampled_filtered_pi.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::control {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("sampled filtered PI: " + detail);
}

void ValidateConfig(const SampledFilteredPiConfig& config) {
    if (!std::isfinite(config.proportional_gain) ||
        !(config.proportional_gain > 0.0)) {
        Reject("proportional_gain must be finite and positive");
    }
    if (!std::isfinite(config.integral_time_seconds) ||
        !(config.integral_time_seconds > 0.0)) {
        Reject("integral_time_seconds must be finite and positive");
    }
    if (!std::isfinite(config.output_filter_time_constant_seconds) ||
        config.output_filter_time_constant_seconds < 0.0) {
        Reject("output_filter_time_constant_seconds must be finite and "
               "nonnegative");
    }
    if (!std::isfinite(config.integral_absolute_limit) ||
        config.integral_absolute_limit < 0.0) {
        Reject("integral_absolute_limit must be finite and nonnegative");
    }
    if (!std::isfinite(config.raw_output_absolute_limit) ||
        config.raw_output_absolute_limit < 0.0) {
        Reject("raw_output_absolute_limit must be finite and nonnegative");
    }
}

double ClampSymmetric(double value, double absolute_limit) {
    if (absolute_limit <= 0.0) {
        return 0.0;
    }
    return std::clamp(value, -absolute_limit, absolute_limit);
}

}  // namespace

SampledFilteredPi::SampledFilteredPi(SampledFilteredPiConfig config)
    : config_(std::move(config)) {
    ValidateConfig(config_);
}

SampledFilteredPiResult SampledFilteredPi::Step(
    double error, double sample_period_seconds,
    const SampledFilteredPiState& previous_state) const {
    if (!std::isfinite(error) || !std::isfinite(sample_period_seconds) ||
        !(sample_period_seconds > 0.0) ||
        !std::isfinite(previous_state.integral) ||
        !std::isfinite(previous_state.filtered_output)) {
        Reject("error, positive sample period and previous state must be "
               "finite");
    }

    SampledFilteredPiResult result;
    result.next_state.integral = ClampSymmetric(
        previous_state.integral + error * sample_period_seconds,
        config_.integral_absolute_limit);
    const double integral_gain =
        config_.proportional_gain / config_.integral_time_seconds;
    const double raw_output = ClampSymmetric(
        config_.proportional_gain * error +
            integral_gain * result.next_state.integral,
        config_.raw_output_absolute_limit);
    const double filter_alpha =
        config_.output_filter_time_constant_seconds <= 0.0
            ? 1.0
            : sample_period_seconds /
                  (config_.output_filter_time_constant_seconds +
                   sample_period_seconds);
    result.next_state.filtered_output =
        (1.0 - filter_alpha) * previous_state.filtered_output +
        filter_alpha * raw_output;
    result.output = result.next_state.filtered_output;
    return result;
}

}  // namespace orvd::control
