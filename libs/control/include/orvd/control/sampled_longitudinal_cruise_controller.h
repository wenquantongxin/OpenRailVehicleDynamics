#pragma once

/// @file
/// One sampled longitudinal cruise PI with caller-owned memory.

#include <string>

#include "orvd/control/sampled_filtered_pi.h"

namespace orvd::control {

/// Immutable parameters of one scalar longitudinal cruise controller.
struct SampledLongitudinalCruiseControllerConfig {
    std::string identifier;
    double sample_period_seconds{0.0};
    double target_speed_meters_per_second{0.0};
    SampledFilteredPiConfig speed_pi;
};

/// Memory carried from one accepted cruise event to the next.
struct SampledLongitudinalCruiseControllerState {
    SampledFilteredPiState speed_pi;
};

/// One common wheel-torque request and the state to commit with it.
struct SampledLongitudinalCruiseControllerResult {
    double speed_error_meters_per_second{0.0};
    double requested_common_wheel_torque_newton_metres{0.0};
    SampledLongitudinalCruiseControllerState next_state;
};

/// Pure scalar cruise control around an externally observed common speed.
///
/// Vehicle topology, wheel-rate signs and rolling radius are deliberately
/// outside this class. The caller supplies one measured forward speed and
/// applies the resulting common wheel-torque request to its own actuator
/// topology. The object owns immutable parameters only; event memory remains
/// caller-owned and is committed atomically by the enclosing event session.
class SampledLongitudinalCruiseController {
   public:
    explicit SampledLongitudinalCruiseController(
        SampledLongitudinalCruiseControllerConfig config);

    [[nodiscard]] const SampledLongitudinalCruiseControllerConfig& config()
        const noexcept {
        return config_;
    }

    [[nodiscard]] SampledLongitudinalCruiseControllerResult Step(
        double measured_speed_meters_per_second,
        const SampledLongitudinalCruiseControllerState& previous_state) const;

   private:
    SampledLongitudinalCruiseControllerConfig config_;
    SampledFilteredPi speed_pi_;
};

}  // namespace orvd::control
