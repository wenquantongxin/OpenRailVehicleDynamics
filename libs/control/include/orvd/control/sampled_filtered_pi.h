#pragma once

/// @file
/// One sampled, output-filtered PI channel with caller-owned memory.

namespace orvd::control {

/// Immutable parameters of one sampled, output-filtered PI channel.
struct SampledFilteredPiConfig {
    double proportional_gain{0.0};
    double integral_time_seconds{0.0};
    double output_filter_time_constant_seconds{0.0};
    double integral_absolute_limit{0.0};
    double raw_output_absolute_limit{0.0};
};

/// Memory carried from one accepted sample to the next.
struct SampledFilteredPiState {
    double integral{0.0};
    double filtered_output{0.0};
};

/// One output sample and the state to commit for the next sample.
struct SampledFilteredPiResult {
    double output{0.0};
    SampledFilteredPiState next_state;
};

/// A pure sampled PI calculation with raw-output limiting and first-order
/// output filtering.
///
/// The object owns immutable parameters only. The caller owns and commits the
/// state, and supplies the single sample-period authority used by its enclosing
/// controller.
class SampledFilteredPi {
   public:
    explicit SampledFilteredPi(SampledFilteredPiConfig config);

    [[nodiscard]] const SampledFilteredPiConfig& config() const noexcept {
        return config_;
    }

    [[nodiscard]] SampledFilteredPiResult Step(
        double error, double sample_period_seconds,
        const SampledFilteredPiState& previous_state) const;

   private:
    SampledFilteredPiConfig config_;
};

}  // namespace orvd::control
