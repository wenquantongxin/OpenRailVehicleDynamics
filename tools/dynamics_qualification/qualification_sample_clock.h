#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace orvd::dynamics_qualification {

// The internal qualification archive clock.
//
// Sample identity is an integer nanosecond tick. Floating-point sample times
// are derived from those ticks; they are never formed by repeatedly adding a
// period. The terminal target and the last archive sample are formed by the
// same clock multiplication, so they have one binary64 identity.
class QualificationSampleClock final {
   public:
    QualificationSampleClock(std::uint64_t terminal_time_nanoseconds,
                             std::uint64_t sample_period_nanoseconds);

    [[nodiscard]] std::uint64_t terminal_time_nanoseconds() const noexcept {
        return terminal_time_nanoseconds_;
    }
    [[nodiscard]] std::uint64_t sample_period_nanoseconds() const noexcept {
        return sample_period_nanoseconds_;
    }
    [[nodiscard]] std::uint64_t terminal_index() const noexcept {
        return static_cast<std::uint64_t>(sample_count_ - 1U);
    }
    [[nodiscard]] std::size_t sample_count() const noexcept {
        return sample_count_;
    }
    [[nodiscard]] double terminal_time_seconds() const noexcept {
        return terminal_time_seconds_;
    }
    [[nodiscard]] double sample_period_seconds() const noexcept {
        return sample_period_seconds_;
    }
    [[nodiscard]] double TargetTimeSeconds(std::uint64_t index) const;
    [[nodiscard]] std::uint64_t TargetTimeNanoseconds(
        std::uint64_t index) const;
    [[nodiscard]] std::vector<double> MakeSampleTimesSeconds() const;

   private:
    std::uint64_t terminal_time_nanoseconds_{};
    std::uint64_t sample_period_nanoseconds_{};
    std::size_t sample_count_{};
    double terminal_time_seconds_{};
    double sample_period_seconds_{};
};

}  // namespace orvd::dynamics_qualification
