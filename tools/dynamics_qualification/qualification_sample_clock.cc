#include "qualification_sample_clock.h"

#include <limits>
#include <stdexcept>

namespace orvd::dynamics_qualification {

QualificationSampleClock::QualificationSampleClock(
    std::uint64_t terminal_time_nanoseconds,
    std::uint64_t sample_period_nanoseconds)
    : terminal_time_nanoseconds_(terminal_time_nanoseconds),
      sample_period_nanoseconds_(sample_period_nanoseconds) {
    if (terminal_time_nanoseconds_ == 0) {
        throw std::invalid_argument(
            "qualification sample clock: terminal time must be positive");
    }
    if (sample_period_nanoseconds_ == 0) {
        throw std::invalid_argument(
            "qualification sample clock: sample period must be positive");
    }
    if (terminal_time_nanoseconds_ % sample_period_nanoseconds_ != 0) {
        throw std::invalid_argument(
            "qualification sample clock: terminal time must be an integer "
            "multiple of the sample period");
    }

    terminal_index_ =
        terminal_time_nanoseconds_ / sample_period_nanoseconds_;
    if (terminal_index_ >=
        static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
        throw std::invalid_argument(
            "qualification sample clock: sample count does not fit in "
            "memory indexing");
    }
    sample_count_ = static_cast<std::size_t>(terminal_index_) + 1U;

    constexpr double kSecondsPerNanosecond = 1.0e-9;
    terminal_time_seconds_ =
        static_cast<double>(terminal_time_nanoseconds_) *
        kSecondsPerNanosecond;
    sample_period_seconds_ =
        static_cast<double>(sample_period_nanoseconds_) *
        kSecondsPerNanosecond;
}

double QualificationSampleClock::TargetTimeSeconds(
    std::uint64_t index) const {
    if (index > terminal_index_) {
        throw std::out_of_range(
            "qualification sample clock: sample index exceeds the terminal "
            "index");
    }
    if (index == terminal_index_) {
        return terminal_time_seconds_;
    }
    return static_cast<double>(index) * sample_period_seconds_;
}

std::vector<double> QualificationSampleClock::MakeSampleTimesSeconds() const {
    std::vector<double> result(sample_count_);
    for (std::size_t index = 0; index < result.size(); ++index) {
        result[index] = TargetTimeSeconds(static_cast<std::uint64_t>(index));
        if (index != 0 && !(result[index] > result[index - 1])) {
            throw std::invalid_argument(
                "qualification sample clock: the configured integer clock "
                "does not map to strictly increasing binary64 times");
        }
    }
    return result;
}

}  // namespace orvd::dynamics_qualification
