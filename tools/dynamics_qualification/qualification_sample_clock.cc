#include "qualification_sample_clock.h"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace orvd::dynamics_qualification {

QualificationSampleClock::QualificationSampleClock(
    std::uint64_t terminal_time_nanoseconds,
    std::uint64_t sample_period_nanoseconds,
    std::optional<QualificationSampleRefinement> local_refinement)
    : terminal_time_nanoseconds_(terminal_time_nanoseconds),
      sample_period_nanoseconds_(sample_period_nanoseconds),
      local_refinement_(local_refinement) {
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

    const std::uint64_t base_terminal_index =
        terminal_time_nanoseconds_ / sample_period_nanoseconds_;
    if (base_terminal_index >=
        static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
        throw std::invalid_argument(
            "qualification sample clock: sample count does not fit in "
            "memory indexing");
    }
    sample_time_nanoseconds_.reserve(
        static_cast<std::size_t>(base_terminal_index) + 1U);
    for (std::uint64_t index = 0; index <= base_terminal_index; ++index) {
        sample_time_nanoseconds_.push_back(index * sample_period_nanoseconds_);
    }

    if (local_refinement_.has_value()) {
        const auto& refinement = *local_refinement_;
        if (refinement.sample_period_nanoseconds == 0 ||
            refinement.begin_time_nanoseconds >=
                refinement.end_time_nanoseconds ||
            refinement.end_time_nanoseconds > terminal_time_nanoseconds_ ||
            refinement.begin_time_nanoseconds %
                    refinement.sample_period_nanoseconds !=
                0 ||
            refinement.end_time_nanoseconds %
                    refinement.sample_period_nanoseconds !=
                0) {
            throw std::invalid_argument(
                "qualification sample clock: local refinement must have a "
                "positive period and a non-empty closed interval inside the "
                "qualification duration on its global integer clock");
        }
        const std::uint64_t refinement_span =
            refinement.end_time_nanoseconds -
            refinement.begin_time_nanoseconds;
        if (refinement_span % refinement.sample_period_nanoseconds != 0) {
            throw std::invalid_argument(
                "qualification sample clock: local-refinement interval must "
                "be an integer multiple of its period");
        }
        const std::uint64_t refinement_terminal_index =
            refinement_span / refinement.sample_period_nanoseconds;
        if (refinement_terminal_index >=
            static_cast<std::uint64_t>(
                std::numeric_limits<std::size_t>::max() -
                sample_time_nanoseconds_.size())) {
            throw std::invalid_argument(
                "qualification sample clock: merged sample count does not "
                "fit in memory indexing");
        }
        sample_time_nanoseconds_.reserve(
            sample_time_nanoseconds_.size() +
            static_cast<std::size_t>(refinement_terminal_index) + 1U);
        for (std::uint64_t index = 0; index <= refinement_terminal_index;
             ++index) {
            sample_time_nanoseconds_.push_back(
                refinement.begin_time_nanoseconds +
                index * refinement.sample_period_nanoseconds);
        }
        std::sort(sample_time_nanoseconds_.begin(),
                  sample_time_nanoseconds_.end());
        sample_time_nanoseconds_.erase(
            std::unique(sample_time_nanoseconds_.begin(),
                        sample_time_nanoseconds_.end()),
            sample_time_nanoseconds_.end());
    }
    sample_count_ = sample_time_nanoseconds_.size();

    constexpr double kSecondsPerNanosecond = 1.0e-9;
    sample_period_seconds_ =
        static_cast<double>(sample_period_nanoseconds_) *
        kSecondsPerNanosecond;
    terminal_time_seconds_ =
        static_cast<double>(base_terminal_index) * sample_period_seconds_;
}

double QualificationSampleClock::TargetTimeSeconds(
    std::uint64_t index) const {
    if (index >= sample_time_nanoseconds_.size()) {
        throw std::out_of_range(
            "qualification sample clock: sample index exceeds the terminal "
            "index");
    }
    const std::uint64_t tick = sample_time_nanoseconds_[
        static_cast<std::size_t>(index)];
    if (tick % sample_period_nanoseconds_ == 0) {
        return static_cast<double>(tick / sample_period_nanoseconds_) *
               sample_period_seconds_;
    }
    const auto& refinement = *local_refinement_;
    constexpr double kSecondsPerNanosecond = 1.0e-9;
    const double refinement_period_seconds =
        static_cast<double>(refinement.sample_period_nanoseconds) *
        kSecondsPerNanosecond;
    return static_cast<double>(tick / refinement.sample_period_nanoseconds) *
           refinement_period_seconds;
}

std::uint64_t QualificationSampleClock::TargetTimeNanoseconds(
    std::uint64_t index) const {
    if (index >= sample_time_nanoseconds_.size()) {
        throw std::out_of_range(
            "qualification sample clock: sample index exceeds the terminal "
            "index");
    }
    return sample_time_nanoseconds_[static_cast<std::size_t>(index)];
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
