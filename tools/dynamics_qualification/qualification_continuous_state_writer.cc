#include "qualification_continuous_state_writer.h"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace orvd::dynamics_qualification {
namespace {

Eigen::Index StateSize(int nq, int nv, int nz) {
    if (nq <= 0 || nv < 0 || nz < 0) {
        throw std::invalid_argument("continuous-state writer: invalid q/v/z dimensions");
    }
    const auto size = static_cast<std::uint64_t>(nq) +
                      static_cast<std::uint64_t>(nv) +
                      static_cast<std::uint64_t>(nz);
    if (size > static_cast<std::uint64_t>(std::numeric_limits<Eigen::Index>::max())) {
        throw std::invalid_argument("continuous-state writer: state size exceeds indexing range");
    }
    return static_cast<Eigen::Index>(size);
}

[[noreturn]] void FileError(const std::filesystem::path& path) {
    throw std::runtime_error("continuous-state writer: could not write '" + path.string() + "'");
}

}  // namespace

QualificationContinuousStateWriter::QualificationContinuousStateWriter(
    const std::filesystem::path& path, const QualificationSampleClock& clock,
    std::span<const double> sample_times_seconds, int nq, int nv, int nz)
    : path_(path), clock_(clock), sample_times_seconds_(sample_times_seconds),
      state_size_(StateSize(nq, nv, nz)) {
    if (sample_times_seconds_.size() != clock_.sample_count() ||
        sample_times_seconds_.empty() || sample_times_seconds_.front() != 0.0) {
        throw std::invalid_argument("continuous-state writer: incompatible sample clock");
    }
    for (std::size_t i = 0; i < sample_times_seconds_.size(); ++i) {
        if (!std::isfinite(sample_times_seconds_[i]) ||
            (i != 0 && !(sample_times_seconds_[i] > sample_times_seconds_[i - 1]))) {
            throw std::invalid_argument("continuous-state writer: sample times must be finite and increasing");
        }
    }
    output_.open(path_, std::ios::out | std::ios::trunc);
    if (!output_) FileError(path_);
    output_ << std::setprecision(std::numeric_limits<double>::max_digits10)
            << "sample_index\ttime_nanoseconds\ttime_seconds";
    for (int i = 0; i < nq; ++i) output_ << "\tq." << i;
    for (int i = 0; i < nv; ++i) output_ << "\tv." << i;
    for (int i = 0; i < nz; ++i) output_ << "\tz." << i;
    output_ << '\n';
    if (!output_) FileError(path_);
}

void QualificationContinuousStateWriter::Append(
    std::size_t sample_index,
    const Eigen::Ref<const Eigen::VectorXd>& physical_state) {
    if (closed_ || sample_index != next_sample_ || sample_index >= clock_.sample_count()) {
        throw std::invalid_argument("continuous-state writer: samples must be appended exactly once in clock order");
    }
    if (physical_state.size() != state_size_ || !physical_state.allFinite()) {
        throw std::invalid_argument("continuous-state writer: incompatible or non-finite physical state");
    }
    output_ << sample_index << '\t'
            << clock_.TargetTimeNanoseconds(static_cast<std::uint64_t>(sample_index))
            << '\t' << sample_times_seconds_[sample_index];
    for (Eigen::Index i = 0; i < state_size_; ++i) output_ << '\t' << physical_state[i];
    output_ << '\n';
    if (!output_) FileError(path_);
    ++next_sample_;
}

void QualificationContinuousStateWriter::Close() {
    if (closed_ || next_sample_ != clock_.sample_count()) {
        throw std::invalid_argument("continuous-state writer: incomplete sample clock or already closed");
    }
    output_.close();
    if (!output_) FileError(path_);
    closed_ = true;
}

void WriteQualificationContinuousStates(
    const std::filesystem::path& path, const QualificationSampleClock& clock,
    std::span<const double> sample_times_seconds,
    const Eigen::Ref<const Eigen::MatrixXd>& continuous_states,
    int nq, int nv, int nz) {
    if (continuous_states.rows() != StateSize(nq, nv, nz) ||
        continuous_states.cols() != static_cast<Eigen::Index>(clock.sample_count())) {
        throw std::invalid_argument("continuous-state writer: incompatible matrix layout or sample count");
    }
    QualificationContinuousStateWriter writer(path, clock, sample_times_seconds, nq, nv, nz);
    for (std::size_t i = 0; i < clock.sample_count(); ++i) {
        writer.Append(i, continuous_states.col(static_cast<Eigen::Index>(i)));
    }
    writer.Close();
}

nlohmann::json ContinuousStateObservationContract(const QualificationSampleClock& clock) {
    return {{"file", "continuous_states.tsv"},
            {"row_join_key", {"sample_index", "time_nanoseconds"}},
            {"time_seconds_role", "audit_only"}, {"state_layout", "[q;v;z]"},
            {"float_precision_digits", std::numeric_limits<double>::max_digits10},
            {"sample_count", clock.sample_count()}, {"start_time_nanoseconds", 0},
            {"terminal_time_nanoseconds", clock.terminal_time_nanoseconds()},
            {"sample_period_nanoseconds", clock.sample_period_nanoseconds()}};
}

}  // namespace orvd::dynamics_qualification
