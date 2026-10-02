#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <span>

#include <Eigen/Core>
#include <nlohmann/json_fwd.hpp>

#include "qualification_sample_clock.h"

namespace orvd::dynamics_qualification {

// Streams only states already returned by the integrator. The owning runner
// places this file inside its unpublished atomic artifact directory. The clock
// and exact binary64 sample times must outlive the writer.
class QualificationContinuousStateWriter final {
   public:
    QualificationContinuousStateWriter(
        const std::filesystem::path& path, const QualificationSampleClock& clock,
        std::span<const double> sample_times_seconds,
        int generalized_position_count, int generalized_velocity_count,
        int series_force_state_count);

    QualificationContinuousStateWriter(const QualificationContinuousStateWriter&) = delete;
    QualificationContinuousStateWriter& operator=(const QualificationContinuousStateWriter&) = delete;
    QualificationContinuousStateWriter(QualificationContinuousStateWriter&&) = delete;
    QualificationContinuousStateWriter& operator=(QualificationContinuousStateWriter&&) = delete;

    void Append(std::size_t sample_index,
                const Eigen::Ref<const Eigen::VectorXd>& physical_state);
    void Close();

   private:
    std::filesystem::path path_;
    const QualificationSampleClock& clock_;
    std::span<const double> sample_times_seconds_;
    Eigen::Index state_size_{};
    std::size_t next_sample_{};
    std::ofstream output_;
    bool closed_{};
};

void WriteQualificationContinuousStates(
    const std::filesystem::path& path, const QualificationSampleClock& clock,
    std::span<const double> sample_times_seconds,
    const Eigen::Ref<const Eigen::MatrixXd>& continuous_states,
    int generalized_position_count, int generalized_velocity_count,
    int series_force_state_count);

[[nodiscard]] nlohmann::json ContinuousStateObservationContract(
    const QualificationSampleClock& clock);
}  // namespace orvd::dynamics_qualification
