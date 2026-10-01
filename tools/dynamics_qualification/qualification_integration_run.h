#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>

#include <nlohmann/json.hpp>

#include "orvd/configuration/assembled_vehicle_system.h"
#include "qualification_run_accounting.h"
#include "orvd/integrators/system_continuous_state_advancer.h"

namespace orvd::dynamics_qualification {

// One private numerical attempt. It owns the backend, borrows the assembled
// system and accepted context, and collects costs before a failed call unwinds.
// Observation, control and file publication remain with their existing runners.
class QualificationIntegrationRun final {
 public:
    QualificationIntegrationRun(
        const std::filesystem::path& output_directory,
        nlohmann::json configuration_metadata,
        integrators::SystemIntegrationConfiguration
            configuration,
        const configuration::AssembledVehicleSystem& assembled,
        system_assembly::SystemRuntimeContext& accepted,
        double requested_terminal_time_seconds);
    QualificationIntegrationRun(const QualificationIntegrationRun&) = delete;
    QualificationIntegrationRun& operator=(const QualificationIntegrationRun&) = delete;

    Eigen::MatrixXd Advance(double stop, std::span<const double> sample_times);
    void Synchronize();
    [[nodiscard]] const integrators::SystemContinuousStateAdvancer& advancer() const;
    [[nodiscard]] const QualificationIntegrationWorkLedger& ledger() const { return ledger_; }
    [[nodiscard]] const QualificationRunTimings& timings() const { return timings_; }

 private:
    void Capture();
    void RecordFailure(const char* phase, double target, std::exception_ptr error) noexcept;
    std::filesystem::path output_directory_;
    nlohmann::json configuration_metadata_;
    system_assembly::SystemRuntimeContext& accepted_;
    double requested_terminal_time_seconds_;
    QualificationIntegrationWorkLedger ledger_;
    QualificationRunTimings timings_;
    std::unique_ptr<integrators::SystemContinuousStateAdvancer> advancer_;
};

// An estimate for the declared, real stop schedule; never a run admission gate.
void AddQualificationBudgetEstimate(nlohmann::json& metadata,
    std::optional<std::uint64_t> step_nanoseconds,
    std::uint64_t interval_nanoseconds, std::uint64_t interval_count,
    std::size_t budget);

}  // namespace orvd::dynamics_qualification
