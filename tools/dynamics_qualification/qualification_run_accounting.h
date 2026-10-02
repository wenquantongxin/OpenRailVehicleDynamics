#pragma once

#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::forces {
class WheelRailContactForcePlan;
}

namespace orvd::dynamics_qualification {

// A source-tree runner ledger. Only a successful backend construction or
// reinitialization starts an epoch; ordinary public calls do not reset it.
class QualificationIntegrationWorkLedger final {
   public:
    using Statistics = integrators::ContinuousStateIntegrationStatistics;

    void BeginEpoch(const Statistics& current);
    void Capture(const Statistics& current);

    // Preserve known work if reading a backend snapshot or accounting for it
    // failed. A later successful read does not erase this qualification gap.
    void MarkStatisticsUnavailable(std::string detail);

    [[nodiscard]] const Statistics& total_statistics() const noexcept {
        return total_;
    }
    [[nodiscard]] std::uint64_t epoch_count() const noexcept { return epoch_count_; }
    [[nodiscard]] bool statistics_available() const noexcept { return available_; }
    [[nodiscard]] bool statistics_complete() const noexcept {
        return available_ && complete_;
    }
    [[nodiscard]] nlohmann::json ToJson() const;

   private:
    void Account(const Statistics& current, bool begin_epoch);

    Statistics total_{};
    Statistics accounted_{};
    std::optional<int> worker_identity_;
    std::uint64_t epoch_count_{};
    bool available_{};
    bool complete_{true};
    std::vector<std::string> errors_;
};

// A single startup snapshot of requested execution resources. It neither
// starts an OpenMP team nor changes thread placement or numerical behavior.
[[nodiscard]] nlohmann::json CaptureQualificationExecutionConditions(
    const forces::WheelRailContactForcePlan& contact_plan);

[[nodiscard]] nlohmann::json QualificationStatisticsToJson(
    const integrators::ContinuousStateIntegrationStatistics& statistics);

struct QualificationRunTimings final {
    double backend_construction_wall_seconds{};
    double advance_wall_seconds{};
    double synchronization_wall_seconds{};

    [[nodiscard]] double total_wall_seconds() const noexcept;
    [[nodiscard]] nlohmann::json ToJson() const;
};

// The referenced accumulator must outlive the scope. This timer also closes
// during exception unwinding; it never queries or changes an integrator.
class ScopedQualificationRunTimer final {
   public:
    explicit ScopedQualificationRunTimer(double& accumulated_seconds);
    ~ScopedQualificationRunTimer() noexcept;
    ScopedQualificationRunTimer(const ScopedQualificationRunTimer&) = delete;
    ScopedQualificationRunTimer& operator=(const ScopedQualificationRunTimer&) = delete;
    ScopedQualificationRunTimer(ScopedQualificationRunTimer&&) = delete;
    ScopedQualificationRunTimer& operator=(ScopedQualificationRunTimer&&) = delete;

   private:
    double& accumulated_seconds_;
    std::chrono::steady_clock::time_point begin_;
};

// Records public numerical classifications and original non-numerical
// exception categories. Private core failures are translated by the library.
[[nodiscard]] nlohmann::json QualificationFailureToJson(std::exception_ptr failure);

[[nodiscard]] std::filesystem::path FailureResultPath(
    const std::filesystem::path& output_directory);

// The success directory and failure result must be absent. The success
// directory's sibling .partial may remain alive while a failure is recorded.
void RequireFailureResultDestinationAvailable(
    const std::filesystem::path& output_directory);

// Publish an object with status FAILED, through an exclusively owned temporary
// directory and an atomic no-replace hard link. Existing paths are never
// overwritten. Filesystems without hard-link support report an I/O failure.
void PublishFailureResult(const std::filesystem::path& output_directory,
                          const nlohmann::json& payload);

// Intended for a runner's catch block: nullopt means published; a populated
// string describes the secondary write failure (possibly empty under memory
// pressure). The caller retains and rethrows its original exception.
[[nodiscard]] std::optional<std::string> TryPublishFailureResult(
    const std::filesystem::path& output_directory,
    const nlohmann::json& payload) noexcept;

}  // namespace orvd::dynamics_qualification
