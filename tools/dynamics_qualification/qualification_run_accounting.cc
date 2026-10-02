#include "qualification_run_accounting.h"

#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>

#include <nlohmann/json.hpp>
#include <omp.h>

#include "orvd/forces/wheel_rail_contact_force_plan.h"

#if defined(__linux__)
#include <sched.h>
#endif

namespace orvd::dynamics_qualification {
namespace {

using Statistics = QualificationIntegrationWorkLedger::Statistics;
using CounterMember = std::uint64_t Statistics::*;
constexpr std::array<CounterMember, 8> kCounters{
    &Statistics::successful_internal_step_count,
    &Statistics::right_hand_side_evaluation_count,
    &Statistics::linear_solver_right_hand_side_evaluation_count,
    &Statistics::error_test_failure_count,
    &Statistics::nonlinear_solver_iteration_count,
    &Statistics::nonlinear_solver_convergence_failure_count,
    &Statistics::linear_solver_setup_count,
    &Statistics::jacobian_evaluation_count};
std::uint64_t CheckedAdd(std::uint64_t total, std::uint64_t addition) {
    if (addition > std::numeric_limits<std::uint64_t>::max() - total) {
        throw std::overflow_error("qualification accounting: counter overflow");
    }
    return total + addition;
}

std::uint64_t CounterDelta(std::uint64_t current, std::uint64_t accounted) {
    if (current < accounted) {
        throw std::logic_error(
            "qualification accounting: counter decreased inside an epoch");
    }
    return current - accounted;
}

std::string_view NumericalReason(
    integrators::ContinuousStateNumericalFailure::Reason reason) {
    using Reason = integrators::ContinuousStateNumericalFailure::Reason;
    switch (reason) {
        case Reason::kAdvanceWorkBudgetExhausted: return "advance_work_budget_exhausted";
        case Reason::kRequestedAccuracyUnattainable: return "requested_accuracy_unattainable";
        case Reason::kRepeatedErrorTestFailure: return "repeated_error_test_failure";
        case Reason::kRepeatedNonlinearConvergenceFailure: return "repeated_nonlinear_convergence_failure";
        case Reason::kNonFiniteRightHandSide: return "non_finite_right_hand_side";
        case Reason::kStepSizeUnderflow: return "step_size_underflow";
        case Reason::kRepeatedSingularLinearSystem: return "repeated_singular_linear_system";
        case Reason::kNonFiniteLinearSystem: return "non_finite_linear_system";
        case Reason::kNonFiniteState: return "non_finite_state";
        case Reason::kNonlinearConvergenceFailure: return "nonlinear_convergence_failure";
        case Reason::kSingularLinearSystem: return "singular_linear_system";
    }
    throw std::logic_error("qualification accounting: unknown numerical failure reason");
}

nlohmann::json NonNumericalFailure(std::string_view type, const char* message) {
    return {{"exception_type", type}, {"reason", nullptr},
            {"backend_code", nullptr}, {"message", message}};
}

void RequireFiniteJson(const nlohmann::json& value) {
    if (value.is_number_float() && !std::isfinite(value.get<double>())) {
        throw std::invalid_argument("qualification failure result: non-finite JSON value");
    }
    if (value.is_object() || value.is_array()) {
        for (const auto& child : value) RequireFiniteJson(child);
    }
}

std::filesystem::path FailureTemporaryPath(const std::filesystem::path& final_path) {
    return final_path.parent_path() / (final_path.filename().string() + ".partial");
}

bool EntryExists(const std::filesystem::path& path) {
    return std::filesystem::exists(std::filesystem::symlink_status(path));
}

class OwnedFailureTemporaryDirectory final {
   public:
    explicit OwnedFailureTemporaryDirectory(std::filesystem::path path)
        : path_(std::move(path)) {
        if (!std::filesystem::create_directory(path_)) {
            throw std::runtime_error(
                "qualification failure result: temporary destination already exists");
        }
    }
    ~OwnedFailureTemporaryDirectory() noexcept {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    OwnedFailureTemporaryDirectory(const OwnedFailureTemporaryDirectory&) = delete;
    OwnedFailureTemporaryDirectory& operator=(const OwnedFailureTemporaryDirectory&) = delete;
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

   private:
    std::filesystem::path path_;
};

}  // namespace

nlohmann::json CaptureQualificationExecutionConditions(
    const forces::WheelRailContactForcePlan& contact_plan) {
    const int maximum_threads = omp_get_max_threads();
    nlohmann::json affinity = nullptr;
#if defined(__linux__)
    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    if (sched_getaffinity(0, sizeof(allowed), &allowed) == 0) {
        affinity = nlohmann::json::array();
        for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
            if (CPU_ISSET(cpu, &allowed)) affinity.push_back(cpu);
        }
    }
#endif
    return {{"openmp_runtime_maximum_threads", maximum_threads},
            {"openmp_dynamic_teams_enabled", omp_get_dynamic() != 0},
            {"contact_batch_worker_cap", contact_plan.maximum_worker_count()},
            {"contact_batch_requested_worker_count",
             contact_plan.requested_worker_count()},
            {"cpu_affinity_at_start", std::move(affinity)}};
}

void QualificationIntegrationWorkLedger::BeginEpoch(
    const Statistics& current) {
    Account(current, true);
}

void QualificationIntegrationWorkLedger::Capture(
    const Statistics& current) {
    Account(current, false);
}

void QualificationIntegrationWorkLedger::Account(
    const Statistics& current, bool begin_epoch) {
    if (!begin_epoch && !available_) {
        throw std::logic_error("qualification accounting: no statistics epoch has begun");
    }
    const int workers = current.requested_dense_finite_difference_jacobian_worker_count;
    if (workers < 0) {
        throw std::invalid_argument("qualification accounting: negative worker identity");
    }
    if (worker_identity_.has_value() && *worker_identity_ != workers) {
        throw std::logic_error("qualification accounting: Jacobian worker identity changed");
    }
    // Validate the complete update in local storage before changing the ledger.
    Statistics next_total = total_;
    const Statistics baseline = begin_epoch ? Statistics{} : accounted_;
    for (const CounterMember field : kCounters) {
        next_total.*field = CheckedAdd(next_total.*field,
                                     CounterDelta(current.*field, baseline.*field));
    }
    static_cast<void>(CheckedAdd(next_total.right_hand_side_evaluation_count,
                                next_total.linear_solver_right_hand_side_evaluation_count));
    next_total.requested_dense_finite_difference_jacobian_worker_count = workers;
    const std::uint64_t next_epoch_count =
        begin_epoch ? CheckedAdd(epoch_count_, 1) : epoch_count_;
    total_ = next_total;
    accounted_ = current;
    worker_identity_ = workers;
    epoch_count_ = next_epoch_count;
    available_ = true;
}

void QualificationIntegrationWorkLedger::MarkStatisticsUnavailable(std::string detail) {
    complete_ = false;
    errors_.push_back(std::move(detail));
}

nlohmann::json QualificationStatisticsToJson(const Statistics& value) {
    return {
        {"successful_internal_step_count", value.successful_internal_step_count},
        {"right_hand_side_evaluation_count", value.right_hand_side_evaluation_count},
        {"linear_solver_right_hand_side_evaluation_count", value.linear_solver_right_hand_side_evaluation_count},
        {"error_test_failure_count", value.error_test_failure_count},
        {"nonlinear_solver_iteration_count", value.nonlinear_solver_iteration_count},
        {"nonlinear_solver_convergence_failure_count", value.nonlinear_solver_convergence_failure_count},
        {"linear_solver_setup_count", value.linear_solver_setup_count},
        {"jacobian_evaluation_count", value.jacobian_evaluation_count},
        {"requested_dense_finite_difference_jacobian_worker_count", value.requested_dense_finite_difference_jacobian_worker_count}};
}

nlohmann::json QualificationIntegrationWorkLedger::ToJson() const {
    nlohmann::json result{
        {"availability", !available_ ? "unavailable" : (complete_ ? "complete" : "partial")},
        {"epoch_count", epoch_count_},
        {"integration_statistics", nullptr},
        {"total_right_hand_side_evaluation_count", nullptr},
        {"statistics_errors", errors_}};
    if (available_) {
        result["integration_statistics"] = QualificationStatisticsToJson(total_);
        result["total_right_hand_side_evaluation_count"] = CheckedAdd(
            total_.right_hand_side_evaluation_count,
            total_.linear_solver_right_hand_side_evaluation_count);
    }
    return result;
}

double QualificationRunTimings::total_wall_seconds() const noexcept {
    return backend_construction_wall_seconds + advance_wall_seconds + synchronization_wall_seconds;
}

nlohmann::json QualificationRunTimings::ToJson() const {
    for (const double value : {backend_construction_wall_seconds, advance_wall_seconds,
                               synchronization_wall_seconds, total_wall_seconds()}) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument("qualification accounting: invalid elapsed time");
        }
    }
    return {{"backend_construction_wall_seconds", backend_construction_wall_seconds},
            {"advance_wall_seconds", advance_wall_seconds},
            {"synchronization_wall_seconds", synchronization_wall_seconds},
            {"total_wall_seconds", total_wall_seconds()}};
}

ScopedQualificationRunTimer::ScopedQualificationRunTimer(double& accumulated_seconds)
    : accumulated_seconds_(accumulated_seconds), begin_(std::chrono::steady_clock::now()) {
    if (!std::isfinite(accumulated_seconds) || accumulated_seconds < 0.0) {
        throw std::invalid_argument("qualification accounting: invalid timer accumulator");
    }
}

ScopedQualificationRunTimer::~ScopedQualificationRunTimer() noexcept {
    accumulated_seconds_ += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin_).count();
}

nlohmann::json QualificationFailureToJson(std::exception_ptr failure) {
    if (!failure) throw std::invalid_argument("qualification accounting: failure is empty");
    try {
        std::rethrow_exception(failure);
    } catch (const integrators::ContinuousStateNumericalFailure& error) {
        return {{"exception_type", "continuous_state_numerical_failure"},
                {"reason", NumericalReason(error.reason())},
                {"backend_code", error.backend_code()}, {"message", error.what()}};
    } catch (const std::invalid_argument& error) {
        return NonNumericalFailure("invalid_argument", error.what());
    } catch (const std::logic_error& error) {
        return NonNumericalFailure("logic_error", error.what());
    } catch (const std::runtime_error& error) {
        return NonNumericalFailure("runtime_error", error.what());
    } catch (const std::exception& error) {
        return NonNumericalFailure("std_exception", error.what());
    } catch (...) {
        return NonNumericalFailure("non_standard_exception", "non-standard exception");
    }
}

std::filesystem::path FailureResultPath(const std::filesystem::path& output_directory) {
    if (output_directory.empty() || output_directory.filename().empty() ||
        output_directory.filename() == "." || output_directory.filename() == "..") {
        throw std::invalid_argument("qualification failure result: output must name a directory");
    }
    return output_directory.parent_path() /
           (output_directory.filename().string() + ".failure_result.json");
}

void RequireFailureResultDestinationAvailable(const std::filesystem::path& output_directory) {
    const auto destination = FailureResultPath(output_directory);
    const auto parent = destination.has_parent_path() ? destination.parent_path() : std::filesystem::path(".");
    if (!std::filesystem::is_directory(parent)) {
        throw std::invalid_argument("qualification failure result: parent directory does not exist");
    }
    if (EntryExists(output_directory) || EntryExists(destination) ||
        EntryExists(FailureTemporaryPath(destination))) {
        throw std::invalid_argument("qualification failure result: success or failure destination already exists");
    }
}

void PublishFailureResult(const std::filesystem::path& output_directory,
                          const nlohmann::json& payload) {
    if (!payload.is_object() ||
        (payload.contains("status") && payload.at("status") != "FAILED")) {
        throw std::invalid_argument("qualification failure result: expected an object with status FAILED");
    }
    RequireFiniteJson(payload);
    nlohmann::json result = payload;
    result["status"] = "FAILED";
    const std::string encoded = result.dump(2) + "\n";
    RequireFailureResultDestinationAvailable(output_directory);
    const auto destination = FailureResultPath(output_directory);
    OwnedFailureTemporaryDirectory temporary(FailureTemporaryPath(destination));
    const auto temporary_file = temporary.path() / "failure_result.json";
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.open(temporary_file, std::ios::binary);
    output.write(encoded.data(), static_cast<std::streamsize>(encoded.size()));
    output.close();
    // Unlike rename on POSIX, hard-link creation cannot replace an existing
    // file, even if another publisher created it after our preflight check.
    std::filesystem::create_hard_link(temporary_file, destination);
}

std::optional<std::string> TryPublishFailureResult(
    const std::filesystem::path& output_directory,
    const nlohmann::json& payload) noexcept {
    try {
        PublishFailureResult(output_directory, payload);
        return std::nullopt;
    } catch (const std::exception& error) {
        try { return std::string(error.what()); }
        catch (...) { return std::string{}; }
    } catch (...) {
        return std::string{};
    }
}

}  // namespace orvd::dynamics_qualification
