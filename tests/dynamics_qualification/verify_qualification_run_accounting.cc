#include "qualification_run_accounting.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

#include <nlohmann/json.hpp>

#include "zhai_core.h"

namespace {

using namespace orvd::dynamics_qualification;
using Statistics = orvd::integrators::ContinuousStateIntegrationStatistics;
using NumericalFailure = orvd::integrators::ContinuousStateNumericalFailure;

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename Exception = std::exception, typename Operation>
void Throws(Operation&& operation, const char* message) {
    bool caught = false;
    try { operation(); } catch (const Exception&) { caught = true; }
    Require(caught, message);
}

Statistics Counted(std::uint64_t value, int workers = 0) {
    return {value, value, value, value, value, value, value, value, workers};
}

void CheckLedgerDeltasAndIdentity() {
    QualificationIntegrationWorkLedger ledger;
    const auto unavailable = ledger.ToJson();
    Require(!ledger.statistics_available() && !ledger.statistics_complete() &&
                unavailable.at("availability") == "unavailable" &&
                unavailable.at("integration_statistics").is_null() &&
                unavailable.at("total_right_hand_side_evaluation_count").is_null(),
            "no snapshot is unavailable, not a fabricated zero-work run");
    Throws<std::logic_error>([&] { ledger.Capture(Counted(0)); },
                              "a capture cannot silently start an epoch");

    Statistics initial{};
    initial.right_hand_side_evaluation_count = 1;
    ledger.BeginEpoch(initial);
    Require(ledger.total_statistics().right_hand_side_evaluation_count == 1 &&
                ledger.total_statistics().requested_dense_finite_difference_jacobian_worker_count == 0 &&
                ledger.epoch_count() == 1 && ledger.statistics_complete(),
            "mechanical initialization is counted and worker zero is a real identity");
    auto current = Counted(5);
    ledger.Capture(current);
    const auto captured = ledger.ToJson();
    ledger.Capture(current);
    Require(ledger.ToJson() == captured, "repeated snapshots add no work");
    current = Counted(8);
    ledger.Capture(current);
    Require(QualificationStatisticsToJson(ledger.total_statistics()) ==
                QualificationStatisticsToJson(current),
            "all counters use within-epoch differences across public calls");

    ledger.BeginEpoch(initial);
    Require(ledger.epoch_count() == 2 &&
                ledger.total_statistics().right_hand_side_evaluation_count == 9 &&
                ledger.total_statistics().linear_solver_right_hand_side_evaluation_count == 8,
            "successful reset immediately includes the new epoch's initial RHS once");
    auto next = initial;
    next.right_hand_side_evaluation_count = 4;
    next.successful_internal_step_count = 3;
    ledger.Capture(next);
    Require(ledger.total_statistics().successful_internal_step_count == 11 &&
                ledger.total_statistics().right_hand_side_evaluation_count == 12 &&
                ledger.ToJson().at("total_right_hand_side_evaluation_count") == 20,
            "a second epoch adds ordinary and Jacobian work without double counting");

    const auto before_refusals = ledger.ToJson();
    auto changed_identity = next;
    changed_identity.requested_dense_finite_difference_jacobian_worker_count = 1;
    Throws<std::logic_error>([&] { ledger.Capture(changed_identity); },
                              "zero-to-one worker drift is rejected");
    Throws<std::logic_error>([&] { ledger.BeginEpoch(changed_identity); },
                              "worker drift across epochs is rejected");
    auto decreased = next;
    decreased.right_hand_side_evaluation_count = 1;
    Throws<std::logic_error>([&] { ledger.Capture(decreased); },
                              "a decreased counter does not infer a successful reset");
    Require(ledger.ToJson() == before_refusals,
            "an invalid snapshot changes none of the accumulated work or epoch state");
    ledger.MarkStatisticsUnavailable("failed synchronization returned a decreased counter");
    Require(ledger.statistics_available() && !ledger.statistics_complete() &&
                ledger.ToJson().at("availability") == "partial" &&
                ledger.total_statistics().right_hand_side_evaluation_count == 12,
            "a missing observation preserves known work and marks completeness honestly");
    ledger.BeginEpoch(initial);
    Require(!ledger.statistics_complete(), "a later snapshot does not erase the earlier accounting gap");

    QualificationIntegrationWorkLedger ode;
    ode.BeginEpoch(Counted(0, 1));
    ode.Capture(Counted(4, 1));
    Require(!ode.ToJson().contains("coordinate_diagnostics"),
            "the tool ledger exposes public work statistics only");
}

void CheckOverflowIsAtomic() {
    QualificationIntegrationWorkLedger ledger;
    auto initial = Counted(0);
    initial.successful_internal_step_count = std::numeric_limits<std::uint64_t>::max();
    ledger.BeginEpoch(initial);
    const auto before = ledger.ToJson();
    auto overflow = Counted(0);
    overflow.successful_internal_step_count = 1;
    Throws<std::overflow_error>([&] { ledger.BeginEpoch(overflow); }, "epoch total overflow is rejected");
    Require(ledger.ToJson() == before, "overflow does not partly add another epoch");

    QualificationIntegrationWorkLedger combined;
    auto rhs = Counted(0);
    rhs.right_hand_side_evaluation_count = std::numeric_limits<std::uint64_t>::max();
    rhs.linear_solver_right_hand_side_evaluation_count = 1;
    Throws<std::overflow_error>([&] { combined.BeginEpoch(rhs); }, "combined RHS total must remain representable");
    Require(!combined.statistics_available(), "overflow cannot create a partial first snapshot");
}

class SwitchableProblem final : public orvd::integrators::internal::CoordinateSecondOrderProblem {
   public:
    int coordinate_size() const override { return 1; }
    int internal_state_size() const override { return 0; }
    void Evaluate(double, const Eigen::Ref<const Eigen::VectorXd>&,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  Eigen::Ref<Eigen::VectorXd> b, Eigen::Ref<Eigen::VectorXd>) override {
        if (fail) throw std::runtime_error("physical evaluation refused");
        b[0] = 1.0;
    }
    bool fail{};
};

void CheckActualCoreInitializationAndFailedReset() {
    using namespace orvd::integrators::internal;
    SwitchableProblem problem;
    const CoordinateState initial{0.0, Eigen::VectorXd::Zero(1),
                                  Eigen::VectorXd::Zero(1), Eigen::VectorXd(0)};
    ZhaiCore core(problem, orvd::integrators::ZhaiConfiguration{0.1}, initial);
    QualificationIntegrationWorkLedger ledger;
    ledger.BeginEpoch(core.integration_statistics());
    for (int step = 0; step < 10; ++step) core.AdvanceOneStep();
    ledger.Capture(core.integration_statistics());
    for (int step = 0; step < 5; ++step) core.AdvanceOneStep();
    ledger.Capture(core.integration_statistics());
    Require(ledger.total_statistics().right_hand_side_evaluation_count == 16 &&
                ledger.total_statistics().successful_internal_step_count == 15,
            "real Zhai initialization and two public accounting intervals are counted once");

    problem.fail = true;
    Throws<std::runtime_error>([&] { core.Reinitialize(initial); }, "real initialization RHS failure is exposed");
    ledger.Capture(core.integration_statistics());
    Require(ledger.epoch_count() == 1 &&
                ledger.total_statistics().right_hand_side_evaluation_count == 17 &&
                ledger.total_statistics().successful_internal_step_count == 15,
            "failed reinitialization appends its attempted RHS to the old epoch");

    problem.fail = false;
    core.Reinitialize(initial);
    ledger.BeginEpoch(core.integration_statistics());
    for (int step = 0; step < 3; ++step) core.AdvanceOneStep();
    ledger.Capture(core.integration_statistics());
    Require(ledger.epoch_count() == 2 &&
                ledger.total_statistics().right_hand_side_evaluation_count == 21,
            "successful recovery opens exactly one new epoch including initialization");
    problem.fail = true;
    Throws<std::runtime_error>([&] { core.AdvanceOneStep(); }, "real positive-step failure is exposed");
    ledger.Capture(core.integration_statistics());
    Require(ledger.total_statistics().right_hand_side_evaluation_count == 22 &&
                ledger.total_statistics().successful_internal_step_count == 18,
            "failed advance preserves the attempted RHS without a successful step");
}

void CheckExceptionSafeTiming() {
    QualificationRunTimings times;
    try {
        ScopedQualificationRunTimer timer(times.backend_construction_wall_seconds);
        const auto until = std::chrono::steady_clock::now() + std::chrono::microseconds(50);
        while (std::chrono::steady_clock::now() < until) {}
        throw std::runtime_error("construction failed after work");
    } catch (const std::runtime_error&) {}
    Require(times.backend_construction_wall_seconds > 0.0 &&
                times.ToJson().at("total_wall_seconds") == times.backend_construction_wall_seconds,
            "exception unwinding closes the construction timer");
    times.advance_wall_seconds = 2.0;
    times.synchronization_wall_seconds = 3.0;
    Require(std::abs(times.total_wall_seconds() - (times.backend_construction_wall_seconds + 5.0)) < 1e-14,
            "numerical lifetime total contains construction, advance and synchronization");
    double invalid = -1.0;
    Throws<std::invalid_argument>([&] { ScopedQualificationRunTimer timer(invalid); },
                                  "invalid timer accumulator is rejected before measurement");
}

class TemporaryDirectory final {
   public:
    TemporaryDirectory() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        const auto parent = std::filesystem::temp_directory_path();
        for (int ordinal = 0; ordinal < 100; ++ordinal) {
            const auto candidate = parent / ("orvd-run-accounting-" + std::to_string(stamp) +
                                             "-" + std::to_string(ordinal));
            if (std::filesystem::create_directory(candidate)) { path = candidate; return; }
        }
        throw std::runtime_error("could not reserve accounting test directory");
    }
    ~TemporaryDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
    std::filesystem::path path;
};

std::string ReadText(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    Require(input.is_open(), "test output file exists");
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void CheckFailureSidecarAndClassification() {
    TemporaryDirectory temporary;
    const auto output = temporary.path / "run";
    const auto success_partial = temporary.path / "run.partial";
    std::filesystem::create_directory(success_partial);
    { std::ofstream marker(success_partial / "owned-by-success-transaction"); marker << "retained"; }

    nlohmann::json failure;
    std::exception_ptr original;
    try {
        throw NumericalFailure(NumericalFailure::Reason::kAdvanceWorkBudgetExhausted,
                               0, "one public advance exhausted its step budget");
    } catch (...) {
        original = std::current_exception();
        failure = QualificationFailureToJson(original);
    }
    Require(failure.at("reason") == "advance_work_budget_exhausted" &&
                failure.at("backend_code") == 0 &&
                failure.at("exception_type") == "continuous_state_numerical_failure",
            "public reason and original backend code have separate serialized identities");
    const nlohmann::json payload{{"phase", "advance"}, {"failure", failure},
                                 {"method_configuration", {{"method", "zhai"}, {"step_size_seconds", 0.01}}}};
    RequireFailureResultDestinationAvailable(output);
    Require(!TryPublishFailureResult(output, payload).has_value(),
            "a failure sidecar can publish while the success .partial directory remains alive");
    const auto destination = FailureResultPath(output);
    const auto bytes = ReadText(destination);
    const auto parsed = nlohmann::json::parse(bytes);
    Require(destination.filename() == "run.failure_result.json" &&
                parsed.at("status") == "FAILED" && parsed.at("failure") == failure &&
                !std::filesystem::exists(output) &&
                !std::filesystem::exists(temporary.path / "run.failure_result.json.partial") &&
                ReadText(success_partial / "owned-by-success-transaction") == "retained",
            "sidecar publication is independent, complete and cleans only its own temporary storage");

    bool original_preserved = false;
    try {
        const auto publication_error = TryPublishFailureResult(output, payload);
        Require(publication_error.has_value(), "publishing a second failure refuses overwrite");
        std::rethrow_exception(original);
    } catch (const NumericalFailure& error) {
        original_preserved = error.reason() == NumericalFailure::Reason::kAdvanceWorkBudgetExhausted;
    }
    Require(original_preserved && ReadText(destination) == bytes,
            "secondary publication failure neither overwrites prior evidence nor replaces the original exception");

    const auto successful = temporary.path / "successful";
    std::filesystem::create_directory(successful);
    Throws<std::invalid_argument>([&] { PublishFailureResult(successful, payload); },
                                  "an existing success directory cannot also acquire a failed result");
    const auto busy = temporary.path / "busy";
    const auto busy_partial = temporary.path / "busy.failure_result.json.partial";
    std::filesystem::create_directory(busy_partial);
    { std::ofstream marker(busy_partial / "foreign"); marker << "untouched"; }
    Throws<std::invalid_argument>([&] { PublishFailureResult(busy, payload); },
                                  "an existing failure staging directory is refused");
    Require(ReadText(busy_partial / "foreign") == "untouched", "foreign partial storage is not removed");
    const auto invalid = temporary.path / "invalid";
    Throws<std::invalid_argument>([&] { PublishFailureResult(invalid, {{"status", "COMPLETE"}}); },
                                  "a failure result cannot carry a success marker");
    Throws<std::invalid_argument>([&] {
        PublishFailureResult(invalid, {{"bad", std::numeric_limits<double>::quiet_NaN()}});
    }, "non-finite values cannot silently become JSON null");
    Require(!std::filesystem::exists(FailureResultPath(invalid)), "invalid payloads publish no sidecar");

    try { throw std::runtime_error("physical model refused the trial"); }
    catch (...) {
        const auto value = QualificationFailureToJson(std::current_exception());
        Require(value.at("exception_type") == "runtime_error" &&
                    value.at("reason").is_null() && value.at("backend_code").is_null() &&
                    value.at("message") == "physical model refused the trial",
                "physical exceptions retain their message without an invented numerical reason");
    }
}

}  // namespace

int main() {
    try {
        CheckLedgerDeltasAndIdentity();
        CheckOverflowIsAtomic();
        CheckActualCoreInitializationAndFailedReset();
        CheckExceptionSafeTiming();
        CheckFailureSidecarAndClassification();
        std::cout << "qualification run accounting verified\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "qualification run accounting failed: " << error.what() << '\n';
        return 1;
    }
}
