#include "orvd/radau5/radau5_core.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace {

using orvd::radau5::Core;
using orvd::radau5::Failure;
using orvd::radau5::RhsEvaluationStatus;
using orvd::radau5::RhsEvaluator;
using orvd::radau5::Statistics;
using orvd::radau5::internal::AssessNewtonCorrection;
using orvd::radau5::internal::SolveShiftedLinearSystemsForQualification;

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void ExpectNear(double actual, double expected, double tolerance,
                const std::string& message) {
    if (!std::isfinite(actual) || !std::isfinite(expected) ||
        !(std::abs(actual - expected) <= tolerance)) {
        throw std::runtime_error(message + ": actual=" +
                                 std::to_string(actual) +
                                 ", expected=" + std::to_string(expected));
    }
}

template <typename Derived>
double VectorInfinityNorm(const Eigen::MatrixBase<Derived>& vector) {
    double result = 0.0;
    for (Eigen::Index index = 0; index < vector.size(); ++index) {
        result = std::max(result, std::abs(vector(index)));
    }
    return result;
}

template <typename Derived>
double MatrixInfinityNorm(const Eigen::MatrixBase<Derived>& matrix) {
    double result = 0.0;
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
        double row_sum = 0.0;
        for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
            row_sum += std::abs(matrix(row, column));
        }
        result = std::max(result, row_sum);
    }
    return result;
}

template <typename Matrix, typename Solution, typename RightHandSide>
double RelativeBackwardError(const Matrix& matrix,
                             const Solution& solution,
                             const RightHandSide& right_hand_side) {
    const double numerator =
        VectorInfinityNorm(matrix * solution - right_hand_side);
    const double denominator =
        MatrixInfinityNorm(matrix) * VectorInfinityNorm(solution) +
        VectorInfinityNorm(right_hand_side);
    Expect(std::isfinite(denominator) && denominator > 0.0,
           "near-singular residual normalization is invalid");
    return numerator / denominator;
}

bool SameStatistics(const Statistics& lhs, const Statistics& rhs) {
    return lhs.successful_internal_step_count ==
               rhs.successful_internal_step_count &&
           lhs.right_hand_side_evaluation_count ==
               rhs.right_hand_side_evaluation_count &&
           lhs.linear_solver_right_hand_side_evaluation_count ==
               rhs.linear_solver_right_hand_side_evaluation_count &&
           lhs.error_test_failure_count == rhs.error_test_failure_count &&
           lhs.nonlinear_solver_iteration_count ==
               rhs.nonlinear_solver_iteration_count &&
           lhs.nonlinear_solver_convergence_failure_count ==
               rhs.nonlinear_solver_convergence_failure_count &&
           lhs.linear_solver_setup_count ==
               rhs.linear_solver_setup_count &&
           lhs.jacobian_evaluation_count ==
               rhs.jacobian_evaluation_count;
}

class FunctionRhs final : public RhsEvaluator {
   public:
    using Function = std::function<void(
        double, const Eigen::Ref<const Eigen::VectorXd>&,
        Eigen::Ref<Eigen::VectorXd>)>;

    FunctionRhs(int state_size, Function function)
        : state_size_(state_size), function_(std::move(function)) {}

    [[nodiscard]] int continuous_state_size() const noexcept override {
        return state_size_;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        ++evaluation_count_;
        if (forced_status_ != RhsEvaluationStatus::kSuccess) {
            return forced_status_;
        }
        try {
            function_(time_seconds, state, derivatives);
            return RhsEvaluationStatus::kSuccess;
        } catch (...) {
            return RhsEvaluationStatus::kFatalFailure;
        }
    }

    void set_forced_status(RhsEvaluationStatus status) {
        forced_status_ = status;
    }

    [[nodiscard]] std::size_t evaluation_count() const noexcept {
        return evaluation_count_;
    }

   private:
    int state_size_;
    Function function_;
    RhsEvaluationStatus forced_status_{RhsEvaluationStatus::kSuccess};
    std::size_t evaluation_count_{};
};

class MutableLinearRhs final : public RhsEvaluator {
   public:
    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        derivatives[0] = slope_ * state[0];
        return RhsEvaluationStatus::kSuccess;
    }

    void set_slope(double slope) noexcept { slope_ = slope; }

   private:
    double slope_{-1.0};
};

class EvaluationPatternRhs final : public RhsEvaluator {
   public:
    explicit EvaluationPatternRhs(std::vector<int> recoverable_attempts)
        : recoverable_attempts_(std::move(recoverable_attempts)) {}

    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double, const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        ++attempt_count_;
        if (std::find(recoverable_attempts_.begin(),
                      recoverable_attempts_.end(),
                      attempt_count_) != recoverable_attempts_.end()) {
            return RhsEvaluationStatus::kRecoverableFailure;
        }
        derivatives[0] = -state[0];
        return RhsEvaluationStatus::kSuccess;
    }

    [[nodiscard]] int attempt_count() const noexcept { return attempt_count_; }

   private:
    std::vector<int> recoverable_attempts_;
    int attempt_count_{};
};

class FatalAttemptRhs final : public RhsEvaluator {
   public:
    explicit FatalAttemptRhs(int fatal_attempt)
        : fatal_attempt_(fatal_attempt) {}

    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double, const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        ++attempt_count_;
        if (attempt_count_ == fatal_attempt_) {
            return RhsEvaluationStatus::kFatalFailure;
        }
        derivatives[0] = -state[0];
        return RhsEvaluationStatus::kSuccess;
    }

    [[nodiscard]] int attempt_count() const noexcept { return attempt_count_; }

   private:
    int fatal_attempt_{};
    int attempt_count_{};
};

class JacobianPerturbationRhs final : public RhsEvaluator {
   public:
    explicit JacobianPerturbationRhs(int recoverable_perturbation_count)
        : recoverable_perturbation_count_(
              recoverable_perturbation_count) {
        perturbations_.reserve(5);
    }

    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        if (time_seconds == 0.0 && state[0] != 1.0) {
            perturbations_.push_back(state[0] - 1.0);
            if (static_cast<int>(perturbations_.size()) <=
                recoverable_perturbation_count_) {
                return RhsEvaluationStatus::kRecoverableFailure;
            }
        }
        derivatives[0] = -state[0];
        return RhsEvaluationStatus::kSuccess;
    }

    [[nodiscard]] const std::vector<double>& perturbations() const noexcept {
        return perturbations_;
    }

   private:
    int recoverable_perturbation_count_{};
    std::vector<double> perturbations_;
};

class LargeStateJacobianPerturbationRhs final : public RhsEvaluator {
   public:
    LargeStateJacobianPerturbationRhs(double baseline_value,
                                      int recoverable_attempt_count)
        : baseline_value_(baseline_value),
          recoverable_attempt_count_(recoverable_attempt_count) {}

    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        if (time_seconds == 0.0 && state[0] != baseline_value_) {
            perturbations_.push_back(state[0] - baseline_value_);
            if (static_cast<int>(perturbations_.size()) <=
                recoverable_attempt_count_) {
                return RhsEvaluationStatus::kRecoverableFailure;
            }
        }
        derivatives[0] = 0.0;
        return RhsEvaluationStatus::kSuccess;
    }

    [[nodiscard]] const std::vector<double>& perturbations() const noexcept {
        return perturbations_;
    }

   private:
    double baseline_value_{};
    int recoverable_attempt_count_{};
    std::vector<double> perturbations_;
};

class MultiColumnJacobianProbeRhs final : public RhsEvaluator {
   public:
    enum class FailureMode {
        kNone,
        kRecoverColumnTwoTwice,
        kFatalColumnTwo,
    };

    explicit MultiColumnJacobianProbeRhs(FailureMode mode) : mode_(mode) {
        perturbation_columns_.reserve(8);
        perturbation_increments_.reserve(8);
    }

    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 4;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        const int active = active_evaluation_count_.fetch_add(1) + 1;
        int observed_maximum = maximum_active_evaluation_count_.load();
        while (active > observed_maximum &&
               !maximum_active_evaluation_count_.compare_exchange_weak(
                   observed_maximum, active)) {
        }
        evaluation_count_.fetch_add(1);

        RhsEvaluationStatus status = RhsEvaluationStatus::kSuccess;
        if (time_seconds == 0.0) {
            constexpr std::array<double, 4> kInitialState{1.0, -2.0, 0.5,
                                                          3.0};
            int changed_column = -1;
            int changed_count = 0;
            for (int column = 0; column < 4; ++column) {
                if (state[column] != kInitialState[column]) {
                    changed_column = column;
                    ++changed_count;
                }
            }
            if (changed_count == 1) {
                std::lock_guard lock(record_mutex_);
                perturbation_columns_.push_back(changed_column);
                perturbation_increments_.push_back(
                    state[changed_column] - kInitialState[changed_column]);
                ++column_attempt_counts_[
                    static_cast<std::size_t>(changed_column)];
                if (changed_column == 2) {
                    if (mode_ == FailureMode::kRecoverColumnTwoTwice &&
                        column_attempt_counts_[2] <= 2) {
                        status = RhsEvaluationStatus::kRecoverableFailure;
                    } else if (mode_ == FailureMode::kFatalColumnTwo) {
                        status = RhsEvaluationStatus::kFatalFailure;
                    }
                }
            }
        }

        if (status == RhsEvaluationStatus::kSuccess) {
            for (int row = 0; row < 4; ++row) {
                derivatives[row] =
                    -static_cast<double>(row + 1) * state[row];
            }
        }
        active_evaluation_count_.fetch_sub(1);
        return status;
    }

    [[nodiscard]] int evaluation_count() const noexcept {
        return evaluation_count_.load();
    }

    [[nodiscard]] int maximum_active_evaluation_count() const noexcept {
        return maximum_active_evaluation_count_.load();
    }

    [[nodiscard]] const std::vector<int>& perturbation_columns() const {
        return perturbation_columns_;
    }

    [[nodiscard]] const std::vector<double>& perturbation_increments() const {
        return perturbation_increments_;
    }

   private:
    FailureMode mode_;
    std::atomic<int> evaluation_count_{};
    std::atomic<int> active_evaluation_count_{};
    std::atomic<int> maximum_active_evaluation_count_{};
    mutable std::mutex record_mutex_;
    std::array<int, 4> column_attempt_counts_{};
    std::vector<int> perturbation_columns_;
    std::vector<double> perturbation_increments_;
};

class ArmedStageFailureRhs final : public RhsEvaluator {
   public:
    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        if (armed_ && !failure_delivered_ &&
            time_seconds > armed_start_time_seconds_) {
            failure_delivered_ = true;
            return RhsEvaluationStatus::kRecoverableFailure;
        }
        derivatives[0] = -state[0];
        return RhsEvaluationStatus::kSuccess;
    }

    void ArmAtAcceptedEndpoint(double time_seconds) noexcept {
        armed_ = true;
        armed_start_time_seconds_ = time_seconds;
        failure_delivered_ = false;
    }

    [[nodiscard]] bool failure_delivered() const noexcept {
        return failure_delivered_;
    }

   private:
    bool armed_{};
    bool failure_delivered_{};
    double armed_start_time_seconds_{};
};

class ArmedPositiveLinearStageFailureRhs final : public RhsEvaluator {
   public:
    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        if (armed_ && !failure_delivered_ &&
            time_seconds > armed_start_time_seconds_) {
            failure_delivered_ = true;
            return RhsEvaluationStatus::kRecoverableFailure;
        }
        derivatives[0] = state[0];
        return RhsEvaluationStatus::kSuccess;
    }

    void ArmAtAcceptedEndpoint(double time_seconds) noexcept {
        armed_ = true;
        armed_start_time_seconds_ = time_seconds;
        failure_delivered_ = false;
    }

    [[nodiscard]] bool failure_delivered() const noexcept {
        return failure_delivered_;
    }

   private:
    bool armed_{};
    bool failure_delivered_{};
    double armed_start_time_seconds_{};
};

class ErrorCorrectionFailureRhs final : public RhsEvaluator {
   public:
    explicit ErrorCorrectionFailureRhs(RhsEvaluationStatus injected_status)
        : injected_status_(injected_status) {}

    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        if (!delivered_ && time_seconds == 0.0 &&
            state[0] < 1.0 - 1.0e-7) {
            delivered_ = true;
            return injected_status_;
        }
        derivatives[0] = -state[0];
        return RhsEvaluationStatus::kSuccess;
    }

    [[nodiscard]] bool delivered() const noexcept { return delivered_; }

   private:
    RhsEvaluationStatus injected_status_;
    bool delivered_{};
};

class OneRecoverableTrialEndRhs final : public RhsEvaluator {
   public:
    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>&,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        ++attempt_count_;
        derivatives[0] = 0.0;
        // Baseline, one Jacobian column, then the three Radau stages.
        if (attempt_count_ == 5) {
            failed_trial_end_time_seconds_ = time_seconds;
            return RhsEvaluationStatus::kRecoverableFailure;
        }
        return RhsEvaluationStatus::kSuccess;
    }

    [[nodiscard]] double failed_trial_end_time_seconds() const noexcept {
        return failed_trial_end_time_seconds_;
    }

   private:
    int attempt_count_{};
    double failed_trial_end_time_seconds_{};
};

class PersistentlyRecoverableStageRhs final : public RhsEvaluator {
   public:
    [[nodiscard]] int continuous_state_size() const noexcept override {
        return 1;
    }

    [[nodiscard]] RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
        if (time_seconds > 0.0) {
            return RhsEvaluationStatus::kRecoverableFailure;
        }
        derivatives[0] = -state[0];
        return RhsEvaluationStatus::kSuccess;
    }
};

Eigen::VectorXd Scalar(double value) {
    Eigen::VectorXd result(1);
    result[0] = value;
    return result;
}

Core MakeScalarCore(FunctionRhs& rhs, double initial_value,
                    double relative_tolerance = 1.0e-7,
                    double absolute_tolerance = 1.0e-9) {
    return Core(rhs, 0.0, Scalar(initial_value), relative_tolerance,
                Scalar(absolute_tolerance));
}

void AdvanceExactly(Core& core, double target_time, double requested_step) {
    Eigen::VectorXd state(core.continuous_state_size());
    const int step_count =
        static_cast<int>(std::llround(target_time / requested_step));
    ExpectNear(step_count * requested_step, target_time, 1.0e-14,
               "fixed-step interval is not integral");
    for (int index = 1; index <= step_count; ++index) {
        const double stop = index == step_count
                                ? target_time
                                : index * requested_step;
        core.SetSuggestedStepSizeForTesting(stop -
                                            core.current_time_seconds());
        const auto before = core.statistics();
        const auto step = core.AdvanceOneAcceptedStepToward(stop);
        const auto after = core.statistics();
        Expect(step.end_time_seconds == stop && step.reached_stop,
               "fixed-step call did not accept exactly the requested step");
        Expect(after.successful_internal_step_count ==
                   before.successful_internal_step_count + 1,
               "fixed-step order test contained adaptive substeps");
    }
    core.CopyCurrentState(state);
}

void AdvanceAdaptively(Core& core, double target_time) {
    constexpr int kMaximumSteps = 100000;
    for (int index = 0; index < kMaximumSteps; ++index) {
        if (core.current_time_seconds() == target_time) return;
        const auto step = core.AdvanceOneAcceptedStepToward(target_time);
        Expect(step.end_time_seconds > step.start_time_seconds &&
                   step.end_time_seconds <= target_time,
               "adaptive core step returned invalid metadata");
        if (step.reached_stop) return;
    }
    throw std::runtime_error("adaptive core step guard was exceeded");
}

void AdvanceOneFixedStep(Core& core, double requested_step) {
    const double stop = core.current_time_seconds() + requested_step;
    const auto before = core.statistics();
    core.SetSuggestedStepSizeForTesting(requested_step);
    const auto step = core.AdvanceOneAcceptedStepToward(stop);
    const auto after = core.statistics();
    Expect(step.reached_stop && step.end_time_seconds == stop,
           "fixed instance-isolation step was adaptively shortened");
    Expect(after.successful_internal_step_count ==
               before.successful_internal_step_count + 1,
           "fixed instance-isolation step changed the accepted-step count "
           "unexpectedly");
}

double LinearDecayError(double step_size) {
    FunctionRhs rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = -state[0];
        });
    Core core(rhs, 0.0, Scalar(1.0), 1.0e-2, Scalar(10.0));
    AdvanceExactly(core, 1.0, step_size);
    Eigen::VectorXd state(1);
    core.CopyCurrentState(state);
    return std::abs(state[0] - std::exp(-1.0));
}

double ExponentialDenseMidpointError(double step_size) {
    FunctionRhs rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = state[0];
        });
    Core core(rhs, 0.0, Scalar(1.0), 1.0e-2, Scalar(10.0));
    core.SetSuggestedStepSizeForTesting(step_size);
    const auto before = core.statistics();
    const auto step = core.AdvanceOneAcceptedStepToward(step_size);
    const auto after = core.statistics();
    Expect(step.reached_stop && step.end_time_seconds == step_size &&
               after.successful_internal_step_count ==
                   before.successful_internal_step_count + 1,
           "dense-order measurement did not accept exactly one fixed step");
    Eigen::VectorXd state(1);
    core.CopyDenseState(0.5 * step_size, state);
    return std::abs(state[0] - std::exp(0.5 * step_size));
}

struct AdaptiveAccuracy final {
    double endpoint_error{};
    double maximum_dense_error{};
    Statistics statistics{};
};

AdaptiveAccuracy MeasureAdaptiveAccuracy(double relative_tolerance) {
    FunctionRhs rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = state[0];
        });
    Core core(rhs, 0.0, Scalar(1.0), relative_tolerance,
              Scalar(0.01 * relative_tolerance));
    constexpr double kTargetTime = 2.0;
    constexpr int kMaximumSteps = 100000;
    double maximum_dense_error = 0.0;
    Eigen::VectorXd dense_state(1);
    for (int index = 0; index < kMaximumSteps; ++index) {
        const auto step = core.AdvanceOneAcceptedStepToward(kTargetTime);
        const auto interval = core.dense_output_interval();
        Expect(interval.has_value() &&
                   interval->start_time_seconds == step.start_time_seconds &&
                   interval->end_time_seconds == step.end_time_seconds,
               "an adaptive step did not publish its dense interval");
        for (double fraction : {0.25, 0.5, 0.75}) {
            const double sample_time =
                step.start_time_seconds +
                fraction *
                    (step.end_time_seconds - step.start_time_seconds);
            core.CopyDenseState(sample_time, dense_state);
            maximum_dense_error =
                std::max(maximum_dense_error,
                         std::abs(dense_state[0] - std::exp(sample_time)));
        }
        if (step.reached_stop) break;
        if (index == kMaximumSteps - 1) {
            throw std::runtime_error(
                "adaptive accuracy step guard was exceeded");
        }
    }
    Eigen::VectorXd endpoint(1);
    core.CopyCurrentState(endpoint);
    return AdaptiveAccuracy{
        std::abs(endpoint[0] - std::exp(kTargetTime)),
        maximum_dense_error, core.statistics()};
}

void VerifyConstructionAndNoOp() {
    FunctionRhs negative_size_rhs(
        -1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
               Eigen::Ref<Eigen::VectorXd>) {});
    bool negative_size_refused = false;
    try {
        Core invalid(negative_size_rhs, 0.0, Eigen::VectorXd{}, 1.0e-7,
                     Eigen::VectorXd{});
    } catch (const std::invalid_argument&) {
        negative_size_refused = true;
    }
    Expect(negative_size_refused,
           "a negative RHS state size was not refused safely");

    FunctionRhs rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 2.0;
        });
    Core core = MakeScalarCore(rhs, 3.0);
    const auto step = core.AdvanceOneAcceptedStepToward(0.0);
    Expect(step.start_time_seconds == 0.0 && step.end_time_seconds == 0.0 &&
               step.reached_stop,
           "same-time core request has wrong metadata");
    Expect(rhs.evaluation_count() == 0,
           "same-time core request evaluated the RHS");
    Expect(core.statistics().successful_internal_step_count == 0,
           "same-time core request changed statistics");

    bool threw = false;
    try {
        Core invalid(rhs, 0.0, Scalar(0.0), 1.0e-15, Scalar(1.0e-9));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    Expect(threw, "Radau5 accepted the forbidden relative-tolerance bound");

    const auto expect_transform_failure = [&](double relative_tolerance,
                                              double absolute_tolerance,
                                              const char* message) {
        bool rejected_as_unsupported_input = false;
        try {
            Core invalid(rhs, 0.0, Scalar(0.0), relative_tolerance,
                         Scalar(absolute_tolerance));
        } catch (const std::invalid_argument&) {
            rejected_as_unsupported_input = true;
        }
        Expect(rejected_as_unsupported_input, message);
    };
    expect_transform_failure(
        1.0e300, 1.0e-30,
        "an underflowed absolute/relative tolerance ratio was not a "
        "construction input refusal");
    expect_transform_failure(
        2.0e-15, 1.0e300,
        "an overflowed absolute/relative tolerance ratio was not a "
        "construction input refusal");
}

void VerifyElementaryProblemsAndDenseOutput() {
    FunctionRhs constant_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 2.0;
        });
    Core constant = MakeScalarCore(constant_rhs, 3.0);
    constant.SetSuggestedStepSizeForTesting(0.25);
    const auto step = constant.AdvanceOneAcceptedStepToward(0.25);
    Eigen::VectorXd state(1);
    constant.CopyCurrentState(state);
    ExpectNear(state[0], 3.5, 2.0e-12,
               "constant derivative endpoint is inaccurate");
    Expect(step.start_time_seconds == 0.0 && step.end_time_seconds == 0.25 &&
               step.reached_stop,
           "constant derivative step metadata is wrong");

    FunctionRhs polynomial_rhs(
        1, [](double time, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 3.0 * time * time;
        });
    Core polynomial = MakeScalarCore(polynomial_rhs, 0.0, 1.0e-5, 1.0e-8);
    polynomial.SetSuggestedStepSizeForTesting(0.4);
    static_cast<void>(polynomial.AdvanceOneAcceptedStepToward(0.4));
    const auto interval = polynomial.dense_output_interval();
    Expect(interval.has_value() && interval->start_time_seconds == 0.0 &&
               interval->end_time_seconds == 0.4,
           "dense interval is unavailable after an accepted step");
    polynomial.CopyDenseState(0.2, state);
    ExpectNear(state[0], std::pow(0.2, 3), 5.0e-12,
               "collocation dense output is inaccurate");
    polynomial.CopyDenseState(0.0, state);
    Expect(state[0] == 0.0, "dense start endpoint was not copied exactly");
    polynomial.CopyDenseState(0.4, state);
    Eigen::VectorXd endpoint(1);
    polynomial.CopyCurrentState(endpoint);
    Expect(state[0] == endpoint[0],
           "dense end endpoint was not copied exactly");

    constexpr double kLargeTime = 1.0e12;
    constexpr double kTrialStep = 0.002;
    FunctionRhs large_time_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 1.0;
        });
    Core large_time(large_time_rhs, kLargeTime, Scalar(0.0), 1.0e-5,
                    Scalar(1.0e-8));
    large_time.SetSuggestedStepSizeForTesting(kTrialStep);
    static_cast<void>(
        large_time.AdvanceOneAcceptedStepToward(kLargeTime + 1.0));
    const auto large_interval = large_time.dense_output_interval();
    Expect(large_interval.has_value(),
           "large-time step did not publish dense output");
    const double public_width = large_interval->end_time_seconds -
                                large_interval->start_time_seconds;
    Expect(public_width != kTrialStep,
           "large-time regression did not exercise rounded endpoint width");
    const double midpoint =
        large_interval->start_time_seconds + 0.5 * public_width;
    large_time.CopyDenseState(midpoint, state);
    const double expected_from_trial_step =
        kTrialStep +
        (midpoint - large_interval->end_time_seconds);
    ExpectNear(state[0], expected_from_trial_step, 2.0e-12,
               "dense output did not retain the accepted trial-step "
               "identity");
}

void VerifyFifthOrderTrend() {
    const std::vector<double> steps{0.2, 0.1, 0.05, 0.025};
    std::vector<double> errors;
    for (double step : steps) {
        errors.push_back(LinearDecayError(step));
    }
    for (std::size_t index = 1; index < errors.size(); ++index) {
        const double ratio = errors[index - 1] / errors[index];
        Expect(ratio > 24.0 && ratio < 40.0,
               "fixed-step endpoint errors do not show fifth-order trend");
    }
}

void VerifyAdaptiveToleranceAndDenseOutputRefinement() {
    const AdaptiveAccuracy coarse = MeasureAdaptiveAccuracy(1.0e-3);
    const AdaptiveAccuracy medium = MeasureAdaptiveAccuracy(1.0e-5);
    const AdaptiveAccuracy fine = MeasureAdaptiveAccuracy(1.0e-7);

    Expect(medium.endpoint_error < 0.35 * coarse.endpoint_error &&
               fine.endpoint_error < 0.35 * medium.endpoint_error,
           "adaptive endpoint error did not refine with tolerance");
    Expect(medium.maximum_dense_error <
                   0.45 * coarse.maximum_dense_error &&
               fine.maximum_dense_error <
                   0.45 * medium.maximum_dense_error,
           "adaptive dense-output error did not refine with tolerance");
    Expect(fine.endpoint_error < 1.0e-7 &&
               fine.maximum_dense_error < 2.0e-6,
           "finest adaptive endpoint or dense-output result is inaccurate: "
           "endpoint_nano=" +
               std::to_string(1.0e9 * fine.endpoint_error) +
               ", dense_nano=" +
               std::to_string(1.0e9 * fine.maximum_dense_error));
    Expect(coarse.statistics.successful_internal_step_count <
                   medium.statistics.successful_internal_step_count &&
               medium.statistics.successful_internal_step_count <
                   fine.statistics.successful_internal_step_count,
           "tolerance refinement did not increase accepted-step work");
}

void VerifyDenseOutputInteriorOrder() {
    const std::vector<double> steps{0.4, 0.2, 0.1, 0.05};
    std::vector<double> errors;
    for (double step : steps) {
        errors.push_back(ExponentialDenseMidpointError(step));
    }
    for (std::size_t index = 1; index < errors.size(); ++index) {
        const double ratio = errors[index - 1] / errors[index];
        Expect(ratio > 12.0 && ratio < 20.0,
               "dense midpoint errors do not show fourth-order trend");
    }
    Expect(errors.back() < 1.0e-8,
           "finest dense midpoint result is inaccurate");
}

void VerifyOscillatorAndInstanceIsolation() {
    auto oscillator = [](double,
                         const Eigen::Ref<const Eigen::VectorXd>& state,
                         Eigen::Ref<Eigen::VectorXd> derivatives) {
        derivatives[0] = state[1];
        derivatives[1] = -state[0];
    };
    auto decay = [](double,
                    const Eigen::Ref<const Eigen::VectorXd>& state,
                    Eigen::Ref<Eigen::VectorXd> derivatives) {
        derivatives[0] = -2.0 * state[0];
    };
    FunctionRhs rhs_a(2, oscillator);
    FunctionRhs rhs_b(1, decay);
    FunctionRhs reference_rhs_a(2, oscillator);
    FunctionRhs reference_rhs_b(1, decay);
    Eigen::Vector2d initial_a(1.0, 0.0);
    Eigen::Vector2d tolerances_a(1.0e-9, 1.0e-9);
    Core a(rhs_a, 0.0, initial_a, 1.0e-7, tolerances_a);
    Core b(rhs_b, -0.5, Scalar(3.0), 3.0e-6, Scalar(7.0e-8));
    Core reference_a(reference_rhs_a, 0.0, initial_a, 1.0e-7,
                     tolerances_a);
    Core reference_b(reference_rhs_b, -0.5, Scalar(3.0), 3.0e-6,
                     Scalar(7.0e-8));

    int b_step_count = 0;
    for (int index = 0; index < 20; ++index) {
        AdvanceOneFixedStep(a, 0.05);
        if (index % 3 != 1) {
            AdvanceOneFixedStep(b, 0.03);
            ++b_step_count;
        }
    }
    for (int index = 0; index < 20; ++index) {
        AdvanceOneFixedStep(reference_a, 0.05);
    }
    for (int index = 0; index < b_step_count; ++index) {
        AdvanceOneFixedStep(reference_b, 0.03);
    }
    Eigen::Vector2d state_a;
    Eigen::Vector2d reference_state_a;
    Eigen::VectorXd state_b(1);
    Eigen::VectorXd reference_state_b(1);
    a.CopyCurrentState(state_a);
    reference_a.CopyCurrentState(reference_state_a);
    b.CopyCurrentState(state_b);
    reference_b.CopyCurrentState(reference_state_b);
    ExpectNear(state_a[0], std::cos(1.0), 2.0e-8,
               "oscillator cosine endpoint is inaccurate");
    ExpectNear(state_a[1], -std::sin(1.0), 2.0e-8,
               "oscillator sine endpoint is inaccurate");
    ExpectNear(state_b[0], 3.0 * std::exp(-2.0 * 0.03 * b_step_count),
               2.0e-8, "interleaved scalar decay is inaccurate");
    Expect((state_a - reference_state_a).norm() == 0.0 &&
               state_b[0] == reference_state_b[0],
           "interleaving changed an instance endpoint");
    Expect(a.statistics().successful_internal_step_count == 20 &&
               b.statistics().successful_internal_step_count ==
                   static_cast<std::uint64_t>(b_step_count),
           "interleaved cores have incorrect accepted-step counts");
    Expect(SameStatistics(a.statistics(), reference_a.statistics()) &&
               SameStatistics(b.statistics(), reference_b.statistics()),
           "interleaving changed instance-owned numerical statistics");
    Expect(rhs_a.evaluation_count() ==
               a.statistics().right_hand_side_evaluation_count +
                   a.statistics()
                       .linear_solver_right_hand_side_evaluation_count &&
               rhs_b.evaluation_count() ==
                   b.statistics().right_hand_side_evaluation_count +
                       b.statistics()
                           .linear_solver_right_hand_side_evaluation_count,
           "instance callback counts do not match their own statistics");
}

void VerifyStiffProblemsAndAdaptiveRejection() {
    FunctionRhs error_rejection_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& input,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = -input[0];
        });
    Core error_rejection(error_rejection_rhs, 0.0, Scalar(1.0), 1.0e-7,
                         Scalar(1.0e-9));
    error_rejection.SetSuggestedStepSizeForTesting(0.2);
    const auto error_recovered =
        error_rejection.AdvanceOneAcceptedStepToward(0.2);
    const auto error_rejection_work = error_rejection.statistics();
    Expect(error_recovered.end_time_seconds > 0.0 &&
               error_recovered.end_time_seconds < 0.2 &&
               error_rejection_work.error_test_failure_count == 1 &&
               error_rejection_work
                       .nonlinear_solver_convergence_failure_count == 0 &&
               error_rejection_work.successful_internal_step_count == 1,
           "a pure error-test rejection was not classified independently");

    constexpr double kLambda = -1000.0;
    FunctionRhs prothero_robinson(
        1, [](double time, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] =
                kLambda * (state[0] - std::sin(time)) + std::cos(time);
        });
    Core prothero(prothero_robinson, 0.0, Scalar(0.0), 1.0e-8,
                  Scalar(1.0e-10));
    AdvanceAdaptively(prothero, 1.0);
    Eigen::VectorXd state(1);
    prothero.CopyCurrentState(state);
    ExpectNear(state[0], std::sin(1.0), 2.0e-7,
               "stiff Prothero-Robinson endpoint is inaccurate");

    FunctionRhs stiff_decay(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& input,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = -10000.0 * input[0];
        });
    Core rejected(stiff_decay, 0.0, Scalar(1.0), 1.0e-9,
                  Scalar(1.0e-12));
    rejected.SetSuggestedStepSizeForTesting(1.0);
    const auto first = rejected.AdvanceOneAcceptedStepToward(1.0);
    const auto work = rejected.statistics();
    Expect(first.end_time_seconds > 0.0 && first.end_time_seconds < 1.0,
           "a deliberately oversized stiff trial was not reduced");
    Expect(work.error_test_failure_count > 0 ||
               work.nonlinear_solver_convergence_failure_count > 0,
           "adaptive stiff retry was not visible in rejection statistics");
    AdvanceAdaptively(rejected, 1.0);
    rejected.CopyCurrentState(state);
    Expect(std::abs(state[0]) < 1.0e-10,
           "stiff scalar decay did not reach its asymptotic endpoint");
}

void VerifyRobertsonAndStiffVanDerPol() {
    FunctionRhs robertson_rhs(
        3, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] =
                -0.04 * state[0] + 1.0e4 * state[1] * state[2];
            derivatives[2] = 3.0e7 * state[1] * state[1];
            derivatives[1] = -derivatives[0] - derivatives[2];
        });
    Eigen::Vector3d robertson_initial(1.0, 0.0, 0.0);
    Eigen::Vector3d robertson_tolerances(1.0e-10, 1.0e-14, 1.0e-10);
    Core robertson(robertson_rhs, 0.0, robertson_initial, 1.0e-7,
                   robertson_tolerances);
    AdvanceAdaptively(robertson, 40.0);
    Eigen::Vector3d robertson_state;
    robertson.CopyCurrentState(robertson_state);
    constexpr std::array<double, 3> kRobertsonAtForty{
        7.158270687e-1, 9.18553476e-6, 2.841637457e-1};
    for (int index = 0; index < 3; ++index) {
        ExpectNear(robertson_state[index], kRobertsonAtForty[index],
                   index == 1 ? 2.0e-9 : 2.0e-6,
                   "Robertson endpoint is inaccurate");
        Expect(robertson_state[index] >= -2.0e-12,
               "Robertson integration produced a negative concentration");
    }
    ExpectNear(robertson_state.sum(), 1.0, 3.0e-11,
               "Robertson mass conservation was lost");
    const auto robertson_work = robertson.statistics();
    Expect(robertson_work.successful_internal_step_count > 1 &&
               robertson_work.jacobian_evaluation_count > 0 &&
               robertson_work.linear_solver_setup_count > 0 &&
               robertson_work.nonlinear_solver_iteration_count > 0,
           "Robertson run did not exercise the implicit solver work");

    constexpr double kEpsilon = 1.0e-6;
    FunctionRhs van_der_pol_rhs(
        2, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = state[1];
            derivatives[1] =
                ((1.0 - state[0] * state[0]) * state[1] - state[0]) /
                kEpsilon;
        });
    Eigen::Vector2d van_der_pol_initial(2.0, -0.66);
    Eigen::Vector2d van_der_pol_tolerances(1.0e-7, 1.0e-7);
    Core van_der_pol(van_der_pol_rhs, 0.0, van_der_pol_initial, 1.0e-5,
                     van_der_pol_tolerances);
    AdvanceAdaptively(van_der_pol, 2.0);
    Eigen::Vector2d van_der_pol_state;
    van_der_pol.CopyCurrentState(van_der_pol_state);
    ExpectNear(van_der_pol_state[0], 1.70616744, 5.0e-5,
               "stiff Van der Pol position is inaccurate");
    ExpectNear(van_der_pol_state[1], -0.89281002, 5.0e-5,
               "stiff Van der Pol velocity is inaccurate");
    const auto van_der_pol_work = van_der_pol.statistics();
    Expect(van_der_pol_work.successful_internal_step_count > 1 &&
               van_der_pol_work.jacobian_evaluation_count > 0 &&
               van_der_pol_work.linear_solver_setup_count > 0 &&
               van_der_pol_work.nonlinear_solver_iteration_count > 0,
           "stiff Van der Pol run did not exercise implicit solver work");
}

void VerifyPredictiveControllerIsAcceptedOnly() {
    constexpr double kSwitchTimeSeconds = 0.1;
    constexpr double kTrialStepSizeSeconds = 0.1;
    constexpr double kRelativeTolerance = 1.0e-6;
    constexpr double kTargetError = 2.0;
    // For f(t)=a(t-t_switch)^4, applying the official three-stage Radau IIA
    // matrix and embedded-error solve gives |error_vector|=a*h^5*C.
    constexpr double kQuarticErrorCoefficient = 0.04947998932722194;
    constexpr int kMaximumNewtonIterations = 7;

    const double transformed_tolerance =
        0.1 * std::pow(kRelativeTolerance, 2.0 / 3.0);
    const double quartic_amplitude =
        kTargetError * (2.0 * transformed_tolerance) /
        (kQuarticErrorCoefficient *
         std::pow(kTrialStepSizeSeconds, 5));

    FunctionRhs rhs(
        1, [quartic_amplitude](
               double time_seconds,
               const Eigen::Ref<const Eigen::VectorXd>&,
               Eigen::Ref<Eigen::VectorXd> derivatives) {
            const double shifted_time = time_seconds - kSwitchTimeSeconds;
            derivatives[0] = shifted_time <= 0.0
                                 ? 0.0
                                 : quartic_amplitude *
                                       std::pow(shifted_time, 4);
        });
    Core core(rhs, 0.0, Scalar(1.0), kRelativeTolerance,
              Scalar(kRelativeTolerance));
    core.SetSuggestedStepSizeForTesting(kTrialStepSizeSeconds);
    const auto history_step =
        core.AdvanceOneAcceptedStepToward(kSwitchTimeSeconds);
    Expect(history_step.reached_stop &&
               core.statistics().successful_internal_step_count == 1,
           "controller regression did not establish accepted history");

    core.SetSuggestedStepSizeForTesting(kTrialStepSizeSeconds);
    const auto before = core.statistics();
    const auto recovered = core.AdvanceOneAcceptedStepToward(
        kSwitchTimeSeconds + kTrialStepSizeSeconds);
    const auto after = core.statistics();

    constexpr int kRejectedTrialNewtonIterations = 2;
    const double safety = std::min(
        0.9, 0.9 * (1.0 + 2.0 * kMaximumNewtonIterations) /
                 (kRejectedTrialNewtonIterations +
                  2.0 * kMaximumNewtonIterations));
    const double ordinary_quotient = std::clamp(
        std::pow(kTargetError, 0.25) / safety, 1.0 / 8.0, 5.0);
    const double expected_retry_step =
        kTrialStepSizeSeconds / ordinary_quotient;
    const double accepted_step_size =
        recovered.end_time_seconds - recovered.start_time_seconds;

    Expect(after.error_test_failure_count ==
                   before.error_test_failure_count + 1 &&
               after.successful_internal_step_count ==
                   before.successful_internal_step_count + 1,
           "post-acceptance controller regression did not reject exactly one "
           "error trial and recover");
    ExpectNear(accepted_step_size, expected_retry_step, 2.0e-13,
               "an error rejection after accepted history used the "
               "Gustafsson predictive quotient instead of the ordinary "
               "controller quotient");
}

void VerifySixthNewtonPredictionBoundary() {
    // At NEWT=6 with NIT=7, the prediction exponent is exactly zero.  With
    // theta=0.5, FACCON is one, so the prediction and the post-application
    // convergence test meet at correction_norm/newton_tolerance == 1.
    const auto accepted = AssessNewtonCorrection(
        6, 0.5, 1.0, 0.5, 0.1, 0.25, 1.0);
    Expect(!accepted.reject_trial &&
               accepted.converges_after_application,
           "the sixth Newton prediction refused its reachable dyth<1 "
           "acceptance side");
    ExpectNear(accepted.theta, 0.5, 0.0,
               "the sixth Newton acceptance-side theta is wrong");
    ExpectNear(accepted.faccon, 1.0, 0.0,
               "the sixth Newton acceptance-side FACCON is wrong");

    const auto rejected = AssessNewtonCorrection(
        6, 1.0, 2.0, 0.5, 0.1, 0.25, 1.0);
    Expect(rejected.reject_trial &&
               !rejected.converges_after_application,
           "the sixth Newton prediction did not refuse the dyth==1 "
           "boundary before applying the correction");
    ExpectNear(rejected.theta, 0.5, 0.0,
               "the sixth Newton rejection-side theta is wrong");
    ExpectNear(rejected.faccon, 1.0, 0.0,
               "the sixth Newton rejection-side FACCON is wrong");
    ExpectNear(rejected.retry_multiplier, 0.8, 0.0,
               "the sixth Newton boundary used the wrong predictive retry");
}

void VerifyMultiColumnSerialJacobianBaseline() {
    const Eigen::Vector4d initial_state(1.0, -2.0, 0.5, 3.0);
    const Eigen::Vector4d absolute_tolerances =
        Eigen::Vector4d::Constant(1.0e-10);
    constexpr double kStopTime = 1.0e-4;
    constexpr double kRoundingUnit = 1.0e-16;
    Eigen::Vector4d expected_endpoint;
    std::array<double, 4> expected_initial_increments{};
    for (int column = 0; column < 4; ++column) {
        expected_endpoint[column] =
            initial_state[column] *
            std::exp(-static_cast<double>(column + 1) * kStopTime);
        const double nominal_increment = std::sqrt(
            kRoundingUnit *
            std::max(1.0e-5, std::abs(initial_state[column])));
        expected_initial_increments[static_cast<std::size_t>(column)] =
            (initial_state[column] + nominal_increment) -
            initial_state[column];
    }

    MultiColumnJacobianProbeRhs successful_rhs(
        MultiColumnJacobianProbeRhs::FailureMode::kNone);
    Core successful(successful_rhs, 0.0, initial_state, 1.0e-8,
                    absolute_tolerances);
    successful.SetSuggestedStepSizeForTesting(kStopTime);
    const auto successful_step =
        successful.AdvanceOneAcceptedStepToward(kStopTime);
    const auto successful_work = successful.statistics();
    const std::vector<int> expected_columns{0, 1, 2, 3};
    Expect(successful_step.reached_stop &&
               successful_work.successful_internal_step_count == 1 &&
               successful_work.jacobian_evaluation_count == 1 &&
               successful_work.linear_solver_right_hand_side_evaluation_count ==
                   4 &&
               successful_rhs.perturbation_columns() == expected_columns &&
               successful_rhs.perturbation_increments().size() == 4 &&
               std::all_of(
                   successful_rhs.perturbation_increments().begin(),
                   successful_rhs.perturbation_increments().end(),
                   [](double increment) {
                       return std::isfinite(increment) && increment > 0.0;
                   }) &&
               successful_rhs.maximum_active_evaluation_count() == 1 &&
               static_cast<std::uint64_t>(successful_rhs.evaluation_count()) ==
                   successful_work.right_hand_side_evaluation_count +
                       successful_work
                           .linear_solver_right_hand_side_evaluation_count,
           "the four-column serial Jacobian baseline lost its one-column "
           "tasks, accounting, or non-overlap identity");
    for (int column = 0; column < 4; ++column) {
        ExpectNear(
            successful_rhs.perturbation_increments()[
                static_cast<std::size_t>(column)],
            expected_initial_increments[static_cast<std::size_t>(column)], 0.0,
            "a serial Jacobian column lost its representable perturbation");
    }
    Eigen::Vector4d successful_endpoint;
    Eigen::Vector4d successful_dense_endpoint;
    successful.CopyCurrentState(successful_endpoint);
    successful.CopyDenseState(kStopTime, successful_dense_endpoint);
    Expect((successful_endpoint - expected_endpoint)
                       .lpNorm<Eigen::Infinity>() < 1.0e-12 &&
               (successful_dense_endpoint - successful_endpoint)
                       .lpNorm<Eigen::Infinity>() < 1.0e-14,
           "the serial four-column Jacobian produced an inaccurate accepted "
           "or dense endpoint");

    MultiColumnJacobianProbeRhs recoverable_rhs(
        MultiColumnJacobianProbeRhs::FailureMode::kRecoverColumnTwoTwice);
    Core recoverable(recoverable_rhs, 0.0, initial_state, 1.0e-8,
                     absolute_tolerances);
    recoverable.SetSuggestedStepSizeForTesting(kStopTime);
    const auto recoverable_step =
        recoverable.AdvanceOneAcceptedStepToward(kStopTime);
    const auto recoverable_work = recoverable.statistics();
    const std::vector<int> expected_retried_columns{0, 1, 2, 2, 2, 3};
    const auto& recoverable_increments =
        recoverable_rhs.perturbation_increments();
    Expect(recoverable_work.successful_internal_step_count == 1 &&
               recoverable_work.jacobian_evaluation_count == 1 &&
               recoverable_work.linear_solver_right_hand_side_evaluation_count ==
                   6 &&
               recoverable_rhs.perturbation_columns() ==
                   expected_retried_columns &&
               recoverable_increments.size() == 6 &&
               std::all_of(
                   recoverable_increments.begin(),
                   recoverable_increments.end(), [](double increment) {
                       return std::isfinite(increment) && increment > 0.0;
                   }) &&
               recoverable_rhs.maximum_active_evaluation_count() == 1 &&
               static_cast<std::uint64_t>(recoverable_rhs.evaluation_count()) ==
                   recoverable_work.right_hand_side_evaluation_count +
                       recoverable_work
                           .linear_solver_right_hand_side_evaluation_count,
           "a recoverable middle column escaped the serial per-column "
           "retry baseline");
    if (recoverable_increments.size() == 6) {
        ExpectNear(recoverable_increments[0],
                   expected_initial_increments[0], 0.0,
                   "recoverable evaluation changed column zero's increment");
        ExpectNear(recoverable_increments[1],
                   expected_initial_increments[1], 0.0,
                   "recoverable evaluation changed column one's increment");
        ExpectNear(recoverable_increments[2],
                   expected_initial_increments[2], 0.0,
                   "recoverable evaluation changed column two's increment");
        ExpectNear(recoverable_increments[5],
                   expected_initial_increments[3], 0.0,
                   "recoverable evaluation changed column three's increment");
        ExpectNear(recoverable_increments[3] / recoverable_increments[2],
                   0.1, 2.0e-7,
                   "the middle-column first retry did not shrink by ten");
        ExpectNear(recoverable_increments[4] / recoverable_increments[3],
                   0.1, 2.0e-6,
                   "the middle-column second retry did not shrink by ten");
    }
    Eigen::Vector4d recoverable_endpoint;
    Eigen::Vector4d recoverable_dense_endpoint;
    recoverable.CopyCurrentState(recoverable_endpoint);
    recoverable.CopyDenseState(kStopTime, recoverable_dense_endpoint);
    Expect(recoverable_step.reached_stop &&
               recoverable_step.end_time_seconds == kStopTime &&
               (recoverable_endpoint - expected_endpoint)
                       .lpNorm<Eigen::Infinity>() < 1.0e-12 &&
               (recoverable_dense_endpoint - recoverable_endpoint)
                       .lpNorm<Eigen::Infinity>() < 1.0e-14,
           "a recovered Jacobian column produced an inaccurate accepted or "
           "dense endpoint");

    MultiColumnJacobianProbeRhs fatal_rhs(
        MultiColumnJacobianProbeRhs::FailureMode::kFatalColumnTwo);
    Core fatal(fatal_rhs, 0.0, initial_state, 1.0e-8,
               absolute_tolerances);
    fatal.SetSuggestedStepSizeForTesting(kStopTime);
    bool fatal_classified = false;
    try {
        static_cast<void>(fatal.AdvanceOneAcceptedStepToward(kStopTime));
    } catch (const Failure& failure) {
        fatal_classified = failure.reason() == Failure::Reason::kFatalRhs;
    }
    const auto fatal_work = fatal.statistics();
    const std::vector<int> expected_fatal_columns{0, 1, 2};
    Eigen::Vector4d fatal_endpoint;
    fatal.CopyCurrentState(fatal_endpoint);
    Expect(fatal_classified &&
               fatal_rhs.perturbation_columns() == expected_fatal_columns &&
               fatal_work.jacobian_evaluation_count == 1 &&
               fatal_work.right_hand_side_evaluation_count == 1 &&
               fatal_work.linear_solver_right_hand_side_evaluation_count == 3 &&
               fatal_work.successful_internal_step_count == 0 &&
               fatal_rhs.evaluation_count() == 4 &&
               fatal.current_time_seconds() == 0.0 &&
               fatal_endpoint == initial_state &&
               !fatal.dense_output_interval().has_value() &&
               fatal_rhs.maximum_active_evaluation_count() == 1,
           "a fatal middle Jacobian column evaluated later columns or "
           "published a partial endpoint");
}

void VerifyRecoverableRhsPolicies() {
    // Baseline and the single Jacobian column are attempts 1 and 2. Attempt 3
    // is therefore the first Newton-stage RHS and must reduce the trial.
    EvaluationPatternRhs stage_recoverable({3});
    Core stage_core(stage_recoverable, 0.0, Scalar(1.0), 1.0e-7,
                    Scalar(1.0e-9));
    stage_core.SetSuggestedStepSizeForTesting(0.1);
    const auto step = stage_core.AdvanceOneAcceptedStepToward(0.1);
    Eigen::VectorXd state(1);
    stage_core.CopyCurrentState(state);
    const auto stage_work = stage_core.statistics();
    Expect(step.end_time_seconds > 0.0 && step.end_time_seconds < 0.1 &&
               stage_recoverable.attempt_count() > 3,
           "recoverable stage RHS was not retried successfully");
    ExpectNear(state[0], std::exp(-step.end_time_seconds), 2.0e-10,
               "recoverable stage retry returned an inaccurate state");
    Expect(stage_work.successful_internal_step_count == 1 &&
               stage_work.jacobian_evaluation_count == 1 &&
               stage_work.linear_solver_setup_count >= 2 &&
               stage_work.error_test_failure_count == 0 &&
               stage_work.nonlinear_solver_convergence_failure_count == 0,
           "recoverable stage failure was misclassified or discarded "
           "completed setup work");

    // Attempts 2 and 3 are two failures of the one Jacobian column. The core
    // must shrink only the perturbation and then finish that same Jacobian.
    EvaluationPatternRhs jacobian_recoverable({2, 3});
    Core jacobian_core(jacobian_recoverable, 0.0, Scalar(1.0), 1.0e-7,
                       Scalar(1.0e-9));
    jacobian_core.SetSuggestedStepSizeForTesting(0.01);
    static_cast<void>(
        jacobian_core.AdvanceOneAcceptedStepToward(0.01));
    const auto work = jacobian_core.statistics();
    Expect(work.jacobian_evaluation_count == 1 &&
               work.linear_solver_right_hand_side_evaluation_count == 3,
           "Jacobian recoverable failures did not stay in the column retry "
           "budget");

    JacobianPerturbationRhs shrinking_perturbation(2);
    Core shrinking_core(shrinking_perturbation, 0.0, Scalar(1.0), 1.0e-7,
                        Scalar(1.0e-9));
    shrinking_core.SetSuggestedStepSizeForTesting(0.01);
    static_cast<void>(
        shrinking_core.AdvanceOneAcceptedStepToward(0.01));
    const auto& perturbations = shrinking_perturbation.perturbations();
    Expect(perturbations.size() == 3,
           "Jacobian perturbation retry count is wrong");
    ExpectNear(perturbations[1] / perturbations[0], 0.1, 2.0e-7,
               "first Jacobian retry did not shrink delta by ten");
    ExpectNear(perturbations[2] / perturbations[1], 0.1, 2.0e-6,
               "second Jacobian retry did not shrink delta by ten");

    JacobianPerturbationRhs exhausted_perturbations(5);
    Core exhausted_core(exhausted_perturbations, 0.0, Scalar(1.0), 1.0e-7,
                        Scalar(1.0e-9));
    exhausted_core.SetSuggestedStepSizeForTesting(0.01);
    bool exhausted = false;
    try {
        static_cast<void>(
            exhausted_core.AdvanceOneAcceptedStepToward(0.01));
    } catch (const Failure& failure) {
        exhausted = failure.reason() ==
                    Failure::Reason::kRecoverableRhsExhausted;
    }
    const auto exhausted_work = exhausted_core.statistics();
    Expect(exhausted &&
               exhausted_perturbations.perturbations().size() == 5 &&
               exhausted_work.jacobian_evaluation_count == 1 &&
               exhausted_work.linear_solver_right_hand_side_evaluation_count ==
                   5 &&
               exhausted_work.successful_internal_step_count == 0 &&
               exhausted_core.current_time_seconds() == 0.0,
           "Jacobian perturbation exhaustion violated retry or failure "
           "accounting");
}

void VerifyEndpointAndNonFiniteRhsClassification() {
    EvaluationPatternRhs endpoint_recoverable({1});
    Core recoverable(endpoint_recoverable, 0.0, Scalar(1.0), 1.0e-7,
                     Scalar(1.0e-9));
    bool exhausted = false;
    try {
        static_cast<void>(recoverable.AdvanceOneAcceptedStepToward(0.1));
    } catch (const Failure& failure) {
        exhausted = failure.reason() ==
                    Failure::Reason::kRecoverableRhsExhausted;
    }
    const auto recoverable_work = recoverable.statistics();
    Expect(exhausted && recoverable.current_time_seconds() == 0.0 &&
               recoverable_work.right_hand_side_evaluation_count == 1 &&
               recoverable_work.linear_solver_right_hand_side_evaluation_count ==
                   0 &&
               recoverable_work.successful_internal_step_count == 0,
           "an accepted-endpoint recoverable RHS was retried or "
           "misclassified");

    FunctionRhs nonfinite_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = std::numeric_limits<double>::quiet_NaN();
        });
    Core nonfinite(nonfinite_rhs, 0.0, Scalar(1.0), 1.0e-7,
                   Scalar(1.0e-9));
    bool classified_nonfinite = false;
    try {
        static_cast<void>(nonfinite.AdvanceOneAcceptedStepToward(0.1));
    } catch (const Failure& failure) {
        classified_nonfinite =
            failure.reason() == Failure::Reason::kNonFiniteRhs;
    }
    const auto nonfinite_work = nonfinite.statistics();
    Expect(classified_nonfinite && nonfinite.current_time_seconds() == 0.0 &&
               nonfinite_work.right_hand_side_evaluation_count == 1 &&
               nonfinite_work.successful_internal_step_count == 0,
           "a successful callback returning NaN was not classified as a "
           "non-finite RHS failure");
}

void VerifyPositionSpecificFatalRhsClassification() {
    struct FatalCase final {
        int attempt{};
        std::uint64_t ordinary_rhs_count{};
        std::uint64_t jacobian_rhs_count{};
        std::uint64_t setup_count{};
    };
    // In one dimension, calls are baseline, Jacobian perturbation, then the
    // first, second and third stage RHS of the first Newton correction.
    constexpr std::array<FatalCase, 4> kFatalCases{{
        {2, 1, 1, 0},
        {3, 2, 1, 1},
        {4, 3, 1, 1},
        {5, 4, 1, 1},
    }};
    for (const FatalCase& fatal_case : kFatalCases) {
        FatalAttemptRhs rhs(fatal_case.attempt);
        const Eigen::VectorXd initial_state = Scalar(1.0);
        Core core(rhs, 0.0, initial_state, 1.0e-7, Scalar(1.0e-9));
        core.SetSuggestedStepSizeForTesting(0.01);
        bool classified = false;
        try {
            static_cast<void>(core.AdvanceOneAcceptedStepToward(0.01));
        } catch (const Failure& failure) {
            classified = failure.reason() == Failure::Reason::kFatalRhs;
        }
        Eigen::VectorXd observed_state(1);
        core.CopyCurrentState(observed_state);
        const auto work = core.statistics();
        Expect(classified && rhs.attempt_count() == fatal_case.attempt &&
                   work.right_hand_side_evaluation_count ==
                       fatal_case.ordinary_rhs_count &&
                   work.linear_solver_right_hand_side_evaluation_count ==
                       fatal_case.jacobian_rhs_count &&
                   work.linear_solver_setup_count == fatal_case.setup_count &&
                   work.nonlinear_solver_iteration_count == 0 &&
                   work.successful_internal_step_count == 0 &&
                   core.current_time_seconds() == 0.0 &&
                   observed_state == initial_state &&
                   !core.dense_output_interval().has_value(),
               "a Jacobian or position-specific stage fatal RHS violated "
               "classification, work accounting or endpoint atomicity");
    }
}

void VerifyErrorCorrectionRhsFailurePolicies() {
    ErrorCorrectionFailureRhs recoverable_rhs(
        RhsEvaluationStatus::kRecoverableFailure);
    Core recoverable(recoverable_rhs, 0.0, Scalar(1.0), 1.0e-7,
                     Scalar(1.0e-9));
    recoverable.SetSuggestedStepSizeForTesting(0.2);
    const auto recovered = recoverable.AdvanceOneAcceptedStepToward(0.2);
    const auto recovered_work = recoverable.statistics();
    Expect(recoverable_rhs.delivered() &&
               recovered.end_time_seconds == 0.1 &&
               recovered_work.successful_internal_step_count == 1 &&
               recovered_work.right_hand_side_evaluation_count == 11 &&
               recovered_work
                       .linear_solver_right_hand_side_evaluation_count == 1 &&
               recovered_work.error_test_failure_count == 0 &&
               recovered_work.nonlinear_solver_iteration_count == 3 &&
               recovered_work.nonlinear_solver_convergence_failure_count ==
                   0 &&
               recovered_work.linear_solver_setup_count == 2 &&
               recovered_work.jacobian_evaluation_count == 1,
           "a recoverable error-correction RHS did not retry with exact "
           "classification");

    ErrorCorrectionFailureRhs fatal_rhs(RhsEvaluationStatus::kFatalFailure);
    Core fatal(fatal_rhs, 0.0, Scalar(1.0), 1.0e-7, Scalar(1.0e-9));
    fatal.SetSuggestedStepSizeForTesting(0.2);
    bool failed_fatally = false;
    try {
        static_cast<void>(fatal.AdvanceOneAcceptedStepToward(0.2));
    } catch (const Failure& failure) {
        failed_fatally = failure.reason() == Failure::Reason::kFatalRhs;
    }
    const auto fatal_work = fatal.statistics();
    Expect(failed_fatally && fatal_rhs.delivered() &&
               fatal.current_time_seconds() == 0.0 &&
               !fatal.dense_output_interval().has_value() &&
               fatal_work.successful_internal_step_count == 0 &&
               fatal_work.right_hand_side_evaluation_count == 8 &&
               fatal_work.linear_solver_right_hand_side_evaluation_count ==
                   1 &&
               fatal_work.error_test_failure_count == 0 &&
               fatal_work.nonlinear_solver_iteration_count == 2 &&
               fatal_work.nonlinear_solver_convergence_failure_count == 0 &&
               fatal_work.linear_solver_setup_count == 1 &&
               fatal_work.jacobian_evaluation_count == 1,
           "a fatal error-correction RHS did not stop with exact "
           "classification");
}

void VerifyJacobianFreshAndStaleRetryPolicy() {
    // A first-trial stage failure occurs while the just-built Jacobian is
    // fresh. Retrying at h/2 must retain that Jacobian and only redo setup.
    EvaluationPatternRhs fresh_failure({3});
    Core fresh_core(fresh_failure, 0.0, Scalar(1.0), 1.0e-7,
                    Scalar(1.0e-9));
    fresh_core.SetSuggestedStepSizeForTesting(0.02);
    const auto fresh_step = fresh_core.AdvanceOneAcceptedStepToward(0.02);
    const auto fresh_work = fresh_core.statistics();
    Expect(fresh_step.end_time_seconds < 0.02 &&
               fresh_work.jacobian_evaluation_count == 1 &&
               fresh_work.linear_solver_setup_count >= 2,
           "fresh-Jacobian retry recomputed the Jacobian or skipped setup");

    // After acceptance the same Jacobian is stale. A one-shot stage failure
    // on the next call must invalidate it and recompute before retrying.
    ArmedStageFailureRhs stale_failure;
    Core stale_core(stale_failure, 0.0, Scalar(1.0), 1.0e-7,
                    Scalar(1.0e-9));
    stale_core.SetSuggestedStepSizeForTesting(0.01);
    static_cast<void>(stale_core.AdvanceOneAcceptedStepToward(0.01));
    const auto before_control_reuse = stale_core.statistics();
    stale_core.SetSuggestedStepSizeForTesting(0.01);
    const auto control_step =
        stale_core.AdvanceOneAcceptedStepToward(0.02);
    const auto after_control_reuse = stale_core.statistics();
    Expect(control_step.reached_stop &&
               after_control_reuse.jacobian_evaluation_count ==
                   before_control_reuse.jacobian_evaluation_count,
           "an accepted linear step did not expose a reusable stale "
           "Jacobian before the failure-path check");

    const auto before = after_control_reuse;
    stale_failure.ArmAtAcceptedEndpoint(stale_core.current_time_seconds());
    stale_core.SetSuggestedStepSizeForTesting(0.01);
    const auto stale_step =
        stale_core.AdvanceOneAcceptedStepToward(0.03);
    const auto after = stale_core.statistics();
    Eigen::VectorXd state(1);
    stale_core.CopyCurrentState(state);
    Expect(stale_failure.failure_delivered() &&
               stale_step.end_time_seconds < 0.03 &&
               after.successful_internal_step_count ==
                   before.successful_internal_step_count + 1 &&
               after.jacobian_evaluation_count ==
                   before.jacobian_evaluation_count + 1 &&
               after.linear_solver_setup_count >=
                   before.linear_solver_setup_count + 2 &&
               after.error_test_failure_count ==
                   before.error_test_failure_count &&
               after.nonlinear_solver_convergence_failure_count ==
                   before.nonlinear_solver_convergence_failure_count,
           "stale-Jacobian recoverable retry did not recompute exactly when "
           "required");
    ExpectNear(state[0], std::exp(-stale_step.end_time_seconds), 2.0e-10,
               "stale-Jacobian retry returned an inaccurate endpoint");
}

void VerifyInclusiveTrialBudget() {
    EvaluationPatternRhs succeeds_on_last_allowed_trial({3});
    Core permitted(succeeds_on_last_allowed_trial, 0.0, Scalar(1.0),
                   1.0e-7, Scalar(1.0e-9));
    permitted.SetMaximumTrialsPerCallForTesting(2);
    permitted.SetSuggestedStepSizeForTesting(0.1);
    const auto accepted = permitted.AdvanceOneAcceptedStepToward(0.1);
    Expect(accepted.end_time_seconds > 0.0 &&
               accepted.end_time_seconds < 0.1 &&
               permitted.statistics().successful_internal_step_count == 1,
           "the last allowed trial was not permitted to succeed");

    EvaluationPatternRhs needs_third_trial({3, 4});
    Core refused(needs_third_trial, 0.0, Scalar(1.0), 1.0e-7,
                 Scalar(1.0e-9));
    refused.SetMaximumTrialsPerCallForTesting(2);
    refused.SetSuggestedStepSizeForTesting(0.1);
    bool recoverable_exhausted = false;
    try {
        static_cast<void>(refused.AdvanceOneAcceptedStepToward(0.1));
    } catch (const Failure& failure) {
        recoverable_exhausted =
            failure.reason() == Failure::Reason::kRecoverableRhsExhausted;
    }
    Expect(recoverable_exhausted && needs_third_trial.attempt_count() == 4 &&
               refused.current_time_seconds() == 0.0 &&
               refused.statistics().successful_internal_step_count == 0,
           "trial budget did not allow N recoverable trials and preserve "
           "their causal exhaustion at trial N+1");

    FunctionRhs error_rejection_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = -state[0];
        });
    Core ordinary_budget(error_rejection_rhs, 0.0, Scalar(1.0), 1.0e-7,
                         Scalar(1.0e-9));
    ordinary_budget.SetMaximumTrialsPerCallForTesting(1);
    ordinary_budget.SetSuggestedStepSizeForTesting(0.2);
    bool ordinary_budget_exhausted = false;
    try {
        static_cast<void>(
            ordinary_budget.AdvanceOneAcceptedStepToward(0.2));
    } catch (const Failure& failure) {
        ordinary_budget_exhausted =
            failure.reason() == Failure::Reason::kTrialBudgetExceeded;
    }
    Expect(ordinary_budget_exhausted &&
               ordinary_budget.statistics().error_test_failure_count == 1 &&
               ordinary_budget.statistics().successful_internal_step_count ==
                   0,
           "a non-RHS retry did not retain the ordinary trial-budget "
           "classification");
}

void VerifySingularSetupAndTinyStepPolicies() {
    constexpr double kRealRadauEigenvalue = 3.6378342527444957322;
    FunctionRhs singular_once_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = state[0];
        });
    Core singular_once(singular_once_rhs, 0.0, Scalar(0.0), 1.0e-7,
                       Scalar(1.0e-9));
    for (int index = 1; index <= 4; ++index) {
        singular_once.SetSuggestedStepSizeForTesting(kRealRadauEigenvalue);
        const double start = singular_once.current_time_seconds();
        // Keep the requested stop well beyond the deliberately singular
        // trial so the independent first-trial endpoint-snap contract cannot
        // perturb the exact shifted matrix being exercised here.
        const double stop = start + 2.0 * kRealRadauEigenvalue;
        const auto recovered =
            singular_once.AdvanceOneAcceptedStepToward(stop);
        const auto work = singular_once.statistics();
        Expect(recovered.end_time_seconds > start &&
                   recovered.end_time_seconds < stop &&
                   work.linear_solver_setup_count ==
                       static_cast<std::uint64_t>(2 * index) &&
                   work.nonlinear_solver_convergence_failure_count ==
                       static_cast<std::uint64_t>(index) &&
                   work.successful_internal_step_count ==
                       static_cast<std::uint64_t>(index),
               "a singular real factorization was not counted, reduced, "
               "and recovered");
    }
    const double endpoint_before_exhaustion =
        singular_once.current_time_seconds();
    Expect(singular_once.dense_output_interval().has_value(),
           "successful singular retries did not retain dense output");
    singular_once.SetSuggestedStepSizeForTesting(kRealRadauEigenvalue);
    bool repeatedly_singular = false;
    try {
        const double stop = endpoint_before_exhaustion +
                            2.0 * kRealRadauEigenvalue;
        static_cast<void>(singular_once.AdvanceOneAcceptedStepToward(stop));
    } catch (const Failure& failure) {
        repeatedly_singular =
            failure.reason() == Failure::Reason::kRepeatedlySingular;
    }
    const auto singular_work = singular_once.statistics();
    Expect(repeatedly_singular &&
               singular_once.current_time_seconds() ==
                   endpoint_before_exhaustion &&
               !singular_once.dense_output_interval().has_value() &&
               singular_work.linear_solver_setup_count == 9 &&
               singular_work.nonlinear_solver_convergence_failure_count ==
                   5 &&
               singular_work.successful_internal_step_count == 4,
           "the fifth singular setup did not fail atomically with exact "
           "statistics");

    FunctionRhs constant_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 1.0;
        });
    Core tiny_step(constant_rhs, 1.0, Scalar(0.0), 1.0e-7,
                   Scalar(1.0e-9));
    const double smallest_positive =
        std::nextafter(1.0, std::numeric_limits<double>::infinity());
    bool refused_as_too_small = false;
    try {
        static_cast<void>(
            tiny_step.AdvanceOneAcceptedStepToward(smallest_positive));
    } catch (const Failure& failure) {
        refused_as_too_small =
            failure.reason() == Failure::Reason::kStepTooSmall;
    }
    Expect(refused_as_too_small && tiny_step.current_time_seconds() == 1.0 &&
               SameStatistics(tiny_step.statistics(), Statistics{}),
           "an unrepresentably small positive stop was not refused "
           "atomically");
}

void VerifyFirstTrialStopSnapAndRetryConsumption() {
    constexpr double kInitialTime = 100000.0;
    constexpr double kStopTime = 100000.00000100005;
    FunctionRhs zero_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 0.0;
        });
    Core constructed(zero_rhs, kInitialTime, Scalar(3.0), 1.0e-7,
                     Scalar(1.0e-9));
    const auto constructed_step =
        constructed.AdvanceOneAcceptedStepToward(kStopTime);
    Expect(constructed_step.reached_stop &&
               constructed_step.end_time_seconds == kStopTime &&
               constructed.current_time_seconds() == kStopTime,
           "the first constructed trial did not snap to the exact nearby "
           "stop");

    Core reinitialized(zero_rhs, 0.0, Scalar(0.0), 1.0e-7,
                       Scalar(1.0e-9));
    reinitialized.Reinitialize(kInitialTime, Scalar(4.0));
    const auto reinitialized_step =
        reinitialized.AdvanceOneAcceptedStepToward(kStopTime);
    Expect(reinitialized_step.reached_stop &&
               reinitialized_step.end_time_seconds == kStopTime &&
               reinitialized.current_time_seconds() == kStopTime,
           "the first trial after reinitialization did not snap to the "
           "exact nearby stop");

    Core later_call(zero_rhs, 0.0, Scalar(2.0), 1.0e-7,
                    Scalar(1.0e-9));
    later_call.SetSuggestedStepSizeForTesting(0.01);
    const auto earlier_step =
        later_call.AdvanceOneAcceptedStepToward(0.01);
    Expect(earlier_step.reached_stop,
           "the setup step for a later stop-snap call did not finish");
    later_call.SetSuggestedStepSizeForTesting(
        std::nextafter(0.01, 0.0));
    const auto later_step =
        later_call.AdvanceOneAcceptedStepToward(0.02);
    Expect(later_step.reached_stop &&
               later_step.end_time_seconds == 0.02 &&
               later_call.current_time_seconds() == 0.02,
           "a later accepted-step call left an unrepresentable floating-"
           "point tail before its exact stop");

    OneRecoverableTrialEndRhs retry_rhs;
    Core rejected_snap(retry_rhs, kInitialTime, Scalar(0.0), 1.0e-7,
                       Scalar(1.0e-9));
    const auto recovered =
        rejected_snap.AdvanceOneAcceptedStepToward(kStopTime);
    Expect(retry_rhs.failed_trial_end_time_seconds() == kStopTime,
           "the eligible first trial was not snapped before its recoverable "
           "stage failure");
    Expect(!recovered.reached_stop && recovered.end_time_seconds < kStopTime,
           "a rejected snapped trial was snapped again instead of retaining "
           "the controller reduction");
}

void VerifyPersistentRecoverableCause() {
    PersistentlyRecoverableStageRhs recoverable_rhs;
    Core recoverable(recoverable_rhs, 0.0, Scalar(1.0), 1.0e-7,
                     Scalar(1.0e-9));
    recoverable.SetMaximumTrialsPerCallForTesting(2);
    bool retained_recoverable_cause = false;
    try {
        static_cast<void>(recoverable.AdvanceOneAcceptedStepToward(0.1));
    } catch (const Failure& failure) {
        retained_recoverable_cause =
            failure.reason() == Failure::Reason::kRecoverableRhsExhausted;
    }
    Expect(retained_recoverable_cause,
           "persistent recoverable stage failures lost their causal "
           "classification at the trial budget");

    ErrorCorrectionFailureRhs error_correction_rhs(
        RhsEvaluationStatus::kRecoverableFailure);
    Core error_correction(error_correction_rhs, 0.0, Scalar(1.0), 1.0e-7,
                          Scalar(1.0e-9));
    error_correction.SetMaximumTrialsPerCallForTesting(1);
    error_correction.SetSuggestedStepSizeForTesting(0.2);
    bool retained_error_correction_cause = false;
    try {
        static_cast<void>(
            error_correction.AdvanceOneAcceptedStepToward(0.2));
    } catch (const Failure& failure) {
        retained_error_correction_cause =
            failure.reason() == Failure::Reason::kRecoverableRhsExhausted;
    }
    Expect(retained_error_correction_cause &&
               error_correction_rhs.delivered(),
           "recoverable error-correction failure lost its causal "
           "classification at the trial budget");

}

void VerifyLinearSetupClassification() {
    FunctionRhs zero_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 0.0;
        });
    Core nonrepresentable_shift(zero_rhs, 0.0, Scalar(0.0), 1.0e-7,
                                Scalar(1.0e-9));
    bool classified_as_step_too_small = false;
    try {
        static_cast<void>(
            nonrepresentable_shift.AdvanceOneAcceptedStepToward(1.0e-309));
    } catch (const Failure& failure) {
        classified_as_step_too_small =
            failure.reason() == Failure::Reason::kStepTooSmall;
    }
    const auto setup_work = nonrepresentable_shift.statistics();
    Expect(classified_as_step_too_small &&
               setup_work.linear_solver_setup_count == 1 &&
               setup_work.nonlinear_solver_convergence_failure_count == 0,
           "a non-finite shifted-system scale was counted as repeated "
           "mathematical singularity");

    constexpr double kRealRadauEigenvalue = 3.6378342527444957322;
    FunctionRhs finite_matrix_overflow_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = -1.0e308 * state[0];
        });
    Core finite_matrix_overflow(finite_matrix_overflow_rhs, 0.0,
                                Scalar(0.0), 1.0e-7,
                                Scalar(1.0e-9));
    bool classified_as_nonfinite_matrix = false;
    try {
        static_cast<void>(
            finite_matrix_overflow.AdvanceOneAcceptedStepToward(
                kRealRadauEigenvalue / 1.0e308));
    } catch (const Failure& failure) {
        classified_as_nonfinite_matrix =
            failure.reason() == Failure::Reason::kNonFiniteLinearSystem;
    }
    const auto matrix_work = finite_matrix_overflow.statistics();
    Expect(classified_as_nonfinite_matrix &&
               matrix_work.linear_solver_setup_count == 1 &&
               matrix_work.nonlinear_solver_convergence_failure_count == 0,
           "finite shifted-system operands that overflowed during matrix "
           "formation were mislabeled as singular");

    const double large =
        0.75 * std::numeric_limits<double>::max();
    FunctionRhs finite_factorization_overflow_rhs(
        2, [large](double,
                   const Eigen::Ref<const Eigen::VectorXd>& state,
                   Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = -large * state[0] - large * state[1];
            derivatives[1] = large * state[0] - large * state[1];
        });
    Core finite_factorization_overflow(
        finite_factorization_overflow_rhs, 0.0,
        Eigen::Vector2d::Zero(), 1.0e-7,
        Eigen::Vector2d::Constant(1.0e-9));
    finite_factorization_overflow.SetSuggestedStepSizeForTesting(1.0);
    bool classified_as_nonfinite_factorization = false;
    try {
        static_cast<void>(
            finite_factorization_overflow.AdvanceOneAcceptedStepToward(1.0));
    } catch (const Failure& failure) {
        classified_as_nonfinite_factorization =
            failure.reason() == Failure::Reason::kNonFiniteLinearSystem;
    }
    const auto factorization_work =
        finite_factorization_overflow.statistics();
    Expect(classified_as_nonfinite_factorization &&
               factorization_work.linear_solver_setup_count == 1 &&
               factorization_work
                       .nonlinear_solver_convergence_failure_count == 0,
           "a finite shifted matrix whose LU overflowed was mislabeled as "
           "singular");
}

void VerifyConfirmedDefectRegressions() {
    int failed_regressions = 0;
    const auto run = [&](const char* name, const auto& verification) {
        try {
            verification();
        } catch (const std::exception& error) {
            std::cerr << "confirmed defect still present [" << name
                      << "]: " << error.what() << '\n';
            ++failed_regressions;
        }
    };
    run("first-trial stop snap",
        VerifyFirstTrialStopSnapAndRetryConsumption);
    run("recoverable cause", VerifyPersistentRecoverableCause);
    run("linear setup classification", VerifyLinearSetupClassification);
    Expect(failed_regressions == 0,
           "one or more independently reproduced Radau5 defects remain");
}

void VerifyRepresentableSignedJacobianPerturbations() {
    FunctionRhs decay_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = -state[0];
        });
    Core admitted(decay_rhs, 0.0, Scalar(1.0e15), 1.0e-7,
                  Scalar(1.0e-9));
    static_cast<void>(admitted.AdvanceOneAcceptedStepToward(1.0e-6));

    for (double initial_value :
         {1.0e16, -1.0e16, std::numeric_limits<double>::max()}) {
        const int recoverable_attempt_count =
            initial_value == std::numeric_limits<double>::max() ? 2 : 0;
        LargeStateJacobianPerturbationRhs large_rhs(
            initial_value, recoverable_attempt_count);
        Core extended(large_rhs, 0.0, Scalar(initial_value), 1.0e-7,
                      Scalar(1.0e-9));
        const auto step =
            extended.AdvanceOneAcceptedStepToward(1.0e-6);
        Eigen::VectorXd endpoint(1);
        extended.CopyCurrentState(endpoint);
        const auto& perturbations = large_rhs.perturbations();
        Expect(step.end_time_seconds > 0.0 && endpoint.allFinite() &&
                   perturbations.size() ==
                       static_cast<std::size_t>(
                           recoverable_attempt_count + 1) &&
                   std::all_of(perturbations.begin(), perturbations.end(),
                               [](double increment) {
                                   return std::isfinite(increment) &&
                                          increment != 0.0;
                               }),
               "a finite large state lacked a signed representable "
               "Jacobian perturbation");
        if (initial_value == std::numeric_limits<double>::max()) {
            Expect(std::all_of(perturbations.begin(), perturbations.end(),
                               [](double increment) {
                                   return increment < 0.0;
                               }),
                   "DBL_MAX did not use the finite negative nextafter "
                   "fallback on every recoverable retry");
        }
    }
}

void VerifyScaledNormsDoNotOverflowBeforeTheirFiniteResults() {
    FunctionRhs huge_stage_rhs(
        1, [](double time_seconds,
              const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = time_seconds > 0.0 ? 1.0e200 : 0.0;
        });
    Core newton_norm(huge_stage_rhs, 0.0, Scalar(0.0), 1.0e-7,
                     Scalar(1.0e-9));
    newton_norm.SetMaximumTrialsPerCallForTesting(1);
    newton_norm.SetSuggestedStepSizeForTesting(1.0e-6);
    bool newton_reached_an_ordinary_retry = false;
    try {
        static_cast<void>(
            newton_norm.AdvanceOneAcceptedStepToward(1.0e-6));
    } catch (const Failure& failure) {
        newton_reached_an_ordinary_retry =
            failure.reason() == Failure::Reason::kTrialBudgetExceeded;
    }
    const auto newton_work = newton_norm.statistics();
    Expect(newton_reached_an_ordinary_retry &&
               newton_work.nonlinear_solver_iteration_count > 0 &&
               newton_work.successful_internal_step_count == 0,
           "a finite scaled Newton-correction norm was poisoned only because "
           "its components would overflow when squared directly");

    FunctionRhs huge_endpoint_only_rhs(
        1, [](double time_seconds,
              const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = time_seconds == 0.0 ? 1.0e200 : 0.0;
        });
    Core error_norm(huge_endpoint_only_rhs, 0.0, Scalar(0.0), 1.0e-7,
                    Scalar(1.0e-9));
    error_norm.SetMaximumTrialsPerCallForTesting(1);
    error_norm.SetSuggestedStepSizeForTesting(1.0e-6);
    bool error_reached_an_ordinary_rejection = false;
    try {
        static_cast<void>(error_norm.AdvanceOneAcceptedStepToward(1.0e-6));
    } catch (const Failure& failure) {
        error_reached_an_ordinary_rejection =
            failure.reason() == Failure::Reason::kTrialBudgetExceeded;
    }
    const auto error_work = error_norm.statistics();
    Expect(error_reached_an_ordinary_rejection &&
               error_work.error_test_failure_count == 1 &&
               error_work.nonlinear_solver_convergence_failure_count == 0 &&
               error_work.successful_internal_step_count == 0,
           "a finite scaled error WRMS norm was poisoned only because its "
           "components would overflow when squared directly");
}

void VerifyLaterSingularFailureOverridesRecoverableCause() {
    constexpr double kRealRadauEigenvalue = 3.6378342527444957322;
    ArmedPositiveLinearStageFailureRhs rhs;
    Core core(rhs, 0.0, Scalar(0.0), 1.0e-7, Scalar(1.0e-9));
    for (int index = 0; index < 4; ++index) {
        core.SetSuggestedStepSizeForTesting(kRealRadauEigenvalue);
        const double stop = core.current_time_seconds() +
                            2.0 * kRealRadauEigenvalue;
        static_cast<void>(core.AdvanceOneAcceptedStepToward(stop));
    }

    rhs.ArmAtAcceptedEndpoint(core.current_time_seconds());
    core.SetSuggestedStepSizeForTesting(2.0 * kRealRadauEigenvalue);
    bool singular_overrode_stale_recoverable = false;
    try {
        static_cast<void>(core.AdvanceOneAcceptedStepToward(
            core.current_time_seconds() +
            3.0 * kRealRadauEigenvalue));
    } catch (const Failure& failure) {
        singular_overrode_stale_recoverable =
            failure.reason() == Failure::Reason::kRepeatedlySingular;
    }
    Expect(rhs.failure_delivered() &&
               singular_overrode_stale_recoverable,
           "a later genuine LU singularity was replaced by an earlier "
           "recoverable RHS exception");
}

void VerifyNumericalRhsHistoryInvalidatesOnlyLinearization() {
    MutableLinearRhs rhs;
    Core core(rhs, 0.0, Eigen::VectorXd::Ones(1), 1.0e-9,
              Eigen::VectorXd::Constant(1, 1.0e-12));
    core.SetSuggestedStepSizeForTesting(0.01);
    const auto first = core.AdvanceOneAcceptedStepToward(0.01);
    Expect(first.reached_stop,
           "linearization invalidation setup did not reach first stop");

    const Statistics before = core.statistics();
    const auto dense_before = core.dense_output_interval();
    Eigen::VectorXd state_before(1);
    Eigen::VectorXd dense_state_before(1);
    core.CopyCurrentState(state_before);
    core.CopyDenseState(0.005, dense_state_before);

    rhs.set_slope(-2.0);
    core.InvalidateLinearizationAfterNumericalRhsHistoryChange();

    Expect(SameStatistics(core.statistics(), before),
           "linearization invalidation changed published statistics");
    const auto dense_after = core.dense_output_interval();
    Expect(dense_before.has_value() && dense_after.has_value() &&
               dense_after->start_time_seconds ==
                   dense_before->start_time_seconds &&
               dense_after->end_time_seconds ==
                   dense_before->end_time_seconds,
           "linearization invalidation discarded accepted dense output");
    Eigen::VectorXd state_after(1);
    Eigen::VectorXd dense_state_after(1);
    core.CopyCurrentState(state_after);
    core.CopyDenseState(0.005, dense_state_after);
    Expect(state_after[0] == state_before[0] &&
               dense_state_after[0] == dense_state_before[0],
           "linearization invalidation changed accepted numerical state");

    core.SetSuggestedStepSizeForTesting(0.01);
    const auto second = core.AdvanceOneAcceptedStepToward(0.02);
    Expect(second.reached_stop,
           "linearization invalidation setup did not reach second stop");
    const Statistics after = core.statistics();
    Expect(after.jacobian_evaluation_count ==
               before.jacobian_evaluation_count + 1,
           "changed numerical RHS history reused the old Jacobian");
    Expect(after.linear_solver_setup_count ==
               before.linear_solver_setup_count + 1,
           "changed numerical RHS history reused the old factorization");
    Eigen::VectorXd endpoint(1);
    core.CopyCurrentState(endpoint);
    ExpectNear(endpoint[0], std::exp(-0.01) * std::exp(-0.02), 2.0e-11,
               "changed numerical RHS history produced wrong endpoint");
}

void VerifyFailedNewtonSolveIsCounted() {
    FunctionRhs nonlinear_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>& state,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 100.0 * state[0] * state[0];
        });
    Core nonlinear(nonlinear_rhs, 0.0, Scalar(0.1), 1.0e-7,
                   Scalar(1.0e-9));
    nonlinear.SetSuggestedStepSizeForTesting(0.2);
    const auto recovered = nonlinear.AdvanceOneAcceptedStepToward(0.2);
    Eigen::VectorXd recovered_state(1);
    nonlinear.CopyCurrentState(recovered_state);
    const double expected = 0.1 / (1.0 - 10.0 * recovered.end_time_seconds);
    const auto recovered_work = nonlinear.statistics();
    Expect(recovered.end_time_seconds > 0.0 &&
               recovered.end_time_seconds < 0.1 &&
               recovered_work.nonlinear_solver_convergence_failure_count >
                   0 &&
               recovered_work.error_test_failure_count > 0 &&
               recovered_work.successful_internal_step_count == 1,
           "a non-singular Newton failure did not reduce and recover");
    ExpectNear(recovered_state[0], expected, 2.0e-8,
               "the recovered nonlinear endpoint is inaccurate");

    FunctionRhs huge_constant_rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 100.0;
        });
    Core core(huge_constant_rhs, 0.0, Scalar(0.0), 1.0e-7,
              Scalar(1.0e-9));
    constexpr double kHugeStep = 1.0e307;
    core.SetSuggestedStepSizeForTesting(kHugeStep);
    bool nonfinite_solve = false;
    try {
        static_cast<void>(core.AdvanceOneAcceptedStepToward(kHugeStep));
    } catch (const Failure& failure) {
        nonfinite_solve =
            failure.reason() == Failure::Reason::kNonFiniteInternalState;
    }
    const auto work = core.statistics();
    Expect(nonfinite_solve && work.linear_solver_setup_count == 1 &&
               work.nonlinear_solver_iteration_count == 1 &&
               work.successful_internal_step_count == 0 &&
               core.current_time_seconds() == 0.0,
           "a non-finite Newton solve did not retain its started-iteration "
           "work count");
}

void VerifyNearSingularRealAndComplexShiftedSolves() {
    constexpr double kRealEigenvalue = 3.6378342527444957322;
    constexpr double kComplexEigenvalueReal = 2.6810828736277521339;
    constexpr double kComplexEigenvalueImaginary = 3.0504301992474105694;
    constexpr std::array<double, 3> kRequestedGaps{1.0e-4, 1.0e-8,
                                                   1.0e-12};
    constexpr double kStepSizeSeconds = 1.0;
    constexpr double kBackwardErrorMultiplier = 512.0;
    constexpr double kForwardErrorMultiplier = 64.0;
    const double machine_epsilon = std::numeric_limits<double>::epsilon();

    for (double requested_gap : kRequestedGaps) {
        Eigen::Matrix2d real_jacobian;
        real_jacobian << kRealEigenvalue - 1.0, -1.0, -1.0,
            kRealEigenvalue - (1.0 + requested_gap);
        const double shifted_one =
            kRealEigenvalue - (kRealEigenvalue - 1.0);
        const double shifted_one_plus_gap =
            kRealEigenvalue -
            (kRealEigenvalue - (1.0 + requested_gap));
        const double actual_gap =
            shifted_one_plus_gap - shifted_one;
        Expect(actual_gap > 0.0 && std::isfinite(actual_gap),
               "real near-singular gap is not representable");

        Eigen::Vector2d real_right_hand_side;
        real_right_hand_side << shifted_one - 1.0,
            1.0 - shifted_one_plus_gap;
        const Eigen::Vector2cd unused_complex_right_hand_side =
            Eigen::Vector2cd::Zero();
        const auto result = SolveShiftedLinearSystemsForQualification(
            real_jacobian, kStepSizeSeconds, real_right_hand_side,
            unused_complex_right_hand_side);
        const Eigen::Vector2d exact_solution(1.0, -1.0);
        const double backward_error = RelativeBackwardError(
            result.real_matrix, result.real_solution,
            real_right_hand_side);
        const double matrix_norm = 2.0 + actual_gap;
        const double inverse_norm =
            (2.0 + actual_gap) / actual_gap;
        const double condition_number = matrix_norm * inverse_norm;
        const double forward_error = VectorInfinityNorm(
            result.real_solution - exact_solution);
        Expect(result.real_solution.allFinite(),
               "real near-singular solve returned a non-finite result");
        Expect(condition_number * machine_epsilon < 0.01,
               "real near-singular fixture has no forward-error resolution");
        Expect(backward_error <=
                   kBackwardErrorMultiplier * machine_epsilon,
               "real near-singular solve has excessive backward error");
        Expect(forward_error <=
                   kForwardErrorMultiplier * condition_number *
                       machine_epsilon,
               "real near-singular solve disagrees with its analytic oracle");

        const double actual_complex_gap =
            kComplexEigenvalueReal -
            (kComplexEigenvalueReal - requested_gap);
        Expect(actual_complex_gap > 0.0 &&
                   std::isfinite(actual_complex_gap),
               "complex near-singular gap is not representable");
        Eigen::Matrix2d complex_jacobian;
        complex_jacobian << kComplexEigenvalueReal,
            kComplexEigenvalueImaginary,
            -kComplexEigenvalueImaginary,
            kComplexEigenvalueReal - requested_gap;
        const Eigen::Vector2d unused_real_right_hand_side =
            Eigen::Vector2d::Zero();
        Eigen::Vector2cd complex_right_hand_side;
        complex_right_hand_side << std::complex<double>(0.0, 0.0),
            std::complex<double>(0.0, actual_complex_gap);
        const auto complex_result =
            SolveShiftedLinearSystemsForQualification(
                complex_jacobian, kStepSizeSeconds,
                unused_real_right_hand_side, complex_right_hand_side);
        Eigen::Vector2cd exact_complex_solution;
        exact_complex_solution << std::complex<double>(1.0, 0.0),
            std::complex<double>(0.0, 1.0);
        const double complex_backward_error = RelativeBackwardError(
            complex_result.complex_matrix,
            complex_result.complex_solution, complex_right_hand_side);
        const double beta = kComplexEigenvalueImaginary;
        const double complex_matrix_norm = std::max(
            2.0 * beta,
            beta + std::hypot(beta, actual_complex_gap));
        const double complex_inverse_norm = std::max(
            (std::hypot(beta, actual_complex_gap) + beta) /
                (beta * actual_complex_gap),
            2.0 / actual_complex_gap);
        const double complex_condition_number =
            complex_matrix_norm * complex_inverse_norm;
        const double complex_forward_error = VectorInfinityNorm(
            complex_result.complex_solution - exact_complex_solution);
        bool complex_solution_is_finite = true;
        for (Eigen::Index index = 0;
             index < complex_result.complex_solution.size(); ++index) {
            complex_solution_is_finite =
                complex_solution_is_finite &&
                std::isfinite(
                    complex_result.complex_solution[index].real()) &&
                std::isfinite(
                    complex_result.complex_solution[index].imag());
        }
        Expect(complex_solution_is_finite,
               "complex near-singular solve returned a non-finite result");
        Expect(complex_condition_number * machine_epsilon < 0.01,
               "complex near-singular fixture has no forward-error resolution");
        Expect(complex_backward_error <=
                   kBackwardErrorMultiplier * machine_epsilon,
               "complex near-singular solve has excessive backward error");
        Expect(complex_forward_error <=
                   kForwardErrorMultiplier * complex_condition_number *
                       machine_epsilon,
               "complex near-singular solve disagrees with its analytic "
               "oracle");
    }
}

void VerifyFailurePoisonAndReinitialize() {
    FunctionRhs rhs(
        1, [](double, const Eigen::Ref<const Eigen::VectorXd>&,
              Eigen::Ref<Eigen::VectorXd> derivatives) {
            derivatives[0] = 1.0;
        });
    Core core = MakeScalarCore(rhs, 0.0);
    rhs.set_forced_status(RhsEvaluationStatus::kFatalFailure);
    bool fatal = false;
    try {
        static_cast<void>(core.AdvanceOneAcceptedStepToward(0.1));
    } catch (const Failure& failure) {
        fatal = failure.reason() == Failure::Reason::kFatalRhs;
    }
    Expect(fatal, "fatal RHS did not preserve its structured failure reason");
    Expect(core.current_time_seconds() == 0.0,
           "failed trial changed the public core endpoint");
    Expect(!core.dense_output_interval().has_value(),
           "failed trial retained dense output");

    bool poisoned = false;
    try {
        static_cast<void>(core.AdvanceOneAcceptedStepToward(0.0));
    } catch (const std::logic_error&) {
        poisoned = true;
    }
    Expect(poisoned, "failed core did not block a same-time request");

    rhs.set_forced_status(RhsEvaluationStatus::kSuccess);
    core.Reinitialize(0.5, Scalar(2.0));
    Expect(core.current_time_seconds() == 0.5,
           "core reinitialization did not publish the new time");
    const auto statistics = core.statistics();
    Expect(SameStatistics(statistics, Statistics{}),
           "successful reinitialization did not clear statistics");
    core.SetSuggestedStepSizeForTesting(0.1);
    static_cast<void>(core.AdvanceOneAcceptedStepToward(0.6));
}

}  // namespace

int main() {
    try {
        const auto run = [](const char* name, const auto& verification) {
            try {
                verification();
            } catch (const std::exception& exception) {
                throw std::runtime_error(std::string(name) + ": " +
                                         exception.what());
            }
        };
        run("construction/no-op", VerifyConstructionAndNoOp);
        run("elementary/dense", VerifyElementaryProblemsAndDenseOutput);
        run("fifth-order trend", VerifyFifthOrderTrend);
        run("adaptive tolerance/dense refinement",
            VerifyAdaptiveToleranceAndDenseOutputRefinement);
        run("dense interior order", VerifyDenseOutputInteriorOrder);
        run("oscillator/isolation", VerifyOscillatorAndInstanceIsolation);
        run("stiff/adaptive", VerifyStiffProblemsAndAdaptiveRejection);
        run("Robertson/Van der Pol", VerifyRobertsonAndStiffVanDerPol);
        run("accepted-only predictive controller",
            VerifyPredictiveControllerIsAcceptedOnly);
        run("sixth Newton prediction boundary",
            VerifySixthNewtonPredictionBoundary);
        run("multi-column serial Jacobian baseline",
            VerifyMultiColumnSerialJacobianBaseline);
        run("recoverable RHS", VerifyRecoverableRhsPolicies);
        run("endpoint/nonfinite RHS",
            VerifyEndpointAndNonFiniteRhsClassification);
        run("position-specific fatal RHS",
            VerifyPositionSpecificFatalRhsClassification);
        run("error-correction RHS",
            VerifyErrorCorrectionRhsFailurePolicies);
        run("Jacobian freshness", VerifyJacobianFreshAndStaleRetryPolicy);
        run("trial budget", VerifyInclusiveTrialBudget);
        run("singular/tiny step", VerifySingularSetupAndTinyStepPolicies);
        run("confirmed defect regressions", VerifyConfirmedDefectRegressions);
        run("signed Jacobian perturbations",
            VerifyRepresentableSignedJacobianPerturbations);
        run("scaled Newton/error norms",
            VerifyScaledNormsDoNotOverflowBeforeTheirFiniteResults);
        run("recoverable then singular precedence",
            VerifyLaterSingularFailureOverridesRecoverableCause);
        run("numerical RHS history invalidation",
            VerifyNumericalRhsHistoryInvalidatesOnlyLinearization);
        run("failed Newton accounting", VerifyFailedNewtonSolveIsCounted);
        run("near-singular real/complex shifted solves",
            VerifyNearSingularRealAndComplexShiftedSolves);
        run("failure/reinitialize", VerifyFailurePoisonAndReinitialize);
        std::cout << "Radau5 core verification passed\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Radau5 core verification failed: " << exception.what()
                  << '\n';
        return 1;
    }
}
