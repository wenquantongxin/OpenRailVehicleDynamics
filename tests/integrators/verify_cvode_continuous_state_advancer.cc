#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>

#include "orvd/integrators/cvode_continuous_state_advancer.h"

namespace {

using orvd::integrators::ContinuousStateErrorTolerances;
using orvd::integrators::ContinuousStateRhs;
using orvd::integrators::CvodeContinuousStateAdvancer;

int failure_count = 0;

void Expect(bool condition, const char* description) {
    if (!condition) {
        std::printf("FAIL %s\n", description);
        ++failure_count;
    }
}

void ExpectNear(double actual, double expected, double tolerance,
                const char* description) {
    if (std::abs(actual - expected) > tolerance) {
        std::printf("FAIL %s: actual=%.17g expected=%.17g tolerance=%.3g\n",
                    description, actual, expected, tolerance);
        ++failure_count;
    }
}

template <typename Function>
void ExpectInvalidArgument(Function&& function, const char* description) {
    bool refused = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused, description);
}

template <typename Function>
void ExpectLogicError(Function&& function, const char* description) {
    bool refused = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        refused = false;
    } catch (const std::logic_error&) {
        refused = true;
    }
    Expect(refused, description);
}

ContinuousStateErrorTolerances MakeTolerances(int size) {
    return ContinuousStateErrorTolerances(
        1.0e-11, Eigen::VectorXd::Constant(size, 1.0e-13));
}

void AdvanceFully(CvodeContinuousStateAdvancer& advancer,
                  double stop_time_seconds) {
    Eigen::VectorXd endpoint(advancer.continuous_state_size());
    double expected_step_begin = advancer.current_time_seconds();
    while (expected_step_begin < stop_time_seconds) {
        const auto step = advancer.AdvanceOneInternalStepToward(
            stop_time_seconds, endpoint);
        Expect(step.start_time_seconds == expected_step_begin,
               "successive CVODE internal endpoints are contiguous");
        Expect(step.end_time_seconds > step.start_time_seconds,
               "a CVODE internal step advances time");
        Expect(advancer.current_time_seconds() == step.end_time_seconds,
               "the backend current time equals the returned endpoint");
        Eigen::VectorXd copied(endpoint.size());
        advancer.CopyCurrentState(copied);
        Expect((copied.array() == endpoint.array()).all(),
               "the returned endpoint equals the backend current state");
        const auto interval = advancer.dense_output_interval();
        Expect(interval.has_value() &&
                   interval->end_time_seconds == step.end_time_seconds,
               "dense output ends at the returned internal endpoint");
        if (interval.has_value()) {
            const double time_tolerance =
                64.0 * std::numeric_limits<double>::epsilon() *
                std::max(1.0, std::abs(step.start_time_seconds));
            ExpectNear(interval->start_time_seconds, step.start_time_seconds,
                       time_tolerance,
                       "adjacent CVODE dense intervals meet at the accepted "
                       "endpoint");
        }
        expected_step_begin = step.end_time_seconds;
        if (step.reached_stop) {
            Expect(step.end_time_seconds == stop_time_seconds,
                   "the final internal step ends at the requested stop");
            return;
        }
    }
}

class ConstantAccelerationRhs final : public ContinuousStateRhs {
   public:
    explicit ConstantAccelerationRhs(double acceleration)
        : acceleration_(acceleration) {}

    int continuous_state_size() const override { return 2; }

    void CalcTimeDerivatives(
        double,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) override {
        ++evaluation_count_;
        derivatives[0] = state[1];
        derivatives[1] = acceleration_;
    }

    int evaluation_count() const { return evaluation_count_; }

   private:
    double acceleration_;
    int evaluation_count_{0};
};

class LinearOscillatorRhs final : public ContinuousStateRhs {
   public:
    explicit LinearOscillatorRhs(double angular_frequency)
        : angular_frequency_(angular_frequency) {}

    int continuous_state_size() const override { return 2; }

    void CalcTimeDerivatives(
        double,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) override {
        derivatives[0] = state[1];
        derivatives[1] =
            -angular_frequency_ * angular_frequency_ * state[0];
    }

   private:
    double angular_frequency_;
};

class TimeDrivenRhs final : public ContinuousStateRhs {
   public:
    int continuous_state_size() const override { return 1; }

    void CalcTimeDerivatives(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>&,
        Eigen::Ref<Eigen::VectorXd> derivatives) override {
        derivatives[0] = time_seconds;
    }
};

class ZeroDimensionalRhs final : public ContinuousStateRhs {
   public:
    int continuous_state_size() const override { return 0; }
    void CalcTimeDerivatives(
        double, const Eigen::Ref<const Eigen::VectorXd>&,
        Eigen::Ref<Eigen::VectorXd>) override {}
};

class DeliberateRhsFailure final : public std::runtime_error {
   public:
    DeliberateRhsFailure() : std::runtime_error("deliberate RHS failure") {}
};

class ToggleFailureRhs final : public ContinuousStateRhs {
   public:
    int continuous_state_size() const override { return 1; }

    void CalcTimeDerivatives(
        double, const Eigen::Ref<const Eigen::VectorXd>&,
        Eigen::Ref<Eigen::VectorXd> derivatives) override {
        ++evaluation_count_;
        if (throw_on_evaluation_) {
            throw DeliberateRhsFailure();
        }
        derivatives[0] = 1.0;
    }

    void set_throw_on_evaluation(bool value) { throw_on_evaluation_ = value; }
    int evaluation_count() const { return evaluation_count_; }

   private:
    bool throw_on_evaluation_{false};
    int evaluation_count_{0};
};

bool SameStatistics(
    const orvd::integrators::ContinuousStateIntegrationStatistics& left,
    const orvd::integrators::ContinuousStateIntegrationStatistics& right) {
    return left.successful_internal_step_count ==
               right.successful_internal_step_count &&
           left.right_hand_side_evaluation_count ==
               right.right_hand_side_evaluation_count &&
           left.linear_solver_right_hand_side_evaluation_count ==
               right.linear_solver_right_hand_side_evaluation_count &&
           left.error_test_failure_count == right.error_test_failure_count &&
           left.nonlinear_solver_iteration_count ==
               right.nonlinear_solver_iteration_count &&
           left.nonlinear_solver_convergence_failure_count ==
               right.nonlinear_solver_convergence_failure_count &&
           left.linear_solver_setup_count == right.linear_solver_setup_count &&
           left.jacobian_evaluation_count == right.jacobian_evaluation_count;
}

bool StatisticsAreZero(
    const orvd::integrators::ContinuousStateIntegrationStatistics& value) {
    return SameStatistics(value, {});
}

Eigen::Vector2d ConstantAccelerationState(double initial_time,
                                          const Eigen::Vector2d& initial,
                                          double acceleration,
                                          double time) {
    const double dt = time - initial_time;
    return Eigen::Vector2d(
        initial[0] + initial[1] * dt + 0.5 * acceleration * dt * dt,
        initial[1] + acceleration * dt);
}

Eigen::Vector2d OscillatorState(double initial_time,
                                const Eigen::Vector2d& initial,
                                double angular_frequency,
                                double time) {
    const double phase = angular_frequency * (time - initial_time);
    return Eigen::Vector2d(
        initial[0] * std::cos(phase) +
            initial[1] * std::sin(phase) / angular_frequency,
        -initial[0] * angular_frequency * std::sin(phase) +
            initial[1] * std::cos(phase));
}

void ExpectStateNear(const Eigen::VectorXd& actual,
                     const Eigen::Vector2d& expected,
                     const char* description) {
    const double scale = std::max(1.0, expected.cwiseAbs().maxCoeff());
    const double tolerance = 1.0e-8 * scale;
    ExpectNear(actual[0], expected[0], tolerance, description);
    ExpectNear(actual[1], expected[1], tolerance, description);
}

void CheckConstructionAndCallerRefusal() {
    ZeroDimensionalRhs zero_rhs;
    ExpectInvalidArgument(
        [&] {
            CvodeContinuousStateAdvancer advancer(
                zero_rhs, 0.0, Eigen::VectorXd{}, MakeTolerances(0));
        },
        "a zero-dimensional RHS is refused");

    ConstantAccelerationRhs rhs(-9.81);
    ExpectInvalidArgument(
        [&] {
            CvodeContinuousStateAdvancer advancer(
                rhs, 0.0, Eigen::VectorXd::Zero(1), MakeTolerances(2));
        },
        "an initial-state dimension mismatch is refused");
    ExpectInvalidArgument(
        [&] {
            CvodeContinuousStateAdvancer advancer(
                rhs, 0.0, Eigen::Vector2d::Zero(), MakeTolerances(1));
        },
        "an absolute-tolerance dimension mismatch is refused");
    Eigen::Vector2d nonfinite_initial = Eigen::Vector2d::Zero();
    nonfinite_initial[1] = std::numeric_limits<double>::quiet_NaN();
    ExpectInvalidArgument(
        [&] {
            CvodeContinuousStateAdvancer advancer(
                rhs, 0.0, nonfinite_initial, MakeTolerances(2));
        },
        "a non-finite initial state is refused");
    ExpectInvalidArgument(
        [&] {
            CvodeContinuousStateAdvancer advancer(
                rhs, std::numeric_limits<double>::infinity(),
                Eigen::Vector2d::Zero(), MakeTolerances(2));
        },
        "a non-finite initial time is refused");
}

void CheckFreeFallAndDenseOutput() {
    constexpr double kInitialTime = 0.3;
    constexpr double kTargetTime = 0.55;
    constexpr double kAcceleration = -9.81;
    const Eigen::Vector2d initial_state(1.2, -0.4);
    ConstantAccelerationRhs rhs(kAcceleration);
    CvodeContinuousStateAdvancer advancer(
        rhs, kInitialTime, initial_state, MakeTolerances(2));
    Expect(StatisticsAreZero(advancer.integration_statistics()),
           "a newly initialized backend reports zero numerical work");

    Eigen::VectorXd output = Eigen::VectorXd::Constant(2, -77.0);
    advancer.CopyCurrentState(output);
    Expect((output.array() == initial_state.array()).all(),
           "construction preserves the exact initial endpoint");
    Expect(!advancer.dense_output_interval().has_value(),
           "a new backend has no dense-output interval");

    Eigen::VectorXd wrong_size = Eigen::VectorXd::Constant(1, 13.0);
    ExpectInvalidArgument(
        [&] { advancer.CopyCurrentState(wrong_size); },
        "a wrong-sized current-state output is refused");
    Expect(wrong_size[0] == 13.0,
           "current-state refusal preserves caller storage");

    AdvanceFully(advancer, kTargetTime);
    const auto advanced_statistics = advancer.integration_statistics();
    Expect(advanced_statistics.successful_internal_step_count > 0 &&
               advanced_statistics.right_hand_side_evaluation_count +
                       advanced_statistics
                           .linear_solver_right_hand_side_evaluation_count ==
                   static_cast<std::uint64_t>(rhs.evaluation_count()),
           "the read-only statistics separate ordinary and linear-solver RHS work");
    Expect(advancer.current_time_seconds() == kTargetTime,
           "CVODE stops at the requested endpoint");
    advancer.CopyCurrentState(output);
    ExpectStateNear(output,
                    ConstantAccelerationState(
                        kInitialTime, initial_state, kAcceleration, kTargetTime),
                    "free-fall endpoint matches its analytic solution");

    const auto interval = advancer.dense_output_interval();
    Expect(interval.has_value(),
           "a successful positive advance reports dense output");
    if (!interval.has_value()) return;
    Expect(interval->start_time_seconds < interval->end_time_seconds,
           "the dense-output interval has positive length");
    Expect(interval->end_time_seconds == kTargetTime,
           "the dense-output interval ends at the public endpoint");

    output.setConstant(37.0);
    ExpectInvalidArgument(
        [&] {
            (void)advancer.AdvanceOneInternalStepToward(
                kTargetTime - 0.01, output);
        },
        "backward advancement is refused before CVODE");
    Expect((output.array() == 37.0).all(),
           "backward-stop refusal preserves endpoint caller storage");
    output.setConstant(41.0);
    ExpectInvalidArgument(
        [&] {
            (void)advancer.AdvanceOneInternalStepToward(
                std::numeric_limits<double>::quiet_NaN(), output);
        },
        "a non-finite target is refused before CVODE");
    Expect((output.array() == 41.0).all(),
           "non-finite-stop refusal preserves endpoint caller storage");
    wrong_size[0] = 43.0;
    ExpectInvalidArgument(
        [&] {
            (void)advancer.AdvanceOneInternalStepToward(
                kTargetTime + 0.01, wrong_size);
        },
        "a wrong-sized endpoint output is refused before CVODE");
    Expect(wrong_size[0] == 43.0,
           "endpoint-size refusal preserves caller storage");
    Expect(advancer.current_time_seconds() == kTargetTime,
           "invalid endpoint requests preserve the successful endpoint");
    const auto interval_after_refusals = advancer.dense_output_interval();
    Expect(interval_after_refusals.has_value() &&
               interval_after_refusals->start_time_seconds ==
                   interval->start_time_seconds &&
               interval_after_refusals->end_time_seconds ==
                   interval->end_time_seconds,
           "invalid endpoint requests preserve dense output");

    const double midpoint =
        0.5 * (interval->start_time_seconds + interval->end_time_seconds);
    advancer.CopyDenseState(midpoint, output);
    ExpectStateNear(output,
                    ConstantAccelerationState(
                        kInitialTime, initial_state, kAcceleration, midpoint),
                    "dense output matches free fall inside the reported interval");
    advancer.CopyDenseState(interval->end_time_seconds, output);
    ExpectStateNear(output,
                    ConstantAccelerationState(
                        kInitialTime, initial_state, kAcceleration, kTargetTime),
                    "dense output agrees with the successful endpoint");

    output.setConstant(29.0);
    ExpectInvalidArgument(
        [&] {
            advancer.CopyDenseState(interval->start_time_seconds - 1.0, output);
        },
        "dense output outside the reported interval is refused");
    Expect((output.array() == 29.0).all(),
           "dense-output refusal preserves caller storage");
    Eigen::VectorXd wrong_dense_size = Eigen::VectorXd::Constant(1, 31.0);
    ExpectInvalidArgument(
        [&] { advancer.CopyDenseState(midpoint, wrong_dense_size); },
        "a wrong-sized dense-state output is refused");
    Expect(wrong_dense_size[0] == 31.0,
           "dense-state size refusal preserves caller storage");

    const int evaluations_before_same_time = rhs.evaluation_count();
    const auto statistics_before_same_time = advancer.integration_statistics();
    output.setConstant(47.0);
    const auto same_time_step = advancer.AdvanceOneInternalStepToward(
        kTargetTime, output);
    Expect(same_time_step.reached_stop &&
               same_time_step.start_time_seconds == kTargetTime &&
               same_time_step.end_time_seconds == kTargetTime,
           "a same-time backend request reports a reached no-op");
    Expect(rhs.evaluation_count() == evaluations_before_same_time,
           "same-time advancement does not evaluate the RHS");
    Expect(SameStatistics(statistics_before_same_time,
                          advancer.integration_statistics()),
           "same-time advancement does not change numerical-work counters");
    ExpectStateNear(output,
                    ConstantAccelerationState(
                        kInitialTime, initial_state, kAcceleration, kTargetTime),
                    "same-time advancement copies the current endpoint");
    const auto interval_after_same_time = advancer.dense_output_interval();
    Expect(interval_after_same_time.has_value() &&
               interval_after_same_time->start_time_seconds ==
                   interval->start_time_seconds &&
               interval_after_same_time->end_time_seconds ==
                   interval->end_time_seconds,
           "same-time advancement preserves dense output");

    const Eigen::Vector2d invalid_state(
        1.0, std::numeric_limits<double>::quiet_NaN());
    Eigen::VectorXd wrong_reinitialization_size = Eigen::VectorXd::Zero(1);
    ExpectInvalidArgument(
        [&] {
            advancer.ReinitializeAfterExternalChange(
                1.0, wrong_reinitialization_size);
        },
        "a wrong-sized reinitialization state is refused before CVODE");
    ExpectInvalidArgument(
        [&] {
            advancer.ReinitializeAfterExternalChange(
                std::numeric_limits<double>::infinity(), initial_state);
        },
        "a non-finite reinitialization time is refused before CVODE");
    ExpectInvalidArgument(
        [&] { advancer.ReinitializeAfterExternalChange(1.0, invalid_state); },
        "an invalid reinitialization state is refused before CVODE");
    Expect(advancer.current_time_seconds() == kTargetTime &&
               advancer.dense_output_interval().has_value(),
           "reinitialization refusal preserves endpoint and dense output");
    AdvanceFully(advancer, kTargetTime + 0.01);
    advancer.CopyCurrentState(output);
    ExpectStateNear(
        output,
        ConstantAccelerationState(kInitialTime, initial_state, kAcceleration,
                                  kTargetTime + 0.01),
        "reinitialization refusal preserves the ability to advance");
}

void CheckLinearOscillator() {
    constexpr double kInitialTime = -0.2;
    constexpr double kTargetTime = 0.25;
    constexpr double kAngularFrequency = 2.3;
    const Eigen::Vector2d initial_state(0.8, -0.25);
    LinearOscillatorRhs rhs(kAngularFrequency);
    CvodeContinuousStateAdvancer advancer(
        rhs, kInitialTime, initial_state, MakeTolerances(2));
    AdvanceFully(advancer, kTargetTime);
    Eigen::VectorXd output(2);
    advancer.CopyCurrentState(output);
    ExpectStateNear(output,
                    OscillatorState(kInitialTime, initial_state,
                                    kAngularFrequency, kTargetTime),
                    "linear-oscillator endpoint matches its analytic solution");
}

void CheckBackendTimePropagation() {
    constexpr double kInitialTime = 0.4;
    constexpr double kTargetTime = 0.7;
    constexpr double kInitialValue = 1.2;
    TimeDrivenRhs rhs;
    Eigen::VectorXd initial(1);
    initial[0] = kInitialValue;
    CvodeContinuousStateAdvancer advancer(
        rhs, kInitialTime, initial, MakeTolerances(1));
    AdvanceFully(advancer, kTargetTime);
    Eigen::VectorXd output(1);
    advancer.CopyCurrentState(output);
    const double expected =
        kInitialValue +
        0.5 * (kTargetTime * kTargetTime - kInitialTime * kInitialTime);
    ExpectNear(output[0], expected, 1.0e-8,
               "the CVODE callback forwards the backend trial time");
}

void CheckRhsFailureAndRecovery() {
    ToggleFailureRhs rhs;
    Eigen::VectorXd initial(1);
    initial[0] = 2.0;
    CvodeContinuousStateAdvancer advancer(
        rhs, 0.0, initial, MakeTolerances(1));
    AdvanceFully(advancer, 0.1);

    Eigen::VectorXd endpoint(1);
    advancer.CopyCurrentState(endpoint);
    const double accepted_value = endpoint[0];
    Expect(advancer.dense_output_interval().has_value(),
           "the control advance establishes dense output");

    rhs.set_throw_on_evaluation(true);
    const auto statistics_before_failure = advancer.integration_statistics();
    bool original_exception_propagated = false;
    try {
        AdvanceFully(advancer, 0.2);
    } catch (const DeliberateRhsFailure&) {
        original_exception_propagated = true;
    }
    Expect(original_exception_propagated,
           "an RHS exception is rethrown with its original C++ type");
    advancer.CopyCurrentState(endpoint);
    Expect(endpoint[0] == accepted_value &&
               advancer.current_time_seconds() == 0.1,
           "an RHS failure preserves the last public endpoint");
    Expect(!advancer.dense_output_interval().has_value(),
           "an RHS failure invalidates old dense output");
    const auto statistics_after_failure = advancer.integration_statistics();
    Expect(statistics_after_failure.successful_internal_step_count ==
                   statistics_before_failure.successful_internal_step_count &&
               statistics_after_failure.right_hand_side_evaluation_count +
                       statistics_after_failure
                           .linear_solver_right_hand_side_evaluation_count >
                   statistics_before_failure.right_hand_side_evaluation_count +
                       statistics_before_failure
                           .linear_solver_right_hand_side_evaluation_count,
           "failed RHS work is visible without being counted as a successful internal step");
    Eigen::VectorXd unavailable_dense = Eigen::VectorXd::Constant(1, 71.0);
    ExpectLogicError(
        [&] { advancer.CopyDenseState(0.1, unavailable_dense); },
        "an RHS failure makes dense-state copying unavailable");
    Expect(unavailable_dense[0] == 71.0,
           "unavailable dense output preserves caller storage");

    const int evaluations_after_failure = rhs.evaluation_count();
    ExpectLogicError(
        [&] { AdvanceFully(advancer, 0.2); },
        "advancement remains poisoned until reinitialization");
    Expect(rhs.evaluation_count() == evaluations_after_failure,
           "a poisoned advance does not call the RHS again");
    Expect(SameStatistics(statistics_after_failure,
                          advancer.integration_statistics()),
           "a poisoned advance does not change numerical-work counters");

    rhs.set_throw_on_evaluation(false);
    Eigen::VectorXd committed(1);
    committed[0] = 4.0;
    advancer.ReinitializeAfterExternalChange(0.15, committed);
    Expect(advancer.current_time_seconds() == 0.15 &&
               !advancer.dense_output_interval().has_value(),
           "successful reinitialization publishes the explicit endpoint and clears history");
    advancer.CopyCurrentState(endpoint);
    Expect(endpoint[0] == committed[0],
           "successful reinitialization immediately publishes the committed state");
    Expect(StatisticsAreZero(advancer.integration_statistics()),
           "successful reinitialization restarts numerical-work counters");
    AdvanceFully(advancer, 0.25);
    advancer.CopyCurrentState(endpoint);
    ExpectNear(endpoint[0], 4.1, 1.0e-8,
               "a reinitialized backend advances from the committed state");
}

}  // namespace

int main() {
    CheckConstructionAndCallerRefusal();
    CheckFreeFallAndDenseOutput();
    CheckLinearOscillator();
    CheckBackendTimePropagation();
    CheckRhsFailureAndRecovery();

    if (failure_count != 0) {
        std::printf("CVODE continuous-state advancer checks failed: %d\n",
                    failure_count);
        return 1;
    }
    std::printf("CVODE continuous-state advancer checks passed\n");
    return 0;
}
