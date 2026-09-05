#include "continuous_state_advancer_contract.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <stdexcept>

namespace orvd::integrators::test {
namespace {

int failure_count = 0;

void Expect(bool condition, const char* description) {
    if (!condition) {
        std::printf("FAIL %s\n", description);
        ++failure_count;
    }
}

void ExpectNear(double actual, double expected, double tolerance,
                const char* description) {
    if (!std::isfinite(actual) || !std::isfinite(expected) ||
        !(std::abs(actual - expected) <= tolerance)) {
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

bool SameStatistics(const ContinuousStateIntegrationStatistics& left,
                    const ContinuousStateIntegrationStatistics& right) {
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
           left.jacobian_evaluation_count == right.jacobian_evaluation_count &&
           left.requested_dense_finite_difference_jacobian_worker_count ==
               right.requested_dense_finite_difference_jacobian_worker_count;
}

bool WorkCountersAreZero(const ContinuousStateIntegrationStatistics& value) {
    return value.successful_internal_step_count == 0 &&
           value.right_hand_side_evaluation_count == 0 &&
           value.linear_solver_right_hand_side_evaluation_count == 0 &&
           value.error_test_failure_count == 0 &&
           value.nonlinear_solver_iteration_count == 0 &&
           value.nonlinear_solver_convergence_failure_count == 0 &&
           value.linear_solver_setup_count == 0 &&
           value.jacobian_evaluation_count == 0;
}

bool SameDenseInterval(
    const std::optional<ContinuousStateDenseOutputInterval>& left,
    const std::optional<ContinuousStateDenseOutputInterval>& right) {
    if (left.has_value() != right.has_value()) return false;
    return !left.has_value() ||
           (left->start_time_seconds == right->start_time_seconds &&
            left->end_time_seconds == right->end_time_seconds);
}

void AdvanceFully(ContinuousStateAdvancer& advancer,
                  double stop_time_seconds) {
    constexpr std::size_t kMaximumContractSteps = 1'000'000;
    Eigen::VectorXd endpoint(advancer.continuous_state_size());
    double expected_step_begin = advancer.current_time_seconds();
    for (std::size_t step_index = 0; step_index < kMaximumContractSteps;
         ++step_index) {
        if (!(expected_step_begin < stop_time_seconds)) return;
        const auto statistics_before = advancer.integration_statistics();
        const auto step = advancer.AdvanceOneInternalStepToward(
            stop_time_seconds, endpoint);
        const auto statistics_after = advancer.integration_statistics();
        Expect(statistics_after.successful_internal_step_count ==
                   statistics_before.successful_internal_step_count + 1,
               "one positive backend call publishes exactly one successful "
               "internal step");
        Expect(step.start_time_seconds == expected_step_begin,
               "successive backend internal endpoints are contiguous");
        Expect(step.end_time_seconds > step.start_time_seconds &&
                   step.end_time_seconds <= stop_time_seconds,
               "one backend internal step advances without passing its stop");
        Expect(step.reached_stop ==
                   (step.end_time_seconds == stop_time_seconds),
               "reached_stop exactly identifies the requested stop endpoint");
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
                       "the dense interval starts at the preceding accepted "
                       "endpoint");
        }
        expected_step_begin = step.end_time_seconds;
        if (step.reached_stop) return;
    }
    Expect(false, "the backend contract run exceeded its internal-step guard");
}

class ConstantAccelerationRhs final : public ContinuousStateRhs {
   public:
    explicit ConstantAccelerationRhs(double acceleration)
        : acceleration_(acceleration) {}

    int continuous_state_size() const override { return 2; }

    void CalcTimeDerivatives(
        double, const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) override {
        ++evaluation_count_;
        derivatives[0] = state[1];
        derivatives[1] = acceleration_;
    }

    [[nodiscard]] int evaluation_count() const { return evaluation_count_; }

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
        double, const Eigen::Ref<const Eigen::VectorXd>& state,
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
        double time_seconds, const Eigen::Ref<const Eigen::VectorXd>&,
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

class DeliberateSizeFailure final : public std::runtime_error {
   public:
    DeliberateSizeFailure()
        : std::runtime_error("deliberate RHS size failure") {}
};

class ThrowingSizeRhs final : public ContinuousStateRhs {
   public:
    int continuous_state_size() const override {
        throw DeliberateSizeFailure();
    }

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
        if (throw_on_evaluation_) throw DeliberateRhsFailure();
        derivatives[0] = 1.0;
    }

    void set_throw_on_evaluation(bool value) { throw_on_evaluation_ = value; }
    [[nodiscard]] int evaluation_count() const { return evaluation_count_; }

   private:
    bool throw_on_evaluation_{false};
    int evaluation_count_{0};
};

class NonFiniteToggleRhs final : public ContinuousStateRhs {
   public:
    [[nodiscard]] int continuous_state_size() const override { return 1; }

    void CalcTimeDerivatives(
        double, const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) override {
        derivatives[0] = return_nonfinite_
                             ? std::numeric_limits<double>::quiet_NaN()
                             : -state[0];
    }

    void set_return_nonfinite(bool value) noexcept {
        return_nonfinite_ = value;
    }

   private:
    bool return_nonfinite_{false};
};

class DeliberateRecoverableStageFailure final : public std::runtime_error {
   public:
    DeliberateRecoverableStageFailure()
        : std::runtime_error("deliberate recoverable stage failure") {}
};

class PersistentRecoverableStageRhs final : public ContinuousStateRhs {
   public:
    [[nodiscard]] int continuous_state_size() const override { return 1; }

    void CalcTimeDerivatives(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) override {
        if (enabled_ && time_seconds > accepted_time_seconds_) {
            throw DeliberateRecoverableStageFailure();
        }
        derivatives[0] = -state[0];
    }

    [[nodiscard]] bool IsRecoverableFailure(
        const std::exception_ptr& failure) const noexcept override {
        if (failure == nullptr) return false;
        try {
            std::rethrow_exception(failure);
        } catch (const DeliberateRecoverableStageFailure&) {
            return true;
        } catch (...) {
            return false;
        }
    }

    void DisableAndMoveAcceptedTime(double accepted_time_seconds) noexcept {
        enabled_ = false;
        accepted_time_seconds_ = accepted_time_seconds;
    }

   private:
    bool enabled_{true};
    double accepted_time_seconds_{};
};

Eigen::Vector2d ConstantAccelerationState(double initial_time,
                                          const Eigen::Vector2d& initial,
                                          double acceleration, double time) {
    const double dt = time - initial_time;
    return Eigen::Vector2d(
        initial[0] + initial[1] * dt + 0.5 * acceleration * dt * dt,
        initial[1] + acceleration * dt);
}

Eigen::Vector2d OscillatorState(double initial_time,
                                const Eigen::Vector2d& initial,
                                double angular_frequency, double time) {
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

void CheckConstructionAndCallerRefusal(
    ContinuousStateAdvancerFactory factory) {
    ThrowingSizeRhs throwing_size_rhs;
    bool original_size_exception = false;
    try {
        static_cast<void>(factory(
            throwing_size_rhs, 0.0, Eigen::VectorXd::Zero(1),
            MakeTolerances(1)));
    } catch (const DeliberateSizeFailure&) {
        original_size_exception = true;
    }
    Expect(original_size_exception,
           "an RHS dimension exception propagates across construction");

    ZeroDimensionalRhs zero_rhs;
    ExpectInvalidArgument(
        [&] {
            (void)factory(zero_rhs, 0.0, Eigen::VectorXd{},
                          MakeTolerances(0));
        },
        "a zero-dimensional RHS is refused");

    ConstantAccelerationRhs rhs(-9.81);
    ExpectInvalidArgument(
        [&] {
            (void)factory(rhs, 0.0, Eigen::VectorXd::Zero(1),
                          MakeTolerances(2));
        },
        "an initial-state dimension mismatch is refused");
    ExpectInvalidArgument(
        [&] {
            (void)factory(rhs, 0.0, Eigen::Vector2d::Zero(),
                          MakeTolerances(1));
        },
        "an absolute-tolerance dimension mismatch is refused");
    Eigen::Vector2d nonfinite_initial = Eigen::Vector2d::Zero();
    nonfinite_initial[1] = std::numeric_limits<double>::quiet_NaN();
    ExpectInvalidArgument(
        [&] {
            (void)factory(rhs, 0.0, nonfinite_initial, MakeTolerances(2));
        },
        "a non-finite initial state is refused");
    ExpectInvalidArgument(
        [&] {
            (void)factory(rhs, std::numeric_limits<double>::infinity(),
                          Eigen::Vector2d::Zero(), MakeTolerances(2));
        },
        "a non-finite initial time is refused");
}

void CheckEndpointDenseAndInputContract(
    ContinuousStateAdvancerFactory factory) {
    constexpr double kInitialTime = 0.3;
    constexpr double kTargetTime = 0.55;
    constexpr double kAcceleration = -9.81;
    const Eigen::Vector2d initial_state(1.2, -0.4);
    ConstantAccelerationRhs rhs(kAcceleration);
    auto advancer = factory(rhs, kInitialTime, initial_state, MakeTolerances(2));
    const auto initial_statistics = advancer->integration_statistics();
    const int initial_worker_identity =
        initial_statistics
            .requested_dense_finite_difference_jacobian_worker_count;
    Expect(WorkCountersAreZero(initial_statistics) &&
               initial_worker_identity > 0,
           "a new backend reports zero work and a positive worker identity");

    Eigen::VectorXd output = Eigen::VectorXd::Constant(2, -77.0);
    advancer->CopyCurrentState(output);
    Expect((output.array() == initial_state.array()).all(),
           "construction preserves the exact initial endpoint");
    Expect(!advancer->dense_output_interval().has_value(),
           "a new backend has no dense-output interval");

    Eigen::VectorXd unavailable_dense = Eigen::VectorXd::Constant(2, 17.0);
    ExpectLogicError(
        [&] { advancer->CopyDenseState(kInitialTime, unavailable_dense); },
        "a new backend refuses dense-state copying");
    Expect((unavailable_dense.array() == 17.0).all(),
           "unavailable dense output preserves caller storage");

    Eigen::VectorXd wrong_size = Eigen::VectorXd::Constant(1, 13.0);
    ExpectInvalidArgument(
        [&] { advancer->CopyCurrentState(wrong_size); },
        "a wrong-sized current-state output is refused");
    Expect(wrong_size[0] == 13.0,
           "current-state refusal preserves caller storage");

    AdvanceFully(*advancer, kTargetTime);
    const auto advanced_statistics = advancer->integration_statistics();
    Expect(advanced_statistics.successful_internal_step_count > 0 &&
               advanced_statistics
                       .requested_dense_finite_difference_jacobian_worker_count ==
                   initial_worker_identity &&
               advanced_statistics.right_hand_side_evaluation_count +
                       advanced_statistics
                           .linear_solver_right_hand_side_evaluation_count ==
                   static_cast<std::uint64_t>(rhs.evaluation_count()),
           "statistics partition every RHS evaluation without overlap");
    Expect(advancer->current_time_seconds() == kTargetTime,
           "the backend stops at the requested endpoint");
    advancer->CopyCurrentState(output);
    const Eigen::VectorXd accepted_endpoint = output;
    ExpectStateNear(
        output,
        ConstantAccelerationState(kInitialTime, initial_state, kAcceleration,
                                  kTargetTime),
        "the endpoint matches the analytic solution");

    const auto interval = advancer->dense_output_interval();
    Expect(interval.has_value(),
           "a successful positive advance reports dense output");
    if (!interval.has_value()) return;
    Expect(interval->start_time_seconds < interval->end_time_seconds &&
               interval->end_time_seconds == kTargetTime,
           "the dense interval is positive and ends at the public endpoint");

    const auto statistics_before_refusals = advancer->integration_statistics();
    output.setConstant(37.0);
    ExpectInvalidArgument(
        [&] {
            (void)advancer->AdvanceOneInternalStepToward(kTargetTime - 0.01,
                                                         output);
        },
        "backward advancement is refused before the backend is entered");
    Expect((output.array() == 37.0).all(),
           "backward-stop refusal preserves endpoint caller storage");
    output.setConstant(41.0);
    ExpectInvalidArgument(
        [&] {
            (void)advancer->AdvanceOneInternalStepToward(
                std::numeric_limits<double>::quiet_NaN(), output);
        },
        "a non-finite target is refused before the backend is entered");
    Expect((output.array() == 41.0).all(),
           "non-finite-stop refusal preserves endpoint caller storage");
    wrong_size[0] = 43.0;
    ExpectInvalidArgument(
        [&] {
            (void)advancer->AdvanceOneInternalStepToward(kTargetTime + 0.01,
                                                         wrong_size);
        },
        "a wrong-sized endpoint output is refused before backend entry");
    Expect(wrong_size[0] == 43.0,
           "endpoint-size refusal preserves caller storage");
    Eigen::VectorXd observed_endpoint(2);
    advancer->CopyCurrentState(observed_endpoint);
    Expect(advancer->current_time_seconds() == kTargetTime &&
               (observed_endpoint.array() == accepted_endpoint.array()).all() &&
               SameDenseInterval(interval, advancer->dense_output_interval()) &&
               SameStatistics(statistics_before_refusals,
                              advancer->integration_statistics()),
           "invalid endpoint requests preserve endpoint, dense history, "
           "statistics and usability");

    const double midpoint =
        0.5 * (interval->start_time_seconds + interval->end_time_seconds);
    advancer->CopyDenseState(interval->start_time_seconds, output);
    ExpectStateNear(
        output,
        ConstantAccelerationState(kInitialTime, initial_state, kAcceleration,
                                  interval->start_time_seconds),
        "dense output matches the analytic state at its first boundary");
    advancer->CopyDenseState(midpoint, output);
    ExpectStateNear(
        output,
        ConstantAccelerationState(kInitialTime, initial_state, kAcceleration,
                                  midpoint),
        "dense output matches the analytic state inside its interval");
    advancer->CopyDenseState(interval->end_time_seconds, output);
    ExpectStateNear(output, accepted_endpoint,
                    "dense output agrees with the successful endpoint");

    const auto statistics_before_dense_refusals =
        advancer->integration_statistics();
    output.setConstant(29.0);
    ExpectInvalidArgument(
        [&] {
            advancer->CopyDenseState(interval->start_time_seconds - 1.0,
                                     output);
        },
        "dense output outside the reported interval is refused");
    Expect((output.array() == 29.0).all(),
           "dense-output refusal preserves caller storage");
    output.setConstant(30.0);
    ExpectInvalidArgument(
        [&] {
            advancer->CopyDenseState(
                std::numeric_limits<double>::quiet_NaN(), output);
        },
        "a non-finite dense-output time is refused");
    Expect((output.array() == 30.0).all(),
           "non-finite dense-output refusal preserves caller storage");
    Eigen::VectorXd wrong_dense_size = Eigen::VectorXd::Constant(1, 31.0);
    ExpectInvalidArgument(
        [&] { advancer->CopyDenseState(midpoint, wrong_dense_size); },
        "a wrong-sized dense-state output is refused");
    Expect(wrong_dense_size[0] == 31.0,
           "dense-state size refusal preserves caller storage");
    Expect(SameDenseInterval(interval, advancer->dense_output_interval()) &&
               SameStatistics(statistics_before_dense_refusals,
                              advancer->integration_statistics()),
           "invalid dense requests preserve dense history and statistics");

    const int evaluations_before_same_time = rhs.evaluation_count();
    const auto statistics_before_same_time = advancer->integration_statistics();
    output.setConstant(47.0);
    const auto same_time_step = advancer->AdvanceOneInternalStepToward(
        kTargetTime, output);
    Expect(same_time_step.reached_stop &&
               same_time_step.start_time_seconds == kTargetTime &&
               same_time_step.end_time_seconds == kTargetTime,
           "a same-time request reports an exact reached no-op");
    Expect(rhs.evaluation_count() == evaluations_before_same_time &&
               SameStatistics(statistics_before_same_time,
                              advancer->integration_statistics()),
           "same-time advancement performs no numerical work");
    Expect((output.array() == accepted_endpoint.array()).all(),
           "same-time advancement copies the exact current endpoint");
    Expect(SameDenseInterval(interval, advancer->dense_output_interval()),
           "same-time advancement preserves dense output");

    const Eigen::Vector2d invalid_state(
        1.0, std::numeric_limits<double>::quiet_NaN());
    Eigen::VectorXd wrong_reinitialization_size = Eigen::VectorXd::Zero(1);
    const auto statistics_before_reinit_refusals =
        advancer->integration_statistics();
    ExpectInvalidArgument(
        [&] {
            advancer->ReinitializeAfterExternalChange(
                1.0, wrong_reinitialization_size);
        },
        "a wrong-sized reinitialization state is refused");
    ExpectInvalidArgument(
        [&] {
            advancer->ReinitializeAfterExternalChange(
                std::numeric_limits<double>::infinity(), initial_state);
        },
        "a non-finite reinitialization time is refused");
    ExpectInvalidArgument(
        [&] {
            advancer->ReinitializeAfterExternalChange(1.0, invalid_state);
        },
        "a non-finite reinitialization state is refused");
    advancer->CopyCurrentState(observed_endpoint);
    Expect(advancer->current_time_seconds() == kTargetTime &&
               (observed_endpoint.array() == accepted_endpoint.array()).all() &&
               SameDenseInterval(interval, advancer->dense_output_interval()) &&
               SameStatistics(statistics_before_reinit_refusals,
                              advancer->integration_statistics()),
           "invalid reinitialization preserves endpoint, dense history, "
           "statistics and usability");
    AdvanceFully(*advancer, kTargetTime + 0.01);
    advancer->CopyCurrentState(observed_endpoint);
    ExpectStateNear(
        observed_endpoint,
        ConstantAccelerationState(kInitialTime, initial_state, kAcceleration,
                                  kTargetTime + 0.01),
        "invalid reinitialization leaves the hidden integration history "
        "unchanged");
}

void CheckLinearOscillator(ContinuousStateAdvancerFactory factory) {
    constexpr double kInitialTime = -0.2;
    constexpr double kTargetTime = 0.25;
    constexpr double kAngularFrequency = 2.3;
    const Eigen::Vector2d initial_state(0.8, -0.25);
    LinearOscillatorRhs rhs(kAngularFrequency);
    auto advancer = factory(rhs, kInitialTime, initial_state, MakeTolerances(2));
    AdvanceFully(*advancer, kTargetTime);
    Eigen::VectorXd output(2);
    advancer->CopyCurrentState(output);
    ExpectStateNear(output,
                    OscillatorState(kInitialTime, initial_state,
                                    kAngularFrequency, kTargetTime),
                    "the oscillator endpoint matches its analytic solution");
}

void CheckBackendTimePropagation(ContinuousStateAdvancerFactory factory) {
    constexpr double kInitialTime = 0.4;
    constexpr double kTargetTime = 0.7;
    constexpr double kInitialValue = 1.2;
    TimeDrivenRhs rhs;
    Eigen::VectorXd initial(1);
    initial[0] = kInitialValue;
    auto advancer = factory(rhs, kInitialTime, initial, MakeTolerances(1));
    AdvanceFully(*advancer, kTargetTime);
    Eigen::VectorXd output(1);
    advancer->CopyCurrentState(output);
    const double expected =
        kInitialValue +
        0.5 * (kTargetTime * kTargetTime - kInitialTime * kInitialTime);
    ExpectNear(output[0], expected, 1.0e-8,
               "the backend forwards trial time to the RHS");
}

void CheckFatalFailurePoisonAndReinitialization(
    ContinuousStateAdvancerFactory factory) {
    ToggleFailureRhs rhs;
    Eigen::VectorXd initial(1);
    initial[0] = 2.0;
    auto advancer = factory(rhs, 0.0, initial, MakeTolerances(1));
    AdvanceFully(*advancer, 0.1);

    Eigen::VectorXd accepted_endpoint(1);
    advancer->CopyCurrentState(accepted_endpoint);
    const double accepted_value = accepted_endpoint[0];
    Expect(advancer->dense_output_interval().has_value(),
           "the control advance establishes dense output");

    rhs.set_throw_on_evaluation(true);
    const auto statistics_before_failure = advancer->integration_statistics();
    Eigen::VectorXd failed_output = Eigen::VectorXd::Constant(1, 59.0);
    bool original_exception_propagated = false;
    try {
        (void)advancer->AdvanceOneInternalStepToward(0.2, failed_output);
    } catch (const DeliberateRhsFailure&) {
        original_exception_propagated = true;
    }
    Expect(original_exception_propagated,
           "a fatal RHS exception is rethrown with its original C++ type");
    Expect(failed_output[0] == 59.0,
           "a failed positive call does not publish a partial output endpoint");
    Eigen::VectorXd observed_endpoint(1);
    advancer->CopyCurrentState(observed_endpoint);
    Expect(observed_endpoint[0] == accepted_value &&
               advancer->current_time_seconds() == 0.1,
           "a fatal RHS failure preserves the last successful endpoint");
    Expect(!advancer->dense_output_interval().has_value(),
           "a fatal RHS failure invalidates old dense output");
    const auto statistics_after_failure = advancer->integration_statistics();
    Expect(statistics_after_failure.successful_internal_step_count ==
                   statistics_before_failure.successful_internal_step_count &&
               statistics_after_failure.right_hand_side_evaluation_count +
                       statistics_after_failure
                           .linear_solver_right_hand_side_evaluation_count >
                   statistics_before_failure.right_hand_side_evaluation_count +
                       statistics_before_failure
                           .linear_solver_right_hand_side_evaluation_count,
           "failed RHS work is visible without a successful-step increment");

    Eigen::VectorXd unavailable_dense = Eigen::VectorXd::Constant(1, 71.0);
    ExpectLogicError(
        [&] { advancer->CopyDenseState(0.1, unavailable_dense); },
        "a backend failure makes dense-state copying unavailable");
    Expect(unavailable_dense[0] == 71.0,
           "failed-backend dense refusal preserves caller storage");

    const int evaluations_after_failure = rhs.evaluation_count();
    Eigen::VectorXd poisoned_output = Eigen::VectorXd::Constant(1, 73.0);
    ExpectLogicError(
        [&] {
            (void)advancer->AdvanceOneInternalStepToward(0.2,
                                                         poisoned_output);
        },
        "positive advancement remains poisoned until reinitialization");
    Expect(poisoned_output[0] == 73.0,
           "a poisoned positive call preserves caller storage");
    Eigen::VectorXd poisoned_same_time_output =
        Eigen::VectorXd::Constant(1, 79.0);
    ExpectLogicError(
        [&] {
            (void)advancer->AdvanceOneInternalStepToward(
                0.1, poisoned_same_time_output);
        },
        "same-time advancement is also poisoned until reinitialization");
    Expect(poisoned_same_time_output[0] == 79.0,
           "a poisoned same-time call preserves caller storage");
    Expect(rhs.evaluation_count() == evaluations_after_failure &&
               SameStatistics(statistics_after_failure,
                              advancer->integration_statistics()),
           "poisoned calls perform no RHS or numerical work");

    rhs.set_throw_on_evaluation(false);
    Eigen::VectorXd committed(1);
    committed[0] = 4.0;
    const int worker_identity =
        statistics_after_failure
            .requested_dense_finite_difference_jacobian_worker_count;
    advancer->ReinitializeAfterExternalChange(0.15, committed);
    Expect(advancer->current_time_seconds() == 0.15 &&
               !advancer->dense_output_interval().has_value(),
           "successful reinitialization publishes the explicit endpoint and "
           "clears dense history");
    advancer->CopyCurrentState(observed_endpoint);
    Expect(observed_endpoint[0] == committed[0],
           "successful reinitialization publishes the committed state exactly");
    const auto reinitialized_statistics = advancer->integration_statistics();
    Expect(WorkCountersAreZero(reinitialized_statistics) &&
               reinitialized_statistics
                       .requested_dense_finite_difference_jacobian_worker_count ==
                   worker_identity,
           "successful reinitialization clears work counters and preserves "
           "worker identity");
    AdvanceFully(*advancer, 0.25);
    advancer->CopyCurrentState(observed_endpoint);
    ExpectNear(observed_endpoint[0], 4.1, 1.0e-8,
               "a reinitialized backend advances from the committed state");
}

void CheckNumericalFailureClassification(
    ContinuousStateAdvancerFactory factory,
    ContinuousStateNumericalFailure::Reason expected_reason,
    int expected_backend_code) {
    NonFiniteToggleRhs rhs;
    Eigen::VectorXd initial(1);
    initial[0] = 1.0;
    auto advancer = factory(rhs, 0.0, initial, MakeTolerances(1));
    Eigen::VectorXd output(1);
    const auto accepted_step =
        advancer->AdvanceOneInternalStepToward(0.1, output);
    const double accepted_time = accepted_step.end_time_seconds;
    Eigen::VectorXd accepted_state(1);
    advancer->CopyCurrentState(accepted_state);
    const auto statistics_before_failure = advancer->integration_statistics();
    Expect(accepted_time > 0.0 &&
               advancer->dense_output_interval().has_value(),
           "a control advance establishes an endpoint before a named "
           "numerical failure");

    rhs.set_return_nonfinite(true);
    output[0] = 91.0;
    bool classified = false;
    try {
        static_cast<void>(advancer->AdvanceOneInternalStepToward(
            accepted_time + 0.1, output));
    } catch (const ContinuousStateNumericalFailure& failure) {
        classified = failure.reason() == expected_reason &&
                     failure.backend_code() == expected_backend_code;
    } catch (...) {
    }
    Expect(classified,
           "a normal-return non-finite RHS receives the named numerical "
           "classification and original backend code");
    Eigen::VectorXd observed_state(1);
    advancer->CopyCurrentState(observed_state);
    const auto statistics_after_failure = advancer->integration_statistics();
    Expect(output[0] == 91.0 &&
               advancer->current_time_seconds() == accepted_time &&
               observed_state[0] == accepted_state[0] &&
               !advancer->dense_output_interval().has_value() &&
               statistics_after_failure.successful_internal_step_count ==
                   statistics_before_failure.successful_internal_step_count &&
               statistics_after_failure.right_hand_side_evaluation_count >
                   statistics_before_failure.right_hand_side_evaluation_count,
           "a named numerical failure preserves the accepted endpoint and "
           "caller output while invalidating dense output");

    output[0] = 92.0;
    ExpectLogicError(
        [&] {
            static_cast<void>(advancer->AdvanceOneInternalStepToward(
                accepted_time + 0.1, output));
        },
        "a named numerical failure poisons reuse until reinitialization");
    Expect(output[0] == 92.0,
           "a poisoned post-failure call preserves caller storage");

    rhs.set_return_nonfinite(false);
    advancer->ReinitializeAfterExternalChange(accepted_time, accepted_state);
    const auto recovered = advancer->AdvanceOneInternalStepToward(
        accepted_time + 1.0e-6, output);
    Expect(recovered.start_time_seconds == accepted_time &&
               recovered.end_time_seconds > accepted_time &&
               recovered.end_time_seconds <= accepted_time + 1.0e-6 &&
               advancer->current_time_seconds() == recovered.end_time_seconds,
           "successful reinitialization reopens advancement after a named "
           "numerical failure");
}

void CheckRecoverableExceptionCausality(
    ContinuousStateAdvancerFactory factory) {
    PersistentRecoverableStageRhs rhs;
    Eigen::VectorXd initial(1);
    initial[0] = 1.0;
    auto advancer = factory(rhs, 0.0, initial, MakeTolerances(1));
    Eigen::VectorXd output = Eigen::VectorXd::Constant(1, 93.0);
    bool original_type_rethrown = false;
    try {
        static_cast<void>(
            advancer->AdvanceOneInternalStepToward(0.1, output));
    } catch (const DeliberateRecoverableStageFailure&) {
        original_type_rethrown = true;
    } catch (...) {
    }
    Eigen::VectorXd observed_state(1);
    advancer->CopyCurrentState(observed_state);
    Expect(original_type_rethrown,
           "persistent recoverable RHS refusal retains its original C++ "
           "exception type");
    Expect(output[0] == 93.0 && advancer->current_time_seconds() == 0.0 &&
               observed_state[0] == initial[0] &&
               !advancer->dense_output_interval().has_value() &&
               advancer->integration_statistics()
                       .successful_internal_step_count == 0,
           "recoverable RHS exhaustion preserves endpoint, output and dense "
           "state atomically");

    output[0] = 94.0;
    ExpectLogicError(
        [&] {
            static_cast<void>(
                advancer->AdvanceOneInternalStepToward(0.1, output));
        },
        "recoverable RHS exhaustion poisons reuse until reinitialization");
    Expect(output[0] == 94.0,
           "a poisoned post-exhaustion call preserves caller storage");

    rhs.DisableAndMoveAcceptedTime(0.0);
    advancer->ReinitializeAfterExternalChange(0.0, initial);
    const auto recovered =
        advancer->AdvanceOneInternalStepToward(1.0e-6, output);
    Expect(recovered.start_time_seconds == 0.0 &&
               recovered.end_time_seconds > 0.0 &&
               recovered.end_time_seconds <= 1.0e-6 &&
               advancer->current_time_seconds() ==
                   recovered.end_time_seconds,
           "successful reinitialization reopens advancement after recoverable "
           "RHS exhaustion");
}

}  // namespace

int RunContinuousStateAdvancerContract(
    ContinuousStateAdvancerFactory factory) {
    failure_count = 0;
    CheckConstructionAndCallerRefusal(factory);
    CheckEndpointDenseAndInputContract(factory);
    CheckLinearOscillator(factory);
    CheckBackendTimePropagation(factory);
    CheckFatalFailurePoisonAndReinitialization(factory);
    return failure_count;
}

int RunContinuousStateFailureContract(
    ContinuousStateAdvancerFactory factory,
    ContinuousStateNumericalFailure::Reason expected_nonfinite_rhs_reason,
    int expected_backend_code) {
    failure_count = 0;
    CheckNumericalFailureClassification(
        factory, expected_nonfinite_rhs_reason, expected_backend_code);
    CheckRecoverableExceptionCausality(factory);
    return failure_count;
}

}  // namespace orvd::integrators::test
