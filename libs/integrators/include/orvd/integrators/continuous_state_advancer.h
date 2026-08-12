#pragma once

/// @file
/// The backend-neutral contracts shared by the continuous-state advancer and
/// its right-hand side.

#include <cstdint>
#include <exception>
#include <optional>

#include <Eigen/Dense>

namespace orvd::integrators {

/// Numerical error tolerances for one dynamically sized continuous state.
class ContinuousStateErrorTolerances {
   public:
    ContinuousStateErrorTolerances(
        double relative_tolerance,
        Eigen::VectorXd component_absolute_tolerances);

    [[nodiscard]] double relative_tolerance() const {
        return relative_tolerance_;
    }
    [[nodiscard]] const Eigen::VectorXd& component_absolute_tolerances() const {
        return component_absolute_tolerances_;
    }

   private:
    double relative_tolerance_;
    Eigen::VectorXd component_absolute_tolerances_;
};

/// A contiguous, dynamically sized right-hand side for an ODE backend.
class ContinuousStateRhs {
   public:
    virtual ~ContinuousStateRhs() = default;

    [[nodiscard]] virtual int continuous_state_size() const = 0;

    /// Evaluates one trial state into caller-owned, already-sized storage.
    /// Implementations must not commit the trial to an accepted state.
    virtual void CalcTimeDerivatives(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
        Eigen::Ref<Eigen::VectorXd> state_time_derivatives) = 0;

    /// Classifies an exception from the preceding trial evaluation.
    ///
    /// Returning true permits a numerical backend to retry that trial, for
    /// example with a smaller step. The default is fatal. Implementations must
    /// classify by exception type rather than diagnostic text and must not
    /// mutate accepted physical state.
    [[nodiscard]] virtual bool IsRecoverableFailure(
        const std::exception_ptr&) const noexcept {
        return false;
    }
};

/// The closed interval on which the most recent successful backend step can
/// provide dense output.
struct ContinuousStateDenseOutputInterval final {
    double start_time_seconds{};
    double end_time_seconds{};
};

/// One successful call to the numerical backend's internal-step surface.
struct ContinuousStateInternalStep final {
    double start_time_seconds{};
    double end_time_seconds{};
    bool reached_stop{false};
};

/// Cumulative numerical work since construction or the most recent successful
/// backend reinitialization.
///
/// These counters are read only when requested. They do not install a callback,
/// sample internal steps or time the right-hand side. Error-test failures and
/// nonlinear convergence failures remain separate because neither is a generic
/// "rejected step" count. The two RHS counters are also separate: a numerical
/// Jacobian may evaluate the RHS through the linear-solver interface without
/// incrementing the ordinary RHS counter.
struct ContinuousStateIntegrationStatistics final {
    std::uint64_t successful_internal_step_count{};
    std::uint64_t right_hand_side_evaluation_count{};
    std::uint64_t linear_solver_right_hand_side_evaluation_count{};
    std::uint64_t error_test_failure_count{};
    std::uint64_t nonlinear_solver_iteration_count{};
    std::uint64_t nonlinear_solver_convergence_failure_count{};
    std::uint64_t linear_solver_setup_count{};
    std::uint64_t jacobian_evaluation_count{};
    // Numerical execution identity, not an observed OpenMP team size. A value
    // of one denotes SUNDIALS' built-in serial dense difference path.
    int requested_dense_finite_difference_jacobian_worker_count{};
};

/// The numerical-backend-independent advancement surface.
///
/// G44 supplies the first implementation, CVODE.  No Drake integrator subtype
/// or compatibility route is present behind this interface.
class ContinuousStateAdvancer {
   public:
    virtual ~ContinuousStateAdvancer() = default;

    /// Returns the fixed continuous-state dimension of this backend instance.
    [[nodiscard]] virtual int continuous_state_size() const = 0;

    [[nodiscard]] virtual double current_time_seconds() const = 0;

    /// Returns a backend-neutral snapshot of cumulative numerical work.
    /// Querying statistics does not alter advancement or dense-output state.
    /// @throws std::runtime_error if the backend cannot supply a valid snapshot.
    [[nodiscard]] virtual ContinuousStateIntegrationStatistics
    integration_statistics() const = 0;

    /// Copies the current successful backend endpoint into pre-sized storage.
    /// A size error must be reported before the output is changed.
    /// @throws std::invalid_argument if the output size is wrong.
    virtual void CopyCurrentState(
        Eigen::Ref<Eigen::VectorXd> continuous_state) const = 0;

    /// Advances by at most one successful internal step toward `stop_time`.
    ///
    /// The returned endpoint is also copied into caller-owned, already-sized
    /// storage. A same-time request is a no-op that reports `reached_stop`.
    /// Invalid caller input is rejected before the backend is entered and
    /// leaves the endpoint output, dense output, and ability to advance
    /// unchanged. A failure after entering the backend preserves the last
    /// successful endpoint, invalidates dense output, and requires successful
    /// reinitialization before another step.
    /// @throws std::invalid_argument if the stop is non-finite or earlier than
    /// `current_time_seconds()`, or if the endpoint output has the wrong size.
    /// @throws std::logic_error if a prior backend failure requires
    /// reinitialization.
    [[nodiscard]] virtual ContinuousStateInternalStep
    AdvanceOneInternalStepToward(
        double stop_time_seconds,
        Eigen::Ref<Eigen::VectorXd> endpoint_continuous_state) = 0;

    /// Rebuilds backend history after an external state, parameter or input
    /// change. The complete committed endpoint is explicit so a numerical
    /// backend never has to retain or discover a system runtime context. Time
    /// and every state component must be finite, and the state size must equal
    /// `continuous_state_size()`; those checks precede any backend mutation.
    /// Backend reinitialization failure preserves the last public endpoint but
    /// leaves advancement unavailable until a reinitialization succeeds.
    /// @throws std::invalid_argument if time or state is non-finite, or the
    /// state size is wrong.
    virtual void ReinitializeAfterExternalChange(
        double committed_time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& committed_continuous_state) =
        0;

    /// Returns the valid interval for dense output from the most recent
    /// successful positive-length internal step. A new or reinitialized
    /// backend, and a backend whose last advance failed, returns no interval.
    [[nodiscard]] virtual std::optional<ContinuousStateDenseOutputInterval>
    dense_output_interval() const = 0;

    /// Copies the interpolated state at a time in `dense_output_interval()`.
    /// Invalid time, unavailable dense output, or wrong-sized output must fail
    /// before the caller's storage is changed.
    /// @throws std::invalid_argument if time is non-finite, outside the current
    /// interval, or the output size is wrong.
    /// @throws std::logic_error if dense output is unavailable.
    virtual void CopyDenseState(
        double time_seconds,
        Eigen::Ref<Eigen::VectorXd> continuous_state) const = 0;
};

}  // namespace orvd::integrators
