#pragma once

/// @file
/// Narrow, stateful Radau IIA order-five numerical core.

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>

#include <Eigen/Dense>

namespace orvd::radau5 {

/// Result of one RHS attempt across the core's non-throwing callback boundary.
enum class RhsEvaluationStatus {
    kSuccess,
    kRecoverableFailure,
    kFatalFailure,
};

/// Borrowed first-order ODE callback used by one Radau5 core instance.
class RhsEvaluator {
   public:
    virtual ~RhsEvaluator() = default;

    [[nodiscard]] virtual int continuous_state_size() const noexcept = 0;

    [[nodiscard]] virtual RhsEvaluationStatus Evaluate(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
        Eigen::Ref<Eigen::VectorXd> state_time_derivatives) noexcept = 0;
};

/// A failure that occurred after a valid positive-length advance entered the
/// numerical state machine.
class Failure final : public std::runtime_error {
   public:
    enum class Reason {
        kFatalRhs,
        kRecoverableRhsExhausted,
        kNonFiniteRhs,
        kTrialBudgetExceeded,
        kStepTooSmall,
        kRepeatedlySingular,
        kNonFiniteLinearSystem,
        kNonFiniteInternalState,
    };

    Failure(Reason reason, const char* message);

    [[nodiscard]] Reason reason() const noexcept { return reason_; }

   private:
    Reason reason_;
};

/// One accepted internal Radau5 step.
struct InternalStep final {
    double start_time_seconds{};
    double end_time_seconds{};
    bool reached_stop{};
};

/// The most recent accepted collocation interval.
struct DenseOutputInterval final {
    double start_time_seconds{};
    double end_time_seconds{};
};

/// Numerical work since construction or the most recent reinitialization.
struct Statistics final {
    std::uint64_t successful_internal_step_count{};
    std::uint64_t right_hand_side_evaluation_count{};
    std::uint64_t linear_solver_right_hand_side_evaluation_count{};
    std::uint64_t error_test_failure_count{};
    std::uint64_t nonlinear_solver_iteration_count{};
    std::uint64_t nonlinear_solver_convergence_failure_count{};
    std::uint64_t linear_solver_setup_count{};
    std::uint64_t jacobian_evaluation_count{};
};

namespace internal {

/// Result of the frozen RADAU5 Newton prediction and stopping test.
///
/// This source-tree-only detail is shared with the core verification so the
/// reachable sixth-correction boundary can be tested deterministically. It is
/// not an installed ORVD integration interface.
struct NewtonCorrectionAssessment final {
    double theta{};
    double previous_ratio{};
    double faccon{};
    double retry_multiplier{0.5};
    bool reject_trial{};
    bool converges_after_application{};
};

/// Matrices and solutions produced by the production shifted-system path.
///
/// This source-tree-only qualification seam keeps near-singular real/complex
/// residual checks out of the adaptive step state machine.  It is not an
/// installed ORVD integration interface.
struct ShiftedLinearSolveQualificationResult final {
    Eigen::MatrixXd real_matrix;
    Eigen::MatrixXcd complex_matrix;
    Eigen::VectorXd real_solution;
    Eigen::VectorXcd complex_solution;
};

[[nodiscard]] NewtonCorrectionAssessment AssessNewtonCorrection(
    int iteration,
    double correction_norm,
    double previous_correction_norm,
    double previous_ratio,
    double theta,
    double faccon,
    double newton_tolerance);

[[nodiscard]] ShiftedLinearSolveQualificationResult
SolveShiftedLinearSystemsForQualification(
    const Eigen::Ref<const Eigen::MatrixXd>& jacobian,
    double step_size_seconds,
    const Eigen::Ref<const Eigen::VectorXd>& real_right_hand_side,
    const Eigen::Ref<const Eigen::VectorXcd>& complex_right_hand_side);

}  // namespace internal

/// Stateful three-stage, fifth-order Radau IIA solver for y'=f(t,y).
///
/// The callback and state dimension are fixed for the lifetime of the object.
/// Every positive-length call publishes at most one accepted internal step.
/// All workspaces, Jacobian history and controller state are instance-owned.
class Core final {
   public:
    Core(RhsEvaluator& rhs,
         double initial_time_seconds,
         Eigen::VectorXd initial_continuous_state,
         double relative_tolerance,
         Eigen::VectorXd component_absolute_tolerances);
    ~Core();

    Core(const Core&) = delete;
    Core& operator=(const Core&) = delete;
    Core(Core&&) = delete;
    Core& operator=(Core&&) = delete;

    [[nodiscard]] int continuous_state_size() const noexcept;
    [[nodiscard]] double current_time_seconds() const noexcept;
    [[nodiscard]] Statistics statistics() const noexcept;

    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd> output) const;

    [[nodiscard]] InternalStep AdvanceOneAcceptedStepToward(
        double stop_time_seconds);

    void Reinitialize(
        double committed_time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& committed_continuous_state);

    /// Discards only Jacobian and shifted-system history after an accepted
    /// endpoint changes numerical RHS history that is not represented in
    /// `(t, y)`.  The accepted state, step controller, collocation
    /// extrapolation and dense-output interval remain valid.
    void InvalidateLinearizationAfterNumericalRhsHistoryChange() noexcept;

    [[nodiscard]] std::optional<DenseOutputInterval> dense_output_interval()
        const noexcept;

    void CopyDenseState(double time_seconds,
                        Eigen::Ref<Eigen::VectorXd> output) const;

    /// Source-tree test seam for fixed-step order measurements. Product
    /// adapters never call this method and retain the frozen 1e-6 initial step.
    void SetSuggestedStepSizeForTesting(double step_size_seconds);

    /// Source-tree test seam for exercising the inclusive trial-budget edge
    /// without running 100000 deliberately rejected trials. Product adapters
    /// never call this method and retain the frozen production budget.
    void SetMaximumTrialsPerCallForTesting(int maximum_trials);

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::radau5
