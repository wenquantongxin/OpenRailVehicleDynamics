#include "orvd/radau5/radau5_core.h"

// Derived from the RADAU5 algorithm by E. Hairer and G. Wanner and the C++
// translation by Blake Ashby. The admitted source identities, UNIGE licence
// and ORVD modifications are recorded beside this file in external/radau5.

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <string>
#include <utility>

#include <Eigen/LU>

#if defined(__FAST_MATH__)
#error "Radau5 qualification forbids __FAST_MATH__"
#endif

#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ != 0
#error "Radau5 qualification forbids finite-math-only semantics"
#endif

#if defined(__GNUC__) && !defined(__clang__) && \
    defined(__GCC_IEC_559) && __GCC_IEC_559 < 1
#error "Radau5 qualification requires GCC IEC-559 semantics"
#endif

namespace orvd::radau5 {
namespace {

static_assert(std::numeric_limits<double>::is_iec559);
static_assert(std::numeric_limits<double>::radix == 2);
static_assert(std::numeric_limits<double>::digits == 53);
static_assert(std::numeric_limits<double>::max_exponent == 1024);

constexpr double kRoundingUnit = 1.0e-16;
constexpr double kInitialStepSizeSeconds = 1.0e-6;
constexpr int kMaximumNewtonIterations = 7;
constexpr int kMaximumTrialsPerCall = 100000;
constexpr int kMaximumSingularSetups = 5;
constexpr int kMaximumJacobianPerturbationRetries = 4;

constexpr double kC1 = 0.15505102572168219018;
constexpr double kC2 = 0.64494897427831780982;
constexpr double kC1MinusOne = kC1 - 1.0;
constexpr double kC2MinusOne = kC2 - 1.0;
constexpr double kC1MinusC2 = kC1 - kC2;

// Eigenvalues of A^{-1} after the real/complex Radau transformation.
constexpr double kRealEigenvalue = 3.6378342527444957322;
constexpr double kComplexEigenvalueReal = 2.6810828736277521339;
constexpr double kComplexEigenvalueImaginary = 3.0504301992474105694;

// T transformation used by the official RADAU5 implementation. Rows map one
// real and one complex mode back to the three physical stage increments.
constexpr std::array<std::array<double, 3>, 3> kTransform{{
    {0.091232394870892942792, -0.14125529502095420843,
     -0.030029194105147424492},
    {0.24171793270710701896, 0.20412935229379993199,
     0.38294211275726193779},
    {0.96604818261509293619, 1.0, 0.0},
}};

// TI maps the three physical stage increments to the real/complex modes.
constexpr std::array<std::array<double, 3>, 3> kInverseTransform{{
    {4.3255798900631553510, 0.33919925181580986954,
     0.54177053993587487119},
    {-4.1787185915519047273, -0.32768282076106238708,
     0.47662355450055045196},
    {-0.50287263494578687595, 2.5719269498556054292,
     -0.59603920482822492497},
}};

constexpr double kErrorCoefficient1 = -10.048809399827415562;
constexpr double kErrorCoefficient2 = 1.3821427331607480805;
constexpr double kErrorCoefficient3 = -1.0 / 3.0;

[[nodiscard]] bool IsStrictlyPositiveAndFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] int RequirePositiveStateSize(RhsEvaluator& rhs) {
    const int state_size = rhs.continuous_state_size();
    if (state_size <= 0) {
        throw std::invalid_argument(
            "Radau5 core: RHS state size must be positive");
    }
    return state_size;
}

void CheckedIncrement(std::uint64_t& counter, const char* message) {
    if (counter == std::numeric_limits<std::uint64_t>::max()) {
        throw Failure(Failure::Reason::kNonFiniteInternalState, message);
    }
    ++counter;
}

[[nodiscard]] double Clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

class ScaledSumOfSquares final {
   public:
    void Add(double value) noexcept {
        const double magnitude = std::abs(value);
        if (magnitude == 0.0) return;
        if (!std::isfinite(magnitude)) {
            scale_ = magnitude;
            sum_of_squares_ = 1.0;
            return;
        }
        if (scale_ < magnitude) {
            const double ratio = scale_ / magnitude;
            sum_of_squares_ =
                1.0 + sum_of_squares_ * ratio * ratio;
            scale_ = magnitude;
        } else {
            const double ratio = magnitude / scale_;
            sum_of_squares_ += ratio * ratio;
        }
    }

    [[nodiscard]] double RootMeanSquare(double count) const noexcept {
        if (scale_ == 0.0) return 0.0;
        return scale_ * std::sqrt(sum_of_squares_ / count);
    }

   private:
    double scale_{};
    double sum_of_squares_{1.0};
};

struct RepresentablePerturbation final {
    double value{};
    double increment{};
};

[[nodiscard]] RepresentablePerturbation SelectRepresentablePerturbation(
    double baseline_value, double nominal_increment) {
    double perturbed_value = baseline_value + nominal_increment;
    if (!std::isfinite(perturbed_value) ||
        perturbed_value == baseline_value) {
        perturbed_value = std::nextafter(
            baseline_value, std::numeric_limits<double>::infinity());
    }
    if (!std::isfinite(perturbed_value) ||
        perturbed_value == baseline_value) {
        perturbed_value = std::nextafter(
            baseline_value, -std::numeric_limits<double>::infinity());
    }
    const double actual_increment = perturbed_value - baseline_value;
    if (!std::isfinite(perturbed_value) ||
        !std::isfinite(actual_increment) || actual_increment == 0.0) {
        throw Failure(
            Failure::Reason::kNonFiniteInternalState,
            "Radau5 core: no finite representable Jacobian perturbation "
            "exists");
    }
    return RepresentablePerturbation{perturbed_value, actual_increment};
}

enum class FactorizationStatus {
    kSuccess,
    kSingular,
    kNonFinite,
};

enum class LinearSetupStatus {
    kSuccess,
    kSingular,
    kNonFiniteShift,
    kNonFiniteMatrix,
    kNonFiniteFactorization,
};

[[nodiscard]] FactorizationStatus ClassifyRealFactorization(
    const Eigen::PartialPivLU<Eigen::MatrixXd>& factorization) {
    const auto& factors = factorization.matrixLU();
    const auto diagonal = factors.diagonal();
    if ((diagonal.array() == 0.0).any()) {
        return FactorizationStatus::kSingular;
    }
    if (!factors.allFinite()) return FactorizationStatus::kNonFinite;
    return FactorizationStatus::kSuccess;
}

[[nodiscard]] FactorizationStatus ClassifyComplexFactorization(
    const Eigen::PartialPivLU<Eigen::MatrixXcd>& factorization) {
    const auto& factors = factorization.matrixLU();
    const auto diagonal = factors.diagonal();
    for (Eigen::Index index = 0; index < diagonal.size(); ++index) {
        if (std::abs(diagonal[index]) == 0.0) {
            return FactorizationStatus::kSingular;
        }
    }
    for (Eigen::Index row = 0; row < factors.rows(); ++row) {
        for (Eigen::Index column = 0; column < factors.cols(); ++column) {
            const std::complex<double> value = factors(row, column);
            if (!std::isfinite(value.real()) ||
                !std::isfinite(value.imag())) {
                return FactorizationStatus::kNonFinite;
            }
        }
    }
    return FactorizationStatus::kSuccess;
}

[[nodiscard]] LinearSetupStatus PrepareShiftedLinearSystems(
    const Eigen::Ref<const Eigen::MatrixXd>& jacobian,
    double step_size_seconds,
    Eigen::MatrixXd* real_matrix,
    Eigen::MatrixXcd* complex_matrix,
    Eigen::PartialPivLU<Eigen::MatrixXd>* real_factorization,
    Eigen::PartialPivLU<Eigen::MatrixXcd>* complex_factorization) {
    const double real_shift = kRealEigenvalue / step_size_seconds;
    const std::complex<double> complex_shift(
        kComplexEigenvalueReal / step_size_seconds,
        kComplexEigenvalueImaginary / step_size_seconds);
    if (!std::isfinite(real_shift) ||
        !std::isfinite(complex_shift.real()) ||
        !std::isfinite(complex_shift.imag())) {
        return LinearSetupStatus::kNonFiniteShift;
    }

    *real_matrix = -jacobian;
    *complex_matrix = -jacobian.cast<std::complex<double>>();
    for (Eigen::Index index = 0; index < jacobian.rows(); ++index) {
        (*real_matrix)(index, index) += real_shift;
        (*complex_matrix)(index, index) += complex_shift;
    }
    if (!real_matrix->allFinite()) {
        return LinearSetupStatus::kNonFiniteMatrix;
    }
    for (Eigen::Index row = 0; row < complex_matrix->rows(); ++row) {
        for (Eigen::Index column = 0; column < complex_matrix->cols();
             ++column) {
            const std::complex<double> value =
                (*complex_matrix)(row, column);
            if (!std::isfinite(value.real()) ||
                !std::isfinite(value.imag())) {
                return LinearSetupStatus::kNonFiniteMatrix;
            }
        }
    }

    real_factorization->compute(*real_matrix);
    const FactorizationStatus real_status =
        ClassifyRealFactorization(*real_factorization);
    if (real_status == FactorizationStatus::kSingular) {
        return LinearSetupStatus::kSingular;
    }
    if (real_status == FactorizationStatus::kNonFinite) {
        return LinearSetupStatus::kNonFiniteFactorization;
    }
    complex_factorization->compute(*complex_matrix);
    const FactorizationStatus complex_status =
        ClassifyComplexFactorization(*complex_factorization);
    if (complex_status == FactorizationStatus::kSingular) {
        return LinearSetupStatus::kSingular;
    }
    if (complex_status == FactorizationStatus::kNonFinite) {
        return LinearSetupStatus::kNonFiniteFactorization;
    }
    return LinearSetupStatus::kSuccess;
}

[[nodiscard]] Eigen::VectorXd SolveRealShiftedSystem(
    const Eigen::PartialPivLU<Eigen::MatrixXd>& factorization,
    const Eigen::Ref<const Eigen::VectorXd>& right_hand_side) {
    return factorization.solve(right_hand_side);
}

[[nodiscard]] Eigen::VectorXcd SolveComplexShiftedSystem(
    const Eigen::PartialPivLU<Eigen::MatrixXcd>& factorization,
    const Eigen::Ref<const Eigen::VectorXcd>& right_hand_side) {
    return factorization.solve(right_hand_side);
}

}  // namespace

Failure::Failure(Reason reason, const char* message)
    : std::runtime_error(message), reason_(reason) {}

internal::NewtonCorrectionAssessment internal::AssessNewtonCorrection(
    int iteration,
    double correction_norm,
    double previous_correction_norm,
    double previous_ratio,
    double theta,
    double faccon,
    double newton_tolerance) {
    NewtonCorrectionAssessment result;
    result.theta = theta;
    result.previous_ratio = previous_ratio;
    result.faccon = faccon;
    if (iteration > 1 && iteration < kMaximumNewtonIterations) {
        const double ratio = correction_norm / previous_correction_norm;
        result.theta = iteration == 2
                           ? ratio
                           : std::sqrt(ratio * previous_ratio);
        result.previous_ratio = ratio;
        if (!std::isfinite(result.theta) || result.theta >= 0.99) {
            result.reject_trial = true;
            return result;
        }
        result.faccon = result.theta / (1.0 - result.theta);
        const double predicted =
            result.faccon * correction_norm *
            std::pow(result.theta,
                     kMaximumNewtonIterations - 1 - iteration) /
            newton_tolerance;
        if (!std::isfinite(predicted)) {
            throw Failure(
                Failure::Reason::kNonFiniteInternalState,
                "Radau5 core: Newton convergence prediction is non-finite");
        }
        if (predicted >= 1.0) {
            const double bounded_prediction =
                Clamp(predicted, 1.0e-4, 20.0);
            result.retry_multiplier =
                0.8 * std::pow(
                          bounded_prediction,
                          -1.0 /
                              (4.0 + kMaximumNewtonIterations - 1 -
                               iteration));
            result.reject_trial = true;
            return result;
        }
    }
    result.converges_after_application =
        result.faccon * correction_norm <= newton_tolerance;
    return result;
}

internal::ShiftedLinearSolveQualificationResult
internal::SolveShiftedLinearSystemsForQualification(
    const Eigen::Ref<const Eigen::MatrixXd>& jacobian,
    double step_size_seconds,
    const Eigen::Ref<const Eigen::VectorXd>& real_right_hand_side,
    const Eigen::Ref<const Eigen::VectorXcd>& complex_right_hand_side) {
    if (jacobian.rows() <= 0 || jacobian.rows() != jacobian.cols() ||
        real_right_hand_side.size() != jacobian.rows() ||
        complex_right_hand_side.size() != jacobian.rows() ||
        !jacobian.allFinite() || !IsStrictlyPositiveAndFinite(
                                    step_size_seconds)) {
        throw std::invalid_argument(
            "Radau5 shifted-system qualification input is invalid");
    }
    ShiftedLinearSolveQualificationResult result;
    Eigen::PartialPivLU<Eigen::MatrixXd> real_factorization;
    Eigen::PartialPivLU<Eigen::MatrixXcd> complex_factorization;
    const LinearSetupStatus status = PrepareShiftedLinearSystems(
        jacobian, step_size_seconds, &result.real_matrix,
        &result.complex_matrix, &real_factorization,
        &complex_factorization);
    if (status != LinearSetupStatus::kSuccess) {
        throw std::runtime_error(
            "Radau5 shifted-system qualification setup failed");
    }
    result.real_solution =
        SolveRealShiftedSystem(real_factorization, real_right_hand_side);
    result.complex_solution = SolveComplexShiftedSystem(
        complex_factorization, complex_right_hand_side);
    return result;
}

class Core::Implementation final {
   public:
    Implementation(RhsEvaluator& rhs,
                   double initial_time_seconds,
                   Eigen::VectorXd initial_continuous_state,
                   double relative_tolerance,
                   Eigen::VectorXd component_absolute_tolerances)
        : rhs_(&rhs),
          state_size_(RequirePositiveStateSize(rhs)),
          time_seconds_(initial_time_seconds),
          state_(std::move(initial_continuous_state)),
          absolute_tolerances_(
              std::move(component_absolute_tolerances)),
          baseline_derivatives_(state_size_),
          perturbed_state_(state_size_),
          perturbed_derivatives_(state_size_),
          stage_state_(state_size_),
          stage_rhs_1_(state_size_),
          stage_rhs_2_(state_size_),
          stage_rhs_3_(state_size_),
          stage_increment_1_(state_size_),
          stage_increment_2_(state_size_),
          stage_increment_3_(state_size_),
          transformed_increment_1_(state_size_),
          transformed_increment_2_(state_size_),
          transformed_increment_3_(state_size_),
          correction_1_(state_size_),
          correction_2_(state_size_),
          correction_3_(state_size_),
          complex_correction_(state_size_),
          error_vector_(state_size_),
          error_rhs_(state_size_),
          scale_(state_size_),
          jacobian_(state_size_, state_size_),
          real_matrix_(state_size_, state_size_),
          complex_matrix_(state_size_, state_size_),
          dense_start_state_(state_size_),
          dense_endpoint_state_(state_size_),
          dense_coefficient_1_(state_size_),
          dense_coefficient_2_(state_size_),
          dense_coefficient_3_(state_size_) {
        ValidateConstruction(relative_tolerance);
        TransformTolerances(relative_tolerance);
        ResetNumericalHistory();
    }

    [[nodiscard]] int state_size() const noexcept { return state_size_; }
    [[nodiscard]] double time_seconds() const noexcept {
        return time_seconds_;
    }
    [[nodiscard]] Statistics statistics() const noexcept {
        return statistics_;
    }

    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd> output) const {
        if (output.size() != state_size_) {
            throw std::invalid_argument(
                "Radau5 core: current-state output has the wrong size");
        }
        output = state_;
    }

    [[nodiscard]] InternalStep AdvanceOneAcceptedStepToward(
        double stop_time_seconds) {
        if (!std::isfinite(stop_time_seconds)) {
            throw std::invalid_argument(
                "Radau5 core: stop time must be finite");
        }
        if (stop_time_seconds < time_seconds_) {
            throw std::invalid_argument(
                "Radau5 core: stop time precedes the current endpoint");
        }
        if (poisoned_) {
            throw std::logic_error(
                "Radau5 core: a prior failure requires reinitialization");
        }
        if (stop_time_seconds == time_seconds_) {
            return InternalStep{time_seconds_, time_seconds_, true};
        }

        dense_interval_.reset();
        try {
            return AdvancePositiveLength(stop_time_seconds);
        } catch (...) {
            poisoned_ = true;
            dense_interval_.reset();
            throw;
        }
    }

    void Reinitialize(
        double committed_time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& committed_continuous_state) {
        if (!std::isfinite(committed_time_seconds)) {
            throw std::invalid_argument(
                "Radau5 core: committed time must be finite");
        }
        if (committed_continuous_state.size() != state_size_) {
            throw std::invalid_argument(
                "Radau5 core: committed state has the wrong size");
        }
        if (!committed_continuous_state.allFinite()) {
            throw std::invalid_argument(
                "Radau5 core: committed state must be finite");
        }

        Eigen::VectorXd candidate = committed_continuous_state;
        time_seconds_ = committed_time_seconds;
        state_ = std::move(candidate);
        statistics_ = {};
        ResetNumericalHistory();
    }

    void InvalidateLinearizationAfterNumericalRhsHistoryChange() noexcept {
        jacobian_available_ = false;
        jacobian_fresh_ = false;
        decomposition_available_ = false;
    }

    [[nodiscard]] std::optional<DenseOutputInterval> dense_interval()
        const noexcept {
        return dense_interval_;
    }

    void CopyDenseState(double time_seconds,
                        Eigen::Ref<Eigen::VectorXd> output) const {
        if (!std::isfinite(time_seconds)) {
            throw std::invalid_argument(
                "Radau5 core: dense-output time must be finite");
        }
        if (output.size() != state_size_) {
            throw std::invalid_argument(
                "Radau5 core: dense-state output has the wrong size");
        }
        if (!dense_interval_.has_value()) {
            throw std::logic_error(
                "Radau5 core: dense output is unavailable");
        }
        if (time_seconds < dense_interval_->start_time_seconds ||
            time_seconds > dense_interval_->end_time_seconds) {
            throw std::invalid_argument(
                "Radau5 core: dense-output time is outside the interval");
        }
        if (time_seconds == dense_interval_->start_time_seconds) {
            output = dense_start_state_;
            return;
        }
        if (time_seconds == dense_interval_->end_time_seconds) {
            output = dense_endpoint_state_;
            return;
        }

        const double s =
            (time_seconds - dense_interval_->end_time_seconds) /
            dense_step_size_seconds_;
        Eigen::VectorXd candidate =
            dense_endpoint_state_ +
            s * (dense_coefficient_1_ +
                 (s - kC2MinusOne) *
                     (dense_coefficient_2_ +
                      (s - kC1MinusOne) * dense_coefficient_3_));
        if (!candidate.allFinite()) {
            throw std::runtime_error(
                "Radau5 core: dense output produced a non-finite state");
        }
        output = std::move(candidate);
    }

    void SetSuggestedStepSizeForTesting(double step_size_seconds) {
        if (!IsStrictlyPositiveAndFinite(step_size_seconds)) {
            throw std::invalid_argument(
                "Radau5 core: test step size must be positive and finite");
        }
        if (poisoned_) {
            throw std::logic_error(
                "Radau5 core: a prior failure requires reinitialization");
        }
        suggested_step_size_seconds_ = step_size_seconds;
        decomposition_available_ = false;
    }

    void SetMaximumTrialsPerCallForTesting(int maximum_trials) {
        if (maximum_trials <= 0) {
            throw std::invalid_argument(
                "Radau5 core: test trial budget must be positive");
        }
        if (poisoned_) {
            throw std::logic_error(
                "Radau5 core: a prior failure requires reinitialization");
        }
        maximum_trials_per_call_ = maximum_trials;
    }

   private:
    enum class RhsPurpose { kOrdinary, kJacobian };

    enum class RetryCause {
        kNone,
        kRecoverableStageRhs,
        kRecoverableErrorCorrectionRhs,
        kNewtonConvergence,
        kErrorRejection,
        kSingularLinearSetup,
    };

    struct NewtonResult final {
        bool converged{};
        RetryCause retry_cause{RetryCause::kNone};
        double retry_multiplier{0.5};
        double theta{0.001};
        int iterations{};
    };

    struct ErrorEstimateResult final {
        RetryCause retry_cause{RetryCause::kNone};
        double error{};
    };

    [[nodiscard]] static bool IsRecoverableRetryCause(
        RetryCause cause) noexcept {
        return cause == RetryCause::kRecoverableStageRhs ||
               cause == RetryCause::kRecoverableErrorCorrectionRhs;
    }

    [[noreturn]] void ThrowTerminalFailure(
        Failure::Reason fallback_reason,
        const char* fallback_message) const {
        if (IsRecoverableRetryCause(pending_retry_cause_)) {
            throw Failure(
                Failure::Reason::kRecoverableRhsExhausted,
                "Radau5 core: a recoverable RHS retry could not execute "
                "another successful RHS evaluation");
        }
        throw Failure(fallback_reason, fallback_message);
    }

    void ValidateConstruction(double relative_tolerance) const {
        if (state_size_ <= 0) {
            throw std::invalid_argument(
                "Radau5 core: RHS state size must be positive");
        }
        if (!std::isfinite(time_seconds_)) {
            throw std::invalid_argument(
                "Radau5 core: initial time must be finite");
        }
        if (state_.size() != state_size_) {
            throw std::invalid_argument(
                "Radau5 core: initial state size does not match the RHS");
        }
        if (!state_.allFinite()) {
            throw std::invalid_argument(
                "Radau5 core: initial state must be finite");
        }
        if (!std::isfinite(relative_tolerance) ||
            !(relative_tolerance > 10.0 * kRoundingUnit)) {
            throw std::invalid_argument(
                "Radau5 core: relative tolerance must exceed 1e-15");
        }
        if (absolute_tolerances_.size() != state_size_) {
            throw std::invalid_argument(
                "Radau5 core: absolute-tolerance size does not match the "
                "RHS");
        }
        for (Eigen::Index index = 0; index < absolute_tolerances_.size();
             ++index) {
            if (!IsStrictlyPositiveAndFinite(absolute_tolerances_[index])) {
                throw std::invalid_argument(
                    "Radau5 core: absolute tolerances must be positive and "
                    "finite");
            }
        }
    }

    void TransformTolerances(double relative_tolerance) {
        transformed_relative_tolerance_ =
            0.1 * std::pow(relative_tolerance, 2.0 / 3.0);
        if (!IsStrictlyPositiveAndFinite(transformed_relative_tolerance_)) {
            throw std::invalid_argument(
                "Radau5 core: transformed relative tolerance is invalid");
        }
        for (Eigen::Index index = 0; index < absolute_tolerances_.size();
             ++index) {
            const double ratio =
                absolute_tolerances_[index] / relative_tolerance;
            if (!IsStrictlyPositiveAndFinite(ratio)) {
                throw std::invalid_argument(
                    "Radau5 core: absolute/relative tolerance ratio is "
                    "invalid");
            }
            const double transformed_absolute_tolerance =
                transformed_relative_tolerance_ * ratio;
            if (!IsStrictlyPositiveAndFinite(
                    transformed_absolute_tolerance)) {
                throw std::invalid_argument(
                    "Radau5 core: transformed absolute tolerance is "
                    "invalid");
            }
            absolute_tolerances_[index] = transformed_absolute_tolerance;
        }
        newton_tolerance_ = std::max(
            10.0 * kRoundingUnit / transformed_relative_tolerance_,
            std::min(0.03, std::sqrt(transformed_relative_tolerance_)));
        if (!IsStrictlyPositiveAndFinite(newton_tolerance_)) {
            throw std::invalid_argument(
                "Radau5 core: Newton tolerance is invalid");
        }
    }

    void ResetNumericalHistory() noexcept {
        suggested_step_size_seconds_ = kInitialStepSizeSeconds;
        faccon_ = 1.0;
        previous_accepted_step_size_seconds_ = 0.0;
        accepted_step_size_for_controller_ = 0.0;
        accepted_error_for_controller_ = 1.0;
        has_successful_step_ = false;
        extrapolation_available_ = false;
        rejection_pending_ = false;
        jacobian_available_ = false;
        jacobian_fresh_ = false;
        decomposition_available_ = false;
        jacobian_generation_ = 0;
        decomposition_jacobian_generation_ = 0;
        decomposition_step_size_seconds_ = 0.0;
        singular_setup_failure_count_ = 0;
        pending_retry_cause_ = RetryCause::kNone;
        dense_step_size_seconds_ = 0.0;
        dense_interval_.reset();
        poisoned_ = false;
    }

    [[nodiscard]] RhsEvaluationStatus EvaluateRhs(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives,
        RhsPurpose purpose) {
        if (purpose == RhsPurpose::kOrdinary) {
            CheckedIncrement(
                statistics_.right_hand_side_evaluation_count,
                "Radau5 core: ordinary RHS evaluation count overflowed");
        } else {
            CheckedIncrement(
                statistics_.linear_solver_right_hand_side_evaluation_count,
                "Radau5 core: Jacobian RHS evaluation count overflowed");
        }
        const RhsEvaluationStatus status =
            rhs_->Evaluate(time_seconds, state, derivatives);
        if (status == RhsEvaluationStatus::kSuccess &&
            IsRecoverableRetryCause(pending_retry_cause_)) {
            pending_retry_cause_ = RetryCause::kNone;
        }
        if (status == RhsEvaluationStatus::kSuccess &&
            !derivatives.allFinite()) {
            throw Failure(Failure::Reason::kNonFiniteRhs,
                          "Radau5 core: RHS returned a non-finite value");
        }
        return status;
    }

    [[noreturn]] void ThrowRhsFailure(RhsEvaluationStatus status,
                                      const char* recoverable_message,
                                      const char* fatal_message) const {
        if (status == RhsEvaluationStatus::kRecoverableFailure) {
            throw Failure(Failure::Reason::kRecoverableRhsExhausted,
                          recoverable_message);
        }
        throw Failure(Failure::Reason::kFatalRhs, fatal_message);
    }

    void UpdateScale() {
        scale_ = absolute_tolerances_ +
                 transformed_relative_tolerance_ * state_.cwiseAbs();
        for (Eigen::Index index = 0; index < scale_.size(); ++index) {
            if (!IsStrictlyPositiveAndFinite(scale_[index])) {
                throw Failure(
                    Failure::Reason::kNonFiniteInternalState,
                    "Radau5 core: an error weight is non-finite or zero");
            }
        }
    }

    void EvaluateAcceptedEndpointRhs() {
        const RhsEvaluationStatus status =
            EvaluateRhs(time_seconds_, state_, baseline_derivatives_,
                        RhsPurpose::kOrdinary);
        if (status != RhsEvaluationStatus::kSuccess) {
            ThrowRhsFailure(
                status,
                "Radau5 core: accepted-endpoint RHS was recoverable but "
                "cannot be changed by reducing the step",
                "Radau5 core: accepted-endpoint RHS failed fatally");
        }
    }

    void ComputeJacobian() {
        CheckedIncrement(statistics_.jacobian_evaluation_count,
                         "Radau5 core: Jacobian evaluation count overflowed");
        perturbed_state_ = state_;
        for (int column = 0; column < state_size_; ++column) {
            const double baseline_value = state_[column];
            double nominal_increment = std::sqrt(
                kRoundingUnit * std::max(1.0e-5, std::abs(baseline_value)));
            bool column_complete = false;
            for (int attempt = 0;
                 attempt <= kMaximumJacobianPerturbationRetries; ++attempt) {
                const RepresentablePerturbation perturbation =
                    SelectRepresentablePerturbation(baseline_value,
                                                    nominal_increment);
                perturbed_state_[column] = perturbation.value;
                const RhsEvaluationStatus status = EvaluateRhs(
                    time_seconds_, perturbed_state_, perturbed_derivatives_,
                    RhsPurpose::kJacobian);
                perturbed_state_[column] = baseline_value;
                if (status == RhsEvaluationStatus::kSuccess) {
                    jacobian_.col(column) =
                        (perturbed_derivatives_ - baseline_derivatives_) /
                        perturbation.increment;
                    column_complete = true;
                    break;
                }
                if (status == RhsEvaluationStatus::kFatalFailure) {
                    ThrowRhsFailure(
                        status, "",
                        "Radau5 core: Jacobian RHS failed fatally");
                }
                if (attempt == kMaximumJacobianPerturbationRetries) {
                    ThrowRhsFailure(
                        status,
                        "Radau5 core: Jacobian perturbation retries were "
                        "exhausted",
                        "");
                }
                nominal_increment *= 0.1;
            }
            if (!column_complete) {
                throw Failure(
                    Failure::Reason::kNonFiniteInternalState,
                    "Radau5 core: Jacobian column was not completed");
            }
        }
        if (!jacobian_.allFinite()) {
            throw Failure(Failure::Reason::kNonFiniteInternalState,
                          "Radau5 core: Jacobian is non-finite");
        }
        jacobian_available_ = true;
        jacobian_fresh_ = true;
        CheckedIncrement(jacobian_generation_,
                         "Radau5 core: Jacobian generation overflowed");
        decomposition_available_ = false;
    }

    [[nodiscard]] LinearSetupStatus PrepareLinearSystems(
        double step_size_seconds) {
        if (decomposition_available_ &&
            decomposition_jacobian_generation_ == jacobian_generation_ &&
            decomposition_step_size_seconds_ == step_size_seconds) {
            return LinearSetupStatus::kSuccess;
        }
        CheckedIncrement(statistics_.linear_solver_setup_count,
                         "Radau5 core: linear setup count overflowed");
        const LinearSetupStatus status = PrepareShiftedLinearSystems(
            jacobian_, step_size_seconds, &real_matrix_, &complex_matrix_,
            &real_factorization_, &complex_factorization_);
        if (status != LinearSetupStatus::kSuccess) return status;
        decomposition_available_ = true;
        decomposition_jacobian_generation_ = jacobian_generation_;
        decomposition_step_size_seconds_ = step_size_seconds;
        return LinearSetupStatus::kSuccess;
    }

    void InitializeStages(double step_size_seconds) {
        if (!extrapolation_available_) {
            stage_increment_1_.setZero();
            stage_increment_2_.setZero();
            stage_increment_3_.setZero();
        } else {
            const double ratio =
                step_size_seconds / previous_accepted_step_size_seconds_;
            const double q1 = kC1 * ratio;
            const double q2 = kC2 * ratio;
            const double q3 = ratio;
            stage_increment_1_ =
                q1 * (dense_coefficient_1_ +
                      (q1 - kC2MinusOne) *
                          (dense_coefficient_2_ +
                           (q1 - kC1MinusOne) * dense_coefficient_3_));
            stage_increment_2_ =
                q2 * (dense_coefficient_1_ +
                      (q2 - kC2MinusOne) *
                          (dense_coefficient_2_ +
                           (q2 - kC1MinusOne) * dense_coefficient_3_));
            stage_increment_3_ =
                q3 * (dense_coefficient_1_ +
                      (q3 - kC2MinusOne) *
                          (dense_coefficient_2_ +
                           (q3 - kC1MinusOne) * dense_coefficient_3_));
        }
        TransformPhysicalStagesToModes();
    }

    void TransformPhysicalStagesToModes() {
        transformed_increment_1_ =
            kInverseTransform[0][0] * stage_increment_1_ +
            kInverseTransform[0][1] * stage_increment_2_ +
            kInverseTransform[0][2] * stage_increment_3_;
        transformed_increment_2_ =
            kInverseTransform[1][0] * stage_increment_1_ +
            kInverseTransform[1][1] * stage_increment_2_ +
            kInverseTransform[1][2] * stage_increment_3_;
        transformed_increment_3_ =
            kInverseTransform[2][0] * stage_increment_1_ +
            kInverseTransform[2][1] * stage_increment_2_ +
            kInverseTransform[2][2] * stage_increment_3_;
    }

    void TransformModesToPhysicalStages() {
        stage_increment_1_ =
            kTransform[0][0] * transformed_increment_1_ +
            kTransform[0][1] * transformed_increment_2_ +
            kTransform[0][2] * transformed_increment_3_;
        stage_increment_2_ =
            kTransform[1][0] * transformed_increment_1_ +
            kTransform[1][1] * transformed_increment_2_ +
            kTransform[1][2] * transformed_increment_3_;
        stage_increment_3_ =
            kTransform[2][0] * transformed_increment_1_ +
            kTransform[2][1] * transformed_increment_2_ +
            kTransform[2][2] * transformed_increment_3_;
    }

    [[nodiscard]] NewtonResult RunNewton(double step_size_seconds,
                                         double trial_end_time_seconds) {
        InitializeStages(step_size_seconds);
        faccon_ = std::pow(std::max(faccon_, kRoundingUnit), 0.8);
        double theta = 0.001;
        double previous_correction_norm = kRoundingUnit;
        double previous_ratio = 0.0;
        const double real_shift = kRealEigenvalue / step_size_seconds;
        const double alpha_shift =
            kComplexEigenvalueReal / step_size_seconds;
        const double beta_shift =
            kComplexEigenvalueImaginary / step_size_seconds;

        for (int iteration = 1; iteration <= kMaximumNewtonIterations;
             ++iteration) {
            stage_state_ = state_ + stage_increment_1_;
            RhsEvaluationStatus status = EvaluateRhs(
                time_seconds_ + kC1 * step_size_seconds, stage_state_,
                stage_rhs_1_, RhsPurpose::kOrdinary);
            if (status == RhsEvaluationStatus::kRecoverableFailure) {
                return NewtonResult{false,
                                    RetryCause::kRecoverableStageRhs,
                                    0.5, theta,
                                    iteration - 1};
            }
            if (status == RhsEvaluationStatus::kFatalFailure) {
                ThrowRhsFailure(status, "",
                                "Radau5 core: stage RHS failed fatally");
            }

            stage_state_ = state_ + stage_increment_2_;
            status = EvaluateRhs(
                time_seconds_ + kC2 * step_size_seconds, stage_state_,
                stage_rhs_2_, RhsPurpose::kOrdinary);
            if (status == RhsEvaluationStatus::kRecoverableFailure) {
                return NewtonResult{false,
                                    RetryCause::kRecoverableStageRhs,
                                    0.5, theta,
                                    iteration - 1};
            }
            if (status == RhsEvaluationStatus::kFatalFailure) {
                ThrowRhsFailure(status, "",
                                "Radau5 core: stage RHS failed fatally");
            }

            stage_state_ = state_ + stage_increment_3_;
            status = EvaluateRhs(trial_end_time_seconds, stage_state_,
                                 stage_rhs_3_, RhsPurpose::kOrdinary);
            if (status == RhsEvaluationStatus::kRecoverableFailure) {
                return NewtonResult{false,
                                    RetryCause::kRecoverableStageRhs,
                                    0.5, theta,
                                    iteration - 1};
            }
            if (status == RhsEvaluationStatus::kFatalFailure) {
                ThrowRhsFailure(status, "",
                                "Radau5 core: stage RHS failed fatally");
            }

            correction_1_ =
                kInverseTransform[0][0] * stage_rhs_1_ +
                kInverseTransform[0][1] * stage_rhs_2_ +
                kInverseTransform[0][2] * stage_rhs_3_ -
                real_shift * transformed_increment_1_;
            correction_2_ =
                kInverseTransform[1][0] * stage_rhs_1_ +
                kInverseTransform[1][1] * stage_rhs_2_ +
                kInverseTransform[1][2] * stage_rhs_3_ -
                alpha_shift * transformed_increment_2_ +
                beta_shift * transformed_increment_3_;
            correction_3_ =
                kInverseTransform[2][0] * stage_rhs_1_ +
                kInverseTransform[2][1] * stage_rhs_2_ +
                kInverseTransform[2][2] * stage_rhs_3_ -
                alpha_shift * transformed_increment_3_ -
                beta_shift * transformed_increment_2_;

            CheckedIncrement(
                statistics_.nonlinear_solver_iteration_count,
                "Radau5 core: nonlinear iteration count overflowed");
            correction_1_ =
                SolveRealShiftedSystem(real_factorization_, correction_1_);
            for (int index = 0; index < state_size_; ++index) {
                complex_correction_[index] = std::complex<double>(
                    correction_2_[index], correction_3_[index]);
            }
            complex_correction_ = SolveComplexShiftedSystem(
                complex_factorization_, complex_correction_);
            for (int index = 0; index < state_size_; ++index) {
                correction_2_[index] = complex_correction_[index].real();
                correction_3_[index] = complex_correction_[index].imag();
            }
            if (!correction_1_.allFinite() ||
                !correction_2_.allFinite() ||
                !correction_3_.allFinite()) {
                throw Failure(
                    Failure::Reason::kNonFiniteInternalState,
                    "Radau5 core: Newton linear solve is non-finite");
            }
            ScaledSumOfSquares correction_sum;
            for (int index = 0; index < state_size_; ++index) {
                correction_sum.Add(correction_1_[index] / scale_[index]);
                correction_sum.Add(correction_2_[index] / scale_[index]);
                correction_sum.Add(correction_3_[index] / scale_[index]);
            }
            const double correction_norm =
                correction_sum.RootMeanSquare(3.0 * state_size_);
            if (!std::isfinite(correction_norm)) {
                throw Failure(
                    Failure::Reason::kNonFiniteInternalState,
                    "Radau5 core: Newton correction norm is non-finite");
            }

            // Preserve the official NEWT boundary literally. With the default
            // NIT=7, iteration 6 uses a zero prediction exponent, so finite
            // arithmetic normally makes that prediction the final accept-or-
            // reject boundary. The iteration-7 correction and loop-limit
            // guards remain for fidelity rather than being reinterpreted as a
            // different prediction range merely to make them test-reachable.
            const internal::NewtonCorrectionAssessment assessment =
                internal::AssessNewtonCorrection(
                    iteration, correction_norm, previous_correction_norm,
                    previous_ratio, theta, faccon_, newton_tolerance_);
            theta = assessment.theta;
            previous_ratio = assessment.previous_ratio;
            faccon_ = assessment.faccon;
            if (assessment.reject_trial) {
                return NewtonResult{false, RetryCause::kNewtonConvergence,
                                    assessment.retry_multiplier, theta,
                                    iteration};
            }

            previous_correction_norm =
                std::max(correction_norm, kRoundingUnit);
            transformed_increment_1_ += correction_1_;
            transformed_increment_2_ += correction_2_;
            transformed_increment_3_ += correction_3_;
            TransformModesToPhysicalStages();
            if (!stage_increment_1_.allFinite() ||
                !stage_increment_2_.allFinite() ||
                !stage_increment_3_.allFinite()) {
                throw Failure(
                    Failure::Reason::kNonFiniteInternalState,
                    "Radau5 core: Newton stage state is non-finite");
            }
            if (assessment.converges_after_application) {
                return NewtonResult{true, RetryCause::kNone, 1.0, theta,
                                    iteration};
            }
        }
        return NewtonResult{false, RetryCause::kNewtonConvergence, 0.5, theta,
                            kMaximumNewtonIterations};
    }

    [[nodiscard]] double WeightedRootMeanSquare(
        const Eigen::Ref<const Eigen::VectorXd>& vector) const {
        ScaledSumOfSquares error_sum;
        for (int index = 0; index < state_size_; ++index) {
            error_sum.Add(vector[index] / scale_[index]);
        }
        const double result =
            std::max(error_sum.RootMeanSquare(state_size_), 1.0e-10);
        if (!std::isfinite(result)) {
            throw Failure(Failure::Reason::kNonFiniteInternalState,
                          "Radau5 core: error estimate is non-finite");
        }
        return result;
    }

    [[nodiscard]] ErrorEstimateResult EstimateError(
        double step_size_seconds) {
        const Eigen::VectorXd defect =
            (kErrorCoefficient1 * stage_increment_1_ +
             kErrorCoefficient2 * stage_increment_2_ +
             kErrorCoefficient3 * stage_increment_3_) /
            step_size_seconds;
        error_rhs_ = defect + baseline_derivatives_;
        error_vector_ =
            SolveRealShiftedSystem(real_factorization_, error_rhs_);
        if (!error_vector_.allFinite()) {
            throw Failure(Failure::Reason::kNonFiniteInternalState,
                          "Radau5 core: error linear solve is non-finite");
        }
        double error = WeightedRootMeanSquare(error_vector_);
        if (error < 1.0 ||
            (has_successful_step_ && !rejection_pending_)) {
            return ErrorEstimateResult{RetryCause::kNone, error};
        }

        stage_state_ = state_ + error_vector_;
        const RhsEvaluationStatus status =
            EvaluateRhs(time_seconds_, stage_state_, stage_rhs_1_,
                        RhsPurpose::kOrdinary);
        if (status == RhsEvaluationStatus::kRecoverableFailure) {
            return ErrorEstimateResult{
                RetryCause::kRecoverableErrorCorrectionRhs, error};
        }
        if (status == RhsEvaluationStatus::kFatalFailure) {
            ThrowRhsFailure(
                status, "",
                "Radau5 core: error-estimate RHS failed fatally");
        }
        error_rhs_ = defect + stage_rhs_1_;
        error_vector_ =
            SolveRealShiftedSystem(real_factorization_, error_rhs_);
        if (!error_vector_.allFinite()) {
            throw Failure(Failure::Reason::kNonFiniteInternalState,
                          "Radau5 core: corrected error solve is non-finite");
        }
        error = WeightedRootMeanSquare(error_vector_);
        return ErrorEstimateResult{RetryCause::kNone, error};
    }

    void PrepareRetry(double next_step_size_seconds) {
        rejection_pending_ = true;
        decomposition_available_ = false;
        if (!jacobian_fresh_) {
            jacobian_available_ = false;
        }
        suggested_step_size_seconds_ = next_step_size_seconds;
    }

    [[nodiscard]] double OrdinaryControllerQuotient(
        double error, int newton_iterations) const {
        const double safety = std::min(
            0.9, 0.9 * (1.0 + 2.0 * kMaximumNewtonIterations) /
                     (newton_iterations +
                      2.0 * kMaximumNewtonIterations));
        return Clamp(std::pow(error, 0.25) / safety, 1.0 / 8.0, 5.0);
    }

    [[nodiscard]] double ControllerStepSizeFromQuotient(
        double step_size_seconds, double quotient) const {
        const double next = step_size_seconds / quotient;
        if (!IsStrictlyPositiveAndFinite(next)) {
            throw Failure(Failure::Reason::kNonFiniteInternalState,
                          "Radau5 core: controller step is invalid");
        }
        return next;
    }

    [[nodiscard]] double OrdinaryControllerStepSize(
        double step_size_seconds, double error, int newton_iterations) const {
        return ControllerStepSizeFromQuotient(
            step_size_seconds,
            OrdinaryControllerQuotient(error, newton_iterations));
    }

    [[nodiscard]] double AcceptedControllerStepSize(
        double step_size_seconds, double error, int newton_iterations) const {
        double quotient = OrdinaryControllerQuotient(error,
                                                      newton_iterations);
        if (has_successful_step_) {
            double gustafsson =
                (accepted_step_size_for_controller_ / step_size_seconds) *
                std::pow(error * error /
                             accepted_error_for_controller_,
                         0.25) /
                0.9;
            gustafsson = Clamp(gustafsson, 1.0 / 8.0, 5.0);
            quotient = std::max(quotient, gustafsson);
        }
        return ControllerStepSizeFromQuotient(step_size_seconds, quotient);
    }

    void BuildDenseCoefficients(const Eigen::VectorXd& endpoint_state) {
        dense_start_state_ = state_;
        dense_endpoint_state_ = endpoint_state;
        dense_coefficient_1_ =
            (stage_increment_2_ - stage_increment_3_) / kC2MinusOne;
        const Eigen::VectorXd difference =
            (stage_increment_1_ - stage_increment_2_) / kC1MinusC2;
        const Eigen::VectorXd third_auxiliary =
            (difference - stage_increment_1_ / kC1) / kC2;
        dense_coefficient_2_ =
            (difference - dense_coefficient_1_) / kC1MinusOne;
        dense_coefficient_3_ = dense_coefficient_2_ - third_auxiliary;
        if (!dense_start_state_.allFinite() ||
            !dense_endpoint_state_.allFinite() ||
            !dense_coefficient_1_.allFinite() ||
            !dense_coefficient_2_.allFinite() ||
            !dense_coefficient_3_.allFinite()) {
            throw Failure(Failure::Reason::kNonFiniteInternalState,
                          "Radau5 core: dense coefficients are non-finite");
        }
    }

    [[nodiscard]] InternalStep AcceptTrial(
        double step_begin_time_seconds,
        double step_size_seconds,
        double step_end_time_seconds,
        double stop_time_seconds,
        double error,
        const NewtonResult& newton) {
        Eigen::VectorXd endpoint_state = state_ + stage_increment_3_;
        if (!endpoint_state.allFinite() ||
            !std::isfinite(step_end_time_seconds) ||
            !(step_end_time_seconds > step_begin_time_seconds)) {
            throw Failure(Failure::Reason::kNonFiniteInternalState,
                          "Radau5 core: accepted endpoint is invalid");
        }
        double next_step_size = AcceptedControllerStepSize(
            step_size_seconds, error, newton.iterations);
        if (rejection_pending_) {
            next_step_size = std::min(next_step_size, step_size_seconds);
        }
        const double ratio = next_step_size / step_size_seconds;
        const bool keep_current_step =
            newton.theta <= 0.001 && ratio >= 1.0 && ratio <= 1.2;
        if (keep_current_step) {
            next_step_size = step_size_seconds;
        }

        BuildDenseCoefficients(endpoint_state);
        CheckedIncrement(
            statistics_.successful_internal_step_count,
            "Radau5 core: successful internal-step count overflowed");
        state_ = std::move(endpoint_state);
        time_seconds_ = step_end_time_seconds;
        dense_step_size_seconds_ = step_size_seconds;
        dense_interval_ =
            DenseOutputInterval{step_begin_time_seconds,
                                step_end_time_seconds};
        accepted_step_size_for_controller_ = step_size_seconds;
        accepted_error_for_controller_ = std::max(1.0e-2, error);
        previous_accepted_step_size_seconds_ = step_size_seconds;
        suggested_step_size_seconds_ = next_step_size;
        has_successful_step_ = true;
        extrapolation_available_ = true;
        rejection_pending_ = false;
        pending_retry_cause_ = RetryCause::kNone;
        jacobian_fresh_ = false;
        if (newton.theta > 0.001) {
            jacobian_available_ = false;
            decomposition_available_ = false;
        } else if (next_step_size != step_size_seconds) {
            decomposition_available_ = false;
        }
        return InternalStep{step_begin_time_seconds, step_end_time_seconds,
                            step_end_time_seconds == stop_time_seconds};
    }

    [[nodiscard]] InternalStep AdvancePositiveLength(
        double stop_time_seconds) {
        const double step_begin_time_seconds = time_seconds_;
        const double remaining = stop_time_seconds - time_seconds_;
        double step_size_seconds =
            std::min(suggested_step_size_seconds_, remaining);
        bool first_trial_stop_snap_available = true;
        bool baseline_available = false;
        int trial_count = 0;

        while (true) {
            if (trial_count >= maximum_trials_per_call_) {
                ThrowTerminalFailure(
                    Failure::Reason::kTrialBudgetExceeded,
                    "Radau5 core: trial budget was exceeded");
            }
            ++trial_count;
            const bool may_snap_first_trial =
                first_trial_stop_snap_available;
            first_trial_stop_snap_available = false;
            if (may_snap_first_trial &&
                step_size_seconds >= remaining / 1.0001) {
                step_size_seconds = remaining;
            } else {
                step_size_seconds = std::min(step_size_seconds, remaining);
            }
            const bool clamped_to_stop = step_size_seconds == remaining;
            const double trial_end_time_seconds =
                clamped_to_stop ? stop_time_seconds
                                : time_seconds_ + step_size_seconds;
            if (!IsStrictlyPositiveAndFinite(step_size_seconds) ||
                !(time_seconds_ + step_size_seconds > time_seconds_) ||
                !(0.1 * std::abs(step_size_seconds) >
                  std::abs(time_seconds_) * kRoundingUnit)) {
                ThrowTerminalFailure(
                    Failure::Reason::kStepTooSmall,
                    "Radau5 core: step size became too small");
            }
            UpdateScale();
            if (!baseline_available) {
                EvaluateAcceptedEndpointRhs();
                baseline_available = true;
            }
            if (!jacobian_available_) {
                ComputeJacobian();
            }
            const LinearSetupStatus setup_status =
                PrepareLinearSystems(step_size_seconds);
            if (setup_status == LinearSetupStatus::kNonFiniteShift) {
                ThrowTerminalFailure(
                    Failure::Reason::kStepTooSmall,
                    "Radau5 core: the shifted-system scale is not "
                    "representable at this step size");
            }
            if (setup_status == LinearSetupStatus::kNonFiniteMatrix ||
                setup_status ==
                    LinearSetupStatus::kNonFiniteFactorization) {
                throw Failure(
                    Failure::Reason::kNonFiniteLinearSystem,
                    "Radau5 core: shifted-system formation or "
                    "factorization produced a non-finite value");
            }
            if (setup_status == LinearSetupStatus::kSingular) {
                pending_retry_cause_ = RetryCause::kSingularLinearSetup;
                CheckedIncrement(
                    statistics_
                        .nonlinear_solver_convergence_failure_count,
                    "Radau5 core: nonlinear convergence-failure count "
                    "overflowed");
                ++singular_setup_failure_count_;
                if (singular_setup_failure_count_ >=
                    kMaximumSingularSetups) {
                    throw Failure(
                        Failure::Reason::kRepeatedlySingular,
                        "Radau5 core: linear systems were repeatedly "
                        "singular");
                }
                PrepareRetry(0.5 * step_size_seconds);
                step_size_seconds = suggested_step_size_seconds_;
                continue;
            }

            const NewtonResult newton =
                RunNewton(step_size_seconds, trial_end_time_seconds);
            if (!newton.converged) {
                pending_retry_cause_ = newton.retry_cause;
                if (newton.retry_cause == RetryCause::kNewtonConvergence) {
                    CheckedIncrement(
                        statistics_
                            .nonlinear_solver_convergence_failure_count,
                        "Radau5 core: nonlinear convergence-failure count "
                        "overflowed");
                }
                PrepareRetry(step_size_seconds * newton.retry_multiplier);
                step_size_seconds = suggested_step_size_seconds_;
                continue;
            }

            const ErrorEstimateResult estimate =
                EstimateError(step_size_seconds);
            if (estimate.retry_cause ==
                RetryCause::kRecoverableErrorCorrectionRhs) {
                pending_retry_cause_ = estimate.retry_cause;
                PrepareRetry(0.5 * step_size_seconds);
                step_size_seconds = suggested_step_size_seconds_;
                continue;
            }
            if (estimate.error >= 1.0) {
                pending_retry_cause_ = RetryCause::kErrorRejection;
                CheckedIncrement(
                    statistics_.error_test_failure_count,
                    "Radau5 core: error-test failure count overflowed");
                const double retry_step =
                    has_successful_step_
                        ? OrdinaryControllerStepSize(
                              step_size_seconds, estimate.error,
                              newton.iterations)
                        : 0.1 * step_size_seconds;
                PrepareRetry(retry_step);
                step_size_seconds = suggested_step_size_seconds_;
                continue;
            }
            return AcceptTrial(step_begin_time_seconds, step_size_seconds,
                               trial_end_time_seconds, stop_time_seconds,
                               estimate.error, newton);
        }
    }

    RhsEvaluator* rhs_;
    int state_size_;
    double time_seconds_;
    Eigen::VectorXd state_;
    Eigen::VectorXd absolute_tolerances_;
    double transformed_relative_tolerance_{};
    double newton_tolerance_{};

    Eigen::VectorXd baseline_derivatives_;
    Eigen::VectorXd perturbed_state_;
    Eigen::VectorXd perturbed_derivatives_;
    Eigen::VectorXd stage_state_;
    Eigen::VectorXd stage_rhs_1_;
    Eigen::VectorXd stage_rhs_2_;
    Eigen::VectorXd stage_rhs_3_;
    Eigen::VectorXd stage_increment_1_;
    Eigen::VectorXd stage_increment_2_;
    Eigen::VectorXd stage_increment_3_;
    Eigen::VectorXd transformed_increment_1_;
    Eigen::VectorXd transformed_increment_2_;
    Eigen::VectorXd transformed_increment_3_;
    Eigen::VectorXd correction_1_;
    Eigen::VectorXd correction_2_;
    Eigen::VectorXd correction_3_;
    Eigen::VectorXcd complex_correction_;
    Eigen::VectorXd error_vector_;
    Eigen::VectorXd error_rhs_;
    Eigen::VectorXd scale_;

    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd real_matrix_;
    Eigen::MatrixXcd complex_matrix_;
    Eigen::PartialPivLU<Eigen::MatrixXd> real_factorization_;
    Eigen::PartialPivLU<Eigen::MatrixXcd> complex_factorization_;

    Eigen::VectorXd dense_start_state_;
    Eigen::VectorXd dense_endpoint_state_;
    Eigen::VectorXd dense_coefficient_1_;
    Eigen::VectorXd dense_coefficient_2_;
    Eigen::VectorXd dense_coefficient_3_;
    double dense_step_size_seconds_{};
    std::optional<DenseOutputInterval> dense_interval_;

    Statistics statistics_{};
    double suggested_step_size_seconds_{kInitialStepSizeSeconds};
    double faccon_{1.0};
    double previous_accepted_step_size_seconds_{};
    double accepted_step_size_for_controller_{};
    double accepted_error_for_controller_{1.0};
    bool has_successful_step_{};
    bool extrapolation_available_{};
    bool rejection_pending_{};
    bool jacobian_available_{};
    bool jacobian_fresh_{};
    bool decomposition_available_{};
    std::uint64_t jacobian_generation_{};
    std::uint64_t decomposition_jacobian_generation_{};
    double decomposition_step_size_seconds_{};
    int singular_setup_failure_count_{};
    RetryCause pending_retry_cause_{RetryCause::kNone};
    int maximum_trials_per_call_{kMaximumTrialsPerCall};
    bool poisoned_{};
};

Core::Core(RhsEvaluator& rhs,
           double initial_time_seconds,
           Eigen::VectorXd initial_continuous_state,
           double relative_tolerance,
           Eigen::VectorXd component_absolute_tolerances)
    : implementation_(std::make_unique<Implementation>(
          rhs, initial_time_seconds, std::move(initial_continuous_state),
          relative_tolerance, std::move(component_absolute_tolerances))) {}

Core::~Core() = default;

int Core::continuous_state_size() const noexcept {
    return implementation_->state_size();
}

double Core::current_time_seconds() const noexcept {
    return implementation_->time_seconds();
}

Statistics Core::statistics() const noexcept {
    return implementation_->statistics();
}

void Core::CopyCurrentState(Eigen::Ref<Eigen::VectorXd> output) const {
    implementation_->CopyCurrentState(output);
}

InternalStep Core::AdvanceOneAcceptedStepToward(double stop_time_seconds) {
    return implementation_->AdvanceOneAcceptedStepToward(stop_time_seconds);
}

void Core::Reinitialize(
    double committed_time_seconds,
    const Eigen::Ref<const Eigen::VectorXd>& committed_continuous_state) {
    implementation_->Reinitialize(committed_time_seconds,
                                  committed_continuous_state);
}

void Core::InvalidateLinearizationAfterNumericalRhsHistoryChange() noexcept {
    implementation_
        ->InvalidateLinearizationAfterNumericalRhsHistoryChange();
}

std::optional<DenseOutputInterval> Core::dense_output_interval() const
    noexcept {
    return implementation_->dense_interval();
}

void Core::CopyDenseState(double time_seconds,
                          Eigen::Ref<Eigen::VectorXd> output) const {
    implementation_->CopyDenseState(time_seconds, output);
}

void Core::SetSuggestedStepSizeForTesting(double step_size_seconds) {
    implementation_->SetSuggestedStepSizeForTesting(step_size_seconds);
}

void Core::SetMaximumTrialsPerCallForTesting(int maximum_trials) {
    implementation_->SetMaximumTrialsPerCallForTesting(maximum_trials);
}

}  // namespace orvd::radau5
