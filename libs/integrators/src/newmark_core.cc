#include "newmark_core.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "coordinate_core_state.h"

namespace orvd::integrators::internal {
namespace {

void ValidateScale(const Eigen::VectorXd& scale, int size, const char* name) {
    if (scale.size() != size || !scale.allFinite() ||
        (scale.array() <= 0.0).any()) {
        throw std::invalid_argument(std::string("Newmark: invalid ") + name);
    }
}

NewmarkCoreConfiguration ValidateConfiguration(
    CoordinateSecondOrderProblem& problem, NewmarkCoreConfiguration configuration) {
    const int nq = problem.coordinate_size();
    const int nz = problem.internal_state_size();
    if (nq <= 0 || nz < 0 || nq > std::numeric_limits<int>::max() - nz) {
        throw std::invalid_argument("Newmark: invalid problem dimensions");
    }
    if (!std::isfinite(configuration.step_size_seconds) ||
        configuration.step_size_seconds <= 0.0) {
        throw std::invalid_argument("Newmark: invalid nominal step size");
    }
    const auto& solver = configuration.nonlinear_solver;
    if (solver.maximum_iterations <= 0) {
        throw std::invalid_argument("Newmark: iteration limit must be positive");
    }
    ValidateScale(solver.position_correction_scales, nq, "position scales");
    ValidateScale(solver.velocity_correction_scales, nq, "velocity scales");
    ValidateScale(solver.internal_state_correction_scales, nz,
                  "internal correction scales");
    ValidateScale(solver.acceleration_residual_scales, nq,
                  "acceleration residual scales");
    ValidateScale(solver.internal_state_residual_scales, nz,
                  "internal residual scales");
    ValidateScale(solver.unknown_reference_scales, nq + nz,
                  "unknown reference scales");
    return configuration;
}

double ScaledMaximum(const Eigen::VectorXd& value,
                     const Eigen::VectorXd& scales) {
    double norm = 0.0;
    for (Eigen::Index i = 0; i < value.size(); ++i) {
        norm = std::max(norm, std::abs(value[i]) / scales[i]);
    }
    return norm;
}

// Positive forward difference where representable; at the largest finite
// value use the adjacent finite value on the other side instead of infinity.
double PerturbedValue(double value, double reference) {
    const double nominal = std::sqrt(std::numeric_limits<double>::epsilon()) *
                           std::max(std::abs(value), reference);
    double perturbed = value + nominal;
    if (!std::isfinite(perturbed)) {
        perturbed = std::nextafter(value, -std::numeric_limits<double>::infinity());
    } else if (perturbed == value) {
        perturbed = std::nextafter(value, std::numeric_limits<double>::infinity());
    }
    const double increment = perturbed - value;
    if (!std::isfinite(perturbed) || !std::isfinite(increment) ||
        increment == 0.0) {
        throw CoordinateIntegrationFailure(
            CoordinateIntegrationFailure::Reason::kNonFiniteLinearSystem,
            "Newmark: no finite representable Jacobian perturbation");
    }
    return perturbed;
}

}  // namespace

class NewmarkCore::Implementation final {
   public:
    Implementation(CoordinateSecondOrderProblem& problem,
                   NewmarkCoreConfiguration configuration,
                   const CoordinateState& initial)
        : configuration_(ValidateConfiguration(problem, std::move(configuration))),
          state_(problem, configuration_.step_size_seconds, initial, 1),
          unknown_(state_.nq_ + state_.nz_),
          perturbed_unknown_(unknown_.size()),
          residual_(unknown_.size()),
          perturbed_residual_(unknown_.size()),
          residual_scales_(unknown_.size()),
          correction_(unknown_.size()),
          scaled_right_hand_side_(unknown_.size()),
          scaled_correction_(unknown_.size()),
          jacobian_(unknown_.size(), unknown_.size()),
          scaled_jacobian_(unknown_.size(), unknown_.size()),
          factorization_(unknown_.size(), unknown_.size()),
          perturbed_state_(initial),
          perturbed_b_(state_.nq_),
          perturbed_g_(state_.nz_) {
        residual_scales_.head(state_.nq_) =
            configuration_.nonlinear_solver.acceleration_residual_scales;
        residual_scales_.tail(state_.nz_) =
            configuration_.nonlinear_solver.internal_state_residual_scales;
    }

    void Reinitialize(const CoordinateState& initial, NewmarkCoreConfiguration configuration) {
        auto prepared = ValidateConfiguration(state_.problem_, std::move(configuration));
        if (prepared.step_size_seconds != configuration_.step_size_seconds) {
            throw std::invalid_argument("Newmark: reinitialization cannot change the nominal step size");
        }
        // Allocate and validate before invoking any callback. The assignments
        // after successful state initialization use only already owned storage.
        Eigen::VectorXd prepared_residual_scales(residual_scales_.size());
        prepared_residual_scales.head(state_.nq_) =
            prepared.nonlinear_solver.acceleration_residual_scales;
        prepared_residual_scales.tail(state_.nz_) =
            prepared.nonlinear_solver.internal_state_residual_scales;
        state_.Reinitialize(initial);
        configuration_ = std::move(prepared);
        residual_scales_.swap(prepared_residual_scales);
    }

    void Advance(double h, std::optional<double> endpoint = std::nullopt) {
        state_.ValidateStepSize(h);
        if (endpoint.has_value()) state_.ValidateExplicitEndpoint(h, *endpoint);
        try {
            const double end = endpoint.has_value() ? *endpoint : state_.StepEnd(h);
            Solve(h, end);
            if (state_.ProjectCandidate()) {
                state_.Evaluate(state_.candidate_, state_.candidate_b_,
                                state_.candidate_g_, false);
            }
            state_.CommitCandidate();
        } catch (...) {
            state_.requires_reinitialization_ = true;
            throw;
        }
    }

    void Residual(const Eigen::VectorXd& unknown, double h, double end,
                  CoordinateState& trial, Eigen::VectorXd& b,
                  Eigen::VectorXd& g, Eigen::VectorXd& residual,
                  bool jacobian_evaluation) {
        const auto& accepted = state_.accepted_;
        trial.time_seconds = end;
        trial.q = accepted.q + h * accepted.s +
                  (0.25 * h * h) * (state_.b_ + unknown.head(state_.nq_));
        trial.s = accepted.s +
                  (0.5 * h) * (state_.b_ + unknown.head(state_.nq_));
        trial.z = unknown.tail(state_.nz_);
        state_.Evaluate(trial, b, g, jacobian_evaluation);
        residual.head(state_.nq_) = unknown.head(state_.nq_) - b;
        residual.tail(state_.nz_) =
            trial.z - accepted.z - (0.5 * h) * (state_.g_ + g);
        if (!residual.allFinite()) {
            Fail(CoordinateIntegrationFailure::Reason::kNonFiniteLinearSystem,
                 "Newmark: non-finite endpoint residual");
        }
    }

    [[noreturn]] void Fail(CoordinateIntegrationFailure::Reason reason,
                          const char* message) {
        ++state_.statistics_.nonlinear_solver_convergence_failure_count;
        throw CoordinateIntegrationFailure(reason, message);
    }

    [[nodiscard]] double CorrectionNorm(double h) const {
        const auto& solver = configuration_.nonlinear_solver;
        double norm = 0.0;
        for (int i = 0; i < state_.nq_; ++i) {
            norm = std::max(norm, std::abs((0.25 * h * h) * correction_[i]) /
                                      solver.position_correction_scales[i]);
            norm = std::max(norm, std::abs((0.5 * h) * correction_[i]) /
                                      solver.velocity_correction_scales[i]);
        }
        for (int i = 0; i < state_.nz_; ++i) {
            norm = std::max(norm, std::abs(correction_[state_.nq_ + i]) /
                                      solver.internal_state_correction_scales[i]);
        }
        return norm;
    }

    void Solve(double h, double end) {
        const auto& solver = configuration_.nonlinear_solver;
        unknown_.head(state_.nq_) = state_.b_;
        unknown_.tail(state_.nz_) = state_.accepted_.z + h * state_.g_;
        Residual(unknown_, h, end, state_.candidate_, state_.candidate_b_,
                 state_.candidate_g_, residual_, false);
        // The unchanged initial guess has zero correction.
        if (ScaledMaximum(residual_, residual_scales_) <= 1.0) {
            return;
        }

        for (int iteration = 0; iteration < solver.maximum_iterations;
             ++iteration) {
            ++state_.statistics_.nonlinear_solver_iteration_count;
            ++state_.statistics_.jacobian_evaluation_count;
            for (Eigen::Index j = 0; j < unknown_.size(); ++j) {
                perturbed_unknown_ = unknown_;
                perturbed_unknown_[j] =
                    PerturbedValue(unknown_[j], solver.unknown_reference_scales[j]);
                const double increment = perturbed_unknown_[j] - unknown_[j];
                Residual(perturbed_unknown_, h, end, perturbed_state_,
                         perturbed_b_, perturbed_g_, perturbed_residual_, true);
                jacobian_.col(j) = (perturbed_residual_ - residual_) / increment;
            }
            for (Eigen::Index i = 0; i < unknown_.size(); ++i) {
                scaled_right_hand_side_[i] = -residual_[i] / residual_scales_[i];
                for (Eigen::Index j = 0; j < unknown_.size(); ++j) {
                    scaled_jacobian_(i, j) =
                        (jacobian_(i, j) / residual_scales_[i]) *
                        solver.unknown_reference_scales[j];
                }
            }
            if (!scaled_jacobian_.allFinite() ||
                !scaled_right_hand_side_.allFinite()) {
                Fail(CoordinateIntegrationFailure::Reason::kNonFiniteLinearSystem,
                     "Newmark: non-finite scaled Newton system");
            }
            ++state_.statistics_.linear_solver_setup_count;
            factorization_.compute(scaled_jacobian_);
            if (!factorization_.matrixLU().allFinite()) {
                Fail(CoordinateIntegrationFailure::Reason::kNonFiniteLinearSystem,
                     "Newmark: non-finite Newton factorization");
            }
            if (!factorization_.isInvertible()) {
                Fail(CoordinateIntegrationFailure::Reason::kSingularJacobian,
                     "Newmark: singular endpoint Jacobian");
            }
            scaled_correction_ = factorization_.solve(scaled_right_hand_side_);
            correction_ = scaled_correction_.cwiseProduct(
                solver.unknown_reference_scales);
            if (!correction_.allFinite()) {
                Fail(CoordinateIntegrationFailure::Reason::kNonFiniteLinearSystem,
                     "Newmark: non-finite Newton correction");
            }
            unknown_ += correction_;
            Residual(unknown_, h, end, state_.candidate_, state_.candidate_b_,
                     state_.candidate_g_, residual_, false);
            if (ScaledMaximum(residual_, residual_scales_) <= 1.0 &&
                CorrectionNorm(h) <= 1.0) {
                return;
            }
        }
        Fail(CoordinateIntegrationFailure::Reason::kNonlinearConvergenceFailure,
             "Newmark: endpoint Newton iteration limit reached");
    }

    NewmarkCoreConfiguration configuration_;
    CoordinateCoreState state_;
    Eigen::VectorXd unknown_;
    Eigen::VectorXd perturbed_unknown_;
    Eigen::VectorXd residual_;
    Eigen::VectorXd perturbed_residual_;
    Eigen::VectorXd residual_scales_;
    Eigen::VectorXd correction_;
    Eigen::VectorXd scaled_right_hand_side_;
    Eigen::VectorXd scaled_correction_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd scaled_jacobian_;
    Eigen::FullPivLU<Eigen::MatrixXd> factorization_;
    CoordinateState perturbed_state_;
    Eigen::VectorXd perturbed_b_;
    Eigen::VectorXd perturbed_g_;
};

NewmarkCore::NewmarkCore(CoordinateSecondOrderProblem& problem,
                         NewmarkCoreConfiguration configuration,
                         const CoordinateState& initial_state)
    : implementation_(std::make_unique<Implementation>(
          problem, std::move(configuration), initial_state)) {}

NewmarkCore::~NewmarkCore() = default;

double NewmarkCore::current_time_seconds() const {
    return implementation_->state_.accepted_.time_seconds;
}

ContinuousStateIntegrationStatistics NewmarkCore::integration_statistics() const {
    return implementation_->state_.statistics_;
}

CoordinateIntegrationDiagnostics NewmarkCore::diagnostics() const {
    return implementation_->state_.diagnostics_;
}

void NewmarkCore::CopyCurrentState(Eigen::Ref<Eigen::VectorXd> q,
                                  Eigen::Ref<Eigen::VectorXd> s,
                                  Eigen::Ref<Eigen::VectorXd> z) const {
    implementation_->state_.CopyCurrentState(q, s, z);
}

void NewmarkCore::AdvanceOneStep() {
    implementation_->Advance(implementation_->configuration_.step_size_seconds);
}

void NewmarkCore::AdvanceOneStep(double step_size_seconds) {
    implementation_->Advance(step_size_seconds);
}

void NewmarkCore::AdvanceOneStep(double step_size_seconds, double endpoint_time_seconds) {
    implementation_->Advance(step_size_seconds, endpoint_time_seconds);
}

void NewmarkCore::Reinitialize(const CoordinateState& initial_state) {
    implementation_->state_.Reinitialize(initial_state);
}

void NewmarkCore::Reinitialize(const CoordinateState& initial_state,
                              NewmarkCoreConfiguration configuration) {
    implementation_->Reinitialize(initial_state, std::move(configuration));
}

const NewmarkCoreConfiguration& NewmarkCore::configuration() const {
    return implementation_->configuration_;
}

}  // namespace orvd::integrators::internal
