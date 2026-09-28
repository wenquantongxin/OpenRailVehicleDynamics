#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

#include "orvd/integrators/continuous_state_advancer.h"

#include "coordinate_second_order_problem.h"
#include "coordinate_step_time.h"

namespace orvd::integrators::internal {

// Shared storage/transaction operations for the two real coordinate cores.
// This does not select a numerical method or provide a public system adapter.
class CoordinateCoreState final {
   public:
    CoordinateCoreState(CoordinateSecondOrderProblem& problem,
                        double nominal_step_size, const CoordinateState& initial,
                        int finite_difference_worker_count)
        : problem_(problem),
          nq_(problem.coordinate_size()),
          nz_(problem.internal_state_size()),
          nominal_step_size_(nominal_step_size),
          worker_count_(finite_difference_worker_count) {
        if (nq_ <= 0 || nz_ < 0 ||
            nq_ > std::numeric_limits<int>::max() - nz_) {
            throw std::invalid_argument("coordinate core: invalid dimensions");
        }
        if (!std::isfinite(nominal_step_size_) || nominal_step_size_ <= 0.0) {
            throw std::invalid_argument(
                "coordinate core: nominal step must be finite and positive");
        }
        accepted_.q.setZero(nq_);
        accepted_.s.setZero(nq_);
        accepted_.z.setZero(nz_);
        candidate_ = accepted_;
        b_.resize(nq_);
        g_.resize(nz_);
        candidate_b_.resize(nq_);
        candidate_g_.resize(nz_);
        reference_q_.resize(nq_);
        Reinitialize(initial);
    }

    CoordinateCoreState(const CoordinateCoreState&) = delete;
    CoordinateCoreState& operator=(const CoordinateCoreState&) = delete;
    CoordinateCoreState(CoordinateCoreState&&) = delete;
    CoordinateCoreState& operator=(CoordinateCoreState&&) = delete;

    void ValidateInitial(const CoordinateState& initial) const {
        if (initial.q.size() != nq_ || initial.s.size() != nq_ ||
            initial.z.size() != nz_ || !std::isfinite(initial.time_seconds) ||
            !initial.q.allFinite() || !initial.s.allFinite() ||
            !initial.z.allFinite()) {
            throw std::invalid_argument(
                "coordinate core: initial state has invalid size or value");
        }
        problem_.ValidateInitialState(initial.time_seconds, initial.q, initial.s,
                                      initial.z);
    }

    void Reinitialize(const CoordinateState& initial) {
        // Invalid caller input leaves an already usable core usable.
        ValidateInitial(initial);
        candidate_ = initial;
        try {
            Evaluate(candidate_, candidate_b_, candidate_g_, false);
        } catch (...) {
            requires_reinitialization_ = true;
            throw;
        }
        accepted_ = candidate_;
        b_ = candidate_b_;
        g_ = candidate_g_;
        reference_q_ = initial.q;
        statistics_ = {};
        statistics_.right_hand_side_evaluation_count = 1;
        statistics_.requested_dense_finite_difference_jacobian_worker_count =
            worker_count_;
        diagnostics_ = {};
        requires_reinitialization_ = false;
    }

    void ValidateStepSize(double h) const {
        if (!std::isfinite(h) || h <= 0.0 || h > nominal_step_size_) {
            throw std::invalid_argument(
                "coordinate core: step must be finite and in (0, nominal]");
        }
        if (requires_reinitialization_) {
            throw std::logic_error(
                "coordinate core: reinitialization required after failure");
        }
    }

    [[nodiscard]] double StepEnd(double h) const {
        const double end = accepted_.time_seconds + h;
        if (!std::isfinite(end) || !(end > accepted_.time_seconds)) {
            throw CoordinateIntegrationFailure(
                CoordinateIntegrationFailure::Reason::kStepSizeUnderflow,
                "coordinate core: step cannot advance a finite time");
        }
        return end;
    }

    void ValidateExplicitEndpoint(double h, double endpoint) const {
        ValidateStepSize(h);
        if (!CoordinateEndpointTimeIsCompatible(accepted_.time_seconds, h, endpoint)) {
            throw std::invalid_argument(
                "coordinate core: explicit endpoint exceeds the clock-rounding allowance");
        }
    }

    void ValidateCandidate() const {
        if (!std::isfinite(candidate_.time_seconds) ||
            !candidate_.q.allFinite() || !candidate_.s.allFinite() ||
            !candidate_.z.allFinite()) {
            throw CoordinateIntegrationFailure(
                CoordinateIntegrationFailure::Reason::kNonFiniteState,
                "coordinate core: non-finite candidate state");
        }
    }

    void Evaluate(const CoordinateState& state,
                  Eigen::Ref<Eigen::VectorXd> b,
                  Eigen::Ref<Eigen::VectorXd> g, bool jacobian_evaluation) {
        if (!state.q.allFinite() || !state.s.allFinite() ||
            !state.z.allFinite() || !std::isfinite(state.time_seconds)) {
            throw CoordinateIntegrationFailure(
                CoordinateIntegrationFailure::Reason::kNonFiniteState,
                "coordinate core: non-finite trial state");
        }
        if (jacobian_evaluation) {
            ++statistics_.linear_solver_right_hand_side_evaluation_count;
        } else {
            ++statistics_.right_hand_side_evaluation_count;
        }
        // Detect incomplete output as well as an explicitly non-finite RHS.
        b.setConstant(std::numeric_limits<double>::quiet_NaN());
        g.setConstant(std::numeric_limits<double>::quiet_NaN());
        problem_.Evaluate(state.time_seconds, state.q, state.s, state.z, b, g);
        if (!b.allFinite() || !g.allFinite()) {
            throw CoordinateIntegrationFailure(
                CoordinateIntegrationFailure::Reason::kNonFiniteEvaluation,
                "coordinate core: non-finite acceleration or internal derivative");
        }
    }

    [[nodiscard]] bool ProjectCandidate() {
        ValidateCandidate();
        ++diagnostics_.endpoint_projection_evaluation_count;
        const bool changed = problem_.ProjectEndpoint(
            reference_q_, candidate_.q, candidate_.s);
        if (changed) {
            ++diagnostics_.endpoint_projection_change_count;
        }
        ValidateCandidate();
        return changed;
    }

    void CommitCandidate() {
        // Dimensions are fixed and all storage is preallocated. No callback is
        // invoked between these assignments and the successful-step count.
        accepted_ = candidate_;
        b_ = candidate_b_;
        g_ = candidate_g_;
        ++statistics_.successful_internal_step_count;
    }

    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd> q,
                          Eigen::Ref<Eigen::VectorXd> s,
                          Eigen::Ref<Eigen::VectorXd> z) const {
        if (q.size() != nq_ || s.size() != nq_ || z.size() != nz_) {
            throw std::invalid_argument(
                "coordinate core: state output has the wrong size");
        }
        q = accepted_.q;
        s = accepted_.s;
        z = accepted_.z;
    }

    CoordinateSecondOrderProblem& problem_;
    const int nq_;
    const int nz_;
    const double nominal_step_size_;
    const int worker_count_;
    CoordinateState accepted_;
    CoordinateState candidate_;
    Eigen::VectorXd b_;
    Eigen::VectorXd g_;
    Eigen::VectorXd candidate_b_;
    Eigen::VectorXd candidate_g_;
    Eigen::VectorXd reference_q_;
    ContinuousStateIntegrationStatistics statistics_;
    CoordinateIntegrationDiagnostics diagnostics_;
    bool requires_reinitialization_{false};
};

}  // namespace orvd::integrators::internal
