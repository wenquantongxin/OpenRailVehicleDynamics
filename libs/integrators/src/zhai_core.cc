#include "zhai_core.h"

#include <optional>

#include "coordinate_core_state.h"

namespace orvd::integrators::internal {

class ZhaiCore::Implementation final {
   public:
    Implementation(CoordinateSecondOrderProblem& problem,
                   ZhaiConfiguration configuration,
                   const CoordinateState& initial_state)
        : state_(problem, configuration.step_size_seconds, initial_state, 0),
          previous_b_(state_.nq_),
          previous_g_(state_.nz_) {}

    void AdvanceOneStep(double h, std::optional<double> endpoint = std::nullopt) {
        state_.ValidateStepSize(h);
        if (endpoint.has_value()) state_.ValidateExplicitEndpoint(h, *endpoint);
        // Compare the specified intervals themselves, not differences of
        // accumulated floating-point times. System stop-time rounding belongs
        // to the future adapter and must not silently loosen this condition.
        const bool normal_step = has_history_ && h == previous_step_size_;
        try {
            state_.candidate_.time_seconds = endpoint.has_value() ? *endpoint : state_.StepEnd(h);
            const auto& current = state_.accepted_;
            auto& candidate = state_.candidate_;
            if (normal_step) {
                candidate.q = current.q + h * current.s +
                              h * (h * (state_.b_ - 0.5 * previous_b_));
                candidate.s = current.s +
                              h * (1.5 * state_.b_ - 0.5 * previous_b_);
                candidate.z = current.z +
                              h * (1.5 * state_.g_ - 0.5 * previous_g_);
            } else {
                candidate.q = current.q + h * current.s +
                              h * (0.5 * h * state_.b_);
                candidate.s = current.s + h * state_.b_;
                candidate.z = current.z + h * state_.g_;
            }

            // No endpoint RHS has been evaluated yet: projecting first gives
            // one evaluation at the endpoint whose history will be retained.
            static_cast<void>(state_.ProjectCandidate());
            state_.Evaluate(candidate, state_.candidate_b_, state_.candidate_g_,
                            false);

            // All callback work has succeeded. These fixed-size assignments
            // roll history together with the accepted state.
            previous_b_ = state_.b_;
            previous_g_ = state_.g_;
            state_.CommitCandidate();
            previous_step_size_ = h;
            has_history_ = true;
            if (!normal_step) ++state_.diagnostics_.startup_step_count;
        } catch (...) {
            state_.requires_reinitialization_ = true;
            throw;
        }
    }

    void Reinitialize(const CoordinateState& state) {
        state_.Reinitialize(state);
        has_history_ = false;
        previous_step_size_ = 0.0;
    }

    CoordinateCoreState state_;
    Eigen::VectorXd previous_b_;
    Eigen::VectorXd previous_g_;
    double previous_step_size_{};
    bool has_history_{};
};

ZhaiCore::ZhaiCore(CoordinateSecondOrderProblem& problem,
                   ZhaiConfiguration configuration,
                   const CoordinateState& initial_state)
    : implementation_(
          std::make_unique<Implementation>(problem, configuration, initial_state)) {}

ZhaiCore::~ZhaiCore() = default;

void ZhaiCore::AdvanceOneStep() {
    implementation_->AdvanceOneStep(implementation_->state_.nominal_step_size_);
}

void ZhaiCore::AdvanceOneStep(double step_size_seconds) {
    implementation_->AdvanceOneStep(step_size_seconds);
}

void ZhaiCore::AdvanceOneStep(double step_size_seconds, double endpoint_time_seconds) {
    implementation_->AdvanceOneStep(step_size_seconds, endpoint_time_seconds);
}

void ZhaiCore::Reinitialize(const CoordinateState& state) {
    implementation_->Reinitialize(state);
}

double ZhaiCore::current_time_seconds() const {
    return implementation_->state_.accepted_.time_seconds;
}

void ZhaiCore::CopyCurrentState(Eigen::Ref<Eigen::VectorXd> q,
                              Eigen::Ref<Eigen::VectorXd> s,
                              Eigen::Ref<Eigen::VectorXd> z) const {
    implementation_->state_.CopyCurrentState(q, s, z);
}

ContinuousStateIntegrationStatistics ZhaiCore::integration_statistics() const {
    return implementation_->state_.statistics_;
}

CoordinateIntegrationDiagnostics ZhaiCore::diagnostics() const {
    return implementation_->state_.diagnostics_;
}

}  // namespace orvd::integrators::internal
