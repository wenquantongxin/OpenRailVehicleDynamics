#include "radau5_continuous_state_advancer.h"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

#include "orvd/radau5/radau5_core.h"

namespace orvd::integrators {

class Radau5ContinuousStateAdvancer::Implementation final {
   public:
    class Callback final : public radau5::RhsEvaluator {
       public:
        Callback(ContinuousStateRhs& rhs, int state_size)
            : rhs_(&rhs), state_size_(state_size) {}

        [[nodiscard]] int continuous_state_size() const noexcept override {
            return state_size_;
        }

        [[nodiscard]] radau5::RhsEvaluationStatus Evaluate(
            double time_seconds,
            const Eigen::Ref<const Eigen::VectorXd>& state,
            Eigen::Ref<Eigen::VectorXd> derivatives) noexcept override {
            try {
                rhs_->CalcTimeDerivatives(time_seconds, state, derivatives);
                exception_ = nullptr;
                return radau5::RhsEvaluationStatus::kSuccess;
            } catch (...) {
                exception_ = std::current_exception();
                return rhs_->IsRecoverableFailure(exception_)
                           ? radau5::RhsEvaluationStatus::kRecoverableFailure
                           : radau5::RhsEvaluationStatus::kFatalFailure;
            }
        }

        [[nodiscard]] const std::exception_ptr& exception() const noexcept {
            return exception_;
        }

        void ClearException() noexcept { exception_ = nullptr; }

       private:
        ContinuousStateRhs* rhs_;
        int state_size_;
        std::exception_ptr exception_;
    };

    Implementation(ContinuousStateRhs& rhs,
                   double initial_time_seconds,
                   Eigen::VectorXd initial_continuous_state,
                   ContinuousStateErrorTolerances tolerances)
        : callback_(rhs, rhs.continuous_state_size()),
          core_(callback_, initial_time_seconds,
                std::move(initial_continuous_state),
                tolerances.relative_tolerance(),
                tolerances.component_absolute_tolerances()) {}

    [[nodiscard]] int state_size() const noexcept {
        return core_.continuous_state_size();
    }

    [[nodiscard]] double time_seconds() const noexcept {
        return core_.current_time_seconds();
    }

    [[nodiscard]] ContinuousStateIntegrationStatistics Statistics()
        const noexcept {
        const radau5::Statistics value = core_.statistics();
        return ContinuousStateIntegrationStatistics{
            value.successful_internal_step_count,
            value.right_hand_side_evaluation_count,
            value.linear_solver_right_hand_side_evaluation_count,
            value.error_test_failure_count,
            value.nonlinear_solver_iteration_count,
            value.nonlinear_solver_convergence_failure_count,
            value.linear_solver_setup_count,
            value.jacobian_evaluation_count,
            1};
    }

    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd> output) const {
        core_.CopyCurrentState(output);
    }

    [[nodiscard]] ContinuousStateInternalStep Advance(
        double stop_time_seconds,
        Eigen::Ref<Eigen::VectorXd> endpoint_continuous_state) {
        if (!std::isfinite(stop_time_seconds)) {
            throw std::invalid_argument(
                "Radau5 continuous-state advancer: stop time must be finite");
        }
        if (stop_time_seconds < core_.current_time_seconds()) {
            throw std::invalid_argument(
                "Radau5 continuous-state advancer: stop time precedes the "
                "current successful endpoint");
        }
        if (endpoint_continuous_state.size() != core_.continuous_state_size()) {
            throw std::invalid_argument(
                "Radau5 continuous-state advancer: endpoint output has the "
                "wrong size; nothing was written");
        }

        try {
            const radau5::InternalStep step =
                core_.AdvanceOneAcceptedStepToward(stop_time_seconds);
            core_.CopyCurrentState(endpoint_continuous_state);
            return ContinuousStateInternalStep{
                step.start_time_seconds, step.end_time_seconds,
                step.reached_stop};
        } catch (const radau5::Failure& failure) {
            const bool rhs_failure =
                failure.reason() == radau5::Failure::Reason::kFatalRhs ||
                failure.reason() ==
                    radau5::Failure::Reason::kRecoverableRhsExhausted;
            if (rhs_failure && callback_.exception() != nullptr) {
                std::rethrow_exception(callback_.exception());
            }
            throw;
        }
    }

    void Reinitialize(
        double committed_time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& committed_continuous_state) {
        core_.Reinitialize(committed_time_seconds, committed_continuous_state);
        callback_.ClearException();
    }

    void InvalidateLinearizationAfterNumericalRhsHistoryChange() noexcept {
        core_.InvalidateLinearizationAfterNumericalRhsHistoryChange();
    }

    [[nodiscard]] std::optional<ContinuousStateDenseOutputInterval>
    DenseInterval() const noexcept {
        const auto interval = core_.dense_output_interval();
        if (!interval.has_value()) return std::nullopt;
        return ContinuousStateDenseOutputInterval{
            interval->start_time_seconds, interval->end_time_seconds};
    }

    void CopyDenseState(double time_seconds,
                        Eigen::Ref<Eigen::VectorXd> output) const {
        core_.CopyDenseState(time_seconds, output);
    }

   private:
    // The callback must outlive the core that borrows it.
    Callback callback_;
    radau5::Core core_;
};

Radau5ContinuousStateAdvancer::Radau5ContinuousStateAdvancer(
    ContinuousStateRhs& rhs,
    double initial_time_seconds,
    Eigen::VectorXd initial_continuous_state,
    ContinuousStateErrorTolerances tolerances)
    : implementation_(std::make_unique<Implementation>(
          rhs, initial_time_seconds, std::move(initial_continuous_state),
          std::move(tolerances))) {}

Radau5ContinuousStateAdvancer::~Radau5ContinuousStateAdvancer() = default;

int Radau5ContinuousStateAdvancer::continuous_state_size() const {
    return implementation_->state_size();
}

double Radau5ContinuousStateAdvancer::current_time_seconds() const {
    return implementation_->time_seconds();
}

ContinuousStateIntegrationStatistics
Radau5ContinuousStateAdvancer::integration_statistics() const {
    return implementation_->Statistics();
}

void Radau5ContinuousStateAdvancer::CopyCurrentState(
    Eigen::Ref<Eigen::VectorXd> continuous_state) const {
    implementation_->CopyCurrentState(continuous_state);
}

ContinuousStateInternalStep
Radau5ContinuousStateAdvancer::AdvanceOneInternalStepToward(
    double stop_time_seconds,
    Eigen::Ref<Eigen::VectorXd> endpoint_continuous_state) {
    return implementation_->Advance(stop_time_seconds,
                                    endpoint_continuous_state);
}

void Radau5ContinuousStateAdvancer::ReinitializeAfterExternalChange(
    double committed_time_seconds,
    const Eigen::Ref<const Eigen::VectorXd>& committed_continuous_state) {
    implementation_->Reinitialize(committed_time_seconds,
                                  committed_continuous_state);
}

void Radau5ContinuousStateAdvancer::
    InvalidateLinearizationAfterNumericalRhsHistoryChange() noexcept {
    implementation_
        ->InvalidateLinearizationAfterNumericalRhsHistoryChange();
}

std::optional<ContinuousStateDenseOutputInterval>
Radau5ContinuousStateAdvancer::dense_output_interval() const {
    return implementation_->DenseInterval();
}

void Radau5ContinuousStateAdvancer::CopyDenseState(
    double time_seconds,
    Eigen::Ref<Eigen::VectorXd> continuous_state) const {
    implementation_->CopyDenseState(time_seconds, continuous_state);
}

}  // namespace orvd::integrators
