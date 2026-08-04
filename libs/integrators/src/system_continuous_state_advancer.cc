#include "orvd/integrators/system_continuous_state_advancer.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "orvd/integrators/cvode_continuous_state_advancer.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"

namespace orvd::integrators {

class SystemContinuousStateAdvancer::Implementation final {
   public:
    Implementation(
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& accepted_context,
        ContinuousStateErrorTolerances tolerances,
        NoCallTimeAppliedForces no_call_time_applied_forces)
        : system_(&system),
          accepted_context_(&accepted_context),
          candidate_state_(system.continuous_state_size()),
          rhs_(system, plan,
               system.CreateDefaultRuntimeContext(
                   accepted_context.time_seconds()),
               no_call_time_applied_forces) {
        system_->CopyContinuousState(*accepted_context_, candidate_state_);
        rhs_.SynchronizeContextParametersFrom(*accepted_context_);
        backend_ = std::make_unique<CvodeContinuousStateAdvancer>(
            rhs_, accepted_context_->time_seconds(), candidate_state_,
            std::move(tolerances));
    }

    void AdvanceTo(double target_time_seconds) {
        if (!std::isfinite(target_time_seconds)) {
            throw std::invalid_argument(
                "system continuous-state advancer: target time must be "
                "finite");
        }
        if (target_time_seconds < accepted_context_->time_seconds()) {
            throw std::invalid_argument(
                "system continuous-state advancer: target time precedes the "
                "accepted time");
        }
        if (requires_synchronization_) {
            throw std::logic_error(
                "system continuous-state advancer: synchronize from the "
                "accepted context before advancing");
        }
        if (target_time_seconds == accepted_context_->time_seconds()) {
            return;
        }

        try {
            backend_->AdvanceTo(target_time_seconds);
            backend_->CopyCurrentState(candidate_state_);
            system_->SetTimeAndContinuousState(
                *accepted_context_, target_time_seconds, candidate_state_);
        } catch (...) {
            requires_synchronization_ = true;
            throw;
        }
    }

    void SynchronizeAfterAcceptedContextChange() {
        requires_synchronization_ = true;
        system_->CopyContinuousState(*accepted_context_, candidate_state_);
        rhs_.SynchronizeContextParametersFrom(*accepted_context_);
        backend_->ReinitializeAfterExternalChange(
            accepted_context_->time_seconds(), candidate_state_);
        requires_synchronization_ = false;
    }

   private:
    const system_assembly::SystemInstance* system_;
    system_assembly::SystemRuntimeContext* accepted_context_;
    Eigen::VectorXd candidate_state_;
    SystemRhsBridge rhs_;
    // Declared after rhs_ so it is destroyed first; CVODE borrows the RHS.
    std::unique_ptr<CvodeContinuousStateAdvancer> backend_;
    bool requires_synchronization_{false};
};

SystemContinuousStateAdvancer::SystemContinuousStateAdvancer(
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& accepted_context,
    ContinuousStateErrorTolerances tolerances,
    NoCallTimeAppliedForces no_call_time_applied_forces)
    : implementation_(std::make_unique<Implementation>(
          system, plan, accepted_context, std::move(tolerances),
          no_call_time_applied_forces)) {}

SystemContinuousStateAdvancer::~SystemContinuousStateAdvancer() = default;

void SystemContinuousStateAdvancer::AdvanceTo(double target_time_seconds) {
    implementation_->AdvanceTo(target_time_seconds);
}

void SystemContinuousStateAdvancer::SynchronizeAfterAcceptedContextChange() {
    implementation_->SynchronizeAfterAcceptedContextChange();
}

}  // namespace orvd::integrators
