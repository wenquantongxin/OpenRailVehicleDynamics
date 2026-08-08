#include "orvd/integrators/system_continuous_state_advancer.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "orvd/integrators/cvode_continuous_state_advancer.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"

#include "integrator_limits.h"

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
          candidate_context_(system.CreateDefaultRuntimeContext(
              accepted_context.time_seconds())),
          candidate_state_(system.continuous_state_size()),
          rhs_(system, plan, *candidate_context_,
               no_call_time_applied_forces) {
        system_->CopyContinuousState(*accepted_context_, candidate_state_);
        system_->SetTimeContinuousStateAndWheelRailProjectionHints(
            *candidate_context_, accepted_context_->time_seconds(),
            candidate_state_,
            accepted_context_->wheel_rail_projection_station_hints_meters());
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
            bool reached_target = false;
            std::size_t successful_internal_step_count = 0;
            double expected_step_begin_time_seconds =
                accepted_context_->time_seconds();
            while (!reached_target) {
                if (successful_internal_step_count ==
                    internal::kMaximumInternalStepsPerPublicAdvance) {
                    throw std::runtime_error(
                        "system continuous-state advancer: one public "
                        "advance exceeded " +
                        std::to_string(
                            internal::kMaximumInternalStepsPerPublicAdvance) +
                        " successful internal steps");
                }
                const ContinuousStateInternalStep step =
                    backend_->AdvanceOneInternalStepToward(
                        target_time_seconds, candidate_state_);
                ++successful_internal_step_count;
                // The last RHS trial need not be at the accepted backend
                // endpoint. Continuity is checked against the preceding
                // backend endpoint, then the returned endpoint is installed
                // explicitly below.
                if (step.start_time_seconds !=
                        expected_step_begin_time_seconds ||
                    !(step.end_time_seconds > step.start_time_seconds)) {
                    throw std::runtime_error(
                        "system continuous-state advancer: the backend "
                        "returned a non-contiguous internal endpoint");
                }
                system_->SetTimeAndContinuousState(
                    *candidate_context_, step.end_time_seconds,
                    candidate_state_);
                system_->UpdateWheelRailProjectionStationHints(
                    *candidate_context_);
                expected_step_begin_time_seconds = step.end_time_seconds;
                reached_target = step.reached_stop;
            }
            system_->SetTimeContinuousStateAndWheelRailProjectionHints(
                *accepted_context_, target_time_seconds, candidate_state_,
                candidate_context_
                    ->wheel_rail_projection_station_hints_meters());
        } catch (...) {
            requires_synchronization_ = true;
            throw;
        }
    }

    void SynchronizeAfterAcceptedContextChange() {
        requires_synchronization_ = true;
        system_->CopyContinuousState(*accepted_context_, candidate_state_);
        system_->SetTimeContinuousStateAndWheelRailProjectionHints(
            *candidate_context_, accepted_context_->time_seconds(),
            candidate_state_,
            accepted_context_->wheel_rail_projection_station_hints_meters());
        rhs_.SynchronizeContextParametersFrom(*accepted_context_);
        backend_->ReinitializeAfterExternalChange(
            accepted_context_->time_seconds(), candidate_state_);
        requires_synchronization_ = false;
    }

   private:
    const system_assembly::SystemInstance* system_;
    system_assembly::SystemRuntimeContext* accepted_context_;
    // Declared before rhs_ because the RHS borrows this context.
    std::unique_ptr<system_assembly::SystemRuntimeContext> candidate_context_;
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
