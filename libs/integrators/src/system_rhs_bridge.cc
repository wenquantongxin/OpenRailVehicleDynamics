#include "orvd/integrators/system_rhs_bridge.h"

#include <stdexcept>

#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_instance.h"

namespace orvd::integrators {

SystemRhsBridge::SystemRhsBridge(
    const system_assembly::SystemInstance& system,
    const system_assembly::CompiledSystemPlan& plan,
    system_assembly::SystemRuntimeContext& trial_context,
    NoCallTimeAppliedForces)
    : system_(&system),
      plan_(&plan),
      trial_context_(&trial_context),
      derivative_buffer_(system.continuous_state_size()) {
    // Resolving the component once validates that the trial context belongs to
    // this system and that the plan was compiled from the same system.  The
    // returned view is deliberately not retained.
    (void)system_->GetMultibodyComponentView(
        *trial_context_, plan_->derivative_component());
}

SystemRhsBridge::~SystemRhsBridge() = default;

int SystemRhsBridge::continuous_state_size() const {
    return system_->continuous_state_size();
}

void SystemRhsBridge::SynchronizeContextParametersFrom(
    const system_assembly::SystemRuntimeContext& source_context) {
    system_->CopyContextLocalParameters(source_context, *trial_context_);
}

void SystemRhsBridge::CalcTimeDerivatives(
    double time_seconds,
    const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
    Eigen::Ref<Eigen::VectorXd> state_time_derivatives) {
    if (continuous_state.size() != continuous_state_size()) {
        throw std::invalid_argument(
            "system RHS bridge: trial state has the wrong size; nothing was "
            "written");
    }
    if (state_time_derivatives.size() != continuous_state_size()) {
        throw std::invalid_argument(
            "system RHS bridge: derivative output has the wrong size; nothing "
            "was written");
    }

    system_->SetTimeAndContinuousState(*trial_context_, time_seconds,
                                       continuous_state);
    plan_->CalcStateTimeDerivatives(*trial_context_, derivative_buffer_);
    state_time_derivatives = derivative_buffer_;
}

}  // namespace orvd::integrators
