#include "orvd/system_assembly/system_instance.h"

#include <atomic>
#include <stdexcept>
#include <string>

#include "orvd/multibody_model/forward_dynamics_workspace.h"
#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/system_assembly_description.h"

namespace orvd::system_assembly {
namespace {

internal::SystemIdentity NextSystemIdentity() {
    static std::atomic<internal::SystemIdentity> counter{0};
    return ++counter;
}

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("system instance: " + detail);
}

}  // namespace

SystemRuntimeContext::SystemRuntimeContext(
    internal::SystemIdentity issuer,
    const multibody_model::MultibodyModel& model)
    : issuer_(issuer),
      multibody_context_(model.CreateDefaultContext()),
      forward_dynamics_workspace_(model.CreateForwardDynamicsWorkspace()) {}

SystemRuntimeContext::~SystemRuntimeContext() = default;

const Eigen::VectorXd& SystemRuntimeContext::generalized_positions() const {
    return multibody_context_->generalized_positions();
}

const Eigen::VectorXd& SystemRuntimeContext::generalized_velocities() const {
    return multibody_context_->generalized_velocities();
}

const multibody_model::MultibodyModel& MultibodyComponentView::model() const {
    return *model_;
}

multibody_model::MultibodyEvaluationContext& MultibodyComponentView::context()
    const {
    return *context_;
}

multibody_model::ForwardDynamicsWorkspace&
MultibodyComponentView::forward_dynamics_workspace() const {
    return *forward_dynamics_workspace_;
}

SystemInstance::SystemInstance(const SystemAssemblyDescription& description)
    : identity_(NextSystemIdentity()),
      multibody_model_(&description.multibody_model()),
      generalized_position_count_(description.generalized_position_count()),
      generalized_velocity_count_(description.generalized_velocity_count()) {}

MultibodyComponentIndex SystemInstance::multibody_component() const {
    return MultibodyComponentIndex(identity_, 0);
}

SystemContinuousStateRange SystemInstance::generalized_positions_state_range()
    const {
    return SystemContinuousStateRange(0, generalized_position_count_);
}

SystemContinuousStateRange SystemInstance::generalized_velocities_state_range()
    const {
    return SystemContinuousStateRange(generalized_position_count_,
                                      generalized_velocity_count_);
}

int SystemInstance::continuous_state_size() const {
    return generalized_position_count_ + generalized_velocity_count_;
}

std::unique_ptr<SystemRuntimeContext>
SystemInstance::CreateDefaultRuntimeContext() const {
    return std::unique_ptr<SystemRuntimeContext>(
        new SystemRuntimeContext(identity_, *multibody_model_));
}

MultibodyComponentView SystemInstance::GetMultibodyComponentView(
    SystemRuntimeContext& context, MultibodyComponentIndex component) const {
    if (context.issuer_ != identity_) {
        Reject("the runtime context belongs to a different system");
    }
    if (component.system_identity_ != identity_ || component.ordinal_ != 0) {
        Reject("the multibody component index belongs to a different system");
    }
    return MultibodyComponentView(*multibody_model_,
                                  *context.multibody_context_,
                                  *context.forward_dynamics_workspace_);
}

}  // namespace orvd::system_assembly
