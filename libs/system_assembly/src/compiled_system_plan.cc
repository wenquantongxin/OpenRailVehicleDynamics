#include "orvd/system_assembly/compiled_system_plan.h"

#include "orvd/multibody_model/multibody_model.h"

namespace orvd::system_assembly {

CompiledSystemPlan::CompiledSystemPlan(const SystemInstance& system)
    : system_(&system), derivative_component_(system.multibody_component()) {}

void CompiledSystemPlan::CalcStateTimeDerivatives(
    SystemRuntimeContext& context,
    std::span<const multibody_model::AppliedBodyWrench> body_wrenches,
    std::span<const multibody_model::AppliedRevoluteJointTorque>
        revolute_joint_torques,
    std::span<const multibody_model::AppliedPrismaticJointForce>
        prismatic_joint_forces,
    Eigen::VectorXd& state_time_derivatives) const {
    const MultibodyComponentView component =
        system_->GetMultibodyComponentView(context, derivative_component_);
    component.model().CalcStateTimeDerivatives(
        component.context(), body_wrenches, revolute_joint_torques,
        prismatic_joint_forces, component.forward_dynamics_workspace(),
        state_time_derivatives);
}

}  // namespace orvd::system_assembly
