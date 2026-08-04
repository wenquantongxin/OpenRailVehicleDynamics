#include "orvd/system_assembly/compiled_system_plan.h"

#include <stdexcept>
#include <string>

#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/system_assembly_description.h"

namespace orvd::system_assembly {

CompiledSystemPlan::CompiledSystemPlan(const SystemInstance& system)
    : system_(&system),
      derivative_component_(system.multibody_component()),
      force_plan_(system.force_plan()) {}

void CompiledSystemPlan::CalcStateTimeDerivatives(
    SystemRuntimeContext& context,
    Eigen::VectorXd& state_time_derivatives) const {
    const int state_size = system_->continuous_state_size();
    if (state_time_derivatives.size() != state_size) {
        throw std::invalid_argument(
            "compiled system plan: the derivative output has " +
            std::to_string(state_time_derivatives.size()) +
            " entries, but this system has " + std::to_string(state_size) +
            "; nothing was written");
    }
    const MultibodyComponentView component =
        system_->GetMultibodyComponentView(context, derivative_component_);

    // Every force element once, in the plan's frozen order, into the context's
    // own buffers. A system with no force plan produces no wrench and its
    // series block is empty.
    if (force_plan_ != nullptr) {
        force_plan_->CalcAppliedForces(
            component.context(), context.series_spring_damper_forces_,
            context.nominal_forces_, context.body_wrenches_,
            context.series_force_derivatives_);
    }

    // The facade writes [qdot; vdot], which is shorter than this system's
    // state. The scratch it writes into belongs to the context, so evaluating
    // two contexts of one system never shares a buffer.
    component.model().CalcStateTimeDerivatives(
        component.context(), context.body_wrenches_, {}, {},
        component.forward_dynamics_workspace(),
        context.multibody_state_time_derivatives_);
    const auto series_range = system_->series_spring_damper_force_state_range();
    state_time_derivatives.head(series_range.start()) =
        context.multibody_state_time_derivatives_;
    state_time_derivatives.segment(series_range.start(), series_range.size()) =
        context.series_force_derivatives_;
}

}  // namespace orvd::system_assembly
