#include "orvd/system_assembly/system_instance.h"

#include <atomic>
#include <cmath>
#include <stdexcept>
#include <string>

#include "orvd/multibody_model/forward_dynamics_workspace.h"
#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/forces/vehicle_force_plan.h"
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
    const multibody_model::MultibodyModel& model, double initial_time_seconds,
    int series_spring_damper_force_state_count,
    int nominal_force_component_count, int body_wrench_count)
    : issuer_(issuer),
      time_seconds_(initial_time_seconds),
      multibody_context_(model.CreateDefaultContext()),
      forward_dynamics_workspace_(model.CreateForwardDynamicsWorkspace()),
      series_spring_damper_forces_(
          Eigen::VectorXd::Zero(series_spring_damper_force_state_count)),
      nominal_forces_(Eigen::VectorXd::Zero(nominal_force_component_count)),
      body_wrenches_(static_cast<std::size_t>(body_wrench_count)),
      series_force_derivatives_(
          Eigen::VectorXd::Zero(series_spring_damper_force_state_count)),
      multibody_state_time_derivatives_(
          model.num_generalized_positions() +
          model.num_generalized_velocities()) {}

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
      generalized_velocity_count_(description.generalized_velocity_count()),
      series_spring_damper_force_state_count_(
          description.series_spring_damper_force_state_count()),
      nominal_force_component_count_(
          description.nominal_force_component_count()),
      force_plan_(description.force_plan()) {}

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

SystemContinuousStateRange
SystemInstance::series_spring_damper_force_state_range() const {
    return SystemContinuousStateRange(
        generalized_position_count_ + generalized_velocity_count_,
        series_spring_damper_force_state_count_);
}

TranslationalSpringDamperIndex
SystemInstance::GetTranslationalSpringDamperIndexByName(
    std::string_view name) const {
    if (force_plan_ == nullptr) {
        Reject("this system has no vehicle force plan, so it has no "
               "translational spring-damper named slots");
    }
    return TranslationalSpringDamperIndex(
        identity_, force_plan_->FindTranslationalSpringDamperOrdinal(name));
}

SeriesSpringViscousDamperIndex
SystemInstance::GetSeriesSpringViscousDamperIndexByName(
    std::string_view name) const {
    if (force_plan_ == nullptr) {
        Reject("this system has no vehicle force plan, so it has no series "
               "spring-viscous damper named slots");
    }
    return SeriesSpringViscousDamperIndex(
        identity_, force_plan_->FindSeriesSpringViscousDamperOrdinal(name));
}

SystemContinuousStateRange
SystemInstance::series_spring_damper_force_state_range(
    SeriesSpringViscousDamperIndex element) const {
    if (element.system_identity_ != identity_ || element.ordinal_ < 0 ||
        element.ordinal_ >= series_spring_damper_force_state_count_) {
        Reject("the series spring-viscous damper index belongs to a different "
               "system or names no element of this system");
    }
    return SystemContinuousStateRange(
        generalized_position_count_ + generalized_velocity_count_ +
            element.ordinal_,
        1);
}

int SystemInstance::continuous_state_size() const {
    return generalized_position_count_ + generalized_velocity_count_ +
           series_spring_damper_force_state_count_;
}

std::unique_ptr<SystemRuntimeContext>
SystemInstance::CreateDefaultRuntimeContext(double initial_time_seconds) const {
    if (!std::isfinite(initial_time_seconds)) {
        Reject("the initial time must be finite");
    }
    return std::unique_ptr<SystemRuntimeContext>(new SystemRuntimeContext(
        identity_, *multibody_model_, initial_time_seconds,
        series_spring_damper_force_state_count_, nominal_force_component_count_,
        force_plan_ != nullptr ? force_plan_->body_wrench_count() : 0));
}

void SystemInstance::CopyContinuousState(
    const SystemRuntimeContext& context,
    Eigen::Ref<Eigen::VectorXd> output) const {
    if (context.issuer_ != identity_) {
        Reject("the runtime context belongs to a different system");
    }
    if (output.size() != continuous_state_size()) {
        Reject("the continuous-state output has " +
               std::to_string(output.size()) + " entries, but this system has " +
               std::to_string(continuous_state_size()) +
               "; nothing was written");
    }
    output.segment(0, generalized_position_count_) =
        context.generalized_positions();
    output.segment(generalized_position_count_, generalized_velocity_count_) =
        context.generalized_velocities();
    output.segment(generalized_position_count_ + generalized_velocity_count_,
                   series_spring_damper_force_state_count_) =
        context.series_spring_damper_forces_;
}

void SystemInstance::SetContinuousState(
    SystemRuntimeContext& context,
    const Eigen::Ref<const Eigen::VectorXd>& continuous_state) const {
    SetTimeAndContinuousState(context, context.time_seconds_, continuous_state);
}

void SystemInstance::SetTimeAndContinuousState(
    SystemRuntimeContext& context, double time_seconds,
    const Eigen::Ref<const Eigen::VectorXd>& continuous_state) const {
    if (context.issuer_ != identity_) {
        Reject("the runtime context belongs to a different system");
    }
    if (!std::isfinite(time_seconds)) {
        Reject("the accepted time must be finite; nothing was written");
    }
    if (continuous_state.size() != continuous_state_size()) {
        Reject("the continuous-state input has " +
               std::to_string(continuous_state.size()) +
               " entries, but this system has " +
               std::to_string(continuous_state_size()) +
               "; nothing was written");
    }
    // The force state is checked before the model is asked to write anything.
    // The model validates and refuses atomically on its own account, so once
    // both checks have passed neither write can leave the other half stale.
    const auto series_forces = continuous_state.segment(
        generalized_position_count_ + generalized_velocity_count_,
        series_spring_damper_force_state_count_);
    if (!series_forces.allFinite()) {
        Reject(
            "a series spring-damper force state entry is not finite; nothing "
            "was written");
    }
    multibody_model_->SetGeneralizedState(
        context.multibody_context_.get(),
        continuous_state.segment(0, generalized_position_count_),
        continuous_state.segment(generalized_position_count_,
                                 generalized_velocity_count_));
    context.series_spring_damper_forces_ = series_forces;
    context.time_seconds_ = time_seconds;
}

void SystemInstance::SetNominalForce(
    SystemRuntimeContext& context, TranslationalSpringDamperIndex element,
    const Eigen::Vector3d&
        nominal_force_on_reference_end_in_reference_frame_newtons) const {
    if (context.issuer_ != identity_) {
        Reject("the runtime context belongs to a different system");
    }
    if (element.system_identity_ != identity_ || element.ordinal_ < 0 ||
        3 * element.ordinal_ + 2 >= nominal_force_component_count_) {
        Reject("the translational spring-damper index belongs to a different "
               "system or names no element of this system; nothing was "
               "written");
    }
    if (!nominal_force_on_reference_end_in_reference_frame_newtons.allFinite()) {
        Reject("a nominal force must be finite; nothing was written");
    }
    context.nominal_forces_.segment<3>(3 * element.ordinal_) =
        nominal_force_on_reference_end_in_reference_frame_newtons;
}

void SystemInstance::CopyContextLocalParameters(
    const SystemRuntimeContext& source,
    SystemRuntimeContext& destination) const {
    if (source.issuer_ != identity_ || destination.issuer_ != identity_) {
        Reject("both runtime contexts must belong to this system before "
               "context-local parameters can be copied; nothing was written");
    }
    multibody_model_->CopyJointDampingParameters(
        *source.multibody_context_, destination.multibody_context_.get());
    destination.nominal_forces_ = source.nominal_forces_;
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
