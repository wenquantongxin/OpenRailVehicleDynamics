#include "orvd/system_assembly/system_instance.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <string>

#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
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
    const multibody_model::MultibodyModel& model, double initial_time_seconds,
    int series_spring_damper_force_state_count,
    int nominal_force_component_count, int held_active_torque_count,
    int body_wrench_count,
    const forces::WheelRailContactForcePlan* contact_force_plan)
    : issuer_(issuer),
      time_seconds_(initial_time_seconds),
      multibody_context_(model.CreateDefaultContext()),
      forward_dynamics_workspace_(model.CreateForwardDynamicsWorkspace()),
      series_spring_damper_forces_(
          Eigen::VectorXd::Zero(series_spring_damper_force_state_count)),
      nominal_forces_(Eigen::VectorXd::Zero(nominal_force_component_count)),
      held_independent_wheel_active_torques_newton_metres_(
          Eigen::VectorXd::Zero(held_active_torque_count)),
      body_wrenches_(static_cast<std::size_t>(body_wrench_count)),
      contact_force_workspace_(contact_force_plan != nullptr
                                   ? contact_force_plan->CreateWorkspace()
                                   : nullptr),
      wheel_rail_projection_station_hints_meters_(
          contact_force_plan != nullptr
              ? static_cast<std::size_t>(contact_force_plan->carrier_count())
              : 0U),
      series_force_derivatives_(
          Eigen::VectorXd::Zero(series_spring_damper_force_state_count)),
      multibody_state_time_derivatives_(
          model.num_generalized_positions() +
          model.num_generalized_velocities()) {
    if (contact_force_plan != nullptr) {
        for (int ordinal = 0; ordinal < contact_force_plan->carrier_count();
             ++ordinal) {
            wheel_rail_projection_station_hints_meters_[
                static_cast<std::size_t>(ordinal)] =
                contact_force_plan->initial_projection_station_meters(ordinal);
        }
    }
}

SystemRuntimeContext::~SystemRuntimeContext() = default;

void internal::SeedExactWheelRailContactEvaluationCaches(
    const SystemRuntimeContext& source, SystemRuntimeContext& destination) {
    if (source.issuer_ != destination.issuer_) {
        Reject("exact wheel-rail cache seed contexts belong to different "
               "system instances");
    }
    if (source.contact_force_workspace_ == nullptr ||
        destination.contact_force_workspace_ == nullptr) {
        if (source.contact_force_workspace_ == nullptr &&
            destination.contact_force_workspace_ == nullptr) {
            return;
        }
        Reject("exact wheel-rail cache seed contexts disagree about contact "
               "workspace ownership");
    }
    forces::internal::SeedExactWheelRailContactEvaluationCaches(
        *source.contact_force_workspace_,
        *destination.contact_force_workspace_);
}

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
      force_plan_(description.force_plan()),
      contact_force_plan_(description.contact_force_plan()),
      active_torque_plan_(description.active_torque_plan()),
      vehicle_body_wrench_count_(description.vehicle_body_wrench_count()),
      contact_body_wrench_count_(description.contact_body_wrench_count()),
      active_torque_body_wrench_count_(
          description.active_torque_body_wrench_count()),
      held_active_torque_count_(description.held_active_torque_count()) {}

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
        held_active_torque_count_,
        vehicle_body_wrench_count_ + contact_body_wrench_count_ +
            active_torque_body_wrench_count_,
        contact_force_plan_));
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
        Reject("the time must be finite; nothing was written");
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

void SystemInstance::SetTimeContinuousStateAndWheelRailProjectionHints(
    SystemRuntimeContext& context, double time_seconds,
    const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
    std::span<const double> projection_station_hints_meters) const {
    if (context.issuer_ != identity_) {
        Reject("the runtime context belongs to a different system");
    }
    const std::size_t expected_count =
        contact_force_plan_ != nullptr
            ? static_cast<std::size_t>(contact_force_plan_->carrier_count())
            : 0U;
    if (projection_station_hints_meters.size() != expected_count) {
        Reject("the wheel-rail projection history has " +
               std::to_string(projection_station_hints_meters.size()) +
               " entries, but this system has " +
               std::to_string(expected_count) +
               " contact carriers; nothing was written");
    }
    for (double hint : projection_station_hints_meters) {
        if (!std::isfinite(hint)) {
            Reject("a wheel-rail projection-station hint is not finite; "
                   "nothing was written");
        }
    }

    SetTimeAndContinuousState(context, time_seconds, continuous_state);
    std::copy(projection_station_hints_meters.begin(),
              projection_station_hints_meters.end(),
              context.wheel_rail_projection_station_hints_meters_.begin());
}

void SystemInstance::UpdateWheelRailProjectionStationHints(
    SystemRuntimeContext& context) const {
    if (context.issuer_ != identity_) {
        Reject("the runtime context belongs to a different system");
    }
    if (contact_force_plan_ == nullptr) {
        return;
    }
    contact_force_plan_->CalcProjectionStationHints(
        *context.multibody_context_, *context.contact_force_workspace_,
        context.wheel_rail_projection_station_hints_meters_,
        context.wheel_rail_projection_station_hints_meters_);
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

void SystemInstance::SetHeldIndependentWheelActiveTorques(
    SystemRuntimeContext& context,
    std::span<const double> held_wheel_torques_newton_metres) const {
    if (context.issuer_ != identity_) {
        Reject("the runtime context belongs to a different system");
    }
    if (held_wheel_torques_newton_metres.size() !=
        static_cast<std::size_t>(held_active_torque_count_)) {
        Reject("the held independent-wheel active torque input has " +
               std::to_string(held_wheel_torques_newton_metres.size()) +
               " entries, but this system has " +
               std::to_string(held_active_torque_count_) +
               " active torque channels; nothing was written");
    }
    for (double torque : held_wheel_torques_newton_metres) {
        if (!std::isfinite(torque)) {
            Reject("a held independent-wheel active torque is not finite; "
                   "nothing was written");
        }
    }
    std::copy(held_wheel_torques_newton_metres.begin(),
              held_wheel_torques_newton_metres.end(),
              context
                  .held_independent_wheel_active_torques_newton_metres_.begin());
}

void SystemInstance::CopyContextLocalData(
    const SystemRuntimeContext& source,
    SystemRuntimeContext& destination) const {
    if (source.issuer_ != identity_ || destination.issuer_ != identity_) {
        Reject("both runtime contexts must belong to this system before "
               "context-local data can be copied; nothing was written");
    }
    multibody_model_->CopyJointDampingParameters(
        *source.multibody_context_, destination.multibody_context_.get());
    destination.nominal_forces_ = source.nominal_forces_;
    destination.held_independent_wheel_active_torques_newton_metres_ =
        source.held_independent_wheel_active_torques_newton_metres_;
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
