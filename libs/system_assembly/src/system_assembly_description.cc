#include "orvd/system_assembly/system_assembly_description.h"

#include <stdexcept>

#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"

namespace orvd::system_assembly {

SystemAssemblyDescription::SystemAssemblyDescription(
    const multibody_model::MultibodyModel& model)
    : multibody_model_(&model) {
    if (!model.is_finalized()) {
        throw std::logic_error(
            "SystemAssemblyDescription requires a finalized multibody model");
    }
    generalized_position_count_ = model.num_generalized_positions();
    generalized_velocity_count_ = model.num_generalized_velocities();
}

SystemAssemblyDescription::SystemAssemblyDescription(
    const multibody_model::MultibodyModel& model,
    const forces::VehicleForcePlan& force_plan)
    : SystemAssemblyDescription(model) {
    if (&force_plan.model() != &model) {
        throw std::invalid_argument(
            "SystemAssemblyDescription: the force plan was compiled against a "
            "different multibody model");
    }
    force_plan_ = &force_plan;
    series_spring_damper_force_state_count_ =
        force_plan.series_spring_damper_force_state_count();
    nominal_force_component_count_ = force_plan.nominal_force_component_count();
    vehicle_body_wrench_count_ = force_plan.body_wrench_count();
}

SystemAssemblyDescription::SystemAssemblyDescription(
    const multibody_model::MultibodyModel& model,
    const forces::VehicleForcePlan& vehicle_force_plan,
    const forces::WheelRailContactForcePlan& contact_force_plan)
    : SystemAssemblyDescription(model, vehicle_force_plan) {
    if (&contact_force_plan.model() != &model) {
        throw std::invalid_argument(
            "SystemAssemblyDescription: the wheel-rail contact force plan "
            "was compiled against a different multibody model");
    }
    contact_force_plan_ = &contact_force_plan;
    contact_body_wrench_count_ = contact_force_plan.body_wrench_count();
}

SystemAssemblyDescription::SystemAssemblyDescription(
    const multibody_model::MultibodyModel& model,
    const forces::VehicleForcePlan& vehicle_force_plan,
    const forces::WheelRailContactForcePlan& contact_force_plan,
    const forces::IndependentWheelActiveTorquePlan& active_torque_plan)
    : SystemAssemblyDescription(model, vehicle_force_plan,
                                contact_force_plan) {
    if (&active_torque_plan.model() != &model) {
        throw std::invalid_argument(
            "SystemAssemblyDescription: the independent-wheel active torque "
            "plan was compiled against a different multibody model");
    }
    active_torque_plan_ = &active_torque_plan;
    active_torque_body_wrench_count_ =
        active_torque_plan.body_wrench_count();
    held_active_torque_count_ = active_torque_plan.channel_count();
}

const multibody_model::MultibodyModel&
SystemAssemblyDescription::multibody_model() const {
    return *multibody_model_;
}

int SystemAssemblyDescription::generalized_position_count() const {
    return generalized_position_count_;
}

int SystemAssemblyDescription::generalized_velocity_count() const {
    return generalized_velocity_count_;
}

int SystemAssemblyDescription::series_spring_damper_force_state_count() const {
    return series_spring_damper_force_state_count_;
}

int SystemAssemblyDescription::nominal_force_component_count() const {
    return nominal_force_component_count_;
}

int SystemAssemblyDescription::vehicle_body_wrench_count() const {
    return vehicle_body_wrench_count_;
}

int SystemAssemblyDescription::contact_body_wrench_count() const {
    return contact_body_wrench_count_;
}

int SystemAssemblyDescription::active_torque_body_wrench_count() const {
    return active_torque_body_wrench_count_;
}

int SystemAssemblyDescription::held_active_torque_count() const {
    return held_active_torque_count_;
}

int SystemAssemblyDescription::state_time_derivative_size() const {
    return generalized_position_count_ + generalized_velocity_count_ +
           series_spring_damper_force_state_count_;
}

}  // namespace orvd::system_assembly
