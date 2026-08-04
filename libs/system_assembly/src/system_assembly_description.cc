#include "orvd/system_assembly/system_assembly_description.h"

#include <stdexcept>

#include "orvd/forces/vehicle_force_plan.h"
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

int SystemAssemblyDescription::state_time_derivative_size() const {
    return generalized_position_count_ + generalized_velocity_count_ +
           series_spring_damper_force_state_count_;
}

}  // namespace orvd::system_assembly
