#include "orvd/system_assembly/system_assembly_description.h"

#include <stdexcept>

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

int SystemAssemblyDescription::state_time_derivative_size() const {
    return generalized_position_count_ + generalized_velocity_count_;
}

}  // namespace orvd::system_assembly
