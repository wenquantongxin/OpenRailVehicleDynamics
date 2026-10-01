#pragma once

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include "orvd/configuration/assembled_vehicle_system.h"

namespace orvd::dynamics_qualification {

// Method-independent physical [q; v; z] ownership. Every range comes from the
// assembled model and binding, not a vehicle-specific positional convention.
// Geometry tags describe stored coordinates; scale_multiplier records only
// quaternion storage norm and is not a trajectory-comparison error budget.
[[nodiscard]] nlohmann::json BuildQualificationStateLayout(
    const configuration::AssembledVehicleSystem& assembled,
    const Eigen::Ref<const Eigen::VectorXd>& initial_physical_state);

}  // namespace orvd::dynamics_qualification
