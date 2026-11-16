#pragma once

#include <cstdint>

#include "orvd/configuration/vehicle_definition.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"

namespace orvd::vehicle_project_editing::internal {

// The preview of `project` at `project_revision` (vehicle_project_preview.h).
//
// `vehicle` is the definition generated from the project and `model` the
// multibody model the mechanical assembly built from it. The project must have
// passed its invariants, so every placement names a free body once, with
// finite offsets and a unit rotation, and, when it has a contact record, the
// contact declaration check, so every interface names a carrier and a wheel
// body that turns with it.
[[nodiscard]] VehicleProjectPreview ComputeVehicleProjectPreview(
    const vehicle_project::VehicleProject& project, const configuration::VehicleDefinition& vehicle,
    const multibody_model::MultibodyModel& model, std::uint64_t project_revision);

}  // namespace orvd::vehicle_project_editing::internal
