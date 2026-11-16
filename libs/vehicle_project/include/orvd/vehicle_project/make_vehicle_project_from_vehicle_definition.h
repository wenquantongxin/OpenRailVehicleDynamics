#pragma once

#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"

namespace orvd::vehicle_project {

// The project a vehicle definition is opened as: the definition, and for every
// rigid body its own name as the name its shapes are stated under, since a
// visual definition made for this vehicle states each body's shapes under the
// body's name in this definition. It has no contact record, no placements and
// no groups yet.
[[nodiscard]] VehicleProject MakeVehicleProject(
    configuration::VehicleDefinition vehicle_definition);

}  // namespace orvd::vehicle_project
