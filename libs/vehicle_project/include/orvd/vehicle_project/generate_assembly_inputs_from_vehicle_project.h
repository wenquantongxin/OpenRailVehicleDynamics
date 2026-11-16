#pragma once

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"

// What the solver side receives from a project: the records a vehicle assembly
// consumes. Placements, shape names and groups take no part in either, and
// neither function checks them.

namespace orvd::vehicle_project {

// The project's vehicle definition with one derivation: while the project has
// a contact record, the wheel-pair station reference bodies of the mechanical
// track station layout are the bodies of its contact carriers, in carrier
// order. Without a contact record the definition's own list is kept.
[[nodiscard]] configuration::VehicleDefinition GenerateVehicleDefinition(
    const VehicleProject& project);

// A copy of the project's contact record.
//
// Throws std::invalid_argument when the project has no contact record yet; the
// message names the wheel-rail contact record as what has not been provided.
[[nodiscard]] configuration::VehicleContactRecord GenerateVehicleContactRecord(
    const VehicleProject& project);

}  // namespace orvd::vehicle_project
