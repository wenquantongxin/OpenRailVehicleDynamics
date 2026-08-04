#pragma once

#include <string>

#include "orvd/configuration/vehicle_definition.h"

namespace orvd::configuration::internal {

// Validates the mechanical station layout consumed when a start-up state is
// placed on a line. This is deliberately shared by the JSON loader and the
// C++ assembly entry point: VehicleDefinition is a public mutable record, so
// parsing cannot be the only place that protects this invariant.
void RequireVehicleMechanicalTrackStationLayoutInvariants(
    const VehicleDefinition& vehicle, const std::string& origin);

}  // namespace orvd::configuration::internal
