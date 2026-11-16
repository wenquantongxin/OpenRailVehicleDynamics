#pragma once

#include <string>

#include "orvd/configuration/vehicle_definition.h"

namespace orvd::configuration::internal {

// Checks identities, declared references, force names and station layout before
// assembly. Finite unfinished records may be encoded without this check.
void RequireVehicleDefinitionDeclarationInvariants(const VehicleDefinition& vehicle,
                                                   const std::string& path);

// Applies the declaration and parameter restrictions of formal vehicle files.
// Shape, type and finite-number checks belong to the JSON decoder.
void RequireVehicleDefinitionInvariants(const VehicleDefinition& vehicle, const std::string& path);

}  // namespace orvd::configuration::internal
