#pragma once

#include <string>

#include "orvd/configuration/vehicle_definition.h"
#include "strict_json.h"

namespace orvd::configuration::internal {

// Encodes and decodes finite records, including incomplete editing states.
// `path` identifies this record within the enclosing JSON document.
VehicleDefinition DecodeVehicleDefinitionFromJson(const strict_json::Json& value,
                                                  const std::string& path);
strict_json::Json EncodeVehicleDefinitionToJson(const VehicleDefinition& vehicle,
                                                const std::string& path);

}  // namespace orvd::configuration::internal
