#pragma once

#include <string>

#include "orvd/configuration/vehicle_contact_record.h"
#include "strict_json.h"

namespace orvd::configuration::internal {

// Object paths are supplied by the containing document, for example $.contact_record.
// Finite intermediate values and incomplete identifiers are preserved.
[[nodiscard]] VehicleContactRecord DecodeVehicleContactRecordFromJson(
    const strict_json::Json& value, const std::string& path);
[[nodiscard]] strict_json::Json EncodeVehicleContactRecordToJson(const VehicleContactRecord& record,
                                                                 const std::string& path);

}  // namespace orvd::configuration::internal
