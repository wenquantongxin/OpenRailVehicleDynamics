#pragma once

#include <filesystem>

#include "orvd/configuration/vehicle_contact_record.h"

namespace orvd::configuration {

// Loads an explicitly supplied contact record. Profile paths remain as stated
// in the record and are resolved against the asset root during contact
// assembly, independently of the record file's directory.
//
// Refuses duplicate, unknown or missing keys, wrong types, non-finite numbers
// and unknown enumeration values. The mechanical definition is not an input:
// its names and joint geometry are checked when the declaration is bound to it.
// Contact models validate the physical computation parameters when assembled.
//
// Throws std::runtime_error when the file cannot be read and
// std::invalid_argument when the document does not describe this record.
[[nodiscard]] VehicleContactRecord LoadVehicleContactRecordFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
