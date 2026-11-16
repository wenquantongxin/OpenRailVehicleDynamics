#include "orvd/configuration/load_vehicle_contact_record.h"

#include "strict_json.h"
#include "vehicle_contact_record_json_reading_and_writing.h"
#include "vehicle_contact_record_validation.h"

namespace orvd::configuration {

VehicleContactRecord LoadVehicleContactRecordFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const auto value = strict_json::ParseStrictJson(
        strict_json::ReadWholeFile(configuration_path, "vehicle contact record"));
    const std::string path = "vehicle contact record";
    auto record = internal::DecodeVehicleContactRecordFromJson(value, path);
    internal::RequireVehicleContactRecordInvariants(record, path);
    return record;
}

}  // namespace orvd::configuration
