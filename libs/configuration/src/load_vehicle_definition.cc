#include "orvd/configuration/load_vehicle_definition.h"

#include "strict_json.h"
#include "vehicle_definition_json_reading_and_writing.h"
#include "vehicle_definition_validation.h"

namespace orvd::configuration {

VehicleDefinition LoadVehicleDefinitionFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const auto root = strict_json::ParseStrictJson(
        strict_json::ReadWholeFile(configuration_path, "vehicle definition"));
    auto vehicle = internal::DecodeVehicleDefinitionFromJson(root, "$");
    internal::RequireVehicleDefinitionInvariants(vehicle, "$");
    return vehicle;
}

}  // namespace orvd::configuration
