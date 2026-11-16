#include "orvd/vehicle_project/generate_assembly_inputs.h"

#include <stdexcept>

namespace orvd::vehicle_project {

configuration::VehicleDefinition GenerateVehicleDefinition(const VehicleProject& project) {
    configuration::VehicleDefinition vehicle = project.vehicle_definition;
    if (project.contact_record.has_value()) {
        vehicle.mechanical_track_station_layout.wheel_pair_station_reference_body_names =
            configuration::CollectWheelPairStationReferenceBodyNames(
                project.contact_record->declaration);
    }
    return vehicle;
}

configuration::VehicleContactRecord GenerateVehicleContactRecord(const VehicleProject& project) {
    if (!project.contact_record.has_value()) {
        throw std::invalid_argument(
            "vehicle project.contact_record is absent: the wheel-rail contact record, with its "
            "contact declaration, wheel and rail contact configuration and contact computation "
            "settings, has not been provided yet");
    }
    return *project.contact_record;
}

}  // namespace orvd::vehicle_project
