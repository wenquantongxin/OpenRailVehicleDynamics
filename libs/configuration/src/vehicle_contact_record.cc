#include "orvd/configuration/vehicle_contact_record.h"

namespace orvd::configuration {

std::vector<std::string> CollectWheelPairStationReferenceBodyNames(
    const VehicleContactDeclaration& declaration) {
    std::vector<std::string> names;
    names.reserve(declaration.carriers.size());
    for (const auto& carrier : declaration.carriers) {
        names.push_back(carrier.body_name);
    }
    return names;
}

}  // namespace orvd::configuration
