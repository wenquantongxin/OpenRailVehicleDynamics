#include "orvd/configuration/vehicle_contact_topology.h"

#ifdef NLOHMANN_JSON_VERSION_MAJOR
#error "the public vehicle contact topology header leaks the JSON implementation"
#endif

namespace {
[[maybe_unused]] void HeaderIsSelfContained(
    const orvd::configuration::VehicleDefinition& vehicle,
    const orvd::configuration::VehicleContactDeclaration& declaration) {
    (void)orvd::configuration::BuildVehicleContactTopology(vehicle, declaration);
}
}  // namespace
