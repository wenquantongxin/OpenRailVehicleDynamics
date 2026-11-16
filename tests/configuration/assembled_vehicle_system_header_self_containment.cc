#include "orvd/configuration/assembled_vehicle_system.h"

// This translation unit includes the header first and alone: a public header
// that needed something included before it would make its own consumers
// responsible for an order nothing states.

namespace {
[[maybe_unused]] void HeaderIsSelfContained(
    const orvd::configuration::VehicleDefinition& vehicle,
    const orvd::configuration::VehicleContactDeclaration& declaration) {
    const auto system = orvd::configuration::AssembleVehicleSystem(vehicle, 9.81, &declaration);
    const orvd::configuration::VehicleContactTopology* topology =
        system->checked_contact_topology();
    (void)topology;
}
}  // namespace
