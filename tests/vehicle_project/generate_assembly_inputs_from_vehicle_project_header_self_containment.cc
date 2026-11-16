#include "orvd/vehicle_project/generate_assembly_inputs_from_vehicle_project.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<decltype(orvd::vehicle_project::GenerateVehicleDefinition(
                                 std::declval<const orvd::vehicle_project::VehicleProject&>())),
                             orvd::configuration::VehicleDefinition>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project::GenerateVehicleContactRecord(
                                 std::declval<const orvd::vehicle_project::VehicleProject&>())),
                             orvd::configuration::VehicleContactRecord>);
