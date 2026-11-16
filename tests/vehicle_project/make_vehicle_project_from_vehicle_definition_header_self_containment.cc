#include "orvd/vehicle_project/make_vehicle_project_from_vehicle_definition.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<decltype(orvd::vehicle_project::MakeVehicleProject(
                                 std::declval<orvd::configuration::VehicleDefinition>())),
                             orvd::vehicle_project::VehicleProject>);
