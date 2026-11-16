#include "orvd/vehicle_project/vehicle_project_invariants.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<decltype(orvd::vehicle_project::RequireVehicleProjectInvariants(
                                 std::declval<const orvd::vehicle_project::VehicleProject&>())),
                             void>);
