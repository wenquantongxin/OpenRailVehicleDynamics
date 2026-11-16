#include "orvd/vehicle_project/vehicle_project_free_body_placements.h"

#include <string>
#include <type_traits>
#include <utility>
#include <vector>

static_assert(std::is_same_v<decltype(orvd::vehicle_project::PlaceFreeBodiesAtStartupPoses(
                                 std::declval<orvd::vehicle_project::VehicleProject&>(),
                                 std::declval<const orvd::configuration::ResolvedStartupState&>())),
                             void>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project::CollectUnplacedFreeBodyNames(
                                 std::declval<const orvd::vehicle_project::VehicleProject&>())),
                             std::vector<std::string>>);
