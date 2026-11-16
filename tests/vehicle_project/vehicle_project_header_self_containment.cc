#include "orvd/vehicle_project/vehicle_project.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::vehicle_project::VehicleProject{
                  .vehicle_definition = std::declval<orvd::configuration::VehicleDefinition>()}),
              orvd::vehicle_project::VehicleProject>);
