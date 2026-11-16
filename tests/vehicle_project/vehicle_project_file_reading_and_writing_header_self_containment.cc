#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"

#include <type_traits>
#include <utility>

static_assert(
    std::is_same_v<decltype(orvd::vehicle_project::WriteVehicleProjectFile(
                       std::declval<const orvd::vehicle_project::VehicleProject&>(),
                       std::declval<const orvd::vehicle_project::VehicleProjectFileLocation&>())),
                   void>);
static_assert(
    std::is_same_v<decltype(orvd::vehicle_project::ReadVehicleProjectFile(
                       std::declval<const orvd::vehicle_project::VehicleProjectFileLocation&>())),
                   orvd::vehicle_project::VehicleProject>);
