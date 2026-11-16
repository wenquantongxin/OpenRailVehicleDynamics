#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"

#include <filesystem>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

static_assert(
    std::is_same_v<decltype(orvd::vehicle_project::VehicleProjectFilePath(
                       std::declval<const orvd::vehicle_project::VehicleProjectFileLocation&>())),
                   std::filesystem::path>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project::ListVehicleProjectNames(
                                 std::declval<const std::filesystem::path&>())),
                             std::vector<std::string>>);
