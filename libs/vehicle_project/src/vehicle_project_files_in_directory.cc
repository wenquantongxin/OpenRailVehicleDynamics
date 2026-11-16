#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"

#include <algorithm>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "configuration_identifier_validation.h"

namespace orvd::vehicle_project {
namespace {

constexpr std::string_view kVehicleProjectFileSuffix = ".vehicle_project.json";

void RequireVehicleProjectName(const std::string& name) {
    configuration::internal::RequireConfigurationIdentifier(name, "the vehicle project name",
                                                            "name");
}

}  // namespace

std::filesystem::path VehicleProjectFilePath(const VehicleProjectFileLocation& location) {
    RequireVehicleProjectName(location.name);
    return location.directory / (location.name + std::string(kVehicleProjectFileSuffix));
}

std::vector<std::string> ListVehicleProjectNames(const std::filesystem::path& directory) {
    std::vector<std::string> names;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(directory)) {
        const std::string file_name = entry.path().filename().string();
        if (!entry.is_regular_file() || !file_name.ends_with(kVehicleProjectFileSuffix)) {
            continue;
        }
        std::string name = file_name.substr(0, file_name.size() - kVehicleProjectFileSuffix.size());
        // A file whose name before the suffix is no project name is not a
        // project file; the rule is the one name rule there is.
        try {
            RequireVehicleProjectName(name);
        } catch (const std::invalid_argument&) {
            continue;
        }
        names.push_back(std::move(name));
    }
    std::ranges::sort(names);
    return names;
}

}  // namespace orvd::vehicle_project
