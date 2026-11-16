#pragma once

#include <filesystem>
#include <string>
#include <vector>

// Where vehicle projects are saved: a directory of projects, one file per
// project, named after the project. A project named N lives in the file
// N.vehicle_project.json of the directory.
//
// A project name is an identifier under the rule the configuration library
// applies to the identifiers it reads: not empty, and only the characters
// A-Z, a-z, 0-9, '.', '_' and '-'. A name therefore never reaches a file
// lookup as a path.

namespace orvd::vehicle_project {

// One project file: the directory of projects and the project's name in it.
struct VehicleProjectFileLocation {
    std::filesystem::path directory;
    std::string name;
};

// The file the project named `location.name` lives in.
//
// Throws std::invalid_argument, naming the name, when it is not a project
// name.
[[nodiscard]] std::filesystem::path VehicleProjectFilePath(
    const VehicleProjectFileLocation& location);

// The names of the projects saved in `directory`, sorted: every regular file
// named N.vehicle_project.json with N a project name. Nothing else in the
// directory is a project.
//
// Throws std::filesystem::filesystem_error when the directory cannot be read.
[[nodiscard]] std::vector<std::string> ListVehicleProjectNames(
    const std::filesystem::path& directory);

}  // namespace orvd::vehicle_project
