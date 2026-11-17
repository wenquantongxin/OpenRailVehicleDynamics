#include <algorithm>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <string>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"
#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// The project templates the modelling service starts new projects from. Each
// is a project file in the writer's own form: the project read from it,
// written again, gives the same bytes, so a change of the project file's
// shape cannot leave a template behind. Each passes its check. The single
// rigid body's template is among them.

namespace {

using namespace orvd;
using editing_verification::Require;
using editing_verification::RequireCheckPasses;
using vehicle_project::VehicleProjectFileLocation;

std::string ReadBytes(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    Require(stream.is_open(), "'" + path.string() + "' cannot be opened");
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void VerifyTemplate(const VehicleProjectFileLocation& template_location,
                    const std::filesystem::path& work_directory) {
    const std::string what = "the template '" + template_location.name + "'";
    const vehicle_project::VehicleProject project =
        vehicle_project::ReadVehicleProjectFile(template_location);
    const VehicleProjectFileLocation written{work_directory, template_location.name};
    vehicle_project::WriteVehicleProjectFile(project, written);
    Require(ReadBytes(vehicle_project::VehicleProjectFilePath(written)) ==
                ReadBytes(vehicle_project::VehicleProjectFilePath(template_location)),
            what + " is not in the writer's own form: written again, it gives other bytes");
    vehicle_project_editing::VehicleProjectEditingSession session(project);
    RequireCheckPasses(session, what);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_templates TEMPLATE_DIRECTORY WORK_DIRECTORY\n");
        return 2;
    }
    try {
        const std::filesystem::path template_directory = argv[1];
        const std::filesystem::path work_directory = argv[2];
        std::filesystem::remove_all(work_directory);
        std::filesystem::create_directories(work_directory);
        const std::vector<std::string> names =
            vehicle_project::ListVehicleProjectNames(template_directory);
        Require(std::ranges::find(names, "single_body") != names.end(),
                "the single rigid body's template is not among the project templates");
        for (const std::string& name : names) {
            VerifyTemplate({template_directory, name}, work_directory);
        }
        std::puts(
            "vehicle project templates passed: each in the writer's own form, each passing "
            "its check");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project templates: %s\n", error.what());
        return 1;
    }
}
