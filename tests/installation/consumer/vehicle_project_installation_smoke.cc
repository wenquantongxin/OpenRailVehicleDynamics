#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "orvd/vehicle_project/generate_assembly_inputs_from_vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"
#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"
#include "orvd/vehicle_project/vehicle_project_free_body_placements.h"
#include "orvd/vehicle_project/vehicle_project_invariants.h"

// The installed vehicle project used through its public headers and archive
// alone: a project is built around a one-body definition, its body placed at a
// start-up pose and grouped, the project checked and the definition generated
// back, and the project saved into a directory of projects, listed there and
// read back.

int main(int argc, char** argv) {
    using namespace orvd;
    if (argc != 2) {
        std::fprintf(stderr, "usage: vehicle_project_installation_smoke PROJECT_DIRECTORY\n");
        return 2;
    }
    try {
        configuration::VehicleDefinition vehicle;
        vehicle.vehicle_name = "smoke_vehicle";
        vehicle.mechanical_definition_identifier = "smoke_vehicle_mechanics";
        vehicle.rigid_bodies.push_back(
            {.name = "carbody", .moves_freely_in_world = true, .mass_kilograms = 1.0});
        configuration::ResolvedStartupState startup_state;
        startup_state.vehicle_binding = {vehicle.vehicle_name,
                                         vehicle.mechanical_definition_identifier};
        startup_state.free_body_startup_states.push_back(
            {.body_name = "carbody", .vertical_offset_in_local_track_frame_meters = -1.0});

        vehicle_project::VehicleProject project{.vehicle_definition = vehicle};
        const bool unplaced_after_import =
            vehicle_project::CollectUnplacedFreeBodyNames(project).size() == 1;
        vehicle_project::PlaceFreeBodiesAtStartupPoses(project, startup_state);
        project.element_groups.push_back(
            {.name = "car", .members = {{vehicle_project::ElementKind::kRigidBody, "carbody"}}});
        vehicle_project::RequireVehicleProjectInvariants(project);
        const auto generated = vehicle_project::GenerateVehicleDefinition(project);
        bool contact_record_refused = false;
        try {
            (void)vehicle_project::GenerateVehicleContactRecord(project);
        } catch (const std::invalid_argument&) {
            contact_record_refused = true;
        }

        project.gravitational_acceleration_meters_per_second_squared = 9.81;
        project.visual_definition_document = R"({"parts": []})";
        const vehicle_project::VehicleProjectFileLocation file{argv[1], "smoke_vehicle"};
        vehicle_project::WriteVehicleProjectFile(project, file);
        const vehicle_project::VehicleProject read = vehicle_project::ReadVehicleProjectFile(file);
        const bool saved_and_read_back =
            vehicle_project::ListVehicleProjectNames(file.directory) ==
                std::vector<std::string>{file.name} &&
            read.vehicle_definition.rigid_bodies.size() == 1 &&
            read.free_body_placements.size() == 1 &&
            read.free_body_placements[0].vertical_offset_in_local_track_frame_meters == -1.0 &&
            read.element_groups.size() == 1 &&
            read.gravitational_acceleration_meters_per_second_squared == 9.81 &&
            !read.visual_definition_document.empty();

        if (!unplaced_after_import ||
            !vehicle_project::CollectUnplacedFreeBodyNames(project).empty() ||
            project.free_body_placements[0].vertical_offset_in_local_track_frame_meters != -1.0 ||
            generated.rigid_bodies.size() != 1 || generated.rigid_bodies[0].name != "carbody" ||
            !contact_record_refused || !saved_and_read_back) {
            std::fprintf(stderr,
                         "installed vehicle project did not place, check, generate and save the "
                         "vehicle it imported\n");
            return 1;
        }
        std::puts("installed vehicle project smoke passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed vehicle project smoke failed: %s\n", error.what());
        return 1;
    }
}
