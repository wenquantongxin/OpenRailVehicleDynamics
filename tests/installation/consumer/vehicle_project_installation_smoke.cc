#include <cstdio>
#include <exception>
#include <stdexcept>

#include "orvd/vehicle_project/free_body_placements.h"
#include "orvd/vehicle_project/generate_assembly_inputs.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_invariants.h"

// The installed vehicle project used through its public headers and archive
// alone: a project is built around a one-body definition, its body placed at a
// start-up pose and grouped, the project checked and the definition generated
// back.

int main() {
    using namespace orvd;
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
        if (!unplaced_after_import ||
            !vehicle_project::CollectUnplacedFreeBodyNames(project).empty() ||
            project.free_body_placements[0].vertical_offset_in_local_track_frame_meters != -1.0 ||
            generated.rigid_bodies.size() != 1 || generated.rigid_bodies[0].name != "carbody" ||
            !contact_record_refused) {
            std::fprintf(stderr,
                         "installed vehicle project did not place, check and generate the "
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
