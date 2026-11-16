#include "orvd/vehicle_project/free_body_placements.h"

#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace orvd::vehicle_project {

void PlaceFreeBodiesAtStartupPoses(VehicleProject& project,
                                   const configuration::ResolvedStartupState& startup_state) {
    const configuration::VehicleDefinition& vehicle = project.vehicle_definition;
    const configuration::StartupVehicleBinding& binding = startup_state.vehicle_binding;
    // Identifier equality is what binds a resolved state to a mechanical
    // definition; keeping the identifier current after an edit is the
    // caller's responsibility, as it is for a start-up assembly.
    if (binding.vehicle_name != vehicle.vehicle_name ||
        binding.mechanical_definition_identifier != vehicle.mechanical_definition_identifier) {
        throw std::invalid_argument(
            "vehicle project: the start-up state was resolved for vehicle '" +
            binding.vehicle_name + "' with mechanical definition '" +
            binding.mechanical_definition_identifier + "', but this project's vehicle is '" +
            vehicle.vehicle_name + "' with mechanical definition '" +
            vehicle.mechanical_definition_identifier + "'");
    }

    std::vector<FreeBodyPlacement> placements;
    placements.reserve(startup_state.free_body_startup_states.size());
    for (const configuration::FreeBodyStartupState& body : startup_state.free_body_startup_states) {
        placements.push_back({
            .body_name = body.body_name,
            .rotation_local_track_from_body = body.rotation_local_track_from_body,
            .lateral_offset_in_local_track_frame_meters =
                body.lateral_offset_in_local_track_frame_meters,
            .vertical_offset_in_local_track_frame_meters =
                body.vertical_offset_in_local_track_frame_meters,
        });
    }
    project.free_body_placements = std::move(placements);
}

std::vector<std::string> CollectUnplacedFreeBodyNames(const VehicleProject& project) {
    std::unordered_set<std::string> placed_body_names;
    for (const FreeBodyPlacement& placement : project.free_body_placements) {
        placed_body_names.insert(placement.body_name);
    }
    std::vector<std::string> unplaced_body_names;
    for (const configuration::VehicleRigidBodyDefinition& body :
         project.vehicle_definition.rigid_bodies) {
        if (body.moves_freely_in_world && !placed_body_names.contains(body.name)) {
            unplaced_body_names.push_back(body.name);
        }
    }
    return unplaced_body_names;
}

}  // namespace orvd::vehicle_project
