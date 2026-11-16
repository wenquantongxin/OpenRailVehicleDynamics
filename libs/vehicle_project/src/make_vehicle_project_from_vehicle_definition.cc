#include "orvd/vehicle_project/make_vehicle_project_from_vehicle_definition.h"

#include <utility>

namespace orvd::vehicle_project {

VehicleProject MakeVehicleProject(configuration::VehicleDefinition vehicle_definition) {
    VehicleProject project{.vehicle_definition = std::move(vehicle_definition)};
    for (const configuration::VehicleRigidBodyDefinition& body :
         project.vehicle_definition.rigid_bodies) {
        project.visual_definition_body_names.push_back(
            {.body_name = body.name, .visual_definition_body_name = body.name});
    }
    return project;
}

}  // namespace orvd::vehicle_project
