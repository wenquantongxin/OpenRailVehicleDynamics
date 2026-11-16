#pragma once

#include <stdexcept>
#include <string>
#include <utility>

#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_vehicle_contact_record.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/make_vehicle_project_from_vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_free_body_placements.h"

// What the vehicle project editing verifications share: failing with a
// message that says what is wrong, and the bundled reference vehicles, loaded
// with the strict loaders and opened as projects.

namespace orvd::editing_verification {

[[noreturn]] inline void Fail(const std::string& detail) {
    throw std::runtime_error(detail);
}

inline void Require(bool condition, const std::string& detail) {
    if (!condition) {
        Fail(detail);
    }
}

struct ReferenceVehicle {
    configuration::VehicleDefinition definition;
    configuration::ResolvedStartupState startup_state;
    configuration::VehicleContactRecord contact_record;
};

inline ReferenceVehicle LoadReferenceVehicle(const char* vehicle_path,
                                             const char* startup_state_path,
                                             const char* contact_record_path) {
    configuration::VehicleDefinition definition =
        configuration::LoadVehicleDefinitionFromJsonFile(vehicle_path);
    configuration::VehicleContactRecord contact_record =
        configuration::LoadVehicleContactRecordFromJsonFile(contact_record_path);
    return {std::move(definition),
            configuration::LoadResolvedStartupStateFromJsonFile(startup_state_path),
            std::move(contact_record)};
}

// The project the vehicle opens as, every free body placed at its reference
// start-up pose, without the contact record.
inline vehicle_project::VehicleProject PlacedProject(const ReferenceVehicle& reference) {
    vehicle_project::VehicleProject project =
        vehicle_project::MakeVehicleProject(reference.definition);
    vehicle_project::PlaceFreeBodiesAtStartupPoses(project, reference.startup_state);
    return project;
}

// The placed project with its contact record.
inline vehicle_project::VehicleProject PlacedProjectWithContact(const ReferenceVehicle& reference) {
    vehicle_project::VehicleProject project = PlacedProject(reference);
    project.contact_record = reference.contact_record;
    return project;
}

inline double GravityOf(const ReferenceVehicle& reference) {
    return reference.startup_state.gravitational_acceleration_meters_per_second_squared;
}

}  // namespace orvd::editing_verification
