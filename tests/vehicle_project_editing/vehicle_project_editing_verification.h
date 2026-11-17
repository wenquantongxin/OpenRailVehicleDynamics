#pragma once

#include <cstdint>
#include <expected>
#include <optional>
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
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"
#include "vehicle_project/vehicle_project_comparison.h"

// What the vehicle project editing verifications share: failing with a
// message that says what is wrong; the bundled reference vehicles, loaded
// with the strict loaders and opened as projects; and requiring an edit to be
// accepted, or refused without a trace, and a session's project to pass its
// check.

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
// start-up pose and checked under that start-up state's gravity, without the
// contact record.
inline vehicle_project::VehicleProject PlacedProject(const ReferenceVehicle& reference) {
    vehicle_project::VehicleProject project =
        vehicle_project::MakeVehicleProject(reference.definition);
    vehicle_project::PlaceFreeBodiesAtStartupPoses(project, reference.startup_state);
    project.gravitational_acceleration_meters_per_second_squared =
        reference.startup_state.gravitational_acceleration_meters_per_second_squared;
    return project;
}

// The placed project with its contact record.
inline vehicle_project::VehicleProject PlacedProjectWithContact(const ReferenceVehicle& reference) {
    vehicle_project::VehicleProject project = PlacedProject(reference);
    project.contact_record = reference.contact_record;
    return project;
}

inline vehicle_project_editing::MechanicalCheckReport RequireCheckPasses(
    vehicle_project_editing::VehicleProjectEditingSession& session, const std::string& what) {
    const vehicle_project_editing::MechanicalCheckReport report = session.CheckMechanics();
    Require(report.refusals.empty(),
            what + " did not pass the check: " +
                (report.refusals.empty() ? std::string() : report.refusals.front()));
    return report;
}

// An accepted edit's result, or a failure saying what was refused and why.
template <typename Value>
Value RequireAccepted(
    const std::expected<Value, vehicle_project_editing::SubjectEditRefusal>& outcome,
    const std::string& what) {
    Require(outcome.has_value(),
            what + " was refused: " + (outcome.has_value() ? "" : outcome.error().reason));
    return *outcome;
}

// A refusal names `element` and `property_name` and leaves the project, its
// history and its revision as they were.
template <typename Value>
void RequireRefused(
    const std::expected<Value, vehicle_project_editing::SubjectEditRefusal>& outcome,
    const vehicle_project::ElementReference& element,
    const std::optional<std::string>& property_name,
    const vehicle_project_editing::VehicleProjectEditingSession& session,
    const vehicle_project::VehicleProject& before, std::uint64_t revision_before,
    bool could_undo_before, const std::string& what) {
    Require(!outcome.has_value(), what + " was accepted");
    const vehicle_project_editing::SubjectEditRefusal& refusal = outcome.error();
    Require(refusal.subject == vehicle_project_editing::PropertySubject{element} &&
                refusal.property_name == property_name && !refusal.reason.empty(),
            what + " was refused without naming its element and property and saying why");
    Require(vehicle_project_comparison::SameProject(before, session.project()) &&
                session.revision() == revision_before && session.can_undo() == could_undo_before &&
                !session.can_redo(),
            what + " changed the project, its history or its revision");
}

}  // namespace orvd::editing_verification
