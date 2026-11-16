#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/generate_assembly_inputs_from_vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"
#include "orvd/vehicle_project/vehicle_project_free_body_placements.h"
#include "orvd/vehicle_project/vehicle_project_invariants.h"
#include "vehicle_definition_validation.h"
#include "vehicle_project_element_creation.h"
#include "vehicle_project_element_property_assignment.h"
#include "vehicle_project_element_removal.h"
#include "vehicle_project_preview_computation.h"

namespace orvd::vehicle_project_editing {

using vehicle_project::ElementReference;

namespace {

// The names a project saved as `name` carries: its vehicle name and its
// mechanical definition identifier.
void GiveProjectNames(vehicle_project::VehicleProject& project, const std::string& name) {
    project.vehicle_definition.vehicle_name = name;
    project.vehicle_definition.mechanical_definition_identifier = name;
}

}  // namespace

VehicleProjectEditingSession::VehicleProjectEditingSession(vehicle_project::VehicleProject project)
    : project_(std::move(project)) {}

VehicleProjectEditingSession::VehicleProjectEditingSession(
    vehicle_project::VehicleProject project,
    vehicle_project::VehicleProjectFileLocation associated_file)
    : project_(std::move(project)), associated_file_(std::move(associated_file)) {}

std::expected<ElementReference, ElementEditRefusal>
VehicleProjectEditingSession::SetElementProperty(const ElementReference& element,
                                                 std::string_view property_name,
                                                 const PropertyValue& value) {
    vehicle_project::VehicleProject edited = project_;
    std::expected<ElementReference, ElementEditRefusal> assigned =
        internal::AssignElementProperty(edited, element, property_name, value);
    if (assigned.has_value()) {
        AdoptEdit(std::move(edited));
    }
    return assigned;
}

std::expected<ElementReference, ElementEditRefusal> VehicleProjectEditingSession::AddElement(
    vehicle_project::ElementKind kind, const std::string& name,
    const PropertyValuesByName& properties) {
    vehicle_project::VehicleProject edited = project_;
    std::expected<ElementReference, ElementEditRefusal> created =
        internal::CreateElement(edited, kind, name, properties);
    if (created.has_value()) {
        AdoptEdit(std::move(edited));
    }
    return created;
}

std::expected<std::vector<ElementReference>, ElementEditRefusal>
VehicleProjectEditingSession::RemoveElement(const ElementReference& element) {
    std::expected<std::vector<ElementReference>, ElementEditRefusal> removed =
        internal::CollectElementsRemovedWith(project_, element);
    if (removed.has_value()) {
        vehicle_project::VehicleProject edited = project_;
        internal::RemoveElements(edited, *removed);
        AdoptEdit(std::move(edited));
    }
    return removed;
}

std::expected<std::vector<ElementReference>, ElementEditRefusal>
VehicleProjectEditingSession::CollectRemovalConsequences(const ElementReference& element) const {
    return internal::CollectElementsRemovedWith(project_, element);
}

void VehicleProjectEditingSession::AdoptEdit(vehicle_project::VehicleProject edited) {
    // The redo history goes, and with it the saved state when it lay there.
    if (saved_state_position_.has_value() && *saved_state_position_ > undo_snapshots_.size()) {
        saved_state_position_.reset();
    }
    undo_snapshots_.push_back(std::move(project_));
    project_ = std::move(edited);
    redo_snapshots_.clear();
    ++revision_;
}

void VehicleProjectEditingSession::Undo() {
    if (undo_snapshots_.empty()) {
        throw std::logic_error("vehicle project editing: there is no step to undo");
    }
    redo_snapshots_.push_back(std::move(project_));
    project_ = std::move(undo_snapshots_.back());
    undo_snapshots_.pop_back();
    ++revision_;
}

void VehicleProjectEditingSession::Redo() {
    if (redo_snapshots_.empty()) {
        throw std::logic_error("vehicle project editing: there is no step to redo");
    }
    undo_snapshots_.push_back(std::move(project_));
    project_ = std::move(redo_snapshots_.back());
    redo_snapshots_.pop_back();
    ++revision_;
}

void VehicleProjectEditingSession::SaveVehicleProject() {
    if (!associated_file_.has_value()) {
        throw std::logic_error(
            "vehicle project editing: the session has no vehicle project file to save to; save "
            "it as a named project first");
    }
    vehicle_project::WriteVehicleProjectFile(project_, *associated_file_);
    saved_state_position_ = undo_snapshots_.size();
}

void VehicleProjectEditingSession::SaveVehicleProjectAs(
    const vehicle_project::VehicleProjectFileLocation& file) {
    vehicle_project::VehicleProject renamed = project_;
    GiveProjectNames(renamed, file.name);
    vehicle_project::WriteVehicleProjectFile(renamed, file);
    // The names are the project's identity, not an edit: the whole history
    // takes them, and no step is added.
    if (project_.vehicle_definition.vehicle_name != file.name ||
        project_.vehicle_definition.mechanical_definition_identifier != file.name) {
        ++revision_;
    }
    project_ = std::move(renamed);
    for (vehicle_project::VehicleProject& snapshot : undo_snapshots_) {
        GiveProjectNames(snapshot, file.name);
    }
    for (vehicle_project::VehicleProject& snapshot : redo_snapshots_) {
        GiveProjectNames(snapshot, file.name);
    }
    associated_file_ = file;
    saved_state_position_ = undo_snapshots_.size();
}

MechanicalCheckReport VehicleProjectEditingSession::CheckMechanics() {
    MechanicalCheckReport report;
    try {
        vehicle_project::RequireVehicleProjectInvariants(project_);
    } catch (const std::exception& refusal) {
        report.refusals.emplace_back(refusal.what());
    }
    // Placements, shape names and groups take no part in generation or
    // assembly, so a refusal above does not keep the assembly from being
    // checked. The declaration rules of a vehicle file come first, on the
    // definition assembly would receive, at the place it holds in a project
    // file; when they refuse, the assembly is not attempted.
    const configuration::VehicleDefinition vehicle =
        vehicle_project::GenerateVehicleDefinition(project_);
    bool assembly_attempted = true;
    try {
        configuration::internal::RequireVehicleDefinitionDeclarationInvariants(
            vehicle, "$.vehicle_definition");
    } catch (const std::exception& refusal) {
        report.refusals.emplace_back(refusal.what());
        assembly_attempted = false;
    }
    const std::optional<double>& gravity =
        project_.gravitational_acceleration_meters_per_second_squared;
    if (!gravity.has_value()) {
        report.refusals.emplace_back(
            "the project states no gravitational acceleration to check the vehicle under");
        assembly_attempted = false;
    }
    std::unique_ptr<configuration::AssembledVehicleSystem> system;
    if (assembly_attempted) {
        try {
            system = configuration::AssembleVehicleSystem(
                vehicle, *gravity,
                project_.contact_record ? &project_.contact_record->declaration : nullptr);
        } catch (const std::exception& refusal) {
            report.refusals.emplace_back(refusal.what());
        }
    }

    for (const std::string& body_name : vehicle_project::CollectUnplacedFreeBodyNames(project_)) {
        report.information.push_back("free body '" + body_name +
                                     "' is not placed; the preview draws neither it nor any "
                                     "body jointed to it");
    }
    if (!project_.contact_record.has_value()) {
        report.information.emplace_back("the project has no wheel-rail contact record yet");
    }

    if (report.refusals.empty()) {
        preview_ =
            internal::ComputeVehicleProjectPreview(project_, vehicle, system->model(), revision_);
    }
    return report;
}

}  // namespace orvd::vehicle_project_editing
