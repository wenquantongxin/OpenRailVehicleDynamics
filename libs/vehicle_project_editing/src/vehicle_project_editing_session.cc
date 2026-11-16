#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"

#include <exception>
#include <memory>
#include <stdexcept>
#include <utility>

#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/generate_assembly_inputs_from_vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_free_body_placements.h"
#include "orvd/vehicle_project/vehicle_project_invariants.h"
#include "vehicle_project_element_creation.h"
#include "vehicle_project_element_property_assignment.h"
#include "vehicle_project_element_removal.h"
#include "vehicle_project_preview_computation.h"

namespace orvd::vehicle_project_editing {

using vehicle_project::ElementReference;

VehicleProjectEditingSession::VehicleProjectEditingSession(
    vehicle_project::VehicleProject project,
    double gravitational_acceleration_magnitude_meters_per_second_squared)
    : project_(std::move(project)),
      gravitational_acceleration_magnitude_meters_per_second_squared_(
          gravitational_acceleration_magnitude_meters_per_second_squared) {}

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

MechanicalCheckReport VehicleProjectEditingSession::CheckMechanics() {
    MechanicalCheckReport report;
    try {
        vehicle_project::RequireVehicleProjectInvariants(project_);
    } catch (const std::exception& refusal) {
        report.refusals.emplace_back(refusal.what());
    }
    // Placements, shape names and groups take no part in generation or
    // assembly, so a refusal above does not keep the assembly from being
    // checked.
    const configuration::VehicleDefinition vehicle =
        vehicle_project::GenerateVehicleDefinition(project_);
    std::unique_ptr<configuration::AssembledVehicleSystem> system;
    try {
        system = configuration::AssembleVehicleSystem(
            vehicle, gravitational_acceleration_magnitude_meters_per_second_squared_,
            project_.contact_record ? &project_.contact_record->declaration : nullptr);
    } catch (const std::exception& refusal) {
        report.refusals.emplace_back(refusal.what());
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
