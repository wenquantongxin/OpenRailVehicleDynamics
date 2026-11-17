#pragma once

#include <expected>
#include <string>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

namespace orvd::vehicle_project_editing::internal {

// Appends a new element of `kind` named `name` to its kind's records: the
// record type's default-constructed value with that name and the given
// properties, set in the order DescribeElementProperties lists them, so that a
// rigid body made to move freely has its station offset and placement offsets
// to set. The checks are those VehicleProjectEditingSession::AddElement
// states. A refusal may leave the project partly changed.
[[nodiscard]] std::expected<vehicle_project::ElementReference, SubjectEditRefusal> CreateElement(
    vehicle_project::VehicleProject& project, vehicle_project::ElementKind kind,
    const std::string& name, const PropertyValuesByName& properties);

}  // namespace orvd::vehicle_project_editing::internal
