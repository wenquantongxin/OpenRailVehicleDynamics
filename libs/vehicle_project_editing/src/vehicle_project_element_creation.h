#pragma once

#include <expected>
#include <string>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

namespace orvd::vehicle_project_editing::internal {

// Appends a new element of `kind` named `name` to its kind's records: the
// record type's default-constructed value with that name and the given
// properties, set in the order DescribeElementProperties lists them. The
// checks are those VehicleProjectEditingSession::AddElement states, for every
// kind: a rigid body and a joint are created through their attachment
// (vehicle_project_rigid_body_attachment_changes.h), which calls this. A
// refusal may leave the project partly changed.
[[nodiscard]] std::expected<vehicle_project::ElementReference, SubjectEditRefusal> CreateElement(
    vehicle_project::VehicleProject& project, vehicle_project::ElementKind kind,
    const std::string& name, const PropertyValuesByName& properties);

}  // namespace orvd::vehicle_project_editing::internal
