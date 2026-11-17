#pragma once

#include <expected>
#include <string_view>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

namespace orvd::vehicle_project_editing::internal {

// Sets one property of the project's `settings`. The edit is checked in the
// order VehicleProjectEditingSession::SetSubjectProperty states, and a
// refusal leaves the project unchanged.
[[nodiscard]] std::expected<void, SubjectEditRefusal> AssignSettingsProperty(
    vehicle_project::VehicleProject& project, ProjectSettings settings,
    std::string_view property_name, const PropertyValue& value);

}  // namespace orvd::vehicle_project_editing::internal
