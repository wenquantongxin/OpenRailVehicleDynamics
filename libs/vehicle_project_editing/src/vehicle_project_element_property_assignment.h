#pragma once

#include <expected>
#include <span>
#include <string>
#include <string_view>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

namespace orvd::vehicle_project_editing::internal {

// How a reason names the kinds a reference may name: "rigid body or fixed
// frame".
[[nodiscard]] std::string DescribeReferencedKinds(
    std::span<const vehicle_project::ElementKind> referenced_kinds);

// Sets one property of one element of `project` and returns the element as it
// is named afterwards, which differs from `element` after a rename. The edit
// is checked in the order VehicleProjectEditingSession::SetSubjectProperty
// states, and a refusal leaves the project unchanged.
[[nodiscard]] std::expected<vehicle_project::ElementReference, SubjectEditRefusal>
AssignElementProperty(vehicle_project::VehicleProject& project,
                      const vehicle_project::ElementReference& element,
                      std::string_view property_name, const PropertyValue& value);

}  // namespace orvd::vehicle_project_editing::internal
