#pragma once

#include <expected>
#include <string>
#include <string_view>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/element_properties.h"

namespace orvd::vehicle_project_editing::internal {

// Checks one property edit in the order
// VehicleProjectEditingSession::SetElementProperty states and, when it is
// admissible, makes it. A refusal returns what is wrong and leaves the project
// unchanged.
[[nodiscard]] std::expected<void, std::string> AssignElementProperty(
    vehicle_project::VehicleProject& project, const vehicle_project::ElementReference& element,
    std::string_view property_name, const PropertyValue& value);

}  // namespace orvd::vehicle_project_editing::internal
