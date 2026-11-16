#pragma once

#include <string>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"

// Which elements name which, read from the references the property tables
// state (vehicle_project_element_property_tables.h). A reference names an element when the
// element's kind is one the reference may name and its name is the one the
// reference holds; the name spaces make that one element at most.

namespace orvd::vehicle_project_editing::internal {

// The elements holding a reference that names `target`, each once.
[[nodiscard]] std::vector<vehicle_project::ElementReference> CollectReferringElements(
    const vehicle_project::VehicleProject& project,
    const vehicle_project::ElementReference& target);

// Makes every reference that names `target` name `new_name` instead.
void RedirectReferences(vehicle_project::VehicleProject& project,
                        const vehicle_project::ElementReference& target,
                        const std::string& new_name);

}  // namespace orvd::vehicle_project_editing::internal
