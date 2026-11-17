#pragma once

#include <expected>
#include <span>
#include <vector>

#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"

namespace orvd::vehicle_project_editing::internal {

// Every element removing `element` takes with it: `element` itself and every
// element holding a reference to one taken, transitively, in outline order,
// kind by kind in ElementKind's order and each kind in declaration order. The
// checks are those VehicleProjectEditingSession::RemoveElement states.
[[nodiscard]] std::expected<std::vector<vehicle_project::ElementReference>, SubjectEditRefusal>
CollectElementsRemovedWith(const vehicle_project::VehicleProject& project,
                           const vehicle_project::ElementReference& element);

// Removes `elements` and the records following each of them.
void RemoveElements(vehicle_project::VehicleProject& project,
                    std::span<const vehicle_project::ElementReference> elements);

}  // namespace orvd::vehicle_project_editing::internal
