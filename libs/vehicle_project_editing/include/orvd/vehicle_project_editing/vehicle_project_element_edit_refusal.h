#pragma once

#include <optional>
#include <string>

#include "orvd/vehicle_project/vehicle_project.h"

namespace orvd::vehicle_project_editing {

// Why an edit of a project's elements was refused: the element it concerns,
// the property at fault when one is, and what is wrong. A refused creation
// names the element that was to be created; a refused removal names no
// property.
struct ElementEditRefusal {
    vehicle_project::ElementReference element;
    std::optional<std::string> property_name;
    std::string reason;
};

}  // namespace orvd::vehicle_project_editing
