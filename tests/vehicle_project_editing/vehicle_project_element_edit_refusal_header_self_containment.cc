#include "orvd/vehicle_project_editing/vehicle_project_element_edit_refusal.h"

#include <optional>
#include <string>
#include <type_traits>

static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::ElementEditRefusal{}.element),
                             orvd::vehicle_project::ElementReference>);
static_assert(
    std::is_same_v<decltype(orvd::vehicle_project_editing::ElementEditRefusal{}.property_name),
                   std::optional<std::string>>);
