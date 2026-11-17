#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"

#include <optional>
#include <string>
#include <type_traits>

static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::SubjectEditRefusal{}.subject),
                             orvd::vehicle_project_editing::PropertySubject>);
static_assert(
    std::is_same_v<decltype(orvd::vehicle_project_editing::SubjectEditRefusal{}.property_name),
                   std::optional<std::string>>);
