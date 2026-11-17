#include "orvd/vehicle_project_editing/vehicle_project_titles_in_english_and_chinese.h"

#include <string>
#include <type_traits>

static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::Title{}.en), std::string>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::Title{}.zh), std::string>);
