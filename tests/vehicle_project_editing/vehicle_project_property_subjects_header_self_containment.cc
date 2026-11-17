#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"

#include <optional>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

static_assert(std::is_same_v<orvd::vehicle_project_editing::PropertySubject,
                             std::variant<orvd::vehicle_project::ElementReference,
                                          orvd::vehicle_project_editing::ProjectSettings>>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::ProjectSettingsWord(
                                 std::declval<orvd::vehicle_project_editing::ProjectSettings>())),
                             std::string_view>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::FindProjectSettingsWrittenAs(
                                 std::declval<std::string_view>())),
                             std::optional<orvd::vehicle_project_editing::ProjectSettings>>);
