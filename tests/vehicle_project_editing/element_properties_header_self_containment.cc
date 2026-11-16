#include "orvd/vehicle_project_editing/element_properties.h"

#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

static_assert(
    std::is_same_v<decltype(orvd::vehicle_project_editing::DescribeElementProperties(
                       std::declval<orvd::vehicle_project::ElementKind>())),
                   std::span<const orvd::vehicle_project_editing::ElementPropertyDescription>>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::CollectElementNames(
                                 std::declval<const orvd::vehicle_project::VehicleProject&>(),
                                 std::declval<orvd::vehicle_project::ElementKind>())),
                             std::vector<std::string>>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::ReadElementProperties(
                                 std::declval<const orvd::vehicle_project::VehicleProject&>(),
                                 std::declval<const orvd::vehicle_project::ElementReference&>())),
                             std::vector<orvd::vehicle_project_editing::ElementPropertyReading>>);
