#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

#include <expected>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::DescribeElementProperties(
                                 std::declval<orvd::vehicle_project::ElementKind>())),
                             std::span<const orvd::vehicle_project_editing::PropertyDescription>>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::DescribeSettingsProperties(
                                 std::declval<orvd::vehicle_project_editing::ProjectSettings>())),
                             std::span<const orvd::vehicle_project_editing::PropertyDescription>>);
static_assert(
    std::is_same_v<
        decltype(orvd::vehicle_project_editing::DescribeElementProperty(
            std::declval<orvd::vehicle_project::ElementKind>(), std::declval<std::string_view>())),
        std::expected<const orvd::vehicle_project_editing::PropertyDescription*, std::string>>);
static_assert(
    std::is_same_v<
        decltype(orvd::vehicle_project_editing::DescribeSubjectProperty(
            std::declval<const orvd::vehicle_project_editing::PropertySubject&>(),
            std::declval<std::string_view>())),
        std::expected<const orvd::vehicle_project_editing::PropertyDescription*, std::string>>);
static_assert(std::is_same_v<decltype(orvd::vehicle_project_editing::CollectElementNames(
                                 std::declval<const orvd::vehicle_project::VehicleProject&>(),
                                 std::declval<orvd::vehicle_project::ElementKind>())),
                             std::vector<std::string>>);
static_assert(
    std::is_same_v<decltype(orvd::vehicle_project_editing::ReadSubjectProperties(
                       std::declval<const orvd::vehicle_project::VehicleProject&>(),
                       std::declval<const orvd::vehicle_project_editing::PropertySubject&>())),
                   std::vector<orvd::vehicle_project_editing::PropertyReading>>);
