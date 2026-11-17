#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"

#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

static_assert(std::is_constructible_v<orvd::vehicle_project_editing::VehicleProjectEditingSession,
                                      orvd::vehicle_project::VehicleProject>);
static_assert(std::is_constructible_v<orvd::vehicle_project_editing::VehicleProjectEditingSession,
                                      orvd::vehicle_project::VehicleProject,
                                      orvd::vehicle_project::VehicleProjectFileLocation>);
static_assert(
    std::is_same_v<
        decltype(std::declval<const orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                     .associated_file()),
        const std::optional<orvd::vehicle_project::VehicleProjectFileLocation>&>);
static_assert(
    std::is_same_v<
        decltype(std::declval<const orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                     .has_unsaved_changes()),
        bool>);
static_assert(std::is_same_v<
              decltype(std::declval<orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                           .SaveVehicleProject()),
              void>);
static_assert(
    std::is_same_v<
        decltype(std::declval<orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                     .SaveVehicleProjectAs(
                         std::declval<const orvd::vehicle_project::VehicleProjectFileLocation&>())),
        void>);
static_assert(
    std::is_same_v<
        decltype(std::declval<orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                     .SetSubjectProperty(
                         std::declval<const orvd::vehicle_project_editing::PropertySubject&>(),
                         std::declval<std::string_view>(),
                         std::declval<const orvd::vehicle_project_editing::PropertyValue&>())),
        std::expected<orvd::vehicle_project_editing::PropertySubject,
                      orvd::vehicle_project_editing::SubjectEditRefusal>>);
static_assert(std::is_same_v<
              decltype(std::declval<orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                           .AddElement(std::declval<orvd::vehicle_project::ElementKind>(),
                                       std::declval<const std::string&>(),
                                       std::declval<const orvd::vehicle_project_editing::
                                                        PropertyValuesByName&>())),
              std::expected<orvd::vehicle_project::ElementReference,
                            orvd::vehicle_project_editing::SubjectEditRefusal>>);
static_assert(std::is_same_v<
              decltype(std::declval<orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                           .RemoveElement(
                               std::declval<const orvd::vehicle_project::ElementReference&>())),
              std::expected<std::vector<orvd::vehicle_project::ElementReference>,
                            orvd::vehicle_project_editing::SubjectEditRefusal>>);
static_assert(
    std::is_same_v<
        decltype(std::declval<const orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                     .CollectRemovalConsequences(
                         std::declval<const orvd::vehicle_project::ElementReference&>())),
        std::expected<std::vector<orvd::vehicle_project::ElementReference>,
                      orvd::vehicle_project_editing::SubjectEditRefusal>>);
static_assert(std::is_same_v<
              decltype(std::declval<orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                           .CheckMechanics()),
              orvd::vehicle_project_editing::MechanicalCheckReport>);
static_assert(
    std::is_same_v<
        decltype(std::declval<const orvd::vehicle_project_editing::VehicleProjectEditingSession&>()
                     .preview()),
        const std::optional<orvd::vehicle_project_editing::VehicleProjectPreview>&>);
