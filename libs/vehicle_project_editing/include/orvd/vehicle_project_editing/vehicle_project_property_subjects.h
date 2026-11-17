#pragma once

#include <array>
#include <optional>
#include <string_view>
#include <variant>

#include "orvd/vehicle_project/vehicle_project.h"

// What a property belongs to: an element of the project, or one of the
// project's settings. Settings are records of the project that are not
// elements: they are read and edited through properties as an element is, but
// they are never created, removed, renamed or named by a reference, so
// nothing an element alone does reaches them.

namespace orvd::vehicle_project_editing {

// The settings a project holds besides its elements.
enum class ProjectSettings {
    // The magnitude of the gravitational acceleration the vehicle is checked
    // under.
    kGravity,
    // The vehicle definition's vehicle name and mechanical definition
    // identifier, which saving a project as a named project sets.
    kVehicleIdentity,
    // The mechanical track station layout's reference body, which every free
    // body's station offset is measured from.
    kStationLayout,
    // The wheel, the rail and the contact computation of the wheel-rail
    // contact record, which a project without the record has none of.
    kWheel,
    kRail,
    kContactComputation,
};

// Every settings, in the order ProjectSettings declares them.
inline constexpr std::array kProjectSettings{
    ProjectSettings::kGravity,       ProjectSettings::kVehicleIdentity,
    ProjectSettings::kStationLayout, ProjectSettings::kWheel,
    ProjectSettings::kRail,          ProjectSettings::kContactComputation,
};

// An element, or one of the project's settings.
using PropertySubject = std::variant<vehicle_project::ElementReference, ProjectSettings>;

// The word the modelling service writes the settings as: "gravity",
// "vehicle_identity", "station_layout", "wheel", "rail" or
// "contact_computation".
[[nodiscard]] std::string_view ProjectSettingsWord(ProjectSettings settings);

// The settings written as `word`, or none when no settings are.
[[nodiscard]] std::optional<ProjectSettings> FindProjectSettingsWrittenAs(std::string_view word);

}  // namespace orvd::vehicle_project_editing
