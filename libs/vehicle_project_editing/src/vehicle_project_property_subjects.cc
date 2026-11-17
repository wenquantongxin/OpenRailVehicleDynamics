#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"

#include <algorithm>

#include "vehicle_project_settings_property_tables.h"

namespace orvd::vehicle_project_editing {

std::string_view ProjectSettingsWord(ProjectSettings settings) {
    return internal::VisitProjectSettings(
        settings, []<typename Records>(Records) { return Records::kSettingsWord; });
}

std::optional<ProjectSettings> FindProjectSettingsWrittenAs(std::string_view word) {
    const auto written = std::ranges::find_if(kProjectSettings, [word](ProjectSettings settings) {
        return ProjectSettingsWord(settings) == word;
    });
    if (written == kProjectSettings.end()) {
        return std::nullopt;
    }
    return *written;
}

}  // namespace orvd::vehicle_project_editing
