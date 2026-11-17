#include "vehicle_project_settings_property_assignment.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <variant>

#include "vehicle_project_property_edit_value_checks.h"
#include "vehicle_project_settings_property_tables.h"

namespace orvd::vehicle_project_editing::internal {
namespace {

using vehicle_project::VehicleProject;

// A number is written as given when it is finite; a number absent until it is
// stated is stated so. Nothing here judges a settings number further: the
// check judges what it covers, and a run what it assembles.
template <typename Record, typename Number>
std::expected<void, std::string> AssignSettingsMember(
    Record& record, std::string_view property_name,
    const SettingsRecordMember<Record, Number>& member, const PropertyValue& value) {
    const double given = std::get<double>(value);
    if (!std::isfinite(given)) {
        return std::unexpected(QuotePropertyName(property_name) + " must be finite");
    }
    member.in_mutable(record) = given;
    return {};
}

// A token must be one of its choices.
template <typename Record, typename Value>
std::expected<void, std::string> AssignSettingsMember(
    Record& record, std::string_view property_name, const SettingsTokenMember<Record, Value>& token,
    const PropertyValue& value) {
    const std::string& given = std::get<PropertyToken>(value).token;
    const auto chosen = std::ranges::find(token.tokens, given);
    if (chosen == token.tokens.end()) {
        return std::unexpected(RefuseTokenOutsideChoices(property_name, token.tokens, given));
    }
    token.member.in_mutable(record) =
        token.values[static_cast<std::size_t>(chosen - token.tokens.begin())];
    return {};
}

template <typename Records>
std::expected<void, SubjectEditRefusal> Assign(VehicleProject& project, ProjectSettings settings,
                                               std::string_view property_name,
                                               const PropertyValue& value) {
    using Record = typename Records::Record;
    const auto refuse = [settings, property_name](std::string reason) {
        return std::unexpected(
            SubjectEditRefusal{settings, std::string(property_name), std::move(reason)});
    };
    Record* const record = Records::RecordIn(project);
    if (record == nullptr) {
        return refuse(DescribeMissingSettingsRecord(settings));
    }
    const std::expected<const PropertyDescription*, std::string> described =
        DescribeSubjectProperty(settings, property_name);
    if (!described.has_value()) {
        return refuse(described.error());
    }
    const std::expected<void, std::string> editable_with_its_kind =
        RequireEditableAndGivenItsValueKind(**described, value);
    if (!editable_with_its_kind.has_value()) {
        return refuse(editable_with_its_kind.error());
    }

    // The settings' descriptions list their table's properties in its order,
    // and an editable description is an editable member's.
    const auto index =
        static_cast<std::size_t>(*described - DescribeSettingsProperties(settings).data());
    const std::expected<void, std::string> assigned = std::visit(
        [record, property_name, &value](const auto& member) {
            return AssignSettingsMember(*record, property_name, member, value);
        },
        std::get<EditableSettingsMember<Record>>(kSettingsRecordProperties<Record>[index].member));
    if (!assigned.has_value()) {
        return refuse(assigned.error());
    }
    return {};
}

}  // namespace

std::expected<void, SubjectEditRefusal> AssignSettingsProperty(VehicleProject& project,
                                                               ProjectSettings settings,
                                                               std::string_view property_name,
                                                               const PropertyValue& value) {
    return VisitProjectSettings(settings, [&]<typename Records>(Records) {
        return Assign<Records>(project, settings, property_name, value);
    });
}

}  // namespace orvd::vehicle_project_editing::internal
