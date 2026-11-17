#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_project_element_property_tables.h"
#include "vehicle_project_records_following_elements.h"
#include "vehicle_project_settings_property_tables.h"

namespace orvd::vehicle_project_editing {
namespace {

using configuration::VehicleRigidBodyDefinition;
using internal::EditableSettingsMember;
using internal::kBodyKeyedProperties;
using internal::kRecordProperties;
using internal::kSettingsRecordProperties;
using internal::ReadOnlySettingsMember;
using internal::RecordProperty;
using internal::ReferenceMember;
using internal::SettingsRecordMember;
using internal::SettingsRecordProperty;
using internal::SettingsReferenceMember;
using internal::SettingsTokenMember;
using internal::TextTokenMember;
using internal::ValueTokenMember;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;
using vehicle_project::VisitElementKind;

// Every property can be edited except a rotation, which nothing edits yet.
PropertyDescription Described(std::string_view name, PropertyValueKind value_kind) {
    return {.name = name,
            .value_kind = value_kind,
            .editable = value_kind != PropertyValueKind::kRotation};
}

// The description of each kind of member.

template <typename Record>
PropertyDescription DescribeMember(std::string_view name, double Record::*) {
    return Described(name, PropertyValueKind::kNumber);
}

template <typename Record>
PropertyDescription DescribeMember(std::string_view name, Eigen::Vector3d Record::*) {
    return Described(name, PropertyValueKind::kVector3);
}

template <typename Record>
PropertyDescription DescribeMember(std::string_view name, Eigen::Matrix3d Record::*) {
    return Described(name, PropertyValueKind::kRotation);
}

// The table's one plain text member is the element's own name.
template <typename Record>
PropertyDescription DescribeMember(std::string_view name, std::string Record::*) {
    return Described(name, PropertyValueKind::kElementName);
}

template <typename Record, typename Name>
PropertyDescription DescribeMember(std::string_view name,
                                   const ReferenceMember<Record, Name>& reference) {
    PropertyDescription description = Described(name, PropertyValueKind::kElementReference);
    description.referenced_kinds = reference.referenced_kinds;
    description.may_name_none = std::is_same_v<Name, std::optional<std::string>>;
    return description;
}

template <typename Record>
PropertyDescription DescribeMember(std::string_view name, const TextTokenMember<Record>& token) {
    PropertyDescription description = Described(name, PropertyValueKind::kToken);
    description.choices = token.tokens;
    return description;
}

template <typename Record, typename Value>
PropertyDescription DescribeMember(std::string_view name,
                                   const ValueTokenMember<Record, Value>& token) {
    PropertyDescription description = Described(name, PropertyValueKind::kToken);
    description.choices = token.tokens;
    return description;
}

template <typename Record>
PropertyDescription DescribeMember(std::string_view name,
                                   SaturatedPiecewiseLinearDamperCurve Record::*) {
    PropertyDescription description =
        Described(name, PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve);
    description.row_fields = internal::kSaturatedPiecewiseLinearDamperCurveRowFields;
    return description;
}

std::optional<Title> DescribeSection(std::optional<internal::PropertySection> section) {
    if (!section.has_value()) {
        return std::nullopt;
    }
    return internal::DescribePropertySection(*section);
}

// The record's properties, then, for a rigid body, those keyed by its name.
template <typename KindRecords>
std::span<const PropertyDescription> Descriptions() {
    using Record = typename KindRecords::Record;
    static_assert(internal::ListsOwnNameAsTheOnlyPlainText<KindRecords>(),
                  "a record table's one plain text member must be the element's own name");
    static const std::vector<PropertyDescription> descriptions = [] {
        std::vector<PropertyDescription> described;
        for (const RecordProperty<Record>& property : kRecordProperties<Record>) {
            described.push_back(std::visit(
                [&property](const auto& member) { return DescribeMember(property.name, member); },
                property.member));
            described.back().section = DescribeSection(property.section);
        }
        if constexpr (std::is_same_v<Record, VehicleRigidBodyDefinition>) {
            for (const internal::BodyKeyedProperty& property : kBodyKeyedProperties) {
                described.push_back(Described(property.name, PropertyValueKind::kNumber));
                described.back().section = DescribeSection(property.section);
            }
        }
        return described;
    }();
    return descriptions;
}

// The value each kind of member holds, as a property value.

template <typename Record, typename Value>
PropertyValue ReadMember(const Record& record, Value Record::* member) {
    return PropertyValue(std::in_place_type<Value>, record.*member);
}

template <typename Record>
PropertyValue ReadMember(const Record& record, std::string Record::* member) {
    return ElementOwnName{record.*member};
}

template <typename Record, typename Name>
PropertyValue ReadMember(const Record& record, const ReferenceMember<Record, Name>& reference) {
    return ReferencedElementName{record.*reference.member};
}

template <typename Record>
PropertyValue ReadMember(const Record& record, const TextTokenMember<Record>& token) {
    return PropertyToken{record.*token.member};
}

// The token standing for `value`, among `tokens` standing for `values` at
// their positions.
template <typename Value>
PropertyToken TokenStandingFor(const Value& value, std::span<const std::string_view> tokens,
                               std::span<const Value> values) {
    const auto stated = std::ranges::find(values, value);
    if (stated == values.end()) {
        // Reached only by a value cast into the member's type that none of
        // the tokens stands for.
        throw std::logic_error("a token member holds a value no token stands for");
    }
    return PropertyToken{std::string(tokens[static_cast<std::size_t>(stated - values.begin())])};
}

template <typename Record, typename Value>
PropertyValue ReadMember(const Record& record, const ValueTokenMember<Record, Value>& token) {
    return TokenStandingFor(record.*token.member, token.tokens, token.values);
}

template <typename KindRecords>
std::vector<PropertyReading> ReadProperties(const VehicleProject& project,
                                            const ElementReference& element) {
    using Record = typename KindRecords::Record;
    const Record* const record =
        vehicle_project::FindElementRecord<KindRecords>(project, element.name);
    if (record == nullptr) {
        throw std::invalid_argument("this project contains no " +
                                    vehicle_project::DescribeElement(element));
    }
    // The descriptions list the properties in the order these loops read them.
    const std::span<const PropertyDescription> descriptions = Descriptions<KindRecords>();
    std::vector<PropertyReading> readings;
    readings.reserve(descriptions.size());
    for (const RecordProperty<Record>& property : kRecordProperties<Record>) {
        readings.push_back(
            {descriptions[readings.size()],
             std::visit([record](const auto& member) { return ReadMember(*record, member); },
                        property.member)});
    }
    if constexpr (std::is_same_v<Record, VehicleRigidBodyDefinition>) {
        for (const internal::BodyKeyedProperty& property : kBodyKeyedProperties) {
            readings.push_back(
                {descriptions[readings.size()],
                 std::visit(
                     [&project, &element](auto member) -> std::optional<PropertyValue> {
                         const auto* const keyed =
                             internal::FindBodyKeyedRecord(project, element.name, member);
                         if (keyed == nullptr) {
                             return std::nullopt;
                         }
                         return keyed->*member;
                     },
                     property.member)});
        }
    }
    // A property whose value is absent for this element is not editable for
    // it. The assignment refuses exactly these edits: a station offset or
    // placement offset whose record is missing is the only editable property
    // a value can be absent from.
    for (PropertyReading& reading : readings) {
        reading.description.editable = reading.description.editable && reading.value.has_value();
    }
    return readings;
}

// The description of each kind of settings member, by what it holds: a
// number, a number absent until stated and a count are numbers, and a text
// and a path are texts.

template <typename Value>
constexpr PropertyValueKind SettingsValueKind() {
    if constexpr (std::is_same_v<Value, std::string> ||
                  std::is_same_v<Value, std::filesystem::path>) {
        return PropertyValueKind::kText;
    } else {
        static_assert(std::is_same_v<Value, double> ||
                          std::is_same_v<Value, std::optional<double>> ||
                          std::is_same_v<Value, std::size_t>,
                      "a settings member holds a number, a count or a text");
        return PropertyValueKind::kNumber;
    }
}

template <typename Record, typename Value>
PropertyDescription DescribeSettingsMember(std::string_view name,
                                           const SettingsRecordMember<Record, Value>&) {
    return {.name = name, .value_kind = SettingsValueKind<Value>()};
}

template <typename Record, typename Value>
PropertyDescription DescribeSettingsMember(std::string_view name,
                                           const SettingsTokenMember<Record, Value>& token) {
    return {.name = name, .value_kind = PropertyValueKind::kToken, .choices = token.tokens};
}

template <typename Record>
PropertyDescription DescribeSettingsMember(std::string_view name,
                                           const SettingsReferenceMember<Record>& reference) {
    return {.name = name,
            .value_kind = PropertyValueKind::kElementReference,
            .referenced_kinds = reference.referenced_kinds};
}

template <typename Records>
std::span<const PropertyDescription> SettingsDescriptions() {
    using Record = typename Records::Record;
    static const std::vector<PropertyDescription> descriptions = [] {
        std::vector<PropertyDescription> described;
        for (const SettingsRecordProperty<Record>& property : kSettingsRecordProperties<Record>) {
            std::visit(
                [&described, &property](const auto& held) {
                    described.push_back(std::visit(
                        [&property](const auto& member) {
                            return DescribeSettingsMember(property.name, member);
                        },
                        held));
                },
                property.member);
            described.back().editable = property.editable();
        }
        return described;
    }();
    return descriptions;
}

// The value each kind of settings member holds, as a property value: a
// count as a number, a path as its text.

template <typename Record>
std::optional<PropertyValue> ReadSettingsMember(
    const Record& record, const SettingsRecordMember<Record, double>& member) {
    return member.in_const(record);
}

template <typename Record>
std::optional<PropertyValue> ReadSettingsMember(
    const Record& record, const SettingsRecordMember<Record, std::optional<double>>& member) {
    const std::optional<double>& stated = member.in_const(record);
    if (!stated.has_value()) {
        return std::nullopt;
    }
    return *stated;
}

template <typename Record>
std::optional<PropertyValue> ReadSettingsMember(
    const Record& record, const SettingsRecordMember<Record, std::size_t>& member) {
    return static_cast<double>(member.in_const(record));
}

template <typename Record>
std::optional<PropertyValue> ReadSettingsMember(
    const Record& record, const SettingsRecordMember<Record, std::string>& member) {
    return PropertyText{member.in_const(record)};
}

template <typename Record>
std::optional<PropertyValue> ReadSettingsMember(
    const Record& record, const SettingsRecordMember<Record, std::filesystem::path>& member) {
    return PropertyText{member.in_const(record).string()};
}

template <typename Record, typename Value>
std::optional<PropertyValue> ReadSettingsMember(const Record& record,
                                                const SettingsTokenMember<Record, Value>& token) {
    return TokenStandingFor(token.member.in_const(record), token.tokens, token.values);
}

template <typename Record>
std::optional<PropertyValue> ReadSettingsMember(const Record& record,
                                                const SettingsReferenceMember<Record>& reference) {
    return ReferencedElementName{reference.member.in_const(record)};
}

template <typename Records>
std::vector<PropertyReading> ReadSettingsProperties(const VehicleProject& project,
                                                    ProjectSettings settings) {
    using Record = typename Records::Record;
    const Record* const record = Records::RecordIn(project);
    if (record == nullptr) {
        throw std::invalid_argument(internal::DescribeMissingSettingsRecord(settings));
    }
    // The descriptions list the properties in the order this loop reads them.
    const std::span<const PropertyDescription> descriptions = SettingsDescriptions<Records>();
    std::vector<PropertyReading> readings;
    readings.reserve(descriptions.size());
    for (const SettingsRecordProperty<Record>& property : kSettingsRecordProperties<Record>) {
        readings.push_back({descriptions[readings.size()],
                            std::visit(
                                [record](const auto& held) {
                                    return std::visit(
                                        [record](const auto& member) {
                                            return ReadSettingsMember(*record, member);
                                        },
                                        held);
                                },
                                property.member)});
    }
    return readings;
}

}  // namespace

std::span<const PropertyDescription> DescribeElementProperties(ElementKind kind) {
    return VisitElementKind(
        kind, []<typename KindRecords>(KindRecords) { return Descriptions<KindRecords>(); });
}

std::span<const PropertyDescription> DescribeSettingsProperties(ProjectSettings settings) {
    return internal::VisitProjectSettings(
        settings, []<typename Records>(Records) { return SettingsDescriptions<Records>(); });
}

std::expected<const PropertyDescription*, std::string> DescribeElementProperty(
    ElementKind kind, std::string_view property_name) {
    const std::span<const PropertyDescription> descriptions = DescribeElementProperties(kind);
    const auto described =
        std::ranges::find(descriptions, property_name, &PropertyDescription::name);
    if (described == descriptions.end()) {
        return std::unexpected("a " + std::string(vehicle_project::DescribeElementKind(kind)) +
                               " has no property named '" + std::string(property_name) + "'");
    }
    return std::to_address(described);
}

std::expected<const PropertyDescription*, std::string> DescribeSubjectProperty(
    const PropertySubject& subject, std::string_view property_name) {
    if (const auto* const element = std::get_if<ElementReference>(&subject)) {
        return DescribeElementProperty(element->kind, property_name);
    }
    const ProjectSettings settings = std::get<ProjectSettings>(subject);
    const std::span<const PropertyDescription> descriptions = DescribeSettingsProperties(settings);
    const auto described =
        std::ranges::find(descriptions, property_name, &PropertyDescription::name);
    if (described == descriptions.end()) {
        return std::unexpected("the " + std::string(internal::DescribeProjectSettings(settings)) +
                               " have no property named '" + std::string(property_name) + "'");
    }
    return std::to_address(described);
}

std::vector<std::string> CollectElementNames(const VehicleProject& project, ElementKind kind) {
    return VisitElementKind(kind, [&project]<typename KindRecords>(KindRecords) {
        std::vector<std::string> names;
        if (const auto* const records = KindRecords::Records(project); records != nullptr) {
            for (const typename KindRecords::Record& record : *records) {
                names.push_back(record.*KindRecords::kElementName);
            }
        }
        return names;
    });
}

std::vector<PropertyReading> ReadSubjectProperties(const VehicleProject& project,
                                                   const PropertySubject& subject) {
    if (const auto* const element = std::get_if<ElementReference>(&subject)) {
        return VisitElementKind(element->kind,
                                [&project, element]<typename KindRecords>(KindRecords) {
                                    return ReadProperties<KindRecords>(project, *element);
                                });
    }
    const ProjectSettings settings = std::get<ProjectSettings>(subject);
    return internal::VisitProjectSettings(
        settings, [&project, settings]<typename Records>(Records) {
            return ReadSettingsProperties<Records>(project, settings);
        });
}

}  // namespace orvd::vehicle_project_editing
