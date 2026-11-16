#include "orvd/vehicle_project_editing/vehicle_project_element_properties.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_project_element_property_tables.h"
#include "vehicle_project_records_following_elements.h"

namespace orvd::vehicle_project_editing {
namespace {

using configuration::VehicleRigidBodyDefinition;
using internal::kBodyKeyedProperties;
using internal::kRecordProperties;
using internal::RecordProperty;
using internal::ReferenceMember;
using internal::TextTokenMember;
using internal::ValueTokenMember;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;
using vehicle_project::VisitElementKind;

// Every property can be edited except a rotation, which nothing edits yet.
ElementPropertyDescription Described(std::string_view name, PropertyValueKind value_kind) {
    return {.name = name,
            .value_kind = value_kind,
            .editable = value_kind != PropertyValueKind::kRotation};
}

// The description of each kind of member.

template <typename Record>
ElementPropertyDescription DescribeMember(std::string_view name, double Record::*) {
    return Described(name, PropertyValueKind::kNumber);
}

template <typename Record>
ElementPropertyDescription DescribeMember(std::string_view name, Eigen::Vector3d Record::*) {
    return Described(name, PropertyValueKind::kVector3);
}

template <typename Record>
ElementPropertyDescription DescribeMember(std::string_view name, bool Record::*) {
    return Described(name, PropertyValueKind::kBoolean);
}

template <typename Record>
ElementPropertyDescription DescribeMember(std::string_view name, Eigen::Matrix3d Record::*) {
    return Described(name, PropertyValueKind::kRotation);
}

// The table's one plain text member is the element's own name.
template <typename Record>
ElementPropertyDescription DescribeMember(std::string_view name, std::string Record::*) {
    return Described(name, PropertyValueKind::kElementName);
}

template <typename Record, typename Name>
ElementPropertyDescription DescribeMember(std::string_view name,
                                          const ReferenceMember<Record, Name>& reference) {
    ElementPropertyDescription description = Described(name, PropertyValueKind::kElementReference);
    description.referenced_kinds = reference.referenced_kinds;
    description.may_name_none = std::is_same_v<Name, std::optional<std::string>>;
    return description;
}

template <typename Record>
ElementPropertyDescription DescribeMember(std::string_view name,
                                          const TextTokenMember<Record>& token) {
    ElementPropertyDescription description = Described(name, PropertyValueKind::kToken);
    description.choices = token.tokens;
    return description;
}

template <typename Record, typename Value>
ElementPropertyDescription DescribeMember(std::string_view name,
                                          const ValueTokenMember<Record, Value>& token) {
    ElementPropertyDescription description = Described(name, PropertyValueKind::kToken);
    description.choices = token.tokens;
    return description;
}

template <typename Record>
ElementPropertyDescription DescribeMember(std::string_view name,
                                          SaturatedPiecewiseLinearDamperCurve Record::*) {
    ElementPropertyDescription description =
        Described(name, PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve);
    description.row_fields = internal::kSaturatedPiecewiseLinearDamperCurveRowFields;
    return description;
}

// The record's properties, then, for a rigid body, those keyed by its name.
template <typename KindRecords>
std::span<const ElementPropertyDescription> Descriptions() {
    using Record = typename KindRecords::Record;
    static_assert(internal::ListsOwnNameAsTheOnlyPlainText<KindRecords>(),
                  "a record table's one plain text member must be the element's own name");
    static const std::vector<ElementPropertyDescription> descriptions = [] {
        std::vector<ElementPropertyDescription> described;
        for (const RecordProperty<Record>& property : kRecordProperties<Record>) {
            described.push_back(std::visit(
                [&property](const auto& member) { return DescribeMember(property.name, member); },
                property.member));
        }
        if constexpr (std::is_same_v<Record, VehicleRigidBodyDefinition>) {
            for (const internal::BodyKeyedProperty& property : kBodyKeyedProperties) {
                described.push_back(Described(property.name, PropertyValueKind::kNumber));
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

template <typename Record, typename Value>
PropertyValue ReadMember(const Record& record, const ValueTokenMember<Record, Value>& token) {
    const auto stated = std::ranges::find(token.values, record.*token.member);
    if (stated == token.values.end()) {
        // Reached only by a value cast into the member's type that none of
        // the tokens stands for.
        throw std::logic_error("a token member holds a value no token stands for");
    }
    return PropertyToken{
        std::string(token.tokens[static_cast<std::size_t>(stated - token.values.begin())])};
}

template <typename KindRecords>
std::vector<ElementPropertyReading> ReadProperties(const VehicleProject& project,
                                                   const ElementReference& element) {
    using Record = typename KindRecords::Record;
    const Record* const record =
        vehicle_project::FindElementRecord<KindRecords>(project, element.name);
    if (record == nullptr) {
        throw std::invalid_argument("this project contains no " +
                                    vehicle_project::DescribeElement(element));
    }
    // The descriptions list the properties in the order these loops read them.
    const std::span<const ElementPropertyDescription> descriptions = Descriptions<KindRecords>();
    std::vector<ElementPropertyReading> readings;
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
    for (ElementPropertyReading& reading : readings) {
        reading.description.editable = reading.description.editable && reading.value.has_value();
    }
    return readings;
}

}  // namespace

std::span<const ElementPropertyDescription> DescribeElementProperties(ElementKind kind) {
    return VisitElementKind(
        kind, []<typename KindRecords>(KindRecords) { return Descriptions<KindRecords>(); });
}

std::expected<const ElementPropertyDescription*, std::string> DescribeElementProperty(
    ElementKind kind, std::string_view property_name) {
    const std::span<const ElementPropertyDescription> descriptions =
        DescribeElementProperties(kind);
    const auto described =
        std::ranges::find(descriptions, property_name, &ElementPropertyDescription::name);
    if (described == descriptions.end()) {
        return std::unexpected("a " + std::string(vehicle_project::DescribeElementKind(kind)) +
                               " has no property named '" + std::string(property_name) + "'");
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

std::vector<ElementPropertyReading> ReadElementProperties(const VehicleProject& project,
                                                          const ElementReference& element) {
    return VisitElementKind(element.kind, [&project, &element]<typename KindRecords>(KindRecords) {
        return ReadProperties<KindRecords>(project, element);
    });
}

}  // namespace orvd::vehicle_project_editing
