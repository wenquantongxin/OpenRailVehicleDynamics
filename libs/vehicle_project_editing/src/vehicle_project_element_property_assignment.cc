#include "vehicle_project_element_property_assignment.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_project_element_name_spaces.h"
#include "vehicle_project_element_property_tables.h"
#include "vehicle_project_element_references.h"
#include "vehicle_project_records_following_elements.h"

namespace orvd::vehicle_project_editing::internal {
namespace {

using configuration::VehicleFreeBodyStationOffsetDefinition;
using configuration::VehicleRigidBodyDefinition;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::FreeBodyPlacement;
using vehicle_project::VehicleProject;

// The value kind of each type a property value holds.
constexpr PropertyValueKind ValueKindOf(std::type_identity<double>) {
    return PropertyValueKind::kNumber;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<Eigen::Vector3d>) {
    return PropertyValueKind::kVector3;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<bool>) {
    return PropertyValueKind::kBoolean;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<Eigen::Matrix3d>) {
    return PropertyValueKind::kRotation;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<ElementOwnName>) {
    return PropertyValueKind::kElementName;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<ReferencedElementName>) {
    return PropertyValueKind::kElementReference;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<PropertyToken>) {
    return PropertyValueKind::kToken;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<SaturatedPiecewiseLinearDamperCurve>) {
    return PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve;
}

PropertyValueKind ValueKindOf(const PropertyValue& value) {
    return std::visit(
        []<typename Held>(const Held&) { return ValueKindOf(std::type_identity<Held>{}); }, value);
}

// How a refusal names a value kind.
std::string DescribeValueKind(PropertyValueKind kind) {
    switch (kind) {
        case PropertyValueKind::kNumber:
            return "a number";
        case PropertyValueKind::kVector3:
            return "a 3-vector";
        case PropertyValueKind::kBoolean:
            return "a boolean";
        case PropertyValueKind::kRotation:
            return "a rotation";
        case PropertyValueKind::kElementName:
            return "an element's own name";
        case PropertyValueKind::kElementReference:
            return "a reference to an element";
        case PropertyValueKind::kToken:
            return "a token";
        case PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve:
            return "a saturated piecewise-linear damper curve";
    }
    // Reached only by a value cast into PropertyValueKind that names none of
    // its enumerators.
    return "a value of no known kind";
}

std::string Quoted(std::string_view text) {
    return "'" + std::string(text) + "'";
}

// Whether every number a value holds is finite.
bool AllFinite(double number) {
    return std::isfinite(number);
}
bool AllFinite(const Eigen::Vector3d& vector) {
    return vector.allFinite();
}
bool AllFinite(const Eigen::Matrix3d& rotation) {
    return rotation.allFinite();
}
bool AllFinite(const SaturatedPiecewiseLinearDamperCurve& curve) {
    return std::ranges::all_of(
        curve,
        [](const configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition& point) {
            return std::isfinite(point.relative_velocity_meters_per_second) &&
                   std::isfinite(point.force_newtons);
        });
}

// One property edit of one element: what the assignments below share. The
// value already holds the property's value kind.
template <typename Record>
struct MemberEdit {
    VehicleProject& project;
    const ElementReference& element;
    Record& record;
    std::string quoted_property_name;
};

// A number, a 3-vector or a curve is written as given when every number in it
// is finite. A rotation would be too, but no rotation is editable.
template <typename Record, typename Value>
std::expected<void, std::string> AssignMember(const MemberEdit<Record>& edit,
                                              Value Record::* member, const PropertyValue& value) {
    const Value& given = std::get<Value>(value);
    if (!AllFinite(given)) {
        return std::unexpected(edit.quoted_property_name + " must be finite");
    }
    edit.record.*member = given;
    return {};
}

// A boolean is written as given. A rigid body's, whether it moves freely in
// the world, brings with it the records a free body has, and the station
// layout's reference body keeps moving freely.
template <typename Record>
std::expected<void, std::string> AssignMember(const MemberEdit<Record>& edit, bool Record::* member,
                                              const PropertyValue& value) {
    const bool given = std::get<bool>(value);
    bool& stated = edit.record.*member;
    if (given == stated) {
        return {};
    }
    if constexpr (std::is_same_v<Record, VehicleRigidBodyDefinition>) {
        if (given) {
            AddFreeBodyRecords(edit.project, edit.record.name);
        } else {
            const std::expected<void, std::string> kept = RequireLayoutKeepsReferenceBody(
                edit.project, edit.record.name, "made not to move freely");
            if (!kept.has_value()) {
                return kept;
            }
            RemoveFreeBodyRecords(edit.project, edit.record.name);
        }
    }
    stated = given;
    return {};
}

// The element's own name: a rename, which every reference to the element and
// every record following it take part in.
template <typename Record>
std::expected<void, std::string> AssignMember(const MemberEdit<Record>& edit,
                                              std::string Record::* member,
                                              const PropertyValue& value) {
    const std::string& new_name = std::get<ElementOwnName>(value).name;
    std::string& name = edit.record.*member;
    if (new_name == name) {
        return {};
    }
    const std::expected<void, std::string> available =
        RequireNameAvailable(edit.project, edit.element.kind, new_name);
    if (!available.has_value()) {
        return available;
    }
    RedirectReferences(edit.project, edit.element, new_name);
    RenameRecordsFollowingElement(edit.project, edit.element, new_name);
    name = new_name;
    return {};
}

// A reference must name an element of a kind it may name, or name none where
// it may.
template <typename Record, typename Name>
std::expected<void, std::string> AssignMember(const MemberEdit<Record>& edit,
                                              const ReferenceMember<Record, Name>& reference,
                                              const PropertyValue& value) {
    const std::optional<std::string>& named = std::get<ReferencedElementName>(value).name;
    if (!named.has_value()) {
        if constexpr (std::is_same_v<Name, std::optional<std::string>>) {
            edit.record.*reference.member = std::nullopt;
            return {};
        } else {
            return std::unexpected(edit.quoted_property_name + " must name a " +
                                   DescribeReferencedKinds(reference.referenced_kinds) +
                                   "; it cannot name none");
        }
    }
    if (!ContainsElementOfKinds(edit.project, reference.referenced_kinds, *named)) {
        return std::unexpected(
            edit.quoted_property_name + " names '" + *named + "', and this project has no " +
            DescribeReferencedKinds(reference.referenced_kinds) + " of that name");
    }
    edit.record.*reference.member = *named;
    return {};
}

std::string RefuseToken(const std::string& quoted_property_name,
                        std::span<const std::string_view> tokens, const std::string& given) {
    std::string choices;
    for (const std::string_view token : tokens) {
        choices += (choices.empty() ? "" : ", ") + Quoted(token);
    }
    return quoted_property_name + " is one of " + choices + ", not " + Quoted(given);
}

// A token must be one of its choices.
template <typename Record>
std::expected<void, std::string> AssignMember(const MemberEdit<Record>& edit,
                                              const TextTokenMember<Record>& token,
                                              const PropertyValue& value) {
    const std::string& given = std::get<PropertyToken>(value).token;
    if (std::ranges::find(token.tokens, given) == token.tokens.end()) {
        return std::unexpected(RefuseToken(edit.quoted_property_name, token.tokens, given));
    }
    edit.record.*token.member = given;
    return {};
}

template <typename Record, typename Value>
std::expected<void, std::string> AssignMember(const MemberEdit<Record>& edit,
                                              const ValueTokenMember<Record, Value>& token,
                                              const PropertyValue& value) {
    const std::string& given = std::get<PropertyToken>(value).token;
    const auto chosen = std::ranges::find(token.tokens, given);
    if (chosen == token.tokens.end()) {
        return std::unexpected(RefuseToken(edit.quoted_property_name, token.tokens, given));
    }
    edit.record.*token.member =
        token.values[static_cast<std::size_t>(chosen - token.tokens.begin())];
    return {};
}

// Why an edit cannot reach the record keyed by `body`'s name.
std::string MissingBodyKeyedRecord(const VehicleRigidBodyDefinition& body,
                                   double VehicleFreeBodyStationOffsetDefinition::*) {
    return body.moves_freely_in_world
               ? "free body '" + body.name + "' has no entry in the mechanical track station layout"
               : "rigid body '" + body.name +
                     "' does not move freely in the world, so the mechanical track station "
                     "layout gives it no station offset";
}

std::string MissingBodyKeyedRecord(const VehicleRigidBodyDefinition& body,
                                   double FreeBodyPlacement::*) {
    return body.moves_freely_in_world
               ? "free body '" + body.name +
                     "' has no placement, and a placement offset needs an existing placement"
               : "rigid body '" + body.name +
                     "' does not move freely in the world, so it has no placement";
}

// A station offset or a placement offset is written to the record keyed by
// the body's name, which must exist.
std::expected<void, std::string> AssignBodyKeyed(VehicleProject& project,
                                                 const VehicleRigidBodyDefinition& body,
                                                 const BodyKeyedProperty& property,
                                                 const PropertyValue& value) {
    const double given = std::get<double>(value);
    if (!std::isfinite(given)) {
        return std::unexpected(Quoted(property.name) + " must be finite");
    }
    return std::visit(
        [&project, &body, given](auto member) -> std::expected<void, std::string> {
            auto* const keyed = FindBodyKeyedRecord(project, body.name, member);
            if (keyed == nullptr) {
                return std::unexpected(MissingBodyKeyedRecord(body, member));
            }
            keyed->*member = given;
            return {};
        },
        property.member);
}

template <typename KindRecords>
std::expected<ElementReference, ElementEditRefusal> Assign(VehicleProject& project,
                                                           const ElementReference& element,
                                                           std::string_view property_name,
                                                           const PropertyValue& value) {
    using Record = typename KindRecords::Record;
    const auto refuse = [&element, property_name](std::string reason) {
        return std::unexpected(
            ElementEditRefusal{element, std::string(property_name), std::move(reason)});
    };
    Record* const record = vehicle_project::FindElementRecord<KindRecords>(project, element.name);
    if (record == nullptr) {
        return refuse("this project contains no " + vehicle_project::DescribeElement(element));
    }
    const std::expected<const ElementPropertyDescription*, std::string> described =
        DescribeElementProperty(element.kind, property_name);
    if (!described.has_value()) {
        return refuse(described.error());
    }
    const ElementPropertyDescription& description = **described;
    const std::string quoted_property_name = Quoted(property_name);
    if (!description.editable) {
        return refuse(quoted_property_name + " is read-only");
    }
    const PropertyValueKind given_kind = ValueKindOf(value);
    if (given_kind != description.value_kind) {
        return refuse(quoted_property_name + " holds " + DescribeValueKind(description.value_kind) +
                      ", but the value given is " + DescribeValueKind(given_kind));
    }

    const auto index =
        static_cast<std::size_t>(&description - DescribeElementProperties(element.kind).data());
    const std::expected<void, std::string> assigned = [&]() -> std::expected<void, std::string> {
        if constexpr (std::is_same_v<Record, VehicleRigidBodyDefinition>) {
            if (index >= kRecordProperties<Record>.size()) {
                return AssignBodyKeyed(
                    project, *record,
                    kBodyKeyedProperties[index - kRecordProperties<Record>.size()], value);
            }
        }
        const MemberEdit<Record> edit{project, element, *record, quoted_property_name};
        return std::visit(
            [&edit, &value](const auto& member) { return AssignMember(edit, member, value); },
            kRecordProperties<Record>[index].member);
    }();
    if (!assigned.has_value()) {
        return refuse(assigned.error());
    }
    return ElementReference{element.kind, record->*KindRecords::kElementName};
}

}  // namespace

std::string DescribeReferencedKinds(std::span<const ElementKind> referenced_kinds) {
    std::string described;
    for (const ElementKind kind : referenced_kinds) {
        described += (described.empty() ? "" : " or ") +
                     std::string(vehicle_project::DescribeElementKind(kind));
    }
    return described;
}

std::expected<ElementReference, ElementEditRefusal> AssignElementProperty(
    VehicleProject& project, const ElementReference& element, std::string_view property_name,
    const PropertyValue& value) {
    return vehicle_project::VisitElementKind(element.kind, [&]<typename KindRecords>(KindRecords) {
        return Assign<KindRecords>(project, element, property_name, value);
    });
}

}  // namespace orvd::vehicle_project_editing::internal
