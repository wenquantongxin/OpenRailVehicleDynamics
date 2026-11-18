#include "vehicle_project_element_property_assignment.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
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
#include "vehicle_project_orientation_property_values.h"
#include "vehicle_project_property_edit_value_checks.h"
#include "vehicle_project_records_following_elements.h"

namespace orvd::vehicle_project_editing::internal {
namespace {

using configuration::VehicleFreeBodyStationOffsetDefinition;
using configuration::VehicleRigidBodyDefinition;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::FreeBodyPlacement;
using vehicle_project::VehicleProject;

// Whether every number a value holds is finite.
bool AllFinite(double number) {
    return std::isfinite(number);
}
bool AllFinite(const Eigen::Vector3d& vector) {
    return vector.allFinite();
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
    std::string_view property_name;
    std::string quoted_property_name;
};

// A number, a 3-vector or a curve is written as given when every number in it
// is finite.
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

// A rotation matrix holds the rotation the edit's three angles give.
template <typename Record>
std::expected<void, std::string> AssignMember(const MemberEdit<Record>& edit,
                                              Eigen::Matrix3d Record::* member,
                                              const PropertyValue& value) {
    const std::expected<Eigen::Matrix3d, std::string> rotation =
        RotationGivenByEdit(edit.property_name, std::get<PropertyOrientation>(value));
    if (!rotation.has_value()) {
        return std::unexpected(rotation.error());
    }
    edit.record.*member = *rotation;
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

// A token must be one of its choices.
template <typename Record>
std::expected<void, std::string> AssignMember(const MemberEdit<Record>& edit,
                                              const TextTokenMember<Record>& token,
                                              const PropertyValue& value) {
    const std::string& given = std::get<PropertyToken>(value).token;
    if (std::ranges::find(token.tokens, given) == token.tokens.end()) {
        return std::unexpected(RefuseTokenOutsideChoices(edit.property_name, token.tokens, given));
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
        return std::unexpected(RefuseTokenOutsideChoices(edit.property_name, token.tokens, given));
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

template <typename Value>
std::string MissingBodyKeyedRecord(const VehicleRigidBodyDefinition& body,
                                   Value FreeBodyPlacement::*) {
    return body.moves_freely_in_world
               ? "free body '" + body.name +
                     "' has no placement, and its placement's offsets and rotation need one"
               : "rigid body '" + body.name +
                     "' does not move freely in the world, so it has no placement";
}

// What a body-keyed member is given: an offset the finite number given, a
// placement's quaternion the rotation the three angles given give.
template <typename KeyedRecord>
std::expected<double, std::string> BodyKeyedValueGiven(std::string_view property_name,
                                                       double KeyedRecord::*,
                                                       const PropertyValue& value) {
    const double given = std::get<double>(value);
    if (!std::isfinite(given)) {
        return std::unexpected(QuotePropertyName(property_name) + " must be finite");
    }
    return given;
}

std::expected<Eigen::Quaterniond, std::string> BodyKeyedValueGiven(
    std::string_view property_name, Eigen::Quaterniond FreeBodyPlacement::*,
    const PropertyValue& value) {
    return RotationGivenByEdit(property_name, std::get<PropertyOrientation>(value))
        .transform([](const Eigen::Matrix3d& rotation) { return Eigen::Quaterniond(rotation); });
}

// A station offset, a placement offset or a placement rotation is written to
// the record keyed by the body's name, which must exist.
std::expected<void, std::string> AssignBodyKeyed(VehicleProject& project,
                                                 const VehicleRigidBodyDefinition& body,
                                                 const BodyKeyedProperty& property,
                                                 const PropertyValue& value) {
    return std::visit(
        [&](auto member) -> std::expected<void, std::string> {
            const auto given = BodyKeyedValueGiven(property.name, member, value);
            if (!given.has_value()) {
                return std::unexpected(given.error());
            }
            auto* const keyed = FindBodyKeyedRecord(project, body.name, member);
            if (keyed == nullptr) {
                return std::unexpected(MissingBodyKeyedRecord(body, member));
            }
            keyed->*member = *given;
            return {};
        },
        property.member);
}

template <typename KindRecords>
std::expected<ElementReference, SubjectEditRefusal> Assign(VehicleProject& project,
                                                           const ElementReference& element,
                                                           std::string_view property_name,
                                                           const PropertyValue& value) {
    using Record = typename KindRecords::Record;
    const auto refuse = [&element, property_name](std::string reason) {
        return std::unexpected(
            SubjectEditRefusal{element, std::string(property_name), std::move(reason)});
    };
    Record* const record = vehicle_project::FindElementRecord<KindRecords>(project, element.name);
    if (record == nullptr) {
        return refuse("this project contains no " + vehicle_project::DescribeElement(element));
    }
    const std::expected<const PropertyDescription*, std::string> described =
        DescribeElementProperty(element.kind, property_name);
    if (!described.has_value()) {
        return refuse(described.error());
    }
    const PropertyDescription& description = **described;
    const std::expected<void, std::string> editable_with_its_kind =
        RequireEditableAndGivenItsValueKind(description, value);
    if (!editable_with_its_kind.has_value()) {
        return refuse(editable_with_its_kind.error());
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
        const MemberEdit<Record> edit{project, element, *record, property_name,
                                      QuotePropertyName(property_name)};
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

std::expected<ElementReference, SubjectEditRefusal> AssignElementProperty(
    VehicleProject& project, const ElementReference& element, std::string_view property_name,
    const PropertyValue& value) {
    return vehicle_project::VisitElementKind(element.kind, [&]<typename KindRecords>(KindRecords) {
        return Assign<KindRecords>(project, element, property_name, value);
    });
}

}  // namespace orvd::vehicle_project_editing::internal
