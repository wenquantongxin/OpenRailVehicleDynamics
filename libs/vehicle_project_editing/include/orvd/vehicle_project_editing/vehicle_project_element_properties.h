#pragma once

#include <Eigen/Core>
#include <expected>
#include <functional>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"

// The properties of a project's elements, for an interface that shows,
// creates and edits elements without knowing any record's fields.
//
// Each element kind has one fixed list of properties: every member of the
// kind's record, in declaration order, named by the member's own name, which
// already states its meaning and unit (`stiffness_newtons_per_meter`). The
// contact carriers and interfaces list the members of their declarations. A
// rigid body then adds where it is along, across and above the track, from
// records outside its own: its entry in the mechanical track station layout,
// `mechanical_track_station_layout.station_offset_meters`, and the two
// offsets of its drawing placement,
// `placement.lateral_offset_in_local_track_frame_meters` and
// `placement.vertical_offset_in_local_track_frame_meters`. The prefixes say
// which record holds them. The rest of the layout, its reference body and its
// wheel-pair station list, is not an element and has no properties here.
//
// A description says what a property holds. The element's own name is a
// property like any other: setting it renames the element. A reference names
// another element; its description lists the kinds it may name, all of one
// name space, so that the name identifies the element, and says whether it may
// name none. A token is one of a fixed set of choices, which its description
// lists, and a curve's description lists the fields of its rows.
//
// Every property can be edited except a rotation. A kind's description states
// whether its property can be edited at all; a reading states whether one
// element's property can be edited now, which a station offset or a placement
// offset cannot while the record holding it is missing. Edits go through
// VehicleProjectEditingSession, which validates them and keeps their history.

namespace orvd::vehicle_project_editing {

// What kind of value a property holds, and the C++ type that holds it.
enum class PropertyValueKind {
    // double
    kNumber,
    // Eigen::Vector3d
    kVector3,
    // bool
    kBoolean,
    // Eigen::Matrix3d, a rotation matrix.
    kRotation,
    // ElementOwnName
    kElementName,
    // ReferencedElementName
    kElementReference,
    // PropertyToken
    kToken,
    // SaturatedPiecewiseLinearDamperCurve
    kSaturatedPiecewiseLinearDamperCurve,
};

// The value of the property holding an element's own name.
struct ElementOwnName {
    std::string name;

    friend bool operator==(const ElementOwnName&, const ElementOwnName&) = default;
};

// The value of a reference: the name of the element it names, or none, which
// only a reference that may name none can hold.
struct ReferencedElementName {
    std::optional<std::string> name;

    friend bool operator==(const ReferencedElementName&, const ReferencedElementName&) = default;
};

// The value of a token property: one of the property's choices.
struct PropertyToken {
    std::string token;

    friend bool operator==(const PropertyToken&, const PropertyToken&) = default;
};

// The non-negative half of a saturated piecewise-linear damper's odd curve.
using SaturatedPiecewiseLinearDamperCurve =
    std::vector<configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition>;

// One property's value, held as the type its value kind names.
using PropertyValue =
    std::variant<double, Eigen::Vector3d, bool, Eigen::Matrix3d, ElementOwnName,
                 ReferencedElementName, PropertyToken, SaturatedPiecewiseLinearDamperCurve>;

// Property values by property name, as a new element is given them.
using PropertyValuesByName = std::map<std::string, PropertyValue, std::less<>>;

// One field of a curve's rows: the name a row states it under, and the member
// of a curve point that holds it.
struct CurveRowField {
    std::string_view name;
    double configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition::* member;
};

struct ElementPropertyDescription {
    std::string_view name;
    PropertyValueKind value_kind{PropertyValueKind::kNumber};
    // Whether the property can be set: for a kind (DescribeElementProperties),
    // whether its property can be edited at all, a property that cannot being
    // read-only; in a reading, whether that element's property can be edited
    // now.
    bool editable{false};
    // A reference's: the kinds of element it may name, in ElementKind's order,
    // and whether it may name none. Empty and false for any other property.
    std::span<const vehicle_project::ElementKind> referenced_kinds{};
    bool may_name_none{false};
    // A token's choices. Empty for any other property.
    std::span<const std::string_view> choices{};
    // A curve's row fields, in the order a row states them. Empty for any
    // other property.
    std::span<const CurveRowField> row_fields{};
};

struct ElementPropertyReading {
    // The property's description as DescribeElementProperties gives it, and
    // whether this element's property can be edited now: a property whose
    // value is absent for this element is not editable, since an edit of it
    // is refused.
    ElementPropertyDescription description;
    // Absent where the element holds no value for the property: a rigid body
    // the mechanical track station layout does not place has no station
    // offset, and a rigid body without a placement has no placement offsets.
    // A reference that names none has a value: a ReferencedElementName holding
    // no name.
    std::optional<PropertyValue> value;
};

// The properties of every element of `kind`, in the order described above.
[[nodiscard]] std::span<const ElementPropertyDescription> DescribeElementProperties(
    vehicle_project::ElementKind kind);

// The description of the property of `kind` named `property_name`, among those
// DescribeElementProperties gives, or a sentence saying the kind has no such
// property.
[[nodiscard]] std::expected<const ElementPropertyDescription*, std::string> DescribeElementProperty(
    vehicle_project::ElementKind kind, std::string_view property_name);

// The names of the project's elements of `kind`, in the order their records
// declare them. A contact kind has none while the project has no contact
// record.
[[nodiscard]] std::vector<std::string> CollectElementNames(
    const vehicle_project::VehicleProject& project, vehicle_project::ElementKind kind);

// Every property of one element with its current value and whether it can be
// edited now, in the order DescribeElementProperties gives.
//
// Throws std::invalid_argument, naming the element, when the project contains
// no element of that kind and name.
[[nodiscard]] std::vector<ElementPropertyReading> ReadElementProperties(
    const vehicle_project::VehicleProject& project,
    const vehicle_project::ElementReference& element);

}  // namespace orvd::vehicle_project_editing
