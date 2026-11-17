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
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/vehicle_project_editing/vehicle_project_titles_in_english_and_chinese.h"

// The properties of a project's subjects, its elements and its settings
// (vehicle_project_property_subjects.h), for an interface that shows, creates
// and edits them without knowing any record's fields.
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
// which record holds them. A rigid body's properties are shown in sections:
// degrees of freedom, mass and inertia, its mechanical station, which the
// dynamics use, and its preview placement, which only places it in the
// drawing.
//
// Each settings has one fixed list of properties as well: every member of its
// record, nested records' members included, in declaration order, under the
// member's own name, prefixed for a nested record's member with the member
// holding that record (`friction.static_coefficient`). The gravity is the
// project's, the vehicle identity the vehicle definition's two names, the
// station layout its reference body, and the wheel, the rail and the contact
// computation the configuration records of the wheel-rail contact record.
//
// A description says what a property holds. The element's own name is a
// property like any other: setting it renames the element. A reference names
// another element; its description lists the kinds it may name, all of one
// name space, so that the name identifies the element, and says whether it may
// name none. A token is one of a fixed set of choices, which its description
// lists, and a curve's description lists the fields of its rows. A text is
// shown as it is: never an element's own name and never a reference. A count
// held as an integer is a number, and an enumeration a token.
//
// Every element property can be edited except a rotation. Of the settings,
// the gravity, the wheel's nominal rolling radius and lateral datum, and the
// rail's gauge, gauge measuring depth, two cants and profile origin mode can
// be edited; nothing here judges them beyond their being finite. A
// description states whether its property can be edited at all; a reading
// states whether one subject's property can be edited now, which an element's
// station offset or placement offset cannot while the record holding it is
// missing. The gravity can be edited while the project states none. Edits go
// through VehicleProjectEditingSession, which validates them and keeps their
// history.

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
    // PropertyText
    kText,
    // ElementOwnName
    kElementName,
    // ReferencedElementName
    kElementReference,
    // PropertyToken
    kToken,
    // SaturatedPiecewiseLinearDamperCurve
    kSaturatedPiecewiseLinearDamperCurve,
};

// The value of a text property, shown as it is.
struct PropertyText {
    std::string text;

    friend bool operator==(const PropertyText&, const PropertyText&) = default;
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
    std::variant<double, Eigen::Vector3d, bool, Eigen::Matrix3d, PropertyText, ElementOwnName,
                 ReferencedElementName, PropertyToken, SaturatedPiecewiseLinearDamperCurve>;

// Property values by property name, as a new element is given them.
using PropertyValuesByName = std::map<std::string, PropertyValue, std::less<>>;

// One field of a curve's rows: the name a row states it under, and the member
// of a curve point that holds it.
struct CurveRowField {
    std::string_view name;
    double configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition::* member;
};

struct PropertyDescription {
    std::string_view name;
    PropertyValueKind value_kind{PropertyValueKind::kNumber};
    // Whether the property can be set: in a description of an element kind's
    // or a settings' properties, whether it can be edited at all, a property
    // that cannot being read-only; in a reading, whether that subject's
    // property can be edited now.
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
    // The section the property is shown in; consecutive properties of one
    // section form it. Absent for a property shown in none.
    std::optional<Title> section{};
};

struct PropertyReading {
    // The property's description, and whether this subject's property can be
    // edited now: an element property whose value is absent for this element
    // is not editable, since an edit of it is refused.
    PropertyDescription description;
    // Absent where the subject holds no value for the property: a rigid body
    // the mechanical track station layout does not place has no station
    // offset, a rigid body without a placement has no placement offsets, and
    // a project that states no gravity has none. A reference that names none
    // has a value: a ReferencedElementName holding no name.
    std::optional<PropertyValue> value;
};

// The properties of every element of `kind`, in the order described above.
[[nodiscard]] std::span<const PropertyDescription> DescribeElementProperties(
    vehicle_project::ElementKind kind);

// The properties of `settings`, in the order described above, whether or not
// a project holds them.
[[nodiscard]] std::span<const PropertyDescription> DescribeSettingsProperties(
    ProjectSettings settings);

// The description of the property of `kind` named `property_name`, among those
// DescribeElementProperties gives, or a sentence saying the kind has no such
// property.
[[nodiscard]] std::expected<const PropertyDescription*, std::string> DescribeElementProperty(
    vehicle_project::ElementKind kind, std::string_view property_name);

// The description of the property of `subject` named `property_name`: for an
// element, its kind's (DescribeElementProperty); for settings, theirs. Or a
// sentence saying the subject has no such property.
[[nodiscard]] std::expected<const PropertyDescription*, std::string> DescribeSubjectProperty(
    const PropertySubject& subject, std::string_view property_name);

// The names of the project's elements of `kind`, in the order their records
// declare them. A contact kind has none while the project has no contact
// record.
[[nodiscard]] std::vector<std::string> CollectElementNames(
    const vehicle_project::VehicleProject& project, vehicle_project::ElementKind kind);

// Every property of one subject with its current value and whether it can be
// edited now, in the order described above.
//
// Throws std::invalid_argument, naming the subject, when the project contains
// no such element, and when it has no wheel-rail contact record for the
// wheel, rail or contact computation settings.
[[nodiscard]] std::vector<PropertyReading> ReadSubjectProperties(
    const vehicle_project::VehicleProject& project, const PropertySubject& subject);

}  // namespace orvd::vehicle_project_editing
