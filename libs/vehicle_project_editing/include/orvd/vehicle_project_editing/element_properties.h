#pragma once

#include <Eigen/Core>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/wheel_rail_contact/profile_points.h"

// The properties of a project's elements, for an interface that shows and
// edits elements without knowing any record's fields.
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
// The numbers and 3-vectors of the mechanical element records are editable,
// and so are a rigid body's station offset and placement offsets while the
// records holding them exist; every other property is read-only. A kind's
// description states whether its property can be edited at all; a reading
// states whether one element's property can be edited now. Edits go through
// VehicleProjectEditingSession, which validates them and keeps their history.

namespace orvd::vehicle_project_editing {

// What kind of value a property holds, and the C++ type that holds it.
enum class PropertyValueKind {
    // double
    kNumber,
    // Eigen::Vector3d
    kVector3,
    // std::string: an element's name, a reference to another element by name,
    // or an axis token such as "lateral".
    kText,
    // bool
    kBoolean,
    // Eigen::Matrix3d, a rotation matrix.
    kRotation,
    // wheel_rail_contact::WheelSide
    kWheelSide,
    // The non-negative half of a saturated piecewise-linear damper's odd curve,
    // std::vector<configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition>.
    kSaturatedPiecewiseLinearDamperCurve,
};

// One property's value, held as the type its value kind names.
using PropertyValue =
    std::variant<double, Eigen::Vector3d, std::string, bool, Eigen::Matrix3d,
                 wheel_rail_contact::WheelSide,
                 std::vector<configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition>>;

struct ElementPropertyDescription {
    std::string_view name;
    PropertyValueKind value_kind{PropertyValueKind::kNumber};
    // Whether the property can be set: for a kind (DescribeElementProperties),
    // whether its property can be edited at all, a property that cannot being
    // read-only; in a reading, whether that element's property can be edited
    // now.
    bool editable{false};
};

struct ElementPropertyReading {
    // The property's name and value kind as DescribeElementProperties gives
    // them, and whether this element's property can be edited now: a property
    // whose value is absent for this element is not editable, since an edit
    // of it is refused.
    ElementPropertyDescription description;
    // Absent where the element holds no value for the property: a rigid body
    // the mechanical track station layout does not place has no station
    // offset, a rigid body without a placement has no placement offsets, and a
    // contact interface whose wheel is not separately jointed names no
    // independent-wheel revolute joint.
    std::optional<PropertyValue> value;
};

// The properties of every element of `kind`, in the order described above.
[[nodiscard]] std::span<const ElementPropertyDescription> DescribeElementProperties(
    vehicle_project::ElementKind kind);

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
