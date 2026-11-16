#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

#include "orvd/configuration/point_to_point_spring_definitions.h"
#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project_editing/vehicle_project_element_properties.h"
#include "orvd/wheel_rail_contact/profile_points.h"

// What every member of every element record is: the one place that knows.
//
// Each record type has a table listing its members in declaration order, each
// under its own name, with what it holds. A number, a 3-vector, a boolean, a
// rotation and a curve say so by their types. A text member is the element's
// own name when it is the member ElementKindRecords states as kElementName;
// every other text member is a reference, stating the kinds it may name, or a
// token, stating its choices. These references are the one statement of which
// element names which: checking a reference that is set, finding what a
// removal takes with it, following a rename and describing a reference to an
// interface all read them here.
//
// DescribeElementProperties lists a kind's properties in its table's order and
// a rigid body's body-keyed properties after them; reading and assigning walk
// the same order.

namespace orvd::vehicle_project_editing::internal {

using vehicle_project::ElementKind;

// The kinds a reference may name. Each set lies within one name space
// (vehicle_project_element_name_spaces.h), so that a name identifies the element named. A
// frame reference names a fixed frame or a rigid body, which stands for the
// body's own frame.
inline constexpr std::array kReferencedRigidBody{ElementKind::kRigidBody};
inline constexpr std::array kReferencedFrame{ElementKind::kRigidBody, ElementKind::kFixedFrame};
inline constexpr std::array kReferencedContactCarrier{ElementKind::kContactCarrier};
inline constexpr std::array kReferencedRevoluteJoint{ElementKind::kRevoluteJoint};

// The axes a force element acts along: the tokens configuration's ParseAxis
// (libs/configuration/src/assemble_vehicle_multibody_model.cc) accepts. The
// kernel exports no list of them, so they are stated here.
inline constexpr std::array<std::string_view, 3> kForceElementAxisTokens{"longitudinal", "lateral",
                                                                         "vertical"};

// The tokens standing for the values of a member that is not text, each
// token's value at the token's position.
template <typename Value, std::size_t count>
struct ValueTokens {
    std::array<std::string_view, count> tokens;
    std::array<Value, count> values;
};

// A wheel's side.
inline constexpr ValueTokens<wheel_rail_contact::WheelSide, 2> kWheelSideTokens{
    .tokens = {"left", "right"},
    .values = {wheel_rail_contact::WheelSide::kLeft, wheel_rail_contact::WheelSide::kRight}};

// The fields of a saturated piecewise-linear damper curve's rows: every member
// of a curve point, in declaration order.
inline constexpr std::array kSaturatedPiecewiseLinearDamperCurveRowFields{
    CurveRowField{"relative_velocity_meters_per_second",
                  &configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition::
                      relative_velocity_meters_per_second},
    CurveRowField{
        "force_newtons",
        &configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition::force_newtons},
};

// A member naming another element of `referenced_kinds`: required as a
// std::string, or as a std::optional<std::string> a reference that may name
// none.
template <typename Record, typename Name>
struct ReferenceMember {
    Name Record::* member;
    std::span<const ElementKind> referenced_kinds;
};

// A member holding one of `tokens` as its text.
template <typename Record>
struct TextTokenMember {
    std::string Record::* member;
    std::span<const std::string_view> tokens;
};

// A member holding the value one of `tokens` stands for.
template <typename Record, typename Value>
struct ValueTokenMember {
    Value Record::* member;
    std::span<const std::string_view> tokens;
    std::span<const Value> values;
};

// A pointer to one member of `Record`, with what the member holds.
template <typename Record>
using RecordMember =
    std::variant<double Record::*, Eigen::Vector3d Record::*, bool Record::*,
                 Eigen::Matrix3d Record::*, std::string Record::*,
                 ReferenceMember<Record, std::string>,
                 ReferenceMember<Record, std::optional<std::string>>, TextTokenMember<Record>,
                 ValueTokenMember<Record, wheel_rail_contact::WheelSide>,
                 SaturatedPiecewiseLinearDamperCurve Record::*>;

// One property: a member of the record and that member's own name.
template <typename Record>
struct RecordProperty {
    std::string_view name;
    RecordMember<Record> member;
};

template <typename Record, typename Name, std::size_t count>
constexpr ReferenceMember<Record, Name> Reference(
    Name Record::* member, const std::array<ElementKind, count>& referenced_kinds) {
    return {member, referenced_kinds};
}

template <typename Record, std::size_t count>
constexpr TextTokenMember<Record> Token(std::string Record::* member,
                                        const std::array<std::string_view, count>& tokens) {
    return {member, tokens};
}

template <typename Record, typename Value, std::size_t count>
constexpr ValueTokenMember<Record, Value> Token(Value Record::* member,
                                                const ValueTokens<Value, count>& tokens) {
    return {member, tokens.tokens, tokens.values};
}

// Every member of each element record, in declaration order, under its own
// name.

constexpr auto PropertiesOfRecord(std::type_identity<configuration::VehicleRigidBodyDefinition>) {
    using Record = configuration::VehicleRigidBodyDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"moves_freely_in_world", &Record::moves_freely_in_world},
        {"mass_kilograms", &Record::mass_kilograms},
        {"center_of_mass_in_body_frame_meters", &Record::center_of_mass_in_body_frame_meters},
        {"inertia_moments_about_center_of_mass_kilogram_square_meters",
         &Record::inertia_moments_about_center_of_mass_kilogram_square_meters},
        {"inertia_products_about_center_of_mass_kilogram_square_meters",
         &Record::inertia_products_about_center_of_mass_kilogram_square_meters},
    });
}

constexpr auto PropertiesOfRecord(std::type_identity<configuration::VehicleFixedFrameDefinition>) {
    using Record = configuration::VehicleFixedFrameDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"body_name", Reference(&Record::body_name, kReferencedRigidBody)},
        {"position_in_body_frame_meters", &Record::position_in_body_frame_meters},
        {"rotation_in_body_frame", &Record::rotation_in_body_frame},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleRevoluteJointDefinition>) {
    using Record = configuration::VehicleRevoluteJointDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"parent_frame_name", Reference(&Record::parent_frame_name, kReferencedFrame)},
        {"child_frame_name", Reference(&Record::child_frame_name, kReferencedFrame)},
        {"axis_in_parent_frame", &Record::axis_in_parent_frame},
        {"damping_newton_metre_seconds_per_radian",
         &Record::damping_newton_metre_seconds_per_radian},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleBallRpyJointDefinition>) {
    using Record = configuration::VehicleBallRpyJointDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"parent_frame_name", Reference(&Record::parent_frame_name, kReferencedFrame)},
        {"child_frame_name", Reference(&Record::child_frame_name, kReferencedFrame)},
        {"default_roll_pitch_yaw_angles_radians", &Record::default_roll_pitch_yaw_angles_radians},
    });
}

constexpr auto PropertiesOfRecord(std::type_identity<configuration::VehicleWeldJointDefinition>) {
    using Record = configuration::VehicleWeldJointDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"parent_frame_name", Reference(&Record::parent_frame_name, kReferencedFrame)},
        {"child_frame_name", Reference(&Record::child_frame_name, kReferencedFrame)},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleTranslationalSpringDamperDefinition>) {
    using Record = configuration::VehicleTranslationalSpringDamperDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", Reference(&Record::reference_frame_name, kReferencedFrame)},
        {"opposite_frame_name", Reference(&Record::opposite_frame_name, kReferencedFrame)},
        {"stiffness_newtons_per_meter", &Record::stiffness_newtons_per_meter},
        {"damping_newton_seconds_per_meter", &Record::damping_newton_seconds_per_meter},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleRollSpringDamperCoupleDefinition>) {
    using Record = configuration::VehicleRollSpringDamperCoupleDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", Reference(&Record::reference_frame_name, kReferencedFrame)},
        {"opposite_frame_name", Reference(&Record::opposite_frame_name, kReferencedFrame)},
        {"stiffness_newton_meters_per_radian", &Record::stiffness_newton_meters_per_radian},
        {"damping_newton_meter_seconds_per_radian",
         &Record::damping_newton_meter_seconds_per_radian},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleSeriesSpringViscousDamperDefinition>) {
    using Record = configuration::VehicleSeriesSpringViscousDamperDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", Reference(&Record::reference_frame_name, kReferencedFrame)},
        {"opposite_frame_name", Reference(&Record::opposite_frame_name, kReferencedFrame)},
        {"axis", Token(&Record::axis, kForceElementAxisTokens)},
        {"series_stiffness_newtons_per_meter", &Record::series_stiffness_newtons_per_meter},
        {"series_damping_newton_seconds_per_meter",
         &Record::series_damping_newton_seconds_per_meter},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehiclePointToPointSeriesSpringBilinearDamperDefinition>) {
    using Record = configuration::VehiclePointToPointSeriesSpringBilinearDamperDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", Reference(&Record::reference_frame_name, kReferencedFrame)},
        {"opposite_frame_name", Reference(&Record::opposite_frame_name, kReferencedFrame)},
        {"series_stiffness_newtons_per_meter", &Record::series_stiffness_newtons_per_meter},
        {"damping_below_knee_newton_seconds_per_meter",
         &Record::damping_below_knee_newton_seconds_per_meter},
        {"knee_force_newtons", &Record::knee_force_newtons},
        {"damping_above_knee_newton_seconds_per_meter",
         &Record::damping_above_knee_newton_seconds_per_meter},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleSaturatedPiecewiseLinearDamperDefinition>) {
    using Record = configuration::VehicleSaturatedPiecewiseLinearDamperDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", Reference(&Record::reference_frame_name, kReferencedFrame)},
        {"opposite_frame_name", Reference(&Record::opposite_frame_name, kReferencedFrame)},
        {"axis", Token(&Record::axis, kForceElementAxisTokens)},
        {"curve", &Record::curve},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleReferenceFrameLinearBushingDefinition>) {
    using Record = configuration::VehicleReferenceFrameLinearBushingDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", Reference(&Record::reference_frame_name, kReferencedFrame)},
        {"opposite_frame_name", Reference(&Record::opposite_frame_name, kReferencedFrame)},
        {"translational_stiffness_newtons_per_meter",
         &Record::translational_stiffness_newtons_per_meter},
        {"translational_damping_newton_seconds_per_meter",
         &Record::translational_damping_newton_seconds_per_meter},
        {"rotational_damping_newton_meter_seconds_per_radian",
         &Record::rotational_damping_newton_meter_seconds_per_radian},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::PointToPointLinearSpringDamperDefinition>) {
    using Record = configuration::PointToPointLinearSpringDamperDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", Reference(&Record::reference_frame_name, kReferencedFrame)},
        {"opposite_frame_name", Reference(&Record::opposite_frame_name, kReferencedFrame)},
        {"nominal_length_meters", &Record::nominal_length_meters},
        {"stiffness_newtons_per_meter", &Record::stiffness_newtons_per_meter},
        {"damping_newton_seconds_per_meter", &Record::damping_newton_seconds_per_meter},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::PointToPointHystereticSpringDefinition>) {
    using Record = configuration::PointToPointHystereticSpringDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", Reference(&Record::reference_frame_name, kReferencedFrame)},
        {"opposite_frame_name", Reference(&Record::opposite_frame_name, kReferencedFrame)},
        {"nominal_length_meters", &Record::nominal_length_meters},
        {"stiffness_newtons_per_meter", &Record::stiffness_newtons_per_meter},
        {"hysteresis_ratio", &Record::hysteresis_ratio},
        {"hysteresis_transition_speed_meters_per_second",
         &Record::hysteresis_transition_speed_meters_per_second},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleContactCarrierDeclaration>) {
    using Record = configuration::VehicleContactCarrierDeclaration;
    return std::to_array<RecordProperty<Record>>({
        {"carrier_name", &Record::carrier_name},
        {"body_name", Reference(&Record::body_name, kReferencedRigidBody)},
        {"rotation_body_from_nonspinning_wheel_profile",
         &Record::rotation_body_from_nonspinning_wheel_profile},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleContactInterfaceDeclaration>) {
    using Record = configuration::VehicleContactInterfaceDeclaration;
    return std::to_array<RecordProperty<Record>>({
        {"interface_name", &Record::interface_name},
        {"carrier_name", Reference(&Record::carrier_name, kReferencedContactCarrier)},
        {"wheel_body_name", Reference(&Record::wheel_body_name, kReferencedRigidBody)},
        {"side", Token(&Record::side, kWheelSideTokens)},
        {"independent_wheel_revolute_joint_name",
         Reference(&Record::independent_wheel_revolute_joint_name, kReferencedRevoluteJoint)},
    });
}

template <typename Record>
inline constexpr auto kRecordProperties = PropertiesOfRecord(std::type_identity<Record>{});

// Whether the one plain text member KindRecords' table lists is the member
// ElementKindRecords states as the element's own name.
template <typename KindRecords>
consteval bool ListsOwnNameAsTheOnlyPlainText() {
    using Record = typename KindRecords::Record;
    std::size_t plain_text_members = 0;
    for (const RecordProperty<Record>& property : kRecordProperties<Record>) {
        if (const auto* const member = std::get_if<std::string Record::*>(&property.member)) {
            if (*member != KindRecords::kElementName) {
                return false;
            }
            ++plain_text_members;
        }
    }
    return plain_text_members == 1;
}

// The rigid-body properties held outside the body's own record, each in a
// record keyed by the body's name (vehicle_project_records_following_elements.h): its station
// offset in the vehicle definition's mechanical track station layout, and the
// lateral and vertical offsets of its drawing placement, so that the three
// state where along, across and above the track the body is. Each is named
// after its record's member, prefixed with where that record lives.
using BodyKeyedMember =
    std::variant<double configuration::VehicleFreeBodyStationOffsetDefinition::*,
                 double vehicle_project::FreeBodyPlacement::*>;

struct BodyKeyedProperty {
    std::string_view name;
    BodyKeyedMember member;
};

inline constexpr std::array kBodyKeyedProperties{
    BodyKeyedProperty{
        "mechanical_track_station_layout.station_offset_meters",
        &configuration::VehicleFreeBodyStationOffsetDefinition::station_offset_meters},
    BodyKeyedProperty{
        "placement.lateral_offset_in_local_track_frame_meters",
        &vehicle_project::FreeBodyPlacement::lateral_offset_in_local_track_frame_meters},
    BodyKeyedProperty{
        "placement.vertical_offset_in_local_track_frame_meters",
        &vehicle_project::FreeBodyPlacement::vertical_offset_in_local_track_frame_meters},
};

}  // namespace orvd::vehicle_project_editing::internal
