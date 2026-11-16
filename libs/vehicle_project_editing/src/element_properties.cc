#include "orvd/vehicle_project_editing/element_properties.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <expected>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "element_property_assignment.h"
#include "orvd/configuration/point_to_point_spring_definitions.h"
#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/vehicle_project/element_kind_records.h"

namespace orvd::vehicle_project_editing {
namespace {

using configuration::VehicleFreeBodyStationOffsetDefinition;
using configuration::VehicleRigidBodyDefinition;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::FreeBodyPlacement;
using vehicle_project::VehicleProject;
using vehicle_project::VisitElementKind;
using SaturatedPiecewiseLinearDamperCurve =
    std::vector<configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition>;

// The value kind of each type an element record's members hold. A name that
// may be absent is text.
constexpr PropertyValueKind ValueKindOf(std::type_identity<double>) {
    return PropertyValueKind::kNumber;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<Eigen::Vector3d>) {
    return PropertyValueKind::kVector3;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<std::string>) {
    return PropertyValueKind::kText;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<std::optional<std::string>>) {
    return PropertyValueKind::kText;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<bool>) {
    return PropertyValueKind::kBoolean;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<Eigen::Matrix3d>) {
    return PropertyValueKind::kRotation;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<wheel_rail_contact::WheelSide>) {
    return PropertyValueKind::kWheelSide;
}
constexpr PropertyValueKind ValueKindOf(std::type_identity<SaturatedPiecewiseLinearDamperCurve>) {
    return PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve;
}

PropertyValueKind ValueKindOf(const PropertyValue& value) {
    return std::visit(
        [](const auto& held) {
            return ValueKindOf(std::type_identity<std::remove_cvref_t<decltype(held)>>{});
        },
        value);
}

// How a refusal names a value kind.
std::string DescribeValueKind(PropertyValueKind kind) {
    switch (kind) {
        case PropertyValueKind::kNumber:
            return "a number";
        case PropertyValueKind::kVector3:
            return "a 3-vector";
        case PropertyValueKind::kText:
            return "text";
        case PropertyValueKind::kBoolean:
            return "a boolean";
        case PropertyValueKind::kRotation:
            return "a rotation";
        case PropertyValueKind::kWheelSide:
            return "a wheel side";
        case PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve:
            return "a saturated piecewise-linear damper curve";
    }
    // Reached only by a value cast into PropertyValueKind that names none of
    // its enumerators.
    return "a value of no known kind";
}

// A pointer to one member of `Record`, of whichever type element records hold
// that member has.
template <typename Record>
using RecordMember =
    std::variant<double Record::*, Eigen::Vector3d Record::*, std::string Record::*,
                 std::optional<std::string> Record::*, bool Record::*, Eigen::Matrix3d Record::*,
                 wheel_rail_contact::WheelSide Record::*,
                 SaturatedPiecewiseLinearDamperCurve Record::*>;

// One property: a member of the record and that member's own name.
template <typename Record>
struct RecordProperty {
    std::string_view name;
    RecordMember<Record> member;
};

// The type of the member a pointer to member points to.
template <typename MemberPointer>
struct PointedMember;

template <typename Value, typename Record>
struct PointedMember<Value Record::*> {
    using type = Value;
};

template <typename MemberPointer>
using PointedMemberType = typename PointedMember<MemberPointer>::type;

// The numbers and 3-vectors of the mechanical element records are editable;
// every other member is read-only. The contact declarations hold neither type,
// so the rule needs no exception for them.
template <typename Value>
constexpr bool kEditableNumberOrVector3 =
    std::is_same_v<Value, double> || std::is_same_v<Value, Eigen::Vector3d>;

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
        {"body_name", &Record::body_name},
        {"position_in_body_frame_meters", &Record::position_in_body_frame_meters},
        {"rotation_in_body_frame", &Record::rotation_in_body_frame},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleRevoluteJointDefinition>) {
    using Record = configuration::VehicleRevoluteJointDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"parent_frame_name", &Record::parent_frame_name},
        {"child_frame_name", &Record::child_frame_name},
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
        {"parent_frame_name", &Record::parent_frame_name},
        {"child_frame_name", &Record::child_frame_name},
        {"default_roll_pitch_yaw_angles_radians", &Record::default_roll_pitch_yaw_angles_radians},
    });
}

constexpr auto PropertiesOfRecord(std::type_identity<configuration::VehicleWeldJointDefinition>) {
    using Record = configuration::VehicleWeldJointDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"parent_frame_name", &Record::parent_frame_name},
        {"child_frame_name", &Record::child_frame_name},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleTranslationalSpringDamperDefinition>) {
    using Record = configuration::VehicleTranslationalSpringDamperDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", &Record::reference_frame_name},
        {"opposite_frame_name", &Record::opposite_frame_name},
        {"stiffness_newtons_per_meter", &Record::stiffness_newtons_per_meter},
        {"damping_newton_seconds_per_meter", &Record::damping_newton_seconds_per_meter},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleRollSpringDamperCoupleDefinition>) {
    using Record = configuration::VehicleRollSpringDamperCoupleDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", &Record::reference_frame_name},
        {"opposite_frame_name", &Record::opposite_frame_name},
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
        {"reference_frame_name", &Record::reference_frame_name},
        {"opposite_frame_name", &Record::opposite_frame_name},
        {"axis", &Record::axis},
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
        {"reference_frame_name", &Record::reference_frame_name},
        {"opposite_frame_name", &Record::opposite_frame_name},
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
        {"reference_frame_name", &Record::reference_frame_name},
        {"opposite_frame_name", &Record::opposite_frame_name},
        {"axis", &Record::axis},
        {"curve", &Record::curve},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleReferenceFrameLinearBushingDefinition>) {
    using Record = configuration::VehicleReferenceFrameLinearBushingDefinition;
    return std::to_array<RecordProperty<Record>>({
        {"name", &Record::name},
        {"reference_frame_name", &Record::reference_frame_name},
        {"opposite_frame_name", &Record::opposite_frame_name},
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
        {"reference_frame_name", &Record::reference_frame_name},
        {"opposite_frame_name", &Record::opposite_frame_name},
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
        {"reference_frame_name", &Record::reference_frame_name},
        {"opposite_frame_name", &Record::opposite_frame_name},
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
        {"body_name", &Record::body_name},
        {"rotation_body_from_nonspinning_wheel_profile",
         &Record::rotation_body_from_nonspinning_wheel_profile},
    });
}

constexpr auto PropertiesOfRecord(
    std::type_identity<configuration::VehicleContactInterfaceDeclaration>) {
    using Record = configuration::VehicleContactInterfaceDeclaration;
    return std::to_array<RecordProperty<Record>>({
        {"interface_name", &Record::interface_name},
        {"carrier_name", &Record::carrier_name},
        {"wheel_body_name", &Record::wheel_body_name},
        {"side", &Record::side},
        {"independent_wheel_revolute_joint_name", &Record::independent_wheel_revolute_joint_name},
    });
}

template <typename Record>
constexpr auto kRecordProperties = PropertiesOfRecord(std::type_identity<Record>{});

// The rigid-body properties held outside the body's own record, each in a
// record keyed by the body's name: its station offset in the vehicle
// definition's mechanical track station layout, and the lateral and vertical
// offsets of its drawing placement, so that the three state where along,
// across and above the track the body is. Each is named after its record's
// member, prefixed with where that record lives.
using BodyKeyedMember =
    std::variant<double VehicleFreeBodyStationOffsetDefinition::*, double FreeBodyPlacement::*>;

struct BodyKeyedProperty {
    std::string_view name;
    BodyKeyedMember member;
};

constexpr std::array kBodyKeyedProperties{
    BodyKeyedProperty{"mechanical_track_station_layout.station_offset_meters",
                      &VehicleFreeBodyStationOffsetDefinition::station_offset_meters},
    BodyKeyedProperty{"placement.lateral_offset_in_local_track_frame_meters",
                      &FreeBodyPlacement::lateral_offset_in_local_track_frame_meters},
    BodyKeyedProperty{"placement.vertical_offset_in_local_track_frame_meters",
                      &FreeBodyPlacement::vertical_offset_in_local_track_frame_meters},
};

// The body's entry in the mechanical track station layout, or null when the
// layout has none for it.
template <typename Project>
auto* FindBodyKeyedRecord(Project& project, const std::string& body_name,
                          double VehicleFreeBodyStationOffsetDefinition::*) {
    auto& offsets =
        project.vehicle_definition.mechanical_track_station_layout.free_body_station_offsets;
    const auto found =
        std::ranges::find(offsets, body_name, &VehicleFreeBodyStationOffsetDefinition::body_name);
    return found == offsets.end() ? nullptr : std::to_address(found);
}

// The body's placement, or null when it has none.
template <typename Project>
auto* FindBodyKeyedRecord(Project& project, const std::string& body_name,
                          double FreeBodyPlacement::*) {
    auto& placements = project.free_body_placements;
    const auto found = std::ranges::find(placements, body_name, &FreeBodyPlacement::body_name);
    return found == placements.end() ? nullptr : std::to_address(found);
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

// The record's properties, then, for a rigid body, those keyed by its name.
template <typename Record>
std::span<const ElementPropertyDescription> Descriptions() {
    static const std::vector<ElementPropertyDescription> descriptions = [] {
        std::vector<ElementPropertyDescription> described;
        for (const RecordProperty<Record>& property : kRecordProperties<Record>) {
            described.push_back(std::visit(
                [&property](auto member) {
                    using Value = PointedMemberType<decltype(member)>;
                    return ElementPropertyDescription{property.name,
                                                      ValueKindOf(std::type_identity<Value>{}),
                                                      kEditableNumberOrVector3<Value>};
                },
                property.member));
        }
        if constexpr (std::is_same_v<Record, VehicleRigidBodyDefinition>) {
            for (const BodyKeyedProperty& property : kBodyKeyedProperties) {
                described.push_back({property.name, PropertyValueKind::kNumber, true});
            }
        }
        return described;
    }();
    return descriptions;
}

// The record of the element of this kind named `name`, or null when the
// project has none.
template <typename KindRecords, typename Project>
auto* FindRecord(Project& project, const std::string& name) {
    auto* const records = KindRecords::Records(project);
    using RecordPointer = decltype(records->data());
    if (records == nullptr) {
        return RecordPointer{nullptr};
    }
    const auto found = std::ranges::find(*records, name, KindRecords::kElementName);
    return found == records->end() ? RecordPointer{nullptr} : std::to_address(found);
}

// A member's value as a property value.
template <typename Value>
std::optional<PropertyValue> ValueOf(const Value& value) {
    return PropertyValue(std::in_place_type<Value>, value);
}

// A name that may be absent has a value only when it is given.
std::optional<PropertyValue> ValueOf(const std::optional<std::string>& name) {
    if (!name.has_value()) {
        return std::nullopt;
    }
    return PropertyValue(std::in_place_type<std::string>, *name);
}

template <typename KindRecords>
std::vector<ElementPropertyReading> ReadProperties(const VehicleProject& project,
                                                   const ElementReference& element) {
    using Record = typename KindRecords::Record;
    const Record* const record = FindRecord<KindRecords>(project, element.name);
    if (record == nullptr) {
        throw std::invalid_argument("vehicle project contains no " +
                                    std::string(KindRecords::kKindDescription) + " named '" +
                                    element.name + "'");
    }
    // The descriptions list the properties in the order these loops read them.
    const std::span<const ElementPropertyDescription> descriptions = Descriptions<Record>();
    std::vector<ElementPropertyReading> readings;
    readings.reserve(descriptions.size());
    for (const RecordProperty<Record>& property : kRecordProperties<Record>) {
        readings.push_back({descriptions[readings.size()],
                            std::visit([record](auto member) { return ValueOf(record->*member); },
                                       property.member)});
    }
    if constexpr (std::is_same_v<Record, VehicleRigidBodyDefinition>) {
        for (const BodyKeyedProperty& property : kBodyKeyedProperties) {
            readings.push_back(
                {descriptions[readings.size()],
                 std::visit(
                     [&project, &element](auto member) -> std::optional<PropertyValue> {
                         const auto* const keyed =
                             FindBodyKeyedRecord(project, element.name, member);
                         if (keyed == nullptr) {
                             return std::nullopt;
                         }
                         return ValueOf(keyed->*member);
                     },
                     property.member)});
        }
    }
    // A property whose value is absent for this element is not editable for
    // it. Assign refuses exactly these edits: a station offset or placement
    // offset whose record is missing is the only editable property a value
    // can be absent from.
    for (ElementPropertyReading& reading : readings) {
        reading.description.editable = reading.description.editable && reading.value.has_value();
    }
    return readings;
}

template <typename KindRecords>
std::expected<void, std::string> Assign(VehicleProject& project, const ElementReference& element,
                                        std::string_view property_name,
                                        const PropertyValue& value) {
    using Record = typename KindRecords::Record;
    Record* const record = FindRecord<KindRecords>(project, element.name);
    if (record == nullptr) {
        return std::unexpected("this project contains no " +
                               std::string(KindRecords::kKindDescription) + " named '" +
                               element.name + "'");
    }
    const std::span<const ElementPropertyDescription> descriptions = Descriptions<Record>();
    const auto described =
        std::ranges::find(descriptions, property_name, &ElementPropertyDescription::name);
    const std::string quoted_property_name = "'" + std::string(property_name) + "'";
    if (described == descriptions.end()) {
        return std::unexpected("a " + std::string(KindRecords::kKindDescription) +
                               " has no property named " + quoted_property_name);
    }
    if (!described->editable) {
        return std::unexpected(quoted_property_name + " is read-only");
    }
    const PropertyValueKind given_kind = ValueKindOf(value);
    if (given_kind != described->value_kind) {
        return std::unexpected(quoted_property_name + " holds " +
                               DescribeValueKind(described->value_kind) +
                               ", but the value given is " + DescribeValueKind(given_kind));
    }
    // Every editable property holds a number or a 3-vector.
    const bool finite = std::holds_alternative<double>(value)
                            ? std::isfinite(std::get<double>(value))
                            : std::get<Eigen::Vector3d>(value).allFinite();
    if (!finite) {
        return std::unexpected(quoted_property_name + " must be finite");
    }

    const auto index = static_cast<std::size_t>(described - descriptions.begin());
    if constexpr (std::is_same_v<Record, VehicleRigidBodyDefinition>) {
        if (index >= kRecordProperties<Record>.size()) {
            return std::visit(
                [&project, &element, record,
                 &value](auto member) -> std::expected<void, std::string> {
                    auto* const keyed = FindBodyKeyedRecord(project, element.name, member);
                    if (keyed == nullptr) {
                        return std::unexpected(MissingBodyKeyedRecord(*record, member));
                    }
                    keyed->*member = std::get<double>(value);
                    return {};
                },
                kBodyKeyedProperties[index - kRecordProperties<Record>.size()].member);
        }
    }
    std::visit(
        [record, &value](auto member) {
            using Value = PointedMemberType<decltype(member)>;
            if constexpr (kEditableNumberOrVector3<Value>) {
                record->*member = std::get<Value>(value);
            }
        },
        kRecordProperties<Record>[index].member);
    return {};
}

}  // namespace

std::span<const ElementPropertyDescription> DescribeElementProperties(ElementKind kind) {
    return VisitElementKind(kind, []<typename KindRecords>(KindRecords) {
        return Descriptions<typename KindRecords::Record>();
    });
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

std::expected<void, std::string> internal::AssignElementProperty(VehicleProject& project,
                                                                 const ElementReference& element,
                                                                 std::string_view property_name,
                                                                 const PropertyValue& value) {
    return VisitElementKind(element.kind, [&]<typename KindRecords>(KindRecords) {
        return Assign<KindRecords>(project, element, property_name, value);
    });
}

}  // namespace orvd::vehicle_project_editing
