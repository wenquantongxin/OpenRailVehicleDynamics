#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/vehicle_project/generate_assembly_inputs_from_vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project/vehicle_project_free_body_placements.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"
#include "vehicle_project/configuration_record_comparison.h"
#include "vehicle_project/vehicle_project_comparison.h"
#include "vehicle_project_editing/vehicle_project_editing_verification.h"

// The editing layer, verified by properties rather than recorded outputs:
// every property of every element kind is described, read from the member of
// its name and, except a rotation, editable; a reference states the kinds it
// may name and whether it may name none, a token its choices and a curve its
// row fields, as stated here from the contract; every property but a name and
// a body's freedom, which the structural editing verification covers, is
// written to its own member and to nothing else; an edit of a GZ18 primary
// suspension changes that one member and is undone and redone with a revision
// that only increases; each kind of refused edit names its element and
// property and changes nothing; both bundled vehicles pass the
// mechanical check, and a project the assembly refuses is reported, not
// thrown, while the older preview stays; a project a vehicle file's
// declaration rules refuse, or stating no gravity, is refused before any
// assembly; the preview places free bodies where
// their station offsets and placements say, moves a body along the track when
// its station offset is edited, puts every force-element end, frame and joint
// where its body's pose takes it, puts every wheel the lateral datum
// magnitude from its carrier's origin along an outward axis pointing to the
// wheel's side, holds IRW's tie rods at their Ball-RPY default angles, and
// reports bodies hanging from an unplaced free body, and the wheels turning
// with them, as not drawable.

namespace {

using namespace orvd;
using configuration_record_comparison::Members;
using configuration_record_comparison::Same;
using editing_verification::Fail;
using editing_verification::PlacedProject;
using editing_verification::ReferenceVehicle;
using editing_verification::Require;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::FreeBodyPlacement;
using vehicle_project::VehicleProject;
using vehicle_project_comparison::SameProject;
using vehicle_project_editing::ElementOwnName;
using vehicle_project_editing::MechanicalCheckReport;
using vehicle_project_editing::PropertyDescription;
using vehicle_project_editing::PropertyReading;
using vehicle_project_editing::PropertySubject;
using vehicle_project_editing::PropertyToken;
using vehicle_project_editing::PropertyValue;
using vehicle_project_editing::PropertyValueKind;
using vehicle_project_editing::ReferencedElementName;
using vehicle_project_editing::RigidBodyDrawing;
using vehicle_project_editing::SaturatedPiecewiseLinearDamperCurve;
using vehicle_project_editing::SubjectEditRefusal;
using vehicle_project_editing::VehicleProjectEditingSession;
using vehicle_project_editing::VehicleProjectPreview;
using vehicle_project_editing::WheelDrawing;

// The preview and this test compose the same rigid transforms in different
// orders, so they agree to a few rounding steps of the largest coordinate
// involved, at most about twenty metres across a vehicle; 64 steps are
// allowed.
constexpr double kCompositionToleranceMeters = 64.0 * std::numeric_limits<double>::epsilon() * 20.0;
constexpr double kCompositionToleranceRadians = 64.0 * std::numeric_limits<double>::epsilon();

std::string Describe(const ElementReference& element) {
    return "element '" + element.name + "' of kind " +
           std::to_string(std::to_underlying(element.kind));
}

std::string Scientific(double value) {
    std::ostringstream text;
    text << std::scientific << std::setprecision(3) << value;
    return text.str();
}

// The record of `records` named `name`.
template <typename Record>
const Record& Named(const std::vector<Record>& records, const std::string& name) {
    const auto found = std::ranges::find(records, name, &Record::name);
    Require(found != records.end(), "no record is named '" + name + "'");
    return *found;
}

// The kind each held type is, stated here independently of the library.
PropertyValueKind KindOfHeldValue(const PropertyValue& value) {
    return std::visit(
        [](const auto& held) {
            using Held = std::remove_cvref_t<decltype(held)>;
            if constexpr (std::is_same_v<Held, double>) {
                return PropertyValueKind::kNumber;
            } else if constexpr (std::is_same_v<Held, Eigen::Vector3d>) {
                return PropertyValueKind::kVector3;
            } else if constexpr (std::is_same_v<Held, bool>) {
                return PropertyValueKind::kBoolean;
            } else if constexpr (std::is_same_v<Held, Eigen::Matrix3d>) {
                return PropertyValueKind::kRotation;
            } else if constexpr (std::is_same_v<Held, ElementOwnName>) {
                return PropertyValueKind::kElementName;
            } else if constexpr (std::is_same_v<Held, ReferencedElementName>) {
                return PropertyValueKind::kElementReference;
            } else if constexpr (std::is_same_v<Held, PropertyToken>) {
                return PropertyValueKind::kToken;
            } else {
                return PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve;
            }
        },
        value);
}

bool SameValue(const PropertyValue& expected, const PropertyValue& actual) {
    return expected.index() == actual.index() &&
           std::visit(
               [&actual](const auto& expected_value) {
                   using Held = std::remove_cvref_t<decltype(expected_value)>;
                   return Same(expected_value, std::get<Held>(actual));
               },
               expected);
}

bool SameReading(const std::optional<PropertyValue>& expected,
                 const std::optional<PropertyValue>& actual) {
    return expected.has_value() == actual.has_value() &&
           (!expected.has_value() || SameValue(*expected, *actual));
}

// --- Property descriptions -------------------------------------------------

// One element of every kind, every member of it holding a value no other
// member of its record holds, so that a property can only read the value of
// the member it names. Its references name elements of the probe, which an
// edit requires of a reference it sets; the probe is read and edited, never
// checked.
VehicleProject MakePropertyProbeProject() {
    configuration::VehicleDefinition vehicle;
    vehicle.vehicle_name = "property_probe";
    vehicle.rigid_bodies = {
        {.name = "probe_body",
         .moves_freely_in_world = true,
         .mass_kilograms = 1.5,
         .center_of_mass_in_body_frame_meters = Eigen::Vector3d(2.1, 2.2, 2.3),
         .inertia_moments_about_center_of_mass_kilogram_square_meters =
             Eigen::Vector3d(3.1, 3.2, 3.3),
         .inertia_products_about_center_of_mass_kilogram_square_meters =
             Eigen::Vector3d(4.1, 4.2, 4.3)},
        // A jointed body has neither a station offset nor placement offsets;
        // an unplaced free body has a station offset only.
        {.name = "probe_jointed_body"},
        {.name = "probe_unplaced_body", .moves_freely_in_world = true},
    };
    vehicle.mechanical_track_station_layout.free_body_station_offsets = {
        {.body_name = "probe_body", .station_offset_meters = 5.0},
        {.body_name = "probe_unplaced_body", .station_offset_meters = 5.3}};
    vehicle.fixed_frames = {
        {.name = "probe_frame",
         .body_name = "probe_jointed_body",
         .position_in_body_frame_meters = Eigen::Vector3d(6.1, 6.2, 6.3),
         .rotation_in_body_frame =
             Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix()}};
    vehicle.revolute_joints = {{.name = "probe_revolute_joint",
                                .parent_frame_name = "probe_frame",
                                .child_frame_name = "probe_jointed_body",
                                .axis_in_parent_frame = Eigen::Vector3d(7.1, 7.2, 7.3),
                                .damping_newton_metre_seconds_per_radian = 7.4}};
    vehicle.ball_rpy_joints = {
        {.name = "probe_ball_rpy_joint",
         .parent_frame_name = "probe_body",
         .child_frame_name = "probe_unplaced_body",
         .default_roll_pitch_yaw_angles_radians = Eigen::Vector3d(8.1, 8.2, 8.3)}};
    vehicle.weld_joints = {{.name = "probe_weld_joint",
                            .parent_frame_name = "probe_frame",
                            .child_frame_name = "probe_unplaced_body"}};
    vehicle.translational_spring_dampers = {
        {.name = "probe_translational_spring_damper",
         .reference_frame_name = "probe_frame",
         .opposite_frame_name = "probe_body",
         .stiffness_newtons_per_meter = Eigen::Vector3d(9.1, 9.2, 9.3),
         .damping_newton_seconds_per_meter = Eigen::Vector3d(10.1, 10.2, 10.3)}};
    vehicle.roll_spring_damper_couples = {{.name = "probe_roll_spring_damper_couple",
                                           .reference_frame_name = "probe_body",
                                           .opposite_frame_name = "probe_frame",
                                           .stiffness_newton_meters_per_radian = 11.1,
                                           .damping_newton_meter_seconds_per_radian = 11.2}};
    vehicle.series_spring_viscous_dampers = {{.name = "probe_series_spring_viscous_damper",
                                              .reference_frame_name = "probe_frame",
                                              .opposite_frame_name = "probe_unplaced_body",
                                              .axis = "lateral",
                                              .series_stiffness_newtons_per_meter = 12.1,
                                              .series_damping_newton_seconds_per_meter = 12.2}};
    vehicle.point_to_point_series_spring_bilinear_dampers = {
        {.name = "probe_bilinear_damper",
         .reference_frame_name = "probe_unplaced_body",
         .opposite_frame_name = "probe_frame",
         .series_stiffness_newtons_per_meter = 13.1,
         .damping_below_knee_newton_seconds_per_meter = 13.2,
         .knee_force_newtons = 13.3,
         .damping_above_knee_newton_seconds_per_meter = 13.4}};
    vehicle.saturated_piecewise_linear_dampers = {{.name = "probe_saturated_damper",
                                                   .reference_frame_name = "probe_jointed_body",
                                                   .opposite_frame_name = "probe_frame",
                                                   .axis = "longitudinal",
                                                   .curve = {{0.0, 0.0}, {14.1, 14.2}}}};
    vehicle.reference_frame_linear_bushings = {
        {.name = "probe_bushing",
         .reference_frame_name = "probe_frame",
         .opposite_frame_name = "probe_jointed_body",
         .translational_stiffness_newtons_per_meter = Eigen::Vector3d(15.1, 15.2, 15.3),
         .translational_damping_newton_seconds_per_meter = Eigen::Vector3d(16.1, 16.2, 16.3),
         .rotational_damping_newton_meter_seconds_per_radian = Eigen::Vector3d(17.1, 17.2, 17.3)}};
    vehicle.point_to_point_linear_spring_dampers = {{.name = "probe_linear_spring_damper",
                                                     .reference_frame_name = "probe_body",
                                                     .opposite_frame_name = "probe_jointed_body",
                                                     .nominal_length_meters = 18.1,
                                                     .stiffness_newtons_per_meter = 18.2,
                                                     .damping_newton_seconds_per_meter = 18.3}};
    vehicle.point_to_point_hysteretic_springs = {
        {.name = "probe_hysteretic_spring",
         .reference_frame_name = "probe_jointed_body",
         .opposite_frame_name = "probe_body",
         .nominal_length_meters = 19.1,
         .stiffness_newtons_per_meter = 19.2,
         .hysteresis_ratio = 19.3,
         .hysteresis_transition_speed_meters_per_second = 19.4}};

    configuration::VehicleContactRecord contact_record;
    contact_record.declaration.carriers = {
        {.carrier_name = "probe_carrier",
         .body_name = "probe_body",
         .rotation_body_from_nonspinning_wheel_profile =
             Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix()},
        {.carrier_name = "probe_second_carrier", .body_name = "probe_unplaced_body"}};
    contact_record.declaration.interfaces = {
        {.interface_name = "probe_interface",
         .carrier_name = "probe_carrier",
         .wheel_body_name = "probe_jointed_body",
         .side = wheel_rail_contact::WheelSide::kLeft,
         .independent_wheel_revolute_joint_name = "probe_revolute_joint"},
        // A wheel that is not separately jointed names no joint.
        {.interface_name = "probe_interface_without_joint",
         .carrier_name = "probe_second_carrier",
         .wheel_body_name = "probe_body",
         .side = wheel_rail_contact::WheelSide::kRight,
         .independent_wheel_revolute_joint_name = std::nullopt},
    };

    return {.vehicle_definition = std::move(vehicle),
            .contact_record = std::move(contact_record),
            .free_body_placements = {{.body_name = "probe_body",
                                      .lateral_offset_in_local_track_frame_meters = 5.1,
                                      .vertical_offset_in_local_track_frame_meters = 5.2}}};
}

struct ExpectedProperty {
    std::string name;
    PropertyValue value;
};

struct ExpectedElementKind {
    ElementKind kind;
    std::string element_name;
    // How many properties the kind must have: the members of its record,
    // counted by the structured binding in Members(), and for a rigid body its
    // station offset and its two placement offsets.
    std::size_t property_count;
    // Each property name paired with the probe member of the same name.
    std::vector<ExpectedProperty> properties;
};

template <typename Record>
std::size_t MemberCount(const Record& record) {
    return std::tuple_size_v<decltype(Members(record))>;
}

// The token a wheel side is written as, stated here independently of the
// library.
std::string WheelSideToken(wheel_rail_contact::WheelSide side) {
    return side == wheel_rail_contact::WheelSide::kLeft ? "left" : "right";
}

std::vector<ExpectedElementKind> ExpectedProbeProperties(const VehicleProject& probe) {
    const configuration::VehicleDefinition& vehicle = probe.vehicle_definition;
    const configuration::VehicleContactDeclaration& declaration = probe.contact_record->declaration;
    const auto& body = vehicle.rigid_bodies.front();
    const auto& station_offset =
        vehicle.mechanical_track_station_layout.free_body_station_offsets.front();
    const FreeBodyPlacement& placement = probe.free_body_placements.front();
    const auto& frame = vehicle.fixed_frames.front();
    const auto& revolute = vehicle.revolute_joints.front();
    const auto& ball_rpy = vehicle.ball_rpy_joints.front();
    const auto& weld = vehicle.weld_joints.front();
    const auto& translational = vehicle.translational_spring_dampers.front();
    const auto& roll = vehicle.roll_spring_damper_couples.front();
    const auto& series = vehicle.series_spring_viscous_dampers.front();
    const auto& bilinear = vehicle.point_to_point_series_spring_bilinear_dampers.front();
    const auto& saturated = vehicle.saturated_piecewise_linear_dampers.front();
    const auto& bushing = vehicle.reference_frame_linear_bushings.front();
    const auto& linear = vehicle.point_to_point_linear_spring_dampers.front();
    const auto& hysteretic = vehicle.point_to_point_hysteretic_springs.front();
    const auto& carrier = declaration.carriers.front();
    const auto& interface = declaration.interfaces.front();
    const auto own_name = [](const std::string& name) { return ElementOwnName{name}; };
    const auto reference = [](const std::optional<std::string>& name) {
        return ReferencedElementName{name};
    };
    return {
        {ElementKind::kRigidBody,
         body.name,
         MemberCount(body) + 3,
         {{"name", own_name(body.name)},
          {"moves_freely_in_world", body.moves_freely_in_world},
          {"mass_kilograms", body.mass_kilograms},
          {"center_of_mass_in_body_frame_meters", body.center_of_mass_in_body_frame_meters},
          {"inertia_moments_about_center_of_mass_kilogram_square_meters",
           body.inertia_moments_about_center_of_mass_kilogram_square_meters},
          {"inertia_products_about_center_of_mass_kilogram_square_meters",
           body.inertia_products_about_center_of_mass_kilogram_square_meters},
          {"mechanical_track_station_layout.station_offset_meters",
           station_offset.station_offset_meters},
          {"placement.lateral_offset_in_local_track_frame_meters",
           placement.lateral_offset_in_local_track_frame_meters},
          {"placement.vertical_offset_in_local_track_frame_meters",
           placement.vertical_offset_in_local_track_frame_meters}}},
        {ElementKind::kFixedFrame,
         frame.name,
         MemberCount(frame),
         {{"name", own_name(frame.name)},
          {"body_name", reference(frame.body_name)},
          {"position_in_body_frame_meters", frame.position_in_body_frame_meters},
          {"rotation_in_body_frame", frame.rotation_in_body_frame}}},
        {ElementKind::kRevoluteJoint,
         revolute.name,
         MemberCount(revolute),
         {{"name", own_name(revolute.name)},
          {"parent_frame_name", reference(revolute.parent_frame_name)},
          {"child_frame_name", reference(revolute.child_frame_name)},
          {"axis_in_parent_frame", revolute.axis_in_parent_frame},
          {"damping_newton_metre_seconds_per_radian",
           revolute.damping_newton_metre_seconds_per_radian}}},
        {ElementKind::kBallRpyJoint,
         ball_rpy.name,
         MemberCount(ball_rpy),
         {{"name", own_name(ball_rpy.name)},
          {"parent_frame_name", reference(ball_rpy.parent_frame_name)},
          {"child_frame_name", reference(ball_rpy.child_frame_name)},
          {"default_roll_pitch_yaw_angles_radians",
           ball_rpy.default_roll_pitch_yaw_angles_radians}}},
        {ElementKind::kWeldJoint,
         weld.name,
         MemberCount(weld),
         {{"name", own_name(weld.name)},
          {"parent_frame_name", reference(weld.parent_frame_name)},
          {"child_frame_name", reference(weld.child_frame_name)}}},
        {ElementKind::kTranslationalSpringDamper,
         translational.name,
         MemberCount(translational),
         {{"name", own_name(translational.name)},
          {"reference_frame_name", reference(translational.reference_frame_name)},
          {"opposite_frame_name", reference(translational.opposite_frame_name)},
          {"stiffness_newtons_per_meter", translational.stiffness_newtons_per_meter},
          {"damping_newton_seconds_per_meter", translational.damping_newton_seconds_per_meter}}},
        {ElementKind::kRollSpringDamperCouple,
         roll.name,
         MemberCount(roll),
         {{"name", own_name(roll.name)},
          {"reference_frame_name", reference(roll.reference_frame_name)},
          {"opposite_frame_name", reference(roll.opposite_frame_name)},
          {"stiffness_newton_meters_per_radian", roll.stiffness_newton_meters_per_radian},
          {"damping_newton_meter_seconds_per_radian",
           roll.damping_newton_meter_seconds_per_radian}}},
        {ElementKind::kSeriesSpringViscousDamper,
         series.name,
         MemberCount(series),
         {{"name", own_name(series.name)},
          {"reference_frame_name", reference(series.reference_frame_name)},
          {"opposite_frame_name", reference(series.opposite_frame_name)},
          {"axis", PropertyToken{series.axis}},
          {"series_stiffness_newtons_per_meter", series.series_stiffness_newtons_per_meter},
          {"series_damping_newton_seconds_per_meter",
           series.series_damping_newton_seconds_per_meter}}},
        {ElementKind::kPointToPointSeriesSpringBilinearDamper,
         bilinear.name,
         MemberCount(bilinear),
         {{"name", own_name(bilinear.name)},
          {"reference_frame_name", reference(bilinear.reference_frame_name)},
          {"opposite_frame_name", reference(bilinear.opposite_frame_name)},
          {"series_stiffness_newtons_per_meter", bilinear.series_stiffness_newtons_per_meter},
          {"damping_below_knee_newton_seconds_per_meter",
           bilinear.damping_below_knee_newton_seconds_per_meter},
          {"knee_force_newtons", bilinear.knee_force_newtons},
          {"damping_above_knee_newton_seconds_per_meter",
           bilinear.damping_above_knee_newton_seconds_per_meter}}},
        {ElementKind::kSaturatedPiecewiseLinearDamper,
         saturated.name,
         MemberCount(saturated),
         {{"name", own_name(saturated.name)},
          {"reference_frame_name", reference(saturated.reference_frame_name)},
          {"opposite_frame_name", reference(saturated.opposite_frame_name)},
          {"axis", PropertyToken{saturated.axis}},
          {"curve", saturated.curve}}},
        {ElementKind::kReferenceFrameLinearBushing,
         bushing.name,
         MemberCount(bushing),
         {{"name", own_name(bushing.name)},
          {"reference_frame_name", reference(bushing.reference_frame_name)},
          {"opposite_frame_name", reference(bushing.opposite_frame_name)},
          {"translational_stiffness_newtons_per_meter",
           bushing.translational_stiffness_newtons_per_meter},
          {"translational_damping_newton_seconds_per_meter",
           bushing.translational_damping_newton_seconds_per_meter},
          {"rotational_damping_newton_meter_seconds_per_radian",
           bushing.rotational_damping_newton_meter_seconds_per_radian}}},
        {ElementKind::kPointToPointLinearSpringDamper,
         linear.name,
         MemberCount(linear),
         {{"name", own_name(linear.name)},
          {"reference_frame_name", reference(linear.reference_frame_name)},
          {"opposite_frame_name", reference(linear.opposite_frame_name)},
          {"nominal_length_meters", linear.nominal_length_meters},
          {"stiffness_newtons_per_meter", linear.stiffness_newtons_per_meter},
          {"damping_newton_seconds_per_meter", linear.damping_newton_seconds_per_meter}}},
        {ElementKind::kPointToPointHystereticSpring,
         hysteretic.name,
         MemberCount(hysteretic),
         {{"name", own_name(hysteretic.name)},
          {"reference_frame_name", reference(hysteretic.reference_frame_name)},
          {"opposite_frame_name", reference(hysteretic.opposite_frame_name)},
          {"nominal_length_meters", hysteretic.nominal_length_meters},
          {"stiffness_newtons_per_meter", hysteretic.stiffness_newtons_per_meter},
          {"hysteresis_ratio", hysteretic.hysteresis_ratio},
          {"hysteresis_transition_speed_meters_per_second",
           hysteretic.hysteresis_transition_speed_meters_per_second}}},
        {ElementKind::kContactCarrier,
         carrier.carrier_name,
         MemberCount(carrier),
         {{"carrier_name", own_name(carrier.carrier_name)},
          {"body_name", reference(carrier.body_name)},
          {"rotation_body_from_nonspinning_wheel_profile",
           carrier.rotation_body_from_nonspinning_wheel_profile}}},
        {ElementKind::kContactInterface,
         interface.interface_name,
         MemberCount(interface),
         {{"interface_name", own_name(interface.interface_name)},
          {"carrier_name", reference(interface.carrier_name)},
          {"wheel_body_name", reference(interface.wheel_body_name)},
          {"side", PropertyToken{WheelSideToken(interface.side)}},
          {"independent_wheel_revolute_joint_name",
           reference(interface.independent_wheel_revolute_joint_name)}}},
    };
}

using PropertyName = std::pair<ElementKind, std::string>;

// The properties that cannot be edited, stated here independently of the
// descriptions: the rotations. Every other property is editable.
std::set<PropertyName> ReadOnlyProperties() {
    return {{ElementKind::kFixedFrame, "rotation_in_body_frame"},
            {ElementKind::kContactCarrier, "rotation_body_from_nonspinning_wheel_profile"}};
}

struct ExpectedReference {
    std::vector<ElementKind> referenced_kinds;
    bool may_name_none;
};

// Who names whom, stated here from the contract independently of the tables:
// a fixed frame names its rigid body; a joint or force element end names a
// fixed frame or a rigid body; a contact carrier names its rigid body; a
// contact interface names its carrier, its wheel body and, optionally, a
// revolute joint.
std::map<PropertyName, ExpectedReference> ExpectedReferences() {
    const std::vector<ElementKind> body{ElementKind::kRigidBody};
    const std::vector<ElementKind> frame{ElementKind::kRigidBody, ElementKind::kFixedFrame};
    std::map<PropertyName, ExpectedReference> references{
        {{ElementKind::kFixedFrame, "body_name"}, {body, false}},
        {{ElementKind::kContactCarrier, "body_name"}, {body, false}},
        {{ElementKind::kContactInterface, "carrier_name"}, {{ElementKind::kContactCarrier}, false}},
        {{ElementKind::kContactInterface, "wheel_body_name"}, {body, false}},
        {{ElementKind::kContactInterface, "independent_wheel_revolute_joint_name"},
         {{ElementKind::kRevoluteJoint}, true}},
    };
    for (const ElementKind joint :
         {ElementKind::kRevoluteJoint, ElementKind::kBallRpyJoint, ElementKind::kWeldJoint}) {
        references.insert({{joint, "parent_frame_name"}, {frame, false}});
        references.insert({{joint, "child_frame_name"}, {frame, false}});
    }
    for (const ElementKind force_element :
         {ElementKind::kTranslationalSpringDamper, ElementKind::kRollSpringDamperCouple,
          ElementKind::kSeriesSpringViscousDamper,
          ElementKind::kPointToPointSeriesSpringBilinearDamper,
          ElementKind::kSaturatedPiecewiseLinearDamper, ElementKind::kReferenceFrameLinearBushing,
          ElementKind::kPointToPointLinearSpringDamper,
          ElementKind::kPointToPointHystereticSpring}) {
        references.insert({{force_element, "reference_frame_name"}, {frame, false}});
        references.insert({{force_element, "opposite_frame_name"}, {frame, false}});
    }
    return references;
}

// Every token property's choices, stated here independently of the tables:
// the axes configuration's ParseAxis accepts, and a wheel's two sides.
std::map<PropertyName, std::vector<std::string>> ExpectedTokenChoices() {
    const std::vector<std::string> axes{"longitudinal", "lateral", "vertical"};
    return {{{ElementKind::kSeriesSpringViscousDamper, "axis"}, axes},
            {{ElementKind::kSaturatedPiecewiseLinearDamper, "axis"}, axes},
            {{ElementKind::kContactInterface, "side"},
             {WheelSideToken(wheel_rail_contact::WheelSide::kLeft),
              WheelSideToken(wheel_rail_contact::WheelSide::kRight)}}};
}

// What a description states beyond name, value kind and editability is what
// this test states for the property: a reference's kinds and whether it may
// name none, a token's choices, and a curve's row fields, which are the
// members of a curve point under their own names.
void RequireDescribedAsStated(const PropertyDescription& description, const PropertyName& property,
                              const std::string& what) {
    const std::map<PropertyName, ExpectedReference> references = ExpectedReferences();
    const auto reference = references.find(property);
    Require((description.value_kind == PropertyValueKind::kElementReference) ==
                (reference != references.end()),
            what + " is described as a reference where this test states none, or the reverse");
    Require(reference == references.end()
                ? description.referenced_kinds.empty() && !description.may_name_none
                : std::ranges::equal(description.referenced_kinds,
                                     reference->second.referenced_kinds) &&
                      description.may_name_none == reference->second.may_name_none,
            what + " is described as naming other kinds than stated here");

    const std::map<PropertyName, std::vector<std::string>> token_choices = ExpectedTokenChoices();
    const auto choices = token_choices.find(property);
    Require(
        (description.value_kind == PropertyValueKind::kToken) == (choices != token_choices.end()),
        what + " is described as a token where this test states none, or the reverse");
    Require(choices == token_choices.end()
                ? description.choices.empty()
                : std::ranges::equal(description.choices, choices->second),
            what + " is described with other choices than stated here");

    if (description.value_kind != PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve) {
        Require(description.row_fields.empty(), what + " is described with row fields");
        return;
    }
    const configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition point{
        .relative_velocity_meters_per_second = 21.1, .force_newtons = 21.2};
    const std::vector<std::pair<std::string, double>> members_by_name{
        {"relative_velocity_meters_per_second", point.relative_velocity_meters_per_second},
        {"force_newtons", point.force_newtons}};
    Require(description.row_fields.size() == MemberCount(point) &&
                std::ranges::equal(description.row_fields, members_by_name,
                                   [&point](const auto& field, const auto& member) {
                                       return field.name == member.first &&
                                              point.*field.member == member.second;
                                   }),
            what + " does not give every curve point member as a row field of its own name");
}

using PropertyKey = std::tuple<ElementKind, std::string, std::string>;

// Every property of every element of the given kinds, by element kind,
// element name and property name.
std::map<PropertyKey, std::optional<PropertyValue>> ReadEveryProperty(
    const VehicleProject& project, std::span<const ExpectedElementKind> kinds) {
    std::map<PropertyKey, std::optional<PropertyValue>> values;
    for (const ExpectedElementKind& kind : kinds) {
        for (const std::string& element_name :
             vehicle_project_editing::CollectElementNames(project, kind.kind)) {
            for (PropertyReading& reading : vehicle_project_editing::ReadSubjectProperties(
                     project, ElementReference{kind.kind, element_name})) {
                values.emplace(
                    PropertyKey{kind.kind, element_name, std::string(reading.description.name)},
                    std::move(reading.value));
            }
        }
    }
    return values;
}

void VerifyPropertiesReadTheirMembers(const VehicleProject& probe,
                                      std::span<const ExpectedElementKind> kinds) {
    const std::set<PropertyName> read_only = ReadOnlyProperties();
    for (const ExpectedElementKind& expected : kinds) {
        const ElementReference element{expected.kind, expected.element_name};
        Require(expected.properties.size() == expected.property_count,
                "this test lists " + std::to_string(expected.properties.size()) +
                    " properties for " + Describe(element) + ", whose kind needs " +
                    std::to_string(expected.property_count));
        const std::span<const PropertyDescription> descriptions =
            vehicle_project_editing::DescribeElementProperties(expected.kind);
        const std::vector<PropertyReading> readings =
            vehicle_project_editing::ReadSubjectProperties(probe, element);
        Require(descriptions.size() == expected.property_count &&
                    readings.size() == expected.property_count,
                Describe(element) + " is described or read with " +
                    std::to_string(descriptions.size()) + " or " + std::to_string(readings.size()) +
                    " properties instead of " + std::to_string(expected.property_count));
        for (std::size_t index = 0; index < descriptions.size(); ++index) {
            const PropertyDescription& description = descriptions[index];
            const PropertyDescription& read_description = readings[index].description;
            const ExpectedProperty& property = expected.properties[index];
            const std::string what = Describe(element) + " property '" + property.name + "'";
            Require(description.name == property.name,
                    what + " is described as '" + std::string(description.name) + "'");
            // These probe elements hold every value, so their readings carry
            // the kind's descriptions, editability included.
            Require(read_description.name == description.name &&
                        read_description.value_kind == description.value_kind &&
                        read_description.editable == description.editable,
                    what + " is read with another description");
            Require(description.value_kind == KindOfHeldValue(property.value),
                    what + " is described with the wrong value kind");
            Require(description.editable != read_only.contains({expected.kind, property.name}),
                    what + " is not editable exactly when this test calls it read-only");
            RequireDescribedAsStated(description, {expected.kind, property.name}, what);
            Require(readings[index].value.has_value() &&
                        SameValue(property.value, *readings[index].value),
                    what + " does not read the member of that name");
        }
    }
}

// A property whose value is absent for an element reads as not editable for
// it, although its kind may call the property editable. A reference that
// names none is not absent: it holds no name and stays editable.
void VerifyAbsentValues(const VehicleProject& probe) {
    const auto reading_of = [&probe](const ElementReference& element,
                                     std::string_view property_name) {
        const std::vector<PropertyReading> readings =
            vehicle_project_editing::ReadSubjectProperties(probe, element);
        const auto reading =
            std::ranges::find_if(readings, [property_name](const PropertyReading& candidate) {
                return candidate.description.name == property_name;
            });
        Require(reading != readings.end(),
                Describe(element) + " has no property '" + std::string(property_name) + "'");
        return *reading;
    };
    const auto require_absent = [&reading_of](const ElementReference& element,
                                              const std::string& property_name,
                                              const std::string& reason) {
        const PropertyReading reading = reading_of(element, property_name);
        Require(!reading.value.has_value() && !reading.description.editable,
                "'" + element.name + "' " + reason + ", but its " + property_name +
                    " has a value or reads as editable");
    };
    // A jointed body is neither in the mechanical track station layout nor
    // placed; an unplaced free body is in the layout without a placement.
    const ElementReference jointed_body{ElementKind::kRigidBody, "probe_jointed_body"};
    const ElementReference unplaced_body{ElementKind::kRigidBody, "probe_unplaced_body"};
    for (const ElementReference& body : {jointed_body, unplaced_body}) {
        for (const std::string property_name :
             {"placement.lateral_offset_in_local_track_frame_meters",
              "placement.vertical_offset_in_local_track_frame_meters"}) {
            require_absent(body, property_name, "has no placement");
        }
    }
    require_absent(jointed_body, "mechanical_track_station_layout.station_offset_meters",
                   "has no entry in the mechanical track station layout");
    const PropertyReading unplaced_station_offset =
        reading_of(unplaced_body, "mechanical_track_station_layout.station_offset_meters");
    Require(unplaced_station_offset.value.has_value() &&
                SameValue(PropertyValue(probe.vehicle_definition.mechanical_track_station_layout
                                            .free_body_station_offsets.back()
                                            .station_offset_meters),
                          *unplaced_station_offset.value) &&
                unplaced_station_offset.description.editable,
            "'probe_unplaced_body' does not read its own station offset as editable");
    const PropertyReading no_joint =
        reading_of({ElementKind::kContactInterface, "probe_interface_without_joint"},
                   "independent_wheel_revolute_joint_name");
    Require(no_joint.value.has_value() &&
                SameValue(ReferencedElementName{std::nullopt}, *no_joint.value) &&
                no_joint.description.editable,
            "an interface naming no independent-wheel joint does not read a reference naming "
            "none that can be edited");
}

// What a reading calls editable is exactly what an edit accepts: writing an
// editable reading's own value back is accepted, and an edit of every other
// reading, with a value of its kind, is refused.
void VerifyReadingsStateWhatEditsAccept(const VehicleProject& probe,
                                        std::span<const ExpectedElementKind> kinds) {
    VehicleProjectEditingSession session(probe);
    for (const ExpectedElementKind& kind : kinds) {
        for (const std::string& element_name :
             vehicle_project_editing::CollectElementNames(probe, kind.kind)) {
            const ElementReference element{kind.kind, element_name};
            for (const PropertyReading& reading :
                 vehicle_project_editing::ReadSubjectProperties(probe, element)) {
                const std::string property_name(reading.description.name);
                const std::string what = Describe(element) + " property '" + property_name + "'";
                // A value is absent only from a station offset or a placement
                // offset, which hold numbers.
                const PropertyValue value =
                    reading.value.has_value() ? *reading.value : PropertyValue(0.0);
                const auto edited = session.SetSubjectProperty(element, property_name, value);
                if (reading.description.editable) {
                    Require(edited.has_value(),
                            what +
                                " reads as editable, but writing its own value back was "
                                "refused: " +
                                (edited.has_value() ? "" : edited.error().reason));
                    session.Undo();
                } else {
                    Require(!edited.has_value(),
                            what + " reads as not editable, but an edit of it was accepted");
                }
            }
        }
    }
    Require(SameProject(probe, session.project()),
            "writing editable readings back and refused edits changed the probe");
}

// Elements are listed in declaration order, and an element the project does
// not contain is refused by name.
void VerifyElementLookup(const VehicleProject& probe) {
    const std::vector<std::string> rigid_body_names =
        vehicle_project_editing::CollectElementNames(probe, ElementKind::kRigidBody);
    Require(rigid_body_names ==
                std::vector<std::string>{"probe_body", "probe_jointed_body", "probe_unplaced_body"},
            "the probe's rigid bodies are not collected in declaration order");

    // Without a contact record there are no contact elements to read.
    VehicleProject without_contact = probe;
    without_contact.contact_record.reset();
    Require(
        vehicle_project_editing::CollectElementNames(without_contact, ElementKind::kContactCarrier)
            .empty(),
        "a project without a contact record lists contact carriers");
    bool refused = false;
    try {
        (void)vehicle_project_editing::ReadSubjectProperties(
            without_contact, ElementReference{ElementKind::kContactCarrier, "probe_carrier"});
    } catch (const std::invalid_argument& error) {
        refused = std::string(error.what()).find("'probe_carrier'") != std::string::npos;
    }
    Require(refused,
            "reading a contact carrier of a project without contact record was not refused by "
            "name");
}

// A value of the same kind that differs from `value`, for a property whose
// reference or token is stated here: another element of a kind the reference
// may name, or none where it may name none, another choice of a token, a
// curve with one more point, and numbers moved by a different amount each.
PropertyValue ChangedValue(const VehicleProject& probe, const PropertyName& property,
                           const PropertyValue& value) {
    return std::visit(
        [&](const auto& held) -> PropertyValue {
            using Held = std::remove_cvref_t<decltype(held)>;
            if constexpr (std::is_same_v<Held, double>) {
                return held + 1000.0;
            } else if constexpr (std::is_same_v<Held, Eigen::Vector3d>) {
                return Eigen::Vector3d(held + Eigen::Vector3d(1000.0, 2000.0, 3000.0));
            } else if constexpr (std::is_same_v<Held, ReferencedElementName>) {
                const ExpectedReference reference = ExpectedReferences().at(property);
                if (reference.may_name_none && held.name.has_value()) {
                    return ReferencedElementName{std::nullopt};
                }
                for (const ElementKind kind : reference.referenced_kinds) {
                    for (const std::string& name :
                         vehicle_project_editing::CollectElementNames(probe, kind)) {
                        if (name != held.name) {
                            return ReferencedElementName{name};
                        }
                    }
                }
                Fail("the probe has no other element that '" + property.second + "' may name");
            } else if constexpr (std::is_same_v<Held, PropertyToken>) {
                const std::vector<std::string> choices = ExpectedTokenChoices().at(property);
                for (const std::string& choice : choices) {
                    if (choice != held.token) {
                        return PropertyToken{choice};
                    }
                }
                Fail("'" + property.second + "' has no other choice");
            } else if constexpr (std::is_same_v<Held, SaturatedPiecewiseLinearDamperCurve>) {
                SaturatedPiecewiseLinearDamperCurve curve = held;
                curve.push_back({.relative_velocity_meters_per_second =
                                     curve.back().relative_velocity_meters_per_second + 1.0,
                                 .force_newtons = curve.back().force_newtons + 2.0});
                return curve;
            } else {
                Fail("a name, a boolean or a rotation is not written here");
            }
        },
        value);
}

// Writing each editable property changes the member of its name and nothing
// else. Since the reads above are shown to come from the member each property
// names, reading every property after a write shows which member the write
// reached. An element's name and a body's freedom also change the records
// that follow them; the structural editing verification covers those.
void VerifyPropertiesWriteTheirMembers(const VehicleProject& probe,
                                       std::span<const ExpectedElementKind> kinds) {
    VehicleProjectEditingSession session(probe);
    const std::map<PropertyKey, std::optional<PropertyValue>> original =
        ReadEveryProperty(probe, kinds);
    const std::set<PropertyName> read_only = ReadOnlyProperties();
    for (const ExpectedElementKind& kind : kinds) {
        for (const ExpectedProperty& property : kind.properties) {
            const PropertyValueKind value_kind = KindOfHeldValue(property.value);
            if (read_only.contains({kind.kind, property.name}) ||
                value_kind == PropertyValueKind::kElementName ||
                value_kind == PropertyValueKind::kBoolean) {
                continue;
            }
            const ElementReference element{kind.kind, kind.element_name};
            const PropertyKey key{kind.kind, element.name, property.name};
            const std::string what = Describe(element) + " property '" + property.name + "'";
            const PropertyValue changed =
                ChangedValue(probe, {kind.kind, property.name}, *original.at(key));
            const auto written = session.SetSubjectProperty(element, property.name, changed);
            Require(
                written.has_value() && *written == PropertySubject{element},
                what + " was not written: " + (written.has_value() ? "" : written.error().reason));
            for (const auto& [read_key, read_value] : ReadEveryProperty(session.project(), kinds)) {
                if (read_key == key) {
                    Require(read_value.has_value() && SameValue(changed, *read_value),
                            what + " does not read back what was written");
                } else {
                    Require(SameReading(original.at(read_key), read_value),
                            "writing " + what + " changed property '" + std::get<2>(read_key) +
                                "' of '" + std::get<1>(read_key) + "'");
                }
            }
            session.Undo();
            Require(SameProject(probe, session.project()),
                    "undoing the write of " + what + " did not restore the probe");
        }
    }
}

// The station offset is the body's entry in the mechanical track station
// layout: writing it changes that entry and nothing else in the project.
void VerifyStationOffsetWritesItsLayoutEntry(const VehicleProject& probe) {
    constexpr double kEditedStationOffsetMeters = 6.5;
    const ElementReference body{ElementKind::kRigidBody, "probe_body"};
    VehicleProjectEditingSession session(probe);
    Require(session
                .SetSubjectProperty(body, "mechanical_track_station_layout.station_offset_meters",
                                    kEditedStationOffsetMeters)
                .has_value(),
            "the probe body's station offset was not written");
    VehicleProject expected = probe;
    auto& station_offsets =
        expected.vehicle_definition.mechanical_track_station_layout.free_body_station_offsets;
    const auto entry =
        std::ranges::find(station_offsets, body.name,
                          &configuration::VehicleFreeBodyStationOffsetDefinition::body_name);
    Require(entry != station_offsets.end(), "the probe body has no station offset entry");
    entry->station_offset_meters = kEditedStationOffsetMeters;
    Require(SameProject(expected, session.project()),
            "writing the probe body's station offset changed more than its layout entry");
}

// A rigid body's properties are shown in the contract's sections, in the order
// its properties are listed: its name in none, then degrees of freedom, mass
// and inertia, its mechanical station and its preview placement. No other
// kind's property, and no settings property, is in a section.
void VerifyPropertySections() {
    const std::vector<std::pair<std::string_view, std::optional<std::string_view>>>
        rigid_body_sections = {
            {"name", std::nullopt},
            {"moves_freely_in_world", "Degrees of freedom"},
            {"mass_kilograms", "Mass and inertia"},
            {"center_of_mass_in_body_frame_meters", "Mass and inertia"},
            {"inertia_moments_about_center_of_mass_kilogram_square_meters", "Mass and inertia"},
            {"inertia_products_about_center_of_mass_kilogram_square_meters", "Mass and inertia"},
            {"mechanical_track_station_layout.station_offset_meters", "Mechanical station"},
            {"placement.lateral_offset_in_local_track_frame_meters", "Preview placement"},
            {"placement.vertical_offset_in_local_track_frame_meters", "Preview placement"}};
    const std::span<const PropertyDescription> body_descriptions =
        vehicle_project_editing::DescribeElementProperties(ElementKind::kRigidBody);
    Require(body_descriptions.size() == rigid_body_sections.size(),
            "a rigid body has properties this test does not place in a section");
    for (std::size_t index = 0; index < body_descriptions.size(); ++index) {
        const PropertyDescription& description = body_descriptions[index];
        const auto& [name, section] = rigid_body_sections[index];
        Require(description.name == name &&
                    description.section.has_value() == section.has_value() &&
                    (!section.has_value() ||
                     (description.section->en == *section && !description.section->zh.empty())),
                "the rigid body property '" + std::string(name) + "' is not in its section");
    }
    for (const ElementKind kind : vehicle_project::kElementKinds) {
        if (kind == ElementKind::kRigidBody) {
            continue;
        }
        for (const PropertyDescription& description :
             vehicle_project_editing::DescribeElementProperties(kind)) {
            Require(!description.section.has_value(),
                    "a " + std::string(vehicle_project::DescribeElementKind(kind)) +
                        " property is in a section");
        }
    }
    for (const vehicle_project_editing::ProjectSettings settings :
         vehicle_project_editing::kProjectSettings) {
        for (const PropertyDescription& description :
             vehicle_project_editing::DescribeSettingsProperties(settings)) {
            Require(!description.section.has_value(), "a settings property is in a section");
        }
    }
}

void VerifyPropertyDescriptions() {
    const VehicleProject probe = MakePropertyProbeProject();
    const std::vector<ExpectedElementKind> kinds = ExpectedProbeProperties(probe);
    VerifyPropertiesReadTheirMembers(probe, kinds);
    VerifyAbsentValues(probe);
    VerifyReadingsStateWhatEditsAccept(probe, kinds);
    VerifyElementLookup(probe);
    VerifyPropertiesWriteTheirMembers(probe, kinds);
    VerifyStationOffsetWritesItsLayoutEntry(probe);
}

// --- Editing the bundled vehicles ------------------------------------------

const ElementReference kPrimarySuspension{ElementKind::kTranslationalSpringDamper,
                                          "front_leading_left_primary_suspension"};

std::size_t PrimarySuspensionIndex(const configuration::VehicleDefinition& vehicle) {
    const auto found =
        std::ranges::find(vehicle.translational_spring_dampers, kPrimarySuspension.name,
                          &configuration::VehicleTranslationalSpringDamperDefinition::name);
    Require(found != vehicle.translational_spring_dampers.end(),
            "GZ18 has no '" + kPrimarySuspension.name + "'");
    return static_cast<std::size_t>(found - vehicle.translational_spring_dampers.begin());
}

void VerifyPrimarySuspensionEdit(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProject(gz18));
    const configuration::VehicleDefinition original =
        vehicle_project::GenerateVehicleDefinition(session.project());
    const std::size_t suspension = PrimarySuspensionIndex(original);
    const Eigen::Vector3d original_stiffness =
        original.translational_spring_dampers[suspension].stiffness_newtons_per_meter;
    const Eigen::Vector3d edited_stiffness = original_stiffness + Eigen::Vector3d(0.0, 0.0, 2.0e5);

    const std::uint64_t opened_revision = session.revision();
    Require(
        session
            .SetSubjectProperty(kPrimarySuspension, "stiffness_newtons_per_meter", edited_stiffness)
            .has_value(),
        "the GZ18 primary suspension stiffness edit was refused");
    const std::uint64_t edited_revision = session.revision();
    const configuration::VehicleDefinition edited =
        vehicle_project::GenerateVehicleDefinition(session.project());
    configuration::VehicleDefinition edited_with_original_stiffness = edited;
    edited_with_original_stiffness.translational_spring_dampers[suspension]
        .stiffness_newtons_per_meter = original_stiffness;
    Require(edited.translational_spring_dampers[suspension].stiffness_newtons_per_meter ==
                    edited_stiffness &&
                Same(original, edited_with_original_stiffness),
            "the generated definition does not differ from the original in exactly the edited "
            "stiffness");
    Require(edited_revision > opened_revision && session.can_undo() && !session.can_redo(),
            "the edit did not start a new revision with one step to undo");

    session.Undo();
    const std::uint64_t undone_revision = session.revision();
    Require(Same(original, vehicle_project::GenerateVehicleDefinition(session.project())),
            "undoing the edit did not restore the original definition");
    Require(undone_revision > edited_revision && !session.can_undo() && session.can_redo(),
            "undoing did not start a new revision with one step to redo");

    session.Redo();
    const std::uint64_t redone_revision = session.revision();
    Require(Same(edited, vehicle_project::GenerateVehicleDefinition(session.project())),
            "redoing the edit did not re-apply it");
    Require(redone_revision > undone_revision && session.can_undo() && !session.can_redo(),
            "redoing did not start a new revision with one step to undo");

    // A new edit after an undo replaces what could have been redone.
    session.Undo();
    Require(session
                .SetSubjectProperty(kPrimarySuspension, "damping_newton_seconds_per_meter",
                                    Eigen::Vector3d(1.0e3, 2.0e3, 3.0e4))
                .has_value(),
            "the GZ18 primary suspension damping edit was refused");
    Require(session.revision() > redone_revision && session.can_undo() && !session.can_redo(),
            "an edit after an undo left a step to redo");
    bool redo_refused = false;
    try {
        session.Redo();
    } catch (const std::logic_error&) {
        redo_refused = true;
    }
    Require(redo_refused, "redoing with an empty redo history was not refused");
}

void VerifyEditRefusals(const ReferenceVehicle& gz18) {
    const std::string unplaced_body_name = "front_bogie_frame";
    VehicleProject project = PlacedProject(gz18);
    std::erase_if(project.free_body_placements, [&](const FreeBodyPlacement& placement) {
        return placement.body_name == unplaced_body_name;
    });
    VehicleProjectEditingSession session(project);
    // One accepted edit first, so that the history holds a step a refusal
    // could disturb.
    Require(session
                .SetSubjectProperty(kPrimarySuspension, "damping_newton_seconds_per_meter",
                                    Eigen::Vector3d(1.0e3, 2.0e3, 3.0e4))
                .has_value(),
            "the GZ18 primary suspension damping edit was refused");
    const VehicleProject edited = session.project();
    const std::uint64_t edited_revision = session.revision();

    struct RefusedEdit {
        std::string what;
        ElementReference element;
        std::string property_name;
        PropertyValue value;
    };
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    const std::vector<RefusedEdit> refused_edits = {
        {"an edit of an element the project does not contain",
         {ElementKind::kTranslationalSpringDamper, "absent_suspension"},
         "stiffness_newtons_per_meter",
         Eigen::Vector3d(0.0, 0.0, 1.0e6)},
        {"an edit of a property the kind does not have", kPrimarySuspension, "stiffness",
         Eigen::Vector3d(0.0, 0.0, 1.0e6)},
        {"an edit of a property that is not editable",
         {ElementKind::kFixedFrame, gz18.definition.fixed_frames.front().name},
         "rotation_in_body_frame",
         Eigen::Matrix3d::Identity().eval()},
        {"an edit with a value of the wrong kind", kPrimarySuspension,
         "stiffness_newtons_per_meter", 1.0e6},
        {"an edit with a number that is not finite",
         {ElementKind::kRigidBody, "carbody"},
         "mass_kilograms",
         not_a_number},
        {"an edit with a 3-vector component that is not finite", kPrimarySuspension,
         "stiffness_newtons_per_meter", Eigen::Vector3d(0.0, 0.0, infinity)},
        {"an edit with a curve point that is not finite",
         {ElementKind::kSaturatedPiecewiseLinearDamper,
          gz18.definition.saturated_piecewise_linear_dampers.front().name},
         "curve",
         SaturatedPiecewiseLinearDamperCurve{
             {.relative_velocity_meters_per_second = 0.0, .force_newtons = 0.0},
             {.relative_velocity_meters_per_second = not_a_number, .force_newtons = 1.0}}},
        {"an edit of a placement offset of a free body without a placement",
         {ElementKind::kRigidBody, unplaced_body_name},
         "placement.lateral_offset_in_local_track_frame_meters",
         0.01},
        {"an edit of the station offset of a body the layout does not place",
         {ElementKind::kRigidBody, "front_leading_left_axlebox_carrier"},
         "mechanical_track_station_layout.station_offset_meters",
         1.0},
        {"an edit of a placement offset of a body that cannot be placed",
         {ElementKind::kRigidBody, "front_leading_left_axlebox_carrier"},
         "placement.vertical_offset_in_local_track_frame_meters",
         -0.4},
    };
    for (const RefusedEdit& edit : refused_edits) {
        const auto outcome =
            session.SetSubjectProperty(edit.element, edit.property_name, edit.value);
        Require(!outcome.has_value(), edit.what + " was accepted");
        const SubjectEditRefusal& refusal = outcome.error();
        Require(refusal.subject == PropertySubject{edit.element} &&
                    refusal.property_name == edit.property_name && !refusal.reason.empty(),
                edit.what + " was refused without naming its element and property and saying why");
        Require(SameProject(edited, session.project()) && session.revision() == edited_revision &&
                    session.can_undo() && !session.can_redo(),
                edit.what + " changed the project, its history or its revision");
    }
    // The history still holds exactly the one accepted edit.
    session.Undo();
    Require(SameProject(project, session.project()) && !session.can_undo(),
            "after the refusals, undoing did not return to the opened project in one step");
}

// --- Checking ----------------------------------------------------------------

void VerifyReferenceVehicleChecks(const ReferenceVehicle& reference) {
    const std::string& vehicle_name = reference.definition.vehicle_name;
    VehicleProjectEditingSession session(PlacedProject(reference));
    const MechanicalCheckReport report = session.CheckMechanics();
    Require(report.refusals.empty(), vehicle_name + " did not pass the mechanical check: " +
                                         (report.refusals.empty() ? "" : report.refusals.front()));
    // Every free body is placed, so the absent contact record is all there is
    // to report.
    Require(report.information.size() == 1,
            vehicle_name + ": the check reports other information than the absent contact record");
    Require(
        !report.coverage_statements.empty() &&
            std::ranges::none_of(report.coverage_statements,
                                 [](const std::string& statement) { return statement.empty(); }),
        vehicle_name + ": the check does not say what it covered");
    Require(
        session.preview().has_value() && session.preview()->project_revision == session.revision(),
        vehicle_name + ": the passing check left no preview of the current revision");

    VehicleProject with_contact_record = PlacedProject(reference);
    with_contact_record.contact_record = reference.contact_record;
    VehicleProjectEditingSession contact_session(with_contact_record);
    const MechanicalCheckReport contact_report = contact_session.CheckMechanics();
    Require(contact_report.refusals.empty() && contact_report.information.empty(),
            vehicle_name +
                ": with its contact record and every body placed, the check still "
                "reports something");
}

void VerifyRefusedAssemblyKeepsOlderPreview(const ReferenceVehicle& gz18) {
    VehicleProjectEditingSession session(PlacedProject(gz18));
    Require(session.CheckMechanics().refusals.empty(), "GZ18 did not pass the mechanical check");
    const std::uint64_t checked_revision = session.revision();

    // The force plan requires a passive element's stiffness to be finite and
    // non-negative; the edit does not judge that.
    Require(session
                .SetSubjectProperty(kPrimarySuspension, "stiffness_newtons_per_meter",
                                    Eigen::Vector3d(0.0, 0.0, -1.4e6))
                .has_value(),
            "a negative stiffness was refused by the edit, which must leave plausibility to "
            "checking");
    std::string assembly_refusal;
    try {
        (void)configuration::AssembleVehicleSystem(
            vehicle_project::GenerateVehicleDefinition(session.project()),
            *session.project().gravitational_acceleration_meters_per_second_squared);
    } catch (const std::exception& error) {
        assembly_refusal = error.what();
    }
    Require(!assembly_refusal.empty(),
            "the mechanical assembly accepted a negative primary suspension stiffness");

    const MechanicalCheckReport report = session.CheckMechanics();
    Require(report.refusals == std::vector<std::string>{assembly_refusal},
            "the refused assembly is not reported with exactly its own message");
    Require(session.preview().has_value() &&
                session.preview()->project_revision == checked_revision &&
                checked_revision < session.revision(),
            "after the refused check, the earlier preview is gone or not marked as older");
}

void VerifyContactDeclarationRefusalKeepsOlderPreview(const ReferenceVehicle& irw) {
    VehicleProject project = PlacedProject(irw);
    project.contact_record = irw.contact_record;
    VehicleProjectEditingSession session(std::move(project));
    Require(session.CheckMechanics().refusals.empty(),
            "IRW did not pass the check with its contact declaration");
    const std::uint64_t checked_revision = session.revision();

    const auto& interface = irw.contact_record.declaration.interfaces.front();
    const std::string& joint_name = *interface.independent_wheel_revolute_joint_name;
    const auto joint = std::ranges::find(irw.definition.revolute_joints, joint_name,
                                         &configuration::VehicleRevoluteJointDefinition::name);
    Require(joint != irw.definition.revolute_joints.end(),
            "the reference contact interface has no revolute joint");
    const Eigen::Vector3d reversed_axis = -joint->axis_in_parent_frame;
    Require(session
                .SetSubjectProperty(ElementReference{ElementKind::kRevoluteJoint, joint_name},
                                    "axis_in_parent_frame", reversed_axis)
                .has_value(),
            "a wheel-axis edit was refused before checking its contact declaration");

    const MechanicalCheckReport report = session.CheckMechanics();
    Require(std::ranges::any_of(report.refusals, [&](const std::string& refusal) {
                return refusal.find(interface.interface_name) != std::string::npos;
            }),
            "the contact check did not identify the interface with an inconsistent wheel axis");
    Require(session.preview().has_value() &&
                session.preview()->project_revision == checked_revision &&
                checked_revision < session.revision(),
            "after a refused contact check, the earlier preview is gone or not marked as older");

    session.Undo();
    Require(session.CheckMechanics().refusals.empty() && session.preview().has_value() &&
                session.preview()->project_revision == session.revision(),
            "restoring the wheel axis did not restore a passing contact check and current preview");
}

// What a vehicle file's declaration rules refuse, and a project without a
// gravity, are refused by the check before any assembly: the refusal names
// the place, the check does not throw and leaves no preview, and the
// assembly is not attempted, so a negative stiffness it would refuse as well
// goes unreported. A force element ending at `world` is one such project: the
// assembly accepts it, and the preview could not draw it.
void VerifyCheckRefusesBeforeAssembly(const ReferenceVehicle& gz18) {
    const auto require_refused_before_assembly = [](VehicleProject project, std::string_view place,
                                                    const std::string& what) {
        project.vehicle_definition.translational_spring_dampers.at(1).stiffness_newtons_per_meter =
            Eigen::Vector3d(0.0, 0.0, -1.4e6);
        VehicleProjectEditingSession session(std::move(project));
        const MechanicalCheckReport report = session.CheckMechanics();
        Require(
            report.refusals.size() == 1 && report.refusals.front().find(place) != std::string::npos,
            what + " is not the check's one refusal, naming " + std::string(place));
        Require(!session.preview().has_value(), what + " was checked into a preview");
    };

    VehicleProject ending_at_world = PlacedProject(gz18);
    ending_at_world.vehicle_definition.translational_spring_dampers.at(0).opposite_frame_name =
        "world";
    require_refused_before_assembly(
        ending_at_world, "$.vehicle_definition.translational_spring_dampers[0].opposite_frame_name",
        "a force element ending at world");

    VehicleProject spaced_identifier = PlacedProject(gz18);
    spaced_identifier.vehicle_definition.mechanical_definition_identifier = "gz18 edited";
    require_refused_before_assembly(spaced_identifier,
                                    "$.vehicle_definition.mechanical_definition_identifier",
                                    "a mechanical definition identifier with a space");

    VehicleProject without_gravity = PlacedProject(gz18);
    without_gravity.gravitational_acceleration_meters_per_second_squared.reset();
    require_refused_before_assembly(without_gravity, "gravitational acceleration",
                                    "a project stating no gravity");
}

// --- Preview -------------------------------------------------------------------

// Every free body moved sideways and down by an amount of its own and turned
// about all three axes, so that no placement is left-right symmetric and no
// two bodies are placed alike.
VehicleProject AsymmetricallyPlacedProject(const ReferenceVehicle& reference) {
    VehicleProject project = PlacedProject(reference);
    double step = 0.0;
    for (FreeBodyPlacement& placement : project.free_body_placements) {
        step += 1.0;
        placement.lateral_offset_in_local_track_frame_meters += 0.013 * step;
        placement.vertical_offset_in_local_track_frame_meters -= 0.007 * step;
        placement.rotation_local_track_from_body =
            Eigen::AngleAxisd(0.004 * step, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(-0.003 * step, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.005 * step, Eigen::Vector3d::UnitX()) *
            placement.rotation_local_track_from_body;
    }
    return project;
}

double GeodesicAngle(const Eigen::Matrix3d& first, const Eigen::Matrix3d& second) {
    return Eigen::AngleAxisd(first.transpose() * second).angle();
}

const RigidBodyDrawing& DrawnBody(const VehicleProjectPreview& preview,
                                  const std::string& body_name) {
    const auto drawn =
        std::ranges::find(preview.rigid_bodies, body_name, &RigidBodyDrawing::body_name);
    Require(drawn != preview.rigid_bodies.end(), "body '" + body_name + "' is not drawn");
    return *drawn;
}

// A frame as the vehicle definition states it: the body carrying it and its
// pose in that body. A body's own name names the body's frame.
struct FrameOnBody {
    std::string body_name;
    Eigen::Vector3d position_in_body_frame_meters;
    Eigen::Matrix3d rotation_body_from_frame;
};

FrameOnBody LocateFrame(const configuration::VehicleDefinition& vehicle,
                        const std::string& frame_name) {
    const auto fixed_frame = std::ranges::find(vehicle.fixed_frames, frame_name,
                                               &configuration::VehicleFixedFrameDefinition::name);
    if (fixed_frame == vehicle.fixed_frames.end()) {
        return {frame_name, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()};
    }
    return {fixed_frame->body_name, fixed_frame->position_in_body_frame_meters,
            fixed_frame->rotation_in_body_frame};
}

// Where a frame's origin is when its body's drawn pose carries it.
Eigen::Vector3d FrameOriginFromBodyPose(const VehicleProjectPreview& preview,
                                        const configuration::VehicleDefinition& vehicle,
                                        const std::string& frame_name) {
    const FrameOnBody frame = LocateFrame(vehicle, frame_name);
    const RigidBodyDrawing& body = DrawnBody(preview, frame.body_name);
    return body.origin_in_drawing_frame_meters +
           body.rotation_drawing_from_body * frame.position_in_body_frame_meters;
}

Eigen::Matrix3d FrameRotationFromBodyPose(const VehicleProjectPreview& preview,
                                          const configuration::VehicleDefinition& vehicle,
                                          const std::string& frame_name) {
    const FrameOnBody frame = LocateFrame(vehicle, frame_name);
    return DrawnBody(preview, frame.body_name).rotation_drawing_from_body *
           frame.rotation_body_from_frame;
}

void RequireAtFrameOrigin(const Eigen::Vector3d& drawn, const VehicleProjectPreview& preview,
                          const configuration::VehicleDefinition& vehicle,
                          const std::string& frame_name, const std::string& what) {
    const double distance = (drawn - FrameOriginFromBodyPose(preview, vehicle, frame_name)).norm();
    Require(distance <= kCompositionToleranceMeters, what + " lies " + Scientific(distance) +
                                                         " m from where the pose of the body of '" +
                                                         frame_name + "' puts that frame's origin");
}

// The elements of a project of the kinds that have drawings, by kind and name:
// every element but the contact carriers.
std::multiset<std::pair<ElementKind, std::string>> ElementsOfDrawnKinds(
    const VehicleProject& project) {
    const configuration::VehicleDefinition& vehicle = project.vehicle_definition;
    std::multiset<std::pair<ElementKind, std::string>> elements;
    const auto add = [&elements](ElementKind kind, const auto& records) {
        for (const auto& record : records) {
            elements.emplace(kind, record.name);
        }
    };
    add(ElementKind::kRigidBody, vehicle.rigid_bodies);
    add(ElementKind::kFixedFrame, vehicle.fixed_frames);
    add(ElementKind::kRevoluteJoint, vehicle.revolute_joints);
    add(ElementKind::kBallRpyJoint, vehicle.ball_rpy_joints);
    add(ElementKind::kWeldJoint, vehicle.weld_joints);
    add(ElementKind::kTranslationalSpringDamper, vehicle.translational_spring_dampers);
    add(ElementKind::kRollSpringDamperCouple, vehicle.roll_spring_damper_couples);
    add(ElementKind::kSeriesSpringViscousDamper, vehicle.series_spring_viscous_dampers);
    add(ElementKind::kPointToPointSeriesSpringBilinearDamper,
        vehicle.point_to_point_series_spring_bilinear_dampers);
    add(ElementKind::kSaturatedPiecewiseLinearDamper, vehicle.saturated_piecewise_linear_dampers);
    add(ElementKind::kReferenceFrameLinearBushing, vehicle.reference_frame_linear_bushings);
    add(ElementKind::kPointToPointLinearSpringDamper, vehicle.point_to_point_linear_spring_dampers);
    add(ElementKind::kPointToPointHystereticSpring, vehicle.point_to_point_hysteretic_springs);
    if (project.contact_record.has_value()) {
        for (const configuration::VehicleContactInterfaceDeclaration& interface :
             project.contact_record->declaration.interfaces) {
            elements.emplace(ElementKind::kContactInterface, interface.interface_name);
        }
    }
    return elements;
}

// Every element of a kind that has drawings is either drawn once or reported
// as not drawable once.
void RequireEveryElementAccountedFor(const VehicleProjectPreview& preview,
                                     const VehicleProject& project) {
    std::multiset<std::pair<ElementKind, std::string>> accounted;
    for (const auto& body : preview.rigid_bodies) {
        accounted.emplace(ElementKind::kRigidBody, body.body_name);
    }
    for (const auto& frame : preview.fixed_frames) {
        accounted.emplace(ElementKind::kFixedFrame, frame.frame_name);
    }
    for (const auto& joint : preview.revolute_joints) {
        accounted.emplace(ElementKind::kRevoluteJoint, joint.joint_name);
    }
    for (const auto& joint : preview.ball_rpy_joints) {
        accounted.emplace(ElementKind::kBallRpyJoint, joint.joint_name);
    }
    for (const auto& joint : preview.weld_joints) {
        accounted.emplace(ElementKind::kWeldJoint, joint.joint_name);
    }
    for (const auto& element : preview.force_elements) {
        accounted.emplace(element.element.kind, element.element.name);
    }
    for (const WheelDrawing& wheel : preview.wheels) {
        accounted.emplace(ElementKind::kContactInterface, wheel.interface_name);
    }
    for (const ElementReference& element : preview.not_drawable_elements) {
        accounted.emplace(element.kind, element.name);
    }
    Require(accounted == ElementsOfDrawnKinds(project),
            project.vehicle_definition.vehicle_name +
                ": the preview does not draw or report every element exactly once");
}

// Everything the preview draws on a body is where the body's drawn pose
// carries it: fixed frames, both ends of every force element, and the origins
// each joint keeps together. A revolute joint's axis is its parent-frame axis.
void RequireDrawnWhereTheirBodiesCarryThem(const VehicleProjectPreview& preview,
                                           const configuration::VehicleDefinition& vehicle) {
    const std::string& vehicle_name = vehicle.vehicle_name;
    for (const auto& frame : preview.fixed_frames) {
        const std::string what = vehicle_name + ": frame '" + frame.frame_name + "'";
        RequireAtFrameOrigin(frame.origin_in_drawing_frame_meters, preview, vehicle,
                             frame.frame_name, what);
        Require(GeodesicAngle(frame.rotation_drawing_from_frame,
                              FrameRotationFromBodyPose(preview, vehicle, frame.frame_name)) <=
                    kCompositionToleranceRadians,
                what + " is not turned as its body's pose turns it");
    }

    const auto require_force_element_ends = [&](ElementKind kind, const auto& elements) {
        for (const auto& element : elements) {
            const auto drawing = std::ranges::find_if(
                preview.force_elements,
                [&](const vehicle_project_editing::ForceElementDrawing& candidate) {
                    return candidate.element.kind == kind && candidate.element.name == element.name;
                });
            if (drawing == preview.force_elements.end()) {
                continue;  // Reported as not drawable; accounted for elsewhere.
            }
            const std::string what = vehicle_name + ": force element '" + element.name + "'";
            RequireAtFrameOrigin(drawing->reference_end_in_drawing_frame_meters, preview, vehicle,
                                 element.reference_frame_name, what + " reference end");
            RequireAtFrameOrigin(drawing->opposite_end_in_drawing_frame_meters, preview, vehicle,
                                 element.opposite_frame_name, what + " opposite end");
        }
    };
    require_force_element_ends(ElementKind::kTranslationalSpringDamper,
                               vehicle.translational_spring_dampers);
    require_force_element_ends(ElementKind::kRollSpringDamperCouple,
                               vehicle.roll_spring_damper_couples);
    require_force_element_ends(ElementKind::kSeriesSpringViscousDamper,
                               vehicle.series_spring_viscous_dampers);
    require_force_element_ends(ElementKind::kPointToPointSeriesSpringBilinearDamper,
                               vehicle.point_to_point_series_spring_bilinear_dampers);
    require_force_element_ends(ElementKind::kSaturatedPiecewiseLinearDamper,
                               vehicle.saturated_piecewise_linear_dampers);
    require_force_element_ends(ElementKind::kReferenceFrameLinearBushing,
                               vehicle.reference_frame_linear_bushings);
    require_force_element_ends(ElementKind::kPointToPointLinearSpringDamper,
                               vehicle.point_to_point_linear_spring_dampers);
    require_force_element_ends(ElementKind::kPointToPointHystereticSpring,
                               vehicle.point_to_point_hysteretic_springs);

    // A joint is drawn at its parent frame's origin; its child frame's origin,
    // carried by the other body, must coincide with it.
    const auto require_joint_location = [&](const auto& joints, const std::string& joint_name,
                                            const Eigen::Vector3d& location) {
        RequireAtFrameOrigin(location, preview, vehicle, Named(joints, joint_name).child_frame_name,
                             vehicle_name + ": joint '" + joint_name + "'");
    };
    for (const auto& drawing : preview.revolute_joints) {
        require_joint_location(vehicle.revolute_joints, drawing.joint_name,
                               drawing.location_in_drawing_frame_meters);
        const configuration::VehicleRevoluteJointDefinition& joint =
            Named(vehicle.revolute_joints, drawing.joint_name);
        const Eigen::Vector3d axis_in_parent_frame =
            FrameRotationFromBodyPose(preview, vehicle, joint.parent_frame_name).transpose() *
            drawing.axis_in_drawing_frame;
        Require((axis_in_parent_frame - joint.axis_in_parent_frame.normalized()).norm() <=
                    kCompositionToleranceRadians,
                vehicle_name + ": revolute joint '" + drawing.joint_name +
                    "' is not drawn about its unit axis in its parent frame");
    }
    for (const auto& drawing : preview.ball_rpy_joints) {
        require_joint_location(vehicle.ball_rpy_joints, drawing.joint_name,
                               drawing.location_in_drawing_frame_meters);
    }
    for (const auto& drawing : preview.weld_joints) {
        require_joint_location(vehicle.weld_joints, drawing.joint_name,
                               drawing.location_in_drawing_frame_meters);
    }
}

// The carrier `interface` names in `contact_record`.
const configuration::VehicleContactCarrierDeclaration& CarrierOf(
    const configuration::VehicleContactRecord& contact_record,
    const configuration::VehicleContactInterfaceDeclaration& interface) {
    const auto carrier =
        std::ranges::find(contact_record.declaration.carriers, interface.carrier_name,
                          &configuration::VehicleContactCarrierDeclaration::carrier_name);
    Require(carrier != contact_record.declaration.carriers.end(),
            "contact interface '" + interface.interface_name + "' names no declared carrier");
    return *carrier;
}

// Each drawn wheel is the wheel of a contact interface of the project's
// contact record: it turns with the interface's wheel body, carries the
// record's nominal rolling radius, has a unit outward spin axis, and its datum
// lies the record's lateral datum magnitude along that axis from the drawn
// origin of its carrier's body.
void RequireWheelsOnTheirCarriers(const VehicleProjectPreview& preview,
                                  const VehicleProject& project) {
    const std::string& vehicle_name = project.vehicle_definition.vehicle_name;
    const configuration::VehicleContactRecord& contact_record = *project.contact_record;
    for (const WheelDrawing& wheel : preview.wheels) {
        const std::string what = vehicle_name + ": the wheel of '" + wheel.interface_name + "'";
        const auto interface =
            std::ranges::find(contact_record.declaration.interfaces, wheel.interface_name,
                              &configuration::VehicleContactInterfaceDeclaration::interface_name);
        Require(interface != contact_record.declaration.interfaces.end(),
                what + " belongs to no contact interface");
        Require(wheel.wheel_body_name == interface->wheel_body_name,
                what + " does not turn with the interface's wheel body");
        Require(wheel.nominal_rolling_radius_meters ==
                    contact_record.wheel.nominal_rolling_radius_meters,
                what + " does not carry the record's nominal rolling radius");
        const Eigen::Vector3d& outward_axis = wheel.outward_spin_axis_in_drawing_frame;
        Require(std::abs(outward_axis.norm() - 1.0) <= kCompositionToleranceRadians,
                what + " has an outward spin axis that is not a unit vector");
        const Eigen::Vector3d& carrier_origin =
            DrawnBody(preview, CarrierOf(contact_record, *interface).body_name)
                .origin_in_drawing_frame_meters;
        const double distance =
            (wheel.datum_in_drawing_frame_meters -
             (carrier_origin +
              contact_record.wheel.wheel_lateral_datum_magnitude_meters * outward_axis))
                .norm();
        Require(distance <= kCompositionToleranceMeters,
                what + " lies " + Scientific(distance) +
                    " m from the lateral datum magnitude along its outward axis from its "
                    "carrier's origin");
    }
}

void VerifyPreviewPlacesWhatTheModelPlaces(const ReferenceVehicle& reference) {
    const configuration::VehicleDefinition& vehicle = reference.definition;
    const std::string& vehicle_name = vehicle.vehicle_name;
    VehicleProject project = AsymmetricallyPlacedProject(reference);
    project.contact_record = reference.contact_record;
    VehicleProjectEditingSession session(project);
    Require(session.CheckMechanics().refusals.empty(),
            vehicle_name + " with asymmetric placements did not pass the check");
    const VehicleProjectPreview& preview = *session.preview();
    Require(preview.not_drawable_elements.empty(),
            vehicle_name + ": with every free body placed, something is not drawable");
    RequireEveryElementAccountedFor(preview, project);

    // A placed free body sits at its mechanical station offset, its
    // placement's lateral and vertical offsets, turned by its placement's
    // rotation.
    const auto& station_offsets = vehicle.mechanical_track_station_layout.free_body_station_offsets;
    for (const FreeBodyPlacement& placement : project.free_body_placements) {
        const auto station_offset =
            std::ranges::find(station_offsets, placement.body_name,
                              &configuration::VehicleFreeBodyStationOffsetDefinition::body_name);
        Require(station_offset != station_offsets.end(),
                vehicle_name + ": the layout gives '" + placement.body_name + "' no station");
        const RigidBodyDrawing& body = DrawnBody(preview, placement.body_name);
        const Eigen::Vector3d placed_origin(station_offset->station_offset_meters,
                                            placement.lateral_offset_in_local_track_frame_meters,
                                            placement.vertical_offset_in_local_track_frame_meters);
        Require((body.origin_in_drawing_frame_meters - placed_origin).norm() <=
                        kCompositionToleranceMeters &&
                    GeodesicAngle(body.rotation_drawing_from_body,
                                  placement.rotation_local_track_from_body.toRotationMatrix()) <=
                        kCompositionToleranceRadians,
                vehicle_name + ": free body '" + placement.body_name +
                    "' is not drawn where its placement puts it");
    }
    RequireDrawnWhereTheirBodiesCarryThem(preview, vehicle);
    RequireWheelsOnTheirCarriers(preview, project);
}

// In the reference start-up placements every body stands nearly upright, so
// each wheel's outward spin axis points to its own side of the track: to the
// drawing frame's +y, the right, for a right wheel and to -y for a left one.
// That ties the wheel's side to the drawing frame through both body-frame
// conventions: GZ18's bodies have +y to the right and +z down, IRW's +y to the
// left and +z up. The wheels follow the contact declaration's interface order.
void VerifyWheelsPointToTheirSides(const ReferenceVehicle& reference) {
    const std::string& vehicle_name = reference.definition.vehicle_name;
    VehicleProject project = PlacedProject(reference);
    project.contact_record = reference.contact_record;
    VehicleProjectEditingSession session(std::move(project));
    Require(session.CheckMechanics().refusals.empty(),
            vehicle_name + " did not pass the check with its contact record");
    const std::vector<WheelDrawing>& wheels = session.preview()->wheels;
    const auto& interfaces = reference.contact_record.declaration.interfaces;
    Require(wheels.size() == interfaces.size(),
            vehicle_name + ": with every free body placed, not every wheel is drawn");
    for (std::size_t index = 0; index < interfaces.size(); ++index) {
        const configuration::VehicleContactInterfaceDeclaration& interface = interfaces[index];
        Require(wheels[index].interface_name == interface.interface_name,
                vehicle_name + ": the wheels do not follow the contact declaration's order");
        const Eigen::Vector3d& outward_axis = wheels[index].outward_spin_axis_in_drawing_frame;
        const double toward_its_side = interface.side == wheel_rail_contact::WheelSide::kRight
                                           ? outward_axis.y()
                                           : -outward_axis.y();
        Require(toward_its_side > std::max(std::abs(outward_axis.x()), std::abs(outward_axis.z())),
                vehicle_name + ": the outward spin axis of '" + interface.interface_name +
                    "' does not point to its side of the track");
    }
}

// Editing a free body's station offset moves its drawing along the track by
// the same amount, and only along it.
void VerifyStationOffsetEditMovesBodyAlongTrack(const ReferenceVehicle& gz18) {
    constexpr double kShiftMeters = 0.25;
    const ElementReference bogie_frame{ElementKind::kRigidBody, "front_bogie_frame"};
    VehicleProjectEditingSession session(PlacedProject(gz18));
    Require(session.CheckMechanics().refusals.empty(), "GZ18 did not pass the mechanical check");
    const Eigen::Vector3d origin_before =
        DrawnBody(*session.preview(), bogie_frame.name).origin_in_drawing_frame_meters;
    const auto& station_offsets =
        gz18.definition.mechanical_track_station_layout.free_body_station_offsets;
    const auto entry =
        std::ranges::find(station_offsets, bogie_frame.name,
                          &configuration::VehicleFreeBodyStationOffsetDefinition::body_name);
    Require(entry != station_offsets.end(), "GZ18's layout gives its front bogie frame no station");
    Require(session
                .SetSubjectProperty(bogie_frame,
                                    "mechanical_track_station_layout.station_offset_meters",
                                    entry->station_offset_meters + kShiftMeters)
                .has_value(),
            "the GZ18 front bogie frame's station offset edit was refused");
    Require(session.CheckMechanics().refusals.empty(),
            "GZ18 with a moved front bogie frame did not pass the mechanical check");
    const Eigen::Vector3d origin_after =
        DrawnBody(*session.preview(), bogie_frame.name).origin_in_drawing_frame_meters;
    Require((origin_after - origin_before - Eigen::Vector3d(kShiftMeters, 0.0, 0.0)).norm() <=
                kCompositionToleranceMeters,
            "editing the GZ18 front bogie frame's station offset did not move its drawing along "
            "the track by the same amount");
}

// Each IRW tie rod hangs from its axle bridge by a Ball-RPY joint whose
// default angles state R_FM = Rz(yaw) Ry(pitch) Rx(roll) between the joint's
// parent frame F and child frame M. The drawn tie rod must hold them; at zero
// angles it would be turned by more than a radian.
void VerifyTieRodsHoldTheirDefaultAngles(const ReferenceVehicle& irw) {
    const configuration::VehicleDefinition& vehicle = irw.definition;
    VehicleProjectEditingSession session(AsymmetricallyPlacedProject(irw));
    Require(session.CheckMechanics().refusals.empty(),
            "IRW with asymmetric placements did not pass the mechanical check");
    const VehicleProjectPreview& preview = *session.preview();
    Require(!vehicle.ball_rpy_joints.empty(), "IRW has no Ball-RPY joints");
    for (const configuration::VehicleBallRpyJointDefinition& joint : vehicle.ball_rpy_joints) {
        const Eigen::Vector3d& angles = joint.default_roll_pitch_yaw_angles_radians;
        const Eigen::Matrix3d rotation_parent_from_child_at_default_angles =
            (Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        const Eigen::Matrix3d drawn_rotation_parent_from_child =
            FrameRotationFromBodyPose(preview, vehicle, joint.parent_frame_name).transpose() *
            FrameRotationFromBodyPose(preview, vehicle, joint.child_frame_name);
        Require(GeodesicAngle(drawn_rotation_parent_from_child,
                              rotation_parent_from_child_at_default_angles) <=
                    kCompositionToleranceRadians,
                "IRW tie rod joint '" + joint.name + "' is not drawn at its default angles");
        Require(GeodesicAngle(drawn_rotation_parent_from_child, Eigen::Matrix3d::Identity()) > 1.0,
                "IRW tie rod joint '" + joint.name + "' is drawn as if at zero angles");
    }
}

// Without its placement, IRW's leading front axle bridge has no pose, and
// neither have the two wheels and two tie rods jointed to it; the wheels of
// the contact carrier it bears are not drawn.
void VerifyUnplacedBodiesAreNotDrawn(const ReferenceVehicle& irw) {
    const configuration::VehicleDefinition& vehicle = irw.definition;
    VehicleProject project = AsymmetricallyPlacedProject(irw);
    project.contact_record = irw.contact_record;
    std::erase_if(project.free_body_placements, [](const FreeBodyPlacement& placement) {
        return placement.body_name == "axlebridge_ff";
    });
    VehicleProjectEditingSession session(project);
    const MechanicalCheckReport report = session.CheckMechanics();
    Require(report.refusals.empty() && report.information.size() == 1,
            "IRW with one unplaced free body did not pass the check reporting that body alone");
    const VehicleProjectPreview& preview = *session.preview();

    std::set<std::string> not_drawable_bodies;
    std::set<std::string> not_drawable_interfaces;
    for (const ElementReference& element : preview.not_drawable_elements) {
        if (element.kind == ElementKind::kRigidBody) {
            not_drawable_bodies.insert(element.name);
        }
        if (element.kind == ElementKind::kContactInterface) {
            not_drawable_interfaces.insert(element.name);
        }
    }
    Require(
        not_drawable_bodies == std::set<std::string>{"axlebridge_ff", "wheel_ff_l", "wheel_ff_r",
                                                     "longibar_front_A", "longibar_front_B"},
        "IRW without the leading front axle bridge's placement reports other bodies than it "
        "and the bodies jointed to it as not drawable");
    std::set<std::string> interfaces_of_carriers_without_pose;
    for (const configuration::VehicleContactInterfaceDeclaration& interface :
         irw.contact_record.declaration.interfaces) {
        if (not_drawable_bodies.contains(CarrierOf(irw.contact_record, interface).body_name)) {
            interfaces_of_carriers_without_pose.insert(interface.interface_name);
        }
    }
    Require(!interfaces_of_carriers_without_pose.empty() &&
                not_drawable_interfaces == interfaces_of_carriers_without_pose,
            "IRW without the leading front axle bridge's placement does not report exactly the "
            "wheels of the carrier it bears as not drawable");
    RequireEveryElementAccountedFor(preview, project);
    RequireDrawnWhereTheirBodiesCarryThem(preview, vehicle);
    RequireWheelsOnTheirCarriers(preview, project);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_editing GZ18_VEHICLE GZ18_STARTUP_STATE "
                     "GZ18_CONTACT IRW_VEHICLE IRW_STARTUP_STATE IRW_CONTACT\n");
        return 2;
    }
    try {
        const ReferenceVehicle gz18 =
            editing_verification::LoadReferenceVehicle(argv[1], argv[2], argv[3]);
        const ReferenceVehicle irw =
            editing_verification::LoadReferenceVehicle(argv[4], argv[5], argv[6]);

        VerifyPropertyDescriptions();
        VerifyPropertySections();
        VerifyPrimarySuspensionEdit(gz18);
        VerifyEditRefusals(gz18);
        VerifyReferenceVehicleChecks(gz18);
        VerifyReferenceVehicleChecks(irw);
        VerifyRefusedAssemblyKeepsOlderPreview(gz18);
        VerifyContactDeclarationRefusalKeepsOlderPreview(irw);
        VerifyCheckRefusesBeforeAssembly(gz18);
        VerifyPreviewPlacesWhatTheModelPlaces(gz18);
        VerifyPreviewPlacesWhatTheModelPlaces(irw);
        VerifyWheelsPointToTheirSides(gz18);
        VerifyWheelsPointToTheirSides(irw);
        VerifyStationOffsetEditMovesBodyAlongTrack(gz18);
        VerifyTieRodsHoldTheirDefaultAngles(irw);
        VerifyUnplacedBodiesAreNotDrawn(irw);
        std::puts(
            "vehicle project editing: property descriptions, edits, refusals, history, checks and "
            "preview passed for the property probe, GZ18 and IRW");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project editing: %s\n", error.what());
        return 1;
    }
}
