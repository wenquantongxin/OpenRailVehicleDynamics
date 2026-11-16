// orvd_vehicle_modeling_service: the resident command layer a graphical
// modelling tool drives over one vehicle project.
//
// The service is deliberately thin. It parses a request, calls
// orvd::vehicle_project_editing, and writes the answer; all JSON lives here.
// It reads a vehicle and never writes a project.
//
// PROTOCOL
//
// Newline-delimited JSON over standard input and standard output. Each request
// is one line holding one JSON object. Every request line, well formed or not,
// receives exactly one response line, in request order, written and flushed
// before the next line is read. Nothing else is written to standard output;
// text for people goes to standard error. The service takes no arguments, and
// the end of its input ends it with exit code 0.
//
// Both ends are our own processes, so a request is read leniently: fields may
// come in any order, fields a command does not use are ignored, and a field
// given twice takes its last value. Only what a command uses is checked.
//
// Request
//   {"id": <integer>, "command": "<command>", <arguments>...}
//   The id is echoed exactly as given and is otherwise unused.
//
// Response to an accepted command
//   {"id": <id>, "ok": true, "revision": <integer>, "can_undo": <bool>,
//    "can_redo": <bool>, "result": {...}}
//   revision, can_undo and can_redo describe the session after the command.
//   Opening a vehicle starts at revision 0, every accepted edit, undo and redo
//   adds one, nothing else changes it, and it never decreases: undoing
//   restores an earlier project, not its revision.
//
// Response to a refused command
//   {"id": <id>, "ok": false,
//    "error": {"element": <element> or null, "property": "<name>" or null,
//              "message": "<text>"}}
//   A refused command changes nothing. element and property name what the
//   refusal concerns when it concerns them. id is null when the line holds no
//   JSON object with an id.
//
// Values
//   element       {"kind": "<element kind>", "name": "<element name>"}
//   element kind  one of, in this order: rigid_body, fixed_frame,
//                 revolute_joint, ball_rpy_joint, weld_joint,
//                 translational_spring_damper, roll_spring_damper_couple,
//                 series_spring_viscous_damper,
//                 point_to_point_series_spring_bilinear_damper,
//                 saturated_piecewise_linear_damper,
//                 reference_frame_linear_bushing,
//                 point_to_point_linear_spring_damper,
//                 point_to_point_hysteretic_spring, contact_carrier,
//                 contact_interface
//   vector        [x, y, z]
//   rotation      [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]], the rows
//                 of a rotation matrix; rotation_a_from_b takes b components to
//                 a components
//   property value, by value kind:
//     number        a JSON number
//     vector3       a vector
//     text          a string: a name, a reference to an element by name, or
//                   an axis token
//     boolean       true or false
//     rotation      a rotation
//     wheel_side    "left" or "right"
//     saturated_piecewise_linear_damper_curve
//                   [{"relative_velocity_meters_per_second": <number>,
//                     "force_newtons": <number>}, ...]
//     or null where the element holds no value: the station offset of a rigid
//     body the mechanical track station layout does not place, the placement
//     offsets of a rigid body without a placement, and the
//     independent_wheel_revolute_joint_name of a contact interface whose
//     wheel is not separately jointed.
//   Property names are the record members' own names, which state meaning and
//   unit. A rigid body adds where it is along, across and above the track:
//   mechanical_track_station_layout.station_offset_meters,
//   placement.lateral_offset_in_local_track_frame_meters and
//   placement.vertical_offset_in_local_track_frame_meters.
//
// Commands
//
//   open_vehicle  {"vehicle_definition_path": "<path>",
//                  "startup_state_path": "<path>",
//                  "contact_record_path": "<path>"}
//     Loads the three records with the strict loaders. The start-up record
//     supplies drawing placements and gravity; contact declaration problems
//     are reported by check. Opens a new session: revision 0, empty history,
//     no preview. A refused open keeps the session that was open. Every other
//     command is refused while no vehicle is open.
//     result: {}
//
//   describe_element_kinds
//     result: {"element_kinds": [{"kind": <element kind>,
//              "properties": [{"name": "<name>", "value_kind": "<value kind>",
//                              "editable": <bool>}, ...]}, ...]}
//     Every element kind, in the order above, with its properties in the order
//     get_properties returns them. editable says whether the kind's property
//     can be edited at all; every other property is read-only. The numbers and
//     3-vectors of the mechanical element records are editable, and so are a
//     rigid body's station offset and placement offsets.
//
//   get_outline
//     result: {"vehicle_name": "<name>", "elements": [<element>, ...],
//              "element_groups": [<group>, ...],
//              "unplaced_free_bodies": ["<body name>", ...],
//              "contact_record_present": <bool>}
//     group: {"name": "<name>", "members": [<element>, ...],
//             "subgroups": [<group>, ...]}
//     elements lists every element, kind by kind in the order above, each kind
//     in declaration order.
//
//   get_properties  {"element": <element>}
//     result: {"properties": [{"name": "<name>", "value_kind": "<value kind>",
//                              "editable": <bool>,
//                              "value": <property value>}, ...]}
//     editable says whether this element's property can be edited now. It is
//     the kind's editable, except that a property whose value is null for this
//     element is not editable: the station offset of a body the mechanical
//     track station layout does not place, and the placement offsets of a
//     body without a placement. set_property refuses an edit of a property
//     that is not editable now.
//
//   set_property  {"element": <element>, "property": "<name>",
//                  "value": <property value>}
//     result: {}
//     The value is read by its shape: a number as a number, an array of three
//     numbers as a vector3, a string as text, true or false as a boolean; any
//     other value is refused. The edit is then refused unless the element
//     exists, its kind has the property, the property is editable, the value
//     is of the property's value kind, every number in it is finite and, for a
//     station offset or a placement offset, the record holding it exists: the
//     body's entry in the mechanical track station layout, or its placement.
//     Physical plausibility and the layout's own consistency, such as its
//     reference body keeping a zero station offset, are not judged here; check
//     judges them.
//
//   undo, redo
//     result: {}
//     Refused when there is nothing to undo or to redo.
//
//   check
//     result: {"passed": <bool>, "refusals": ["<message>", ...],
//              "information": ["<message>", ...],
//              "coverage_statements": ["<sentence>", ...]}
//     A mechanical and contact-declaration check of the current project. A
//     faulty project is reported in refusals, never by refusing the command.
//     coverage_statements says, in words a user can be shown, what the check
//     covered and what passing means. A check that passes replaces the preview
//     with one of the current revision.
//
//   get_preview
//     result: {"preview": null} before any check has passed, otherwise
//     {"preview": {
//       "project_revision": <integer>,
//       "rigid_bodies": [{"element": <element>,
//                         "rotation_drawing_from_body": <rotation>,
//                         "origin_in_drawing_frame_meters": <vector>}, ...],
//       "fixed_frames": [{"element": <element>,
//                         "rotation_drawing_from_frame": <rotation>,
//                         "origin_in_drawing_frame_meters": <vector>}, ...],
//       "revolute_joints": [{"element": <element>,
//                            "location_in_drawing_frame_meters": <vector>,
//                            "axis_in_drawing_frame": <vector>}, ...],
//       "ball_rpy_joints": [{"element": <element>,
//                            "location_in_drawing_frame_meters": <vector>}, ...],
//       "weld_joints": [{"element": <element>,
//                        "location_in_drawing_frame_meters": <vector>}, ...],
//       "force_elements": [{"element": <element>,
//                           "reference_end_in_drawing_frame_meters": <vector>,
//                           "opposite_end_in_drawing_frame_meters": <vector>},
//                          ...],
//       "wheels": [{"element": <element>, "wheel_body": <element>,
//                   "datum_in_drawing_frame_meters": <vector>,
//                   "outward_spin_axis_in_drawing_frame": <vector>,
//                   "nominal_rolling_radius_meters": <number>}, ...],
//       "not_drawable_elements": [<element>, ...]}}
//     Every drawing names the element it shows, by kind and name: names are
//     shared across kinds, a wheelset body and its contact carrier for
//     instance. The kind is the one the drawing's list implies, a force
//     element's being one of the force-element kinds. No element is drawn
//     twice, and the drawings with not_drawable_elements hold exactly the
//     outline's elements of the kinds that have drawings: rigid bodies, fixed
//     frames, joints, force elements and contact interfaces; contact carriers
//     have none.
//     The preview of the last check that passed. project_revision is the
//     revision it was computed from; when it is below the response's revision,
//     the preview shows an older project. The drawing frame is the local track
//     frame at the layout reference station of a straight, level line: +x
//     forward along increasing station, +y to the right, +z downward, lengths
//     in metres. A placed free body is drawn at x = its station offset and at
//     y and z = its placement offsets, turned by its placement's rotation. A
//     revolute joint's axis is a unit vector.
//     A wheel is drawn for each contact interface of the contact record, in
//     its declaration's interface order; a project without a contact record
//     has no wheels. A wheel's element is its contact interface and its
//     wheel_body the rigid body it turns with: the wheelset body of a rigid
//     wheelset, the wheel's own body when it rotates independently. It is
//     drawn as its nominal rolling circle: the datum is the circle's centre,
//     the outward spin axis a unit vector along the spin axis pointing from
//     the carrier's wheel-profile origin towards the wheel, so the two wheels
//     of a carrier point apart, and the radius the record's nominal rolling
//     radius.
//     A free body without a placement, every body jointed to it, every frame,
//     joint and force element on those bodies and the contact interface of
//     every wheel turning with one of them are listed in not_drawable_elements
//     and appear in no other list.
//
// Example
//   > {"id": 7, "command": "set_property",
//      "element": {"kind": "rigid_body", "name": "carbody"},
//      "property": "name", "value": "car"}
//   < {"id": 7, "ok": false,
//      "error": {"element": {"kind": "rigid_body", "name": "carbody"},
//                "property": "name", "message": "'name' is read-only"}}

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstdio>
#include <exception>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_vehicle_contact_record.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/vehicle_project/free_body_placements.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project_editing/element_properties.h"
#include "orvd/vehicle_project_editing/mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"

namespace {

using Json = nlohmann::ordered_json;
using orvd::vehicle_project::ElementGroup;
using orvd::vehicle_project::ElementKind;
using orvd::vehicle_project::ElementReference;
using orvd::vehicle_project::VehicleProject;
using orvd::vehicle_project_editing::ElementPropertyDescription;
using orvd::vehicle_project_editing::ElementPropertyReading;
using orvd::vehicle_project_editing::JointLocationDrawing;
using orvd::vehicle_project_editing::MechanicalCheckReport;
using orvd::vehicle_project_editing::PropertyEditRefusal;
using orvd::vehicle_project_editing::PropertyValue;
using orvd::vehicle_project_editing::PropertyValueKind;
using orvd::vehicle_project_editing::VehicleProjectEditingSession;
using orvd::vehicle_project_editing::VehicleProjectPreview;

struct ElementKindName {
    ElementKind kind;
    std::string_view name;
};

// Every element kind and its protocol name, in the order the protocol lists
// them.
constexpr auto kElementKindNames = std::to_array<ElementKindName>({
    {ElementKind::kRigidBody, "rigid_body"},
    {ElementKind::kFixedFrame, "fixed_frame"},
    {ElementKind::kRevoluteJoint, "revolute_joint"},
    {ElementKind::kBallRpyJoint, "ball_rpy_joint"},
    {ElementKind::kWeldJoint, "weld_joint"},
    {ElementKind::kTranslationalSpringDamper, "translational_spring_damper"},
    {ElementKind::kRollSpringDamperCouple, "roll_spring_damper_couple"},
    {ElementKind::kSeriesSpringViscousDamper, "series_spring_viscous_damper"},
    {ElementKind::kPointToPointSeriesSpringBilinearDamper,
     "point_to_point_series_spring_bilinear_damper"},
    {ElementKind::kSaturatedPiecewiseLinearDamper, "saturated_piecewise_linear_damper"},
    {ElementKind::kReferenceFrameLinearBushing, "reference_frame_linear_bushing"},
    {ElementKind::kPointToPointLinearSpringDamper, "point_to_point_linear_spring_damper"},
    {ElementKind::kPointToPointHystereticSpring, "point_to_point_hysteretic_spring"},
    {ElementKind::kContactCarrier, "contact_carrier"},
    {ElementKind::kContactInterface, "contact_interface"},
});

std::string NameOfElementKind(ElementKind kind) {
    const auto named = std::ranges::find(kElementKindNames, kind, &ElementKindName::kind);
    if (named == kElementKindNames.end()) {
        throw std::logic_error("element kind " + std::to_string(std::to_underlying(kind)) +
                               " has no protocol name");
    }
    return std::string(named->name);
}

ElementKind ElementKindNamed(std::string_view name) {
    const auto named = std::ranges::find(kElementKindNames, name, &ElementKindName::name);
    if (named == kElementKindNames.end()) {
        throw std::invalid_argument("'" + std::string(name) + "' is not an element kind");
    }
    return named->kind;
}

std::string NameOfValueKind(PropertyValueKind kind) {
    switch (kind) {
        case PropertyValueKind::kNumber:
            return "number";
        case PropertyValueKind::kVector3:
            return "vector3";
        case PropertyValueKind::kText:
            return "text";
        case PropertyValueKind::kBoolean:
            return "boolean";
        case PropertyValueKind::kRotation:
            return "rotation";
        case PropertyValueKind::kWheelSide:
            return "wheel_side";
        case PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve:
            return "saturated_piecewise_linear_damper_curve";
    }
    throw std::logic_error("property value kind " + std::to_string(std::to_underlying(kind)) +
                           " has no protocol name");
}

Json EncodeVector(const Eigen::Vector3d& vector) {
    return Json::array({vector.x(), vector.y(), vector.z()});
}

Json EncodeRotation(const Eigen::Matrix3d& rotation) {
    Json rows = Json::array();
    for (int row = 0; row < 3; ++row) {
        rows.push_back(EncodeVector(rotation.row(row).transpose()));
    }
    return rows;
}

Json EncodeElement(const ElementReference& element) {
    return Json{{"kind", NameOfElementKind(element.kind)}, {"name", element.name}};
}

// One encoding per type a property value holds.
Json EncodeValue(double number) {
    return number;
}
Json EncodeValue(const Eigen::Vector3d& vector) {
    return EncodeVector(vector);
}
Json EncodeValue(const std::string& text) {
    return text;
}
Json EncodeValue(bool truth) {
    return truth;
}
Json EncodeValue(const Eigen::Matrix3d& rotation) {
    return EncodeRotation(rotation);
}
Json EncodeValue(orvd::wheel_rail_contact::WheelSide side) {
    return side == orvd::wheel_rail_contact::WheelSide::kLeft ? "left" : "right";
}
Json EncodeValue(
    const std::vector<orvd::configuration::VehicleSaturatedPiecewiseLinearDamperPointDefinition>&
        curve) {
    Json points = Json::array();
    for (const auto& point : curve) {
        points.push_back(
            Json{{"relative_velocity_meters_per_second", point.relative_velocity_meters_per_second},
                 {"force_newtons", point.force_newtons}});
    }
    return points;
}

Json EncodePropertyValue(const std::optional<PropertyValue>& value) {
    if (!value.has_value()) {
        return nullptr;
    }
    return std::visit([](const auto& held) { return EncodeValue(held); }, *value);
}

// The property value a JSON value states by its shape: only the shapes an
// edit can carry.
std::optional<PropertyValue> DecodePropertyValue(const Json& value) {
    if (value.is_number()) {
        return value.get<double>();
    }
    if (value.is_boolean()) {
        return value.get<bool>();
    }
    if (value.is_string()) {
        return value.get<std::string>();
    }
    if (value.is_array() && value.size() == 3 && std::ranges::all_of(value, &Json::is_number)) {
        return Eigen::Vector3d(value[0].get<double>(), value[1].get<double>(),
                               value[2].get<double>());
    }
    return std::nullopt;
}

ElementReference DecodeElement(const Json& element) {
    return {ElementKindNamed(element.at("kind").get<std::string>()),
            element.at("name").get<std::string>()};
}

Json EncodeDescription(const ElementPropertyDescription& description) {
    return Json{{"name", std::string(description.name)},
                {"value_kind", NameOfValueKind(description.value_kind)},
                {"editable", description.editable}};
}

Json EncodeGroups(const std::vector<ElementGroup>& groups) {
    Json encoded = Json::array();
    for (const ElementGroup& group : groups) {
        Json members = Json::array();
        for (const ElementReference& member : group.members) {
            members.push_back(EncodeElement(member));
        }
        encoded.push_back(Json{{"name", group.name},
                               {"members", std::move(members)},
                               {"subgroups", EncodeGroups(group.subgroups)}});
    }
    return encoded;
}

Json DescribeElementKinds() {
    Json kinds = Json::array();
    for (const ElementKindName& kind : kElementKindNames) {
        Json properties = Json::array();
        for (const ElementPropertyDescription& description :
             orvd::vehicle_project_editing::DescribeElementProperties(kind.kind)) {
            properties.push_back(EncodeDescription(description));
        }
        kinds.push_back(
            Json{{"kind", std::string(kind.name)}, {"properties", std::move(properties)}});
    }
    return Json{{"element_kinds", std::move(kinds)}};
}

Json EncodeOutline(const VehicleProject& project) {
    Json elements = Json::array();
    for (const ElementKindName& kind : kElementKindNames) {
        for (const std::string& name :
             orvd::vehicle_project_editing::CollectElementNames(project, kind.kind)) {
            elements.push_back(Json{{"kind", std::string(kind.name)}, {"name", name}});
        }
    }
    return Json{
        {"vehicle_name", project.vehicle_definition.vehicle_name},
        {"elements", std::move(elements)},
        {"element_groups", EncodeGroups(project.element_groups)},
        {"unplaced_free_bodies", orvd::vehicle_project::CollectUnplacedFreeBodyNames(project)},
        {"contact_record_present", project.contact_record.has_value()}};
}

Json EncodeCheckReport(const MechanicalCheckReport& report) {
    return Json{{"passed", report.refusals.empty()},
                {"refusals", report.refusals},
                {"information", report.information},
                {"coverage_statements", report.coverage_statements}};
}

// Every drawing names its element by kind and name, the kind being the one its
// list implies: names are shared across kinds, so a name alone cannot say
// which element a drawing shows.
Json EncodeJointLocations(ElementKind kind, const std::vector<JointLocationDrawing>& joints) {
    Json encoded = Json::array();
    for (const JointLocationDrawing& joint : joints) {
        encoded.push_back(Json{{"element", EncodeElement({kind, joint.joint_name})},
                               {"location_in_drawing_frame_meters",
                                EncodeVector(joint.location_in_drawing_frame_meters)}});
    }
    return encoded;
}

Json EncodePreview(const std::optional<VehicleProjectPreview>& preview) {
    if (!preview.has_value()) {
        return nullptr;
    }
    Json rigid_bodies = Json::array();
    for (const auto& body : preview->rigid_bodies) {
        rigid_bodies.push_back(Json{
            {"element", EncodeElement({ElementKind::kRigidBody, body.body_name})},
            {"rotation_drawing_from_body", EncodeRotation(body.rotation_drawing_from_body)},
            {"origin_in_drawing_frame_meters", EncodeVector(body.origin_in_drawing_frame_meters)}});
    }
    Json fixed_frames = Json::array();
    for (const auto& frame : preview->fixed_frames) {
        fixed_frames.push_back(
            Json{{"element", EncodeElement({ElementKind::kFixedFrame, frame.frame_name})},
                 {"rotation_drawing_from_frame", EncodeRotation(frame.rotation_drawing_from_frame)},
                 {"origin_in_drawing_frame_meters",
                  EncodeVector(frame.origin_in_drawing_frame_meters)}});
    }
    Json revolute_joints = Json::array();
    for (const auto& joint : preview->revolute_joints) {
        revolute_joints.push_back(
            Json{{"element", EncodeElement({ElementKind::kRevoluteJoint, joint.joint_name})},
                 {"location_in_drawing_frame_meters",
                  EncodeVector(joint.location_in_drawing_frame_meters)},
                 {"axis_in_drawing_frame", EncodeVector(joint.axis_in_drawing_frame)}});
    }
    Json force_elements = Json::array();
    for (const auto& element : preview->force_elements) {
        force_elements.push_back(
            Json{{"element", EncodeElement(element.element)},
                 {"reference_end_in_drawing_frame_meters",
                  EncodeVector(element.reference_end_in_drawing_frame_meters)},
                 {"opposite_end_in_drawing_frame_meters",
                  EncodeVector(element.opposite_end_in_drawing_frame_meters)}});
    }
    Json wheels = Json::array();
    for (const auto& wheel : preview->wheels) {
        wheels.push_back(Json{
            {"element", EncodeElement({ElementKind::kContactInterface, wheel.interface_name})},
            {"wheel_body", EncodeElement({ElementKind::kRigidBody, wheel.wheel_body_name})},
            {"datum_in_drawing_frame_meters", EncodeVector(wheel.datum_in_drawing_frame_meters)},
            {"outward_spin_axis_in_drawing_frame",
             EncodeVector(wheel.outward_spin_axis_in_drawing_frame)},
            {"nominal_rolling_radius_meters", wheel.nominal_rolling_radius_meters}});
    }
    Json not_drawable_elements = Json::array();
    for (const ElementReference& element : preview->not_drawable_elements) {
        not_drawable_elements.push_back(EncodeElement(element));
    }
    return Json{
        {"project_revision", preview->project_revision},
        {"rigid_bodies", std::move(rigid_bodies)},
        {"fixed_frames", std::move(fixed_frames)},
        {"revolute_joints", std::move(revolute_joints)},
        {"ball_rpy_joints",
         EncodeJointLocations(ElementKind::kBallRpyJoint, preview->ball_rpy_joints)},
        {"weld_joints", EncodeJointLocations(ElementKind::kWeldJoint, preview->weld_joints)},
        {"force_elements", std::move(force_elements)},
        {"wheels", std::move(wheels)},
        {"not_drawable_elements", std::move(not_drawable_elements)}};
}

Json Accepted(const Json& id, const VehicleProjectEditingSession& session, Json result) {
    return Json{{"id", id},
                {"ok", true},
                {"revision", session.revision()},
                {"can_undo", session.can_undo()},
                {"can_redo", session.can_redo()},
                {"result", std::move(result)}};
}

Json Refused(const Json& id, Json element, Json property, std::string_view message) {
    return Json{{"id", id},
                {"ok", false},
                {"error", Json{{"element", std::move(element)},
                               {"property", std::move(property)},
                               {"message", std::string(message)}}}};
}

VehicleProjectEditingSession OpenVehicle(const Json& request) {
    VehicleProject project{.vehicle_definition =
                               orvd::configuration::LoadVehicleDefinitionFromJsonFile(
                                   request.at("vehicle_definition_path").get<std::string>())};
    const orvd::configuration::ResolvedStartupState startup_state =
        orvd::configuration::LoadResolvedStartupStateFromJsonFile(
            request.at("startup_state_path").get<std::string>());
    orvd::vehicle_project::PlaceFreeBodiesAtStartupPoses(project, startup_state);
    project.contact_record = orvd::configuration::LoadVehicleContactRecordFromJsonFile(
        request.at("contact_record_path").get<std::string>());
    return VehicleProjectEditingSession(
        std::move(project), startup_state.gravitational_acceleration_meters_per_second_squared);
}

Json GetProperties(const Json& id, const Json& request,
                   const VehicleProjectEditingSession& session) {
    const ElementReference element = DecodeElement(request.at("element"));
    std::vector<ElementPropertyReading> readings;
    try {
        readings = orvd::vehicle_project_editing::ReadElementProperties(session.project(), element);
    } catch (const std::invalid_argument& refusal) {
        return Refused(id, EncodeElement(element), nullptr, refusal.what());
    }
    Json properties = Json::array();
    for (const ElementPropertyReading& reading : readings) {
        Json property = EncodeDescription(reading.description);
        property["value"] = EncodePropertyValue(reading.value);
        properties.push_back(std::move(property));
    }
    return Accepted(id, session, Json{{"properties", std::move(properties)}});
}

Json SetProperty(const Json& id, const Json& request, VehicleProjectEditingSession& session) {
    const ElementReference element = DecodeElement(request.at("element"));
    const std::string property_name = request.at("property").get<std::string>();
    const std::optional<PropertyValue> value = DecodePropertyValue(request.value("value", Json()));
    if (!value.has_value()) {
        return Refused(id, EncodeElement(element), property_name,
                       "the value must be a number, an array of three numbers, a string or a "
                       "boolean");
    }
    const auto edited = session.SetElementProperty(element, property_name, *value);
    if (!edited.has_value()) {
        const PropertyEditRefusal& refusal = edited.error();
        return Refused(id, EncodeElement(refusal.element), refusal.property_name, refusal.reason);
    }
    return Accepted(id, session, Json::object());
}

// The response to one request line. Whatever a command throws becomes a
// refusal, so every line is answered.
Json Respond(const std::string& line, std::optional<VehicleProjectEditingSession>& session) {
    Json id = nullptr;
    try {
        const Json request = Json::parse(line);
        if (request.is_object() && request.contains("id")) {
            id = request.at("id");
        }
        const std::string command = request.at("command").get<std::string>();
        if (command == "open_vehicle") {
            session = OpenVehicle(request);
            return Accepted(id, *session, Json::object());
        }
        if (!session.has_value()) {
            throw std::invalid_argument("no vehicle is open; send open_vehicle first");
        }
        VehicleProjectEditingSession& open_session = *session;
        if (command == "describe_element_kinds") {
            return Accepted(id, open_session, DescribeElementKinds());
        }
        if (command == "get_outline") {
            return Accepted(id, open_session, EncodeOutline(open_session.project()));
        }
        if (command == "get_properties") {
            return GetProperties(id, request, open_session);
        }
        if (command == "set_property") {
            return SetProperty(id, request, open_session);
        }
        if (command == "undo") {
            open_session.Undo();
            return Accepted(id, open_session, Json::object());
        }
        if (command == "redo") {
            open_session.Redo();
            return Accepted(id, open_session, Json::object());
        }
        if (command == "check") {
            return Accepted(id, open_session, EncodeCheckReport(open_session.CheckMechanics()));
        }
        if (command == "get_preview") {
            return Accepted(id, open_session,
                            Json{{"preview", EncodePreview(open_session.preview())}});
        }
        throw std::invalid_argument("'" + command + "' is not a command");
    } catch (const std::exception& refusal) {
        return Refused(id, nullptr, nullptr, refusal.what());
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 1) {
        std::fprintf(stderr,
                     "usage: %s\n"
                     "  takes no arguments: it answers newline-delimited JSON requests on "
                     "standard input, one response line each on standard output\n",
                     argv[0]);
        return 2;
    }
    std::optional<VehicleProjectEditingSession> session;
    std::string line;
    while (std::getline(std::cin, line)) {
        std::cout << Respond(line, session).dump() << '\n' << std::flush;
    }
    return 0;
}
