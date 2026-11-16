// orvd_vehicle_modeling_service: the resident command layer a graphical
// modelling tool drives over one vehicle project.
//
// The service is deliberately thin. It parses a request, calls
// orvd::vehicle_project_editing, and writes the answer: all JSON lives here,
// and every rule an edit follows lives there. It reads a vehicle and never
// writes a project.
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
// given twice takes its last value. Only what a command uses is checked, and
// every argument a command shows below is one it uses: a request missing one
// is refused and changes nothing. An absent argument is never read as null or
// as empty.
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
//   refusal concerns when it concerns them; for add_element, element is the
//   one that was to be created. id is null when the line holds no JSON object
//   with an id.
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
//
// Name spaces
//   Rigid bodies and fixed frames share one name space, in which "world" is
//   taken as well; the three joint kinds share one; the eight force element
//   kinds share one; contact carriers and contact interfaces have one each.
//   No two elements of one name space have the same name, while elements of
//   different name spaces may: a wheelset body and its contact carrier, for
//   instance.
//
// Properties
//   Each element kind has one fixed list of properties, named by its record
//   members' own names, which state meaning and unit. A rigid body adds where
//   it is along, across and above the track:
//   mechanical_track_station_layout.station_offset_meters,
//   placement.lateral_offset_in_local_track_frame_meters and
//   placement.vertical_offset_in_local_track_frame_meters.
//   A property is described as
//     {"name": "<name>", "value_kind": "<value kind>", "editable": <bool>,
//      "referenced_kinds": [<element kind>, ...], "optional": <bool>,
//      "choices": ["<token>", ...], "row_fields": ["<field name>", ...]}
//   where referenced_kinds and optional are given only for an
//   element_reference, choices only for a token and row_fields only for a
//   saturated_piecewise_linear_damper_curve, and holds a value of its value
//   kind:
//     number             a JSON number
//     vector3            a vector
//     boolean            true or false
//     rotation           a rotation
//     element_name       a string: the element's own name; setting it renames
//                        the element
//     element_reference  a string naming an element of one of
//                        referenced_kinds, which all lie in one name space, or
//                        null where optional is true and it names none
//     token              a string, one of choices: a force element's axis, a
//                        wheel's side
//     saturated_piecewise_linear_damper_curve
//                        [{"<row field>": <number>, ...}, ...], each row
//                        giving every one of row_fields; possibly empty
//   A value is also null where the element holds no value for the property:
//   the station offset of a rigid body the mechanical track station layout
//   does not place, and the placement offsets of a rigid body without a
//   placement. Every property is editable except a rotation.
//
// Commands
//
//   open_vehicle  {"vehicle_definition_path": "<path>",
//                  "startup_state_path": "<path>",
//                  "contact_record_path": "<path>"}
//     Loads the three records with the strict loaders. The start-up record
//     supplies drawing placements and gravity; contact declaration problems
//     are reported by check. Every rigid body's shapes are stated under its
//     name in the vehicle definition. Opens a new session: revision 0, empty
//     history, no preview. A refused open keeps the session that was open.
//     Every other command is refused while no vehicle is open.
//     result: {}
//
//   describe_element_kinds
//     result: {"element_kinds": [{"kind": <element kind>,
//                                 "properties": [<description>, ...]}, ...]}
//     Every element kind, in the order above, with its properties described in
//     the order get_properties returns them. editable says whether the kind's
//     property can be edited at all.
//
//   get_outline
//     result: {"vehicle_name": "<name>", "elements": [<element>, ...],
//              "element_groups": [<group>, ...],
//              "unplaced_free_bodies": ["<body name>", ...],
//              "contact_record_present": <bool>}
//     group: {"name": "<name>", "members": [<element>, ...],
//             "subgroups": [<group>, ...]}
//     elements lists every element, kind by kind in the order above, each kind
//     in declaration order: the outline order.
//
//   get_properties  {"element": <element>}
//     result: {"properties": [<description> with "value": <value>, ...]}
//     editable says whether this element's property can be edited now. It is
//     the kind's editable, except that a station offset or a placement offset
//     whose value is null is not editable; a reference that names none is
//     null and editable.
//
//   set_property  {"element": <element>, "property": "<name>", "value": <value>}
//     result: {"element": <element>}, the element after the edit, whose name
//     differs from the one given after a rename.
//     value is required, null included: an explicit null makes a reference
//     that may name none name none. The value is read as the property's value
//     kind, and a value of another shape is refused. The edit is then refused
//     unless the element exists, the property is editable, every number in
//     the value is finite, a station offset or placement offset has the record
//     holding it, a reference names an element of one of its referenced_kinds
//     or, where optional, none, a token is one of its choices, and a new name
//     is not empty and taken by no other element of the element's name space.
//     Renaming changes every reference to the element and its group
//     memberships, and for a rigid body its station layout entry, the
//     layout's reference body and wheel-pair station list and its placement;
//     the name its shapes are stated under stays. Making a rigid body move
//     freely gives it a station layout entry and a placement where it has
//     none, at zero offsets and the identity rotation; making it no longer
//     move freely takes both away, and is refused for the station layout's
//     reference body. References the project already holds are not judged
//     here, physical plausibility and the layout's own consistency neither;
//     check judges them.
//
//   add_element  {"kind": <element kind>, "name": "<name>",
//                 "properties": {"<property name>": <value>, ...}}
//     result: {"element": <element>}
//     Appends a new element to its kind's records: the record's default value
//     with this name and the given properties, each read and checked as
//     set_property reads and checks it. properties is required and may be
//     empty: it may leave any property out, except a reference that cannot
//     name none, and cannot hold the element's own name or a property that is
//     not editable. The name is refused as a rename's would be. A contact kind
//     needs the project to have a contact record. A new rigid body that moves
//     freely gets its station layout entry and placement before their offsets
//     are set, and has no shapes.
//
//   remove_element  {"element": <element>}
//     result: {"removed_elements": [<element>, ...]}
//     Removes the element and every element holding a reference to one
//     removed, transitively, with their station layout entries, placements,
//     shape names and group memberships; groups stay. removed_elements lists
//     everything removed, the element included, in outline order. The station
//     layout's reference body cannot be removed.
//
//   get_removal_consequences  {"element": <element>}
//     result: {"removed_elements": [<element>, ...]}
//     What remove_element would remove, or its refusal. Changes nothing.
//
//   set_property, add_element and remove_element are each one edit: one undo
//   step and one revision. After any edit, undo or redo an element may be
//   gone or renamed, so a client reads the outline again.
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
//                         "origin_in_drawing_frame_meters": <vector>,
//                         "visual_definition_body_name": "<name>" or null},
//                        ...],
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
//     Every drawing names the element it shows, by kind and name. The kind is
//     the one the drawing's list implies, a force element's being one of the
//     force-element kinds. No element is drawn twice, and the drawings with
//     not_drawable_elements hold exactly the outline's elements of the kinds
//     that have drawings: rigid bodies, fixed frames, joints, force elements
//     and contact interfaces; contact carriers have none.
//     The preview of the last check that passed. project_revision is the
//     revision it was computed from; when it is below the response's revision,
//     the preview shows an older project. The drawing frame is the local track
//     frame at the layout reference station of a straight, level line: +x
//     forward along increasing station, +y to the right, +z downward, lengths
//     in metres. A placed free body is drawn at x = its station offset and at
//     y and z = its placement offsets, turned by its placement's rotation.
//     visual_definition_body_name is the name a rigid body's shapes are
//     stated under: its name when the vehicle was opened, kept through
//     renames, and null for a body created since, which has no shapes. A
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
//      "property": "name", "value": ""}
//   < {"id": 7, "ok": false,
//      "error": {"element": {"kind": "rigid_body", "name": "carbody"},
//                "property": "name",
//                "message": "an element's name cannot be empty"}}

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <expected>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_vehicle_contact_record.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/vehicle_project/make_vehicle_project_from_vehicle_definition.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_free_body_placements.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_element_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_element_properties.h"
#include "orvd/vehicle_project_editing/vehicle_project_mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"

namespace {

using Json = nlohmann::ordered_json;
using orvd::vehicle_project::ElementGroup;
using orvd::vehicle_project::ElementKind;
using orvd::vehicle_project::ElementReference;
using orvd::vehicle_project::VehicleProject;
using orvd::vehicle_project_editing::CurveRowField;
using orvd::vehicle_project_editing::ElementEditRefusal;
using orvd::vehicle_project_editing::ElementOwnName;
using orvd::vehicle_project_editing::ElementPropertyDescription;
using orvd::vehicle_project_editing::ElementPropertyReading;
using orvd::vehicle_project_editing::JointLocationDrawing;
using orvd::vehicle_project_editing::MechanicalCheckReport;
using orvd::vehicle_project_editing::PropertyToken;
using orvd::vehicle_project_editing::PropertyValue;
using orvd::vehicle_project_editing::PropertyValueKind;
using orvd::vehicle_project_editing::PropertyValuesByName;
using orvd::vehicle_project_editing::ReferencedElementName;
using orvd::vehicle_project_editing::SaturatedPiecewiseLinearDamperCurve;
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
        case PropertyValueKind::kBoolean:
            return "boolean";
        case PropertyValueKind::kRotation:
            return "rotation";
        case PropertyValueKind::kElementName:
            return "element_name";
        case PropertyValueKind::kElementReference:
            return "element_reference";
        case PropertyValueKind::kToken:
            return "token";
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

Json EncodeElements(const std::vector<ElementReference>& elements) {
    Json encoded = Json::array();
    for (const ElementReference& element : elements) {
        encoded.push_back(EncodeElement(element));
    }
    return encoded;
}

// One encoding per type a property value holds.
Json EncodeValue(double number) {
    return number;
}
Json EncodeValue(const Eigen::Vector3d& vector) {
    return EncodeVector(vector);
}
Json EncodeValue(bool truth) {
    return truth;
}
Json EncodeValue(const Eigen::Matrix3d& rotation) {
    return EncodeRotation(rotation);
}
Json EncodeValue(const ElementOwnName& own_name) {
    return own_name.name;
}
Json EncodeValue(const ReferencedElementName& referenced) {
    return referenced.name.has_value() ? Json(*referenced.name) : Json(nullptr);
}
Json EncodeValue(const PropertyToken& token) {
    return token.token;
}

// A curve, row by row under the property's row fields.
Json EncodeCurve(const SaturatedPiecewiseLinearDamperCurve& curve,
                 std::span<const CurveRowField> row_fields) {
    Json rows = Json::array();
    for (const auto& point : curve) {
        Json row = Json::object();
        for (const CurveRowField& field : row_fields) {
            row[std::string(field.name)] = point.*field.member;
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

Json EncodePropertyValue(const std::optional<PropertyValue>& value,
                         const ElementPropertyDescription& description) {
    if (!value.has_value()) {
        return nullptr;
    }
    return std::visit(
        [&description]<typename Held>(const Held& held) {
            if constexpr (std::is_same_v<Held, SaturatedPiecewiseLinearDamperCurve>) {
                return EncodeCurve(held, description.row_fields);
            } else {
                return EncodeValue(held);
            }
        },
        *value);
}

bool IsNumbers(const Json& value, std::size_t count) {
    return value.is_array() && value.size() == count &&
           std::ranges::all_of(value, &Json::is_number);
}

Eigen::Vector3d DecodeVector(const Json& value) {
    return {value[0].get<double>(), value[1].get<double>(), value[2].get<double>()};
}

// The property value a JSON value states, read as the value kind the property
// is described with, or why its shape is not that kind's.
std::expected<PropertyValue, std::string> DecodePropertyValue(
    const Json& value, const ElementPropertyDescription& description) {
    const std::string property = "'" + std::string(description.name) + "'";
    switch (description.value_kind) {
        case PropertyValueKind::kNumber:
            if (value.is_number()) {
                return value.get<double>();
            }
            return std::unexpected(property + " takes a number");
        case PropertyValueKind::kVector3:
            if (IsNumbers(value, 3)) {
                return DecodeVector(value);
            }
            return std::unexpected(property + " takes an array of three numbers");
        case PropertyValueKind::kBoolean:
            if (value.is_boolean()) {
                return value.get<bool>();
            }
            return std::unexpected(property + " takes true or false");
        case PropertyValueKind::kRotation:
            if (value.is_array() && value.size() == 3 &&
                std::ranges::all_of(value, [](const Json& row) { return IsNumbers(row, 3); })) {
                Eigen::Matrix3d rotation;
                for (int row = 0; row < 3; ++row) {
                    rotation.row(row) = DecodeVector(value[row]).transpose();
                }
                return rotation;
            }
            return std::unexpected(property + " takes three rows of three numbers");
        case PropertyValueKind::kElementName:
            if (value.is_string()) {
                return ElementOwnName{value.get<std::string>()};
            }
            return std::unexpected(property + " takes a string");
        case PropertyValueKind::kElementReference:
            if (value.is_string()) {
                return ReferencedElementName{value.get<std::string>()};
            }
            if (value.is_null()) {
                return ReferencedElementName{std::nullopt};
            }
            return std::unexpected(property + " takes a string or null");
        case PropertyValueKind::kToken:
            if (value.is_string()) {
                return PropertyToken{value.get<std::string>()};
            }
            return std::unexpected(property + " takes a string");
        case PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve: {
            const auto is_row = [&description](const Json& row) {
                return row.is_object() &&
                       std::ranges::all_of(description.row_fields,
                                           [&row](const CurveRowField& field) {
                                               const auto entry = row.find(std::string(field.name));
                                               return entry != row.end() && entry->is_number();
                                           });
            };
            if (!value.is_array() || !std::ranges::all_of(value, is_row)) {
                return std::unexpected(property +
                                       " takes an array of rows, each giving a number for every "
                                       "one of its row fields");
            }
            SaturatedPiecewiseLinearDamperCurve curve;
            for (const Json& row : value) {
                auto& point = curve.emplace_back();
                for (const CurveRowField& field : description.row_fields) {
                    point.*field.member = row.at(std::string(field.name)).get<double>();
                }
            }
            return curve;
        }
    }
    throw std::logic_error("property value kind " +
                           std::to_string(std::to_underlying(description.value_kind)) +
                           " has no decoding");
}

ElementReference DecodeElement(const Json& element) {
    return {ElementKindNamed(element.at("kind").get<std::string>()),
            element.at("name").get<std::string>()};
}

Json EncodeDescription(const ElementPropertyDescription& description) {
    Json encoded{{"name", std::string(description.name)},
                 {"value_kind", NameOfValueKind(description.value_kind)},
                 {"editable", description.editable}};
    if (description.value_kind == PropertyValueKind::kElementReference) {
        Json referenced_kinds = Json::array();
        for (const ElementKind kind : description.referenced_kinds) {
            referenced_kinds.push_back(NameOfElementKind(kind));
        }
        encoded["referenced_kinds"] = std::move(referenced_kinds);
        encoded["optional"] = description.may_name_none;
    }
    if (description.value_kind == PropertyValueKind::kToken) {
        Json choices = Json::array();
        for (const std::string_view token : description.choices) {
            choices.push_back(std::string(token));
        }
        encoded["choices"] = std::move(choices);
    }
    if (description.value_kind == PropertyValueKind::kSaturatedPiecewiseLinearDamperCurve) {
        Json row_fields = Json::array();
        for (const CurveRowField& field : description.row_fields) {
            row_fields.push_back(std::string(field.name));
        }
        encoded["row_fields"] = std::move(row_fields);
    }
    return encoded;
}

Json EncodeGroups(const std::vector<ElementGroup>& groups) {
    Json encoded = Json::array();
    for (const ElementGroup& group : groups) {
        encoded.push_back(Json{{"name", group.name},
                               {"members", EncodeElements(group.members)},
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
            {"origin_in_drawing_frame_meters", EncodeVector(body.origin_in_drawing_frame_meters)},
            {"visual_definition_body_name", body.visual_definition_body_name.has_value()
                                                ? Json(*body.visual_definition_body_name)
                                                : Json(nullptr)}});
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
        {"not_drawable_elements", EncodeElements(preview->not_drawable_elements)}};
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

Json RefusedEdit(const Json& id, const ElementEditRefusal& refusal) {
    return Refused(id, EncodeElement(refusal.element),
                   refusal.property_name.has_value() ? Json(*refusal.property_name) : Json(nullptr),
                   refusal.reason);
}

VehicleProjectEditingSession OpenVehicle(const Json& request) {
    VehicleProject project = orvd::vehicle_project::MakeVehicleProject(
        orvd::configuration::LoadVehicleDefinitionFromJsonFile(
            request.at("vehicle_definition_path").get<std::string>()));
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
        property["value"] = EncodePropertyValue(reading.value, reading.description);
        properties.push_back(std::move(property));
    }
    return Accepted(id, session, Json{{"properties", std::move(properties)}});
}

// The value given for the property of `kind` named `property_name`, read as
// the value kind the property is described with, or why it cannot be.
std::expected<PropertyValue, std::string> DecodeGivenValue(ElementKind kind,
                                                           const std::string& property_name,
                                                           const Json& value) {
    const auto described =
        orvd::vehicle_project_editing::DescribeElementProperty(kind, property_name);
    if (!described.has_value()) {
        return std::unexpected(described.error());
    }
    return DecodePropertyValue(value, **described);
}

Json SetProperty(const Json& id, const Json& request, VehicleProjectEditingSession& session) {
    const ElementReference element = DecodeElement(request.at("element"));
    const std::string property_name = request.at("property").get<std::string>();
    const std::expected<PropertyValue, std::string> value =
        DecodeGivenValue(element.kind, property_name, request.at("value"));
    if (!value.has_value()) {
        return Refused(id, EncodeElement(element), property_name, value.error());
    }
    const auto edited = session.SetElementProperty(element, property_name, *value);
    if (!edited.has_value()) {
        return RefusedEdit(id, edited.error());
    }
    return Accepted(id, session, Json{{"element", EncodeElement(*edited)}});
}

Json AddElement(const Json& id, const Json& request, VehicleProjectEditingSession& session) {
    const ElementReference element{ElementKindNamed(request.at("kind").get<std::string>()),
                                   request.at("name").get<std::string>()};
    const Json& given = request.at("properties");
    if (!given.is_object()) {
        return Refused(id, EncodeElement(element), nullptr,
                       "properties must be an object of property values by property name");
    }
    PropertyValuesByName properties;
    for (const auto& [property_name, given_value] : given.items()) {
        std::expected<PropertyValue, std::string> value =
            DecodeGivenValue(element.kind, property_name, given_value);
        if (!value.has_value()) {
            return Refused(id, EncodeElement(element), property_name, value.error());
        }
        properties.emplace(property_name, std::move(*value));
    }
    const auto created = session.AddElement(element.kind, element.name, properties);
    if (!created.has_value()) {
        return RefusedEdit(id, created.error());
    }
    return Accepted(id, session, Json{{"element", EncodeElement(*created)}});
}

Json RemoveElement(const Json& id, const Json& request, VehicleProjectEditingSession& session) {
    const auto removed = session.RemoveElement(DecodeElement(request.at("element")));
    if (!removed.has_value()) {
        return RefusedEdit(id, removed.error());
    }
    return Accepted(id, session, Json{{"removed_elements", EncodeElements(*removed)}});
}

Json GetRemovalConsequences(const Json& id, const Json& request,
                            const VehicleProjectEditingSession& session) {
    const auto removed = session.CollectRemovalConsequences(DecodeElement(request.at("element")));
    if (!removed.has_value()) {
        return RefusedEdit(id, removed.error());
    }
    return Accepted(id, session, Json{{"removed_elements", EncodeElements(*removed)}});
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
        if (command == "add_element") {
            return AddElement(id, request, open_session);
        }
        if (command == "remove_element") {
            return RemoveElement(id, request, open_session);
        }
        if (command == "get_removal_consequences") {
            return GetRemovalConsequences(id, request, open_session);
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
