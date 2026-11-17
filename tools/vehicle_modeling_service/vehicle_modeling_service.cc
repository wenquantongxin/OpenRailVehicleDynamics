// orvd_vehicle_modeling_service: the resident command layer a graphical
// modelling tool drives over one vehicle project.
//
// The service is deliberately thin. It parses a request, calls
// orvd::vehicle_project and orvd::vehicle_project_editing, and writes the
// answer: the protocol's JSON lives here, and every rule an edit, a check or a
// project file follows lives there. It opens a vehicle from its own files or a
// project from its project file, and the only files it writes are project
// files, each <name>.vehicle_project.json in the directory a request names.
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
//    "can_redo": <bool>, "has_unsaved_changes": <bool>, "result": {...}}
//   revision, can_undo, can_redo and has_unsaved_changes describe the session
//   after the command. All four are null while nothing is open, when only
//   get_session and list_vehicle_projects are answered. Opening a vehicle or a
//   project starts at revision 0; every accepted edit, undo and redo adds one,
//   and so does a save_vehicle_project_as that changes the vehicle's names;
//   nothing else changes it, and it never decreases: undoing restores an
//   earlier project, not its revision. has_unsaved_changes says whether the
//   project differs from the state last saved or, before any save, from the
//   state it was opened with. It follows undo and redo: undoing back to that
//   state leaves nothing unsaved, and an edit made after undoing past it
//   leaves the changes unsaved until the next save.
//
// Response to a refused command
//   {"id": <id>, "ok": false,
//    "error": {"subject": <subject> or null, "property": "<name>" or null,
//              "message": "<text>"}}
//   A refused command changes nothing. subject and property name what the
//   refusal concerns when it concerns them; for add_element, subject is the
//   element that was to be created. id is null when the line holds no JSON
//   object with an id.
//
// Values
//   element        {"kind": "<element kind>", "name": "<element name>"}
//   element kind   one of, in this order: rigid_body, fixed_frame,
//                  revolute_joint, ball_rpy_joint, weld_joint,
//                  translational_spring_damper, roll_spring_damper_couple,
//                  series_spring_viscous_damper,
//                  point_to_point_series_spring_bilinear_damper,
//                  saturated_piecewise_linear_damper,
//                  reference_frame_linear_bushing,
//                  point_to_point_linear_spring_damper,
//                  point_to_point_hysteretic_spring, contact_carrier,
//                  contact_interface
//   settings word  one of gravity, vehicle_identity, wheel, rail,
//                  contact_computation: the project's settings, which are not
//                  elements and are never created, removed, renamed or named
//                  by a reference
//   subject        {"element": <element>} or {"settings": "<settings word>"}:
//                  what has properties
//   title          {"en": "<text>", "zh": "<text>"}: a text for people, in
//                  English and in Chinese
//   vector         [x, y, z]
//   rotation       [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]], the
//                  rows of a rotation matrix; rotation_a_from_b takes b
//                  components to a components
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
//   A subject has one fixed list of properties, named by its record members'
//   own names, which state meaning and unit. An element kind's list holds its
//   record's members. A rigid body's leaves out whether it moves freely in the
//   world, which its attachment states, and adds where it is along, across and
//   above the track, which only a free body has:
//   mechanical_track_station_layout.station_offset_meters,
//   placement.lateral_offset_in_local_track_frame_meters and
//   placement.vertical_offset_in_local_track_frame_meters. Settings list their
//   record's members, a member of a record held inside under the holding
//   member's name, a dot and its own (friction.static_coefficient):
//     gravity              the gravitational acceleration the project is
//                          checked under
//     vehicle_identity     the vehicle name and the mechanical definition
//                          identifier
//     wheel, rail,         every member of the wheel-rail contact record's
//     contact_computation  three configuration records
//   A property is described as
//     {"name": "<name>", "value_kind": "<value kind>", "editable": <bool>,
//      "section": <title> or null,
//      "referenced_kinds": [<element kind>, ...], "optional": <bool>,
//      "choices": ["<token>", ...], "row_fields": ["<field name>", ...]}
//   where referenced_kinds and optional are given only for an
//   element_reference, choices only for a token and row_fields only for a
//   saturated_piecewise_linear_damper_curve, and holds a value of its value
//   kind:
//     number             a JSON number; a count held as an integer is one
//     vector3            a vector
//     rotation           a rotation
//     text               a string shown as it is: never an element's own name
//                        and never a reference
//     element_name       a string: the element's own name; setting it renames
//                        the element
//     element_reference  a string naming an element of one of
//                        referenced_kinds, which all lie in one name space, or
//                        null where optional is true and it names none
//     token              a string, one of choices: a force element's axis, a
//                        wheel's side, a value of an enumeration
//     saturated_piecewise_linear_damper_curve
//                        [{"<row field>": <number>, ...}, ...], each row
//                        giving every one of row_fields; possibly empty
//   A value is also null where the subject holds no value for the property:
//   the station offset of a rigid body the mechanical track station layout
//   does not place, the placement offsets of a rigid body without a
//   placement, and the gravity of a project that states none.
//   section is where the property is shown: consecutive properties of one
//   section form it. A rigid body's properties are shown in the sections mass
//   and inertia, station along the track from the vehicle layout zero (the
//   station offset, which the dynamics use) and preview placement (the
//   placement offsets, which only place it in the drawing), after its
//   attachment; every other property is in none.
//   Every element property is editable except a rotation. Of the settings,
//   the gravity, the wheel's nominal_rolling_radius_meters and
//   wheel_lateral_datum_magnitude_meters, and the rail's track_gauge_meters,
//   gauge_measuring_depth_meters, pose_rail_cant_radians (the wheel-rail pose
//   and gauge datum's), geometry_rail_cant_radians (the contact geometry's)
//   and profile_origin_mode are editable, the gravity also while the project
//   states none; the rest is read-only. A settings value is judged no
//   further than being finite: check judges the gravity, and a run the wheel
//   and rail values it assembles.
//
// Commands
//
//   get_session
//     result: {"open": false}, or {"open": true, "vehicle_project_file":
//             {"directory": "<path>", "name": "<name>"} or null}
//     Never refused, changes nothing. vehicle_project_file is the project file
//     the session was opened from or last saved as; null for a vehicle opened
//     from its own files and not saved yet.
//
//   list_vehicle_projects  {"directory": "<path>"}
//     result: {"vehicle_projects": ["<name>", ...]}
//     The names of the projects saved in the directory, sorted: one for each
//     file <name>.vehicle_project.json there whose <name> is a project name.
//     Answered whether or not anything is open; changes nothing. Refused when
//     the directory cannot be read.
//
//   open_vehicle  {"vehicle_definition_path": "<path>",
//                  "startup_state_path": "<path>",
//                  "contact_record_path": "<path>",
//                  "visual_definition_path": "<path>" or null}
//     Loads the three records with the strict loaders. The start-up record
//     supplies drawing placements and the gravity the project is checked
//     under; contact declaration problems are reported by check. The visual
//     definition file must hold a JSON object, which becomes the project's
//     visual definition; with null the project has none. Every rigid body's
//     shapes are stated under its name in the vehicle definition. Opens a new
//     session: revision 0, empty history, no preview, no project file and
//     nothing unsaved. A refused open keeps the session that was open.
//     result: {}
//
//   open_vehicle_project  {"directory": "<path>", "name": "<name>"}
//     result: {}
//     Reads the project file of that name in that directory. Reading refuses
//     only what is wrong with the document; whatever is wrong with the vehicle
//     is reported by check once it is open. Opens a new session as
//     open_vehicle does, associated with that file. A refused open keeps the
//     session that was open.
//
//   Every other command is refused while nothing is open.
//
//   get_model_tree
//     result: {"nodes": [<node>, ...]}
//     node: {"key": "<text>", "title": <title> or null,
//            "subject": <subject> or null, "provided": <bool>,
//            "note": <title> or null,
//            "creates": [{"kind": <element kind>,
//                         "fixed_properties": {"<property name>": <value>,
//                                              ...}}, ...],
//            "attaches": [<element>, ...],
//            "children": [<node>, ...]}
//     The project as a model tree divided into domains, made from its records
//     whatever check would say of them: every element of the outline is one
//     node, exactly once. The nodes are the domains, in this order:
//       Globals         the gravity and vehicle identity settings; the
//                       vehicle layout zero, which the free bodies' station
//                       offsets are measured from and which no record names,
//                       a node selecting nothing; initial conditions, views
//                       and simulation settings, not provided
//       Bodies          every rigid body, with its markers (the fixed frames
//                       naming it) and its geometry (a note on the name its
//                       shapes are stated under); fixed frames naming a body
//                       the project does not have are under "Owner not found"
//       Joints          one list of the joints, in outline order
//       Force elements  one list of the force elements, in outline order
//       Constraints     not provided
//       Wheel-rail      the wheel, rail and contact computation settings and
//                       the contact carriers, each with the contact interfaces
//                       naming it; interfaces naming a carrier the project does
//                       not have are under "Owner not found". Without a contact
//                       record it has no children and a note saying so.
//       Track           not provided
//       Substructures   not provided
//     key is unique in the tree and stays a node's key across edits, except
//     that an element node's key follows the element's name and the keys of
//     the nodes below it follow it. title is null for an element node, which
//     shows the element's name; an element of a list of several kinds shows
//     its kind's title as a tag. subject is what selecting the node selects,
//     null for a node that only groups or only says what it is, as the
//     vehicle layout zero does. provided is false for what is not provided
//     yet: such a node has no subject, no offers and no children, and its note
//     says what will be there. note is one line shown with the node:
//     a rigid body's node carries its attachment's note while the body is
//     unattached or conflicting. creates lists what the node's "+" creates,
//     each with the properties a creation is given and cannot change: a marker
//     on its body, a contact interface on its carrier. attaches lists, in
//     outline order, the rigid bodies the node's "+" attaches by a joint
//     (set_attachment): the joints node lists the free bodies that can take a
//     joint and the unattached ones, and creates nothing; every other node's
//     attaches is empty. A node with neither has no "+".
//
//   describe_element_kinds
//     result: {"element_kinds": [{"kind": <element kind>, "title": <title>,
//                                 "attachment": <attachment description>
//                                               or null,
//                                 "properties": [<description>, ...]}, ...]}
//     attachment description:
//       {"title": <title>, "free_title": <title>, "joint_title": <title>,
//        "held_at_title": <title>, "joint_kinds": [<element kind>, ...],
//        "body_end_property": "<name>", "other_end_property": "<name>",
//        "free_only_properties": ["<name>", ...]}
//     Every element kind, in the order above, with the name people know it by
//     and its properties described in the order get_properties returns them.
//     editable says whether the kind's property can be edited at all.
//     attachment is null but for the rigid body, which states how it is
//     attached: free in the world, titled free_title, or by a joint of one of
//     joint_kinds, whose body_end_property (child_frame_name) names the body or
//     one of its fixed frames and whose other_end_property (parent_frame_name)
//     the frame it is held at. The joint a body hangs on is shown under
//     joint_title and the frame it is held at under held_at_title.
//     free_only_properties are the rigid body's properties only a free body
//     has: its station offset and placement offsets.
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
//   get_properties  {"subject": <subject>}
//     result: {"properties": [<description> with "value": <value>, ...],
//              "attachment": <attachment> or null}
//     attachment: {"summary": <title>, "note": <title> or null,
//                  "joint": <element> or null, "held_at": <element> or null,
//                  "can_become_free": <bool>, "can_take_joint": <bool>,
//                  "frames_on_body": [<element>, ...],
//                  "frames_it_can_be_held_at": [<element>, ...]}
//     editable says whether this subject's property can be edited now. It is
//     the property's editable, except that a station offset or a placement
//     offset whose value is null is not editable; a reference that names none
//     is null and editable, and so is the gravity of a project that states
//     none. Refused for wheel, rail and contact_computation while the project
//     has no contact record.
//     attachment is null but for a rigid body, whose attachment is read from
//     the records alone, whatever check would say of them. A joint relates two
//     bodies when its two ends name frames on two different bodies of the
//     project, a body's name being its own frame and a fixed frame being on
//     the body it names; a joint with an end naming no such frame, or with both
//     ends on one body, relates nothing. Bodies related directly or through
//     others form a group: if its joints close a loop, every body of it is
//     conflicting; otherwise, with exactly one free body, that body is free and
//     every other one hangs on the joint on its way to it, whichever end of the
//     record is on it; with none, every body of it is unattached; with more
//     than one, every body of it is conflicting. summary says in one short
//     line how the body is held, naming no element; note what is the matter or
//     why something cannot change; joint is the joint it hangs on, and held_at
//     the frame it is held at, at that joint's other end: a rigid body or a
//     fixed frame. A free body can take a joint; a hanging or unattached body
//     can become free and can take a joint; a conflicting one neither:
//     removing one of its joints repairs it. frames_on_body is what a new
//     joint's end on the body may name, the body and then its fixed frames,
//     and frames_it_can_be_held_at what its other end may name: every frame on
//     a body this body does not reach through joints once the joint it hangs
//     on is taken away, body by body, each before its fixed frames.
//
//   set_property  {"subject": <subject>, "property": "<name>", "value": <value>}
//     result: {"subject": <subject>}, the subject after the edit: an element
//     whose name differs from the one given after a rename.
//     value is required, null included: an explicit null makes a reference
//     that may name none name none. The value is read as the property's value
//     kind, and a value of another shape is refused. The edit is then refused
//     unless the element exists or the project holds the settings, which it
//     holds of wheel, rail and contact_computation only with a contact record,
//     the property is editable, every number in the value is finite, a
//     station offset or placement offset has the record holding it, a
//     reference names an element of one of its referenced_kinds or, where
//     optional, none, a token is one of its choices, and a new name is not
//     empty and taken by no other element of the element's name space.
//     Renaming changes every reference to the element and its group
//     memberships, and for a rigid body its station layout entry, the layout's
//     wheel-pair station list and its placement; the name its shapes are
//     stated under stays. References the project
//     already holds are not judged here, physical plausibility and the
//     layout's own consistency neither; check judges them.
//
//   add_element  {"kind": <element kind>, "name": "<name>",
//                 "properties": {"<property name>": <value>, ...},
//                 "attachment": <attachment choice>}
//     result: {"element": <element>}
//     Appends a new element to its kind's records: the record's default value
//     with this name and the given properties, each read and checked as
//     set_property reads and checks it. properties is required and may be
//     empty: it may leave any property out, except a reference that cannot
//     name none, and cannot hold the element's own name or a property that is
//     not editable. The name is refused as a rename's would be. A contact kind
//     needs the project to have a contact record. A joint kind is refused: a
//     joint is created only as a rigid body's attachment. attachment is
//     required for a rigid body and refused for every other kind; a new rigid
//     body is attached as set_attachment attaches one, in the same edit, a
//     joint's end on the body naming the new body and left out of the joint's
//     properties. The properties only a free body has are set once the body
//     is attached, so a body attached by a joint is refused them. A new rigid
//     body has no shapes.
//
//   set_attachment  {"body": <element>, "attachment": <attachment choice>}
//     result: {"joint": <element> or null}, the joint the body hangs on
//     afterwards, null when it moves freely.
//     attachment choice: {"free": true}
//                        or {"joint": {"kind": <joint kind>, "name": "<name>",
//                                      "properties": {"<property name>":
//                                                     <value>, ...}}}
//     Changes how a rigid body is attached, as its attachment allows. Becoming
//     free states that the body moves freely and gives it a station layout
//     entry and a placement where it has none, at zero offsets and the
//     identity rotation, so that it is drawn there until it is placed; the
//     joint it hung on is removed, and while another element names that joint
//     the change is refused, naming them: remove_element takes such a joint
//     with what names it. Taking a joint creates one of the chosen kind with
//     the name and properties given, read and checked as add_element reads and
//     checks an element's, both ends included: child_frame_name names the body
//     or one of its fixed frames, parent_frame_name a frame of
//     frames_it_can_be_held_at. A free body then no longer moves freely and
//     loses its station layout entry and placement; a body hanging on a joint
//     has that joint removed first, so that the new one may take its name, and
//     every reference to it names the new one, the change being refused,
//     naming the element, when a reference cannot name the new kind. Refused
//     for a conflicting body. A refusal about the joint names the joint as its
//     subject, with the property at fault; any other names the body.
//
//   remove_element  {"element": <element>}
//     result: {"removed_elements": [<element>, ...]}
//     Removes the element and every element holding a reference to one
//     removed, transitively, with their station layout entries, placements,
//     shape names and group memberships; groups stay. removed_elements lists
//     everything removed, the element included, in outline order.
//
//   get_removal_consequences  {"element": <element>}
//     result: {"removed_elements": [<element>, ...]}
//     What remove_element would remove, or its refusal. Changes nothing.
//
//   set_property, add_element, set_attachment and remove_element are each one
//   edit: one undo step and one revision. After any edit, undo, redo or
//   save_vehicle_project_as an element may be gone, renamed or under another
//   owner, so a client reads the model tree and the outline again.
//
//   undo, redo
//     result: {}
//     Refused when there is nothing to undo or to redo.
//
//   save_vehicle_project
//     result: {}
//     Writes the project, whatever state it is in, to the session's project
//     file, replacing it whole or not at all, and leaves nothing unsaved.
//     Refused when the session has no project file. Changes no revision.
//
//   save_vehicle_project_as  {"directory": "<path>", "name": "<name>"}
//     result: {}
//     Writes the project, its vehicle definition's vehicle_name and
//     mechanical_definition_identifier set to <name>, as the file
//     <name>.vehicle_project.json of the directory, replacing a project of that
//     name whole or not at all. Only once it is written do the two names
//     become those of the session's project and of every project its undo and
//     redo history holds: they are the project's identity, not an edit, so
//     can_undo and can_redo stay as they were and no undo or redo brings an
//     earlier identity back. The revision increases by one when the names
//     change. The session is then associated with that file and nothing is
//     unsaved. Nothing else is renamed or rewritten. A project name is not
//     empty and holds only A-Z, a-z, 0-9, '.', '_' and '-'. A refused save
//     changes nothing and leaves no file half written.
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
//   get_visual_definition
//     result: {"visual_definition": <object> or null}
//     The visual definition the project's bodies are drawn with, as the JSON
//     object it is; null when the project has none.
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
//      "subject": {"element": {"kind": "rigid_body", "name": "carbody"}},
//      "property": "name", "value": ""}
//   < {"id": 7, "ok": false,
//      "error": {"subject": {"element": {"kind": "rigid_body",
//                                        "name": "carbody"}},
//                "property": "name",
//                "message": "an element's name cannot be empty"}}

#include <Eigen/Core>
#include <algorithm>
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
#include "orvd/vehicle_project/read_visual_definition_document_from_file.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"
#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"
#include "orvd/vehicle_project/vehicle_project_free_body_placements.h"
#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_model_tree_by_domains.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"
#include "orvd/vehicle_project_editing/vehicle_project_property_subjects.h"
#include "orvd/vehicle_project_editing/vehicle_project_rigid_body_attachments.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_edit_refusal.h"
#include "orvd/vehicle_project_editing/vehicle_project_subject_properties.h"
#include "orvd/vehicle_project_editing/vehicle_project_titles_in_english_and_chinese.h"

namespace {

using Json = nlohmann::ordered_json;
using orvd::vehicle_project::ElementGroup;
using orvd::vehicle_project::ElementKind;
using orvd::vehicle_project::ElementReference;
using orvd::vehicle_project::VehicleProject;
using orvd::vehicle_project::VehicleProjectFileLocation;
using orvd::vehicle_project_editing::CurveRowField;
using orvd::vehicle_project_editing::ElementOwnName;
using orvd::vehicle_project_editing::FreeInWorldAttachment;
using orvd::vehicle_project_editing::JointAttachment;
using orvd::vehicle_project_editing::JointLocationDrawing;
using orvd::vehicle_project_editing::MechanicalCheckReport;
using orvd::vehicle_project_editing::ModelTreeCreationOffer;
using orvd::vehicle_project_editing::ModelTreeNode;
using orvd::vehicle_project_editing::ProjectSettings;
using orvd::vehicle_project_editing::PropertyDescription;
using orvd::vehicle_project_editing::PropertyReading;
using orvd::vehicle_project_editing::PropertySubject;
using orvd::vehicle_project_editing::PropertyText;
using orvd::vehicle_project_editing::PropertyToken;
using orvd::vehicle_project_editing::PropertyValue;
using orvd::vehicle_project_editing::PropertyValueKind;
using orvd::vehicle_project_editing::PropertyValuesByName;
using orvd::vehicle_project_editing::ReferencedElementName;
using orvd::vehicle_project_editing::RigidBodyAttachment;
using orvd::vehicle_project_editing::RigidBodyAttachmentChoice;
using orvd::vehicle_project_editing::RigidBodyAttachmentDescription;
using orvd::vehicle_project_editing::SaturatedPiecewiseLinearDamperCurve;
using orvd::vehicle_project_editing::SubjectEditRefusal;
using orvd::vehicle_project_editing::Title;
using orvd::vehicle_project_editing::VehicleProjectEditingSession;
using orvd::vehicle_project_editing::VehicleProjectPreview;

// The protocol writes an element kind as its word
// (vehicle_project_element_kind_records.h), the word a project file uses too.
std::string NameOfElementKind(ElementKind kind) {
    return std::string(orvd::vehicle_project::ElementKindWord(kind));
}

ElementKind ElementKindNamed(std::string_view word) {
    const std::optional<ElementKind> kind = orvd::vehicle_project::FindElementKindWrittenAs(word);
    if (!kind.has_value()) {
        throw std::invalid_argument("'" + std::string(word) + "' is not an element kind");
    }
    return *kind;
}

std::string NameOfValueKind(PropertyValueKind kind) {
    switch (kind) {
        case PropertyValueKind::kNumber:
            return "number";
        case PropertyValueKind::kVector3:
            return "vector3";
        case PropertyValueKind::kRotation:
            return "rotation";
        case PropertyValueKind::kText:
            return "text";
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

// The protocol writes settings as their word (vehicle_project_property_subjects.h).
ProjectSettings ProjectSettingsNamed(std::string_view word) {
    const std::optional<ProjectSettings> settings =
        orvd::vehicle_project_editing::FindProjectSettingsWrittenAs(word);
    if (!settings.has_value()) {
        throw std::invalid_argument("'" + std::string(word) + "' is not a settings word");
    }
    return *settings;
}

Json EncodeSubject(const PropertySubject& subject) {
    if (const auto* const element = std::get_if<ElementReference>(&subject)) {
        return Json{{"element", EncodeElement(*element)}};
    }
    return Json{{"settings", std::string(orvd::vehicle_project_editing::ProjectSettingsWord(
                                 std::get<ProjectSettings>(subject)))}};
}

Json EncodeTitle(const Title& title) {
    return Json{{"en", title.en}, {"zh", title.zh}};
}

Json EncodeOptionalTitle(const std::optional<Title>& title) {
    return title.has_value() ? EncodeTitle(*title) : Json(nullptr);
}

// One encoding per type a property value holds.
Json EncodeValue(double number) {
    return number;
}
Json EncodeValue(const Eigen::Vector3d& vector) {
    return EncodeVector(vector);
}
Json EncodeValue(const Eigen::Matrix3d& rotation) {
    return EncodeRotation(rotation);
}
Json EncodeValue(const PropertyText& text) {
    return text.text;
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
                         const PropertyDescription& description) {
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
    const Json& value, const PropertyDescription& description) {
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
        case PropertyValueKind::kText:
            if (value.is_string()) {
                return PropertyText{value.get<std::string>()};
            }
            return std::unexpected(property + " takes a string");
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

// An element when the subject names one, otherwise settings.
PropertySubject DecodeSubject(const Json& subject) {
    if (subject.contains("element")) {
        return DecodeElement(subject.at("element"));
    }
    return ProjectSettingsNamed(subject.at("settings").get<std::string>());
}

Json EncodeDescription(const PropertyDescription& description) {
    Json encoded{{"name", std::string(description.name)},
                 {"value_kind", NameOfValueKind(description.value_kind)},
                 {"editable", description.editable},
                 {"section", EncodeOptionalTitle(description.section)}};
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

Json EncodeAttachmentDescription(const RigidBodyAttachmentDescription& description) {
    Json joint_kinds = Json::array();
    for (const ElementKind kind : description.joint_kinds) {
        joint_kinds.push_back(NameOfElementKind(kind));
    }
    Json free_only_properties = Json::array();
    for (const std::string_view property_name : description.free_only_properties) {
        free_only_properties.push_back(std::string(property_name));
    }
    return Json{{"title", EncodeTitle(description.title)},
                {"free_title", EncodeTitle(description.free_title)},
                {"joint_title", EncodeTitle(description.joint_title)},
                {"held_at_title", EncodeTitle(description.held_at_title)},
                {"joint_kinds", std::move(joint_kinds)},
                {"body_end_property", std::string(description.body_end_property)},
                {"other_end_property", std::string(description.other_end_property)},
                {"free_only_properties", std::move(free_only_properties)}};
}

Json DescribeElementKinds() {
    Json kinds = Json::array();
    for (const ElementKind kind : orvd::vehicle_project::kElementKinds) {
        Json properties = Json::array();
        for (const PropertyDescription& description :
             orvd::vehicle_project_editing::DescribeElementProperties(kind)) {
            properties.push_back(EncodeDescription(description));
        }
        kinds.push_back(Json{
            {"kind", NameOfElementKind(kind)},
            {"title", EncodeTitle(orvd::vehicle_project_editing::ElementKindTitle(kind))},
            {"attachment", kind == ElementKind::kRigidBody
                               ? EncodeAttachmentDescription(
                                     orvd::vehicle_project_editing::DescribeRigidBodyAttachment())
                               : Json(nullptr)},
            {"properties", std::move(properties)}});
    }
    return Json{{"element_kinds", std::move(kinds)}};
}

Json EncodeAttachment(const RigidBodyAttachment& attachment) {
    return Json{
        {"summary", EncodeTitle(attachment.summary)},
        {"note", EncodeOptionalTitle(attachment.note)},
        {"joint", attachment.joint.has_value() ? EncodeElement(*attachment.joint) : Json(nullptr)},
        {"held_at",
         attachment.held_at.has_value() ? EncodeElement(*attachment.held_at) : Json(nullptr)},
        {"can_become_free", attachment.can_become_free},
        {"can_take_joint", attachment.can_take_joint},
        {"frames_on_body", EncodeElements(attachment.frames_on_body)},
        {"frames_it_can_be_held_at", EncodeElements(attachment.frames_it_can_be_held_at)}};
}

Json EncodeOutline(const VehicleProject& project) {
    Json elements = Json::array();
    for (const ElementKind kind : orvd::vehicle_project::kElementKinds) {
        for (const std::string& name :
             orvd::vehicle_project_editing::CollectElementNames(project, kind)) {
            elements.push_back(EncodeElement({kind, name}));
        }
    }
    return Json{
        {"vehicle_name", project.vehicle_definition.vehicle_name},
        {"elements", std::move(elements)},
        {"element_groups", EncodeGroups(project.element_groups)},
        {"unplaced_free_bodies", orvd::vehicle_project::CollectUnplacedFreeBodyNames(project)},
        {"contact_record_present", project.contact_record.has_value()}};
}

Json EncodeCreationOffer(const ModelTreeCreationOffer& offer) {
    Json fixed_properties = Json::object();
    for (const auto& [property_name, value] : offer.fixed_properties) {
        fixed_properties[property_name] = EncodePropertyValue(
            value,
            *orvd::vehicle_project_editing::DescribeElementProperty(offer.kind, property_name)
                 .value());
    }
    return Json{{"kind", NameOfElementKind(offer.kind)},
                {"fixed_properties", std::move(fixed_properties)}};
}

Json EncodeModelTreeNode(const ModelTreeNode& node) {
    Json creates = Json::array();
    for (const ModelTreeCreationOffer& offer : node.creates) {
        creates.push_back(EncodeCreationOffer(offer));
    }
    Json children = Json::array();
    for (const ModelTreeNode& child : node.children) {
        children.push_back(EncodeModelTreeNode(child));
    }
    return Json{
        {"key", node.key},
        {"title", EncodeOptionalTitle(node.title)},
        {"subject", node.subject.has_value() ? EncodeSubject(*node.subject) : Json(nullptr)},
        {"provided", node.provided},
        {"note", EncodeOptionalTitle(node.note)},
        {"creates", std::move(creates)},
        {"attaches", EncodeElements(node.attaches)},
        {"children", std::move(children)}};
}

Json EncodeModelTree(const VehicleProject& project) {
    Json nodes = Json::array();
    for (const ModelTreeNode& node : orvd::vehicle_project_editing::DescribeModelTree(project)) {
        nodes.push_back(EncodeModelTreeNode(node));
    }
    return Json{{"nodes", std::move(nodes)}};
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
                {"has_unsaved_changes", session.has_unsaved_changes()},
                {"result", std::move(result)}};
}

// The answer of a command accepted while nothing is open, which has no
// session to describe.
Json AcceptedWithNothingOpen(const Json& id, Json result) {
    return Json{{"id", id},
                {"ok", true},
                {"revision", nullptr},
                {"can_undo", nullptr},
                {"can_redo", nullptr},
                {"has_unsaved_changes", nullptr},
                {"result", std::move(result)}};
}

Json AcceptedWhetherOrNotOpen(const Json& id,
                              const std::optional<VehicleProjectEditingSession>& session,
                              Json result) {
    return session.has_value() ? Accepted(id, *session, std::move(result))
                               : AcceptedWithNothingOpen(id, std::move(result));
}

Json Refused(const Json& id, Json subject, Json property, std::string_view message) {
    return Json{{"id", id},
                {"ok", false},
                {"error", Json{{"subject", std::move(subject)},
                               {"property", std::move(property)},
                               {"message", std::string(message)}}}};
}

Json RefusedEdit(const Json& id, const SubjectEditRefusal& refusal) {
    return Refused(id, EncodeSubject(refusal.subject),
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
    project.gravitational_acceleration_meters_per_second_squared =
        startup_state.gravitational_acceleration_meters_per_second_squared;
    project.contact_record = orvd::configuration::LoadVehicleContactRecordFromJsonFile(
        request.at("contact_record_path").get<std::string>());
    if (const Json& visual_definition_path = request.at("visual_definition_path");
        !visual_definition_path.is_null()) {
        project.visual_definition_document =
            orvd::vehicle_project::ReadVisualDefinitionDocumentFromFile(
                visual_definition_path.get<std::string>());
    }
    return VehicleProjectEditingSession(std::move(project));
}

VehicleProjectFileLocation DecodeVehicleProjectFileLocation(const Json& request) {
    return {.directory = request.at("directory").get<std::string>(),
            .name = request.at("name").get<std::string>()};
}

VehicleProjectEditingSession OpenVehicleProject(const Json& request) {
    const VehicleProjectFileLocation location = DecodeVehicleProjectFileLocation(request);
    return VehicleProjectEditingSession(orvd::vehicle_project::ReadVehicleProjectFile(location),
                                        location);
}

Json GetSession(const Json& id, const std::optional<VehicleProjectEditingSession>& session) {
    if (!session.has_value()) {
        return AcceptedWithNothingOpen(id, Json{{"open", false}});
    }
    const std::optional<VehicleProjectFileLocation>& file = session->associated_file();
    return Accepted(
        id, *session,
        Json{{"open", true},
             {"vehicle_project_file",
              file.has_value() ? Json{{"directory", file->directory.string()}, {"name", file->name}}
                               : Json(nullptr)}});
}

Json ListVehicleProjects(const Json& id, const Json& request,
                         const std::optional<VehicleProjectEditingSession>& session) {
    return AcceptedWhetherOrNotOpen(
        id, session,
        Json{{"vehicle_projects", orvd::vehicle_project::ListVehicleProjectNames(
                                      request.at("directory").get<std::string>())}});
}

Json EncodeVisualDefinition(const VehicleProject& project) {
    return Json{{"visual_definition", project.visual_definition_document.empty()
                                          ? Json(nullptr)
                                          : Json::parse(project.visual_definition_document)}};
}

Json GetProperties(const Json& id, const Json& request,
                   const VehicleProjectEditingSession& session) {
    const PropertySubject subject = DecodeSubject(request.at("subject"));
    std::vector<PropertyReading> readings;
    try {
        readings = orvd::vehicle_project_editing::ReadSubjectProperties(session.project(), subject);
    } catch (const std::invalid_argument& refusal) {
        return Refused(id, EncodeSubject(subject), nullptr, refusal.what());
    }
    Json properties = Json::array();
    for (const PropertyReading& reading : readings) {
        Json property = EncodeDescription(reading.description);
        property["value"] = EncodePropertyValue(reading.value, reading.description);
        properties.push_back(std::move(property));
    }
    const auto* const element = std::get_if<ElementReference>(&subject);
    return Accepted(
        id, session,
        Json{{"properties", std::move(properties)},
             {"attachment",
              element != nullptr && element->kind == ElementKind::kRigidBody
                  ? EncodeAttachment(orvd::vehicle_project_editing::ReadRigidBodyAttachment(
                        session.project(), element->name))
                  : Json(nullptr)}});
}

// The value given for the property `described`, read as the value kind the
// property is described with, or why the property or its value is not one.
std::expected<PropertyValue, std::string> DecodeGivenValue(
    const std::expected<const PropertyDescription*, std::string>& described, const Json& value) {
    if (!described.has_value()) {
        return std::unexpected(described.error());
    }
    return DecodePropertyValue(value, **described);
}

Json SetProperty(const Json& id, const Json& request, VehicleProjectEditingSession& session) {
    const PropertySubject subject = DecodeSubject(request.at("subject"));
    const std::string property_name = request.at("property").get<std::string>();
    const std::expected<PropertyValue, std::string> value = DecodeGivenValue(
        orvd::vehicle_project_editing::DescribeSubjectProperty(subject, property_name),
        request.at("value"));
    if (!value.has_value()) {
        return Refused(id, EncodeSubject(subject), property_name, value.error());
    }
    const auto edited = session.SetSubjectProperty(subject, property_name, *value);
    if (!edited.has_value()) {
        return RefusedEdit(id, edited.error());
    }
    return Accepted(id, session, Json{{"subject", EncodeSubject(*edited)}});
}

// The properties given for `element`, each read as the value kind its kind's
// property is described with, or the refusal of the first that cannot be.
std::expected<PropertyValuesByName, SubjectEditRefusal> DecodeGivenProperties(
    const ElementReference& element, const Json& given) {
    if (!given.is_object()) {
        return std::unexpected(
            SubjectEditRefusal{element, std::nullopt,
                               "properties must be an object of property values by property name"});
    }
    PropertyValuesByName properties;
    for (const auto& [property_name, given_value] : given.items()) {
        std::expected<PropertyValue, std::string> value = DecodeGivenValue(
            orvd::vehicle_project_editing::DescribeElementProperty(element.kind, property_name),
            given_value);
        if (!value.has_value()) {
            return std::unexpected(SubjectEditRefusal{element, property_name, value.error()});
        }
        properties.emplace(property_name, std::move(*value));
    }
    return properties;
}

// How `body` is to be attached: {"free": true}, or a joint with its kind, name
// and properties, read as add_element reads an element's; or the refusal of
// what cannot be read, the joint being the subject of one about it.
std::expected<RigidBodyAttachmentChoice, SubjectEditRefusal> DecodeAttachmentChoice(
    const ElementReference& body, const Json& attachment) {
    if (attachment.contains("free")) {
        if (attachment.at("free") != true) {
            return std::unexpected(SubjectEditRefusal{
                body, std::nullopt, "a body attached free in the world states \"free\": true"});
        }
        return FreeInWorldAttachment{};
    }
    const Json& joint = attachment.at("joint");
    const ElementReference joint_element{ElementKindNamed(joint.at("kind").get<std::string>()),
                                         joint.at("name").get<std::string>()};
    std::expected<PropertyValuesByName, SubjectEditRefusal> properties =
        DecodeGivenProperties(joint_element, joint.at("properties"));
    if (!properties.has_value()) {
        return std::unexpected(properties.error());
    }
    return JointAttachment{.kind = joint_element.kind,
                           .name = joint_element.name,
                           .properties = *std::move(properties)};
}

Json AddElement(const Json& id, const Json& request, VehicleProjectEditingSession& session) {
    const ElementReference element{ElementKindNamed(request.at("kind").get<std::string>()),
                                   request.at("name").get<std::string>()};
    const std::expected<PropertyValuesByName, SubjectEditRefusal> properties =
        DecodeGivenProperties(element, request.at("properties"));
    if (!properties.has_value()) {
        return RefusedEdit(id, properties.error());
    }
    if (element.kind != ElementKind::kRigidBody) {
        if (request.contains("attachment")) {
            return Refused(id, EncodeSubject(element), nullptr,
                           "only a rigid body is created with an attachment");
        }
        const auto created = session.AddElement(element.kind, element.name, *properties);
        if (!created.has_value()) {
            return RefusedEdit(id, created.error());
        }
        return Accepted(id, session, Json{{"element", EncodeElement(*created)}});
    }
    if (!request.contains("attachment")) {
        return Refused(id, EncodeSubject(element), nullptr,
                       "a rigid body is created with its attachment");
    }
    const std::expected<RigidBodyAttachmentChoice, SubjectEditRefusal> attachment =
        DecodeAttachmentChoice(element, request.at("attachment"));
    if (!attachment.has_value()) {
        return RefusedEdit(id, attachment.error());
    }
    const auto created = session.AddRigidBody(element.name, *properties, *attachment);
    if (!created.has_value()) {
        return RefusedEdit(id, created.error());
    }
    return Accepted(id, session, Json{{"element", EncodeElement(*created)}});
}

Json SetAttachment(const Json& id, const Json& request, VehicleProjectEditingSession& session) {
    const ElementReference body = DecodeElement(request.at("body"));
    if (body.kind != ElementKind::kRigidBody) {
        return Refused(id, EncodeSubject(body), nullptr, "only a rigid body is attached");
    }
    const std::expected<RigidBodyAttachmentChoice, SubjectEditRefusal> attachment =
        DecodeAttachmentChoice(body, request.at("attachment"));
    if (!attachment.has_value()) {
        return RefusedEdit(id, attachment.error());
    }
    const auto changed = session.SetRigidBodyAttachment(body.name, *attachment);
    if (!changed.has_value()) {
        return RefusedEdit(id, changed.error());
    }
    return Accepted(
        id, session,
        Json{{"joint", changed->has_value() ? EncodeElement(**changed) : Json(nullptr)}});
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
        if (command == "get_session") {
            return GetSession(id, session);
        }
        if (command == "list_vehicle_projects") {
            return ListVehicleProjects(id, request, session);
        }
        if (command == "open_vehicle") {
            session = OpenVehicle(request);
            return Accepted(id, *session, Json::object());
        }
        if (command == "open_vehicle_project") {
            session = OpenVehicleProject(request);
            return Accepted(id, *session, Json::object());
        }
        if (!session.has_value()) {
            throw std::invalid_argument(
                "nothing is open; send open_vehicle or open_vehicle_project first");
        }
        VehicleProjectEditingSession& open_session = *session;
        if (command == "describe_element_kinds") {
            return Accepted(id, open_session, DescribeElementKinds());
        }
        if (command == "get_model_tree") {
            return Accepted(id, open_session, EncodeModelTree(open_session.project()));
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
        if (command == "set_attachment") {
            return SetAttachment(id, request, open_session);
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
        if (command == "save_vehicle_project") {
            open_session.SaveVehicleProject();
            return Accepted(id, open_session, Json::object());
        }
        if (command == "save_vehicle_project_as") {
            open_session.SaveVehicleProjectAs(DecodeVehicleProjectFileLocation(request));
            return Accepted(id, open_session, Json::object());
        }
        if (command == "get_visual_definition") {
            return Accepted(id, open_session, EncodeVisualDefinition(open_session.project()));
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
