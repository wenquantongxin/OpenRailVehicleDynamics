"""Drive the vehicle modelling service through its standard input and output.

Opens GZ18 with its reference contact record, lists the element kinds, reads,
edits, creates, renames, removes, checks, previews, undoes, redoes and refuses,
asserting the protocol's structure and its revision behaviour rather than any
numeric result. Missing inputs keep the open session; mismatched contact
declarations open and are reported by checking. Then saves GZ18 as a project,
edited and left failing its check, into a directory of projects, lists it,
and opens each project again after another vehicle to find everything an
interface reads of it as it was saved; follows the session and its unsaved
changes throughout, the names Save As gives surviving undo and redo; and finds
the reference vehicles' files unchanged.
"""
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

ELEMENT_KINDS = [
    "rigid_body", "fixed_frame", "revolute_joint", "ball_rpy_joint", "weld_joint",
    "translational_spring_damper", "roll_spring_damper_couple", "series_spring_viscous_damper",
    "point_to_point_series_spring_bilinear_damper", "saturated_piecewise_linear_damper",
    "reference_frame_linear_bushing", "point_to_point_linear_spring_damper",
    "point_to_point_hysteretic_spring", "contact_carrier", "contact_interface",
]
CONTACT_ELEMENT_KINDS = ELEMENT_KINDS[13:]
FORCE_ELEMENT_KINDS = set(ELEMENT_KINDS[5:13])
# The preview lists, each with the element kinds its drawings show.
DRAWING_LIST_KINDS = {
    "rigid_bodies": {"rigid_body"}, "fixed_frames": {"fixed_frame"},
    "revolute_joints": {"revolute_joint"}, "ball_rpy_joints": {"ball_rpy_joint"},
    "weld_joints": {"weld_joint"}, "force_elements": FORCE_ELEMENT_KINDS,
    "wheels": {"contact_interface"},
}
DRAWN_ELEMENT_KINDS = set().union(*DRAWING_LIST_KINDS.values())
# The envelope fields that describe the session after a command; all null
# while nothing is open.
SESSION_FIELDS = ("revision", "can_undo", "can_redo", "has_unsaved_changes")
NOTHING_OPEN = (None, None, None, None)
CURVE = "saturated_piecewise_linear_damper_curve"
# The description fields each value kind adds to name, value_kind and editable.
DESCRIPTION_FIELDS_OF_VALUE_KIND = {
    "number": set(), "vector3": set(), "boolean": set(), "rotation": set(),
    "element_name": set(), "element_reference": {"referenced_kinds", "optional"},
    "token": {"choices"}, CURVE: {"row_fields"},
}


class Service:
    def __init__(self, executable):
        self.process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                        text=True, bufsize=1)
        self.last_id = 0

    def request(self, command, **arguments):
        self.last_id += 1
        self.process.stdin.write(json.dumps({"id": self.last_id, "command": command,
                                             **arguments}) + "\n")
        self.process.stdin.flush()
        response = json.loads(self.process.stdout.readline())
        assert response["id"] == self.last_id, response
        if response["ok"]:
            assert set(response) == {"id", "ok", *SESSION_FIELDS, "result"}, response
        else:
            assert set(response) == {"id", "ok", "error"}, response
            assert set(response["error"]) == {"element", "property", "message"}, response
            assert response["error"]["message"], response
        return response

    def accepted(self, command, **arguments):
        response = self.request(command, **arguments)
        assert response["ok"], response
        return response

    def refused(self, command, **arguments):
        response = self.request(command, **arguments)
        assert not response["ok"], response
        return response["error"]

    def finish(self):
        self.process.stdin.close()
        assert self.process.stdout.read() == "", "the service wrote more than its responses"
        assert self.process.wait(timeout=60) == 0, self.process.stderr.read()


def session_of(response):
    return tuple(response[field] for field in SESSION_FIELDS)


def is_vector(value):
    return isinstance(value, list) and len(value) == 3 and all(
        isinstance(component, (int, float)) for component in value)


def is_rotation(value):
    return isinstance(value, list) and len(value) == 3 and all(is_vector(row) for row in value)


def require_description(description):
    """A property description has the fields its value kind gives it: a
    reference the kinds it may name and whether it may name none, a token its
    choices, a curve its row fields. Every property is editable but a
    rotation."""
    value_kind = description["value_kind"]
    assert value_kind in DESCRIPTION_FIELDS_OF_VALUE_KIND, description
    assert set(description) - {"value"} == ({"name", "value_kind", "editable"} |
                                             DESCRIPTION_FIELDS_OF_VALUE_KIND[value_kind]), description
    assert description["editable"] == (value_kind != "rotation"), description
    if value_kind == "element_reference":
        assert description["referenced_kinds"], description
        assert all(kind in ELEMENT_KINDS for kind in description["referenced_kinds"]), description
        assert isinstance(description["optional"], bool), description
    if value_kind == "token":
        assert description["choices"] and all(
            isinstance(choice, str) for choice in description["choices"]), description
    if value_kind == CURVE:
        assert description["row_fields"] and all(
            isinstance(field, str) for field in description["row_fields"]), description


def require_drawings_name_outline_elements(preview, outline):
    """Every drawing has its shape and names an element of the outline of the
    kind its list shows, and every wheel a rigid body of the outline it turns
    with; no element is drawn twice; and the drawn elements with those not
    drawable are exactly the outline's elements of the kinds with drawings."""
    assert all(set(body) == {"element", "rotation_drawing_from_body",
                             "origin_in_drawing_frame_meters", "visual_definition_body_name"}
               and is_rotation(body["rotation_drawing_from_body"])
               and is_vector(body["origin_in_drawing_frame_meters"])
               and (body["visual_definition_body_name"] is None
                    or isinstance(body["visual_definition_body_name"], str))
               for body in preview["rigid_bodies"])
    assert all(set(frame) == {"element", "rotation_drawing_from_frame",
                              "origin_in_drawing_frame_meters"}
               and is_rotation(frame["rotation_drawing_from_frame"])
               and is_vector(frame["origin_in_drawing_frame_meters"])
               for frame in preview["fixed_frames"])
    assert all(set(joint) == {"element", "location_in_drawing_frame_meters",
                              "axis_in_drawing_frame"}
               and is_vector(joint["location_in_drawing_frame_meters"])
               and math.isclose(math.hypot(*joint["axis_in_drawing_frame"]), 1.0, rel_tol=1e-12)
               for joint in preview["revolute_joints"])
    assert all(set(joint) == {"element", "location_in_drawing_frame_meters"}
               and is_vector(joint["location_in_drawing_frame_meters"])
               for joint in preview["ball_rpy_joints"] + preview["weld_joints"])
    assert all(set(drawing) == {"element", "reference_end_in_drawing_frame_meters",
                                "opposite_end_in_drawing_frame_meters"}
               and is_vector(drawing["reference_end_in_drawing_frame_meters"])
               and is_vector(drawing["opposite_end_in_drawing_frame_meters"])
               for drawing in preview["force_elements"])
    assert all(set(wheel) == {"element", "wheel_body", "datum_in_drawing_frame_meters",
                              "outward_spin_axis_in_drawing_frame",
                              "nominal_rolling_radius_meters"}
               and wheel["wheel_body"]["kind"] == "rigid_body"
               and wheel["wheel_body"] in outline["elements"]
               and is_vector(wheel["datum_in_drawing_frame_meters"])
               and math.isclose(math.hypot(*wheel["outward_spin_axis_in_drawing_frame"]), 1.0,
                                rel_tol=1e-12)
               and isinstance(wheel["nominal_rolling_radius_meters"], (int, float))
               for wheel in preview["wheels"])
    outline_elements = {(element["kind"], element["name"]) for element in outline["elements"]}
    drawn = []
    for list_name, list_kinds in DRAWING_LIST_KINDS.items():
        for drawing in preview[list_name]:
            assert set(drawing["element"]) == {"kind", "name"}
            assert drawing["element"]["kind"] in list_kinds, (list_name, drawing["element"])
            drawn.append((drawing["element"]["kind"], drawing["element"]["name"]))
    assert set(drawn) <= outline_elements
    assert len(drawn) == len(set(drawn)), "an element is drawn twice"
    not_drawable = [(element["kind"], element["name"])
                    for element in preview["not_drawable_elements"]]
    assert sorted(drawn + not_drawable) == sorted(
        element for element in outline_elements if element[0] in DRAWN_ELEMENT_KINDS)


def everything_read_of(service):
    """What an interface reads of the open project: the outline, every
    element's properties, the check, the preview without the revision it was
    computed from, and the visual definition."""
    outline = service.accepted("get_outline")["result"]
    properties = [service.accepted("get_properties", element=element)["result"]["properties"]
                  for element in outline["elements"]]
    check = service.accepted("check")["result"]
    preview = service.accepted("get_preview")["result"]["preview"]
    if preview is not None:
        preview = {key: value for key, value in preview.items() if key != "project_revision"}
    visual_definition = service.accepted("get_visual_definition")["result"]["visual_definition"]
    return {"outline": outline, "properties": properties, "check": check, "preview": preview,
            "visual_definition": visual_definition}


def verify_saving_and_reopening(executable, gz18, irw, work_directory):
    """Save As, Save and opening a project through commands alone, and what the
    session and its unsaved changes say at each step."""
    projects = work_directory / "vehicle_projects"
    projects.mkdir()
    inputs = work_directory / "inputs"
    inputs.mkdir()
    service = Service(executable)

    # While nothing is open, only the session and the list of projects are
    # answered, with the session fields null.
    nothing_open = service.accepted("get_session")
    assert nothing_open["result"] == {"open": False} and session_of(nothing_open) == NOTHING_OPEN
    listed = service.accepted("list_vehicle_projects", directory=str(projects))
    assert listed["result"] == {"vehicle_projects": []} and session_of(listed) == NOTHING_OPEN
    for command in ("save_vehicle_project", "get_visual_definition"):
        service.refused(command)
    service.refused("list_vehicle_projects", directory=str(projects / "missing_directory"))

    opened = service.accepted("open_vehicle", **gz18)
    assert session_of(opened) == (0, False, False, False)
    assert service.accepted("get_session")["result"] == {"open": True,
                                                         "vehicle_project_file": None}
    visual_definition = service.accepted("get_visual_definition")["result"]["visual_definition"]
    assert visual_definition == json.loads(Path(gz18["visual_definition_path"]).read_text())

    # Opening needs a visual definition path, which may be null, and a file
    # holding a JSON object; a refused open keeps the open session.
    not_an_object = inputs / "visual_definition_array.json"
    not_an_object.write_text("[]\n")
    without_visual_definition = {key: value for key, value in gz18.items()
                                 if key != "visual_definition_path"}
    for arguments in (without_visual_definition,
                      {**gz18, "visual_definition_path": str(not_an_object)}):
        refusal = service.refused("open_vehicle", **arguments)
        assert refusal["element"] is None and refusal["property"] is None
        assert session_of(service.accepted("get_outline")) == (0, False, False, False)

    # A vehicle opened from its files has no project file to save to.
    service.refused("save_vehicle_project")
    suspension = {"kind": "translational_spring_damper",
                  "name": "front_leading_left_primary_suspension"}
    stiffness = next(prop["value"] for prop in service.accepted(
        "get_properties", element=suspension)["result"]["properties"]
        if prop["name"] == "stiffness_newtons_per_meter")

    def edit_stiffness(factor):
        return service.accepted("set_property", element=suspension,
                                property="stiffness_newtons_per_meter",
                                value=[component * factor for component in stiffness])

    assert session_of(edit_stiffness(1.5)) == (1, True, False, True)
    assert session_of(service.accepted("undo")) == (2, False, True, False)
    assert session_of(service.accepted("redo")) == (3, True, False, True)

    # A name that is not a project name is refused and writes nothing.
    refusal = service.refused("save_vehicle_project_as", directory=str(projects),
                              name="edited gz18")
    assert refusal["element"] is None and refusal["property"] is None
    assert session_of(service.accepted("get_outline")) == (3, True, False, True)
    assert list(projects.iterdir()) == []

    # Save As names the vehicle after the project in one revision without an
    # undo step, and the session is then the project file's, with nothing
    # unsaved.
    saved_as = service.accepted("save_vehicle_project_as", directory=str(projects),
                                name="edited_gz18")
    assert session_of(saved_as) == (4, True, False, False) and saved_as["result"] == {}
    assert service.accepted("get_outline")["result"]["vehicle_name"] == "edited_gz18"
    project_file = {"directory": str(projects), "name": "edited_gz18"}
    assert service.accepted("get_session")["result"] == {"open": True,
                                                         "vehicle_project_file": project_file}
    assert service.accepted("list_vehicle_projects", directory=str(projects))["result"] == {
        "vehicle_projects": ["edited_gz18"]}

    # The names are the project's identity, not an edit: undoing the edit made
    # before Save As keeps them, and Save writes them again.
    assert session_of(service.accepted("undo")) == (5, False, True, True)
    assert service.accepted("get_outline")["result"]["vehicle_name"] == "edited_gz18"
    assert session_of(service.accepted("save_vehicle_project")) == (5, False, True, False)
    written = json.loads((projects / "edited_gz18.vehicle_project.json").read_text())
    assert (written["vehicle_definition"]["vehicle_name"],
            written["vehicle_definition"]["mechanical_definition_identifier"]) == (
        "edited_gz18", "edited_gz18")
    assert session_of(service.accepted("redo")) == (6, True, False, True)
    assert service.accepted("get_outline")["result"]["vehicle_name"] == "edited_gz18"
    # Under the names the vehicle has, Save As adds no revision.
    resaved = service.accepted("save_vehicle_project_as", directory=str(projects),
                               name="edited_gz18")
    assert session_of(resaved) == (6, True, False, False)

    # Save, edit, undo leaves nothing unsaved; Save changes no revision.
    assert session_of(edit_stiffness(2.0)) == (7, True, False, True)
    assert session_of(service.accepted("undo")) == (8, True, True, False)
    assert session_of(service.accepted("redo")) == (9, True, False, True)
    saved = service.accepted("save_vehicle_project")
    assert session_of(saved) == (9, True, False, False) and saved["result"] == {}
    as_saved = everything_read_of(service)
    assert as_saved["check"]["passed"] and as_saved["preview"] is not None

    # Another vehicle, opened without a visual definition, then the project
    # again: everything as it was saved, in a new session of the file.
    other = service.accepted("open_vehicle", **{**irw, "visual_definition_path": None})
    assert session_of(other) == (0, False, False, False)
    assert service.accepted("get_session")["result"] == {"open": True,
                                                         "vehicle_project_file": None}
    assert service.accepted("get_visual_definition")["result"] == {"visual_definition": None}
    service.refused("open_vehicle_project", directory=str(projects), name="absent_project")
    assert service.accepted("get_outline")["result"]["vehicle_name"] == "IRW"
    reopened = service.accepted("open_vehicle_project", **project_file)
    assert session_of(reopened) == (0, False, False, False) and reopened["result"] == {}
    assert service.accepted("get_session")["result"] == {"open": True,
                                                         "vehicle_project_file": project_file}
    assert everything_read_of(service) == as_saved

    # The same for a project left failing its check.
    service.accepted("open_vehicle", **gz18)
    service.accepted("add_element", kind="rigid_body", name="body_of_zero_mass", properties={})
    assert not service.accepted("check")["result"]["passed"]
    service.accepted("save_vehicle_project_as", directory=str(projects), name="unfinished_gz18")
    unfinished = everything_read_of(service)
    assert not unfinished["check"]["passed"] and unfinished["preview"] is None
    service.accepted("open_vehicle", **irw)
    service.accepted("open_vehicle_project", directory=str(projects), name="unfinished_gz18")
    assert everything_read_of(service) == unfinished

    # Every file the service wrote is a project file in the directory of
    # projects.
    assert service.accepted("list_vehicle_projects", directory=str(projects))["result"] == {
        "vehicle_projects": ["edited_gz18", "unfinished_gz18"]}
    assert sorted(path.name for path in projects.iterdir()) == [
        "edited_gz18.vehicle_project.json", "unfinished_gz18.vehicle_project.json"]
    service.finish()


def main():
    executable = Path(sys.argv[1]).resolve()
    vehicle_library = Path(sys.argv[2]).resolve() / "vehicle_library"
    work_directory = Path(sys.argv[3]).resolve()
    shutil.rmtree(work_directory, ignore_errors=True)
    work_directory.mkdir(parents=True)
    gz18, irw = ({"vehicle_definition_path": str(vehicle_library / name / "vehicle_definition.json"),
                  "startup_state_path": str(vehicle_library / name / "startup_states" /
                                            "moving_startup_60kmh.json"),
                  "contact_record_path": str(vehicle_library / name / "contact_record.json"),
                  "visual_definition_path": str(vehicle_library / name / "visualization" /
                                                "visual_definition.json")}
                 for name in ("gz18", "irw"))
    reference_files = {path: path.read_bytes() for name in ("gz18", "irw")
                       for path in sorted((vehicle_library / name).rglob("*")) if path.is_file()}
    service = Service(executable)

    refusal = service.refused("get_outline")
    assert refusal["element"] is None and refusal["property"] is None

    opened = service.accepted("open_vehicle", **gz18)
    assert (opened["revision"], opened["can_undo"], opened["can_redo"]) == (0, False, False)
    assert opened["result"] == {}

    kinds = service.accepted("describe_element_kinds")["result"]["element_kinds"]
    assert [kind["kind"] for kind in kinds] == ELEMENT_KINDS
    described = {kind["kind"]: kind["properties"] for kind in kinds}
    for properties in described.values():
        assert properties
        for description in properties:
            require_description(description)
        # Exactly one property is the element's own name.
        assert [prop["value_kind"] for prop in properties].count("element_name") == 1
    suspension_kind = {prop["name"]: prop for prop in described["translational_spring_damper"]}
    assert suspension_kind["stiffness_newtons_per_meter"]["value_kind"] == "vector3"
    assert suspension_kind["name"]["value_kind"] == "element_name"
    assert suspension_kind["reference_frame_name"]["referenced_kinds"] == ["rigid_body",
                                                                          "fixed_frame"]
    rigid_body_kind = {prop["name"]: prop for prop in described["rigid_body"]}
    station_offset = "mechanical_track_station_layout.station_offset_meters"
    assert rigid_body_kind[station_offset]["value_kind"] == "number"
    assert rigid_body_kind["moves_freely_in_world"]["value_kind"] == "boolean"
    interface_kind = {prop["name"]: prop for prop in described["contact_interface"]}
    assert interface_kind["independent_wheel_revolute_joint_name"]["optional"] is True
    assert interface_kind["carrier_name"]["optional"] is False
    assert interface_kind["side"]["value_kind"] == "token"
    damper_kind = {prop["name"]: prop for prop in described["saturated_piecewise_linear_damper"]}
    row_fields = damper_kind["curve"]["row_fields"]

    outline = service.accepted("get_outline")["result"]
    assert set(outline) == {"vehicle_name", "elements", "element_groups",
                            "unplaced_free_bodies", "contact_record_present"}
    assert outline["vehicle_name"] == "GZ18"
    assert outline["element_groups"] == [] and outline["unplaced_free_bodies"] == []
    assert outline["contact_record_present"] is True
    suspension = {"kind": "translational_spring_damper",
                  "name": "front_leading_left_primary_suspension"}
    assert suspension in outline["elements"]
    assert all(element["kind"] in ELEMENT_KINDS for element in outline["elements"])

    def readings_of(element):
        properties = service.accepted("get_properties", element=element)["result"]["properties"]
        # A reading carries its kind's description; it is editable when its
        # kind's property is and this element holds a value for it, a
        # reference naming none holding the value null.
        kind_properties = described[element["kind"]]
        assert [{key: value for key, value in prop.items() if key not in ("value", "editable")}
                for prop in properties] == [
            {key: value for key, value in prop.items() if key != "editable"}
            for prop in kind_properties]
        assert all(prop["editable"] == (kind_prop["editable"] and (
                       prop["value"] is not None or prop["value_kind"] == "element_reference"))
                   for prop, kind_prop in zip(properties, kind_properties)), properties
        return {prop["name"]: prop for prop in properties}

    def properties_of(element):
        return {name: reading["value"] for name, reading in readings_of(element).items()}

    def outline_elements():
        return service.accepted("get_outline")["result"]["elements"]

    # The reference contact record's carriers and interfaces are listed and
    # read, each naming itself by its element_name property.
    for kind, name_property in zip(CONTACT_ELEMENT_KINDS, ("carrier_name", "interface_name")):
        contact_elements = [element for element in outline["elements"] if element["kind"] == kind]
        assert contact_elements, kind
        for element in contact_elements:
            assert properties_of(element)[name_property] == element["name"]
    first_interface = next(element for element in outline["elements"]
                           if element["kind"] == "contact_interface")
    interface_readings = readings_of(first_interface)
    assert interface_readings["side"]["value"] in interface_readings["side"]["choices"]
    joint_reading = interface_readings["independent_wheel_revolute_joint_name"]
    assert joint_reading["value"] is None and joint_reading["editable"] is True

    # A jointed body holds no station offset and no placement: its kind can
    # edit those properties, this body cannot, and an edit of one is refused.
    jointed_body = {"kind": "rigid_body", "name": "front_leading_left_axlebox_carrier"}
    jointed_readings = readings_of(jointed_body)
    for name in (station_offset, "placement.lateral_offset_in_local_track_frame_meters",
                 "placement.vertical_offset_in_local_track_frame_meters"):
        assert rigid_body_kind[name]["editable"]
        assert jointed_readings[name]["value"] is None and not jointed_readings[name]["editable"]
    assert jointed_readings["mass_kilograms"]["editable"]
    refusal = service.refused("set_property", element=jointed_body, property=station_offset,
                              value=1.0)
    assert refusal["element"] == jointed_body and refusal["property"] == station_offset

    stiffness = properties_of(suspension)["stiffness_newtons_per_meter"]
    assert is_vector(stiffness)
    edited_stiffness = [component * 1.25 for component in stiffness]

    edited = service.accepted("set_property", element=suspension,
                              property="stiffness_newtons_per_meter", value=edited_stiffness)
    assert (edited["revision"], edited["can_undo"], edited["can_redo"]) == (1, True, False)
    assert edited["result"] == {"element": suspension}
    assert properties_of(suspension)["stiffness_newtons_per_meter"] == edited_stiffness

    # Missing records are refused without replacing the edited session.
    without_contact = {key: value for key, value in gz18.items() if key != "contact_record_path"}
    refused_opens = (
        without_contact,
        {**gz18, "contact_record_path": str(vehicle_library / "absent_contact_record.json")},
    )
    for arguments in refused_opens:
        refusal = service.refused("open_vehicle", **arguments)
        assert refusal["element"] is None and refusal["property"] is None
        kept = service.accepted("get_outline")
        assert kept["result"]["vehicle_name"] == "GZ18"
        assert (kept["revision"], kept["can_undo"], kept["can_redo"]) == (1, True, False)
    assert properties_of(suspension)["stiffness_newtons_per_meter"] == edited_stiffness

    checked = service.accepted("check")
    assert checked["revision"] == 1
    report = checked["result"]
    assert set(report) == {"passed", "refusals", "information", "coverage_statements"}
    assert report["passed"] and report["refusals"] == []
    assert report["information"] == []  # every body placed, the contact record present
    assert report["coverage_statements"] and all(report["coverage_statements"])

    preview = service.accepted("get_preview")["result"]["preview"]
    assert preview["project_revision"] == 1
    require_drawings_name_outline_elements(preview, outline)
    assert preview["not_drawable_elements"] == []  # every free body is placed
    # Every body of the opened vehicle has its shapes under its own name.
    assert all(body["visual_definition_body_name"] == body["element"]["name"]
               for body in preview["rigid_bodies"])

    undone = service.accepted("undo")
    assert (undone["revision"], undone["can_undo"], undone["can_redo"]) == (2, False, True)
    assert properties_of(suspension)["stiffness_newtons_per_meter"] == stiffness
    stale = service.accepted("get_preview")
    assert stale["revision"] == 2 and stale["result"]["preview"]["project_revision"] == 1

    refusal = service.refused("set_property", element=suspension, property="name", value="")
    assert refusal["element"] == suspension and refusal["property"] == "name"
    refusal = service.refused("set_property", element=suspension, property="name", value=1.0)
    assert refusal["element"] == suspension and refusal["property"] == "name"
    unchanged = service.accepted("get_outline")
    assert (unchanged["revision"], unchanged["can_undo"], unchanged["can_redo"]) == (2, False, True)
    assert properties_of(suspension)["name"] == suspension["name"]

    redone = service.accepted("redo")
    assert (redone["revision"], redone["can_undo"], redone["can_redo"]) == (3, True, False)
    assert properties_of(suspension)["stiffness_newtons_per_meter"] == edited_stiffness

    refusal = service.refused("rename_vehicle")
    assert refusal["element"] is None and refusal["property"] is None

    # Structural edits: a free body is created, a frame on it and a force
    # element from that frame to an existing one; the body is renamed, and
    # its frame follows; the removal's consequences are asked, then the body
    # is removed with what refers to it; undo brings it back.
    opened_outline = outline_elements()
    body = {"kind": "rigid_body", "name": "service_box"}
    created = service.accepted("add_element", kind="rigid_body", name="service_box", properties={
        "moves_freely_in_world": True, "mass_kilograms": 10.0,
        "inertia_moments_about_center_of_mass_kilogram_square_meters": [1.0, 1.0, 1.0],
        "placement.vertical_offset_in_local_track_frame_meters": -2.0})
    assert created["result"] == {"element": body} and created["revision"] == 4
    body_readings = readings_of(body)
    assert body_readings["placement.vertical_offset_in_local_track_frame_meters"]["value"] == -2.0
    assert body_readings[station_offset]["editable"]
    frame = {"kind": "fixed_frame", "name": "service_box_top"}
    assert service.accepted("add_element", kind="fixed_frame", name=frame["name"], properties={
        "body_name": body["name"], "position_in_body_frame_meters": [0.0, 0.0, -0.5]},
    )["result"] == {"element": frame}
    spring = {"kind": "translational_spring_damper", "name": "service_spring"}
    assert service.accepted("add_element", kind=spring["kind"], name=spring["name"], properties={
        "reference_frame_name": frame["name"],
        "opposite_frame_name": properties_of(suspension)["reference_frame_name"]},
    )["result"] == {"element": spring}

    refusal = service.refused("add_element", kind="fixed_frame", name="frame_without_body",
                              properties={})
    assert refusal["element"] == {"kind": "fixed_frame", "name": "frame_without_body"}
    assert refusal["property"] == "body_name"
    refusal = service.refused("add_element", kind="fixed_frame", name="frame_with_bad_body",
                              properties={"body_name": 3.0})
    assert refusal["element"] == {"kind": "fixed_frame", "name": "frame_with_bad_body"}
    assert refusal["property"] == "body_name"

    renamed_body = {"kind": "rigid_body", "name": "service_crate"}
    renamed = service.accepted("set_property", element=body, property="name",
                               value=renamed_body["name"])
    assert renamed["result"] == {"element": renamed_body}
    assert properties_of(frame)["body_name"] == renamed_body["name"]
    assert renamed_body in outline_elements() and body not in outline_elements()

    assert service.accepted("check")["result"]["passed"]
    drawn_bodies = {drawing["element"]["name"]: drawing for drawing in
                    service.accepted("get_preview")["result"]["preview"]["rigid_bodies"]}
    assert drawn_bodies[renamed_body["name"]]["visual_definition_body_name"] is None
    assert drawn_bodies["carbody"]["visual_definition_body_name"] == "carbody"

    revision_before_query = service.accepted("get_outline")["revision"]
    consequences = service.accepted("get_removal_consequences", element=renamed_body)
    assert consequences["result"] == {"removed_elements": [renamed_body, frame, spring]}
    assert consequences["revision"] == revision_before_query
    removed = service.accepted("remove_element", element=renamed_body)
    assert removed["result"] == consequences["result"]
    assert removed["revision"] == revision_before_query + 1
    assert outline_elements() == opened_outline
    service.accepted("undo")
    assert renamed_body in outline_elements() and spring in outline_elements()

    refusal = service.refused("remove_element", element={"kind": "rigid_body", "name": "carbody"})
    assert refusal["element"] == {"kind": "rigid_body", "name": "carbody"}
    assert refusal["property"] is None

    # Values are read as the property's value kind: a token, a reference
    # naming none, and a curve by its row fields.
    lateral_damper = {"kind": "series_spring_viscous_damper", "name": "front_lateral_damper"}
    refusal = service.refused("set_property", element=lateral_damper, property="axis",
                              value="diagonal")
    assert refusal["element"] == lateral_damper and refusal["property"] == "axis"
    joint_name = next(element["name"] for element in outline["elements"]
                      if element["kind"] == "revolute_joint")
    for joint in (joint_name, None):
        service.accepted("set_property", element=first_interface,
                         property="independent_wheel_revolute_joint_name", value=joint)
        assert properties_of(first_interface)["independent_wheel_revolute_joint_name"] == joint
        # An argument left out is not a null: without its value the edit is
        # refused and the reference keeps what it names.
        before = service.accepted("get_outline")
        service.refused("set_property", element=first_interface,
                        property="independent_wheel_revolute_joint_name")
        assert properties_of(first_interface)["independent_wheel_revolute_joint_name"] == joint
        after = service.accepted("get_outline")
        assert (after["revision"], after["can_undo"], after["can_redo"]) == (
            before["revision"], before["can_undo"], before["can_redo"])
    # Nor is a missing properties object an empty one.
    before = service.accepted("get_outline")
    service.refused("add_element", kind="rigid_body", name="body_without_properties")
    after = service.accepted("get_outline")
    assert after["result"]["elements"] == before["result"]["elements"]
    assert after["revision"] == before["revision"]
    yaw_damper = next(element for element in outline["elements"]
                      if element["kind"] == "saturated_piecewise_linear_damper")
    curve = properties_of(yaw_damper)["curve"]
    # The vehicle file states each curve point under its record members' own
    # names, which are the row fields.
    vehicle_file = json.loads(Path(gz18["vehicle_definition_path"]).read_text())
    assert curve and curve == next(damper["curve"] for damper in
                                   vehicle_file["saturated_piecewise_linear_dampers"]
                                   if damper["name"] == yaw_damper["name"])
    assert all(set(row) == set(row_fields) for row in curve)
    steeper = [{field: row[field] * 2.0 for field in row_fields} for row in curve]
    service.accepted("set_property", element=yaw_damper, property="curve", value=steeper)
    assert properties_of(yaw_damper)["curve"] == steeper
    refusal = service.refused("set_property", element=yaw_damper, property="curve",
                              value=[{row_fields[0]: 0.0}])
    assert refusal["element"] == yaw_damper and refusal["property"] == "curve"

    # An unfinished project opens; a mismatched contact declaration is
    # reported by check and leaves the preview absent.
    contact_records = {
        name: json.loads(Path(arguments["contact_record_path"]).read_text())
        for name, arguments in (("gz18", gz18), ("irw", irw))
    }
    wrong_contacts = (
        ({**irw, "contact_record_path": gz18["contact_record_path"]},
         contact_records["gz18"]["declaration"]["carriers"][0]["body_name"]),
        ({**gz18, "contact_record_path": irw["contact_record_path"]},
         contact_records["irw"]["declaration"]["carriers"][0]["body_name"]),
    )
    for arguments, missing_body in wrong_contacts:
        opened = service.accepted("open_vehicle", **arguments)
        assert (opened["revision"], opened["can_undo"], opened["can_redo"]) == (0, False, False)
        report = service.accepted("check")["result"]
        assert not report["passed"]
        assert any(missing_body in refusal for refusal in report["refusals"]), report
        assert service.accepted("get_preview")["result"]["preview"] is None

    reopened = service.accepted("open_vehicle", **irw)
    assert (reopened["revision"], reopened["can_undo"], reopened["can_redo"]) == (0, False, False)
    irw_outline = service.accepted("get_outline")["result"]
    assert irw_outline["vehicle_name"] == "IRW" and irw_outline["contact_record_present"] is True
    assert service.accepted("check")["result"]["passed"]
    irw_preview = service.accepted("get_preview")["result"]["preview"]
    assert irw_preview["project_revision"] == 0
    require_drawings_name_outline_elements(irw_preview, irw_outline)

    service.finish()

    verify_saving_and_reopening(executable, gz18, irw, work_directory)
    assert {path: path.read_bytes() for path in reference_files} == reference_files, (
        "a reference vehicle's file changed")
    print("vehicle modelling service protocol passed")


if __name__ == "__main__":
    main()
