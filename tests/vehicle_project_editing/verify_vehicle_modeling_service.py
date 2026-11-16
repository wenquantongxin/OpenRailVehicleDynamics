"""Drive the vehicle modelling service through its standard input and output.

Opens GZ18 with its reference contact record, lists the element kinds, reads,
edits, checks, previews, undoes, redoes and refuses, asserting the protocol's
structure and its revision behaviour rather than any numeric result. Missing
inputs keep the open session; mismatched contact declarations open and are
reported by checking.
"""
import json
import math
from pathlib import Path
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
VALUE_KINDS = {"number", "vector3", "text", "boolean", "rotation", "wheel_side",
               "saturated_piecewise_linear_damper_curve"}


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
            assert set(response) == {"id", "ok", "revision", "can_undo", "can_redo", "result"}
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


def is_vector(value):
    return isinstance(value, list) and len(value) == 3 and all(
        isinstance(component, (int, float)) for component in value)


def is_rotation(value):
    return isinstance(value, list) and len(value) == 3 and all(is_vector(row) for row in value)


def require_drawings_name_outline_elements(preview, outline):
    """Every drawing has its shape and names an element of the outline of the
    kind its list shows, and every wheel a rigid body of the outline it turns
    with; no element is drawn twice; and the drawn elements with those not
    drawable are exactly the outline's elements of the kinds with drawings."""
    assert all(set(body) == {"element", "rotation_drawing_from_body",
                             "origin_in_drawing_frame_meters"}
               and is_rotation(body["rotation_drawing_from_body"])
               and is_vector(body["origin_in_drawing_frame_meters"])
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


def main():
    executable = Path(sys.argv[1]).resolve()
    vehicle_library = Path(sys.argv[2]).resolve() / "vehicle_library"
    gz18, irw = ({"vehicle_definition_path": str(vehicle_library / name / "vehicle_definition.json"),
                  "startup_state_path": str(vehicle_library / name / "startup_states" /
                                            "moving_startup_60kmh.json"),
                  "contact_record_path": str(vehicle_library / name / "contact_record.json")}
                 for name in ("gz18", "irw"))
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
        assert properties and all(set(prop) == {"name", "value_kind", "editable"}
                                  and prop["value_kind"] in VALUE_KINDS
                                  and isinstance(prop["editable"], bool) for prop in properties)
    suspension_kind = {prop["name"]: prop for prop in described["translational_spring_damper"]}
    assert suspension_kind["stiffness_newtons_per_meter"]["value_kind"] == "vector3"
    assert suspension_kind["stiffness_newtons_per_meter"]["editable"]
    assert not suspension_kind["name"]["editable"]
    rigid_body_kind = {prop["name"]: prop for prop in described["rigid_body"]}
    station_offset = "mechanical_track_station_layout.station_offset_meters"
    assert rigid_body_kind[station_offset]["value_kind"] == "number"
    assert rigid_body_kind[station_offset]["editable"]
    for kind in CONTACT_ELEMENT_KINDS:
        assert not any(prop["editable"] for prop in described[kind])

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
        # A reading carries its kind's name and value kind; it is editable
        # when its kind's property is and this element holds a value for it.
        kind_properties = described[element["kind"]]
        assert [(prop["name"], prop["value_kind"]) for prop in properties] == [
            (prop["name"], prop["value_kind"]) for prop in kind_properties]
        assert all(prop["editable"] == (kind_prop["editable"] and prop["value"] is not None)
                   for prop, kind_prop in zip(properties, kind_properties)), properties
        return {prop["name"]: prop for prop in properties}

    def properties_of(element):
        return {name: reading["value"] for name, reading in readings_of(element).items()}

    # The reference contact record's carriers and interfaces are listed and
    # read, and read-only.
    for kind, name_property in zip(CONTACT_ELEMENT_KINDS, ("carrier_name", "interface_name")):
        contact_elements = [element for element in outline["elements"] if element["kind"] == kind]
        assert contact_elements, kind
        for element in contact_elements:
            assert properties_of(element)[name_property] == element["name"]
        refusal = service.refused("set_property", element=contact_elements[0],
                                  property=name_property, value="renamed")
        assert refusal["element"] == contact_elements[0] and refusal["property"] == name_property

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

    undone = service.accepted("undo")
    assert (undone["revision"], undone["can_undo"], undone["can_redo"]) == (2, False, True)
    assert properties_of(suspension)["stiffness_newtons_per_meter"] == stiffness
    stale = service.accepted("get_preview")
    assert stale["revision"] == 2 and stale["result"]["preview"]["project_revision"] == 1

    refusal = service.refused("set_property", element=suspension, property="name",
                              value="renamed_suspension")
    assert refusal["element"] == suspension and refusal["property"] == "name"
    unchanged = service.accepted("get_outline")
    assert (unchanged["revision"], unchanged["can_undo"], unchanged["can_redo"]) == (2, False, True)
    assert properties_of(suspension)["name"] == suspension["name"]

    redone = service.accepted("redo")
    assert (redone["revision"], redone["can_undo"], redone["can_redo"]) == (3, True, False)
    assert properties_of(suspension)["stiffness_newtons_per_meter"] == edited_stiffness

    refusal = service.refused("rename_vehicle")
    assert refusal["element"] is None and refusal["property"] is None

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

    service.process.stdin.close()
    assert service.process.stdout.read() == "", "the service wrote more than its responses"
    assert service.process.wait(timeout=60) == 0, service.process.stderr.read()
    print("vehicle modelling service protocol passed")


if __name__ == "__main__":
    main()
