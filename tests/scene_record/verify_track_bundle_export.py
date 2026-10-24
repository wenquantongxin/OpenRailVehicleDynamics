"""Exercise the public exporter with explicit and recovered rail placements."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile


def rotation(q):
    w, x, y, z = q
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def check_track(table, expected):
    for side, datum in zip(("left", "right"), expected):
        saved = table["rail_datums"][side]
        for key, value in zip(("lateral_meters", "vertical_meters"), datum):
            assert math.isclose(saved[key], value, abs_tol=2e-13)
        for center, q, point in zip(table["centerline_in_inertial_meters"],
                                    table["rotation_inertial_from_track_wxyz"],
                                    table[f"{side}_rail_datum_in_inertial_meters"]):
            r = rotation(q)
            recovered = [sum(r[j][i]*(point[j]-center[j]) for j in range(3)) for i in range(3)]
            assert abs(recovered[0]) < 2e-13
            assert abs(recovered[1]-datum[0]) < 2e-13
            assert abs(recovered[2]-datum[1]) < 2e-13


def main():
    executable = Path(sys.argv[1]).resolve()
    source = Path(sys.argv[2]).resolve()
    with tempfile.TemporaryDirectory(prefix="orvd-track-bundle-") as temporary:
        root = Path(temporary)
        line = json.loads((source / "track_library/geometries/straight_level_1100m.json").read_text())
        expected = ((-0.751, -0.024), (0.754, -0.021))
        empty_visual = root / "empty-visual.json"
        empty_visual.write_text("")
        datums = ["--left-lateral", "-0.751", "--left-vertical", "-0.024",
                  "--right-lateral", "0.754", "--right-vertical", "-0.021"]
        for label, grade, cant in (("straight", 0.0, 0.0), ("sloped", 0.035, 0.12)):
            line["vertical_profile"] = {"segments": [{"shape": "constant_grade", "length_meters": 1100.0,
                                                      "centerline_upward_grade": grade}], "seam_transitions": []}
            line["superelevation_profile"] = {"segments": [{"shape": "constant", "length_meters": 1100.0,
                                                            "superelevation_meters": cant}], "seam_transitions": []}
            path = root / f"{label}.json"
            path.write_text(json.dumps(line))
            base = [str(executable), "--track", str(path), "--start", "17.3", "--end", "21.0", "--spacing", "0.7"]

            def run(name, extra, success=True):
                directory = root / name
                completed = subprocess.run(base + ["--output", str(directory)] + extra,
                                           capture_output=True, text=True)
                assert (completed.returncode == 0) == success, completed.stderr
                if success:
                    document = json.loads((directory / "scene.json").read_text())
                    assert document["schema"] == "orvd.scene_bundle"
                    assert set(document) == {"schema", "assets"}
                    assert not (directory / "frames.f64le").exists()
                    check_track(document["assets"]["track"], expected)
                    return directory, document

            directory, document = run(f"{label}-explicit", datums)
            stations = document["assets"]["track"]["stations_meters"]
            assert stations[0] == 17.3 and stations[-1] == 21.0
            assert len(stations) == 7 and all(a < b for a, b in zip(stations, stations[1:]))
            run(f"{label}-bundle", ["--rail-datums-from", str(directory / "scene.json")])
            # Static extraction does not depend on topology or either frame file.
            document["schema"] = "orvd.scene_record"
            document["description"] = {"world_frame": document["assets"]["world_frame"]}
            record_path = root / f"{label}-record.json"
            record_path.write_text(json.dumps(document))
            run(f"{label}-record", ["--rail-datums-from", str(record_path)])
            run(f"{label}-conflict", datums + ["--rail-datums-from", str(record_path)], False)
            run(f"{label}-missing", datums[:-2], False)
            run(f"{label}-empty-visual", datums + ["--visual-definition", str(empty_visual)], False)
            run(f"{label}-duplicate", datums + ["--spacing", "1"], False)
            run(f"{label}-nonfinite", datums[:-1] + ["nan"], False)
            # An existing completed bundle cannot be overwritten.
            run(f"{label}-explicit", datums, False)
        print("track bundle export passed")


if __name__ == "__main__":
    main()
