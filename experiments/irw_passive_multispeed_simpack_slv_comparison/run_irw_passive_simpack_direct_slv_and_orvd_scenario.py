#!/usr/bin/env python3
"""Run one IRW passive SIMPACK direct-SLV and ORVD scenario serially."""

from __future__ import annotations

import argparse
import difflib
import fcntl
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from extract_irw_passive_simpack_direct_slv_sbr_channels import (
    DEFAULT_LIBSBR,
    extract,
)
from irw_passive_multispeed_simpack_direct_slv_scenario_catalog import (
    Scenario,
    require_scenario,
)


DEFAULT_SIMPACK_SLV = Path("/opt/Simpack-2021x/run/bin/linux64/simpack-slv")
EXPERIMENT_DIRECTORY = Path(__file__).resolve().parent
REPOSITORY_ROOT = EXPERIMENT_DIRECTORY.parents[1]
CANONICAL_SPCK = (
    REPOSITORY_ROOT
    / "vehicle_library/irw/reference_models/simpack/main_model/irw_vehicle.spck"
)
CANONICAL_ORVD_STARTUP = (
    REPOSITORY_ROOT / "vehicle_library/irw/startup_states/moving_startup_60kmh.json"
)
IRW_WHEEL_REVOLUTE_JOINT_NAMES = (
    "rev_wheel_ff_l",
    "rev_wheel_ff_r",
    "rev_wheel_fr_l",
    "rev_wheel_fr_r",
    "rev_wheel_rf_l",
    "rev_wheel_rf_r",
    "rev_wheel_rr_l",
    "rev_wheel_rr_r",
)
FROZEN_SIMPACK_IRREGULARITY_CONTRACTS = {
    "aar5_irregularity": {
        "lateral_excitation": "$E_AAR5_LAT",
        "vertical_excitation": "$E_AAR5_VER",
        "start_parameter": "$_AAR5IrregStart",
        "end_parameter": "$_AAR5IrregEnd",
        "fade_parameter": "$_AAR5IrregFade",
        "start_value": "'160'",
        "end_value": "'1100'",
        "fade_value": "'40'",
    },
    "aar6_irregularity": {
        "lateral_excitation": "$E_AAR6_LAT",
        "vertical_excitation": "$E_AAR6_VER",
        "start_parameter": "$_AAR6IrregStart",
        "end_parameter": "$_AAR6IrregEnd",
        "fade_parameter": "$_AAR6IrregFade",
        "start_value": "'50'",
        "end_value": "'300'",
        "fade_value": "'50'",
    },
    "erri_low_irregularity": {
        "lateral_excitation": "$E_ERRI_low_lat",
        "vertical_excitation": "$E_ERRI_low_ver",
        "start_parameter": "$_ERRILowIrregStart",
        "end_parameter": "$_ERRILowIrregEnd",
        "fade_parameter": "$_ERRILowIrregFade",
        "start_value": "'50'",
        "end_value": "'500'",
        "fade_value": "'50'",
    },
}


def assignment_line(line: str, property_name: str) -> bool:
    view = line.lstrip()
    if not view.startswith(property_name):
        return False
    if len(view) == len(property_name):
        return True
    return view[len(property_name)].isspace() or view[len(property_name)] == "("


def replace_assignment(text: str, property_name: str, value: str) -> str:
    lines = text.splitlines(keepends=True)
    matches = [
        index
        for index, line in enumerate(lines)
        if assignment_line(line, property_name)
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected one {property_name!r} assignment, found {len(matches)}"
        )
    index = matches[0]
    line = lines[index]
    newline = "\n" if line.endswith("\n") else ""
    body = line[:-1] if newline else line
    equals = body.find("=")
    if equals < 0:
        raise RuntimeError(f"malformed {property_name!r} assignment")
    comment = body.find("!", equals + 1)
    value_end = len(body) if comment < 0 else comment
    original_field = body[equals + 1 : value_end]
    if original_field.strip() == value:
        return text
    leading = original_field[: len(original_field) - len(original_field.lstrip())]
    trailing = original_field[len(original_field.rstrip()) :]
    suffix = "" if comment < 0 else body[comment:]
    lines[index] = body[: equals + 1] + leading + value + trailing + suffix + newline
    return "".join(lines)


def scoped_assignment_value(text: str, property_name: str, object_name: str) -> str:
    values: list[str] = []
    for line in text.splitlines():
        if not assignment_line(line, property_name):
            continue
        opening = line.find("(")
        closing = line.find(")", opening + 1)
        if opening < 0 or closing < 0:
            continue
        if object_name not in line[opening + 1 : closing].split():
            continue
        equals = line.find("=", closing + 1)
        if equals < 0:
            raise RuntimeError(
                f"malformed {property_name!r} assignment for {object_name!r}"
            )
        comment = line.find("!", equals + 1)
        end = len(line) if comment < 0 else comment
        values.append(line[equals + 1 : end].strip())
    if len(values) != 1:
        raise RuntimeError(
            f"expected one {property_name!r} assignment for {object_name!r}, "
            f"found {len(values)}"
        )
    return values[0]


def verify_simpack_track_irregularity_contract(source: str, scenario: Scenario) -> None:
    try:
        contract = FROZEN_SIMPACK_IRREGULARITY_CONTRACTS[
            scenario.irregularity_identifier
        ]
    except KeyError as error:
        raise RuntimeError(
            "scenario does not select an authorized frozen irregularity"
        ) from error
    track_expectations = {
        "track.excit.type": "1",
        "track.excit.lat": contract["lateral_excitation"],
        "track.excit.vert": contract["vertical_excitation"],
        "track.excit.roll": "null",
        "track.excit.left.lat": "null",
        "track.excit.left.vert": "null",
        "track.excit.left.roll": "null",
        "track.excit.right.lat": "null",
        "track.excit.right.vert": "null",
        "track.excit.right.roll": "null",
        "track.excit.gauge": "null",
        "track.excit.start": contract["start_parameter"],
        "track.excit.end": contract["end_parameter"],
        "track.excit.fade.len": contract["fade_parameter"],
    }
    for property_name, expected in track_expectations.items():
        actual = scoped_assignment_value(source, property_name, scenario.simpack_track)
        if actual != expected:
            raise RuntimeError(
                f"SIMPACK track {scenario.simpack_track!r} has {property_name}="
                f"{actual!r}, expected {expected!r}"
            )
    for role in ("start", "end", "fade"):
        parameter = contract[f"{role}_parameter"]
        expected = contract[f"{role}_value"]
        actual = scoped_assignment_value(source, "subvar.str", parameter)
        if actual != expected:
            raise RuntimeError(
                f"SIMPACK irregularity parameter {parameter!r} is {actual!r}, "
                f"expected {expected!r}"
            )


def set_binary64_results(text: str) -> str:
    property_name = "slv.meas.prec.dbl"
    matches = sum(assignment_line(line, property_name) for line in text.splitlines())
    if matches == 1:
        return replace_assignment(text, property_name, "1")
    if matches != 0:
        raise RuntimeError(
            f"expected at most one {property_name!r} assignment, found {matches}"
        )

    lines = text.splitlines(keepends=True)
    anchors = [
        index
        for index, line in enumerate(lines)
        if assignment_line(line, "slv.meas.result")
    ]
    if len(anchors) != 1:
        raise RuntimeError(
            "cannot insert binary64 result setting: slv.meas.result is not unique"
        )
    lines.insert(
        anchors[0] + 1,
        "slv.meas.prec.dbl (                 $SLV_SolverSettings           ) = 1"
        "                       ! Result file in double precision\n",
    )
    return "".join(lines)


def prepare_spck(source: str, scenario: Scenario) -> str:
    escaped_track = re.escape(scenario.simpack_track)
    track_definition = re.compile(
        rf"^track\.type\s*\([^\n]*{escaped_track}\s*\)\s*=",
        flags=re.MULTILINE,
    )
    if len(track_definition.findall(source)) != 1:
        raise RuntimeError(
            f"SIMPACK track {scenario.simpack_track!r} is not uniquely defined"
        )
    verify_simpack_track_irregularity_contract(source, scenario)
    result = source
    result = replace_assignment(result, "track.active", scenario.simpack_track)
    speed = format(scenario.initial_speed_kilometres_per_hour, ".17g")
    result = replace_assignment(result, "vehicle.startvel", f"{{ {speed}/3.6 }}")
    result = replace_assignment(result, "vehicle.applystartvel", "1")
    duration = format(scenario.duration_seconds, ".17g")
    frequency = format(scenario.output_frequency_hertz, ".17g")
    result = replace_assignment(result, "slv.integ.tend.time", f"{{ {duration} s }}")
    result = replace_assignment(result, "slv.integ.tout.freq", f"{{ {frequency} Hz }}")
    return set_binary64_results(result)


def write_json(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def prepare_orvd_startup_state(
    scenario: Scenario, input_directory: Path
) -> tuple[Path, dict[str, object]]:
    document = json.loads(CANONICAL_ORVD_STARTUP.read_text(encoding="utf-8"))
    source_speed = float(document["initial_longitudinal_speed_meters_per_second"])
    expected_source_speed = 60.0 / 3.6
    if not math.isclose(
        source_speed, expected_source_speed, rel_tol=0.0, abs_tol=1e-14
    ):
        raise RuntimeError("canonical IRW startup is not the frozen 60 km/h state")
    target_speed = scenario.initial_speed_kilometres_per_hour / 3.6
    scale = target_speed / source_speed
    document["initial_longitudinal_speed_meters_per_second"] = target_speed

    wheel_states = document.get("revolute_joint_startup_states")
    if not isinstance(wheel_states, list) or len(wheel_states) != 8:
        raise RuntimeError("canonical IRW startup does not contain eight wheel joints")
    by_name: dict[str, dict[str, object]] = {}
    for state in wheel_states:
        name = state.get("joint_name")
        if not isinstance(name, str) or name in by_name:
            raise RuntimeError("IRW startup contains an invalid wheel-joint identity")
        by_name[name] = state
    if set(by_name) != set(IRW_WHEEL_REVOLUTE_JOINT_NAMES):
        raise RuntimeError(
            "IRW startup wheel-joint set differs from the closed topology"
        )
    for name in IRW_WHEEL_REVOLUTE_JOINT_NAMES:
        rate = by_name[name].get("rate")
        if not isinstance(rate, dict) or rate.get("kind") != "explicit_angular_rate":
            raise RuntimeError(f"IRW startup wheel {name} has no explicit rate")
        source_rate = rate.get("angular_rate_radians_per_second")
        if (
            isinstance(source_rate, bool)
            or not isinstance(source_rate, (int, float))
            or not math.isfinite(source_rate)
        ):
            raise RuntimeError(f"IRW startup wheel {name} rate is not finite")
        rate["angular_rate_radians_per_second"] = float(source_rate) * scale

    speed_name = format(scenario.initial_speed_kilometres_per_hour, "g").replace(
        ".", "p"
    )
    path = input_directory / (
        "irw_resolved_startup_state_scaled_from_60kmph_to_" f"{speed_name}kmph.json"
    )
    write_json(path, document)
    return path, {
        "source_startup_state": str(CANONICAL_ORVD_STARTUP),
        "source_speed_metres_per_second": source_speed,
        "target_speed_metres_per_second": target_speed,
        "wheel_rate_scale": scale,
        "scaled_wheel_joint_count": len(IRW_WHEEL_REVOLUTE_JOINT_NAMES),
    }


def git_revision() -> str:
    process = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPOSITORY_ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return process.stdout.strip()


def read_git_revision_file(revision: str, path: Path) -> str:
    try:
        relative = path.resolve().relative_to(REPOSITORY_ROOT.resolve())
    except ValueError as error:
        raise RuntimeError(
            f"Git-backed input lies outside the repository: {path}"
        ) from error
    process = subprocess.run(
        ["git", "show", f"{revision}:{relative.as_posix()}"],
        cwd=REPOSITORY_ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if process.returncode != 0:
        diagnostic = process.stderr.strip()
        raise RuntimeError(
            f"cannot read {relative.as_posix()} from Git revision {revision}: "
            f"{diagnostic}"
        )
    return process.stdout


def run_process(
    command: list[str], cwd: Path, stdout_path: Path, environment: dict[str, str]
) -> tuple[int, float]:
    begin = time.perf_counter()
    with stdout_path.open("w", encoding="utf-8") as output:
        process = subprocess.run(
            command,
            cwd=cwd,
            env=environment,
            stdout=output,
            stderr=subprocess.STDOUT,
            check=False,
            text=True,
        )
    return process.returncode, time.perf_counter() - begin


def parse_simpack_integration_statistics(solver_text: str) -> dict[str, object]:
    fields: tuple[tuple[str, str, type], ...] = (
        ("thread_count", "Threads", int),
        ("integration_step_count", "Integration steps", int),
        ("right_hand_side_evaluation_count", "RHS evaluations", int),
        ("jacobian_evaluation_count", "Jacobian evaluations", int),
        ("step_rejection_count", "Step rejections", int),
        ("corrector_iteration_error_count", "Corrector iteration errors", int),
        ("integration_cpu_seconds", "CPU time", float),
        ("integration_wall_seconds", "Wall clock time", float),
    )
    result: dict[str, object] = {}
    for output_name, label, conversion in fields:
        unit = r"\s+s" if conversion is float else ""
        match = re.search(
            rf"^\s*{re.escape(label)}:\s+([0-9.Ee+-]+){unit}\s*$",
            solver_text,
            flags=re.MULTILINE,
        )
        if match is None:
            raise RuntimeError(
                f"SIMPACK solver log is missing integration statistic {label!r}"
            )
        result[output_name] = conversion(match.group(1))
    return result


def run_simpack(
    scenario: Scenario,
    run_root: Path,
    libsbr_path: Path,
    simpack_slv: Path,
    git_revision_value: str,
    source: str,
) -> dict[str, object]:
    simpack_root = run_root / "simpack"
    output_directory = simpack_root / "output"
    extracted_directory = simpack_root / "extracted"
    simpack_root.mkdir()
    output_directory.mkdir()

    prepared = prepare_spck(source, scenario)
    temporary_stem = f"orvd_{scenario.identifier}_{os.getpid()}"
    temporary_spck = CANONICAL_SPCK.parent / f"{temporary_stem}.spck"
    descriptor = os.open(temporary_spck, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            stream.write(prepared)

        relative_spck = CANONICAL_SPCK.relative_to(REPOSITORY_ROOT).as_posix()
        diff = "".join(
            difflib.unified_diff(
                source.splitlines(keepends=True),
                prepared.splitlines(keepends=True),
                fromfile=f"{git_revision_value}:{relative_spck}",
                tofile=str(temporary_spck),
            )
        )
        (simpack_root / "temporary_model.patch").write_text(diff, encoding="utf-8")
        solver_log = simpack_root / "solver.log"
        stdout_log = simpack_root / "stdout.log"
        command = [
            str(simpack_slv),
            "--file",
            str(temporary_spck),
            "--integration",
            "--output-path",
            str(output_directory),
            "--log-file",
            str(solver_log),
        ]
        expected_sbr = output_directory / f"{temporary_stem}.sbr"
        manifest = {
            "scenario_identifier": scenario.identifier,
            "git_revision": git_revision_value,
            "simpack_model_source": f"{git_revision_value}:{relative_spck}",
            "canonical_spck_worktree_path": str(CANONICAL_SPCK),
            "temporary_spck": str(temporary_spck),
            "expected_sbr": str(expected_sbr),
            "active_track": scenario.simpack_track,
            "initial_speed_kilometres_per_hour": (
                scenario.initial_speed_kilometres_per_hour
            ),
            "duration_seconds": scenario.duration_seconds,
            "output_frequency_hertz": scenario.output_frequency_hertz,
            "command": command,
        }
        write_json(simpack_root / "manifest.json", manifest)

        previous_sigterm = signal.getsignal(signal.SIGTERM)

        def terminate(_signum: int, _frame: object) -> None:
            raise KeyboardInterrupt("received SIGTERM")

        signal.signal(signal.SIGTERM, terminate)
        try:
            return_code, wall_seconds = run_process(
                command, CANONICAL_SPCK.parent, stdout_log, os.environ.copy()
            )
        finally:
            signal.signal(signal.SIGTERM, previous_sigterm)
    finally:
        temporary_spck.unlink(missing_ok=True)

    performance: dict[str, object] = {
        "process_wall_seconds": wall_seconds,
        "return_code": return_code,
    }
    write_json(simpack_root / "performance.json", performance)
    if return_code != 0:
        raise RuntimeError(f"SIMPACK direct solver exited with {return_code}")
    if not solver_log.is_file():
        raise RuntimeError("SIMPACK direct solver did not write its solver log")
    solver_text = solver_log.read_text(encoding="utf-8", errors="replace")
    if "successfully finished." not in solver_text:
        raise RuntimeError("SIMPACK solver log has no successful completion marker")
    performance.update(parse_simpack_integration_statistics(solver_text))
    performance["integration_realtime_coefficient"] = scenario.duration_seconds / float(
        performance["integration_wall_seconds"]
    )
    performance["total_realtime_coefficient"] = scenario.duration_seconds / wall_seconds
    write_json(simpack_root / "performance.json", performance)
    if not expected_sbr.is_file():
        raise RuntimeError(f"SIMPACK did not write expected SBR {expected_sbr}")

    extracted = extract(
        expected_sbr, extracted_directory, scenario.identifier, libsbr_path
    )
    (simpack_root / "COMPLETE").write_text("complete\n", encoding="utf-8")
    return {
        "performance": performance,
        "sbr": str(expected_sbr),
        "extracted": extracted,
    }


def run_orvd(
    scenario: Scenario,
    run_root: Path,
    qualification_binary: Path,
    git_revision_value: str,
    worker_count: int,
) -> dict[str, object]:
    if scenario.orvd_qualification_scenario_identifier is None:
        raise RuntimeError(
            f"scenario {scenario.identifier} has no frozen ORVD recipe yet"
        )
    qualification_binary = qualification_binary.resolve()
    if not qualification_binary.is_file():
        raise FileNotFoundError(qualification_binary)

    orvd_root = run_root / "orvd"
    artifact = orvd_root / "artifact"
    orvd_root.mkdir()
    input_directory = orvd_root / "inputs"
    input_directory.mkdir()
    vehicle = REPOSITORY_ROOT / "vehicle_library/irw/vehicle_definition.json"
    startup, startup_scaling = prepare_orvd_startup_state(scenario, input_directory)
    geometry = REPOSITORY_ROOT / "track_library/geometries" / scenario.track_geometry
    duration_nanoseconds = int(round(scenario.duration_seconds * 1.0e9))
    sample_period_nanoseconds = 500_000
    command = [
        str(qualification_binary),
        scenario.orvd_qualification_scenario_identifier,
        str(vehicle),
        str(startup),
        str(geometry),
        str(REPOSITORY_ROOT),
        scenario.irregularity_identifier,
        str(artifact),
        str(duration_nanoseconds),
        str(sample_period_nanoseconds),
    ]
    environment = os.environ.copy()
    environment.update(
        {
            "OMP_NUM_THREADS": str(worker_count),
            "OMP_DYNAMIC": "FALSE",
            "OMP_PROC_BIND": "close",
            "OMP_PLACES": "cores",
        }
    )
    write_json(
        orvd_root / "manifest.json",
        {
            "scenario_identifier": scenario.identifier,
            "orvd_qualification_scenario_identifier": (
                scenario.orvd_qualification_scenario_identifier
            ),
            "git_revision": git_revision_value,
            "resolved_startup_state": str(startup),
            "startup_scaling": startup_scaling,
            "command": command,
            "environment": {
                name: environment[name]
                for name in (
                    "OMP_NUM_THREADS",
                    "OMP_DYNAMIC",
                    "OMP_PROC_BIND",
                    "OMP_PLACES",
                )
            },
        },
    )
    return_code, wall_seconds = run_process(
        command,
        REPOSITORY_ROOT,
        orvd_root / "stdout.log",
        environment,
    )
    write_json(
        orvd_root / "process_performance.json",
        {"process_wall_seconds": wall_seconds, "return_code": return_code},
    )
    if return_code != 0:
        raise RuntimeError(f"ORVD qualification runner exited with {return_code}")
    if not (artifact / "COMPLETE").is_file():
        raise RuntimeError("ORVD qualification runner did not publish COMPLETE")
    metadata = json.loads((artifact / "metadata.json").read_text(encoding="utf-8"))
    performance = json.loads(
        (artifact / "performance.json").read_text(encoding="utf-8")
    )
    actual_speed = float(metadata["initial_longitudinal_speed_meters_per_second"])
    expected_speed = scenario.initial_speed_kilometres_per_hour / 3.6
    if actual_speed != expected_speed:
        raise RuntimeError(
            "ORVD artifact initial speed differs from the scenario identity"
        )
    if metadata["track_irregularity_identifier"] != scenario.irregularity_identifier:
        raise RuntimeError(
            "ORVD artifact irregularity differs from the scenario identity"
        )
    return {
        "process_wall_seconds": wall_seconds,
        "metadata": metadata,
        "performance": performance,
    }


def positive_integer(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("worker count must be positive")
    return parsed


def preflight(
    scenario: Scenario,
    qualification_binary: Path,
    libsbr_path: Path,
    simpack_slv: Path,
) -> tuple[str, str]:
    if scenario.orvd_qualification_scenario_identifier is None:
        raise RuntimeError(
            f"scenario {scenario.identifier} has no frozen ORVD recipe yet"
        )
    required_files = (
        simpack_slv,
        libsbr_path,
        qualification_binary,
        CANONICAL_SPCK,
        CANONICAL_ORVD_STARTUP,
        REPOSITORY_ROOT / "vehicle_library/irw/vehicle_definition.json",
        REPOSITORY_ROOT / "track_library/geometries" / scenario.track_geometry,
        REPOSITORY_ROOT
        / "track_library/irregularities"
        / f"{scenario.irregularity_identifier}.json",
    )
    for path in required_files:
        if not path.resolve().is_file():
            raise FileNotFoundError(path.resolve())
    revision = git_revision()
    source = read_git_revision_file(revision, CANONICAL_SPCK)
    prepare_spck(source, scenario)
    return revision, source


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument(
        "--qualification-binary",
        type=Path,
        default=(
            REPOSITORY_ROOT / "build/release/tools/dynamics_qualification/"
            "orvd_irw_passive_scenario"
        ),
    )
    parser.add_argument("--libsbr", type=Path, default=DEFAULT_LIBSBR)
    parser.add_argument("--simpack-slv", type=Path, default=DEFAULT_SIMPACK_SLV)
    parser.add_argument("--worker-count", type=positive_integer, default=16)
    arguments = parser.parse_args()
    scenario = require_scenario(arguments.scenario)
    qualification_binary = arguments.qualification_binary.resolve()
    libsbr_path = arguments.libsbr.resolve()
    simpack_slv = arguments.simpack_slv.resolve()
    revision, source = preflight(
        scenario, qualification_binary, libsbr_path, simpack_slv
    )
    run_root = arguments.run_root.resolve()
    if run_root.exists():
        raise FileExistsError(run_root)
    run_root.parent.mkdir(parents=True, exist_ok=True)
    run_root.mkdir()

    lock_directory = (
        REPOSITORY_ROOT / "tmp/irw_passive_multispeed_simpack_slv_comparison"
    )
    lock_directory.mkdir(parents=True, exist_ok=True)
    lock_path = lock_directory / "simpack.lock"
    with lock_path.open("a+", encoding="utf-8") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(
                "another comparison owns the SIMPACK run lock"
            ) from error
        simpack = run_simpack(
            scenario,
            run_root,
            libsbr_path,
            simpack_slv,
            revision,
            source,
        )

    orvd = run_orvd(
        scenario,
        run_root,
        qualification_binary,
        revision,
        arguments.worker_count,
    )
    write_json(
        run_root / "run_summary.json",
        {
            "complete": True,
            "scenario_identifier": scenario.identifier,
            "simpack": simpack,
            "orvd": orvd,
        },
    )
    (run_root / "RUN_COMPLETE").write_text("complete\n", encoding="utf-8")
    print(f"completed {scenario.identifier}: {run_root}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"comparison failed: {error}", file=sys.stderr)
        raise
