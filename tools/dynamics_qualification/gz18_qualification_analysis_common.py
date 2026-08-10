#!/usr/bin/env python3
"""Private mechanical helpers for the GZ18 qualification analyses.

The G60, G61, and G62 entry points deliberately keep separate frozen-reference
contracts.  This module only shares parsing, naming, statistics, and execution
identity mechanics; it does not choose a reference corpus or acceptance window.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


SAMPLE_PERIOD_SECONDS = 0.0005
MACRO_ALARM_LIMIT = 1.0e-4
MACRO_COORDINATE_FRAME = (
    "ORVD standard track T: +x toward increasing station, +y right, +z down"
)
COMMON_W_TO_ORVD_TRACK_T = (
    "diag(1,-1,-1); negate lateral displacement and yaw angle exactly once"
)

AXLES = (
    ("ff", "front_leading_wheelset", 9.1),
    ("fr", "front_trailing_wheelset", 6.6),
    ("rf", "rear_leading_wheelset", -6.6),
    ("rr", "rear_trailing_wheelset", -9.1),
)
SIDES = ("left", "right")
CONTACT_QUANTITIES = {
    "Q": "vertical_support_force_on_wheel_newtons",
    "N": "primary_patch_normal_force_newtons",
    "Tx": "longitudinal_force_on_wheel_newtons",
    "Ty": "lateral_force_on_wheel_newtons",
}


class AnalysisError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AnalysisError(message)


def load_json_object(path: Path, name: str) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise AnalysisError(f"could not read {name} '{path}': {error}") from error
    require(isinstance(value, dict), f"{name} must contain a JSON object")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
    except OSError as error:
        raise AnalysisError(f"could not hash '{path}': {error}") from error
    return digest.hexdigest()


def expected_times(sample_count: int, terminal_time_seconds: float) -> np.ndarray:
    result = np.arange(sample_count, dtype=np.float64) * np.float64(
        SAMPLE_PERIOD_SECONDS
    )
    result[-1] = terminal_time_seconds
    return result


def require_finite(array: np.ndarray, name: str) -> None:
    require(np.issubdtype(array.dtype, np.number), f"{name} is not numeric")
    require(bool(np.isfinite(array).all()), f"{name} contains a non-finite value")


def load_orvd_artifact(
    directory: Path,
    *,
    goal_name: str,
    sample_count: int,
    terminal_time_nanoseconds: int,
    track_irregularity_identifier: str = "gz18_aar6_reference_irregularity",
    track_geometry_filename: str = "straight_level_2000m.json",
    base_track_definition_interval_meters: tuple[int, int] = (0, 2000),
) -> dict[str, Any]:
    """Load one complete ORVD artifact with a goal-specific clock contract."""

    terminal_time_seconds = terminal_time_nanoseconds * 1.0e-9
    require(directory.is_dir(), f"ORVD artifact directory does not exist: {directory}")
    for filename in ("COMPLETE", "metadata.json", "performance.json",
                     "observations.tsv"):
        require((directory / filename).is_file(),
                f"ORVD artifact is missing {filename}")

    metadata = load_json_object(directory / "metadata.json", "ORVD metadata")
    performance = load_json_object(directory / "performance.json", "ORVD performance")
    require(metadata.get("completed") is True, "ORVD metadata is not complete")
    expected_identity = {
        "vehicle_name": "GZ18",
        "mechanical_definition_identifier": "gz18_reference_mechanics",
        "load_condition_identifier": "gz18_reference_load_condition",
        "wheel_profile_identifier": "gz18_reference_wheel_profile",
        "rail_profile_identifier": "uic60_rail_profile",
        "contact_strategy_identifier": "gz18_reference_wheel_rail_contact",
        "track_irregularity_identifier": track_irregularity_identifier,
        "sample_period_nanoseconds": 500_000,
        "terminal_time_nanoseconds": terminal_time_nanoseconds,
        "sample_count": sample_count,
        "vehicle_layout_reference_track_station_meters": 0,
    }
    for key, value in expected_identity.items():
        require(metadata.get(key) == value,
                f"ORVD metadata {key!r} does not match the {goal_name} identity")
    require(metadata.get("initial_longitudinal_speed_meters_per_second") ==
            16.666666666666668,
            "ORVD metadata has the wrong initial longitudinal speed")
    require(metadata.get("base_track_definition_interval_meters") ==
            list(base_track_definition_interval_meters),
            "ORVD metadata has the wrong base-track definition interval")
    input_paths = metadata.get("input_paths")
    require(isinstance(input_paths, dict),
            "ORVD metadata input_paths is not an object")
    track_path = input_paths.get("track_geometry")
    require(isinstance(track_path, str) and
            Path(track_path).name == track_geometry_filename,
            f"ORVD metadata does not identify the {goal_name} track asset")

    complete_text = (directory / "COMPLETE").read_text(encoding="utf-8")
    require(complete_text == f"{sample_count} samples\n",
            "ORVD COMPLETE marker has the wrong sample count")

    observation_path = directory / "observations.tsv"
    try:
        with observation_path.open("r", encoding="utf-8") as stream:
            header = stream.readline().rstrip("\n").split("\t")
    except OSError as error:
        raise AnalysisError(f"could not read ORVD observation header: {error}") from error
    require(len(header) == len(set(header)), "ORVD observation columns are not unique")
    try:
        values = np.loadtxt(observation_path, delimiter="\t", skiprows=1,
                            dtype=np.float64)
    except (OSError, ValueError) as error:
        raise AnalysisError(f"could not parse ORVD observations: {error}") from error
    if values.ndim == 1:
        values = values.reshape(1, -1)
    require(values.shape == (sample_count, len(header)),
            f"ORVD observations do not have the declared {sample_count}-row width")
    require_finite(values, "ORVD observations")
    columns = {name: values[:, index] for index, name in enumerate(header)}

    required_columns = {"sample_index", "time_seconds"}
    for _, name, _ in AXLES:
        required_columns.update({
            f"{name}.track_station_meters",
            f"{name}.lateral_meters",
            f"{name}.yaw_radians",
        })
        for side in SIDES:
            prefix = f"{name}.{side}"
            required_columns.add(f"{prefix}.contact_patch_count")
            required_columns.update(
                f"{prefix}.{suffix}" for suffix in CONTACT_QUANTITIES.values()
            )
    missing = sorted(required_columns - set(columns))
    require(not missing, f"ORVD observations are missing columns: {missing}")

    expected_index = np.arange(sample_count, dtype=np.float64)
    require(np.array_equal(columns["sample_index"], expected_index),
            f"ORVD sample_index is not the exact integer sequence 0..{sample_count - 1}")
    time_error = np.max(np.abs(
        columns["time_seconds"] - expected_times(sample_count, terminal_time_seconds)
    ))
    require(time_error <= 2.0e-15,
            f"ORVD sample clock differs from index*0.5ms by {time_error:.17g} s")
    for _, name, initial_station in AXLES:
        station = columns[f"{name}.track_station_meters"]
        require(station[0] == initial_station,
                f"{name} has initial station {station[0]:.17g}, expected {initial_station}")
        require(bool(np.all(np.diff(station) > 0.0)),
                f"{name} station is not strictly increasing")

    patch_matrix = np.column_stack([
        columns[f"{name}.{side}.contact_patch_count"]
        for _, name, _ in AXLES for side in SIDES
    ])
    require(bool(np.all(patch_matrix >= 0.0)) and
            np.array_equal(patch_matrix, np.rint(patch_matrix)),
            "ORVD contact patch counts are not non-negative integers")

    required_performance = (
        "advance_wall_seconds", "observation_wall_seconds",
        "endpoint_diagnostics_wall_seconds",
        "data_and_metadata_write_wall_seconds", "dense_state_bytes",
        "observation_buffer_bytes",
    )
    for key in required_performance:
        value = performance.get(key)
        require(isinstance(value, (int, float)) and math.isfinite(value) and value >= 0,
                f"ORVD performance field {key!r} is absent or invalid")

    return {
        "metadata": metadata,
        "performance": performance,
        "columns": columns,
        "patch_count": patch_matrix.astype(np.int64),
        "observation_sha256": sha256_file(observation_path),
    }


def load_execution_identity(
    path: Path, artifact_directory: Path, *, goal_name: str
) -> dict[str, Any]:
    value = load_json_object(path, "execution identity")
    required = {
        "orvd_revision", "build_type", "compiler",
        "hardware", "requested_cpu_affinity", "applied_cpu_affinity",
        "openmp_environment", "runner_arguments",
        "qualification_artifact_directory", "process_wall_seconds",
        "maximum_resident_set_kilobytes", "executable_sha256", "exit_status",
    }
    missing = sorted(required - set(value))
    require(not missing, f"execution identity is missing fields: {missing}")
    require(value["build_type"] == "Release",
            f"{goal_name} was not run with a Release build")
    require(isinstance(value["openmp_environment"], dict),
            "execution identity openmp_environment is not an object")
    for key in ("orvd_revision", "compiler", "hardware",
                "requested_cpu_affinity", "executable_sha256"):
        require(isinstance(value[key], str) and value[key],
                f"execution identity {key!r} is empty")
    for key in ("process_wall_seconds", "maximum_resident_set_kilobytes"):
        require(isinstance(value[key], (int, float)) and
                math.isfinite(value[key]) and value[key] > 0,
                f"execution identity {key!r} is invalid")
    require(value["exit_status"] == 0,
            f"the recorded {goal_name} process did not succeed")
    require(isinstance(value["applied_cpu_affinity"], list) and
            value["applied_cpu_affinity"] and
            all(isinstance(cpu, int) and cpu >= 0
                for cpu in value["applied_cpu_affinity"]),
            "execution identity applied_cpu_affinity is invalid")
    require(isinstance(value["runner_arguments"], list) and
            len(value["runner_arguments"]) == 8 and
            all(isinstance(argument, str) for argument in value["runner_arguments"]),
            "execution identity runner_arguments is invalid")
    recorded_artifact = value["qualification_artifact_directory"]
    require(isinstance(recorded_artifact, str) and
            Path(recorded_artifact).resolve() == artifact_directory,
            "execution identity belongs to a different qualification artifact")
    require(Path(value["runner_arguments"][5]).resolve() == artifact_directory,
            "execution identity runner arguments name a different artifact")
    return value


def column_stack(columns: dict[str, np.ndarray], suffix: str) -> np.ndarray:
    return np.column_stack([columns[f"{name}.{suffix}"] for _, name, _ in AXLES])


def orvd_contact(columns: dict[str, np.ndarray], quantity: str) -> np.ndarray:
    suffix = CONTACT_QUANTITIES[quantity]
    return np.column_stack([
        columns[f"{name}.{side}.{suffix}"]
        for _, name, _ in AXLES for side in SIDES
    ])


def common_w_macro_in_orvd_track_frame(
    pair: dict[str, np.ndarray], source: str, quantity: str
) -> np.ndarray:
    """Convert one frozen common-W macro response to ORVD track-T."""

    require(source in ("simpack", "drake"), "unknown paired macro source")
    require(quantity in ("lateral_m", "yaw_rad"),
            "unknown paired macro quantity")
    return -pair[f"{source}_{quantity}"]


def error_statistics(
    candidate: np.ndarray,
    reference: np.ndarray,
    abscissa: np.ndarray,
    stations_candidate: np.ndarray,
    stations_reference: np.ndarray,
    *,
    station_domain: bool = False,
) -> dict[str, Any]:
    require(candidate.shape == reference.shape == abscissa.shape,
            "internal error-statistic arrays have different shapes")
    require(candidate.size > 0, "an error statistic has no samples")
    error = candidate - reference
    absolute = np.abs(error)
    peak_local = int(np.argmax(absolute))
    result = {
        "sample_count": int(error.size),
        "signed_mean": float(np.mean(error)),
        "rms": float(np.sqrt(np.mean(np.square(error)))),
        "absolute_error_percentile_95": float(np.percentile(absolute, 95.0)),
        "maximum_absolute_error": float(absolute[peak_local]),
        "signed_error_at_maximum": float(error[peak_local]),
        "candidate_station_at_peak_meters": float(stations_candidate[peak_local]),
        "reference_station_at_peak_meters": float(stations_reference[peak_local]),
    }
    if station_domain:
        result["peak_track_station_meters"] = float(abscissa[peak_local])
    else:
        peak_index = int(abscissa[peak_local])
        result["peak_sample_index"] = peak_index
        result["peak_time_seconds"] = float(peak_index * SAMPLE_PERIOD_SECONDS)
    return result


def longest_zero_run(values: np.ndarray) -> dict[str, int | None]:
    zero = values == 0
    best_length = 0
    best_begin: int | None = None
    current_begin = 0
    current_length = 0
    for index, is_zero in enumerate(zero):
        if is_zero:
            if current_length == 0:
                current_begin = index
            current_length += 1
            if current_length > best_length:
                best_length = current_length
                best_begin = current_begin
        else:
            current_length = 0
    return {
        "zero_sample_count": int(np.count_nonzero(zero)),
        "longest_zero_run_samples": best_length,
        "longest_zero_run_begin_index": best_begin,
        "longest_zero_run_end_index": (
            None if best_begin is None else best_begin + best_length - 1
        ),
    }


def write_json(path: Path, value: dict[str, Any]) -> None:
    try:
        with path.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
    except OSError as error:
        raise AnalysisError(f"could not write '{path}': {error}") from error
