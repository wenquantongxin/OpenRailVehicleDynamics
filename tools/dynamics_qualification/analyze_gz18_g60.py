#!/usr/bin/env python3
"""Compare one ORVD G60 artifact with the frozen P038/P039 references.

This is a migration-only, non-installed analysis tool.  It never searches for
an external reference root: every input and the output directory are explicit.
Long arrays and figures remain outside the repository.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


SAMPLE_COUNT = 20_001
SAMPLE_PERIOD_SECONDS = 0.0005
TERMINAL_TIME_SECONDS = 10.0
PRIMARY_STATION_BEGIN_METERS = 100.0
PRIMARY_STATION_END_METERS = 150.0
MACRO_ALARM_LIMIT = 1.0e-4
MACRO_COORDINATE_FRAME = (
    "ORVD standard track T: +x toward increasing station, +y right, +z down"
)
P039_COMMON_W_TO_ORVD_TRACK_T = (
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
    "N": "normal_force_newtons",
    "Tx": "longitudinal_force_on_wheel_newtons",
    "Ty": "lateral_force_on_wheel_newtons",
}

PAIR_KEYS = {
    "time_s",
    "axle_names",
    "wheel_names",
    "simpack_station_m",
    "drake_station_m",
    "simpack_lateral_m",
    "drake_lateral_m",
    "simpack_yaw_rad",
    "drake_yaw_rad",
    "lateral_Drake_minus_SIMPACK_m",
    "yaw_Drake_minus_SIMPACK_rad",
    *(f"mask_{phase}_{axle}" for axle, _, _ in AXLES for phase in
      ("pre_activation", "fade_in", "bearing_100_150")),
    *(f"{source}_contact_{quantity}" for source in ("simpack", "drake")
      for quantity in ("Q", "N", "Tx", "Ty", "Mz", "force",
                       "complete_moment")),
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


def expected_times() -> np.ndarray:
    result = np.arange(SAMPLE_COUNT, dtype=np.float64) * np.float64(
        SAMPLE_PERIOD_SECONDS
    )
    result[-1] = TERMINAL_TIME_SECONDS
    return result


def require_finite(array: np.ndarray, name: str) -> None:
    require(np.issubdtype(array.dtype, np.number), f"{name} is not numeric")
    require(bool(np.isfinite(array).all()), f"{name} contains a non-finite value")


def load_orvd_artifact(directory: Path) -> dict[str, Any]:
    require(directory.is_dir(), f"ORVD artifact directory does not exist: {directory}")
    for filename in ("COMPLETE", "metadata.json", "performance.json",
                     "observations.tsv"):
        require((directory / filename).is_file(),
                f"ORVD artifact is missing {filename}")

    metadata = load_json_object(directory / "metadata.json", "ORVD metadata")
    performance = load_json_object(directory / "performance.json", "ORVD performance")
    require(metadata.get("internal_format_revision") == 3,
            "ORVD metadata is not internal format revision 3")
    require(metadata.get("completed") is True, "ORVD metadata is not complete")
    expected_identity = {
        "vehicle_name": "GZ18",
        "mechanical_definition_identifier": "gz18_reference_mechanics",
        "load_condition_identifier": "gz18_reference_load_condition",
        "wheel_profile_identifier": "gz18_reference_wheel_profile",
        "rail_profile_identifier": "uic60_rail_profile",
        "contact_strategy_identifier": "gz18_reference_wheel_rail_contact",
        "track_irregularity_identifier": "gz18_aar6_reference_irregularity",
        "sample_period_nanoseconds": 500_000,
        "terminal_time_nanoseconds": 10_000_000_000,
        "sample_count": SAMPLE_COUNT,
        "vehicle_layout_reference_track_station_meters": 0,
    }
    for key, value in expected_identity.items():
        require(metadata.get(key) == value,
                f"ORVD metadata {key!r} does not match the G60 identity")
    require(metadata.get("initial_longitudinal_speed_meters_per_second") ==
            16.666666666666668,
            "ORVD metadata has the wrong initial longitudinal speed")
    require(metadata.get("base_track_definition_interval_meters") == [0, 2000],
            "ORVD metadata has the wrong base-track definition interval")
    input_paths = metadata.get("input_paths")
    require(isinstance(input_paths, dict),
            "ORVD metadata input_paths is not an object")
    track_path = input_paths.get("track_geometry")
    require(isinstance(track_path, str) and
            Path(track_path).name == "straight_level_2000m.json",
            "ORVD metadata does not identify the G60 straight track asset")

    complete_text = (directory / "COMPLETE").read_text(encoding="utf-8")
    require(complete_text == f"{SAMPLE_COUNT} samples\n",
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
    require(values.shape == (SAMPLE_COUNT, len(header)),
            "ORVD observations do not have the declared 20001-row fixed width")
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

    expected_index = np.arange(SAMPLE_COUNT, dtype=np.float64)
    require(np.array_equal(columns["sample_index"], expected_index),
            "ORVD sample_index is not the exact integer sequence 0..20000")
    time_error = np.max(np.abs(columns["time_seconds"] - expected_times()))
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


def load_p039_pair(path: Path) -> dict[str, np.ndarray]:
    require(path.is_file(), f"P039 paired reference does not exist: {path}")
    try:
        with np.load(path, allow_pickle=False) as archive:
            require(set(archive.files) == PAIR_KEYS,
                    "P039 paired reference has an unexpected key set")
            result = {key: np.array(archive[key], copy=True) for key in archive.files}
    except (OSError, ValueError) as error:
        raise AnalysisError(f"could not load P039 paired reference: {error}") from error

    require(np.array_equal(result["axle_names"], np.array([a[0] for a in AXLES])),
            "P039 axle names or order differ from ff/fr/rf/rr")
    expected_wheels = np.array([
        f"{axle}_{side[0]}" for axle, _, _ in AXLES for side in SIDES
    ])
    require(np.array_equal(result["wheel_names"], expected_wheels),
            "P039 wheel names or order differ from the named mapping")
    require(result["time_s"].shape == (SAMPLE_COUNT,) and
            np.array_equal(result["time_s"], expected_times()),
            "P039 time_s is not the exact 20001-point integer clock")

    for source in ("simpack", "drake"):
        for quantity in ("station_m", "lateral_m", "yaw_rad"):
            key = f"{source}_{quantity}"
            require(result[key].shape == (SAMPLE_COUNT, 4),
                    f"P039 {key} has the wrong shape")
            require_finite(result[key], f"P039 {key}")
        require(bool(np.all(np.diff(result[f"{source}_station_m"], axis=0) > 0.0)),
                f"P039 {source} stations are not strictly increasing")
        for quantity in CONTACT_QUANTITIES:
            key = f"{source}_contact_{quantity}"
            require(result[key].shape == (SAMPLE_COUNT, 8),
                    f"P039 {key} has the wrong shape")
            require_finite(result[key], f"P039 {key}")
    return result


def load_p038_patch_counts(path: Path, pair: dict[str, np.ndarray]) -> np.ndarray:
    require(path.is_file(), f"P038 native contact reference does not exist: {path}")
    required = {"time_f32"}
    for axle, _, _ in AXLES:
        for side in SIDES:
            wheel = f"{axle}_{side[0]}"
            required.update(f"{wheel}__{name}" for name in
                            ("Q", "N", "Tx", "Ty", "patch_count"))
    try:
        with np.load(path, allow_pickle=False) as archive:
            missing = sorted(required - set(archive.files))
            require(not missing, f"P038 native contact reference is missing {missing}")
            native = {key: np.array(archive[key], copy=True) for key in required}
    except (OSError, ValueError) as error:
        raise AnalysisError(f"could not load P038 native contact reference: {error}") from error

    native_time = native["time_f32"]
    require(native_time.shape == (SAMPLE_COUNT,),
            "P038 native time has the wrong shape")
    require_finite(native["time_f32"], "P038 native time")
    expected_native_time = (
        np.arange(SAMPLE_COUNT, dtype=np.float64) * SAMPLE_PERIOD_SECONDS
    ).astype(np.float32)
    require(np.array_equal(native_time, expected_native_time),
            "P038 native time is not the binary32 cast of the 0.5 ms clock")
    patch_columns = []
    for wheel_index, (axle, _, _) in enumerate(AXLES):
        for side_index, side in enumerate(SIDES):
            wheel = f"{axle}_{side[0]}"
            reference_index = 2 * wheel_index + side_index
            for quantity in ("Q", "N"):
                source = native[f"{wheel}__{quantity}"].astype(np.float64)
                reference = pair[f"simpack_contact_{quantity}"][:, reference_index]
                require(np.array_equal(source, reference),
                        f"P038 {wheel} {quantity} does not match normalized P039")
            for quantity in ("Tx", "Ty"):
                source = -native[f"{wheel}__{quantity}"].astype(np.float64)
                reference = pair[f"simpack_contact_{quantity}"][:, reference_index]
                require(np.array_equal(source, reference),
                        f"P038 {wheel} {quantity} was not normalized exactly once")
            patch_columns.append(native[f"{wheel}__patch_count"].astype(np.float64))
    patches = np.column_stack(patch_columns)
    require_finite(patches, "P038 patch counts")
    require(bool(np.all(patches >= 0.0)) and np.array_equal(patches, np.rint(patches)),
            "P038 patch counts are not non-negative integers")
    return patches.astype(np.int64)


def load_execution_identity(path: Path, artifact_directory: Path) -> dict[str, Any]:
    value = load_json_object(path, "execution identity")
    required = {
        "schema_version", "orvd_revision", "build_type", "compiler",
        "hardware", "requested_cpu_affinity", "applied_cpu_affinity",
        "openmp_environment", "runner_arguments",
        "qualification_artifact_directory",
        "process_wall_seconds", "maximum_resident_set_kilobytes",
        "executable_sha256", "exit_status",
    }
    missing = sorted(required - set(value))
    require(not missing, f"execution identity is missing fields: {missing}")
    require(value["schema_version"] == 1, "execution identity schema is not 1")
    require(value["build_type"] == "Release", "G60 was not run with a Release build")
    require(isinstance(value["openmp_environment"], dict),
            "execution identity openmp_environment is not an object")
    for key in ("orvd_revision", "compiler", "hardware",
                "requested_cpu_affinity",
                "executable_sha256"):
        require(isinstance(value[key], str) and value[key],
                f"execution identity {key!r} is empty")
    for key in ("process_wall_seconds", "maximum_resident_set_kilobytes"):
        require(isinstance(value[key], (int, float)) and
                math.isfinite(value[key]) and value[key] > 0,
                f"execution identity {key!r} is invalid")
    require(value["exit_status"] == 0, "the recorded G60 process did not succeed")
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


def p039_macro_in_orvd_track_frame(
    pair: dict[str, np.ndarray], source: str, quantity: str
) -> np.ndarray:
    """Return P039 macro response in ORVD's standard track frame.

    P039 stores both SIMPACK and historical WRL after the declared
    SIMPACK-ISYS-to-common-W transform diag(1,-1,-1).  ORVD reports lateral
    displacement and yaw in the standard track frame, whose +y and +z axes
    have the opposite signs.  This fixed source-coordinate conversion is
    applied once; it is not inferred from the result and never touches the
    already-normalized contact scalars.
    """
    require(source in ("simpack", "drake"), "unknown P039 macro source")
    require(quantity in ("lateral_m", "yaw_rad"),
            "unknown P039 macro quantity")
    return -pair[f"{source}_{quantity}"]


def error_statistics(candidate: np.ndarray, reference: np.ndarray,
                     abscissa: np.ndarray, stations_candidate: np.ndarray,
                     stations_reference: np.ndarray,
                     *, station_domain: bool = False) -> dict[str, Any]:
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


def same_station_grid(candidate_station: np.ndarray,
                      reference_station: np.ndarray) -> np.ndarray:
    begin = max(PRIMARY_STATION_BEGIN_METERS, float(candidate_station[0]),
                float(reference_station[0]))
    end = min(PRIMARY_STATION_END_METERS, float(candidate_station[-1]),
              float(reference_station[-1]))
    first_centimetre = math.ceil(begin * 100.0 - 1.0e-12)
    last_centimetre = math.floor(end * 100.0 + 1.0e-12)
    require(first_centimetre <= last_centimetre,
            "a wheelset has no common 0.01 m diagnostic station grid")
    return np.arange(first_centimetre, last_centimetre + 1,
                     dtype=np.float64) / 100.0


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


def build_statistics(orvd: dict[str, Any], pair: dict[str, np.ndarray],
                     p038_patches: np.ndarray) -> dict[str, Any]:
    columns = orvd["columns"]
    time = columns["time_seconds"]
    all_indices = np.arange(SAMPLE_COUNT, dtype=np.int64)
    orvd_station = column_stack(columns, "track_station_meters")
    orvd_lateral = column_stack(columns, "lateral_meters")
    orvd_yaw = column_stack(columns, "yaw_radians")
    simpack_station = pair["simpack_station_m"]
    simpack_lateral = p039_macro_in_orvd_track_frame(
        pair, "simpack", "lateral_m"
    )
    simpack_yaw = p039_macro_in_orvd_track_frame(pair, "simpack", "yaw_rad")

    macro: dict[str, Any] = {}
    masks = []
    all_macro_within_alarm = True
    for axle_index, (axle, _, _) in enumerate(AXLES):
        mask = ((orvd_station[:, axle_index] >= PRIMARY_STATION_BEGIN_METERS) &
                (orvd_station[:, axle_index] <= PRIMARY_STATION_END_METERS) &
                (simpack_station[:, axle_index] >= PRIMARY_STATION_BEGIN_METERS) &
                (simpack_station[:, axle_index] <= PRIMARY_STATION_END_METERS))
        masks.append(mask)
        selected = all_indices[mask]
        require(selected.size > 0,
                f"{axle} has no same-time samples in the primary station band")
        axle_result: dict[str, Any] = {
            "same_time_primary_window": {
                "sample_index_begin": int(selected[0]),
                "sample_index_end": int(selected[-1]),
                "sample_count": int(selected.size),
                "time_begin_seconds": float(time[selected[0]]),
                "time_end_seconds": float(time[selected[-1]]),
                "orvd_station_range_meters": [
                    float(orvd_station[selected[0], axle_index]),
                    float(orvd_station[selected[-1], axle_index]),
                ],
                "simpack_station_range_meters": [
                    float(simpack_station[selected[0], axle_index]),
                    float(simpack_station[selected[-1], axle_index]),
                ],
            }
        }
        for quantity, candidate_values, reference_values in (
            ("lateral_meters", orvd_lateral, simpack_lateral),
            ("yaw_radians", orvd_yaw, simpack_yaw),
        ):
            candidate = candidate_values[mask, axle_index]
            reference = reference_values[mask, axle_index]
            candidate_station = orvd_station[mask, axle_index]
            reference_station = simpack_station[mask, axle_index]
            raw = error_statistics(candidate, reference, selected,
                                   candidate_station, reference_station)
            delta = error_statistics(
                candidate - candidate_values[0, axle_index],
                reference - reference_values[0, axle_index], selected,
                candidate_station, reference_station)
            alarm = (raw["rms"] < MACRO_ALARM_LIMIT and
                     raw["maximum_absolute_error"] < MACRO_ALARM_LIMIT)
            all_macro_within_alarm = all_macro_within_alarm and alarm
            axle_result[quantity] = {
                "raw": raw,
                "own_t0_increment": delta,
                "raw_rms_and_max_within_preregistered_alarm": alarm,
            }

            grid = same_station_grid(orvd_station[:, axle_index],
                                     simpack_station[:, axle_index])
            candidate_grid = np.interp(grid, orvd_station[:, axle_index],
                                       candidate_values[:, axle_index])
            reference_grid = np.interp(grid, simpack_station[:, axle_index],
                                       reference_values[:, axle_index])
            axle_result.setdefault("same_station_diagnostic", {})[quantity] = {
                "grid_begin_meters": float(grid[0]),
                "grid_end_meters": float(grid[-1]),
                "grid_step_meters": 0.01,
                "raw": error_statistics(candidate_grid, reference_grid,
                                        grid, grid, grid, station_domain=True),
                "own_t0_increment": error_statistics(
                    candidate_grid - candidate_values[0, axle_index],
                    reference_grid - reference_values[0, axle_index],
                    grid, grid, grid, station_domain=True),
            }
        macro[axle] = axle_result

    orvd_contacts = {
        quantity: orvd_contact(columns, quantity) for quantity in CONTACT_QUANTITIES
    }
    contact: dict[str, Any] = {}
    for wheel_index, wheel in enumerate(pair["wheel_names"].tolist()):
        axle_index = wheel_index // 2
        mask = masks[axle_index]
        selected = all_indices[mask]
        wheel_result: dict[str, Any] = {}
        for quantity in CONTACT_QUANTITIES:
            candidate_all = orvd_contacts[quantity][:, wheel_index]
            reference_all = pair[f"simpack_contact_{quantity}"][:, wheel_index]
            raw = error_statistics(candidate_all[mask], reference_all[mask],
                                   selected, orvd_station[mask, axle_index],
                                   simpack_station[mask, axle_index])
            delta = error_statistics(
                candidate_all[mask] - candidate_all[0],
                reference_all[mask] - reference_all[0], selected,
                orvd_station[mask, axle_index], simpack_station[mask, axle_index])
            grid = same_station_grid(orvd_station[:, axle_index],
                                     simpack_station[:, axle_index])
            candidate_grid = np.interp(grid, orvd_station[:, axle_index], candidate_all)
            reference_grid = np.interp(grid, simpack_station[:, axle_index], reference_all)
            wheel_result[quantity] = {
                "same_time_primary_window": {
                    "raw": raw,
                    "own_t0_increment": delta,
                },
                "same_station_diagnostic": {
                    "grid_begin_meters": float(grid[0]),
                    "grid_end_meters": float(grid[-1]),
                    "grid_step_meters": 0.01,
                    "raw": error_statistics(candidate_grid, reference_grid,
                                            grid, grid, grid,
                                            station_domain=True),
                    "own_t0_increment": error_statistics(
                        candidate_grid - candidate_all[0],
                        reference_grid - reference_all[0], grid, grid, grid,
                        station_domain=True),
                },
            }
        wheel_result["orvd_contact_topology"] = longest_zero_run(
            orvd["patch_count"][:, wheel_index]
        )
        wheel_result["simpack_contact_topology"] = longest_zero_run(
            p038_patches[:, wheel_index]
        )
        contact[str(wheel)] = wheel_result

    computed_longest = [
        longest_zero_run(orvd["patch_count"][:, index])["longest_zero_run_samples"]
        for index in range(8)
    ]
    require(orvd["metadata"].get("longest_zero_contact_run_samples") ==
            computed_longest,
            "ORVD metadata zero-contact runs differ from observations")
    sustained_failures = []
    for index, wheel in enumerate(pair["wheel_names"].tolist()):
        if np.all(orvd["patch_count"][:, index] == 0):
            sustained_failures.append(f"{wheel}: zero contact for the full window")
        if all(np.all(orvd_contacts[q][:, index] == 0.0)
               for q in ("Q", "Tx", "Ty")):
            sustained_failures.append(f"{wheel}: all three force components are zero")

    return {
        "primary_alignment": {
            "kind": "same integer time index",
            "macro_coordinate_frame": MACRO_COORDINATE_FRAME,
            "p039_common_W_to_orvd_track_T": P039_COMMON_W_TO_ORVD_TRACK_T,
            "station_band_meters": [PRIMARY_STATION_BEGIN_METERS,
                                      PRIMARY_STATION_END_METERS],
            "no_time_or_station_shift": True,
            "no_filter_demean_or_result_scaling": True,
        },
        "macro_response": macro,
        "contact_response": contact,
        "macro_raw_rms_and_max_all_within_0p1_mm_or_mrad":
            all_macro_within_alarm,
        "simultaneous_all_eight_zero_contact_sample_count": int(np.count_nonzero(
            np.all(orvd["patch_count"] == 0, axis=1)
        )),
        "sustained_contact_or_force_failures": sustained_failures,
    }


def plot_main_response(path: Path, orvd: dict[str, Any],
                       pair: dict[str, np.ndarray],
                       historical_wrl_revision: str | None) -> None:
    columns = orvd["columns"]
    time = columns["time_seconds"]
    orvd_station = column_stack(columns, "track_station_meters")
    quantities = (
        ("lateral_meters", column_stack(columns, "lateral_meters"),
         p039_macro_in_orvd_track_frame(pair, "simpack", "lateral_m"),
         p039_macro_in_orvd_track_frame(pair, "drake", "lateral_m"), 1.0e3,
         "Lateral displacement [mm]"),
        ("yaw_radians", column_stack(columns, "yaw_radians"),
         p039_macro_in_orvd_track_frame(pair, "simpack", "yaw_rad"),
         p039_macro_in_orvd_track_frame(pair, "drake", "yaw_rad"), 1.0e3,
         "Yaw angle [mrad]"),
    )
    fig, axes = plt.subplots(4, 4, figsize=(18, 13))
    wrl_label = (None if historical_wrl_revision is None else
                 f"Historical WRL {historical_wrl_revision[:8]}")
    colours = {"ORVD": "#0072B2", "SIMPACK": "#D55E00"}
    if wrl_label is not None:
        colours[wrl_label] = "#009E73"
    handles = []
    labels = []
    for axle_index, (axle, _, _) in enumerate(AXLES):
        axes[0, axle_index].set_title(axle.upper())
        for quantity_index, quantity_data in enumerate(quantities):
            _, candidate, reference, historical, scale, label = quantity_data
            time_axis = axes[quantity_index, axle_index]
            station_axis = axes[quantity_index + 2, axle_index]
            plotted = [
                ("ORVD", time, candidate[:, axle_index], orvd_station[:, axle_index]),
                ("SIMPACK", time, reference[:, axle_index],
                 pair["simpack_station_m"][:, axle_index]),
            ]
            if wrl_label is not None:
                plotted.append((wrl_label, time, historical[:, axle_index],
                                pair["drake_station_m"][:, axle_index]))
            for series_label, series_time, values, stations in plotted:
                line, = time_axis.plot(series_time, values * scale,
                                       color=colours[series_label], linewidth=0.85,
                                       label=series_label)
                station_axis.plot(stations, values * scale,
                                  color=colours[series_label], linewidth=0.85)
                if axle_index == 0 and quantity_index == 0:
                    handles.append(line)
                    labels.append(series_label)
            station_axis.axvspan(PRIMARY_STATION_BEGIN_METERS,
                                 PRIMARY_STATION_END_METERS,
                                 color="#BBBBBB", alpha=0.13, linewidth=0)
            if axle_index == 0:
                time_axis.set_ylabel(label)
                station_axis.set_ylabel(label)
            time_axis.grid(True, alpha=0.22)
            station_axis.grid(True, alpha=0.22)
            if quantity_index == 1:
                time_axis.set_xlabel("Time [s]")
                station_axis.set_xlabel("Track station [m]")
    fig.suptitle("G60 GZ18 AAR6 — wheelset response (no shifting or fitting)",
                 y=0.985)
    fig.legend(handles, labels, loc="upper center", ncol=len(labels),
               bbox_to_anchor=(0.5, 0.958), frameon=False)
    fig.subplots_adjust(left=0.065, right=0.985, bottom=0.06, top=0.91,
                        hspace=0.34, wspace=0.25)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_contact_quantity(path: Path, quantity: str, orvd: dict[str, Any],
                          pair: dict[str, np.ndarray],
                          historical_wrl_revision: str | None) -> None:
    columns = orvd["columns"]
    time = columns["time_seconds"]
    stations = column_stack(columns, "track_station_meters")
    candidate = orvd_contact(columns, quantity) / 1000.0
    reference = pair[f"simpack_contact_{quantity}"] / 1000.0
    historical = pair[f"drake_contact_{quantity}"] / 1000.0
    fig, axes = plt.subplots(4, 4, figsize=(18, 13))
    wrl_label = (None if historical_wrl_revision is None else
                 f"Historical WRL {historical_wrl_revision[:8]}")
    colours = {"ORVD": "#0072B2", "SIMPACK": "#D55E00"}
    if wrl_label is not None:
        colours[wrl_label] = "#009E73"
    handles = []
    labels = []
    for axle_index, (axle, _, _) in enumerate(AXLES):
        axes[0, axle_index].set_title(axle.upper())
        for side_index, side in enumerate(SIDES):
            wheel_index = 2 * axle_index + side_index
            for domain_index, domain in enumerate(("time", "station")):
                row = side_index * 2 + domain_index
                axis = axes[row, axle_index]
                x_candidate = time if domain == "time" else stations[:, axle_index]
                x_simpack = time if domain == "time" else pair["simpack_station_m"][:, axle_index]
                x_wrl = time if domain == "time" else pair["drake_station_m"][:, axle_index]
                plotted = [
                    ("ORVD", x_candidate, candidate[:, wheel_index]),
                    ("SIMPACK", x_simpack, reference[:, wheel_index]),
                ]
                if wrl_label is not None:
                    plotted.append((wrl_label, x_wrl,
                                    historical[:, wheel_index]))
                for series_label, x_values, y_values in plotted:
                    line, = axis.plot(x_values, y_values,
                                      color=colours[series_label], linewidth=0.8,
                                      label=series_label)
                    if axle_index == 0 and row == 0:
                        handles.append(line)
                        labels.append(series_label)
                if domain == "station":
                    axis.axvspan(PRIMARY_STATION_BEGIN_METERS,
                                 PRIMARY_STATION_END_METERS,
                                 color="#BBBBBB", alpha=0.13, linewidth=0)
                    axis.set_xlabel("Track station [m]")
                else:
                    axis.set_xlabel("Time [s]")
                if axle_index == 0:
                    axis.set_ylabel(f"{side.capitalize()} {quantity} [kN]")
                axis.grid(True, alpha=0.22)
    fig.suptitle(f"G60 GZ18 AAR6 — wheel-side {quantity} (Q is not N)",
                 y=0.985)
    fig.legend(handles, labels, loc="upper center", ncol=len(labels),
               bbox_to_anchor=(0.5, 0.958), frameon=False)
    fig.subplots_adjust(left=0.065, right=0.985, bottom=0.06, top=0.91,
                        hspace=0.34, wspace=0.25)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def write_json(path: Path, value: dict[str, Any]) -> None:
    try:
        with path.open("w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
    except OSError as error:
        raise AnalysisError(f"could not write '{path}': {error}") from error


def publish_analysis(output: Path, summary: dict[str, Any],
                     orvd: dict[str, Any], pair: dict[str, np.ndarray],
                     historical_wrl_revision: str | None) -> None:
    require(not output.exists(), f"analysis output already exists: {output}")
    partial = output.with_name(output.name + ".partial")
    require(not partial.exists(), f"analysis partial directory already exists: {partial}")
    output.parent.mkdir(parents=True, exist_ok=True)
    partial.mkdir()
    try:
        write_json(partial / "comparison.json", summary)
        plot_main_response(partial / "gz18_g60_wheelset_main_response.png",
                           orvd, pair, historical_wrl_revision)
        for quantity in CONTACT_QUANTITIES:
            plot_contact_quantity(
                partial / f"gz18_g60_contact_{quantity}.png",
                quantity, orvd, pair, historical_wrl_revision)
        (partial / "COMPLETE").write_text(
            "G60 comparison completed\n", encoding="utf-8"
        )
        partial.rename(output)
    except Exception:
        shutil.rmtree(partial, ignore_errors=True)
        raise


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orvd-artifact-directory", type=Path, required=True)
    parser.add_argument("--p039-paired-reference-npz", type=Path, required=True)
    parser.add_argument("--p038-simpack-native-core-npz", type=Path, required=True)
    parser.add_argument("--execution-identity-json", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--include-historical-wrl", action="store_true")
    parser.add_argument("--historical-wrl-source-revision")
    arguments = parser.parse_args(list(argv))
    if arguments.include_historical_wrl and not arguments.historical_wrl_source_revision:
        parser.error("--include-historical-wrl requires --historical-wrl-source-revision")
    return arguments


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    try:
        orvd = load_orvd_artifact(arguments.orvd_artifact_directory.resolve())
        pair = load_p039_pair(arguments.p039_paired_reference_npz.resolve())
        p038_patches = load_p038_patch_counts(
            arguments.p038_simpack_native_core_npz.resolve(), pair
        )
        execution = load_execution_identity(
            arguments.execution_identity_json.resolve(),
            arguments.orvd_artifact_directory.resolve(),
        )
        statistics = build_statistics(orvd, pair, p038_patches)
        performance = orvd["performance"]
        advance_wall = float(performance["advance_wall_seconds"])
        process_wall = float(execution["process_wall_seconds"])
        summary = {
            "schema_version": 1,
            "scope": "G60 GZ18 straight AAR6 10 s activation qualification",
            "scientific_interpretation_requires_curve_review": True,
            "sources": {
                "orvd_artifact_directory": str(arguments.orvd_artifact_directory.resolve()),
                "orvd_observations_sha256": orvd["observation_sha256"],
                "p039_paired_reference": str(arguments.p039_paired_reference_npz.resolve()),
                "p039_paired_reference_sha256": sha256_file(
                    arguments.p039_paired_reference_npz.resolve()
                ),
                "p038_simpack_native_core": str(
                    arguments.p038_simpack_native_core_npz.resolve()
                ),
                "p038_simpack_native_core_sha256": sha256_file(
                    arguments.p038_simpack_native_core_npz.resolve()
                ),
                "historical_wrl_source_revision": (
                    arguments.historical_wrl_source_revision
                    if arguments.include_historical_wrl else None
                ),
                "reference_endpoint_semantics": (
                    "P039 Tx/Ty already converted from SIMPACK Type-80 rail end "
                    "to the canonical wheel end; no second sign conversion"
                ),
                "macro_coordinate_frame": MACRO_COORDINATE_FRAME,
                "p039_common_W_to_orvd_track_T":
                    P039_COMMON_W_TO_ORVD_TRACK_T,
            },
            "execution_identity": execution,
            "orvd_numerical_execution_contract": orvd["metadata"].get(
                "numerical_execution_contract"
            ),
            "performance": {
                **performance,
                "process_wall_seconds": process_wall,
                "maximum_resident_set_kilobytes": execution[
                    "maximum_resident_set_kilobytes"
                ],
                "advance_realtime_factor": TERMINAL_TIME_SECONDS / advance_wall,
                "end_to_end_realtime_factor": TERMINAL_TIME_SECONDS / process_wall,
                "cvode_internal_counters": "unavailable; not inferred",
            },
            "comparison": statistics,
        }
        require(not statistics["sustained_contact_or_force_failures"],
                "sustained contact/force failure: " + "; ".join(
                    statistics["sustained_contact_or_force_failures"]
                ))
        publish_analysis(
            arguments.output_directory.resolve(), summary, orvd, pair,
            (arguments.historical_wrl_source_revision
             if arguments.include_historical_wrl else None),
        )
    except (AnalysisError, OSError) as error:
        print(f"G60 analysis failed: {error}", file=sys.stderr)
        return 1
    print(f"published G60 comparison to {arguments.output_directory.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
