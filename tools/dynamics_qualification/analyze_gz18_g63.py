#!/usr/bin/env python3
"""Analyze the native G63 R300+AAR5 response without fitting the result.

P047 binary64 is the SIMPACK authority for both wheelset response and all
eight Q/N/Tx/Ty contact channels.  P040 binary32 is used only as a source
down-conversion cross-check.  ORVD and SIMPACK are compared on their native
integer sample identities and, independently, on a predeclared 0.01 m station
grid.  The current and historical WRL runs are named controls, never substitutes
for P047.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
import shutil
import sys
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.interpolate import CubicSpline  # noqa: E402

from gz18_qualification_analysis_common import (  # noqa: E402
    AnalysisError,
    AXLES,
    CONTACT_QUANTITIES,
    error_statistics,
    expected_times,
    load_execution_identity,
    load_json_object,
    load_orvd_artifact,
    longest_zero_run,
    orvd_contact,
    require,
    require_finite,
    sha256_file,
    write_json,
)


# ``statistic_pair`` is deliberately local below.  G61's function is not a
# public tool API and importing it would couple two independent source schemas.

SAMPLE_COUNT = 32_001
TERMINAL_TIME_SECONDS = 16.0
TERMINAL_TIME_NANOSECONDS = 16_000_000_000
STATION_STEP_METERS = 0.01
PRIMARY_STATION_BEGIN_METERS = 100.0
PRIMARY_STATION_END_METERS = 250.0
MACRO_ALARM_LIMIT = 1.0e-4
CURRENT_WRL_REVISION = "d7e272df8c29a2074d70eee7cd5a24cfede78d83"
HISTORICAL_WRL_REVISION = "62b0d588b83c7fb6209744ae7ef2afaab01bf71b"
HISTORICAL_WRL_CSV_SHA256 = (
    "cf5c6b90a2b2025471d10816c384f5c04bcdf95bfc542e82d5f8319c724c6843"
)
P047_SBR_SHA256 = "f8673e57c5956467c81c3ba95a677f4efc6422e69b1aa7fc07848726b8bb4b68"
P040_AAR5_CONTENT_SHA256 = (
    "aea7b95f89b805ec40e62e0510924e679468158b70ea8e69b2f4eef14691d10b"
)
WHEEL_NAMES = tuple(f"{axle}_{side}" for axle, _, _ in AXLES for side in ("l", "r"))
SEGMENTS = (
    ("initial_straight", None, 50.0),
    ("transition_without_irregularity", 50.0, 60.0),
    ("transition_and_aar5_fade_in", 60.0, 100.0),
    ("circular_curve_and_aar5_full_amplitude", 100.0, 250.0),
    ("post_primary_window", 250.0, None),
)
P047_KEYS = {
    "time_seconds",
    "axle_names",
    "wheel_names",
    "macro_station_meters",
    "macro_joint_lateral_meters",
    "macro_joint_yaw_radians",
    "macro_lateral_irregularity_meters",
    "contact_station_meters",
    "contact_patch_count",
    "contact_Q_newtons",
    "contact_N_newtons",
    "contact_Tx_on_wheel_newtons",
    "contact_Ty_on_wheel_newtons",
    "channel_roles",
    "channel_paths",
    "channel_descriptions",
    "source_sbr_sha256",
    "source_sbr_precision_bytes",
    "source_sbr_channel_count",
}


def load_g63_orvd(directory: Path) -> dict[str, Any]:
    result = load_orvd_artifact(
        directory,
        goal_name="G63",
        sample_count=SAMPLE_COUNT,
        terminal_time_nanoseconds=TERMINAL_TIME_NANOSECONDS,
        track_irregularity_identifier="gz18_r300_aar5_reference_irregularity",
        track_geometry_filename="r300_centerline_superelevation_1150m.json",
        base_track_definition_interval_meters=(0, 1150),
    )
    missing: list[str] = []
    for _, body, _ in AXLES:
        for side in ("left", "right"):
            name = (
                f"{body}.{side}." "rail_profile_reference_marker_track_station_meters"
            )
            if name not in result["columns"]:
                missing.append(name)
    require(not missing, f"G63 ORVD observations are missing columns: {missing}")
    return result


def load_p047(path: Path) -> dict[str, np.ndarray]:
    require(path.is_file(), f"P047 compact authority does not exist: {path}")
    try:
        with np.load(path, allow_pickle=False) as archive:
            require(
                set(archive.files) == P047_KEYS,
                "P047 compact authority has an unexpected key set",
            )
            result = {key: np.array(archive[key], copy=True) for key in archive.files}
    except (OSError, ValueError) as error:
        raise AnalysisError(f"could not load P047 authority: {error}") from error
    require(
        np.array_equal(
            result["axle_names"], np.asarray([axle for axle, _, _ in AXLES])
        ),
        "P047 axle names or order differ from ff/fr/rf/rr",
    )
    require(
        np.array_equal(result["wheel_names"], np.asarray(WHEEL_NAMES)),
        "P047 wheel names or order differ from axle-major left/right",
    )
    require(
        result["source_sbr_sha256"].shape == ()
        and result["source_sbr_sha256"].item() == P047_SBR_SHA256,
        "P047 compact authority names a different source SBR",
    )
    require(
        int(result["source_sbr_precision_bytes"]) == 8
        and int(result["source_sbr_channel_count"]) == 5_162,
        "P047 source is not the qualified 5162-channel binary64 result",
    )
    require(
        np.array_equal(
            result["time_seconds"], expected_times(SAMPLE_COUNT, TERMINAL_TIME_SECONDS)
        ),
        "P047 time is not the exact 0--16 s / 0.5 ms clock",
    )
    shapes = {
        "macro_station_meters": (SAMPLE_COUNT, 4),
        "macro_joint_lateral_meters": (SAMPLE_COUNT, 4),
        "macro_joint_yaw_radians": (SAMPLE_COUNT, 4),
        "macro_lateral_irregularity_meters": (SAMPLE_COUNT, 4),
        "contact_station_meters": (SAMPLE_COUNT, 8),
        "contact_patch_count": (SAMPLE_COUNT, 8),
        "contact_Q_newtons": (SAMPLE_COUNT, 8),
        "contact_N_newtons": (SAMPLE_COUNT, 8),
        "contact_Tx_on_wheel_newtons": (SAMPLE_COUNT, 8),
        "contact_Ty_on_wheel_newtons": (SAMPLE_COUNT, 8),
    }
    for key, shape in shapes.items():
        require(result[key].shape == shape, f"P047 {key} has the wrong shape")
        require_finite(result[key], f"P047 {key}")
    require(
        bool(np.all(np.diff(result["macro_station_meters"], axis=0) > 0.0)),
        "P047 macro station is not strictly increasing",
    )
    require(
        bool(np.all(np.diff(result["contact_station_meters"], axis=0) > 0.0)),
        "P047 per-wheel contact station is not strictly increasing",
    )
    require(
        np.array_equal(
            result["contact_patch_count"], np.ones((SAMPLE_COUNT, 8), dtype=np.int64)
        ),
        "P047 qualified run is not eight-wheel single-patch throughout",
    )
    require(
        result["channel_roles"].shape == (64,)
        and result["channel_paths"].shape == (64,)
        and result["channel_descriptions"].shape == (64,),
        "P047 selected-channel ledger does not contain 64 named channels",
    )
    return result


def cross_check_p040(path: Path, p047: dict[str, np.ndarray]) -> dict[str, Any]:
    require(path.is_file(), f"P040 native core does not exist: {path}")
    try:
        with np.load(path, allow_pickle=False) as archive:
            p040 = {key: np.array(archive[key], copy=True) for key in archive.files}
    except (OSError, ValueError) as error:
        raise AnalysisError(f"could not load P040 native core: {error}") from error
    expected_time = (
        np.arange(SAMPLE_COUNT, dtype=np.float64) * np.float64(0.0005)
    ).astype(np.float32)
    require(
        "time_f32" in p040 and np.array_equal(p040["time_f32"], expected_time),
        "P040 time_f32 differs from the declared binary32 sample identity",
    )
    p047_keys = {
        "Q": "contact_Q_newtons",
        "N": "contact_N_newtons",
        "Tx": "contact_Tx_on_wheel_newtons",
        "Ty": "contact_Ty_on_wheel_newtons",
        "patch_count": "contact_patch_count",
    }
    checks: dict[str, bool] = {}
    for wheel_index, wheel in enumerate(WHEEL_NAMES):
        for quantity, p047_key in p047_keys.items():
            key = f"{wheel}__{quantity}"
            require(
                key in p040 and p040[key].shape == (SAMPLE_COUNT,),
                f"P040 native core is missing {key}",
            )
            reference = p047[p047_key][:, wheel_index].astype(np.float32)
            if quantity in ("Tx", "Ty"):
                # P040 preserves SIMPACK Type-80's rail-end sign.  P047 compact
                # has already converted once to the canonical wheel end.
                reference = -reference
            checks[key] = bool(np.array_equal(p040[key], reference))
    require(
        all(checks.values()),
        "P047 binary64 down-conversion differs from P040 binary32 core",
    )
    return {
        "clock_binary32_identical": True,
        "physical_contact_channels_checked": len(checks),
        "all_checked_channels_binary32_identical": True,
        "p040_role": "source/down-conversion cross-check; it supplies no missing G63 column",
    }


class ZeroOutsideNaturalSpline:
    def __init__(self, path: Path, expected_identifier: str):
        value = load_json_object(path, "AAR5 lateral series")
        require(
            set(value)
            == {"series_identifier", "track_station_meters", "displacement_meters"},
            "AAR5 lateral series has an unexpected key set",
        )
        require(
            value["series_identifier"] == expected_identifier,
            "AAR5 lateral series has the wrong identifier",
        )
        self.x = np.asarray(value["track_station_meters"], dtype=np.float64)
        self.y = np.asarray(value["displacement_meters"], dtype=np.float64)
        require(
            self.x.shape == self.y.shape == (37_620,),
            "AAR5 lateral series does not contain 37620 points",
        )
        require_finite(self.x, "AAR5 station")
        require_finite(self.y, "AAR5 lateral displacement")
        require(
            bool(np.all(np.diff(self.x) > 0.0)),
            "AAR5 station is not strictly increasing",
        )
        require(
            self.x[0] == 60.0 and self.x[-1] == 1000.0,
            "AAR5 lateral support is not [60,1000] m",
        )
        self.spline = CubicSpline(self.x, self.y, bc_type="natural", extrapolate=False)

    def evaluate(self, station: np.ndarray, derivative: int = 0) -> np.ndarray:
        station = np.asarray(station, dtype=np.float64)
        result = np.zeros_like(station)
        inside = (station >= self.x[0]) & (station <= self.x[-1])
        result[inside] = self.spline(station[inside], derivative)
        require_finite(result, "evaluated AAR5 field")
        return result


def read_csv(path: Path, name: str) -> dict[str, np.ndarray]:
    require(path.is_file(), f"{name} CSV does not exist: {path}")
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            header = next(csv.reader(stream))
        values = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.float64)
    except (OSError, StopIteration, ValueError) as error:
        raise AnalysisError(f"could not read {name} CSV: {error}") from error
    if values.ndim == 1:
        values = values.reshape(1, -1)
    require(len(header) == len(set(header)), f"{name} columns are not unique")
    require(
        values.ndim == 2 and values.shape[1] == len(header),
        f"{name} CSV width differs from its header",
    )
    require_finite(values, f"{name} CSV")
    return {column: values[:, index] for index, column in enumerate(header)}


def load_wrl(
    csv_path: Path,
    *,
    name: str,
    expected_rows: int,
    expected_last_time: float,
) -> dict[str, Any]:
    columns = read_csv(csv_path, name)
    required = {"t"}
    for axle, _, _ in AXLES:
        required.update(
            {
                f"s_wheelset_{axle}_m",
                f"y_wheelset_{axle}_m",
                f"yaw_wheelset_{axle}_rad",
            }
        )
        for side in ("l", "r"):
            required.update(
                {
                    f"n_patches_wheel_{axle}_{side}",
                    f"N_maxN_patch_wheel_{axle}_{side}_N",
                    f"Tx_maxN_patch_wheel_{axle}_{side}_N",
                    f"Ty_maxN_patch_wheel_{axle}_{side}_N",
                }
            )
    missing = sorted(required - set(columns))
    require(not missing, f"{name} is missing columns: {missing}")
    time = columns["t"]
    require(
        time.shape == (expected_rows,)
        and time[0] == 0.0
        and abs(float(time[-1]) - expected_last_time) <= 1.0e-10,
        f"{name} has the wrong native time extent",
    )
    native = np.arange(expected_rows, dtype=np.float64) * np.float64(0.0005)
    require(
        float(np.max(np.abs(time - native))) <= 1.0e-10,
        f"{name} time does not resolve to the 0.5 ms native clock",
    )
    station = np.column_stack([columns[f"s_wheelset_{axle}_m"] for axle, _, _ in AXLES])
    require(
        bool(np.all(np.diff(station, axis=0) > 0.0)),
        f"{name} wheelset stations are not strictly increasing",
    )
    patch = np.column_stack(
        [
            columns[f"n_patches_wheel_{axle}_{side}"]
            for axle, _, _ in AXLES
            for side in ("l", "r")
        ]
    )
    require(
        np.array_equal(patch, np.rint(patch)) and bool(np.all(patch >= 0.0)),
        f"{name} patch counts are not non-negative integers",
    )
    return {
        "columns": columns,
        "time_seconds": time,
        "station_meters": station,
        "patch_count": patch.astype(np.int64),
    }


def validate_current_wrl_identity(
    path: Path, csv_path: Path, executable: Path, source_revision: str
) -> dict[str, Any]:
    require(
        source_revision == CURRENT_WRL_REVISION,
        "current WRL source revision is not d7e272d",
    )
    value = load_json_object(path, "current WRL run identity")
    require(
        set(value) == {"timestamp", "args", "result", "startup_identity", "static_eq"},
        "current WRL run identity has an unexpected top-level key set",
    )
    args = value.get("args")
    result = value.get("result")
    require(
        isinstance(args, dict) and isinstance(result, dict),
        "current WRL identity args/result are not objects",
    )
    expected_args = {
        "vehicle_type": "gz18",
        "track_preset": "crv300m",
        "initial_speed_kmph": 60.0,
        "sim_duration_s": 16.0,
        "sim_stepsize_s": 0.0001,
        "log_hz": 2000.0,
        "integrator_method": "radau3",
        "integrator_fixed_step": True,
        "integrator_target_accuracy": 0.0001,
        "contact_update_mode": "rhs_state_eval",
        "rhs_state_eval_backend": "drake_radau3",
        "rhs_state_eval_hint_mode": "numeric_parameter",
        "rhs_state_eval_hint_update_stride_effective": 1,
        "track_irregularity_source_kind": "external_pair",
        "track_irregularity_source_content_sha256": P040_AAR5_CONTENT_SHA256,
        "track_irregularity_lateral_node_count": 37_620,
        "track_irregularity_vertical_node_count": 37_620,
    }
    for key, expected in expected_args.items():
        require(
            args.get(key) == expected,
            f"current WRL argument {key!r} differs from G63 identity",
        )
    require(
        result.get("completed") is True
        and result.get("scientific_archive_sample_count") == SAMPLE_COUNT
        and abs(float(result.get("scientific_archive_last_time_s")) - 16.0) <= 1.0e-10,
        "current WRL did not publish a complete 32001-point run",
    )
    require(
        Path(str(result.get("csv_path"))).resolve() == csv_path.resolve(),
        "current WRL identity names a different CSV",
    )
    require(executable.is_file(), f"current WRL executable is absent: {executable}")
    return {
        "source_revision": source_revision,
        "executable_sha256": sha256_file(executable),
        "run_identity_sha256": sha256_file(path),
        "csv_sha256": sha256_file(csv_path),
        "internal_simulation_wall_seconds": float(result["wall_time_s"]),
        "integrator": {
            key: args[key]
            for key in (
                "integrator_method",
                "integrator_fixed_step",
                "sim_stepsize_s",
                "integrator_target_accuracy",
                "contact_update_mode",
                "rhs_state_eval_backend",
                "rhs_state_eval_hint_mode",
                "rhs_state_eval_hint_update_stride_effective",
            )
        },
        "openmp_environment": {
            "RWC_CONTACT_BATCH_MODE": "omp",
            "OMP_NUM_THREADS": "8",
            "OMP_DYNAMIC": "FALSE",
            "OMP_PLACES": "cores",
            "OMP_PROC_BIND": "close",
        },
        "window_statistics": result.get("cvode_window_stats"),
    }


def parse_time_report(path: Path) -> dict[str, Any]:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        raise AnalysisError(f"could not read WRL time report: {error}") from error
    elapsed_match = re.search(r"Elapsed \(wall clock\) time.*: (\d+):(\d+\.\d+)", text)
    rss_match = re.search(r"Maximum resident set size \(kbytes\): (\d+)", text)
    cpu_match = re.search(r"Percent of CPU this job got: (\d+)%", text)
    status_match = re.search(r"Exit status: (\d+)", text)
    require(
        all(
            match is not None
            for match in (elapsed_match, rss_match, cpu_match, status_match)
        ),
        "WRL time report is missing required fields",
    )
    assert elapsed_match is not None and rss_match is not None
    assert cpu_match is not None and status_match is not None
    wall = float(elapsed_match.group(1)) * 60.0 + float(elapsed_match.group(2))
    require(
        int(status_match.group(1)) == 0 and wall > 0.0,
        "WRL outer process did not complete successfully",
    )
    return {
        "process_wall_seconds": wall,
        "simulation_to_wall_factor": TERMINAL_TIME_SECONDS / wall,
        "maximum_resident_set_kilobytes": int(rss_match.group(1)),
        "average_cpu_percent": int(cpu_match.group(1)),
        "time_report_sha256": sha256_file(path),
    }


def orvd_station(columns: dict[str, np.ndarray]) -> np.ndarray:
    return np.column_stack(
        [columns[f"{body}.track_station_meters"] for _, body, _ in AXLES]
    )


def orvd_contact_station(columns: dict[str, np.ndarray]) -> np.ndarray:
    return np.column_stack(
        [
            columns[
                f"{body}.{'left' if side == 'l' else 'right'}."
                "rail_profile_reference_marker_track_station_meters"
            ]
            for _, body, _ in AXLES
            for side in ("l", "r")
        ]
    )


def macro_arrays(
    orvd: dict[str, Any],
    p047: dict[str, np.ndarray],
    current: dict[str, Any],
    historical: dict[str, Any],
    field: ZeroOutsideNaturalSpline,
) -> dict[str, Any]:
    columns = orvd["columns"]
    station_orvd = (
        orvd_contact_station(columns).reshape(SAMPLE_COUNT, 4, 2).mean(axis=2)
    )
    lateral_orvd: list[np.ndarray] = []
    yaw_orvd: list[np.ndarray] = []
    body_station_sensitivity: list[dict[str, float]] = []
    for _, body, _ in AXLES:
        left = columns[
            f"{body}.left.rail_profile_reference_marker_track_station_meters"
        ]
        right = columns[
            f"{body}.right.rail_profile_reference_marker_track_station_meters"
        ]
        mean_y = 0.5 * (field.evaluate(left) + field.evaluate(right))
        mean_slope = 0.5 * (field.evaluate(left, 1) + field.evaluate(right, 1))
        base_lateral = columns[f"{body}.lateral_meters"]
        base_yaw = columns[f"{body}.yaw_radians"]
        lateral_orvd.append(base_lateral - mean_y)
        yaw_orvd.append(-base_yaw + np.arctan(mean_slope))
        body_s = columns[f"{body}.track_station_meters"]
        body_station_sensitivity.append(
            {
                "lateral_maximum_absolute_difference_meters": float(
                    np.max(
                        np.abs(
                            (base_lateral - field.evaluate(body_s)) - lateral_orvd[-1]
                        )
                    )
                ),
                "yaw_maximum_absolute_difference_radians": float(
                    np.max(
                        np.abs(
                            (-base_yaw + np.arctan(field.evaluate(body_s, 1)))
                            - yaw_orvd[-1]
                        )
                    )
                ),
            }
        )
    station_simpack = p047["macro_station_meters"]
    irregularity_simpack = p047["macro_lateral_irregularity_meters"]
    heading_simpack = np.empty_like(irregularity_simpack)
    for axle_index in range(4):
        heading_simpack[:, axle_index] = np.arctan(
            CubicSpline(
                station_simpack[:, axle_index],
                irregularity_simpack[:, axle_index],
                bc_type="natural",
            )(station_simpack[:, axle_index], 1)
        )
    columns_current = current["columns"]
    columns_historical = historical["columns"]
    current_lateral = []
    current_yaw = []
    historical_lateral = []
    historical_yaw = []
    for axle, _, _ in AXLES:
        current_s = columns_current[f"s_wheelset_{axle}_m"]
        current_lateral.append(
            columns_current[f"y_wheelset_{axle}_m"] - field.evaluate(current_s)
        )
        current_yaw.append(
            -columns_current[f"yaw_wheelset_{axle}_rad"]
            + np.arctan(field.evaluate(current_s, 1))
        )
        historical_s = columns_historical[f"s_wheelset_{axle}_m"]
        historical_lateral.append(
            columns_historical[f"y_wheelset_{axle}_m"] - field.evaluate(historical_s)
        )
        historical_yaw.append(
            -columns_historical[f"yaw_wheelset_{axle}_rad"]
            + np.arctan(field.evaluate(historical_s, 1))
        )
    return {
        "orvd_station": station_orvd,
        "orvd_lateral": np.column_stack(lateral_orvd),
        "orvd_yaw": np.column_stack(yaw_orvd),
        "simpack_station": station_simpack,
        "simpack_lateral": (p047["macro_joint_lateral_meters"] - irregularity_simpack),
        "simpack_yaw": -p047["macro_joint_yaw_radians"] + heading_simpack,
        "current_wrl_station": current["station_meters"],
        "current_wrl_lateral": np.column_stack(current_lateral),
        "current_wrl_yaw": np.column_stack(current_yaw),
        "historical_wrl_station": historical["station_meters"],
        "historical_wrl_lateral": np.column_stack(historical_lateral),
        "historical_wrl_yaw": np.column_stack(historical_yaw),
        "orvd_body_station_transform_sensitivity": body_station_sensitivity,
    }


def contact_arrays(
    orvd: dict[str, Any],
    p047: dict[str, np.ndarray],
    current: dict[str, Any],
    historical: dict[str, Any],
) -> dict[str, Any]:
    current_columns = current["columns"]
    historical_columns = historical["columns"]
    result: dict[str, Any] = {
        "orvd_station": orvd_contact_station(orvd["columns"]),
        "simpack_station": p047["contact_station_meters"],
        "current_wrl_station": np.repeat(current["station_meters"], 2, axis=1),
        "historical_wrl_station": np.repeat(historical["station_meters"], 2, axis=1),
        "orvd_patch_count": orvd["patch_count"],
        "simpack_patch_count": p047["contact_patch_count"],
        "current_wrl_patch_count": current["patch_count"],
        "historical_wrl_patch_count": historical["patch_count"],
    }
    p047_keys = {
        "Q": "contact_Q_newtons",
        "N": "contact_N_newtons",
        "Tx": "contact_Tx_on_wheel_newtons",
        "Ty": "contact_Ty_on_wheel_newtons",
    }
    for quantity in CONTACT_QUANTITIES:
        result[f"orvd_{quantity}"] = orvd_contact(orvd["columns"], quantity)
        result[f"simpack_{quantity}"] = p047[p047_keys[quantity]]
        if quantity != "Q":
            result[f"current_wrl_{quantity}"] = np.column_stack(
                [
                    current_columns[f"{quantity}_maxN_patch_wheel_{wheel}_N"]
                    for wheel in WHEEL_NAMES
                ]
            )
            result[f"historical_wrl_{quantity}"] = np.column_stack(
                [
                    historical_columns[f"{quantity}_maxN_patch_wheel_{wheel}_N"]
                    for wheel in WHEEL_NAMES
                ]
            )
    return result


def segment_mask(
    station: np.ndarray, begin: float | None, end: float | None
) -> np.ndarray:
    result = np.ones(station.shape, dtype=bool)
    if begin is not None:
        result &= station >= begin
    if end is not None:
        result &= station < end
    return result


def statistic_pair(
    candidate: np.ndarray,
    reference: np.ndarray,
    abscissa: np.ndarray,
    candidate_station: np.ndarray,
    reference_station: np.ndarray,
    *,
    candidate_t0: float,
    reference_t0: float,
    station_domain: bool,
) -> dict[str, Any]:
    return {
        "raw": error_statistics(
            candidate,
            reference,
            abscissa,
            candidate_station,
            reference_station,
            station_domain=station_domain,
        ),
        "own_t0_increment": error_statistics(
            candidate - candidate_t0,
            reference - reference_t0,
            abscissa,
            candidate_station,
            reference_station,
            station_domain=station_domain,
        ),
    }


def alarm_excursion(error: np.ndarray, grid: np.ndarray) -> dict[str, Any]:
    over = np.abs(error) > MACRO_ALARM_LIMIT
    best_begin: int | None = None
    best_end: int | None = None
    run_begin: int | None = None
    for index, active in enumerate(over):
        if active and run_begin is None:
            run_begin = index
        if run_begin is not None and (not active or index == over.size - 1):
            run_end = index if active and index == over.size - 1 else index - 1
            if best_begin is None or run_end - run_begin > best_end - best_begin:
                best_begin, best_end = run_begin, run_end
            run_begin = None
    if best_begin is None or best_end is None:
        return {
            "sample_count_over_alarm": 0,
            "longest_contiguous_sample_count": 0,
            "longest_contiguous_length_meters": 0.0,
            "longest_begin_meters": None,
            "longest_end_meters": None,
        }
    longest_sample_count = best_end - best_begin + 1
    return {
        "sample_count_over_alarm": int(np.count_nonzero(over)),
        "longest_contiguous_sample_count": int(longest_sample_count),
        "longest_contiguous_length_meters": float(grid[best_end] - grid[best_begin]),
        "longest_begin_meters": float(grid[best_begin]),
        "longest_end_meters": float(grid[best_end]),
    }


def primary_grid() -> np.ndarray:
    return (
        np.arange(
            int(round(PRIMARY_STATION_BEGIN_METERS / STATION_STEP_METERS)),
            int(round(PRIMARY_STATION_END_METERS / STATION_STEP_METERS)) + 1,
            dtype=np.float64,
        )
        * STATION_STEP_METERS
    )


def build_statistics(
    orvd: dict[str, Any],
    macro: dict[str, Any],
    contact: dict[str, Any],
    current: dict[str, Any],
    historical: dict[str, Any],
) -> dict[str, Any]:
    indices = np.arange(SAMPLE_COUNT, dtype=np.int64)
    grid = primary_grid()
    macro_result: dict[str, Any] = {}
    all_within_alarm = True
    all_rms_within_alarm = True
    for axle_index, (axle, _, _) in enumerate(AXLES):
        axle_result: dict[str, Any] = {
            "same_station_100_to_250_meters": {},
            "same_time_segments": {},
            "current_wrl_control": {},
            "historical_wrl_control": {},
        }
        for quantity in ("lateral", "yaw"):
            candidate_all = macro[f"orvd_{quantity}"][:, axle_index]
            reference_all = macro[f"simpack_{quantity}"][:, axle_index]
            candidate_grid = np.interp(
                grid, macro["orvd_station"][:, axle_index], candidate_all
            )
            reference_grid = np.interp(
                grid, macro["simpack_station"][:, axle_index], reference_all
            )
            stats = statistic_pair(
                candidate_grid,
                reference_grid,
                grid,
                grid,
                grid,
                candidate_t0=candidate_all[0],
                reference_t0=reference_all[0],
                station_domain=True,
            )
            stats["raw_alarm_excursion"] = alarm_excursion(
                candidate_grid - reference_grid, grid
            )
            stats["own_t0_increment_alarm_excursion"] = alarm_excursion(
                (candidate_grid - candidate_all[0])
                - (reference_grid - reference_all[0]),
                grid,
            )
            stats["rms_and_max_raw_and_increment_within_0p1_mm_or_mrad"] = all(
                stats[form][metric] <= MACRO_ALARM_LIMIT
                for form in ("raw", "own_t0_increment")
                for metric in ("rms", "maximum_absolute_error")
            )
            all_within_alarm &= bool(
                stats["rms_and_max_raw_and_increment_within_0p1_mm_or_mrad"]
            )
            all_rms_within_alarm &= all(
                stats[form]["rms"] <= MACRO_ALARM_LIMIT
                for form in ("raw", "own_t0_increment")
            )
            axle_result["same_station_100_to_250_meters"][quantity] = stats
            current_values = macro[f"current_wrl_{quantity}"][:, axle_index]
            current_grid = np.interp(
                grid, macro["current_wrl_station"][:, axle_index], current_values
            )
            axle_result["current_wrl_control"][quantity] = statistic_pair(
                candidate_grid,
                current_grid,
                grid,
                grid,
                grid,
                candidate_t0=candidate_all[0],
                reference_t0=current_values[0],
                station_domain=True,
            )
            historical_values = macro[f"historical_wrl_{quantity}"][:, axle_index]
            historical_grid = np.interp(
                grid,
                macro["historical_wrl_station"][:, axle_index],
                historical_values,
            )
            axle_result["historical_wrl_control"][quantity] = statistic_pair(
                candidate_grid,
                historical_grid,
                grid,
                grid,
                grid,
                candidate_t0=candidate_all[0],
                reference_t0=historical_values[0],
                station_domain=True,
            )
        for name, begin, end in SEGMENTS:
            mask = segment_mask(
                macro["orvd_station"][:, axle_index], begin, end
            ) & segment_mask(macro["simpack_station"][:, axle_index], begin, end)
            selected = indices[mask]
            if selected.size == 0:
                continue
            axle_result["same_time_segments"][name] = {
                quantity: statistic_pair(
                    macro[f"orvd_{quantity}"][mask, axle_index],
                    macro[f"simpack_{quantity}"][mask, axle_index],
                    selected,
                    macro["orvd_station"][mask, axle_index],
                    macro["simpack_station"][mask, axle_index],
                    candidate_t0=macro[f"orvd_{quantity}"][0, axle_index],
                    reference_t0=macro[f"simpack_{quantity}"][0, axle_index],
                    station_domain=False,
                )
                for quantity in ("lateral", "yaw")
            }
        macro_result[axle] = axle_result

    contact_result: dict[str, Any] = {}
    for wheel_index, wheel in enumerate(WHEEL_NAMES):
        wheel_result: dict[str, Any] = {
            "same_station_100_to_250_meters": {},
            "same_time_full_window": {},
            "orvd_topology": longest_zero_run(
                contact["orvd_patch_count"][:, wheel_index]
            ),
            "simpack_topology": longest_zero_run(
                contact["simpack_patch_count"][:, wheel_index]
            ),
            "current_wrl_topology": longest_zero_run(
                contact["current_wrl_patch_count"][:, wheel_index]
            ),
            "historical_wrl_topology": longest_zero_run(
                contact["historical_wrl_patch_count"][:, wheel_index]
            ),
        }
        for quantity in CONTACT_QUANTITIES:
            candidate = contact[f"orvd_{quantity}"][:, wheel_index]
            reference = contact[f"simpack_{quantity}"][:, wheel_index]
            candidate_grid = np.interp(
                grid, contact["orvd_station"][:, wheel_index], candidate
            )
            reference_grid = np.interp(
                grid, contact["simpack_station"][:, wheel_index], reference
            )
            wheel_result["same_station_100_to_250_meters"][quantity] = statistic_pair(
                candidate_grid,
                reference_grid,
                grid,
                grid,
                grid,
                candidate_t0=candidate[0],
                reference_t0=reference[0],
                station_domain=True,
            )
            wheel_result["same_time_full_window"][quantity] = statistic_pair(
                candidate,
                reference,
                indices,
                contact["orvd_station"][:, wheel_index],
                contact["simpack_station"][:, wheel_index],
                candidate_t0=candidate[0],
                reference_t0=reference[0],
                station_domain=False,
            )
            if quantity != "Q":
                current_values = contact[f"current_wrl_{quantity}"][:, wheel_index]
                wheel_result.setdefault("orvd_minus_current_wrl", {})[quantity] = (
                    statistic_pair(
                        candidate,
                        current_values,
                        indices,
                        contact["orvd_station"][:, wheel_index],
                        contact["current_wrl_station"][:, wheel_index],
                        candidate_t0=candidate[0],
                        reference_t0=current_values[0],
                        station_domain=False,
                    )
                )
        contact_result[wheel] = wheel_result

    sustained_failures: list[str] = []
    for wheel_index, wheel in enumerate(WHEEL_NAMES):
        for source in ("orvd", "simpack", "current_wrl"):
            patch = contact[f"{source}_patch_count"][:, wheel_index]
            if np.all(patch == 0):
                sustained_failures.append(
                    f"{source}:{wheel}: no contact for full window"
                )
        if all(
            np.all(contact[f"orvd_{quantity}"][:, wheel_index] == 0.0)
            for quantity in ("Q", "Tx", "Ty")
        ):
            sustained_failures.append(
                f"orvd:{wheel}: all three force components are zero"
            )
    return {
        "primary_alignment": {
            "same_time": "native integer sample index; no interpolation",
            "same_station": "each source independently interpolated to 100--250 m / 0.01 m",
            "macro_observer": (
                "wheelset rigid-body response against the mean left/right "
                "rail-profile reference station"
            ),
            "force_observer": "each wheel's own rail-profile reference station",
            "no_time_or_station_shift": True,
            "no_filter_demean_fit_scaling_or_result_dependent_sign_change": True,
        },
        "macro_response": macro_result,
        "contact_response": contact_result,
        "macro_all_axes_raw_and_increment_rms_and_max_within_0p1_mm_or_mrad": all_within_alarm,
        "macro_all_axes_raw_and_increment_rms_within_0p1_mm_or_mrad": all_rms_within_alarm,
        "sustained_contact_or_force_failures": sustained_failures,
        "orvd_longest_zero_contact_run_matches_metadata": (
            orvd["metadata"].get("longest_zero_contact_run_samples")
            == [
                longest_zero_run(orvd["patch_count"][:, index])[
                    "longest_zero_run_samples"
                ]
                for index in range(8)
            ]
        ),
        "current_wrl_native_sample_count": int(current["time_seconds"].size),
        "historical_wrl_native_sample_count": int(historical["time_seconds"].size),
    }


def add_station_bands(axis: Any) -> None:
    for begin, end, colour in (
        (50.0, 60.0, "#F0E442"),
        (60.0, 100.0, "#E69F00"),
        (100.0, 250.0, "#009E73"),
    ):
        axis.axvspan(begin, end, color=colour, alpha=0.07, linewidth=0)


def direct_wrl_control_consistency(
    macro: dict[str, Any],
    contact: dict[str, Any],
) -> dict[str, Any]:
    """Summarize the current-vs-historical WRL control on their native clock."""
    common_count = 32_000

    def summary(left: np.ndarray, right: np.ndarray) -> dict[str, float]:
        difference = np.asarray(left[:common_count] - right[:common_count])
        return {
            "rms": float(np.sqrt(np.mean(difference * difference))),
            "maximum_absolute_difference": float(np.max(np.abs(difference))),
        }

    return {
        "sample_count": common_count,
        "last_time_seconds": 15.9995,
        "wheelset_station_meters": summary(
            macro["current_wrl_station"], macro["historical_wrl_station"]
        ),
        "wheelset_lateral_meters": summary(
            macro["current_wrl_lateral"], macro["historical_wrl_lateral"]
        ),
        "wheelset_yaw_radians": summary(
            macro["current_wrl_yaw"], macro["historical_wrl_yaw"]
        ),
        "contact_patch_count_identical": bool(
            np.array_equal(
                contact["current_wrl_patch_count"][:common_count],
                contact["historical_wrl_patch_count"],
            )
        ),
        **{
            f"contact_{quantity}_newtons": summary(
                contact[f"current_wrl_{quantity}"],
                contact[f"historical_wrl_{quantity}"],
            )
            for quantity in ("N", "Tx", "Ty")
        },
    }


def current_wrl_simpack_force_difference(
    contact: dict[str, Any],
) -> dict[str, dict[str, float]]:
    """Quantify whether the ORVD force residual is inherited from current WRL."""
    result: dict[str, dict[str, float]] = {}
    for quantity in ("N", "Tx", "Ty"):
        difference = contact[f"current_wrl_{quantity}"] - contact[f"simpack_{quantity}"]
        result[quantity] = {
            "rms_newtons": float(np.sqrt(np.mean(difference * difference))),
            "maximum_absolute_difference_newtons": float(np.max(np.abs(difference))),
        }
    return result


def plot_macro(path: Path, macro: dict[str, Any]) -> None:
    fig, axes = plt.subplots(4, 4, figsize=(20, 13.5))
    time = expected_times(SAMPLE_COUNT, TERMINAL_TIME_SECONDS)
    colours = {"ORVD": "#0072B2", "SIMPACK": "#D55E00"}
    handles: list[Any] = []
    labels: list[str] = []
    grid = primary_grid()
    for axle_index, (axle, _, _) in enumerate(AXLES):
        axes[0, axle_index].set_title(axle.upper())
        for quantity_index, (quantity, scale, ylabel) in enumerate(
            (
                ("lateral", 1.0e3, "Lateral displacement [mm]"),
                ("yaw", 1.0e3, "Yaw angle [mrad]"),
            )
        ):
            time_axis = axes[quantity_index, axle_index]
            station_axis = axes[quantity_index + 2, axle_index]
            for label, key in (("ORVD", "orvd"), ("SIMPACK", "simpack")):
                (line,) = time_axis.plot(
                    time,
                    macro[f"{key}_{quantity}"][:, axle_index] * scale,
                    color=colours[label],
                    linewidth=0.78,
                    linestyle="-" if label == "ORVD" else "--",
                    label=label,
                )
                if axle_index == 0 and quantity_index == 0:
                    handles.append(line)
                    labels.append(label)
                station_axis.plot(
                    grid,
                    np.interp(
                        grid,
                        macro[f"{key}_station"][:, axle_index],
                        macro[f"{key}_{quantity}"][:, axle_index],
                    )
                    * scale,
                    color=colours[label],
                    linewidth=0.78,
                    linestyle="-" if label == "ORVD" else "--",
                )
            add_station_bands(station_axis)
            station_axis.set_xlim(
                PRIMARY_STATION_BEGIN_METERS, PRIMARY_STATION_END_METERS
            )
            time_axis.grid(True, alpha=0.2)
            station_axis.grid(True, alpha=0.2)
            if axle_index == 0:
                time_axis.set_ylabel(ylabel)
                station_axis.set_ylabel(ylabel)
            if quantity_index == 1:
                time_axis.set_xlabel("Time [s]")
                station_axis.set_xlabel("Track station [m]")
    fig.suptitle(
        "G63 GZ18 R300+AAR5 16 s — wheelset response "
        "(native time and 100–250 m common station)",
        y=0.985,
    )
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=2,
        bbox_to_anchor=(0.5, 0.958),
        frameon=False,
    )
    fig.subplots_adjust(
        left=0.06, right=0.99, bottom=0.055, top=0.91, hspace=0.34, wspace=0.24
    )
    fig.savefig(path, dpi=220, facecolor="white")
    plt.close(fig)


def plot_contact_quantity(
    path: Path,
    quantity: str,
    contact: dict[str, Any],
    current_time: np.ndarray,
) -> None:
    fig, axes = plt.subplots(4, 4, figsize=(20, 13.5))
    time = expected_times(SAMPLE_COUNT, TERMINAL_TIME_SECONDS)
    colours = {"ORVD": "#0072B2", "SIMPACK": "#D55E00", "Current WRL": "#009E73"}
    handles: list[Any] = []
    labels: list[str] = []
    for axle_index, (axle, _, _) in enumerate(AXLES):
        axes[0, axle_index].set_title(axle.upper())
        for side_index, side in enumerate(("left", "right")):
            wheel_index = 2 * axle_index + side_index
            for domain_index, domain in enumerate(("time", "station")):
                axis = axes[2 * side_index + domain_index, axle_index]
                series = [
                    (
                        "ORVD",
                        (
                            time
                            if domain == "time"
                            else contact["orvd_station"][:, wheel_index]
                        ),
                        contact[f"orvd_{quantity}"][:, wheel_index],
                    ),
                    (
                        "SIMPACK",
                        (
                            time
                            if domain == "time"
                            else contact["simpack_station"][:, wheel_index]
                        ),
                        contact[f"simpack_{quantity}"][:, wheel_index],
                    ),
                ]
                if quantity != "Q" and domain == "time":
                    series.append(
                        (
                            "Current WRL",
                            (
                                current_time
                                if domain == "time"
                                else contact["current_wrl_station"][:, wheel_index]
                            ),
                            contact[f"current_wrl_{quantity}"][:, wheel_index],
                        )
                    )
                for label, x_values, values in series:
                    (line,) = axis.plot(
                        x_values,
                        values / 1000.0,
                        color=colours[label],
                        linewidth=0.72,
                        linestyle={"ORVD": "-", "SIMPACK": "--", "Current WRL": ":"}[
                            label
                        ],
                        label=label,
                    )
                    if axle_index == 0 and side_index == 0 and domain_index == 0:
                        handles.append(line)
                        labels.append(label)
                if domain == "station":
                    add_station_bands(axis)
                    axis.set_xlim(
                        PRIMARY_STATION_BEGIN_METERS, PRIMARY_STATION_END_METERS
                    )
                    axis.set_xlabel("Track station [m]")
                else:
                    axis.set_xlabel("Time [s]")
                if axle_index == 0:
                    axis.set_ylabel(f"{side.capitalize()} {quantity} [kN]")
                axis.grid(True, alpha=0.2)
    fig.suptitle(
        f"G63 GZ18 R300+AAR5 16 s — wheel-side {quantity} " "(diagnostic; Q is not N)",
        y=0.985,
    )
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=len(labels),
        bbox_to_anchor=(0.5, 0.958),
        frameon=False,
    )
    fig.subplots_adjust(
        left=0.06, right=0.99, bottom=0.055, top=0.91, hspace=0.34, wspace=0.24
    )
    fig.savefig(path, dpi=220, facecolor="white")
    plt.close(fig)


def plot_force_summary(path: Path, contact: dict[str, Any]) -> None:
    time = expected_times(SAMPLE_COUNT, TERMINAL_TIME_SECONDS)
    grid = primary_grid()
    fig, axes = plt.subplots(3, 2, figsize=(17.5, 11.2), sharex="col")
    ylabels = {
        "Q": "Vertical support force Q [kN]",
        "Tx": "Longitudinal force Tx [kN]",
        "Ty": "Lateral force Ty [kN]",
    }
    for row, quantity in enumerate(("Q", "Tx", "Ty")):
        difference = np.abs(
            contact[f"orvd_{quantity}"] - contact[f"simpack_{quantity}"]
        )
        wheel_index = int(np.unravel_index(np.argmax(difference), difference.shape)[1])
        axle, side = WHEEL_NAMES[wheel_index].split("_")
        wheel_label = f"{axle.upper()} {'left' if side == 'l' else 'right'}"
        for column, domain in enumerate(("time", "station")):
            axis = axes[row, column]
            x_values = time if domain == "time" else grid
            for source, colour, linestyle in (
                ("orvd", "#0072B2", "-"),
                ("simpack", "#D55E00", "--"),
            ):
                values = contact[f"{source}_{quantity}"][:, wheel_index]
                if domain == "station":
                    values = np.interp(
                        grid, contact[f"{source}_station"][:, wheel_index], values
                    )
                axis.plot(
                    x_values,
                    values / 1000.0,
                    color=colour,
                    linewidth=0.8,
                    linestyle=linestyle,
                    label=source.upper() if source == "orvd" else "SIMPACK",
                )
            axis.set_title(
                f"{quantity}: {wheel_label} — "
                f"{'time sequence' if domain == 'time' else 'same station'}"
            )
            axis.set_ylabel(ylabels[quantity])
            if domain == "station":
                add_station_bands(axis)
                axis.set_xlim(
                    PRIMARY_STATION_BEGIN_METERS, PRIMARY_STATION_END_METERS
                )
            axis.grid(True, alpha=0.2)
        axes[row, 0].set_xlabel("Time [s]")
        axes[row, 1].set_xlabel("Track station [m]")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.suptitle(
        "G63 GZ18 R300+AAR5 16 s — canonical wheel-side force response",
        y=0.985,
    )
    fig.text(
        0.5,
        0.955,
        "Raw Q/Tx/Ty values; the displayed wheel maximizes the native-time "
        "ORVD–SIMPACK peak for that component",
        ha="center",
    )
    fig.legend(
        handles,
        labels,
        loc="upper center",
        ncol=2,
        bbox_to_anchor=(0.5, 0.925),
        frameon=False,
    )
    fig.subplots_adjust(
        left=0.075, right=0.985, bottom=0.06, top=0.87, hspace=0.42, wspace=0.24
    )
    fig.savefig(path, dpi=220, facecolor="white")
    plt.close(fig)


def publish(
    output: Path,
    summary: dict[str, Any],
    macro: dict[str, Any],
    contact: dict[str, Any],
    current_time: np.ndarray,
) -> None:
    require(not output.exists(), f"analysis output already exists: {output}")
    partial = output.with_name(output.name + ".partial")
    require(not partial.exists(), f"analysis partial exists: {partial}")
    output.parent.mkdir(parents=True, exist_ok=True)
    partial.mkdir()
    try:
        write_json(partial / "comparison.json", summary)
        plot_macro(partial / "gz18_g63_wheelset_response.png", macro)
        for quantity in CONTACT_QUANTITIES:
            plot_contact_quantity(
                partial / f"gz18_g63_contact_{quantity}.png",
                quantity,
                contact,
                current_time,
            )
        plot_force_summary(partial / "gz18_g63_wheel_force_response.png", contact)
        (partial / "COMPLETE").write_text(
            "G63 comparison completed\n", encoding="utf-8"
        )
        partial.rename(output)
    except Exception:
        shutil.rmtree(partial, ignore_errors=True)
        raise


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orvd-artifact-directory", type=Path, required=True)
    parser.add_argument("--execution-identity-json", type=Path, required=True)
    parser.add_argument("--p047-authority-npz", type=Path, required=True)
    parser.add_argument("--p040-native-core-npz", type=Path, required=True)
    parser.add_argument("--aar5-lateral-json", type=Path, required=True)
    parser.add_argument("--current-wrl-csv", type=Path, required=True)
    parser.add_argument("--current-wrl-run-json", type=Path, required=True)
    parser.add_argument("--current-wrl-time-report", type=Path, required=True)
    parser.add_argument("--current-wrl-executable", type=Path, required=True)
    parser.add_argument("--current-wrl-source-revision", required=True)
    parser.add_argument("--historical-wrl-csv", type=Path, required=True)
    parser.add_argument("--historical-wrl-source-revision", required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    try:
        orvd_directory = arguments.orvd_artifact_directory.resolve()
        orvd = load_g63_orvd(orvd_directory)
        execution = load_execution_identity(
            arguments.execution_identity_json.resolve(),
            orvd_directory,
            goal_name="G63",
        )
        p047 = load_p047(arguments.p047_authority_npz.resolve())
        p040_check = cross_check_p040(arguments.p040_native_core_npz.resolve(), p047)
        field = ZeroOutsideNaturalSpline(
            arguments.aar5_lateral_json.resolve(),
            "gz18_r300_aar5_reference_lateral",
        )
        current_csv = arguments.current_wrl_csv.resolve()
        current = load_wrl(
            current_csv,
            name="current WRL",
            expected_rows=SAMPLE_COUNT,
            expected_last_time=TERMINAL_TIME_SECONDS,
        )
        current_identity = validate_current_wrl_identity(
            arguments.current_wrl_run_json.resolve(),
            current_csv,
            arguments.current_wrl_executable.resolve(),
            arguments.current_wrl_source_revision,
        )
        current_identity.update(
            parse_time_report(arguments.current_wrl_time_report.resolve())
        )
        require(
            arguments.historical_wrl_source_revision == HISTORICAL_WRL_REVISION,
            "historical WRL source revision is not the P055 control",
        )
        historical_path = arguments.historical_wrl_csv.resolve()
        require(
            sha256_file(historical_path) == HISTORICAL_WRL_CSV_SHA256,
            "historical WRL CSV is not the frozen P055 control",
        )
        historical = load_wrl(
            historical_path,
            name="historical WRL P055",
            expected_rows=32_000,
            expected_last_time=15.9995,
        )
        macro = macro_arrays(orvd, p047, current, historical, field)
        contact = contact_arrays(orvd, p047, current, historical)
        statistics = build_statistics(orvd, macro, contact, current, historical)
        statistics["current_vs_historical_wrl_native_time"] = (
            direct_wrl_control_consistency(macro, contact)
        )
        statistics["current_wrl_vs_simpack_force_native_time"] = (
            current_wrl_simpack_force_difference(contact)
        )
        require(
            statistics["orvd_longest_zero_contact_run_matches_metadata"],
            "ORVD zero-contact observations differ from metadata",
        )
        require(
            not statistics["sustained_contact_or_force_failures"],
            "G63 contains sustained loss of contact or all-zero wheel force",
        )
        endpoint = orvd["metadata"].get("endpoint_assembly_and_state_slice_diagnostics")
        require(isinstance(endpoint, dict), "ORVD endpoint diagnostics are absent")
        summary = {
            "scope": "G63 GZ18 R300+AAR5 16 s curve response",
            "sources": {
                "orvd": {
                    "revision": execution["orvd_revision"],
                    "artifact_directory": str(orvd_directory),
                    "observations_sha256": orvd["observation_sha256"],
                    "execution_identity_sha256": sha256_file(
                        arguments.execution_identity_json.resolve()
                    ),
                },
                "simpack_p047": {
                    "source_sbr_sha256": P047_SBR_SHA256,
                    "precision": "binary64",
                    "sample_count": SAMPLE_COUNT,
                },
                "simpack_p040_cross_check": p040_check,
                "current_wrl": current_identity,
                "historical_wrl": {
                    "source_revision": HISTORICAL_WRL_REVISION,
                    "csv_sha256": HISTORICAL_WRL_CSV_SHA256,
                    "sample_count": 32_000,
                    "last_time_seconds": 15.9995,
                    "role": "named historical control only",
                },
            },
            "coordinate_contract": {
                "simpack_macro_lateral": "jointPos st_002 minus RWT ch_007",
                "simpack_macro_yaw": "minus jointPos st_005 plus atan(d(RWT ch_007)/d(RWT ch_003))",
                "orvd_macro_irregularity_station": (
                    "evaluate the qualified natural spline independently at the "
                    "left/right rail-profile reference stations, then average"
                ),
                "orvd_macro_lateral": "base-track lateral minus mean irregularity",
                "orvd_macro_yaw": "minus base-track yaw plus atan(mean irregularity slope)",
                "type80_tangential_force": "rail end negated exactly once to canonical wheel end",
                "Q_is_not_N": True,
                "current_wrl_Q_omitted": (
                    "current CSV Fz is world-z force and is not canonical Q on a curve"
                ),
            },
            "statistics": statistics,
            "orvd_performance": {
                **orvd["performance"],
                "process_wall_seconds": execution["process_wall_seconds"],
                "simulation_to_process_wall_factor": (
                    TERMINAL_TIME_SECONDS / execution["process_wall_seconds"]
                ),
                "maximum_resident_set_kilobytes": execution[
                    "maximum_resident_set_kilobytes"
                ],
                "hardware": execution["hardware"],
                "compiler": execution["compiler"],
                "openmp_environment": execution["openmp_environment"],
            },
            "endpoint_assembly_and_state_slice_diagnostics": endpoint,
            "contact_force_role": (
                "diagnostic observation; no post-result Q/N/Tx/Ty threshold"
            ),
            "conclusion": {
                "macro_preregistered_rms_satisfied_for_all_axes": statistics[
                    "macro_all_axes_raw_and_increment_rms_within_0p1_mm_or_mrad"
                ],
                "macro_preregistered_alarm_satisfied_for_all_axes": statistics[
                    "macro_all_axes_raw_and_increment_rms_and_max_within_0p1_mm_or_mrad"
                ],
                "contact_or_force_sustained_failure": False,
            },
        }
        publish(
            arguments.output_directory.resolve(),
            summary,
            macro,
            contact,
            current["time_seconds"],
        )
    except (AnalysisError, KeyError, OSError, ValueError) as error:
        print(f"G63 analysis failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
