#!/usr/bin/env python3
"""Analyze the G77 controlled IRW long window against SIMPACK Realtime.

The primary comparison is deliberately narrow and source based:

* ORVD and SIMPACK Realtime must each provide exactly 60,001 observations on
  the same integer 0.5 ms clock;
* axle-bridge lateral displacement and source-basis yaw are compared both at
  the native time ordinal and on each axle's own 100--450 m / 0.01 m grid;
* Q is an interface total, while N/Tx/Ty are selected from the maximum-N
  active patch and are never summed across patch-local frames;
* station interpolation of patch-local quantities is confined to contiguous
  runs with one unchanged source-local primary-patch ordinal;
* ORVD and SIMPACK controller/conditioner traces are paired directly by
  integer event ordinal U0..U3000, independently of their floating timestamps.

No input is shifted, filtered, fitted, demeaned or rescaled from its observed
result.  An optional frozen WRL CSV adds a historical third macro-response
line and native-time local-force trace; its 0.1 ms clock is decimated at exact
common instants without time interpolation.  WRL local force values are not
put on the station grid because the frozen wide table does not retain the
selected patch ordinal.  A separate optional frozen WRL control NPZ contributes
only request/applied torque after the documented row-2432 control-record
relabeling; it never moves vehicle data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402


AXLES = ("ff", "fr", "rf", "rr")
WHEELS = (
    "ff_l", "ff_r", "fr_l", "fr_r",
    "rf_l", "rf_r", "rr_l", "rr_r",
)
SIMPACK_AXLES = ("axle01", "axle02", "axle03", "axle04")
SIMPACK_WHEELS = tuple(
    f"{axle}_{side}"
    for axle in SIMPACK_AXLES
    for side in ("left", "right")
)
ORVD_INTERFACES = tuple(f"wheel_{wheel}" for wheel in WHEELS)

OBSERVATION_PERIOD_NS = 500_000
OBSERVATION_PERIOD_SECONDS = 0.0005
TERMINAL_TIME_NS = 30_000_000_000
SAMPLE_COUNT = 60_001
CONTROL_PERIOD_SECONDS = 0.01
CONTROL_EVENT_COUNT = 3_001
CONTROL_AUDIT_COUNT = 3_002
POSITIVE_HOLD_INTERVAL_COUNT = 3_000
BACKEND_SYNCHRONIZATION_COUNT = 2_999
STATION_GRID = np.arange(10_000, 45_001, dtype=np.float64) * 0.01

PLOT_LABELS = {
    "orvd": "ORVD",
    "simpack": "SIMPACK Realtime",
    "wrl": "Frozen WRL",
}
PLOT_STYLES = {
    "orvd": {"color": "#0072B2", "linestyle": "-", "linewidth": 1.05,
             "zorder": 2},
    "simpack": {"color": "#D55E00", "linestyle": "--",
                "linewidth": 1.0, "zorder": 3},
    "wrl": {"color": "#009E73", "linestyle": "-.", "linewidth": 0.85,
            "zorder": 1},
}


class AnalysisError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AnalysisError(message)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, name: str) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise AnalysisError(f"could not read {name} '{path}': {error}") from error
    require(isinstance(value, dict), f"{name} is not a JSON object")
    return value


def require_finite(value: np.ndarray, name: str) -> None:
    require(bool(np.isfinite(value).all()), f"{name} contains a non-finite value")


def clean_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return clean_json(value.tolist())
    if isinstance(value, np.generic):
        return clean_json(value.item())
    if isinstance(value, float):
        if not math.isfinite(value):
            raise AnalysisError(f"refusing non-finite JSON value {value}")
        return float(value)
    return value


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(
        json.dumps(clean_json(value), ensure_ascii=False, indent=2,
                   allow_nan=False) + "\n",
        encoding="utf-8",
    )


def expected_ticks() -> np.ndarray:
    return np.arange(SAMPLE_COUNT, dtype=np.int64) * OBSERVATION_PERIOD_NS


def statistics(value: np.ndarray, coordinate: np.ndarray) -> dict[str, Any]:
    values = np.asarray(value, dtype=np.float64)
    coordinates = np.asarray(coordinate, dtype=np.float64)
    require(values.shape == coordinates.shape and values.size > 0,
            "statistics input and coordinate do not have one nonempty shape")
    require_finite(values, "statistics input")
    require_finite(coordinates, "statistics coordinate")
    peak = int(np.argmax(np.abs(values)))
    return {
        "sample_count": int(values.size),
        "signed_mean": float(np.mean(values)),
        "rms": float(np.sqrt(np.mean(np.square(values)))),
        "maximum_absolute": float(np.max(np.abs(values))),
        "maximum_absolute_coordinate": float(coordinates[peak]),
    }


def masked_statistics(value: np.ndarray, valid: np.ndarray,
                      coordinate: np.ndarray) -> dict[str, Any]:
    require(value.shape == valid.shape == coordinate.shape,
            "masked statistics inputs have different shapes")
    require(valid.dtype == np.bool_, "masked statistics selector is not boolean")
    require(bool(np.any(valid)), "masked statistics has no common valid sample")
    result = statistics(value[valid], coordinate[valid])
    result["population_sample_count"] = int(valid.size)
    result["available_sample_count"] = int(np.count_nonzero(valid))
    result["available_fraction"] = float(np.count_nonzero(valid) / valid.size)
    return result


def original_and_increment_statistics(
    left: np.ndarray, right: np.ndarray, coordinate: np.ndarray,
    scale: float,
) -> dict[str, Any]:
    require(left.shape == right.shape == coordinate.shape,
            "macro comparison inputs have different shapes")
    return {
        "original": statistics((left - right) * scale, coordinate),
        "relative_to_each_source_initial": statistics(
            ((left - left[0]) - (right - right[0])) * scale, coordinate
        ),
    }


def frame_matrix(
    frame: pd.DataFrame, prefix: str, names: tuple[str, ...], source: str,
) -> np.ndarray:
    columns = [f"{prefix}{name}" for name in names]
    missing = sorted(set(columns) - set(frame.columns))
    require(not missing, f"{source} is missing columns: {missing}")
    value = frame[columns].to_numpy(np.float64)
    require_finite(value, f"{source} {prefix}")
    return value


def require_integer_matrix(value: np.ndarray, source: str) -> np.ndarray:
    require(np.array_equal(value, np.rint(value)),
            f"{source} contains a non-integer discrete value")
    return value.astype(np.int64)


def read_orvd_control_events(path: Path) -> dict[str, Any]:
    frame = pd.read_csv(path, sep="\t")
    require(frame.shape[0] == CONTROL_AUDIT_COUNT and
            len(frame.columns) == len(set(frame.columns)),
            "ORVD control audit has the wrong row count or duplicate columns")
    require(frame["event_kind"].iloc[0] == "initialization" and
            bool((frame["event_kind"].iloc[1:] == "periodic").all()),
            "ORVD control audit does not contain one initialization then "
            "U0..U3000")
    periodic = frame.iloc[1:].reset_index(drop=True)
    ordinal = periodic["periodic_event_ordinal"].to_numpy(np.int64)
    require(np.array_equal(ordinal, np.arange(CONTROL_EVENT_COUNT,
                                               dtype=np.int64)),
            "ORVD periodic control events are not U0..U3000")
    event_time = periodic["event_time_seconds"].to_numpy(np.float64)
    expected_time = ordinal.astype(np.float64) * CONTROL_PERIOD_SECONDS
    require_finite(event_time, "ORVD control event time")
    require(np.max(np.abs(event_time - expected_time)) <= 4.0e-15,
            "ORVD control event time is detached from its integer ordinal")

    def wheels(prefix: str) -> np.ndarray:
        return frame_matrix(periodic, prefix, WHEELS, "ORVD control audit")

    def axles(prefix: str) -> np.ndarray:
        return frame_matrix(periodic, prefix, AXLES, "ORVD control audit")

    return {
        "ordinal": ordinal,
        "time": event_time,
        "request": wheels("controller.request."),
        "actual": wheels("conditioner.actual_torque."),
        "dynamic_limit": wheels("conditioner.dynamic_limit."),
        "limit_flags": require_integer_matrix(
            wheels("conditioner.limit_flag."), "ORVD conditioner flags"
        ),
        "base_speed": wheels("controller.base_speed_reference."),
        "reference_speed": wheels("controller.speed_reference."),
        "delta_reference": axles("controller.delta_omega_reference."),
        "delta_measured": axles("controller.delta_omega_measured."),
        "delta_equilibrium": axles("controller.delta_omega_equilibrium."),
        "filtered_lateral_velocity": axles(
            "controller.filtered_lateral_velocity."
        ),
        "filtered_yaw_rate": axles("controller.filtered_yaw_rate."),
        "guidance_active": require_integer_matrix(
            axles("controller.guidance_active."), "ORVD guidance flags"
        ),
    }


def parse_return_codes(path: Path) -> dict[str, int]:
    require(path.is_file(), f"SIMPACK run is missing {path.name}")
    output: dict[str, int] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        fields = line.split("=", 1)
        require(len(fields) == 2 and fields[0] and fields[0] not in output,
                "SIMPACK return-code file is malformed")
        try:
            output[fields[0]] = int(fields[1])
        except ValueError as error:
            raise AnalysisError("SIMPACK return code is not an integer") from error
    require(output == {"driver_rc": 0},
            "SIMPACK direct-only driver did not finish successfully")
    return output


def expected_simpack_output_names() -> list[str]:
    names = [f"$Y_SpeedDiff_{name}" for name in (
        "FrontA", "FrontB", "FrontC", "FrontD",
        "RearA", "RearB", "RearC", "RearD",
    )]
    names += [f"$Y_{axle}_y" for axle in SIMPACK_AXLES]
    names += [f"$Y_{axle}_yaw" for axle in SIMPACK_AXLES]
    names += [
        f"$Y_{axle}_{side}_w"
        for axle in SIMPACK_AXLES for side in ("left", "right")
    ]
    names += [f"$Y_{axle}_s" for axle in SIMPACK_AXLES]
    names += [
        f"$Y_QVertical_{wheel}" for wheel in SIMPACK_WHEELS
    ]
    for wheel in SIMPACK_WHEELS:
        for patch in range(1, 6):
            names += [
                f"$Y_RawMinusN_{wheel}_patch{patch:02d}",
                f"$Y_RawMinusTx_{wheel}_patch{patch:02d}",
                f"$Y_RawMinusTy_{wheel}_patch{patch:02d}",
            ]
    require(len(names) == 156, "internal SIMPACK output-name table is incomplete")
    return names


def require_simpack_mapping(metadata: dict[str, Any]) -> None:
    require(metadata.get("output_names") == expected_simpack_output_names() and
            metadata.get("ny") == 156 and metadata.get("nu") == 8,
            "SIMPACK Realtime output identity is not the closed 156-channel run")
    expected_motor_names = (
        "front_a", "front_b", "front_c", "front_d",
        "rear_a", "rear_b", "rear_c", "rear_d",
    )
    motor_map = metadata.get("motor_channel_map")
    require(isinstance(motor_map, list) and len(motor_map) == len(WHEELS) and
            all(isinstance(entry, dict) for entry in motor_map),
            "SIMPACK motor channel map is incomplete")
    for index, name in enumerate(expected_motor_names):
        require(motor_map[index].get("canonical_name") == name and
                motor_map[index].get("input_index") == index and
                motor_map[index].get("speed_error_index") == index,
                f"SIMPACK motor channel map differs at channel {name}")
    startup = metadata.get("startup_initialization_only")
    require(metadata.get("mode") == "full_state_pi_proxy" and
            metadata.get("torque_sign") == 1.0 and
            isinstance(startup, dict) and startup.get("performed") is True and
            startup.get("applied_to_plant") is False,
            "SIMPACK control execution is not the frozen P179 double update")
    q_map = metadata.get("q_vertical_force_element_channel_map")
    require(isinstance(q_map, list) and len(q_map) == 8,
            "SIMPACK Q channel map is incomplete")
    for index, entry in enumerate(q_map):
        require(entry == {
            "force_element_output_value": 1,
            "force_element_result_type": 78,
            "interface": "Realtime type-12 Force/Control Element",
            "output_index": 28 + index,
            "output_name": f"$Y_QVertical_{SIMPACK_WHEELS[index]}",
            "wheel": SIMPACK_WHEELS[index],
        }, f"SIMPACK Q channel map differs at wheel {SIMPACK_WHEELS[index]}")

    patch_map = metadata.get("contact_patch_force_element_channel_map")
    require(isinstance(patch_map, list) and len(patch_map) == 120,
            "SIMPACK patch channel map is incomplete")
    quantity_contract = (
        ("raw_minus_n", "RawMinusN", 11),
        ("raw_minus_tx", "RawMinusTx", 9),
        ("raw_minus_ty", "RawMinusTy", 10),
    )
    ordinal = 0
    for wheel in SIMPACK_WHEELS:
        for patch in range(1, 6):
            for quantity, output_quantity, first_value in quantity_contract:
                expected = {
                    "force_element_output_value": first_value + 9 * (patch - 1),
                    "force_element_result_type": 78,
                    "interface": "Realtime type-12 Force/Control Element",
                    "output_index": 36 + ordinal,
                    "output_name": (
                        f"$Y_{output_quantity}_{wheel}_patch{patch:02d}"
                    ),
                    "patch": patch,
                    "quantity": quantity,
                    "wheel": wheel,
                }
                require(patch_map[ordinal] == expected,
                        f"SIMPACK patch channel map differs at output {36 + ordinal}")
                ordinal += 1

    require(metadata.get("wheel_load_contract") == {
        "absolute_value_forbidden": True,
        "comparison_interface":
            "SBR Rail-Wheel Pair result: Q Vertical wheel force",
        "compressive_field": "compressive_wheel_load_N",
        "compressive_transform": "-q_vertical_force_element_raw_N",
        "interface_sign_relation":
            "opposite sign for the accepted balanced startup state",
        "raw_field": "q_vertical_force_element_raw_N",
        "raw_interface": (
            "Realtime type-12 Force/Control Element, type-78 output value 1"
        ),
        "raw_required_sign": "finite and strictly negative; +0 and -0 rejected",
    }, "SIMPACK Q sign contract is not the pre-G77 authority")
    require(metadata.get("contact_patch_output_contract") == {
        "driver_patch_selection": "none",
        "driver_transformation": "none",
        "offline_wide_comparison":
            "retain all slots and select maximum-N active patch",
        "patch_frame_semantics": "patch-local; no cross-patch scalar summation",
        "patch_slots_per_wheel": 5,
        "quantities": ["raw_minus_n", "raw_minus_tx", "raw_minus_ty"],
        "sign_semantics": "raw SIMPACK Type-78 -N/-Tx/-Ty values",
        "storage": "IEEE-754 binary64-sized double via SpckRtGetY(double*)",
    }, "SIMPACK patch-force contract is not the pre-G77 authority")


def select_orvd_primary_patches(
    patch_frame: pd.DataFrame, ticks: np.ndarray, times: np.ndarray,
    expected_counts: np.ndarray,
) -> tuple[dict[str, np.ndarray], np.ndarray]:
    required = {
        "sample_index", "time_nanoseconds", "time_seconds", "interface_name",
        "patch_ordinal", "normal_force_newtons",
        "longitudinal_force_on_wheel_in_contact_frame_newtons",
        "lateral_force_on_wheel_in_contact_frame_newtons",
    }
    missing = sorted(required - set(patch_frame.columns))
    require(not missing, f"ORVD patch table is missing columns: {missing}")
    interface_index = {name: index for index, name in enumerate(ORVD_INTERFACES)}
    require(bool(patch_frame["interface_name"].isin(interface_index).all()),
            "ORVD patch table contains an unknown interface")
    sample = patch_frame["sample_index"].to_numpy(np.int64)
    require(bool(np.all((sample >= 0) & (sample < SAMPLE_COUNT))),
            "ORVD patch table contains an invalid sample index")
    require(np.array_equal(
        patch_frame["time_nanoseconds"].to_numpy(np.int64), ticks[sample]
    ) and np.array_equal(
        patch_frame["time_seconds"].to_numpy(np.float64), times[sample]
    ), "ORVD patch rows do not share the observation clock")
    wheel = patch_frame["interface_name"].map(interface_index).to_numpy(np.int64)
    patch = patch_frame["patch_ordinal"].to_numpy(np.int64)
    require(bool(np.all(patch >= 0)), "ORVD patch ordinal is negative")
    triples = np.stack((sample, wheel, patch), axis=1)
    require(np.unique(triples, axis=0).shape[0] == triples.shape[0],
            "ORVD patch table repeats a sample/interface/patch ordinal")
    actual_counts = np.zeros((SAMPLE_COUNT, len(WHEELS)), dtype=np.int64)
    np.add.at(actual_counts, (sample, wheel), 1)
    require(np.array_equal(actual_counts, expected_counts),
            "ORVD patch rows do not reproduce fixed-width patch counts")

    column_by_quantity = {
        "N": "normal_force_newtons",
        "Tx": "longitudinal_force_on_wheel_in_contact_frame_newtons",
        "Ty": "lateral_force_on_wheel_in_contact_frame_newtons",
    }
    for quantity, column in column_by_quantity.items():
        require_finite(patch_frame[column].to_numpy(np.float64),
                       f"ORVD patch-local {quantity}")
    require(bool(np.all(
        patch_frame["normal_force_newtons"].to_numpy(np.float64) > 0.0
    )), "ORVD returned patch has a non-positive normal force")

    ordered = patch_frame.assign(_wheel=wheel).sort_values(
        ["sample_index", "_wheel", "normal_force_newtons", "patch_ordinal"],
        ascending=[True, True, False, True], kind="mergesort",
    )
    primary = ordered.drop_duplicates(["sample_index", "_wheel"], keep="first")
    primary_sample = primary["sample_index"].to_numpy(np.int64)
    primary_wheel = primary["_wheel"].to_numpy(np.int64)
    primary_ordinal = np.full((SAMPLE_COUNT, len(WHEELS)), -1, dtype=np.int64)
    primary_ordinal[primary_sample, primary_wheel] = primary[
        "patch_ordinal"
    ].to_numpy(np.int64)
    values = {
        quantity: np.full((SAMPLE_COUNT, len(WHEELS)), np.nan, dtype=np.float64)
        for quantity in column_by_quantity
    }
    for quantity, column in column_by_quantity.items():
        values[quantity][primary_sample, primary_wheel] = primary[
            column
        ].to_numpy(np.float64)
    require(np.array_equal(primary_ordinal >= 0, expected_counts > 0),
            "ORVD primary-patch selection does not match contact presence")
    return values, primary_ordinal


def read_orvd(directory: Path, execution_identity_path: Path) -> dict[str, Any]:
    require(directory.is_dir(), f"ORVD artifact is absent: {directory}")
    for filename in (
        "COMPLETE", "metadata.json", "performance.json", "observations.tsv",
        "contact_patches.tsv", "control_events.tsv", "endpoint_diagnostics.tsv",
    ):
        require((directory / filename).is_file(),
                f"ORVD artifact is missing {filename}")
    metadata = load_json(directory / "metadata.json", "ORVD metadata")
    performance = load_json(directory / "performance.json", "ORVD performance")
    execution = load_json(execution_identity_path, "ORVD execution identity")
    expected_identity = {
        "completed": True,
        "qualification_vehicle_recipe": "IRW_P179_100HZ_CONTROLLED",
        "vehicle_name": "IRW",
        "track_irregularity_identifier":
            "irw_r300_aar5_reference_irregularity",
        "controller_identifier":
            "irw_r300_v60_full_state_wheel_speed_guidance_controller",
        "torque_conditioner_identifier":
            "irw_reference_wheel_drive_torque_conditioner",
        "control_event_period_seconds": CONTROL_PERIOD_SECONDS,
        "mechanical_observation_period_nanoseconds": OBSERVATION_PERIOD_NS,
        "duration_nanoseconds": TERMINAL_TIME_NS,
        "observation_count": SAMPLE_COUNT,
        "control_audit_count": CONTROL_AUDIT_COUNT,
        "positive_hold_interval_count": POSITIVE_HOLD_INTERVAL_COUNT,
        "backend_synchronization_count": BACKEND_SYNCHRONIZATION_COUNT,
    }
    for key, expected in expected_identity.items():
        require(metadata.get(key) == expected,
                f"ORVD metadata {key!r} does not identify G77")
    require(metadata.get("control_observation_basis") == (
        "lateral displacement in Track-T; yaw in the frozen P179 source "
        "physical axle-bridge body basis"
    ), "ORVD macro-response basis differs from the P179 source basis")
    require(metadata.get("contact_observation_contract") == {
        "interface_totals_frame": "carrier-projection Track-T axes",
        "patch_local_components": (
            "one row per returned patch; local N/Tx/Ty are not summed across "
            "contact frames"
        ),
        "maximum_returned_patch_count": 16,
    }, "ORVD contact observation contract differs from G77")
    require(metadata.get("assembled_state_and_force_layout") == {
        "generalized_position_count": 81,
        "generalized_velocity_count": 74,
        "series_force_state_count": 2,
        "vehicle_body_wrench_count": 96,
        "contact_body_wrench_count": 8,
        "active_torque_body_wrench_count": 16,
    }, "ORVD G77 assembly layout differs from the frozen IRW recipe")
    require(metadata.get("numerical_tolerances") == {
        "relative": 1.0e-7,
        "q_absolute": 1.0e-9,
        "v_absolute": 1.0e-8,
        "z_absolute_newtons": 1.0e-6,
    }, "ORVD G77 numerical tolerances differ from the frozen recipe")
    require((directory / "COMPLETE").read_text(encoding="utf-8") ==
            f"{SAMPLE_COUNT} controlled observations\n",
            "ORVD COMPLETE marker has the wrong observation count")
    require(execution.get("exit_status") == 0 and
            execution.get("vehicle_recipe") == "irw-p179-controlled" and
            execution.get("build_type") == "Release" and
            Path(str(execution.get("qualification_artifact_directory"))).resolve() ==
            directory.resolve(),
            "ORVD execution identity does not name this successful Release run")

    frame = pd.read_csv(directory / "observations.tsv", sep="\t")
    require(frame.shape[0] == SAMPLE_COUNT and
            len(frame.columns) == len(set(frame.columns)),
            "ORVD observations have the wrong rows or duplicate columns")
    required = {"sample_index", "time_nanoseconds", "time_seconds"}
    for axle in AXLES:
        required.update({f"station.{axle}", f"lateral.{axle}",
                         f"source_body_yaw.{axle}"})
    for wheel in WHEELS:
        required.update({f"patch_count.{wheel}", f"Q.{wheel}", f"N.{wheel}"})
    missing = sorted(required - set(frame.columns))
    require(not missing, f"ORVD observations are missing columns: {missing}")
    sample_index = frame["sample_index"].to_numpy(np.int64)
    ticks = frame["time_nanoseconds"].to_numpy(np.int64)
    times = frame["time_seconds"].to_numpy(np.float64)
    require(np.array_equal(sample_index, np.arange(SAMPLE_COUNT, dtype=np.int64)) and
            np.array_equal(ticks, expected_ticks()),
            "ORVD observations are not the integer 0.5 ms sequence")
    require(np.max(np.abs(
        times - ticks.astype(np.float64) * 1.0e-9
    )) <= 4.0e-15,
            "ORVD floating time is detached from its integer clock")

    def matrix(prefix: str, names: tuple[str, ...]) -> np.ndarray:
        value = frame[[f"{prefix}.{name}" for name in names]].to_numpy(np.float64)
        require_finite(value, f"ORVD {prefix}")
        return value

    patch_count_float = matrix("patch_count", WHEELS)
    require(np.array_equal(patch_count_float, np.rint(patch_count_float)) and
            bool(np.all((patch_count_float >= 0.0) & (patch_count_float <= 16.0))),
            "ORVD patch counts are not integers within the declared capacity")
    patch_count = patch_count_float.astype(np.int64)
    primary, primary_ordinal = select_orvd_primary_patches(
        pd.read_csv(directory / "contact_patches.tsv", sep="\t"),
        ticks, times, patch_count,
    )
    control = read_orvd_control_events(directory / "control_events.tsv")
    endpoint = pd.read_csv(directory / "endpoint_diagnostics.tsv", sep="\t")
    require(endpoint.shape[0] == CONTROL_EVENT_COUNT and
            len(endpoint.columns) == len(set(endpoint.columns)),
            "ORVD endpoint diagnostics have the wrong rows or duplicate columns")
    endpoint_required = {
        "held_torque_event_ordinal", "time_seconds",
        "state_derivative_inf_norm",
        "generalized_force_residual_inf_norm",
        "virtual_power_residual_watts",
    }
    require(endpoint_required <= set(endpoint.columns),
            "ORVD endpoint diagnostics are missing required columns")
    endpoint_time = endpoint["time_seconds"].to_numpy(np.float64)
    endpoint_hold = endpoint["held_torque_event_ordinal"].to_numpy(np.int64)
    endpoint_state_norm = endpoint[
        "state_derivative_inf_norm"
    ].to_numpy(np.float64)
    endpoint_force_residual = endpoint[
        "generalized_force_residual_inf_norm"
    ].to_numpy(np.float64)
    endpoint_power_residual = endpoint[
        "virtual_power_residual_watts"
    ].to_numpy(np.float64)
    for value, name in (
        (endpoint_time, "endpoint time"),
        (endpoint_state_norm, "endpoint state derivative norm"),
        (endpoint_force_residual, "endpoint generalized-force residual"),
        (endpoint_power_residual, "endpoint virtual-power residual"),
    ):
        require_finite(value, f"ORVD {name}")
    expected_endpoint_time = (
        np.arange(CONTROL_EVENT_COUNT, dtype=np.float64) *
        CONTROL_PERIOD_SECONDS
    )
    expected_endpoint_hold = np.maximum(
        np.arange(CONTROL_EVENT_COUNT, dtype=np.int64) - 1, 0
    )
    require(np.max(np.abs(endpoint_time - expected_endpoint_time)) <= 4.0e-15 and
            np.array_equal(endpoint_hold, expected_endpoint_hold),
            "ORVD endpoint diagnostics do not identify the preceding hold")
    require(math.isclose(
                float(np.max(endpoint_force_residual)),
                float(performance.get(
                    "maximum_generalized_force_residual_inf_norm")),
                rel_tol=4.0e-15, abs_tol=0.0) and
            math.isclose(
                float(np.max(np.abs(endpoint_power_residual))),
                float(performance.get(
                    "maximum_absolute_virtual_power_residual_watts")),
                rel_tol=4.0e-15, abs_tol=0.0),
            "ORVD performance residual maxima do not match endpoint diagnostics")
    return {
        "time": times,
        "ticks": ticks,
        "station": matrix("station", AXLES),
        "lateral": matrix("lateral", AXLES),
        "yaw": matrix("source_body_yaw", AXLES),
        "Q": matrix("Q", WHEELS),
        "N_total": matrix("N", WHEELS),
        "patch_count": patch_count,
        "primary_ordinal": primary_ordinal,
        "control": control,
        **primary,
        "metadata": metadata,
        "performance": performance,
        "execution": execution,
        "source_sha256": {
            "observations.tsv": sha256_file(directory / "observations.tsv"),
            "contact_patches.tsv": sha256_file(directory / "contact_patches.tsv"),
            "control_events.tsv": sha256_file(directory / "control_events.tsv"),
            "endpoint_diagnostics.tsv": sha256_file(
                directory / "endpoint_diagnostics.tsv"
            ),
            "metadata.json": sha256_file(directory / "metadata.json"),
            "performance.json": sha256_file(directory / "performance.json"),
            "execution_identity": sha256_file(execution_identity_path),
        },
    }


def read_simpack(directory: Path) -> dict[str, Any]:
    require(directory.is_dir(), f"SIMPACK run is absent: {directory}")
    csv_path = directory / "realtime.csv"
    metadata_path = directory / "realtime.json"
    validation_path = directory / "validation_summary.json"
    for path in (csv_path, metadata_path, validation_path):
        require(path.is_file(), f"SIMPACK run is missing {path.name}")
    parse_return_codes(directory / "return_codes.txt")
    metadata = load_json(metadata_path, "SIMPACK Realtime metadata")
    validation = load_json(validation_path, "SIMPACK validation summary")
    require_simpack_mapping(metadata)
    require(metadata.get("duration_s") == 30.0 and
            metadata.get("observation_step_s") == OBSERVATION_PERIOD_SECONDS and
            metadata.get("controller_step_s") == CONTROL_PERIOD_SECONDS and
            metadata.get("completed_steps") == SAMPLE_COUNT - 1 and
            metadata.get("completed_observation_steps") == SAMPLE_COUNT - 1 and
            metadata.get("completed_controller_intervals") == 3_000 and
            metadata.get("numeric_storage") ==
            "IEEE-754 binary64-sized double via SpckRtGetY(double*)",
            "SIMPACK Realtime metadata does not identify the 30 s direct run")
    require(validation.get("status") == "pass" and
            validation.get("failures") == [],
            "SIMPACK pre-G77 validation did not pass")
    data_contract = validation.get("data")
    require(isinstance(data_contract, dict) and
            data_contract.get("expected_row_count") == SAMPLE_COUNT and
            data_contract.get("row_count") == SAMPLE_COUNT and
            data_contract.get("nonfinite_yout_value_count") == 0 and
            data_contract.get("contact_observation_semantics") == {
                "active_patch": "raw_minus_n < 0.0",
                "canonical_transform":
                    "N=-raw_minus_n; Tx=raw_minus_tx; Ty=raw_minus_ty",
                "cross_patch_tx_ty_summation":
                    "forbidden because patch frames differ",
                "detachment": (
                    "reported as sampled zero-contact rows and longest sample "
                    "span; no qualification threshold is imposed here"
                ),
                "main_patch":
                    "active patch with maximum N (minimum raw_minus_n)",
            }, "SIMPACK validation uses a different contact interpretation")
    source = validation.get("source")
    require(isinstance(source, dict) and
            Path(str(source.get("csv"))).resolve() == csv_path.resolve() and
            Path(str(source.get("metadata"))).resolve() == metadata_path.resolve() and
            Path(str(source.get("run_directory"))).resolve() == directory.resolve(),
            "SIMPACK validation does not bind this run directory")

    trace_columns = [
        *(f"full_state_pi_request_Nm_{index}" for index in range(8)),
        *(f"bridge_applied_wheel_torque_Nm_{index}" for index in range(8)),
        *(f"bridge_limit_wheel_torque_Nm_{index}" for index in range(8)),
        *(f"bridge_limit_flags_{index}" for index in range(8)),
        *(f"v_base_wheel_mps_{index}" for index in range(8)),
        *(f"v_ref_wheel_mps_{index}" for index in range(8)),
        *(f"delta_omega_ref_radps_{index}" for index in range(4)),
        *(f"delta_omega_meas_radps_{index}" for index in range(4)),
        *(f"delta_omega_eq_radps_{index}" for index in range(4)),
        *(f"y_dot_hat_mps_{index}" for index in range(4)),
        *(f"psi_dot_hat_radps_{index}" for index in range(4)),
        *(f"curve_mode_active_{index}" for index in range(4)),
    ]
    usecols = [
        "time_s", *(f"u_{index}" for index in range(8)),
        *(f"y_{index}" for index in range(8, 156)), *trace_columns,
    ]
    frame = pd.read_csv(csv_path, usecols=usecols, float_precision="round_trip")
    require(frame.shape[0] == SAMPLE_COUNT,
            "SIMPACK Realtime CSV does not have 60,001 rows")
    time = frame["time_s"].to_numpy(np.float64)
    expected_time = expected_ticks().astype(np.float64) * 1.0e-9
    require(np.max(np.abs(time - expected_time)) <= 4.0e-15 and
            bool(np.all(np.diff(time) > 0.0)),
            "SIMPACK Realtime CSV is detached from the integer 0.5 ms "
            "sequence")

    raw = frame[[f"y_{index}" for index in range(36, 156)]].to_numpy(
        np.float64
    ).reshape(SAMPLE_COUNT, len(WHEELS), 5, 3)
    require_finite(raw, "SIMPACK raw patch forces")
    active = raw[:, :, :, 0] < 0.0
    patch_count = np.sum(active, axis=2, dtype=np.int64)
    canonical = raw.copy()
    canonical[:, :, :, 0] = -raw[:, :, :, 0]
    masked_n = np.where(active, canonical[:, :, :, 0], -np.inf)
    primary_ordinal = np.argmax(masked_n, axis=2).astype(np.int64)
    primary_ordinal[patch_count == 0] = -1
    sample_indices = np.arange(SAMPLE_COUNT, dtype=np.int64)[:, None]
    wheel_indices = np.arange(len(WHEELS), dtype=np.int64)[None, :]
    selected = canonical[
        sample_indices, wheel_indices, np.maximum(primary_ordinal, 0), :
    ]
    selected[patch_count == 0] = np.nan

    def columns(begin: int, end: int) -> np.ndarray:
        value = frame[[f"y_{index}" for index in range(begin, end)]].to_numpy(
            np.float64
        )
        require_finite(value, f"SIMPACK y_{begin}..y_{end - 1}")
        return value

    control_rows = np.arange(0, SAMPLE_COUNT, 20, dtype=np.int64)
    require(control_rows.size == CONTROL_EVENT_COUNT,
            "internal SIMPACK control-row selection is incomplete")
    control_ordinal = np.arange(CONTROL_EVENT_COUNT, dtype=np.int64)
    control_time = time[control_rows]
    require(np.max(np.abs(
        control_time - control_ordinal.astype(np.float64) *
        CONTROL_PERIOD_SECONDS
    )) <= 4.0e-15,
            "SIMPACK every-20th rows are not U0..U3000")

    def trace(prefix: str, size: int) -> np.ndarray:
        names = tuple(str(index) for index in range(size))
        return frame_matrix(frame.iloc[control_rows], prefix, names,
                            "SIMPACK control trace")

    actual = trace("bridge_applied_wheel_torque_Nm_", len(WHEELS))
    plant_input = trace("u_", len(WHEELS))
    require(np.array_equal(actual, plant_input),
            "SIMPACK bridge-applied trace differs from the held plant input")
    control = {
        "ordinal": control_ordinal,
        "time": control_time,
        "request": trace("full_state_pi_request_Nm_", len(WHEELS)),
        "actual": actual,
        "dynamic_limit": trace("bridge_limit_wheel_torque_Nm_", len(WHEELS)),
        "limit_flags": require_integer_matrix(
            trace("bridge_limit_flags_", len(WHEELS)),
            "SIMPACK bridge flags",
        ),
        "base_speed": trace("v_base_wheel_mps_", len(WHEELS)),
        "reference_speed": trace("v_ref_wheel_mps_", len(WHEELS)),
        "delta_reference": trace("delta_omega_ref_radps_", len(AXLES)),
        "delta_measured": trace("delta_omega_meas_radps_", len(AXLES)),
        "delta_equilibrium": trace("delta_omega_eq_radps_", len(AXLES)),
        "filtered_lateral_velocity": trace("y_dot_hat_mps_", len(AXLES)),
        "filtered_yaw_rate": trace("psi_dot_hat_radps_", len(AXLES)),
        "guidance_active": require_integer_matrix(
            trace("curve_mode_active_", len(AXLES)),
            "SIMPACK guidance flags",
        ),
    }

    return {
        "time": time,
        "ticks": expected_ticks(),
        "station": columns(24, 28),
        "lateral": columns(8, 12),
        "yaw": columns(12, 16),
        "Q": -columns(28, 36),
        "patch_count": patch_count,
        "primary_ordinal": primary_ordinal,
        "control": control,
        "N": selected[:, :, 0],
        "Tx": selected[:, :, 1],
        "Ty": selected[:, :, 2],
        "metadata": metadata,
        "validation": validation,
        "source_sha256": {
            "realtime.csv": sha256_file(csv_path),
            "realtime.json": sha256_file(metadata_path),
            "validation_summary.json": sha256_file(validation_path),
        },
    }


def read_optional_wrl(csv_path: Path | None) -> dict[str, Any] | None:
    if csv_path is None:
        return None
    require(csv_path.is_file(), f"frozen WRL CSV is absent: {csv_path}")
    usecols = ["t"]
    usecols += [f"s_wheelset_{axle}_m" for axle in AXLES]
    usecols += [f"y_wheelset_{axle}_m" for axle in AXLES]
    usecols += [f"yaw_wheelset_{axle}_rad" for axle in AXLES]
    usecols += [f"n_patches_wheel_{wheel}" for wheel in WHEELS]
    for quantity in ("N", "Tx", "Ty"):
        usecols += [
            f"{quantity}_maxN_patch_wheel_{wheel}_N" for wheel in WHEELS
        ]
    frame = pd.read_csv(csv_path, usecols=usecols, float_precision="round_trip")
    require(frame.shape[0] == 300_001,
            "frozen WRL CSV does not have the native 0.1 ms 30 s clock")
    native_time = frame["t"].to_numpy(np.float64)
    require(np.max(np.abs(
        native_time - np.arange(300_001, dtype=np.float64) * 0.0001
    )) <= 4.0e-15, "frozen WRL CSV is not the native 0.1 ms clock")
    selected = np.arange(0, 300_001, 5, dtype=np.int64)
    time = native_time[selected]
    require(np.max(np.abs(
        time - expected_ticks().astype(np.float64) * 1.0e-9
    )) <= 4.0e-15, "frozen WRL does not contain every common 0.5 ms instant")

    def matrix(prefix: str, suffix: str) -> np.ndarray:
        value = frame[
            [f"{prefix}{axle}{suffix}" for axle in AXLES]
        ].to_numpy(np.float64)[selected]
        require_finite(value, f"frozen WRL {prefix}")
        return value

    patch_count = frame[
        [f"n_patches_wheel_{wheel}" for wheel in WHEELS]
    ].to_numpy(np.float64)[selected]
    require_finite(patch_count, "frozen WRL patch count")
    require(np.array_equal(patch_count, np.rint(patch_count)) and
            bool(np.all(patch_count >= 0.0)),
            "frozen WRL patch count is not nonnegative integer data")
    patch_count = np.rint(patch_count).astype(np.int64)

    local_force: dict[str, np.ndarray] = {}
    for quantity in ("N", "Tx", "Ty"):
        value = frame[
            [f"{quantity}_maxN_patch_wheel_{wheel}_N" for wheel in WHEELS]
        ].to_numpy(np.float64)[selected]
        value[patch_count == 0] = np.nan
        require(bool(np.isfinite(value[patch_count > 0]).all()),
                f"frozen WRL {quantity} is non-finite at a positive count")
        local_force[quantity] = value

    return {
        "time": time,
        "station": matrix("s_wheelset_", "_m"),
        "lateral": matrix("y_wheelset_", "_m"),
        "yaw": matrix("yaw_wheelset_", "_rad"),
        "patch_count": patch_count,
        **local_force,
        "source_sha256": sha256_file(csv_path),
    }


def read_optional_wrl_control(npz_path: Path | None) -> dict[str, Any] | None:
    if npz_path is None:
        return None
    require(npz_path.is_file(),
            f"frozen WRL control NPZ is absent: {npz_path}")
    required = {
        "axis_names", "wheel_names", "control_tick", "nominal_time_s",
        "wheel_speed_pi_request_Nm", "proxy_applied_torque_record_Nm",
        "plant_applied_torque_observed_Nm",
    }
    with np.load(npz_path, allow_pickle=False) as archive:
        missing = sorted(required - set(archive.files))
        require(not missing,
                f"frozen WRL control NPZ is missing arrays: {missing}")
        require(tuple(archive["axis_names"].tolist()) == AXLES and
                tuple(archive["wheel_names"].tolist()) == tuple(
                    f"{axle}_{side}" for axle in AXLES for side in ("L", "R")
                ), "frozen WRL control channel order differs from G77")
        native_tick = np.asarray(archive["control_tick"], dtype=np.int64)
        native_time = np.asarray(archive["nominal_time_s"], dtype=np.float64)
        request = np.asarray(
            archive["wheel_speed_pi_request_Nm"], dtype=np.float64
        )
        proxy_applied = np.asarray(
            archive["proxy_applied_torque_record_Nm"], dtype=np.float64
        )
        plant_applied = np.asarray(
            archive["plant_applied_torque_observed_Nm"], dtype=np.float64
        )
    require(np.array_equal(
        native_tick, np.arange(CONTROL_EVENT_COUNT, dtype=np.int64)
    ), "frozen WRL control rows are not the standardized 0..3000 labels")
    require(np.max(np.abs(
        native_time - native_tick.astype(np.float64) * CONTROL_PERIOD_SECONDS
    )) <= 4.0e-15, "frozen WRL nominal control time is malformed")
    for value, name in (
        (request, "request"), (proxy_applied, "proxy applied"),
        (plant_applied, "plant applied"),
    ):
        require(value.shape == (CONTROL_EVENT_COUNT, len(WHEELS)),
                f"frozen WRL {name} array has the wrong shape")
        require_finite(value, f"frozen WRL {name}")
    require(np.array_equal(proxy_applied, plant_applied),
            "frozen WRL proxy and plant-applied torque records differ")
    for value, name in ((request, "request"),
                        (plant_applied, "applied")):
        require(np.array_equal(value[2_432], value[2_431]),
                f"frozen WRL {name} does not carry the known row-2432 repeat")

    # Rows 0..2431 carry U0..U2431; row 2432 repeats U2431; rows
    # 2433..3000 carry U2432..U2999.  This relabels control records only and
    # never moves the vehicle's mechanical time or station series.
    keep = np.concatenate((
        np.arange(0, 2_432, dtype=np.int64),
        np.arange(2_433, CONTROL_EVENT_COUNT, dtype=np.int64),
    ))
    require(keep.size == 3_000,
            "internal frozen WRL control relabeling is incomplete")
    return {
        "ordinal": np.arange(3_000, dtype=np.int64),
        "request": request[keep],
        "actual": plant_applied[keep],
        "source_sha256": sha256_file(npz_path),
        "relabeling": (
            "rows 0..2431 -> U0..U2431; row 2432 duplicate U2431 omitted; "
            "rows 2433..3000 -> U2432..U2999; U3000 absent"
        ),
    }


def interpolate_macro(source: dict[str, Any], quantity: str,
                      axle_index: int, relative_to_initial: bool) -> np.ndarray:
    station = source["station"][:, axle_index]
    value = source[quantity][:, axle_index]
    require(bool(np.all(np.diff(station) > 0.0)) and
            station[0] <= STATION_GRID[0] and station[-1] >= STATION_GRID[-1],
            f"source axle {AXLES[axle_index]} does not cover 100--450 m")
    if relative_to_initial:
        value = value - value[0]
    return np.interp(STATION_GRID, station, value)


def interpolate_primary_patch_segments(
    station: np.ndarray, value: np.ndarray, primary_ordinal: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    require(station.shape == value.shape == primary_ordinal.shape,
            "patch-local station interpolation inputs have different shapes")
    require(bool(np.all(np.diff(station) > 0.0)),
            "patch-local station is not strictly increasing")
    output = np.full(STATION_GRID.shape, np.nan, dtype=np.float64)
    output_ordinal = np.full(STATION_GRID.shape, -1, dtype=np.int64)
    begin = 0
    while begin < station.size:
        ordinal = int(primary_ordinal[begin])
        end = begin + 1
        while end < station.size and int(primary_ordinal[end]) == ordinal:
            end += 1
        if ordinal >= 0:
            finite = np.isfinite(value[begin:end])
            require(bool(np.all(finite)),
                    "active primary patch contains a non-finite local force")
            if end - begin >= 2:
                selected = ((STATION_GRID >= station[begin]) &
                            (STATION_GRID <= station[end - 1]))
                output[selected] = np.interp(
                    STATION_GRID[selected], station[begin:end], value[begin:end]
                )
                output_ordinal[selected] = ordinal
        begin = end
    return output, output_ordinal


def macro_statistics(
    orvd: dict[str, Any], simpack: dict[str, Any],
    wrl: dict[str, Any] | None,
) -> tuple[dict[str, Any], dict[str, dict[str, np.ndarray]],
           dict[str, dict[str, np.ndarray]]]:
    sources = {"orvd": orvd, "simpack": simpack}
    if wrl is not None:
        sources["wrl"] = wrl
    time_values: dict[str, dict[str, np.ndarray]] = {name: {} for name in sources}
    station_values: dict[str, dict[str, np.ndarray]] = {
        name: {} for name in sources
    }
    output: dict[str, Any] = {"same_time": {}, "same_station": {}}
    for axle_index, axle in enumerate(AXLES):
        output["same_time"][axle] = {}
        output["same_station"][axle] = {}
        for quantity in ("lateral", "yaw"):
            key = f"{axle}.{quantity}"
            for name, source in sources.items():
                time_values[name][key] = source[quantity][:, axle_index]
                station_values[name][key] = interpolate_macro(
                    source, quantity, axle_index, False
                )
                station_values[name][f"{key}.increment"] = interpolate_macro(
                    source, quantity, axle_index, True
                )
            scale = 1.0e6
            time_result = {
                "orvd_minus_simpack_micro": original_and_increment_statistics(
                    time_values["orvd"][key], time_values["simpack"][key],
                    orvd["time"], scale,
                )
            }
            station_result = {
                "orvd_minus_simpack_micro": {
                    "original": statistics(
                        (station_values["orvd"][key] -
                         station_values["simpack"][key]) * scale,
                        STATION_GRID,
                    ),
                    "relative_to_each_source_initial": statistics(
                        (station_values["orvd"][f"{key}.increment"] -
                         station_values["simpack"][f"{key}.increment"]) * scale,
                        STATION_GRID,
                    ),
                }
            }
            if wrl is not None:
                time_result["wrl_minus_simpack_micro"] = (
                    original_and_increment_statistics(
                        time_values["wrl"][key], time_values["simpack"][key],
                        orvd["time"], scale,
                    )
                )
                station_result["wrl_minus_simpack_micro"] = {
                    "original": statistics(
                        (station_values["wrl"][key] -
                         station_values["simpack"][key]) * scale,
                        STATION_GRID,
                    ),
                    "relative_to_each_source_initial": statistics(
                        (station_values["wrl"][f"{key}.increment"] -
                         station_values["simpack"][f"{key}.increment"]) * scale,
                        STATION_GRID,
                    ),
                }
            output["same_time"][axle][quantity] = time_result
            output["same_station"][axle][quantity] = station_result
    return output, time_values, station_values


def contact_statistics(
    orvd: dict[str, Any], simpack: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, dict[str, np.ndarray]],
           dict[str, dict[str, np.ndarray]]]:
    time_values = {
        quantity: {"orvd": orvd[quantity], "simpack": simpack[quantity]}
        for quantity in ("Q", "N", "Tx", "Ty")
    }
    station_values = {
        quantity: {
            "orvd": np.full((STATION_GRID.size, len(WHEELS)), np.nan),
            "simpack": np.full((STATION_GRID.size, len(WHEELS)), np.nan),
        }
        for quantity in ("Q", "N", "Tx", "Ty")
    }
    station_primary = {
        source: np.full((STATION_GRID.size, len(WHEELS)), -1, dtype=np.int64)
        for source in ("orvd", "simpack")
    }
    output: dict[str, Any] = {"same_time": {}, "same_station": {}}
    for wheel_index, wheel in enumerate(WHEELS):
        axle_index = wheel_index // 2
        output["same_time"][wheel] = {}
        output["same_station"][wheel] = {}
        for source_name, source in (("orvd", orvd), ("simpack", simpack)):
            station = source["station"][:, axle_index]
            require(bool(np.all(np.diff(station) > 0.0)) and
                    station[0] <= STATION_GRID[0] and
                    station[-1] >= STATION_GRID[-1],
                    f"{source_name} {wheel} does not cover 100--450 m")
            station_values["Q"][source_name][:, wheel_index] = np.interp(
                STATION_GRID, station, source["Q"][:, wheel_index]
            )
            for quantity in ("N", "Tx", "Ty"):
                interpolated, ordinal = interpolate_primary_patch_segments(
                    station, source[quantity][:, wheel_index],
                    source["primary_ordinal"][:, wheel_index],
                )
                station_values[quantity][source_name][:, wheel_index] = interpolated
                if quantity == "N":
                    station_primary[source_name][:, wheel_index] = ordinal

        output["same_time"][wheel]["Q"] = original_and_increment_statistics(
            orvd["Q"][:, wheel_index], simpack["Q"][:, wheel_index],
            orvd["time"], 1.0,
        )
        output["same_station"][wheel]["Q"] = {
            "original": statistics(
                station_values["Q"]["orvd"][:, wheel_index] -
                station_values["Q"]["simpack"][:, wheel_index], STATION_GRID
            )
        }
        for quantity in ("N", "Tx", "Ty"):
            time_valid = (
                np.isfinite(orvd[quantity][:, wheel_index]) &
                np.isfinite(simpack[quantity][:, wheel_index])
            )
            station_valid = (
                np.isfinite(station_values[quantity]["orvd"][:, wheel_index]) &
                np.isfinite(station_values[quantity]["simpack"][:, wheel_index])
            )
            output["same_time"][wheel][quantity] = masked_statistics(
                orvd[quantity][:, wheel_index] -
                simpack[quantity][:, wheel_index],
                time_valid, orvd["time"],
            )
            output["same_station"][wheel][quantity] = masked_statistics(
                station_values[quantity]["orvd"][:, wheel_index] -
                station_values[quantity]["simpack"][:, wheel_index],
                station_valid, STATION_GRID,
            )
    output["same_station_primary_selection_coverage"] = {
        source: {
            "selection_semantics": (
                "true returned patch ordinal" if source == "orvd" else
                "maximum-N active Type-78 source-local slot proxy"
            ),
            "per_wheel": {
                wheel: {
                    "available_grid_points": int(np.count_nonzero(
                        station_primary[source][:, index] >= 0)),
                    "available_fraction": float(np.count_nonzero(
                        station_primary[source][:, index] >= 0) /
                                                STATION_GRID.size),
                }
                for index, wheel in enumerate(WHEELS)
            },
        }
        for source in ("orvd", "simpack")
    }
    return output, time_values, station_values


def per_channel_continuous_statistics(
    left: np.ndarray, right: np.ndarray, names: tuple[str, ...],
    coordinate: np.ndarray,
) -> dict[str, Any]:
    require(left.shape == right.shape == (coordinate.size, len(names)),
            "control comparison arrays have the wrong shape")
    return {
        name: statistics(left[:, index] - right[:, index], coordinate)
        for index, name in enumerate(names)
    }


def discrete_mismatch_statistics(
    left: np.ndarray, right: np.ndarray, names: tuple[str, ...],
) -> dict[str, Any]:
    require(left.shape == right.shape and left.shape[1] == len(names),
            "discrete control comparison arrays have the wrong shape")
    mismatch = left != right
    return {
        "event_count": int(left.shape[0]),
        "total_channel_mismatch_count": int(np.count_nonzero(mismatch)),
        "event_with_any_mismatch_count": int(np.count_nonzero(
            np.any(mismatch, axis=1)
        )),
        "per_channel_mismatch_count": {
            name: int(np.count_nonzero(mismatch[:, index]))
            for index, name in enumerate(names)
        },
    }


def control_event_statistics(
    orvd: dict[str, Any], simpack: dict[str, Any],
    wrl: dict[str, Any] | None,
) -> dict[str, Any]:
    orvd_control = orvd["control"]
    simpack_control = simpack["control"]
    require(np.array_equal(orvd_control["ordinal"],
                           simpack_control["ordinal"]),
            "ORVD and SIMPACK control events are not U0..U3000")
    coordinate = orvd_control["ordinal"].astype(np.float64)
    continuous = (
        ("requested_torque_newton_metres", "request", WHEELS),
        ("conditioned_actual_torque_newton_metres", "actual", WHEELS),
        ("dynamic_torque_limit_newton_metres", "dynamic_limit", WHEELS),
        ("base_speed_reference_meters_per_second", "base_speed", WHEELS),
        ("wheel_speed_reference_meters_per_second", "reference_speed", WHEELS),
        ("delta_omega_reference_radians_per_second", "delta_reference", AXLES),
        ("delta_omega_measured_radians_per_second", "delta_measured", AXLES),
        ("delta_omega_equilibrium_radians_per_second",
         "delta_equilibrium", AXLES),
        ("filtered_lateral_velocity_meters_per_second",
         "filtered_lateral_velocity", AXLES),
        ("filtered_yaw_rate_radians_per_second", "filtered_yaw_rate", AXLES),
    )
    output: dict[str, Any] = {
        "contract": {
            "event_identity": "integer U0..U3000; no timestamp pairing",
            "event_count": CONTROL_EVENT_COUNT,
            "acceptance_threshold": None,
        },
        "orvd_minus_simpack_realtime": {
            "continuous": {
                label: per_channel_continuous_statistics(
                    orvd_control[field], simpack_control[field], names,
                    coordinate,
                )
                for label, field, names in continuous
            },
            "discrete": {
                "conditioner_limit_flags": discrete_mismatch_statistics(
                    orvd_control["limit_flags"],
                    simpack_control["limit_flags"], WHEELS,
                ),
                "guidance_active": discrete_mismatch_statistics(
                    orvd_control["guidance_active"],
                    simpack_control["guidance_active"], AXLES,
                ),
            },
        },
    }
    if wrl is not None:
        require(np.array_equal(wrl["ordinal"],
                               np.arange(3_000, dtype=np.int64)),
                "frozen WRL relabeled control events are not U0..U2999")
        wrl_coordinate = wrl["ordinal"].astype(np.float64)
        output["frozen_wrl_historical_minus_simpack_realtime"] = {
            "event_identity": wrl["relabeling"],
            "event_count": 3_000,
            "requested_torque_newton_metres": per_channel_continuous_statistics(
                wrl["request"], simpack_control["request"][:3_000], WHEELS,
                wrl_coordinate,
            ),
            "applied_torque_newton_metres": per_channel_continuous_statistics(
                wrl["actual"], simpack_control["actual"][:3_000], WHEELS,
                wrl_coordinate,
            ),
            "acceptance_threshold": None,
        }
    return output


def longest_true_run(mask: np.ndarray) -> dict[str, Any]:
    require(mask.ndim == 1 and mask.dtype == np.bool_,
            "run-length input is not one boolean series")
    padded = np.concatenate((np.array([False]), mask, np.array([False])))
    changes = np.diff(padded.astype(np.int8))
    begins = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1)
    if begins.size == 0:
        return {
            "sample_count": 0,
            "first_sample_index": None,
            "last_sample_index": None,
            "sample_span_seconds": 0.0,
        }
    lengths = ends - begins
    selected = int(np.argmax(lengths))
    count = int(lengths[selected])
    begin = int(begins[selected])
    end = int(ends[selected] - 1)
    return {
        "sample_count": count,
        "first_sample_index": begin,
        "last_sample_index": end,
        "sample_span_seconds": float(max(0, count - 1) *
                                     OBSERVATION_PERIOD_SECONDS),
    }


def contact_count_summary(
    source: dict[str, Any], count_semantics: str,
) -> dict[str, Any]:
    count = source["patch_count"]
    primary = source["primary_ordinal"]
    interfaces_with_positive_count = np.sum(count > 0, axis=1)
    wheel_output: dict[str, Any] = {}
    for index, wheel in enumerate(WHEELS):
        active_neighbors = ((primary[1:, index] >= 0) &
                            (primary[:-1, index] >= 0))
        switches = active_neighbors & (
            primary[1:, index] != primary[:-1, index]
        )
        exact_zero_q = source["Q"][:, index] == 0.0
        wheel_output[wheel] = {
            "zero_count_sample_count": int(np.count_nonzero(count[:, index] == 0)),
            "count_equal_to_two_sample_count": int(np.count_nonzero(
                count[:, index] == 2)),
            "count_at_least_three_sample_count": int(np.count_nonzero(
                count[:, index] >= 3)),
            "peak_count": int(np.max(count[:, index])),
            "primary_selected_source_local_ordinal_switch_count_between_"
            "adjacent_positive_count_samples":
                int(np.count_nonzero(switches)),
            "longest_zero_count_run": longest_true_run(count[:, index] == 0),
            "exact_zero_Q_sample_count": int(np.count_nonzero(exact_zero_q)),
            "longest_exact_zero_Q_run": longest_true_run(exact_zero_q),
        }
    return {
        "count_semantics": count_semantics,
        "minimum_interfaces_with_positive_count": int(np.min(
            interfaces_with_positive_count)),
        "samples_with_fewer_than_eight_positive_counts": int(np.count_nonzero(
            interfaces_with_positive_count < 8)),
        "longest_any_interface_zero_count_run": longest_true_run(
            interfaces_with_positive_count < 8),
        "per_wheel": wheel_output,
        "interpretation": (
            "sampled count observation only; no exact event-time, exact "
            "detachment, cross-source topology or safety gate"
        ),
    }


def plot_macro(
    path: Path, coordinate: np.ndarray,
    values: dict[str, dict[str, np.ndarray]], x_label: str, title: str,
) -> None:
    source_order = tuple(values)
    figure, axes = plt.subplots(2, 4, figsize=(16, 7), sharex=True)
    handles = []
    for axle_index, axle in enumerate(AXLES):
        for quantity_index, quantity in enumerate(("lateral", "yaw")):
            axis = axes[quantity_index, axle_index]
            key = f"{axle}.{quantity}"
            for source in source_order:
                line, = axis.plot(coordinate, values[source][key],
                                  label=PLOT_LABELS[source],
                                  **PLOT_STYLES[source])
                if axle_index == 0 and quantity_index == 0:
                    handles.append(line)
            axis.set_title(f"{axle} {quantity}")
            axis.set_ylabel("Lateral [m]" if quantity == "lateral" else
                            "Yaw [rad]")
            if quantity_index == 1:
                axis.set_xlabel(x_label)
            axis.grid(True, alpha=0.25)
    figure.suptitle(title, y=0.985)
    figure.legend(handles, [PLOT_LABELS[source] for source in source_order],
                  loc="upper center", ncol=len(source_order),
                  bbox_to_anchor=(0.5, 0.955), frameon=False)
    figure.subplots_adjust(left=0.065, right=0.99, bottom=0.08, top=0.88,
                           hspace=0.30, wspace=0.26)
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_contact(
    path: Path, coordinate: np.ndarray,
    values: dict[str, np.ndarray], quantity: str, x_label: str, title: str,
) -> None:
    figure, axes = plt.subplots(2, 4, figsize=(16, 7), sharex=True)
    handles = []
    for wheel_index, wheel in enumerate(WHEELS):
        axis = axes.flat[wheel_index]
        for source in ("orvd", "simpack"):
            line, = axis.plot(coordinate, values[source][:, wheel_index],
                              label=PLOT_LABELS[source],
                              **PLOT_STYLES[source])
            if wheel_index == 0:
                handles.append(line)
        axis.set_title(wheel)
        axis.set_ylabel("Force [N]")
        if wheel_index >= 4:
            axis.set_xlabel(x_label)
        axis.grid(True, alpha=0.25)
    figure.suptitle(f"{title}: {quantity}", y=0.985)
    figure.legend(handles, [PLOT_LABELS["orvd"], PLOT_LABELS["simpack"]],
                  loc="upper center", ncol=2, bbox_to_anchor=(0.5, 0.955),
                  frameon=False)
    figure.subplots_adjust(left=0.065, right=0.99, bottom=0.08, top=0.88,
                           hspace=0.30, wspace=0.26)
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_readme_macro_summary(
    path: Path, coordinate: np.ndarray,
    values: dict[str, dict[str, np.ndarray]],
) -> None:
    source_order = tuple(values)
    figure, axes = plt.subplots(2, 4, figsize=(16, 7), sharex=True)
    handles = []
    for axle_index, axle in enumerate(AXLES):
        for quantity_index, quantity in enumerate(("lateral", "yaw")):
            axis = axes[quantity_index, axle_index]
            key = f"{axle}.{quantity}"
            for source in source_order:
                line, = axis.plot(
                    coordinate, values[source][key] * 1.0e3,
                    label=PLOT_LABELS[source], **PLOT_STYLES[source],
                )
                if axle_index == 0 and quantity_index == 0:
                    handles.append(line)
            axis.set_title(axle.upper())
            axis.set_ylabel(
                "Lateral displacement [mm]" if quantity == "lateral"
                else "Yaw angle [mrad]"
            )
            if quantity_index == 1:
                axis.set_xlabel("Time [s]")
            axis.grid(True, alpha=0.25)
    figure.suptitle(
        "IRW P179 controlled response: native 0.5 ms same-time comparison",
        y=0.985,
    )
    figure.legend(
        handles, [PLOT_LABELS[source] for source in source_order],
        loc="upper center", ncol=len(source_order),
        bbox_to_anchor=(0.5, 0.955), frameon=False,
    )
    figure.subplots_adjust(
        left=0.075, right=0.99, bottom=0.08, top=0.88,
        hspace=0.30, wspace=0.28,
    )
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_readme_force_summary(
    path: Path, time: np.ndarray,
    time_values: dict[str, dict[str, np.ndarray]],
    station_values: dict[str, dict[str, np.ndarray]],
    statistics_by_domain: dict[str, Any],
    wrl: dict[str, Any] | None,
) -> None:
    quantities = ("N", "Tx", "Ty")
    figure, axes = plt.subplots(3, 2, figsize=(14, 10), sharex="col")
    handles_by_source: dict[str, Any] = {}
    for row, quantity in enumerate(quantities):
        wheel_index = max(
            range(len(WHEELS)),
            key=lambda index: statistics_by_domain["same_time"][
                WHEELS[index]
            ][quantity]["maximum_absolute"],
        )
        wheel = WHEELS[wheel_index]
        time_axis = axes[row, 0]
        for source in ("orvd", "simpack"):
            line, = time_axis.plot(
                time, time_values[quantity][source][:, wheel_index] / 1.0e3,
                label=PLOT_LABELS[source], **PLOT_STYLES[source],
            )
            handles_by_source.setdefault(source, line)
        if wrl is not None:
            line, = time_axis.plot(
                wrl["time"], wrl[quantity][:, wheel_index] / 1.0e3,
                label=PLOT_LABELS["wrl"], **PLOT_STYLES["wrl"],
            )
            handles_by_source.setdefault("wrl", line)
        time_axis.set_title(f"{quantity}, {wheel.upper()}: native time")
        time_axis.set_ylabel("Contact force [kN]")
        time_axis.grid(True, alpha=0.25)

        station_axis = axes[row, 1]
        for source in ("orvd", "simpack"):
            station_axis.plot(
                STATION_GRID,
                station_values[quantity][source][:, wheel_index] / 1.0e3,
                label=PLOT_LABELS[source], **PLOT_STYLES[source],
            )
        station_axis.set_title(f"{quantity}, {wheel.upper()}: same station")
        station_axis.set_ylabel("Contact force [kN]")
        station_axis.grid(True, alpha=0.25)
    axes[-1, 0].set_xlabel("Time [s]")
    axes[-1, 1].set_xlabel("Own axle-bridge station [m]")
    source_order = tuple(
        source for source in ("orvd", "simpack", "wrl")
        if source in handles_by_source
    )
    figure.suptitle(
        "IRW P179 controlled primary-patch force comparison",
        y=0.99,
    )
    figure.legend(
        [handles_by_source[source] for source in source_order],
        [PLOT_LABELS[source] for source in source_order],
        loc="upper center", ncol=len(source_order),
        bbox_to_anchor=(0.5, 0.965), frameon=False,
    )
    figure.subplots_adjust(
        left=0.08, right=0.99, bottom=0.07, top=0.91,
        hspace=0.32, wspace=0.20,
    )
    figure.savefig(path, dpi=220)
    plt.close(figure)


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orvd-artifact", type=Path, required=True)
    parser.add_argument("--orvd-execution-identity", type=Path, required=True)
    parser.add_argument("--simpack-realtime-directory", type=Path,
                        required=True)
    parser.add_argument("--wrl-csv", type=Path)
    parser.add_argument("--wrl-control-npz", type=Path)
    parser.add_argument("--output-directory", type=Path, required=True)
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    output = arguments.output_directory.resolve()
    if output.exists():
        print(f"G77 analysis failed: output already exists: {output}",
              file=sys.stderr)
        return 1
    try:
        orvd = read_orvd(
            arguments.orvd_artifact.resolve(),
            arguments.orvd_execution_identity.resolve(),
        )
        simpack = read_simpack(arguments.simpack_realtime_directory.resolve())
        wrl = read_optional_wrl(
            arguments.wrl_csv.resolve() if arguments.wrl_csv is not None else None
        )
        wrl_control = read_optional_wrl_control(
            arguments.wrl_control_npz.resolve()
            if arguments.wrl_control_npz is not None else None
        )
        require(np.array_equal(orvd["ticks"], simpack["ticks"]) and
                np.max(np.abs(orvd["time"] - simpack["time"])) <= 4.0e-15,
                "ORVD and SIMPACK are not the same integer time sequence")
        if wrl is not None:
            require(np.max(np.abs(wrl["time"] - orvd["time"])) <= 4.0e-15,
                    "frozen WRL common instants differ from the native clock")

        macro_stats, macro_time_values, macro_station_values = macro_statistics(
            orvd, simpack, wrl
        )
        contact_stats, contact_time_values, contact_station_values = (
            contact_statistics(orvd, simpack)
        )
        control_stats = control_event_statistics(orvd, simpack, wrl_control)
        payload = {
            "classification": "irw_G77_controlled_long_window_comparison",
            "scope": (
                "IRW H3 + R300 + frozen AAR5 + 100 Hz full-state control, "
                "30 s"
            ),
            "contract": {
                "native_time_clock": {
                    "period_nanoseconds": OBSERVATION_PERIOD_NS,
                    "sample_count": SAMPLE_COUNT,
                    "terminal_time_nanoseconds": TERMINAL_TIME_NS,
                    "pairing": "same integer sample index; no time interpolation",
                },
                "control_event_clock": {
                    "period_seconds": CONTROL_PERIOD_SECONDS,
                    "event_count": CONTROL_EVENT_COUNT,
                    "pairing": "same integer event ordinal U0..U3000",
                    "timestamp_pairing_forbidden": True,
                },
                "same_station_grid_meters": [100.0, 450.0, 0.01],
                "macro_statistics": (
                    "original and relative-to-each-source-t=0 RMS/maximum"
                ),
                "transformations": (
                    "no time/station shift, filtering, fitting, demeaning or "
                    "result-related scaling; ORVD source_body_yaw and SIMPACK "
                    "axle yaw share the frozen P179 source physical body basis"
                ),
                "simpack_contact_sign": (
                    "Q=-raw QVertical; N=-raw_minus_n; wheel-side "
                    "Tx=raw_minus_tx and Ty=raw_minus_ty"
                ),
                "primary_selection": (
                    "ORVD selects the maximum-N returned patch; SIMPACK selects "
                    "the maximum-N active Type-78 slot proxy; ties retain the "
                    "lowest source-local ordinal"
                ),
                "local_force_station_interpolation": (
                    "only inside contiguous positive-count runs with unchanged "
                    "source-local selected ordinal; never across zero counts or "
                    "selection switches"
                ),
                "cross_patch_tx_ty_summation": "forbidden",
                "acceptance_threshold": None,
            },
            "macro_response_error": macro_stats,
            "contact_force_error_newtons": contact_stats,
            "control_event_error": control_stats,
            "contact_count_and_primary_selection_observation": {
                "orvd": contact_count_summary(
                    orvd,
                    "true WheelRailContactModel returned patch count",
                ),
                "simpack_realtime": contact_count_summary(
                    simpack,
                    "active compressive Type-78 slot proxy count from "
                    "raw_minus_n < 0; not result.count/ch_022",
                ),
                "not_an_exact_topology_or_safety_gate": True,
            },
            "performance": {
                "orvd_artifact": orvd["performance"],
                "orvd_process_wall_seconds":
                    orvd["execution"].get("process_wall_seconds"),
                "orvd_peak_resident_set_kibibytes":
                    orvd["execution"].get("maximum_resident_set_kilobytes"),
                "simpack_realtime": simpack["validation"].get("performance"),
                "comparison_note": (
                    "solver and workflow boundaries differ; these are observed "
                    "execution costs, not a common speed gate"
                ),
            },
            "source_identity": {
                "orvd": orvd["source_sha256"],
                "orvd_revision": orvd["execution"].get("orvd_revision"),
                "orvd_executable_sha256":
                    orvd["execution"].get("executable_sha256"),
                "simpack_realtime": simpack["source_sha256"],
                "optional_frozen_wrl_csv_sha256": (
                    None if wrl is None else wrl["source_sha256"]
                ),
                "optional_frozen_wrl_control_npz_sha256": (
                    None if wrl_control is None else
                    wrl_control["source_sha256"]
                ),
            },
        }

        output.mkdir(parents=True)
        plot_macro(
            output / "irw_g77_macro_same_time.png", orvd["time"],
            macro_time_values, "Time [s]",
            "IRW G77 controlled response: native 0.5 ms same time",
        )
        station_plot_values = {
            source: {
                key: value for key, value in values.items()
                if not key.endswith(".increment")
            }
            for source, values in macro_station_values.items()
        }
        plot_macro(
            output / "irw_g77_macro_same_station.png", STATION_GRID,
            station_plot_values, "Own axle-bridge station [m]",
            "IRW G77 controlled response: same station over 100--450 m",
        )
        for quantity in ("Q", "N", "Tx", "Ty"):
            plot_contact(
                output / f"irw_g77_contact_{quantity}_same_time.png",
                orvd["time"], contact_time_values[quantity], quantity,
                "Time [s]", "IRW G77 native 0.5 ms wheel response",
            )
            plot_contact(
                output / f"irw_g77_contact_{quantity}_same_station.png",
                STATION_GRID, contact_station_values[quantity], quantity,
                "Own axle-bridge station [m]",
                "IRW G77 wheel response over 100--450 m",
            )
        plot_readme_macro_summary(
            output / "irw_p179_controlled_30s_axlebridge_response.png",
            orvd["time"], macro_time_values,
        )
        plot_readme_force_summary(
            output / "irw_p179_controlled_30s_wheel_force_response.png",
            orvd["time"], contact_time_values, contact_station_values,
            contact_stats, wrl,
        )
        payload["figures"] = sorted(path.name for path in output.glob("*.png"))
        write_json(output / "irw_g77_analysis.json", payload)
    except (AnalysisError, KeyError, OSError, ValueError) as error:
        print(f"G77 analysis failed: {error}", file=sys.stderr)
        return 1
    print(f"wrote G77 analysis to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
