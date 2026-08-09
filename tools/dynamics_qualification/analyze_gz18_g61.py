#!/usr/bin/env python3
"""Compare one ORVD G61 artifact with the frozen P057 20 s references.

This migration-only entry point accepts only the P057 full-envelope corpus.
It uses same-station centimetre grids for the primary macro-response gate and
keeps same-time comparisons as propagation diagnostics.  It never searches
for references, shifts a series, or infers a sign from the result.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import shutil
import sys
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gz18_qualification_analysis_common import (  # noqa: E402
    AnalysisError,
    AXLES,
    COMMON_W_TO_ORVD_TRACK_T,
    CONTACT_QUANTITIES,
    MACRO_ALARM_LIMIT,
    MACRO_COORDINATE_FRAME,
    SAMPLE_PERIOD_SECONDS,
    SIDES,
    column_stack,
    common_w_macro_in_orvd_track_frame,
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


SAMPLE_COUNT = 40_001
TERMINAL_TIME_SECONDS = 20.0
TERMINAL_TIME_NANOSECONDS = 20_000_000_000
STATION_STEP_METERS = 0.01
MINIMUM_COMMON_END_METERS = 324.0
P057_HISTORICAL_WRL_SOURCE_REVISION = (
    "b4466a40ff4cd74ae7ae7ad2f7fa858e952cf563"
)

SEGMENTS = (
    ("pre_activation", None, 49.99),
    ("fade_in_50_100m", 50.0, 100.0),
    ("full_100_250m", 100.0, 250.0),
    ("fade_out_250_300m", 250.0, 300.0),
    ("recovery_after_300m", 300.01, None),
    ("excitation_50_300m", 50.0, 300.0),
)

PAIR_KEYS = {
    "time_s", "axle_names", "wheel_names",
    *(f"{source}_{quantity}" for source in ("simpack", "drake")
      for quantity in (
          "station_m", "lateral_m", "yaw_rad", "Q_N", "N_N", "Tx_N",
          "Ty_N", "patch_count",
      )),
}


def load_p057_pair(path: Path) -> dict[str, np.ndarray]:
    require(path.is_file(), f"P057 paired reference does not exist: {path}")
    try:
        with np.load(path, allow_pickle=False) as archive:
            require(set(archive.files) == PAIR_KEYS,
                    "P057 paired reference has an unexpected key set")
            result = {key: np.array(archive[key], copy=True) for key in archive.files}
    except (OSError, ValueError) as error:
        raise AnalysisError(f"could not load P057 paired reference: {error}") from error

    require(np.array_equal(result["axle_names"],
                           np.array([axle for axle, _, _ in AXLES])),
            "P057 axle names or order differ from ff/fr/rf/rr")
    expected_wheels = np.array([
        f"{axle}_{side[0]}" for axle, _, _ in AXLES for side in SIDES
    ])
    require(np.array_equal(result["wheel_names"], expected_wheels),
            "P057 wheel names or order differ from the named mapping")
    require(result["time_s"].shape == (SAMPLE_COUNT,) and
            np.array_equal(result["time_s"],
                           expected_times(SAMPLE_COUNT, TERMINAL_TIME_SECONDS)),
            "P057 time_s is not the exact 40001-point integer clock")

    for source in ("simpack", "drake"):
        for quantity in ("station_m", "lateral_m", "yaw_rad"):
            key = f"{source}_{quantity}"
            require(result[key].shape == (SAMPLE_COUNT, 4),
                    f"P057 {key} has the wrong shape")
            require_finite(result[key], f"P057 {key}")
        require(bool(np.all(np.diff(result[f"{source}_station_m"], axis=0) > 0.0)),
                f"P057 {source} stations are not strictly increasing")
        for quantity in ("Q_N", "N_N", "Tx_N", "Ty_N", "patch_count"):
            key = f"{source}_{quantity}"
            require(result[key].shape == (SAMPLE_COUNT, 8),
                    f"P057 {key} has the wrong shape")
            require_finite(result[key], f"P057 {key}")
        patches = result[f"{source}_patch_count"]
        require(bool(np.all(patches >= 0.0)) and
                np.array_equal(patches, np.rint(patches)),
                f"P057 {source} patch counts are not non-negative integers")
    return result


def validate_p057_native(path: Path, pair: dict[str, np.ndarray]) -> None:
    """Cross-check the paired wheel-end scalars against native Type-80 data."""

    require(path.is_file(), f"P057 native SIMPACK core does not exist: {path}")
    required = {"time_f32"}
    for axle, _, _ in AXLES:
        for side in SIDES:
            wheel = f"{axle}_{side[0]}"
            required.update(
                f"{wheel}__{quantity}"
                for quantity in ("Q", "N", "Tx", "Ty", "patch_count")
            )
    try:
        with np.load(path, allow_pickle=False) as archive:
            missing = sorted(required - set(archive.files))
            require(not missing, f"P057 native core is missing {missing}")
            native = {key: np.array(archive[key], copy=True) for key in required}
    except (OSError, ValueError) as error:
        raise AnalysisError(f"could not load P057 native SIMPACK core: {error}") from error

    expected_native_time = (
        np.arange(SAMPLE_COUNT, dtype=np.float64) * SAMPLE_PERIOD_SECONDS
    ).astype(np.float32)
    require(native["time_f32"].shape == (SAMPLE_COUNT,) and
            np.array_equal(native["time_f32"], expected_native_time),
            "P057 native time is not the binary32 cast of the 0.5 ms clock")
    for axle_index, (axle, _, _) in enumerate(AXLES):
        for side_index, side in enumerate(SIDES):
            wheel = f"{axle}_{side[0]}"
            wheel_index = 2 * axle_index + side_index
            for quantity in ("Q", "N", "patch_count"):
                require(np.array_equal(
                    native[f"{wheel}__{quantity}"].astype(np.float64),
                    pair[f"simpack_{quantity}_N" if quantity != "patch_count"
                         else "simpack_patch_count"][:, wheel_index],
                ), f"P057 native {wheel} {quantity} differs from paired data")
            for quantity in ("Tx", "Ty"):
                require(np.array_equal(
                    -native[f"{wheel}__{quantity}"].astype(np.float64),
                    pair[f"simpack_{quantity}_N"][:, wheel_index],
                ), f"P057 native {wheel} {quantity} was not negated exactly once")


def load_g60_identity(
    artifact_directory: Path, execution_identity_path: Path
) -> tuple[dict[str, Any], dict[str, Any]]:
    require(artifact_directory.is_dir(),
            f"G60 artifact directory does not exist: {artifact_directory}")
    metadata = load_json_object(
        artifact_directory / "metadata.json", "G60 ORVD metadata"
    )
    require(metadata.get("terminal_time_nanoseconds") == 10_000_000_000 and
            metadata.get("sample_period_nanoseconds") == 500_000 and
            metadata.get("sample_count") == 20_001,
            "G60 comparison identity is not the 10 s / 0.5 ms artifact")
    execution = load_execution_identity(
        execution_identity_path, artifact_directory, goal_name="G60"
    )
    return metadata, execution


def verify_g60_g61_execution_match(
    g60_metadata: dict[str, Any],
    g60_execution: dict[str, Any],
    g61_metadata: dict[str, Any],
    g61_execution: dict[str, Any],
) -> dict[str, Any]:
    execution_keys = (
        "build_type", "compiler", "hardware", "applied_cpu_affinity",
        "openmp_environment", "executable_sha256",
    )
    execution_equal = {
        key: g60_execution.get(key) == g61_execution.get(key)
        for key in execution_keys
    }
    require(all(execution_equal.values()),
            "G61 hardware/build/thread execution differs from G60")

    stable_numerical_keys = (
        "relative_tolerance",
        "generalized_position_absolute_tolerance",
        "generalized_velocity_absolute_tolerance",
        "series_force_absolute_tolerance_newtons",
        "openmp_dynamic_teams_enabled",
        "openmp_runtime_maximum_threads",
        "contact_batch_worker_cap",
        "contact_batch_requested_worker_count",
        "rhs_contact_projection_half_width_meters",
        "observation_projection_rule",
        "carrier_observation_projection_base_half_width_meters",
        "representative_body_observation_projection_base_half_width_meters",
    )
    g60_contract = g60_metadata.get("numerical_execution_contract")
    g61_contract = g61_metadata.get("numerical_execution_contract")
    require(isinstance(g60_contract, dict) and isinstance(g61_contract, dict),
            "G60/G61 numerical execution contract is absent")
    numerical_equal = {
        key: g60_contract.get(key) == g61_contract.get(key)
        for key in stable_numerical_keys
    }
    require(all(numerical_equal.values()),
            "G61 numerical tolerances or projection/thread personality differ from G60")
    return {
        "execution_identity_fields_equal": execution_equal,
        "stable_numerical_contract_fields_equal": numerical_equal,
        "orvd_revisions": {
            "g60": g60_execution["orvd_revision"],
            "g61": g61_execution["orvd_revision"],
        },
        "same_release_executable_sha256": g61_execution["executable_sha256"],
    }


def common_station_grid(
    candidate_station: np.ndarray, reference_station: np.ndarray
) -> np.ndarray:
    begin = max(float(candidate_station[0]), float(reference_station[0]))
    end = min(float(candidate_station[-1]), float(reference_station[-1]))
    first_tick = math.ceil(begin * 100.0 - 1.0e-12)
    last_tick = math.floor(end * 100.0 + 1.0e-12)
    require(first_tick <= last_tick,
            "a wheelset has no common 0.01 m station support")
    grid = np.arange(first_tick, last_tick + 1, dtype=np.float64) / 100.0
    require(grid[-1] >= MINIMUM_COMMON_END_METERS,
            f"a wheelset common support ends at {grid[-1]:.2f} m, below 324 m")
    return grid


def segment_mask(station: np.ndarray, begin: float | None,
                 end: float | None) -> np.ndarray:
    mask = np.ones(station.shape, dtype=bool)
    if begin is not None:
        mask &= station >= begin
    if end is not None:
        mask &= station <= end
    return mask


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
            candidate, reference, abscissa, candidate_station, reference_station,
            station_domain=station_domain,
        ),
        "own_t0_increment": error_statistics(
            candidate - candidate_t0, reference - reference_t0, abscissa,
            candidate_station, reference_station, station_domain=station_domain,
        ),
    }


def macro_segment_statistics(
    grid: np.ndarray,
    candidate_station: np.ndarray,
    reference_station: np.ndarray,
    candidate: np.ndarray,
    reference: np.ndarray,
    *,
    candidate_t0: float,
    reference_t0: float,
) -> tuple[dict[str, Any], bool]:
    candidate_grid = np.interp(grid, candidate_station, candidate)
    reference_grid = np.interp(grid, reference_station, reference)
    result = statistic_pair(
        candidate_grid, reference_grid, grid, grid, grid,
        candidate_t0=candidate_t0, reference_t0=reference_t0,
        station_domain=True,
    )
    alarm = all(
        result[form][metric] <= MACRO_ALARM_LIMIT
        for form in ("raw", "own_t0_increment")
        for metric in ("rms", "maximum_absolute_error")
    )
    result["rms_and_max_raw_and_increment_within_0p1_mm_or_mrad"] = alarm
    return result, alarm


def build_statistics(
    orvd: dict[str, Any], pair: dict[str, np.ndarray], include_historical: bool
) -> dict[str, Any]:
    columns = orvd["columns"]
    indices = np.arange(SAMPLE_COUNT, dtype=np.int64)
    orvd_station = column_stack(columns, "track_station_meters")
    orvd_macro = {
        "lateral_meters": column_stack(columns, "lateral_meters"),
        "yaw_radians": column_stack(columns, "yaw_radians"),
    }
    simpack_macro = {
        "lateral_meters": common_w_macro_in_orvd_track_frame(
            pair, "simpack", "lateral_m"
        ),
        "yaw_radians": common_w_macro_in_orvd_track_frame(
            pair, "simpack", "yaw_rad"
        ),
    }
    historical_macro = {
        "lateral_meters": common_w_macro_in_orvd_track_frame(
            pair, "drake", "lateral_m"
        ),
        "yaw_radians": common_w_macro_in_orvd_track_frame(
            pair, "drake", "yaw_rad"
        ),
    }
    simpack_station = pair["simpack_station_m"]
    historical_station = pair["drake_station_m"]

    grids: list[np.ndarray] = []
    macro: dict[str, Any] = {}
    all_macro_within_alarm = True
    for axle_index, (axle, _, _) in enumerate(AXLES):
        grid = common_station_grid(
            orvd_station[:, axle_index], simpack_station[:, axle_index]
        )
        grids.append(grid)
        axle_result: dict[str, Any] = {
            "common_station_support_meters": [float(grid[0]), float(grid[-1])],
            "common_station_step_meters": STATION_STEP_METERS,
            "segments": {},
        }
        for segment_name, begin, end in SEGMENTS:
            grid_mask = segment_mask(grid, begin, end)
            segment_grid = grid[grid_mask]
            require(segment_grid.size > 0,
                    f"{axle} has no common stations in {segment_name}")
            time_mask = (
                segment_mask(orvd_station[:, axle_index], begin, end) &
                segment_mask(simpack_station[:, axle_index], begin, end)
            )
            selected = indices[time_mask]
            require(selected.size > 0,
                    f"{axle} has no same-time samples in {segment_name}")
            segment_result: dict[str, Any] = {
                "station_range_meters": [
                    float(segment_grid[0]), float(segment_grid[-1])
                ],
                "station_sample_count": int(segment_grid.size),
                "same_station_orvd_minus_simpack": {},
                "same_time_orvd_minus_simpack_diagnostic": {},
            }
            for quantity in ("lateral_meters", "yaw_radians"):
                primary, alarm = macro_segment_statistics(
                    segment_grid,
                    orvd_station[:, axle_index],
                    simpack_station[:, axle_index],
                    orvd_macro[quantity][:, axle_index],
                    simpack_macro[quantity][:, axle_index],
                    candidate_t0=orvd_macro[quantity][0, axle_index],
                    reference_t0=simpack_macro[quantity][0, axle_index],
                )
                segment_result["same_station_orvd_minus_simpack"][quantity] = primary
                all_macro_within_alarm &= alarm
                segment_result["same_time_orvd_minus_simpack_diagnostic"][quantity] = (
                    statistic_pair(
                        orvd_macro[quantity][time_mask, axle_index],
                        simpack_macro[quantity][time_mask, axle_index],
                        selected,
                        orvd_station[time_mask, axle_index],
                        simpack_station[time_mask, axle_index],
                        candidate_t0=orvd_macro[quantity][0, axle_index],
                        reference_t0=simpack_macro[quantity][0, axle_index],
                        station_domain=False,
                    )
                )
                if include_historical:
                    historical_begin = max(float(segment_grid[0]),
                                           float(historical_station[0, axle_index]))
                    historical_end = min(float(segment_grid[-1]),
                                         float(historical_station[-1, axle_index]))
                    first_tick = math.ceil(historical_begin * 100.0 - 1.0e-12)
                    last_tick = math.floor(historical_end * 100.0 + 1.0e-12)
                    require(first_tick <= last_tick,
                            f"{axle} historical WRL has no support in {segment_name}")
                    historical_grid = (
                        np.arange(first_tick, last_tick + 1, dtype=np.float64) / 100.0
                    )
                    candidate_grid = np.interp(
                        historical_grid, orvd_station[:, axle_index],
                        orvd_macro[quantity][:, axle_index]
                    )
                    reference_grid = np.interp(
                        historical_grid, historical_station[:, axle_index],
                        historical_macro[quantity][:, axle_index]
                    )
                    segment_result.setdefault(
                        "same_station_orvd_minus_historical_wrl", {}
                    )[quantity] = statistic_pair(
                        candidate_grid, reference_grid, historical_grid,
                        historical_grid, historical_grid,
                        candidate_t0=orvd_macro[quantity][0, axle_index],
                        reference_t0=historical_macro[quantity][0, axle_index],
                        station_domain=True,
                    )
            axle_result["segments"][segment_name] = segment_result
        macro[axle] = axle_result

    orvd_contacts = {
        quantity: orvd_contact(columns, quantity) for quantity in CONTACT_QUANTITIES
    }
    contact: dict[str, Any] = {}
    for wheel_index, wheel_value in enumerate(pair["wheel_names"].tolist()):
        wheel = str(wheel_value)
        axle_index = wheel_index // 2
        grid = grids[axle_index]
        wheel_result: dict[str, Any] = {"segments": {}}
        for segment_name, begin, end in SEGMENTS:
            segment_grid = grid[segment_mask(grid, begin, end)]
            time_mask = (
                segment_mask(orvd_station[:, axle_index], begin, end) &
                segment_mask(simpack_station[:, axle_index], begin, end)
            )
            selected = indices[time_mask]
            segment_result: dict[str, Any] = {
                "same_station_orvd_minus_simpack": {},
                "same_time_orvd_minus_simpack_diagnostic": {},
            }
            for quantity in CONTACT_QUANTITIES:
                candidate_all = orvd_contacts[quantity][:, wheel_index]
                reference_all = pair[f"simpack_{quantity}_N"][:, wheel_index]
                candidate_grid = np.interp(
                    segment_grid, orvd_station[:, axle_index], candidate_all
                )
                reference_grid = np.interp(
                    segment_grid, simpack_station[:, axle_index], reference_all
                )
                segment_result["same_station_orvd_minus_simpack"][quantity] = (
                    statistic_pair(
                        candidate_grid, reference_grid, segment_grid,
                        segment_grid, segment_grid,
                        candidate_t0=candidate_all[0], reference_t0=reference_all[0],
                        station_domain=True,
                    )
                )
                segment_result["same_time_orvd_minus_simpack_diagnostic"][quantity] = (
                    statistic_pair(
                        candidate_all[time_mask], reference_all[time_mask], selected,
                        orvd_station[time_mask, axle_index],
                        simpack_station[time_mask, axle_index],
                        candidate_t0=candidate_all[0], reference_t0=reference_all[0],
                        station_domain=False,
                    )
                )
                if include_historical:
                    historical_begin = max(
                        float(segment_grid[0]), float(historical_station[0, axle_index])
                    )
                    historical_end = min(
                        float(segment_grid[-1]), float(historical_station[-1, axle_index])
                    )
                    first_tick = math.ceil(historical_begin * 100.0 - 1.0e-12)
                    last_tick = math.floor(historical_end * 100.0 + 1.0e-12)
                    historical_grid = (
                        np.arange(first_tick, last_tick + 1, dtype=np.float64) / 100.0
                    )
                    candidate_historical_grid = np.interp(
                        historical_grid, orvd_station[:, axle_index], candidate_all
                    )
                    historical_values = pair[f"drake_{quantity}_N"][:, wheel_index]
                    reference_historical_grid = np.interp(
                        historical_grid, historical_station[:, axle_index],
                        historical_values,
                    )
                    segment_result.setdefault(
                        "same_station_orvd_minus_historical_wrl", {}
                    )[quantity] = statistic_pair(
                        candidate_historical_grid, reference_historical_grid,
                        historical_grid, historical_grid, historical_grid,
                        candidate_t0=candidate_all[0],
                        reference_t0=historical_values[0], station_domain=True,
                    )
            wheel_result["segments"][segment_name] = segment_result
        wheel_result["orvd_contact_topology"] = longest_zero_run(
            orvd["patch_count"][:, wheel_index]
        )
        wheel_result["simpack_contact_topology"] = longest_zero_run(
            pair["simpack_patch_count"][:, wheel_index]
        )
        wheel_result["historical_wrl_contact_topology"] = longest_zero_run(
            pair["drake_patch_count"][:, wheel_index]
        )
        contact[wheel] = wheel_result

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
        if all(np.all(orvd_contacts[quantity][:, index] == 0.0)
               for quantity in ("Q", "Tx", "Ty")):
            sustained_failures.append(f"{wheel}: all three force components are zero")

    return {
        "primary_alignment": {
            "kind": "per-wheelset common 0.01 m station grid",
            "macro_coordinate_frame": MACRO_COORDINATE_FRAME,
            "p057_common_W_to_orvd_track_T": COMMON_W_TO_ORVD_TRACK_T,
            "segments": [segment[0] for segment in SEGMENTS],
            "no_extrapolation": True,
            "no_time_or_station_shift": True,
            "no_filter_demean_sign_fit_or_result_scaling": True,
        },
        "same_time_role": "propagation and phase diagnostic, not the G61 primary gate",
        "macro_response": macro,
        "contact_response": contact,
        "macro_raw_and_increment_rms_and_max_all_within_0p1_mm_or_mrad":
            all_macro_within_alarm,
        "simultaneous_all_eight_zero_contact_sample_count": int(np.count_nonzero(
            np.all(orvd["patch_count"] == 0, axis=1)
        )),
        "sustained_contact_or_force_failures": sustained_failures,
        "contact_force_threshold": (
            "none; P057 qualified macro response, while Q/N/Tx/Ty remain "
            "diagnostic observations"
        ),
    }


def add_station_bands(axis: Any) -> None:
    for begin, end, colour in (
        (50.0, 100.0, "#F0E442"),
        (100.0, 250.0, "#009E73"),
        (250.0, 300.0, "#E69F00"),
        (300.0, 324.0, "#56B4E9"),
    ):
        axis.axvspan(begin, end, color=colour, alpha=0.08, linewidth=0)


def plot_main_response(
    path: Path,
    orvd: dict[str, Any],
    pair: dict[str, np.ndarray],
    historical_revision: str | None,
) -> None:
    columns = orvd["columns"]
    time = columns["time_seconds"]
    orvd_station = column_stack(columns, "track_station_meters")
    quantities = (
        ("lateral_meters", column_stack(columns, "lateral_meters"),
         common_w_macro_in_orvd_track_frame(pair, "simpack", "lateral_m"),
         common_w_macro_in_orvd_track_frame(pair, "drake", "lateral_m"),
         1.0e3, "Lateral displacement [mm]"),
        ("yaw_radians", column_stack(columns, "yaw_radians"),
         common_w_macro_in_orvd_track_frame(pair, "simpack", "yaw_rad"),
         common_w_macro_in_orvd_track_frame(pair, "drake", "yaw_rad"),
         1.0e3, "Yaw angle [mrad]"),
    )
    fig, axes = plt.subplots(4, 4, figsize=(20, 13.5))
    historical_label = (
        None if historical_revision is None
        else f"Historical WRL {historical_revision[:8]}"
    )
    colours = {"ORVD": "#0072B2", "SIMPACK": "#D55E00"}
    if historical_label is not None:
        colours[historical_label] = "#009E73"
    handles: list[Any] = []
    labels: list[str] = []
    for axle_index, (axle, _, _) in enumerate(AXLES):
        axes[0, axle_index].set_title(axle.upper())
        grid = common_station_grid(
            orvd_station[:, axle_index], pair["simpack_station_m"][:, axle_index]
        )
        for quantity_index, quantity_data in enumerate(quantities):
            _, candidate, reference, historical, scale, ylabel = quantity_data
            time_axis = axes[quantity_index, axle_index]
            station_axis = axes[quantity_index + 2, axle_index]
            time_series = [
                ("ORVD", candidate[:, axle_index]),
                ("SIMPACK", reference[:, axle_index]),
            ]
            if historical_label is not None:
                time_series.append((historical_label, historical[:, axle_index]))
            for series_label, values in time_series:
                line, = time_axis.plot(
                    time, values * scale, color=colours[series_label],
                    linewidth=0.8, label=series_label,
                )
                if axle_index == 0 and quantity_index == 0:
                    handles.append(line)
                    labels.append(series_label)
            station_series = [
                ("ORVD", np.interp(
                    grid, orvd_station[:, axle_index], candidate[:, axle_index]
                )),
                ("SIMPACK", np.interp(
                    grid, pair["simpack_station_m"][:, axle_index],
                    reference[:, axle_index]
                )),
            ]
            if historical_label is not None:
                historical_begin = max(grid[0], pair["drake_station_m"][0, axle_index])
                historical_end = min(grid[-1], pair["drake_station_m"][-1, axle_index])
                historical_grid = np.arange(
                    math.ceil(historical_begin * 100.0 - 1.0e-12),
                    math.floor(historical_end * 100.0 + 1.0e-12) + 1,
                    dtype=np.float64,
                ) / 100.0
            for series_label, values in station_series:
                station_axis.plot(grid, values * scale,
                                  color=colours[series_label], linewidth=0.8)
            if historical_label is not None:
                station_axis.plot(
                    historical_grid,
                    np.interp(historical_grid,
                              pair["drake_station_m"][:, axle_index],
                              historical[:, axle_index]) * scale,
                    color=colours[historical_label], linewidth=0.8,
                )
            add_station_bands(station_axis)
            if axle_index == 0:
                time_axis.set_ylabel(ylabel)
                station_axis.set_ylabel(ylabel)
            time_axis.grid(True, alpha=0.22)
            station_axis.grid(True, alpha=0.22)
            if quantity_index == 1:
                time_axis.set_xlabel("Time [s]")
                station_axis.set_xlabel("Track station [m]")
    fig.suptitle(
        "G61 GZ18 AAR6 — full-envelope wheelset response "
        "(no shifting or fitting)", y=0.985,
    )
    fig.legend(handles, labels, loc="upper center", ncol=len(labels),
               bbox_to_anchor=(0.5, 0.958), frameon=False)
    fig.subplots_adjust(left=0.06, right=0.99, bottom=0.055, top=0.91,
                        hspace=0.34, wspace=0.24)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_contact_quantity(
    path: Path,
    quantity: str,
    orvd: dict[str, Any],
    pair: dict[str, np.ndarray],
    historical_revision: str | None,
) -> None:
    columns = orvd["columns"]
    time = columns["time_seconds"]
    orvd_station = column_stack(columns, "track_station_meters")
    candidate = orvd_contact(columns, quantity) / 1000.0
    reference = pair[f"simpack_{quantity}_N"] / 1000.0
    historical = pair[f"drake_{quantity}_N"] / 1000.0
    fig, axes = plt.subplots(4, 4, figsize=(20, 13.5))
    historical_label = (
        None if historical_revision is None
        else f"Historical WRL {historical_revision[:8]}"
    )
    colours = {"ORVD": "#0072B2", "SIMPACK": "#D55E00"}
    if historical_label is not None:
        colours[historical_label] = "#009E73"
    handles: list[Any] = []
    labels: list[str] = []
    for axle_index, (axle, _, _) in enumerate(AXLES):
        axes[0, axle_index].set_title(axle.upper())
        for side_index, side in enumerate(SIDES):
            wheel_index = 2 * axle_index + side_index
            for domain_index, domain in enumerate(("time", "station")):
                row = side_index * 2 + domain_index
                axis = axes[row, axle_index]
                plotted = [
                    ("ORVD", time if domain == "time" else
                     orvd_station[:, axle_index], candidate[:, wheel_index]),
                    ("SIMPACK", time if domain == "time" else
                     pair["simpack_station_m"][:, axle_index],
                     reference[:, wheel_index]),
                ]
                if historical_label is not None:
                    plotted.append((
                        historical_label,
                        time if domain == "time" else
                        pair["drake_station_m"][:, axle_index],
                        historical[:, wheel_index],
                    ))
                for series_label, x_values, y_values in plotted:
                    line, = axis.plot(
                        x_values, y_values, color=colours[series_label],
                        linewidth=0.72, label=series_label,
                    )
                    if axle_index == 0 and row == 0:
                        handles.append(line)
                        labels.append(series_label)
                if domain == "station":
                    add_station_bands(axis)
                    axis.set_xlim(left=min(-10.0, float(np.min(orvd_station[:, axle_index]))),
                                  right=max(324.0, float(np.max(orvd_station[:, axle_index]))))
                    axis.set_xlabel("Track station [m]")
                else:
                    axis.set_xlabel("Time [s]")
                if axle_index == 0:
                    axis.set_ylabel(f"{side.capitalize()} {quantity} [kN]")
                axis.grid(True, alpha=0.22)
    fig.suptitle(
        f"G61 GZ18 AAR6 — wheel-side {quantity} (diagnostic; Q is not N)",
        y=0.985,
    )
    fig.legend(handles, labels, loc="upper center", ncol=len(labels),
               bbox_to_anchor=(0.5, 0.958), frameon=False)
    fig.subplots_adjust(left=0.06, right=0.99, bottom=0.055, top=0.91,
                        hspace=0.34, wspace=0.24)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def publish_analysis(
    output: Path,
    summary: dict[str, Any],
    orvd: dict[str, Any],
    pair: dict[str, np.ndarray],
    historical_revision: str | None,
) -> None:
    require(not output.exists(), f"analysis output already exists: {output}")
    partial = output.with_name(output.name + ".partial")
    require(not partial.exists(), f"analysis partial directory already exists: {partial}")
    output.parent.mkdir(parents=True, exist_ok=True)
    partial.mkdir()
    try:
        write_json(partial / "comparison.json", summary)
        plot_main_response(
            partial / "gz18_g61_wheelset_main_response.png",
            orvd, pair, historical_revision,
        )
        for quantity in CONTACT_QUANTITIES:
            plot_contact_quantity(
                partial / f"gz18_g61_contact_{quantity}.png",
                quantity, orvd, pair, historical_revision,
            )
        (partial / "COMPLETE").write_text(
            "G61 comparison completed\n", encoding="utf-8"
        )
        partial.rename(output)
    except Exception:
        shutil.rmtree(partial, ignore_errors=True)
        raise


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orvd-artifact-directory", type=Path, required=True)
    parser.add_argument("--p057-paired-reference-npz", type=Path, required=True)
    parser.add_argument("--p057-simpack-native-core-npz", type=Path, required=True)
    parser.add_argument("--execution-identity-json", type=Path, required=True)
    parser.add_argument("--g60-artifact-directory", type=Path, required=True)
    parser.add_argument("--g60-execution-identity-json", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--include-historical-wrl", action="store_true")
    parser.add_argument("--historical-wrl-source-revision")
    arguments = parser.parse_args(list(argv))
    if arguments.include_historical_wrl and not arguments.historical_wrl_source_revision:
        parser.error("--include-historical-wrl requires --historical-wrl-source-revision")
    if (arguments.include_historical_wrl and
            arguments.historical_wrl_source_revision !=
            P057_HISTORICAL_WRL_SOURCE_REVISION):
        parser.error(
            "P057 historical WRL arrays require source revision "
            f"{P057_HISTORICAL_WRL_SOURCE_REVISION}"
        )
    return arguments


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    try:
        artifact_directory = arguments.orvd_artifact_directory.resolve()
        orvd = load_orvd_artifact(
            artifact_directory, goal_name="G61", sample_count=SAMPLE_COUNT,
            terminal_time_nanoseconds=TERMINAL_TIME_NANOSECONDS,
        )
        pair_path = arguments.p057_paired_reference_npz.resolve()
        native_path = arguments.p057_simpack_native_core_npz.resolve()
        pair = load_p057_pair(pair_path)
        validate_p057_native(native_path, pair)
        execution = load_execution_identity(
            arguments.execution_identity_json.resolve(), artifact_directory,
            goal_name="G61",
        )
        g60_artifact = arguments.g60_artifact_directory.resolve()
        g60_metadata, g60_execution = load_g60_identity(
            g60_artifact, arguments.g60_execution_identity_json.resolve()
        )
        execution_match = verify_g60_g61_execution_match(
            g60_metadata, g60_execution, orvd["metadata"], execution
        )
        statistics = build_statistics(
            orvd, pair, arguments.include_historical_wrl
        )
        require(not statistics["sustained_contact_or_force_failures"],
                "sustained contact/force failure: " + "; ".join(
                    statistics["sustained_contact_or_force_failures"]
                ))
        performance = orvd["performance"]
        advance_wall = float(performance["advance_wall_seconds"])
        process_wall = float(execution["process_wall_seconds"])
        summary = {
            "schema_version": 1,
            "scope": "G61 GZ18 straight AAR6 20 s full-envelope qualification",
            "scientific_interpretation_requires_curve_review": True,
            "sources": {
                "orvd_artifact_directory": str(artifact_directory),
                "orvd_observations_sha256": orvd["observation_sha256"],
                "p057_paired_reference": str(pair_path),
                "p057_paired_reference_sha256": sha256_file(pair_path),
                "p057_simpack_native_core": str(native_path),
                "p057_simpack_native_core_sha256": sha256_file(native_path),
                "historical_wrl_source_revision": (
                    arguments.historical_wrl_source_revision
                    if arguments.include_historical_wrl else None
                ),
                "reference_endpoint_semantics": (
                    "P057 Tx/Ty are already converted from the SIMPACK Type-80 "
                    "rail end to the canonical wheel end; no second sign conversion"
                ),
                "macro_coordinate_frame": MACRO_COORDINATE_FRAME,
                "p057_common_W_to_orvd_track_T": COMMON_W_TO_ORVD_TRACK_T,
            },
            "g60_g61_execution_match": execution_match,
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
        publish_analysis(
            arguments.output_directory.resolve(), summary, orvd, pair,
            (arguments.historical_wrl_source_revision
             if arguments.include_historical_wrl else None),
        )
    except (AnalysisError, OSError) as error:
        print(f"G61 analysis failed: {error}", file=sys.stderr)
        return 1
    print(f"published G61 comparison to {arguments.output_directory.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
