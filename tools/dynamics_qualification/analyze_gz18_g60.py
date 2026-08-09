#!/usr/bin/env python3
"""Compare one ORVD G60 artifact with the frozen P038/P039 references.

This is a migration-only, non-installed analysis tool.  It never searches for
an external reference root: every input and the output directory are explicit.
Long arrays and figures remain outside the repository.
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
    expected_times as common_expected_times,
    load_execution_identity as common_load_execution_identity,
    load_orvd_artifact as common_load_orvd_artifact,
    longest_zero_run,
    orvd_contact,
    require,
    require_finite,
    sha256_file,
    write_json,
)


SAMPLE_COUNT = 20_001
TERMINAL_TIME_SECONDS = 10.0
PRIMARY_STATION_BEGIN_METERS = 100.0
PRIMARY_STATION_END_METERS = 150.0
P039_COMMON_W_TO_ORVD_TRACK_T = COMMON_W_TO_ORVD_TRACK_T

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


def expected_times() -> np.ndarray:
    return common_expected_times(
        SAMPLE_COUNT, TERMINAL_TIME_SECONDS
    )


def load_orvd_artifact(directory: Path) -> dict[str, Any]:
    return common_load_orvd_artifact(
        directory, goal_name="G60", sample_count=SAMPLE_COUNT,
        terminal_time_nanoseconds=10_000_000_000,
    )


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
    return common_load_execution_identity(
        path, artifact_directory, goal_name="G60"
    )


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
    return common_w_macro_in_orvd_track_frame(pair, source, quantity)


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
