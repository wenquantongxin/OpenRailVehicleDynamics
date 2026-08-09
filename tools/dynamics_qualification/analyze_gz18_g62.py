#!/usr/bin/env python3
"""Compare G62 station propagation without mixing station observers.

The primary comparison pairs the ORVD average of the left/right rail-profile
reference stations with SIMPACK P047 Result Element 82 channel 3.  Separate
control comparisons pair ORVD wheelset-body projections with current WRL and
historical P055 wheelset-body projections.  No series is interpolated, shifted,
filtered, fitted, or rescaled.
"""

from __future__ import annotations

import argparse
import csv
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
    expected_times,
    load_execution_identity,
    load_orvd_artifact,
    require,
    require_finite,
    sha256_file,
    write_json,
)


SAMPLE_COUNT = 32_001
TERMINAL_TIME_SECONDS = 16.0
TERMINAL_TIME_NANOSECONDS = 16_000_000_000
SAMPLE_PERIOD_NANOSECONDS = 500_000
STATION_GRID_STEP_METERS = 0.01
STATION_SEGMENTS = (
    ("initial_straight", None, 50.0),
    ("transition_without_irregularity", 50.0, 60.0),
    ("transition_and_aar5_fade_in", 60.0, 100.0),
    ("circular_curve_and_aar5_full_amplitude", 100.0, None),
)
P047_KEYS = {
    "time_seconds", "axle_names", "station_meters", "channel_paths",
    "channel_descriptions", "source_sbr_sha256", "source_sbr_precision_bytes",
}
P047_PATHS = (
    "RS_result.RS_S_BF_F.RS_S_WS_F."
    "S_BF_F__S_WS_F__RS_RWT_track.ch_003",
    "RS_result.RS_S_BF_F.RS_S_WS_R."
    "S_BF_F__S_WS_R__RS_RWT_track.ch_003",
    "RS_result.RS_S_BF_R.RS_S_WS_F."
    "S_BF_R__S_WS_F__RS_RWT_track.ch_003",
    "RS_result.RS_S_BF_R.RS_S_WS_R."
    "S_BF_R__S_WS_R__RS_RWT_track.ch_003",
)
P047_SOURCE_SBR_SHA256 = (
    "f8673e57c5956467c81c3ba95a677f4efc6422e69b1aa7fc07848726b8bb4b68"
)
CURRENT_WRL_SOURCE_REVISION = "d7e272df8c29a2074d70eee7cd5a24cfede78d83"
P055_HISTORICAL_WRL_CSV_SHA256 = (
    "cf5c6b90a2b2025471d10816c384f5c04bcdf95bfc542e82d5f8319c724c6843"
)


def load_g62_orvd(directory: Path) -> dict[str, Any]:
    result = load_orvd_artifact(
        directory,
        goal_name="G62",
        sample_count=SAMPLE_COUNT,
        terminal_time_nanoseconds=TERMINAL_TIME_NANOSECONDS,
        track_irregularity_identifier="gz18_r300_aar5_reference_irregularity",
        track_geometry_filename="r300_centerline_superelevation_1150m.json",
        base_track_definition_interval_meters=(0, 1150),
    )
    columns = result["columns"]
    missing = []
    for _, body, _ in AXLES:
        for side in ("left", "right"):
            name = (
                f"{body}.{side}."
                "rail_profile_reference_marker_track_station_meters"
            )
            if name not in columns:
                missing.append(name)
    require(not missing, f"G62 ORVD observations are missing columns: {missing}")
    return result


def load_p047(path: Path) -> dict[str, Any]:
    require(path.is_file(), f"P047 station reference does not exist: {path}")
    try:
        with np.load(path, allow_pickle=False) as archive:
            require(set(archive.files) == P047_KEYS,
                    "P047 station reference has an unexpected key set")
            result = {key: np.array(archive[key], copy=True) for key in archive.files}
    except (OSError, ValueError) as error:
        raise AnalysisError(f"could not load P047 station reference: {error}") from error

    expected_axles = np.asarray([axle for axle, _, _ in AXLES])
    require(np.array_equal(result["axle_names"], expected_axles),
            "P047 axle names or order differ from ff/fr/rf/rr")
    require(np.array_equal(result["channel_paths"], np.asarray(P047_PATHS)),
            "P047 Result Element 82 channel paths differ from the qualified mapping")
    require(np.array_equal(
        result["channel_descriptions"],
        np.asarray(["s Position along track"] * 4),
    ), "P047 Result Element 82 channel descriptions differ from channel 3")
    require(result["source_sbr_precision_bytes"].shape == () and
            int(result["source_sbr_precision_bytes"]) == 8,
            "P047 station reference is not identified as binary64")
    digest = result["source_sbr_sha256"]
    require(digest.shape == () and digest.item() == P047_SOURCE_SBR_SHA256,
            "P047 station reference names a different source SBR")
    expected_time = expected_times(SAMPLE_COUNT, TERMINAL_TIME_SECONDS)
    require(result["time_seconds"].shape == (SAMPLE_COUNT,) and
            np.array_equal(result["time_seconds"], expected_time),
            "P047 time is not the exact 0--16 s / 0.5 ms clock")
    station = result["station_meters"]
    require(station.shape == (SAMPLE_COUNT, 4),
            "P047 Result Element 82 stations have the wrong shape")
    require_finite(station, "P047 Result Element 82 stations")
    require(bool(np.all(np.diff(station, axis=0) > 0.0)),
            "P047 Result Element 82 stations are not strictly increasing")
    return result


def load_wrl_body_csv(path: Path, name: str) -> dict[str, np.ndarray]:
    require(path.is_file(), f"{name} CSV does not exist: {path}")
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            header = next(csv.reader(stream))
        values = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.float64)
    except (OSError, StopIteration, ValueError) as error:
        raise AnalysisError(f"could not read {name} CSV: {error}") from error
    if values.ndim == 1:
        values = values.reshape(1, -1)
    require(len(header) == len(set(header)), f"{name} CSV columns are not unique")
    require(values.ndim == 2 and values.shape[1] == len(header),
            f"{name} CSV width differs from its header")
    require_finite(values, f"{name} CSV")
    columns = {column: values[:, index] for index, column in enumerate(header)}
    required = {"t", *(f"s_wheelset_{axle}_m" for axle, _, _ in AXLES)}
    missing = sorted(required - set(columns))
    require(not missing, f"{name} CSV is missing columns: {missing}")

    time = columns["t"]
    require(time.size >= 2 and time[0] == 0.0 and
            bool(np.all(np.diff(time) > 0.0)),
            f"{name} time is not a strictly increasing native sequence from zero")
    ticks = np.rint(time * 1.0e9).astype(np.int64)
    # The WRL CSV writer advances the logged clock by repeated binary64
    # addition, so the printed 0.5 ms identities accumulate about 8 ps over
    # 16 s.  Rounding back to the declared integer-nanosecond sample identity
    # is not resampling; no state value is evaluated between native rows.
    require(float(np.max(np.abs(
        time - ticks.astype(np.float64) * 1.0e-9
    ))) <= 1.0e-10,
            f"{name} times do not resolve to integer-nanosecond native samples")
    require(np.unique(ticks).size == ticks.size,
            f"{name} has duplicate native time samples")
    require(ticks[-1] >= TERMINAL_TIME_NANOSECONDS - SAMPLE_PERIOD_NANOSECONDS and
            ticks[-1] <= TERMINAL_TIME_NANOSECONDS,
            f"{name} does not cover the native 16 s window")
    station = np.column_stack([
        columns[f"s_wheelset_{axle}_m"] for axle, _, _ in AXLES
    ])
    require(bool(np.all(np.diff(station, axis=0) > 0.0)),
            f"{name} wheelset-body stations are not strictly increasing")
    return {"time_seconds": time, "time_nanoseconds": ticks,
            "station_meters": station}


def orvd_result82_station(columns: dict[str, np.ndarray]) -> np.ndarray:
    result = []
    for _, body, _ in AXLES:
        left = columns[
            f"{body}.left.rail_profile_reference_marker_track_station_meters"
        ]
        right = columns[
            f"{body}.right.rail_profile_reference_marker_track_station_meters"
        ]
        result.append(0.5 * (left + right))
    station = np.column_stack(result)
    require(bool(np.all(np.diff(station, axis=0) > 0.0)),
            "ORVD Result Element 82-equivalent stations are not strictly increasing")
    return station


def orvd_body_station(columns: dict[str, np.ndarray]) -> np.ndarray:
    return np.column_stack([
        columns[f"{body}.track_station_meters"] for _, body, _ in AXLES
    ])


def compact_statistics(candidate: np.ndarray, reference: np.ndarray,
                       abscissa: np.ndarray) -> dict[str, Any]:
    require(candidate.shape == reference.shape == abscissa.shape and
            candidate.size > 0,
            "station statistic inputs have different or empty shapes")
    error = candidate - reference
    absolute = np.abs(error)
    peak = int(np.argmax(absolute))
    return {
        "sample_count": int(error.size),
        "signed_mean": float(np.mean(error)),
        "rms": float(np.sqrt(np.mean(np.square(error)))),
        "maximum_absolute_error": float(absolute[peak]),
        "signed_error_at_maximum": float(error[peak]),
        "abscissa_at_maximum": float(abscissa[peak]),
    }


def raw_and_increment_statistics(candidate: np.ndarray, reference: np.ndarray,
                                 abscissa: np.ndarray) -> dict[str, Any]:
    return {
        "raw": compact_statistics(candidate, reference, abscissa),
        "own_initial_value_increment": compact_statistics(
            candidate - candidate[0], reference - reference[0], abscissa
        ),
    }


def common_native_samples(
    candidate_time_ns: np.ndarray,
    candidate: np.ndarray,
    reference_time_ns: np.ndarray,
    reference: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    common, candidate_index, reference_index = np.intersect1d(
        candidate_time_ns, reference_time_ns, assume_unique=True,
        return_indices=True,
    )
    require(common.size > 0, "two native series have no common time samples")
    return common, candidate[candidate_index], reference[reference_index]


def common_station_grid(candidate: np.ndarray,
                        reference: np.ndarray) -> np.ndarray:
    begin_tick = math.ceil(max(float(candidate[0]), float(reference[0])) * 100.0)
    end_tick = math.floor(min(float(candidate[-1]), float(reference[-1])) * 100.0)
    require(begin_tick <= end_tick, "two station series have no 0.01 m overlap")
    return np.arange(begin_tick, end_tick + 1, dtype=np.float64) / 100.0


def shared_station_grids(series: list[np.ndarray]) -> list[np.ndarray]:
    require(len(series) >= 2, "a shared station grid needs at least two series")
    grids = []
    for axle_index in range(4):
        begin_tick = math.ceil(
            max(float(values[0, axle_index]) for values in series) * 100.0
        )
        end_tick = math.floor(
            min(float(values[-1, axle_index]) for values in series) * 100.0
        )
        require(begin_tick <= end_tick,
                "station series have no shared 0.01 m support")
        grids.append(
            np.arange(begin_tick, end_tick + 1, dtype=np.float64) / 100.0
        )
    return grids


def first_native_arrival(time: np.ndarray, station: np.ndarray,
                         grid: np.ndarray) -> np.ndarray:
    indices = np.searchsorted(station, grid, side="left")
    require(bool(np.all(indices < station.size)),
            "station grid extends beyond a native station series")
    return time[indices]


def station_segment_mask(values: np.ndarray, begin: float | None,
                         end: float | None) -> np.ndarray:
    mask = np.ones(values.shape, dtype=bool)
    if begin is not None:
        mask &= values >= begin
    if end is not None:
        mask &= values < end
    return mask


def compare_pair(
    candidate_time: np.ndarray,
    candidate_station: np.ndarray,
    reference_time: np.ndarray,
    reference_station: np.ndarray,
) -> tuple[dict[str, Any], list[np.ndarray]]:
    candidate_ns = np.rint(candidate_time * 1.0e9).astype(np.int64)
    reference_ns = np.rint(reference_time * 1.0e9).astype(np.int64)
    result: dict[str, Any] = {}
    grids: list[np.ndarray] = []
    for axle_index, (axle, _, _) in enumerate(AXLES):
        common_ns, candidate_native, reference_native = common_native_samples(
            candidate_ns, candidate_station[:, axle_index],
            reference_ns, reference_station[:, axle_index],
        )
        native_time = common_ns.astype(np.float64) * 1.0e-9
        grid = common_station_grid(
            candidate_station[:, axle_index], reference_station[:, axle_index]
        )
        grids.append(grid)
        candidate_arrival = first_native_arrival(
            candidate_time, candidate_station[:, axle_index], grid
        )
        reference_arrival = first_native_arrival(
            reference_time, reference_station[:, axle_index], grid
        )
        axle_result = {
            "same_native_time_station_meters": raw_and_increment_statistics(
                candidate_native, reference_native, native_time
            ),
            "first_native_sample_at_or_beyond_station_seconds":
                raw_and_increment_statistics(
                    candidate_arrival, reference_arrival, grid
                ),
            "common_native_time_interval_seconds": [
                float(native_time[0]), float(native_time[-1])
            ],
            "common_station_grid_meters": [float(grid[0]), float(grid[-1])],
            "common_station_step_meters": STATION_GRID_STEP_METERS,
            "time_interpolation": "none",
            "station_arrival_rule": "first native sample with station >= grid station",
            "segments": {},
        }
        for segment_name, begin, end in STATION_SEGMENTS:
            time_mask = (
                station_segment_mask(candidate_native, begin, end) &
                station_segment_mask(reference_native, begin, end)
            )
            grid_mask = station_segment_mask(grid, begin, end)
            require(bool(np.any(time_mask)),
                    f"{axle} has no common native samples in {segment_name}")
            require(bool(np.any(grid_mask)),
                    f"{axle} has no common station grid in {segment_name}")
            axle_result["segments"][segment_name] = {
                "same_native_time_station_meters": raw_and_increment_statistics(
                    candidate_native[time_mask], reference_native[time_mask],
                    native_time[time_mask],
                ),
                "first_native_sample_at_or_beyond_station_seconds":
                    raw_and_increment_statistics(
                        candidate_arrival[grid_mask],
                        reference_arrival[grid_mask], grid[grid_mask],
                    ),
                "common_native_time_interval_seconds": [
                    float(native_time[time_mask][0]),
                    float(native_time[time_mask][-1]),
                ],
                "common_station_grid_meters": [
                    float(grid[grid_mask][0]), float(grid[grid_mask][-1]),
                ],
            }
        result[axle] = axle_result
    return result, grids


def plot_time_series(path: Path, title: str, time_series: list[tuple[str, np.ndarray,
                                                                      np.ndarray]]) -> None:
    require(2 <= len(time_series) <= 3, "a G62 subplot must have two or three lines")
    figure, axes = plt.subplots(4, 1, figsize=(12.0, 10.0), sharex=True)
    for axle_index, (axis, (axle, _, _)) in enumerate(zip(axes, AXLES)):
        for label, time, station in time_series:
            axis.plot(time, station[:, axle_index], linewidth=0.8, label=label)
        axis.set_ylabel(f"{axle} s [m]")
        axis.grid(True, alpha=0.22)
    axes[-1].set_xlabel("Time [s]")
    axes[0].legend(loc="best", frameon=False)
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_arrival_series(
    path: Path,
    title: str,
    series: list[tuple[str, np.ndarray, np.ndarray]],
    grids: list[np.ndarray],
) -> None:
    require(2 <= len(series) <= 3, "a G62 subplot must have two or three lines")
    figure, axes = plt.subplots(4, 1, figsize=(12.0, 10.0), sharex=False)
    for axle_index, (axis, (axle, _, _)) in enumerate(zip(axes, AXLES)):
        grid = grids[axle_index]
        for label, time, station in series:
            arrival = first_native_arrival(time, station[:, axle_index], grid)
            axis.plot(grid, arrival, linewidth=0.8, label=label)
        axis.set_ylabel(f"{axle} t [s]")
        axis.grid(True, alpha=0.22)
    axes[-1].set_xlabel("Track station [m]")
    axes[0].legend(loc="best", frameon=False)
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def publish(
    output: Path,
    summary: dict[str, Any],
    orvd_time: np.ndarray,
    orvd_result82: np.ndarray,
    p047: dict[str, Any],
    orvd_body: np.ndarray,
    current: dict[str, np.ndarray],
    historical: dict[str, np.ndarray],
    result82_grids: list[np.ndarray],
    body_grids: list[np.ndarray],
) -> None:
    require(not output.exists(), f"analysis output already exists: {output}")
    partial = output.with_name(output.name + ".partial")
    require(not partial.exists(), f"analysis partial directory already exists: {partial}")
    output.parent.mkdir(parents=True, exist_ok=True)
    partial.mkdir()
    try:
        write_json(partial / "comparison.json", summary)
        plot_time_series(
            partial / "g62_result82_station_vs_time.png",
            "G62 Result Element 82-equivalent station — same observer",
            [("ORVD", orvd_time, orvd_result82),
             ("SIMPACK P047", p047["time_seconds"], p047["station_meters"])],
        )
        plot_arrival_series(
            partial / "g62_result82_arrival_time_vs_station.png",
            "G62 first native-sample arrival — Result Element 82-equivalent",
            [("ORVD", orvd_time, orvd_result82),
             ("SIMPACK P047", p047["time_seconds"], p047["station_meters"])],
            result82_grids,
        )
        body_series = [
            ("ORVD", orvd_time, orvd_body),
            ("current WRL", current["time_seconds"], current["station_meters"]),
            ("historical WRL P055", historical["time_seconds"],
             historical["station_meters"]),
        ]
        plot_time_series(
            partial / "g62_wheelset_body_station_vs_time.png",
            "G62 wheelset-body projection station — same observer",
            body_series,
        )
        plot_arrival_series(
            partial / "g62_wheelset_body_arrival_time_vs_station.png",
            "G62 first native-sample arrival — wheelset-body projection",
            body_series, body_grids,
        )
        (partial / "COMPLETE").write_text(
            "G62 station comparison completed\n", encoding="utf-8"
        )
        partial.rename(output)
    except Exception:
        shutil.rmtree(partial, ignore_errors=True)
        raise


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orvd-artifact-directory", type=Path, required=True)
    parser.add_argument("--execution-identity-json", type=Path, required=True)
    parser.add_argument("--p047-station-reference-npz", type=Path, required=True)
    parser.add_argument("--current-wrl-csv", type=Path, required=True)
    parser.add_argument("--current-wrl-source-revision", required=True)
    parser.add_argument("--p055-historical-wrl-csv", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    try:
        artifact = arguments.orvd_artifact_directory.resolve()
        orvd = load_g62_orvd(artifact)
        execution = load_execution_identity(
            arguments.execution_identity_json.resolve(), artifact, goal_name="G62"
        )
        p047_path = arguments.p047_station_reference_npz.resolve()
        p047 = load_p047(p047_path)
        current_path = arguments.current_wrl_csv.resolve()
        historical_path = arguments.p055_historical_wrl_csv.resolve()
        current = load_wrl_body_csv(current_path, "current WRL")
        historical = load_wrl_body_csv(historical_path, "historical P055 WRL")
        require(arguments.current_wrl_source_revision == CURRENT_WRL_SOURCE_REVISION,
                "current WRL source revision differs from the G62 frozen baseline")
        require(sha256_file(historical_path) == P055_HISTORICAL_WRL_CSV_SHA256,
                "historical WRL CSV is not the frozen P055 control")
        columns = orvd["columns"]
        orvd_time = columns["time_seconds"]
        result82 = orvd_result82_station(columns)
        body = orvd_body_station(columns)
        primary, result82_grids = compare_pair(
            orvd_time, result82, p047["time_seconds"], p047["station_meters"]
        )
        current_control, _ = compare_pair(
            orvd_time, body, current["time_seconds"], current["station_meters"]
        )
        historical_control, _ = compare_pair(
            orvd_time, body, historical["time_seconds"], historical["station_meters"]
        )
        body_grids = shared_station_grids([
            body, current["station_meters"], historical["station_meters"]
        ])
        summary = {
            "scope": "G62 GZ18 R300+AAR5 16 s station propagation",
            "comparison_rules": {
                "time_domain": "common native sample identities only; no interpolation",
                "station_domain": (
                    "0.01 m common grid; arrival is the first native sample at or "
                    "beyond each station; no interpolation"
                ),
                "shift_filter_fit_scale": "none",
                "station_segments": [segment[0] for segment in STATION_SEGMENTS],
            },
            "observer_contracts": {
                "primary": (
                    "ORVD mean of left/right rail-profile reference stations versus "
                    "SIMPACK Result Element 82 channel 3"
                ),
                "control": (
                    "ORVD wheelset-body centerline projection versus WRL/P055 "
                    "wheelset-body centerline projection"
                ),
                "observers_are_not_cross_compared": True,
            },
            "sources": {
                "orvd_artifact_directory": str(artifact),
                "orvd_observations_sha256": orvd["observation_sha256"],
                "p047_station_reference_npz": str(p047_path),
                "p047_station_reference_npz_sha256": sha256_file(p047_path),
                "p047_source_sbr_sha256": p047["source_sbr_sha256"].item(),
                "current_wrl_csv": str(current_path),
                "current_wrl_csv_sha256": sha256_file(current_path),
                "current_wrl_source_revision": arguments.current_wrl_source_revision,
                "historical_p055_wrl_csv": str(historical_path),
                "historical_p055_wrl_csv_sha256": sha256_file(historical_path),
            },
            "execution_identity": execution,
            "orvd_numerical_execution_contract": orvd["metadata"].get(
                "numerical_execution_contract"
            ),
            "comparison": {
                "orvd_vs_simpack_result82": primary,
                "orvd_vs_current_wrl_body_projection": current_control,
                "orvd_vs_historical_p055_body_projection": historical_control,
            },
        }
        publish(
            arguments.output_directory.resolve(), summary, orvd_time, result82,
            p047, body, current, historical, result82_grids, body_grids,
        )
    except (AnalysisError, OSError, ValueError) as error:
        print(f"G62 analysis failed: {error}", file=sys.stderr)
        return 1
    print(f"published G62 comparison to {arguments.output_directory.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
