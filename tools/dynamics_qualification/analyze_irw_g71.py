#!/usr/bin/env python3
"""Analyze the G71 IRW A-layer 30 s qualification against frozen evidence.

The primary response comparison uses each axle bridge's own station on the
fixed 100--450 m / 0.01 m grid. Contact quantities use the native 0.5 ms
clock. A 100 us local clock around the known WRL topology event is reported as
an observation only. No series is shifted, filtered, fitted, demeaned, rescaled
or assigned a sign from the observed result. SIMPACK and WRL source yaw is
converted once into the ORVD axle-bridge body convention by the frozen body
basis relation R_TB,ORVD = R_TB,source * diag(1,-1,-1).
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

plt.rcParams["axes.unicode_minus"] = False


AXLES = ("ff", "fr", "rf", "rr")
WHEELS = (
    "ff_l", "ff_r", "fr_l", "fr_r",
    "rf_l", "rf_r", "rr_l", "rr_r",
)
ORVD_CARRIERS = tuple(f"axlebridge_{axle}" for axle in AXLES)
ORVD_INTERFACES = tuple(f"wheel_{wheel}" for wheel in WHEELS)
BASE_PERIOD_NS = 500_000
TERMINAL_NS = 30_000_000_000
REFINEMENT_BEGIN_NS = 3_640_000_000
REFINEMENT_END_NS = 3_680_000_000
REFINEMENT_PERIOD_NS = 100_000
BASE_COUNT = 60_001
UNION_COUNT = 60_321
WRL_COUNT = 300_001
STATION_GRID = np.arange(10_000, 45_001, dtype=np.float64) * 0.01
MACRO_ALARM_MICRO = 100.0
PLOT_LABELS = {"orvd": "ORVD", "simpack": "SIMPACK", "wrl": "WRL"}
PLOT_STYLES = {
    "orvd": {"color": "#0072B2", "linestyle": "-", "linewidth": 1.1,
             "zorder": 1},
    "simpack": {"color": "#D55E00", "linestyle": "--", "linewidth": 1.0,
                "zorder": 3},
    "wrl": {"color": "#009E73", "linestyle": "-.", "linewidth": 0.9,
            "zorder": 2},
}
PLOT_SOURCE_ORDER = ("orvd", "simpack", "wrl")
SIMPACK_CSV_SHA256 = (
    "15b687dcb7a668d3b6f4e3d3155935a5fd3ed11d869701dc434d9de05d659fe4"
)
WRL_CSV_SHA256 = (
    "a35cf965164d71a92c5f877057587060a4b5853d149d77e01ad3208cb5f122e3"
)


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


def source_yaw_in_orvd_track_frame(value: np.ndarray) -> np.ndarray:
    """Convert the frozen SIMPACK/WRL axle-bridge yaw to ORVD Track-T.

    Source positions are already expressed in the physical track frame, so
    lateral displacement is unchanged. ORVD retains the WRL/Drake axle-bridge
    body basis. Its fixed relation to the source physical body basis is a right
    multiplication by C = diag(1,-1,-1). For the shared X-Z-Y resolver this
    leaves the denominator unchanged and negates yaw exactly. This is a source
    coordinate conversion, never a result-dependent sign choice.
    """
    return -np.asarray(value, dtype=np.float64)


def statistics(value: np.ndarray, scale: float = 1.0,
               coordinate: np.ndarray | None = None) -> dict[str, float | int]:
    scaled = np.asarray(value, dtype=np.float64) * scale
    require_finite(scaled, "statistics input")
    require(scaled.size > 0, "statistics input is empty")
    peak_index = int(np.argmax(np.abs(scaled)))
    result: dict[str, float | int] = {
        "sample_count": int(scaled.size),
        "signed_mean": float(np.mean(scaled)),
        "rms": float(np.sqrt(np.mean(np.square(scaled)))),
        "maximum_absolute": float(np.max(np.abs(scaled))),
    }
    if coordinate is not None:
        require(coordinate.shape == scaled.shape,
                "statistics coordinate shape differs from its values")
        result["maximum_absolute_coordinate"] = float(coordinate[peak_index])
    return result


def pair_statistics(left: np.ndarray, right: np.ndarray,
                    coordinate: np.ndarray | None = None
                    ) -> dict[str, float | int]:
    require(left.shape == right.shape, "same-time pair shapes differ")
    return statistics(left - right, coordinate=coordinate)


def masked_pair_statistics(left: np.ndarray, right: np.ndarray,
                           valid: np.ndarray, coordinate: np.ndarray
                           ) -> dict[str, float | int]:
    require(left.shape == right.shape == valid.shape == coordinate.shape,
            "masked pair shapes differ")
    require(valid.dtype == np.bool_, "masked pair selector is not boolean")
    result = statistics(left[valid] - right[valid],
                        coordinate=coordinate[valid])
    result["population_sample_count"] = int(valid.size)
    result["available_fraction"] = float(np.count_nonzero(valid) / valid.size)
    return result


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


def expected_union_ticks() -> np.ndarray:
    base = np.arange(0, TERMINAL_NS + BASE_PERIOD_NS, BASE_PERIOD_NS,
                     dtype=np.int64)
    local = np.arange(REFINEMENT_BEGIN_NS,
                      REFINEMENT_END_NS + REFINEMENT_PERIOD_NS,
                      REFINEMENT_PERIOD_NS, dtype=np.int64)
    result = np.union1d(base, local)
    require(result.shape == (UNION_COUNT,),
            "internal G71 union clock has the wrong sample count")
    return result


def read_orvd(
    directory: Path,
    expected_irregularity_identifier: str | None = None,
    layer_label: str = "A",
) -> dict[str, Any]:
    require(directory.is_dir(), f"ORVD artifact is absent: {directory}")
    for filename in ("COMPLETE", "metadata.json", "performance.json",
                     "observations.tsv", "contact_patches.tsv"):
        require((directory / filename).is_file(),
                f"ORVD artifact is missing {filename}")
    metadata = load_json(directory / "metadata.json", "ORVD metadata")
    performance = load_json(directory / "performance.json", "ORVD performance")
    expected_identity = {
        "completed": True,
        "qualification_vehicle_recipe": "IRW",
        "vehicle_name": "IRW",
        "mechanical_definition_identifier":
            "irw_reference_mechanical_definition",
        "load_condition_identifier": "irw_balanced_nominal_preload",
        "wheel_profile_identifier": "irw_reference_wheel_profile",
        "rail_profile_identifier": "uic60_rail_profile",
        "contact_strategy_identifier": "irw_reference_wheel_rail_contact",
        "track_irregularity_identifier": expected_irregularity_identifier,
        "initial_longitudinal_speed_meters_per_second": 16.666666666666668,
        "vehicle_layout_reference_track_station_meters": 0,
        "sample_period_nanoseconds": BASE_PERIOD_NS,
        "terminal_time_nanoseconds": TERMINAL_NS,
        "sample_count": UNION_COUNT,
        "base_track_definition_interval_meters": [0, 1150],
    }
    for key, expected in expected_identity.items():
        require(metadata.get(key) == expected,
                f"ORVD metadata {key!r} does not identify IRW layer {layer_label}")
    require(metadata.get("local_sample_refinement") == {
        "begin_time_nanoseconds": REFINEMENT_BEGIN_NS,
        "end_time_nanoseconds": REFINEMENT_END_NS,
        "sample_period_nanoseconds": REFINEMENT_PERIOD_NS,
    }, "ORVD metadata has the wrong G71 local clock")
    layout = metadata.get("assembled_state_and_force_layout")
    require(isinstance(layout, dict) and
            layout.get("generalized_position_count") == 81 and
            layout.get("generalized_velocity_count") == 74 and
            layout.get("series_force_state_count") == 2 and
            layout.get("vehicle_body_wrench_count") == 96 and
            layout.get("contact_body_wrench_count") == 8,
            "ORVD metadata has the wrong IRW layout")
    numerical = metadata.get("numerical_execution_contract")
    require(isinstance(numerical, dict) and
            numerical.get("relative_tolerance") == 1.0e-7 and
            numerical.get("generalized_position_absolute_tolerance") == 1.0e-9 and
            numerical.get("generalized_velocity_absolute_tolerance") == 1.0e-8 and
            numerical.get("series_force_absolute_tolerance_newtons") == 1.0e-6,
            "ORVD metadata has the wrong frozen IRW tolerances")
    require((directory / "COMPLETE").read_text(encoding="utf-8") ==
            f"{UNION_COUNT} samples\n",
            "ORVD COMPLETE marker has the wrong sample count")

    frame = pd.read_csv(directory / "observations.tsv", sep="\t")
    require(frame.shape[0] == UNION_COUNT,
            "ORVD observations have the wrong row count")
    require(len(frame.columns) == len(set(frame.columns)),
            "ORVD observations have duplicate columns")
    required = {"sample_index", "time_nanoseconds", "time_seconds"}
    for carrier in ORVD_CARRIERS:
        required.update({f"{carrier}.track_station_meters",
                         f"{carrier}.lateral_meters",
                         f"{carrier}.yaw_radians"})
    for interface in ORVD_INTERFACES:
        required.update({
            f"{interface}.contact_patch_count",
            f"{interface}.vertical_support_force_on_wheel_newtons",
            f"{interface}.normal_force_newtons",
            f"{interface}.primary_patch_normal_force_newtons",
            f"{interface}.longitudinal_force_on_wheel_newtons",
            f"{interface}.lateral_force_on_wheel_newtons",
        })
    missing = sorted(required - set(frame.columns))
    require(not missing, f"ORVD observations are missing columns: {missing}")
    require(np.array_equal(frame["sample_index"].to_numpy(np.int64),
                           np.arange(UNION_COUNT, dtype=np.int64)),
            "ORVD sample_index is not the union-clock ordinal")
    ticks = frame["time_nanoseconds"].to_numpy(np.int64)
    require(np.array_equal(ticks, expected_union_ticks()),
            "ORVD time_nanoseconds is not the G71 union clock")
    times = frame["time_seconds"].to_numpy(np.float64)
    require(np.max(np.abs(times - ticks.astype(np.float64) * 1.0e-9)) <= 4e-15,
            "ORVD floating time is detached from its integer tick")
    base_mask = ticks % BASE_PERIOD_NS == 0
    fine_mask = ((ticks >= REFINEMENT_BEGIN_NS) &
                 (ticks <= REFINEMENT_END_NS))
    require(np.count_nonzero(base_mask) == BASE_COUNT and
            np.count_nonzero(fine_mask) == 401,
            "ORVD union clock cannot be split into base and local clocks")

    def matrix(suffix: str, names: tuple[str, ...]) -> np.ndarray:
        value = frame[[f"{name}.{suffix}" for name in names]].to_numpy(np.float64)
        require_finite(value, f"ORVD {suffix}")
        return value

    patch_count = matrix("contact_patch_count", ORVD_INTERFACES)
    require(np.array_equal(patch_count, np.rint(patch_count)) and
            bool(np.all(patch_count >= 0)),
            "ORVD patch counts are not non-negative integers")
    patch_count_int = patch_count.astype(np.int64)
    patch_frame = pd.read_csv(directory / "contact_patches.tsv", sep="\t")
    patch_required = {
        "sample_index", "time_nanoseconds", "time_seconds", "interface_name",
        "patch_ordinal", "normal_force_newtons",
        "longitudinal_force_on_wheel_in_contact_frame_newtons",
        "lateral_force_on_wheel_in_contact_frame_newtons",
    }
    patch_missing = sorted(patch_required - set(patch_frame.columns))
    require(not patch_missing,
            f"ORVD patch observations are missing columns: {patch_missing}")
    patch_sample = patch_frame["sample_index"].to_numpy(np.int64)
    require(bool(np.all((patch_sample >= 0) & (patch_sample < UNION_COUNT))),
            "ORVD patch observation has an invalid sample ordinal")
    require(np.array_equal(
        patch_frame["time_nanoseconds"].to_numpy(np.int64), ticks[patch_sample]
    ) and np.array_equal(
        patch_frame["time_seconds"].to_numpy(np.float64), times[patch_sample]
    ), "ORVD patch and fixed-width observation clocks differ")
    local = {
        quantity: np.full((UNION_COUNT, len(WHEELS)), np.nan, dtype=np.float64)
        for quantity in ("N", "Tx", "Ty")
    }
    patch_columns = {
        "N": "normal_force_newtons",
        "Tx": "longitudinal_force_on_wheel_in_contact_frame_newtons",
        "Ty": "lateral_force_on_wheel_in_contact_frame_newtons",
    }
    for interface_index, interface in enumerate(ORVD_INTERFACES):
        selected = patch_frame["interface_name"] == interface
        rows = patch_frame.loc[selected]
        row_sample = rows["sample_index"].to_numpy(np.int64)
        require(np.array_equal(
            np.bincount(row_sample, minlength=UNION_COUNT),
            patch_count_int[:, interface_index],
        ), f"ORVD patch rows do not reproduce {interface} patch counts")
        single_rows = patch_count_int[row_sample, interface_index] == 1
        single_sample = row_sample[single_rows]
        require(np.array_equal(
            np.sort(single_sample),
            np.flatnonzero(patch_count_int[:, interface_index] == 1),
        ), f"ORVD {interface} does not have one row at every single-patch sample")
        for quantity, column in patch_columns.items():
            values = rows[column].to_numpy(np.float64)[single_rows]
            require_finite(values, f"ORVD {interface} single-patch {quantity}")
            local[quantity][single_sample, interface_index] = values
    summary_columns = {
        "N": "primary_patch_normal_force_newtons",
        "Tx": "longitudinal_force_on_wheel_newtons",
        "Ty": "lateral_force_on_wheel_newtons",
    }
    for quantity, suffix in summary_columns.items():
        summary = matrix(suffix, ORVD_INTERFACES)
        single = patch_count_int == 1
        require(np.array_equal(summary[single], local[quantity][single]),
                f"ORVD fixed-width {quantity} does not equal its sole patch")
    return {
        "metadata": metadata,
        "performance": performance,
        "ticks": ticks,
        "time": times,
        "base_mask": base_mask,
        "fine_mask": fine_mask,
        "station": matrix("track_station_meters", ORVD_CARRIERS),
        "lateral": matrix("lateral_meters", ORVD_CARRIERS),
        "yaw": matrix("yaw_radians", ORVD_CARRIERS),
        "patch_count": patch_count_int,
        "Q": matrix("vertical_support_force_on_wheel_newtons", ORVD_INTERFACES),
        "N_total": matrix("normal_force_newtons", ORVD_INTERFACES),
        "N": local["N"],
        "Tx": local["Tx"],
        "Ty": local["Ty"],
        "observation_sha256": sha256_file(directory / "observations.tsv"),
        "patch_observation_sha256": sha256_file(directory / "contact_patches.tsv"),
    }


def read_simpack(
    realtime_csv: Path,
    contact_npz: Path,
    expected_realtime_sha256: str = SIMPACK_CSV_SHA256,
    layer_label: str = "A",
) -> dict[str, Any]:
    require(sha256_file(realtime_csv) == expected_realtime_sha256,
            f"SIMPACK realtime CSV is not the frozen {layer_label}-layer authority")
    usecols = ["time_s", *(f"y_{i}" for i in range(8, 16)),
               *(f"y_{i}" for i in range(24, 28))]
    frame = pd.read_csv(realtime_csv, usecols=usecols,
                        float_precision="round_trip")
    require(frame.shape[0] == BASE_COUNT,
            "SIMPACK realtime CSV does not have 60001 rows")
    time = frame["time_s"].to_numpy(np.float64)
    expected_time = np.arange(BASE_COUNT, dtype=np.float64) * 0.0005
    require(np.max(np.abs(time - expected_time)) <= 2e-15,
            "SIMPACK realtime clock is not the native 0.5 ms sequence")
    with np.load(contact_npz, allow_pickle=False) as archive:
        required = {
            "time_seconds", "time_binary32_raw", "wheel_names",
            "contact_station_meters", "contact_Q_newtons",
            "contact_patch_count", "contact_slot_active",
            "contact_slot_N_newtons", "contact_slot_Tx_on_wheel_newtons",
            "contact_slot_Ty_on_wheel_newtons", "source_sbr_sha256",
            "source_sbr_precision_bytes", "source_sbr_storage_bytes",
            "force_basis_crosscheck",
        }
        require(required <= set(archive.files),
                "SIMPACK contact authority NPZ is incomplete")
        source_hash = str(np.asarray(archive["source_sbr_sha256"]).item())
        require(len(source_hash) == 64 and
                all(character in "0123456789abcdef" for character in source_hash),
                "SIMPACK contact authority lacks a valid source SBR digest")
        require(int(np.asarray(archive["source_sbr_precision_bytes"]).item()) == 4 and
                int(np.asarray(archive["source_sbr_storage_bytes"]).item()) == 4,
                "SIMPACK contact authority is not binary32")
        require(str(np.asarray(archive["force_basis_crosscheck"]).item()) ==
                "forceOv[-Tx,-Ty,-N] == [Tx_on_wheel,Ty_on_wheel,-N]",
                "SIMPACK contact authority did not validate the force basis")
        require(np.array_equal(np.asarray(archive["wheel_names"]),
                               np.asarray(WHEELS)),
                "SIMPACK contact authority has the wrong wheel order")
        sbr_time = np.array(archive["time_seconds"], copy=True)
        raw_time = np.array(archive["time_binary32_raw"], copy=True)
        station = np.array(archive["contact_station_meters"], copy=True)
        q = np.array(archive["contact_Q_newtons"], copy=True)
        count = np.array(archive["contact_patch_count"], copy=True)
        active = np.array(archive["contact_slot_active"], copy=True)
        n = np.array(archive["contact_slot_N_newtons"], copy=True)
        tx = np.array(archive["contact_slot_Tx_on_wheel_newtons"], copy=True)
        ty = np.array(archive["contact_slot_Ty_on_wheel_newtons"], copy=True)
    require(np.array_equal(sbr_time, expected_time) and
            np.array_equal(raw_time, expected_time.astype(np.float32)),
            "SIMPACK contact clocks are not the frozen native sequence")
    require(station.shape == q.shape == count.shape == (BASE_COUNT, 8) and
            active.shape == n.shape == tx.shape == ty.shape == (BASE_COUNT, 8, 5),
            "SIMPACK contact authority has the wrong shape")
    require(np.array_equal(np.sum(active, axis=2), count),
            "SIMPACK active slots do not equal Pair patch count")
    for value, name in ((station, "station"), (q, "Q"), (n, "N"),
                        (tx, "Tx"), (ty, "Ty")):
        require_finite(value, f"SIMPACK {name}")
    masked_n = np.where(active, n, -np.inf)
    primary_index = np.argmax(masked_n, axis=2)
    single_patch = count == 1
    gather = primary_index[..., None]
    primary = {}
    for name, value in (("N", n), ("Tx", tx), ("Ty", ty)):
        selected = np.take_along_axis(value, gather, axis=2)[..., 0]
        selected[~single_patch] = np.nan
        primary[name] = selected
    return {
        "time": time,
        "station": frame[[f"y_{i}" for i in range(24, 28)]].to_numpy(np.float64),
        "lateral": frame[[f"y_{i}" for i in range(8, 12)]].to_numpy(np.float64),
        "yaw": source_yaw_in_orvd_track_frame(
            frame[[f"y_{i}" for i in range(12, 16)]].to_numpy(np.float64)
        ),
        "contact_station": station,
        "Q": q,
        "patch_count": count.astype(np.int64),
        "N_total": np.sum(np.where(active, n, 0.0), axis=2),
        **primary,
        "sbr_sha256": source_hash,
        "realtime_sha256": expected_realtime_sha256,
    }


def read_wrl(
    csv_path: Path,
    result_json: Path,
    run_manifest_path: Path,
    expected_csv_sha256: str = WRL_CSV_SHA256,
    expected_manifest_arm: str = "A_passive_smooth",
    layer_label: str = "A",
) -> dict[str, Any]:
    require(sha256_file(csv_path) == expected_csv_sha256,
            f"WRL CSV is not the frozen {layer_label}-layer authority")
    usecols = ["t"]
    usecols += [f"s_wheelset_{axle}_m" for axle in AXLES]
    usecols += [f"y_wheelset_{axle}_m" for axle in AXLES]
    usecols += [f"yaw_wheelset_{axle}_rad" for axle in AXLES]
    usecols += [f"Fz_wheel_{wheel}_N" for wheel in WHEELS]
    usecols += [f"n_patches_wheel_{wheel}" for wheel in WHEELS]
    for quantity in ("N", "Tx", "Ty"):
        usecols += [f"{quantity}_maxN_patch_wheel_{wheel}_N" for wheel in WHEELS]
    frame = pd.read_csv(csv_path, usecols=usecols, float_precision="round_trip")
    require(frame.shape[0] == WRL_COUNT,
            f"WRL {layer_label}-layer CSV does not have 300001 rows")
    time = frame["t"].to_numpy(np.float64)
    require(np.max(np.abs(
        time - np.arange(WRL_COUNT, dtype=np.float64) * 0.0001
    )) <= 4.0e-15,
            f"WRL {layer_label}-layer CSV is not the native 100 us clock")
    document = load_json(result_json, "WRL result metadata")
    result = document.get("result")
    identity = document.get("numerical_execution_identity")
    require(isinstance(result, dict) and result.get("completed") is True,
            f"WRL {layer_label}-layer result did not complete")
    require(isinstance(identity, dict) and
            identity.get("integrator_backend") == "cvode" and
            identity.get("integrator_method") == "bdf" and
            identity.get("max_bdf_order") == 2 and
            identity.get("rtol") == 1.0e-7 and
            identity.get("atol_position") == 1.0e-9 and
            identity.get("atol_velocity") == 1.0e-8 and
            identity.get("atol_force_state") == 1.0e-6 and
            identity.get("linear_solver") == "spgmr",
            f"WRL {layer_label}-layer numerical identity is not the frozen run")
    manifest = load_json(run_manifest_path, "WRL run manifest")
    require(manifest.get("completed") is True and
            manifest.get("duration_s") == 30.0 and
            manifest.get("arm") == expected_manifest_arm,
            f"WRL run manifest is not the frozen {layer_label} layer")
    artifacts = manifest.get("artifacts_sha256")
    require(isinstance(artifacts, dict) and
            artifacts.get(csv_path.name) == expected_csv_sha256 and
            artifacts.get(result_json.name) == sha256_file(result_json) and
            Path(str(result.get("csv_path"))).resolve() == csv_path,
            f"WRL {layer_label}-layer CSV, result and manifest are not one run")

    def wheel_columns(prefix: str) -> np.ndarray:
        return frame[[f"{prefix}{wheel}_N" for wheel in WHEELS]].to_numpy(np.float64)

    patch_count = frame[
        [f"n_patches_wheel_{wheel}" for wheel in WHEELS]
    ].to_numpy(np.int64)
    local = {
        "N": wheel_columns("N_maxN_patch_wheel_"),
        "Tx": wheel_columns("Tx_maxN_patch_wheel_"),
        "Ty": wheel_columns("Ty_maxN_patch_wheel_"),
    }
    for value in local.values():
        value[patch_count != 1] = np.nan

    return {
        "time": time,
        "station": frame[[f"s_wheelset_{axle}_m" for axle in AXLES]].to_numpy(np.float64),
        "lateral": frame[[f"y_wheelset_{axle}_m" for axle in AXLES]].to_numpy(np.float64),
        "yaw": source_yaw_in_orvd_track_frame(
            frame[[f"yaw_wheelset_{axle}_rad" for axle in AXLES]].to_numpy(
                np.float64
            )
        ),
        "Fz_world": wheel_columns("Fz_wheel_"),
        "patch_count": patch_count,
        **local,
        "result": result,
        "numerical_identity": identity,
        "run_manifest": manifest,
        "csv_sha256": expected_csv_sha256,
    }


def same_station_statistics(sources: dict[str, dict[str, Any]]) -> tuple[dict[str, Any], dict[str, dict[str, np.ndarray]]]:
    values: dict[str, dict[str, np.ndarray]] = {name: {} for name in sources}
    output: dict[str, Any] = {}
    pairs = (("orvd_minus_simpack", "orvd", "simpack"),
             ("wrl_minus_simpack", "wrl", "simpack"),
             ("orvd_minus_wrl", "orvd", "wrl"))
    for axle_index, axle in enumerate(AXLES):
        output[axle] = {}
        for source_name, source in sources.items():
            station = source["station"][:, axle_index]
            require(bool(np.all(np.diff(station) > 0.0)),
                    f"{source_name} {axle} station is not strictly increasing")
            require(station[0] <= 100.0 and station[-1] >= 450.0,
                    f"{source_name} {axle} does not cover 100--450 m")
            for quantity in ("lateral", "yaw"):
                values[source_name][f"{axle}.{quantity}"] = np.interp(
                    STATION_GRID, station, source[quantity][:, axle_index]
                )
        for quantity in ("lateral", "yaw"):
            output[axle][quantity] = {
                label: statistics(values[left][f"{axle}.{quantity}"] -
                                  values[right][f"{axle}.{quantity}"], 1.0e6,
                                  STATION_GRID)
                for label, left, right in pairs
            }
    return output, values


def common_single_patch_values(
    sources: dict[str, dict[str, Any]], quantity: str
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    valid = np.ones_like(sources["orvd"][quantity], dtype=np.bool_)
    for source in ("simpack", "wrl", "orvd"):
        valid &= np.isfinite(sources[source][quantity])
    values = {}
    for source in ("simpack", "wrl", "orvd"):
        values[source] = np.where(valid, sources[source][quantity], np.nan)
    return valid, values


def contact_time_statistics(sources: dict[str, dict[str, Any]],
                            time: np.ndarray) -> tuple[dict[str, Any], dict[str, dict[str, np.ndarray]]]:
    pairs = (("orvd_minus_simpack", "orvd", "simpack"),
             ("wrl_minus_simpack", "wrl", "simpack"),
             ("orvd_minus_wrl", "orvd", "wrl"))
    output: dict[str, Any] = {}
    plotted: dict[str, dict[str, np.ndarray]] = {
        "Q": {source: sources[source]["Q"]
              for source in ("simpack", "orvd")}
    }
    local_values = {
        quantity: common_single_patch_values(sources, quantity)
        for quantity in ("N", "Tx", "Ty")
    }
    plotted.update({quantity: values for quantity, (_, values) in local_values.items()})
    for wheel_index, wheel in enumerate(WHEELS):
        output[wheel] = {
            "Q": {
                "orvd_minus_simpack": pair_statistics(
                    sources["orvd"]["Q"][:, wheel_index],
                    sources["simpack"]["Q"][:, wheel_index], time)
            },
        }
        for quantity in ("N", "Tx", "Ty"):
            valid = local_values[quantity][0][:, wheel_index]
            output[wheel][quantity] = {
                label: masked_pair_statistics(
                    sources[left][quantity][:, wheel_index],
                    sources[right][quantity][:, wheel_index], valid, time)
                for label, left, right in pairs
            }
        output[wheel]["total_N_orvd_minus_simpack"] = pair_statistics(
            sources["orvd"]["N_total"][:, wheel_index],
            sources["simpack"]["N_total"][:, wheel_index],
            time,
        )
    return output, plotted


def interpolate_valid_segments(grid: np.ndarray, station: np.ndarray,
                               value: np.ndarray) -> np.ndarray:
    require(station.shape == value.shape and station.ndim == 1,
            "segmented interpolation inputs have different shapes")
    require(bool(np.all(np.diff(station) > 0.0)),
            "segmented interpolation station is not strictly increasing")
    result = np.full(grid.shape, np.nan, dtype=np.float64)
    valid = np.isfinite(value)
    begins = np.flatnonzero(valid & np.r_[True, ~valid[:-1]])
    ends = np.flatnonzero(valid & np.r_[~valid[1:], True])
    for begin, end in zip(begins, ends, strict=True):
        if end <= begin:
            continue
        selected = (grid >= station[begin]) & (grid <= station[end])
        result[selected] = np.interp(
            grid[selected], station[begin:end + 1], value[begin:end + 1]
        )
    return result


def contact_station_statistics(
    sources: dict[str, dict[str, Any]]
) -> tuple[dict[str, Any], dict[str, dict[str, np.ndarray]]]:
    pairs = (("orvd_minus_simpack", "orvd", "simpack"),
             ("wrl_minus_simpack", "wrl", "simpack"),
             ("orvd_minus_wrl", "orvd", "wrl"))
    values: dict[str, dict[str, np.ndarray]] = {
        "Q": {}, "N": {}, "Tx": {}, "Ty": {},
    }
    output: dict[str, Any] = {}
    for wheel_index, wheel in enumerate(WHEELS):
        axle_index = wheel_index // 2
        output[wheel] = {}
        for source in ("simpack", "orvd"):
            station = sources[source]["station"][:, axle_index]
            values["Q"][source] = values["Q"].get(
                source, np.empty((STATION_GRID.size, len(WHEELS))))
            values["Q"][source][:, wheel_index] = np.interp(
                STATION_GRID, station, sources[source]["Q"][:, wheel_index]
            )
        output[wheel]["Q"] = {
            "orvd_minus_simpack": pair_statistics(
                values["Q"]["orvd"][:, wheel_index],
                values["Q"]["simpack"][:, wheel_index], STATION_GRID)
        }
        for quantity in ("N", "Tx", "Ty"):
            for source in ("simpack", "wrl", "orvd"):
                values[quantity][source] = values[quantity].get(
                    source, np.empty((STATION_GRID.size, len(WHEELS))))
                values[quantity][source][:, wheel_index] = interpolate_valid_segments(
                    STATION_GRID, sources[source]["station"][:, axle_index],
                    sources[source][quantity][:, wheel_index]
                )
            valid = np.ones(STATION_GRID.shape, dtype=np.bool_)
            for source in ("simpack", "wrl", "orvd"):
                valid &= np.isfinite(values[quantity][source][:, wheel_index])
            for source in ("simpack", "wrl", "orvd"):
                values[quantity][source][~valid, wheel_index] = np.nan
            output[wheel][quantity] = {
                label: masked_pair_statistics(
                    values[quantity][left][:, wheel_index],
                    values[quantity][right][:, wheel_index], valid, STATION_GRID)
                for label, left, right in pairs
            }
    return output, values


def zero_intervals(time: np.ndarray, count: np.ndarray) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for wheel_index, wheel in enumerate(WHEELS):
        zero = count[:, wheel_index] == 0
        begins = np.flatnonzero(zero & np.r_[True, ~zero[:-1]])
        ends = np.flatnonzero(zero & np.r_[~zero[1:], True])
        for begin, end in zip(begins, ends, strict=True):
            result.append({"wheel": wheel,
                           "begin_time_seconds": float(time[begin]),
                           "end_time_seconds": float(time[end]),
                           "sample_count": int(end - begin + 1)})
    return result


def plot_macro(path: Path, x: np.ndarray,
               values: dict[str, dict[str, np.ndarray]],
               x_label: str, title: str) -> None:
    figure, axes = plt.subplots(2, 4, figsize=(16, 7), sharex=True)
    handles = []
    for axle_index, axle in enumerate(AXLES):
        for row, (quantity, scale, unit) in enumerate((
            ("lateral", 1.0e3, "Lateral displacement [mm]"),
            ("yaw", 1.0e3, "Yaw [mrad]"),
        )):
            axis = axes[row, axle_index]
            for source in PLOT_SOURCE_ORDER:
                line, = axis.plot(
                    x, values[source][f"{axle}.{quantity}"] * scale,
                    label=PLOT_LABELS[source], **PLOT_STYLES[source])
                if axle_index == 0 and row == 0:
                    handles.append(line)
            axis.set_title(axle.upper())
            axis.set_ylabel(unit)
            axis.grid(True, alpha=0.25)
            if row == 1:
                axis.set_xlabel(x_label)
    figure.suptitle(title, y=0.985)
    figure.legend(handles, [PLOT_LABELS[source] for source in PLOT_SOURCE_ORDER],
                  loc="upper center", ncol=3, bbox_to_anchor=(0.5, 0.958),
                  frameon=False)
    figure.subplots_adjust(left=0.065, right=0.99, bottom=0.08, top=0.88,
                           hspace=0.32, wspace=0.28)
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_contact(path: Path, x: np.ndarray,
                 values: dict[str, np.ndarray], quantity: str,
                 source_order: tuple[str, ...], x_label: str,
                 comparison_kind: str, layer_label: str = "A") -> None:
    figure, axes = plt.subplots(2, 4, figsize=(16, 7), sharex=True)
    handles = []
    title = {
        "Q": "Q (total vertical support on wheel)",
        "N": "N (common single-patch samples)",
        "Tx": "Tx (common single-patch samples, wheel end)",
        "Ty": "Ty (common single-patch samples, wheel end)",
    }[quantity]
    for wheel_index, wheel in enumerate(WHEELS):
        axis = axes.flat[wheel_index]
        for source in source_order:
            line, = axis.plot(x, values[source][:, wheel_index],
                              label=PLOT_LABELS[source],
                              **PLOT_STYLES[source])
            if wheel_index == 0:
                handles.append(line)
        axis.set_title(wheel)
        axis.set_ylabel("Force [N]")
        if wheel_index >= 4:
            axis.set_xlabel(x_label)
        axis.grid(True, alpha=0.25)
    figure.suptitle(
        f"IRW layer {layer_label}, 30 s: {title} ({comparison_kind})",
        y=0.985)
    figure.legend(handles, [PLOT_LABELS[source] for source in source_order],
                  loc="upper center", ncol=len(source_order),
                  bbox_to_anchor=(0.5, 0.958), frameon=False)
    figure.subplots_adjust(left=0.065, right=0.99, bottom=0.08, top=0.88,
                           hspace=0.32, wspace=0.28)
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_topology(path: Path, begin: float, end: float,
                  sources: dict[str, dict[str, Any]], title: str) -> None:
    figure, axes = plt.subplots(2, 4, figsize=(16, 7), sharex=True)
    handles = []
    for wheel_index, wheel in enumerate(WHEELS):
        axis = axes.flat[wheel_index]
        for source in PLOT_SOURCE_ORDER:
            time = sources[source]["time"]
            mask = (time >= begin) & (time <= end)
            line, = axis.step(
                time[mask], sources[source]["patch_count"][mask, wheel_index],
                where="post", label=PLOT_LABELS[source],
                **PLOT_STYLES[source])
            if wheel_index == 0:
                handles.append(line)
        axis.set_title(wheel)
        axis.set_ylabel("Contact patch count")
        if wheel_index >= 4:
            axis.set_xlabel("Time [s]")
        axis.set_ylim(-0.1, 2.4)
        axis.grid(True, alpha=0.25)
    figure.suptitle(title, y=0.985)
    figure.legend(handles, [PLOT_LABELS[source] for source in PLOT_SOURCE_ORDER],
                  loc="upper center", ncol=3, bbox_to_anchor=(0.5, 0.958),
                  frameon=False)
    figure.subplots_adjust(left=0.065, right=0.99, bottom=0.08, top=0.88,
                           hspace=0.32, wspace=0.28)
    figure.savefig(path, dpi=220)
    plt.close(figure)


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orvd-artifact", type=Path, required=True)
    parser.add_argument("--orvd-execution-identity", type=Path, required=True)
    parser.add_argument("--simpack-realtime-csv", type=Path, required=True)
    parser.add_argument("--simpack-contact-npz", type=Path, required=True)
    parser.add_argument("--simpack-run-manifest", type=Path, required=True)
    parser.add_argument("--wrl-csv", type=Path, required=True)
    parser.add_argument("--wrl-result-json", type=Path, required=True)
    parser.add_argument("--wrl-run-manifest", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    output = arguments.output_directory.resolve()
    if output.exists():
        print(f"G71 analysis failed: output already exists: {output}", file=sys.stderr)
        return 1
    try:
        orvd_raw = read_orvd(arguments.orvd_artifact.resolve())
        execution = load_json(arguments.orvd_execution_identity.resolve(),
                              "ORVD execution identity")
        require(execution.get("exit_status") == 0 and
                execution.get("vehicle_recipe") == "irw" and
                execution.get("build_type") == "Release" and
                Path(str(execution.get("qualification_artifact_directory"))).resolve() ==
                arguments.orvd_artifact.resolve(),
                "ORVD execution identity is not the successful G71 Release run")
        simpack = read_simpack(arguments.simpack_realtime_csv.resolve(),
                               arguments.simpack_contact_npz.resolve())
        wrl_raw = read_wrl(arguments.wrl_csv.resolve(),
                           arguments.wrl_result_json.resolve(),
                           arguments.wrl_run_manifest.resolve())
        simpack_manifest = load_json(arguments.simpack_run_manifest.resolve(),
                                     "SIMPACK run manifest")
        require(simpack_manifest.get("completed") is True and
                simpack_manifest.get("duration_s") == 30.0 and
                simpack_manifest.get("arm") == "A_passive_smooth_R300",
                "SIMPACK manifest is not the frozen A layer")

        base = orvd_raw["base_mask"]
        fine = orvd_raw["fine_mask"]
        orvd = {key: value[base] for key, value in orvd_raw.items()
                if isinstance(value, np.ndarray) and value.shape[:1] == (UNION_COUNT,)}
        orvd["time"] = orvd_raw["time"][base]
        wrl_indices = np.arange(0, WRL_COUNT, 5, dtype=np.int64)
        wrl = {key: value[wrl_indices] for key, value in wrl_raw.items()
               if isinstance(value, np.ndarray) and value.shape[:1] == (WRL_COUNT,)}
        wrl["time"] = wrl_raw["time"][wrl_indices]
        base_time = np.arange(BASE_COUNT, dtype=np.float64) * 0.0005
        require(np.array_equal(orvd_raw["ticks"][base],
                               np.arange(BASE_COUNT, dtype=np.int64) * BASE_PERIOD_NS) and
                np.max(np.abs(orvd["time"] - base_time)) <= 4e-15 and
                np.max(np.abs(simpack["time"] - base_time)) <= 4.0e-15 and
                np.max(np.abs(wrl["time"] - base_time)) <= 4.0e-15,
                "the three 0.5 ms clocks are not the same integer sequence")

        macro_time_sources = {
            "orvd": {key: orvd[key] for key in ("station", "lateral", "yaw")},
            "simpack": {key: simpack[key] for key in ("station", "lateral", "yaw")},
            "wrl": {key: wrl[key] for key in ("station", "lateral", "yaw")},
        }
        macro_station_sources = {
            "orvd": {key: orvd_raw[key] for key in ("station", "lateral", "yaw")},
            "simpack": {key: simpack[key] for key in ("station", "lateral", "yaw")},
            "wrl": {key: wrl_raw[key] for key in ("station", "lateral", "yaw")},
        }
        macro_stats, station_values = same_station_statistics(macro_station_sources)
        macro_time_stats = {
            axle: {
                quantity: {
                    label: statistics(
                        macro_time_sources[left][quantity][:, axle_index] -
                        macro_time_sources[right][quantity][:, axle_index],
                        1.0e6, base_time)
                    for label, left, right in (
                        ("orvd_minus_simpack", "orvd", "simpack"),
                        ("wrl_minus_simpack", "wrl", "simpack"),
                        ("orvd_minus_wrl", "orvd", "wrl"),
                    )
                }
                for quantity in ("lateral", "yaw")
            }
            for axle_index, axle in enumerate(AXLES)
        }
        qualified = all(
            macro_stats[axle][quantity]["orvd_minus_simpack"][metric] <=
            MACRO_ALARM_MICRO
            for axle in AXLES for quantity in ("lateral", "yaw")
            for metric in ("rms", "maximum_absolute")
        )
        contact_time_sources = {
            "orvd": {key: orvd[key] for key in
                     ("Q", "N_total", "N", "Tx", "Ty", "patch_count")},
            "simpack": {key: simpack[key] for key in
                        ("Q", "N_total", "N", "Tx", "Ty", "patch_count")},
            "wrl": {key: wrl[key] for key in
                    ("N", "Tx", "Ty", "patch_count")},
        }
        for source in ("orvd", "simpack", "wrl"):
            contact_time_sources[source]["station"] = macro_time_sources[source]["station"]
        force_time_stats, force_time_values = contact_time_statistics(
            contact_time_sources, base_time)
        contact_station_sources = {
            "orvd": {key: orvd_raw[key] for key in
                     ("Q", "N_total", "N", "Tx", "Ty", "patch_count", "station")},
            "simpack": {key: simpack[key] for key in
                        ("Q", "N_total", "N", "Tx", "Ty", "patch_count", "station")},
            "wrl": {key: wrl_raw[key] for key in
                    ("N", "Tx", "Ty", "patch_count", "station")},
        }
        force_station_stats, force_station_values = contact_station_statistics(
            contact_station_sources)

        fine_begin = REFINEMENT_BEGIN_NS // REFINEMENT_PERIOD_NS
        fine_end = REFINEMENT_END_NS // REFINEMENT_PERIOD_NS
        orvd_fine = {key: value[fine] for key, value in orvd_raw.items()
                     if isinstance(value, np.ndarray) and value.shape[:1] == (UNION_COUNT,)}
        orvd_fine["time"] = orvd_raw["time"][fine]
        wrl_fine = {key: value[fine_begin:fine_end + 1]
                    for key, value in wrl_raw.items()
                    if isinstance(value, np.ndarray) and value.shape[:1] == (WRL_COUNT,)}
        wrl_fine["time"] = wrl_raw["time"][fine_begin:fine_end + 1]
        require(np.max(np.abs(
            orvd_fine["time"] - wrl_fine["time"]
        )) <= 4.0e-15,
                "ORVD/WRL local 100 us clocks differ")

        output.mkdir(parents=True)
        plot_macro(output / "irw_g71_macro_same_station.png", STATION_GRID,
                   station_values, "Own axle-bridge station [m]",
                   "IRW layer A: same-station response over 100–450 m")
        same_time_values = {
            source: {f"{axle}.{quantity}": macro_time_sources[source][quantity][:, index]
                     for index, axle in enumerate(AXLES)
                     for quantity in ("lateral", "yaw")}
            for source in ("simpack", "wrl", "orvd")
        }
        plot_macro(output / "irw_g71_macro_same_time.png", base_time,
                   same_time_values, "Time [s]",
                   "IRW layer A: native 0.5 ms same-time response")
        for quantity in ("Q", "N", "Tx", "Ty"):
            source_order = (("orvd", "simpack") if quantity == "Q" else
                            PLOT_SOURCE_ORDER)
            plot_contact(
                output / f"irw_g71_contact_{quantity}_same_time.png",
                base_time, force_time_values[quantity], quantity, source_order,
                "Time [s]", "native 0.5 ms same time",
            )
            plot_contact(
                output / f"irw_g71_contact_{quantity}_same_station.png",
                STATION_GRID, force_station_values[quantity], quantity,
                source_order, "Own axle-bridge station [m]",
                "same station over 100–450 m",
            )
        topology_base = {name: {"time": base_time,
                                "patch_count": contact_time_sources[name]["patch_count"]}
                         for name in ("simpack", "wrl", "orvd")}
        plot_topology(output / "irw_g71_topology_full.png", 0.0, 30.0,
                      topology_base,
                      "IRW layer A: full-window contact topology (observation)")
        topology_fine = {
            "simpack": {"time": simpack["time"],
                        "patch_count": simpack["patch_count"]},
            "wrl": {"time": wrl_fine["time"],
                    "patch_count": wrl_fine["patch_count"]},
            "orvd": {"time": orvd_fine["time"],
                     "patch_count": orvd_fine["patch_count"]},
        }
        plot_topology(output / "irw_g71_topology_3p64_3p68s.png", 3.64, 3.68,
                      topology_fine,
                      "IRW layer A: contact topology, 3.64–3.68 s "
                      "(ORVD/WRL 100 us; SIMPACK native 0.5 ms)")

        integration = orvd_raw["performance"].get("integration_statistics")
        require(isinstance(integration, dict),
                "ORVD performance lacks integration statistics")
        wall = execution.get("process_wall_seconds")
        peak_kib = execution.get("maximum_resident_set_kilobytes")
        require(isinstance(wall, (int, float)) and wall > 0 and
                isinstance(peak_kib, (int, float)) and peak_kib > 0,
                "ORVD execution identity lacks wall time or peak memory")
        nfe = int(integration.get("right_hand_side_evaluation_count", -1))
        nfels = int(integration.get("linear_solver_right_hand_side_evaluation_count", -1))
        require(nfe >= 0 and nfels >= 0,
                "ORVD integration statistics lack RHS counts")
        payload = {
            "classification": ("irw_A_layer_R300_30s_macro_qualified"
                               if qualified else
                               "irw_A_layer_R300_30s_macro_not_qualified"),
            "qualified": qualified,
            "scope": "IRW passive no-track-irregularity R300, 30 s macro response",
            "contract": {
                "same_station_grid_meters": [100.0, 450.0, 0.01],
                "macro_alarm_micro": MACRO_ALARM_MICRO,
                "base_clock": {"period_nanoseconds": BASE_PERIOD_NS,
                               "sample_count": BASE_COUNT},
                "local_topology_clock": {
                    "begin_nanoseconds": REFINEMENT_BEGIN_NS,
                    "end_nanoseconds": REFINEMENT_END_NS,
                    "period_nanoseconds": REFINEMENT_PERIOD_NS,
                    "sample_count": 401,
                },
                "union_sample_count": UNION_COUNT,
                "transformations": (
                    "no shift, fit, demeaning, filtering or result-related "
                    "sign/scale; SIMPACK and WRL source yaw is negated once "
                    "by R_TB,ORVD = R_TB,source * diag(1,-1,-1), while "
                    "source lateral displacement is already in physical "
                    "Track-T and is unchanged"
                ),
                "source_coordinate_conversion": {
                    "body_basis_relation": (
                        "R_TB,ORVD = R_TB,source * diag(1,-1,-1)"
                    ),
                    "lateral_multiplier": 1,
                    "yaw_multiplier": -1,
                },
                "contact_force_scope": (
                    "Q is the whole-interface Track-T support from SIMPACK and ORVD; "
                    "N/Tx/Ty compare all three implementations only where all have "
                    "exactly one patch, with no interpolation across topology changes"
                ),
                "contact_force_acceptance_threshold": None,
            },
            "same_station_macro_error_micro": macro_stats,
            "same_time_macro_error_micro": macro_time_stats,
            "same_time_station_difference_meters": {
                axle: {
                    label: pair_statistics(
                        macro_time_sources[left]["station"][:, index],
                        macro_time_sources[right]["station"][:, index], base_time)
                    for label, left, right in (
                        ("orvd_minus_simpack", "orvd", "simpack"),
                        ("wrl_minus_simpack", "wrl", "simpack"),
                        ("orvd_minus_wrl", "orvd", "wrl"),
                    )}
                for index, axle in enumerate(AXLES)
            },
            "same_time_contact_force_error_newtons": force_time_stats,
            "same_station_contact_force_error_newtons": force_station_stats,
            "contact_topology_observation": {
                "base_clock": {
                    source: {
                        "minimum_wheels_in_contact": int(np.min(np.sum(
                            contact_time_sources[source]["patch_count"] > 0, axis=1))),
                        "samples_with_fewer_than_eight_wheels": int(np.count_nonzero(
                            np.sum(contact_time_sources[source]["patch_count"] > 0, axis=1) < 8)),
                        "peak_patch_count": int(np.max(contact_time_sources[source]["patch_count"])),
                        "zero_patch_intervals": zero_intervals(
                            base_time, contact_time_sources[source]["patch_count"]),
                    } for source in ("simpack", "wrl", "orvd")
                },
                "local_100us_clock": {
                    "wrl": zero_intervals(wrl_fine["time"], wrl_fine["patch_count"]),
                    "orvd": zero_intervals(orvd_fine["time"], orvd_fine["patch_count"]),
                    "not_a_safety_or_exact_event_time_gate": True,
                },
            },
            "performance": {
                "orvd_process_wall_seconds": float(wall),
                "orvd_simulated_seconds_per_wall_second": 30.0 / float(wall),
                "orvd_wall_seconds_per_simulated_second": float(wall) / 30.0,
                "orvd_advance_wall_seconds": float(orvd_raw["performance"]["advance_wall_seconds"]),
                "orvd_postprocessing_wall_seconds": float(
                    orvd_raw["performance"]["observation_wall_seconds"] +
                    orvd_raw["performance"]["endpoint_diagnostics_wall_seconds"] +
                    orvd_raw["performance"]["data_and_metadata_write_wall_seconds"]),
                "orvd_peak_resident_memory_mib": float(peak_kib) / 1024.0,
                "orvd_dense_state_mib": float(orvd_raw["performance"]["dense_state_bytes"]) / 1048576.0,
                "orvd_observation_buffer_mib": float(orvd_raw["performance"]["observation_buffer_bytes"]) / 1048576.0,
                "orvd_integration_statistics": integration,
                "orvd_nominal_rhs_contact_evaluation_opportunity_upper_bound": (nfe + nfels) * 8,
                "orvd_offline_contact_observation_count": UNION_COUNT * 8,
                "wrl_simulation_wall_seconds": float(wrl_raw["result"]["wall_time_s"]),
                "wrl_wrapper_wall_seconds": float(wrl_raw["run_manifest"]["wall_time_s"]),
                "simpack_workflow_wall_seconds": float(simpack_manifest["wall_time_s"]),
                "comparison_note": "different solver/workflow boundaries; observations, not speed gates",
            },
            "source_identity": {
                "orvd_observations_sha256": orvd_raw["observation_sha256"],
                "orvd_contact_patches_sha256": orvd_raw["patch_observation_sha256"],
                "orvd_executable_sha256": execution.get("executable_sha256"),
                "orvd_revision": execution.get("orvd_revision"),
                "simpack_realtime_sha256": simpack["realtime_sha256"],
                "simpack_sbr_sha256": simpack["sbr_sha256"],
                "simpack_contact_authority_npz_sha256": sha256_file(
                    arguments.simpack_contact_npz.resolve()),
                "wrl_csv_sha256": wrl_raw["csv_sha256"],
                "wrl_numerical_execution_identity": wrl_raw["numerical_identity"],
            },
            "figures": sorted(path.name for path in output.glob("*.png")),
        }
        write_json(output / "irw_g71_analysis.json", payload)
    except (AnalysisError, KeyError, OSError, ValueError) as error:
        print(f"G71 analysis failed: {error}", file=sys.stderr)
        return 1
    print(f"wrote G71 analysis to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
