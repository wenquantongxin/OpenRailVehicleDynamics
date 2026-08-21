#!/usr/bin/env python3
"""Plot IRW passive SIMPACK direct-SLV and ORVD dynamics responses."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from irw_passive_multispeed_simpack_direct_slv_scenario_catalog import (
    Scenario,
    require_scenario,
)


AXLES = ("ff", "fr", "rf", "rr")
AXLE_TITLES = ("FF", "FR", "RF", "RR")
WHEELS = (
    "wheel_ff_l",
    "wheel_ff_r",
    "wheel_fr_l",
    "wheel_fr_r",
    "wheel_rf_l",
    "wheel_rf_r",
    "wheel_rr_l",
    "wheel_rr_r",
)
ORVD_BLUE = "#0072B2"
SIMPACK_ORANGE = "#D55E00"
GRID = "#d6d6d6"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def exact_clock(
    frame: pd.DataFrame, period: float, count: int, description: str
) -> np.ndarray:
    require(frame.shape[0] == count, f"{description} sample count mismatch")
    expected = np.arange(count, dtype=np.float64) * period
    actual = frame["time_seconds"].to_numpy(np.float64)
    require(
        float(np.max(np.abs(actual - expected))) <= 5.0e-13,
        f"{description} does not use the requested native clock",
    )
    return actual


def error_metrics(left: np.ndarray, right: np.ndarray) -> dict[str, float]:
    require(left.shape == right.shape and left.size > 0, "metric shape mismatch")
    valid = np.isfinite(left) & np.isfinite(right)
    require(bool(np.all(valid)), "metric contains non-finite values")
    error = left - right
    return {
        "rms": float(np.sqrt(np.mean(np.square(error)))),
        "maximum_absolute": float(np.max(np.abs(error))),
    }


def style_axis(axis: plt.Axes, ylabel: str, xlabel: bool) -> None:
    axis.set_ylabel(ylabel)
    if xlabel:
        axis.set_xlabel("Time [s]")
    axis.grid(True, color=GRID, linewidth=0.65, alpha=0.62)
    axis.margins(x=0)
    axis.spines["top"].set_color("#4a4a4a")
    axis.spines["right"].set_color("#4a4a4a")


def patch_force_totals(path: Path, sample_count: int) -> dict[str, np.ndarray]:
    frame = pd.read_csv(path, sep="\t")
    wheel_indices = frame["interface_name"].map(
        {name: index for index, name in enumerate(WHEELS)}
    )
    require(not bool(wheel_indices.isna().any()), "unknown wheel in patch table")
    samples = frame["sample_index"].to_numpy(np.int64)
    wheels = wheel_indices.to_numpy(np.int64)
    require(
        samples.size > 0
        and int(np.min(samples)) >= 0
        and int(np.max(samples)) < sample_count,
        "patch table has an invalid sample index",
    )
    result = {
        quantity: np.zeros((sample_count, len(WHEELS)), dtype=np.float64)
        for quantity in ("N", "Tx", "Ty")
    }
    columns = {
        "N": "normal_force_newtons",
        "Tx": "longitudinal_force_on_wheel_in_contact_frame_newtons",
        "Ty": "lateral_force_on_wheel_in_contact_frame_newtons",
    }
    if columns["Tx"] not in frame.columns:
        columns["Tx"] = "longitudinal_force_newtons"
        columns["Ty"] = "lateral_force_newtons"
    for quantity, column in columns.items():
        values = frame[column].to_numpy(np.float64)
        require(bool(np.isfinite(values).all()), f"non-finite {quantity} force")
        np.add.at(result[quantity], (samples, wheels), values)
    return result


def plot_macro(
    output: Path,
    title: str,
    orvd_time: np.ndarray,
    simpack_time: np.ndarray,
    orvd_lateral: np.ndarray,
    simpack_lateral: np.ndarray,
    orvd_yaw: np.ndarray,
    simpack_yaw: np.ndarray,
) -> None:
    figure, axes = plt.subplots(2, 4, figsize=(16, 6.8), sharex=True)
    for column, axle_title in enumerate(AXLE_TITLES):
        axes[0, column].plot(
            orvd_time,
            orvd_lateral[:, column] * 1.0e3,
            color=ORVD_BLUE,
            linewidth=1.05,
            label="ORVD",
        )
        axes[0, column].plot(
            simpack_time,
            simpack_lateral[:, column] * 1.0e3,
            color=SIMPACK_ORANGE,
            linewidth=0.95,
            linestyle="--",
            label="SIMPACK direct SLV",
        )
        axes[1, column].plot(
            orvd_time,
            orvd_yaw[:, column] * 1.0e3,
            color=ORVD_BLUE,
            linewidth=1.05,
            label="ORVD",
        )
        axes[1, column].plot(
            simpack_time,
            simpack_yaw[:, column] * 1.0e3,
            color=SIMPACK_ORANGE,
            linewidth=0.95,
            linestyle="--",
            label="SIMPACK direct SLV",
        )
        axes[0, column].set_title(axle_title, fontsize=12)
        style_axis(axes[0, column], "Lateral [mm]", False)
        style_axis(axes[1, column], "Yaw [mrad]", True)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.suptitle(title, fontsize=15, y=0.985)
    figure.legend(
        handles,
        labels,
        loc="upper center",
        ncol=2,
        bbox_to_anchor=(0.5, 0.948),
        frameon=False,
    )
    figure.subplots_adjust(
        left=0.062,
        right=0.992,
        bottom=0.09,
        top=0.875,
        hspace=0.20,
        wspace=0.25,
    )
    figure.savefig(output, dpi=220, facecolor="white")
    plt.close(figure)


def plot_forces(
    output: Path,
    title: str,
    orvd_time: np.ndarray,
    simpack_time: np.ndarray,
    orvd_force: dict[str, np.ndarray],
    simpack_force: dict[str, np.ndarray],
) -> None:
    quantities = ("N", "Tx", "Ty")
    figure, axes = plt.subplots(4, 6, figsize=(20, 11), sharex=True)
    for row, axle_title in enumerate(AXLE_TITLES):
        for side in range(2):
            wheel = row * 2 + side
            for quantity_index, quantity in enumerate(quantities):
                column = side * 3 + quantity_index
                axis = axes[row, column]
                axis.plot(
                    orvd_time,
                    orvd_force[quantity][:, wheel] * 1.0e-3,
                    color=ORVD_BLUE,
                    linewidth=0.65,
                    label="ORVD",
                )
                axis.plot(
                    simpack_time,
                    simpack_force[quantity][:, wheel] * 1.0e-3,
                    color=SIMPACK_ORANGE,
                    linewidth=0.75,
                    linestyle="--",
                    label="SIMPACK direct SLV",
                )
                if row == 0:
                    side_name = "L" if side == 0 else "R"
                    axis.set_title(f"{side_name} — {quantity}", fontsize=11)
                if column == 0:
                    axis.set_ylabel(f"{axle_title}\nForce [kN]")
                if row == 3:
                    axis.set_xlabel("Time [s]")
                axis.grid(True, color=GRID, linewidth=0.55, alpha=0.58)
                axis.margins(x=0)
                axis.spines["top"].set_color("#4a4a4a")
                axis.spines["right"].set_color("#4a4a4a")
                axis.tick_params(axis="both", labelsize=8)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.suptitle(title, fontsize=15, y=0.992)
    figure.legend(
        handles,
        labels,
        loc="upper center",
        ncol=2,
        bbox_to_anchor=(0.5, 0.958),
        frameon=False,
    )
    figure.subplots_adjust(
        left=0.06,
        right=0.992,
        bottom=0.065,
        top=0.90,
        hspace=0.24,
        wspace=0.22,
    )
    figure.savefig(output, dpi=220, facecolor="white")
    plt.close(figure)


def initial_orvd_wheel_rates(path: Path) -> np.ndarray:
    document = json.loads(path.read_text(encoding="utf-8"))
    rates = {}
    for joint in document["revolute_joint_startup_states"]:
        rates[joint["joint_name"]] = joint["rate"]["angular_rate_radians_per_second"]
    names = (
        "rev_wheel_ff_l",
        "rev_wheel_ff_r",
        "rev_wheel_fr_l",
        "rev_wheel_fr_r",
        "rev_wheel_rf_l",
        "rev_wheel_rf_r",
        "rev_wheel_rr_l",
        "rev_wheel_rr_r",
    )
    require(
        set(names) == set(rates),
        "startup state does not contain the closed eight-wheel rate set",
    )
    return np.array([rates[name] for name in names], dtype=np.float64)


def verify_run_scenario_identity(run_root: Path, scenario: Scenario) -> None:
    summary_path = run_root / "run_summary.json"
    require(summary_path.is_file(), "comparison run has no run_summary.json")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    require(summary.get("complete") is True, "comparison summary is incomplete")
    actual_identifier = summary.get("scenario_identifier")
    require(
        isinstance(actual_identifier, str) and bool(actual_identifier),
        "comparison summary has no scenario identity",
    )
    extracted_identifier = (
        summary.get("simpack", {}).get("extracted", {}).get("scenario_identifier")
    )
    require(
        extracted_identifier == actual_identifier,
        "SIMPACK extraction scenario differs from the run summary",
    )
    simpack_manifest = json.loads(
        (run_root / "simpack/manifest.json").read_text(encoding="utf-8")
    )
    orvd_manifest = json.loads(
        (run_root / "orvd/manifest.json").read_text(encoding="utf-8")
    )
    require(
        simpack_manifest.get("scenario_identifier") == actual_identifier
        and orvd_manifest.get("scenario_identifier") == actual_identifier,
        "solver manifests differ from the run scenario identity",
    )
    require(
        simpack_manifest.get("active_track") == scenario.simpack_track,
        "SIMPACK manifest track differs from the requested scenario",
    )
    for field, expected in (
        (
            "initial_speed_kilometres_per_hour",
            scenario.initial_speed_kilometres_per_hour,
        ),
        ("duration_seconds", scenario.duration_seconds),
        ("output_frequency_hertz", scenario.output_frequency_hertz),
    ):
        require(
            float(simpack_manifest.get(field, math.nan)) == expected,
            f"SIMPACK manifest {field} differs from the requested scenario",
        )
    orvd_metadata = summary.get("orvd", {}).get("metadata", {})
    require(
        orvd_metadata.get("track_irregularity_identifier")
        == scenario.irregularity_identifier,
        "ORVD run irregularity differs from the requested scenario",
    )
    require(
        float(
            orvd_metadata.get("initial_longitudinal_speed_meters_per_second", math.nan)
        )
        == scenario.initial_speed_kilometres_per_hour / 3.6,
        "ORVD run speed differs from the requested scenario",
    )
    geometry = orvd_metadata.get("input_paths", {}).get("track_geometry", "")
    require(
        Path(geometry).name == scenario.track_geometry,
        "ORVD run geometry differs from the requested scenario",
    )


def plot(run_root: Path, scenario_identifier: str) -> dict[str, object]:
    scenario = require_scenario(scenario_identifier)
    run_root = run_root.resolve()
    require((run_root / "RUN_COMPLETE").is_file(), "comparison run is incomplete")
    verify_run_scenario_identity(run_root, scenario)
    orvd_artifact = run_root / "orvd/artifact"
    simpack_artifact = run_root / "simpack/extracted"
    orvd = pd.read_csv(orvd_artifact / "observations.tsv", sep="\t")
    simpack = pd.read_csv(simpack_artifact / "observations.tsv", sep="\t")

    orvd_count = int(round(scenario.duration_seconds / 0.0005)) + 1
    simpack_count = (
        int(round(scenario.duration_seconds * scenario.output_frequency_hertz)) + 1
    )
    orvd_time = exact_clock(orvd, 0.0005, orvd_count, "ORVD")
    simpack_period = 1.0 / scenario.output_frequency_hertz
    simpack_time = exact_clock(
        simpack, simpack_period, simpack_count, "SIMPACK direct SLV"
    )
    decimation = int(round(simpack_period / 0.0005))
    require(
        np.array_equal(orvd_time[::decimation], simpack_time),
        "ORVD and SIMPACK have no exact common native clock",
    )

    orvd_lateral = np.column_stack(
        [
            orvd[f"axlebridge_{axle}.lateral_meters"].to_numpy(np.float64)
            for axle in AXLES
        ]
    )
    orvd_yaw = np.column_stack(
        [orvd[f"axlebridge_{axle}.yaw_radians"].to_numpy(np.float64) for axle in AXLES]
    )
    simpack_lateral = np.column_stack(
        [
            simpack[f"axlebridge_{axle}.lateral_meters"].to_numpy(np.float64)
            for axle in AXLES
        ]
    )
    simpack_yaw = -np.column_stack(
        [
            simpack[f"axlebridge_{axle}.raw_simpack_yaw_radians"].to_numpy(np.float64)
            for axle in AXLES
        ]
    )
    orvd_stations = np.column_stack(
        [
            orvd[f"axlebridge_{axle}.track_station_meters"].to_numpy(np.float64)
            for axle in AXLES
        ]
    )
    simpack_stations = np.column_stack(
        [
            simpack[f"axlebridge_{axle}.track_station_meters"].to_numpy(np.float64)
            for axle in AXLES
        ]
    )
    require(
        bool(np.isfinite(orvd_stations).all())
        and bool(np.isfinite(simpack_stations).all()),
        "track-station history contains non-finite values",
    )
    maximum_station = max(float(np.max(orvd_stations)), float(np.max(simpack_stations)))
    require(
        maximum_station <= scenario.maximum_comparison_track_station_meters + 1.0e-9,
        "comparison exceeds the frozen irregularity definition interval: "
        f"observed {maximum_station:.17g} m, upper bound "
        f"{scenario.maximum_comparison_track_station_meters:.17g} m",
    )

    plots = run_root / "plots"
    plots.mkdir(exist_ok=False)
    title_prefix = f"{scenario.response_plot_title}, {scenario.duration_seconds:g} s"
    macro_plot = plots / (scenario.response_plot_file_stem + "_axlebridge_response.png")
    plot_macro(
        macro_plot,
        title_prefix + " — native time response",
        orvd_time,
        simpack_time,
        orvd_lateral,
        simpack_lateral,
        orvd_yaw,
        simpack_yaw,
    )

    orvd_force = patch_force_totals(orvd_artifact / "contact_patches.tsv", orvd_count)
    simpack_force = {
        quantity: np.column_stack(
            [simpack[f"{wheel}.{suffix}"].to_numpy(np.float64) for wheel in WHEELS]
        )
        for quantity, suffix in (
            ("N", "normal_force_newtons"),
            ("Tx", "longitudinal_force_newtons"),
            ("Ty", "lateral_force_newtons"),
        )
    }
    force_plot = plots / (
        scenario.response_plot_file_stem + "_wheel_force_response.png"
    )
    plot_forces(
        force_plot,
        title_prefix + " — per-wheel patch-summed contact forces",
        orvd_time,
        simpack_time,
        orvd_force,
        simpack_force,
    )

    common_lateral = orvd_lateral[::decimation]
    common_yaw = orvd_yaw[::decimation]
    macro = {}
    for axle_index, axle in enumerate(AXLES):
        lateral = error_metrics(
            common_lateral[:, axle_index], simpack_lateral[:, axle_index]
        )
        yaw = error_metrics(common_yaw[:, axle_index], simpack_yaw[:, axle_index])
        macro[axle] = {
            "lateral_rms_millimetres": lateral["rms"] * 1.0e3,
            "lateral_maximum_absolute_millimetres": (
                lateral["maximum_absolute"] * 1.0e3
            ),
            "yaw_rms_milliradians": yaw["rms"] * 1.0e3,
            "yaw_maximum_absolute_milliradians": (yaw["maximum_absolute"] * 1.0e3),
        }
    force_metrics = {}
    for quantity in ("N", "Tx", "Ty"):
        metric = error_metrics(
            orvd_force[quantity][::decimation], simpack_force[quantity]
        )
        force_metrics[quantity] = {
            "rms_kilonewtons": metric["rms"] * 1.0e-3,
            "maximum_absolute_kilonewtons": (metric["maximum_absolute"] * 1.0e-3),
        }

    orvd_station = orvd_stations[0]
    simpack_station = simpack_stations[0]
    simpack_initial_rates = -np.array(
        [
            simpack[f"{wheel}.raw_simpack_rate_radians_per_second"].iloc[0]
            for wheel in WHEELS
        ],
        dtype=np.float64,
    )
    orvd_metadata = json.loads(
        (orvd_artifact / "metadata.json").read_text(encoding="utf-8")
    )
    startup_rates = initial_orvd_wheel_rates(
        Path(orvd_metadata["input_paths"]["resolved_startup_state"])
    )
    summary = {
        "scenario_identifier": scenario.identifier,
        "comparison_clock_period_seconds": simpack_period,
        "time_shift_applied_seconds": 0.0,
        "initial_identity": {
            "maximum_absolute_axle_station_difference_meters": float(
                np.max(np.abs(orvd_station - simpack_station))
            ),
            "maximum_absolute_forward_wheel_rate_difference_radians_per_second": (
                float(np.max(np.abs(startup_rates - simpack_initial_rates)))
            ),
            "orvd_reported_initial_speed_metres_per_second": float(
                orvd_metadata["initial_longitudinal_speed_meters_per_second"]
            ),
            "simpack_configured_startvel_metres_per_second": (
                scenario.initial_speed_kilometres_per_hour / 3.6
            ),
        },
        "maximum_observed_track_station_meters": maximum_station,
        "maximum_comparison_track_station_meters": (
            scenario.maximum_comparison_track_station_meters
        ),
        "macro_response": macro,
        "summed_contact_force_diagnostic": force_metrics,
        "force_output_qualification": (
            "diagnostic only at native 100 Hz SIMPACK output"
        ),
        "plots": {
            "axlebridge_response": str(macro_plot),
            "wheel_force_response": str(force_plot),
        },
    }
    (run_root / "comparison_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument("--scenario", required=True)
    arguments = parser.parse_args()
    result = plot(arguments.run_root, arguments.scenario)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
