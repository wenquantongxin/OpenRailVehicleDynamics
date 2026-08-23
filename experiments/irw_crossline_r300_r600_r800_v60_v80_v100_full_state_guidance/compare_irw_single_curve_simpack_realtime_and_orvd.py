#!/usr/bin/env python3
"""Compare one single-curve guidance arm on native SIMPACK and ORVD clocks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ORVD_PERIOD_NANOSECONDS = 500_000
SIMPACK_PERIOD_SECONDS = 0.001
ORVD_BLUE = "#0072B2"
SIMPACK_ORANGE = "#D55E00"
GRID = "#d6d6d6"
AXLES = (1, 3)
LEFT_WHEELS = {1: 1, 3: 5}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def read_orvd(run_directory: Path) -> pd.DataFrame:
    require((run_directory / "COMPLETE").is_file(), "ORVD run is incomplete")
    frame = pd.read_csv(run_directory / "observations.tsv", sep="\t")
    require(not frame.empty, "ORVD observations are empty")
    index = frame["sample_index"].to_numpy(np.int64)
    expected_index = np.arange(frame.shape[0], dtype=np.int64)
    require(np.array_equal(index, expected_index), "ORVD sample index is not contiguous")
    nanoseconds = frame["time_nanoseconds"].to_numpy(np.int64)
    require(
        np.array_equal(nanoseconds, expected_index * ORVD_PERIOD_NANOSECONDS),
        "ORVD observations do not use the native 0.5 ms clock",
    )
    require(
        (frame.shape[0] - 1) % 2 == 0,
        "ORVD terminal time does not lie on the SIMPACK 1 ms clock",
    )
    return frame


def read_simpack(run_directory: Path) -> pd.DataFrame:
    require((run_directory / "COMPLETE").is_file(), "SIMPACK run is incomplete")
    frame = pd.read_csv(run_directory / "observations.tsv", sep="\t")
    require(not frame.empty, "SIMPACK observations are empty")
    index = frame["sample_index"].to_numpy(np.int64)
    expected_index = np.arange(frame.shape[0], dtype=np.int64)
    require(
        np.array_equal(index, expected_index),
        "SIMPACK sample index is not contiguous",
    )
    expected_time = expected_index.astype(np.float64) * SIMPACK_PERIOD_SECONDS
    time = frame["time_seconds"].to_numpy(np.float64)
    require(
        float(np.max(np.abs(time - expected_time))) <= 1.0e-12,
        "SIMPACK observations do not use the requested native 1 ms clock",
    )
    return frame


def read_orvd_contact_forces(run_directory: Path, sample_count: int) -> dict[str, np.ndarray]:
    result = {
        quantity: np.zeros((sample_count, 8), dtype=np.float64)
        for quantity in ("N", "Tx", "Ty")
    }
    columns = {
        "N": "normal_force_newtons",
        "Tx": "longitudinal_force_on_wheel_in_contact_frame_newtons",
        "Ty": "lateral_force_on_wheel_in_contact_frame_newtons",
    }
    row_count = 0
    for chunk in pd.read_csv(
        run_directory / "contact_patches.tsv",
        sep="\t",
        usecols=["sample_index", "wheel_index", *columns.values()],
        chunksize=250_000,
    ):
        sample = chunk["sample_index"].to_numpy(np.int64)
        wheel = chunk["wheel_index"].to_numpy(np.int64)
        require(
            bool(
                (sample >= 0).all()
                and (sample < sample_count).all()
                and (wheel >= 0).all()
                and (wheel < 8).all()
            ),
            "ORVD contact table contains an invalid sample or wheel index",
        )
        for quantity, column in columns.items():
            values = chunk[column].to_numpy(np.float64)
            require(bool(np.isfinite(values).all()), f"ORVD {quantity} is non-finite")
            np.add.at(result[quantity], (sample, wheel), values)
        row_count += chunk.shape[0]
    require(row_count > 0, "ORVD contact table contains no patches")
    return result


def simpack_contact_forces(frame: pd.DataFrame) -> dict[str, np.ndarray]:
    # The frozen Type-78 ABI exposes wheel-side Tx/Ty. Its raw normal output is
    # negative in compression and is the only component requiring a sign change.
    return {
        "N": -np.column_stack(
            [frame[f"wheel_{wheel}_normal_force_newtons"] for wheel in range(1, 9)]
        ),
        "Tx": np.column_stack(
            [
                frame[f"wheel_{wheel}_longitudinal_force_newtons"]
                for wheel in range(1, 9)
            ]
        ),
        "Ty": np.column_stack(
            [frame[f"wheel_{wheel}_lateral_force_newtons"] for wheel in range(1, 9)]
        ),
    }


def style_axis(axis: plt.Axes, ylabel: str, xlabel: bool = True) -> None:
    axis.set_ylabel(ylabel)
    if xlabel:
        axis.set_xlabel("Time [s]")
    axis.grid(True, color=GRID, linewidth=0.65, alpha=0.62)
    axis.margins(x=0)
    axis.spines["top"].set_color("#4a4a4a")
    axis.spines["right"].set_color("#4a4a4a")


def plot_pair(axis: plt.Axes, time: np.ndarray, orvd: np.ndarray, simpack: np.ndarray) -> None:
    axis.plot(time, orvd, color=ORVD_BLUE, linewidth=0.9, label="ORVD")
    axis.plot(
        time,
        simpack,
        color=SIMPACK_ORANGE,
        linewidth=0.82,
        linestyle="--",
        label="SIMPACK",
    )


def add_figure_legend(figure: plt.Figure, axis: plt.Axes) -> None:
    handles, labels = axis.get_legend_handles_labels()
    figure.legend(
        handles,
        labels,
        loc="upper center",
        ncol=2,
        frameon=False,
        bbox_to_anchor=(0.5, 0.95),
    )


def plot_kinematics(
    path: Path,
    time: np.ndarray,
    orvd: pd.DataFrame,
    simpack: pd.DataFrame,
    title: str,
) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(13.2, 7.4), sharex=True)
    for column, axle in enumerate(AXLES):
        plot_pair(
            axes[0, column],
            time,
            orvd[f"axle_{axle}_lateral_displacement_meters"].to_numpy(np.float64)
            * 1.0e3,
            simpack[f"axle_{axle}_lateral_displacement_meters"].to_numpy(np.float64)
            * 1.0e3,
        )
        plot_pair(
            axes[1, column],
            time,
            orvd[f"axle_{axle}_yaw_angle_radians"].to_numpy(np.float64) * 1.0e3,
            simpack[f"axle_{axle}_yaw_angle_radians"].to_numpy(np.float64) * 1.0e3,
        )
        axes[0, column].set_title(f"Axle {axle}")
        style_axis(axes[0, column], "Lateral displacement [mm]", xlabel=False)
        style_axis(axes[1, column], "Yaw angle [mrad]")
    add_figure_legend(figure, axes[0, 0])
    figure.suptitle(f"{title}: axle response", y=0.995)
    figure.tight_layout(rect=(0, 0, 1, 0.92))
    figure.savefig(path, dpi=200, facecolor="white")
    plt.close(figure)


def plot_torques(
    path: Path,
    time: np.ndarray,
    orvd: pd.DataFrame,
    simpack: pd.DataFrame,
    title: str,
) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(13.2, 4.5), sharex=True)
    for axis, axle in zip(axes, AXLES, strict=True):
        wheel = LEFT_WHEELS[axle]
        column = f"wheel_{wheel}_actual_drive_torque_newton_metres"
        plot_pair(
            axis,
            time,
            orvd[column].to_numpy(np.float64),
            simpack[column].to_numpy(np.float64),
        )
        axis.set_title(f"Axle {axle} left wheel")
        style_axis(axis, "Drive torque [N m]")
    add_figure_legend(figure, axes[0])
    figure.suptitle(f"{title}: drive torque", y=0.995)
    figure.tight_layout(rect=(0, 0, 1, 0.88))
    figure.savefig(path, dpi=200, facecolor="white")
    plt.close(figure)


def station_derived_speed(station: np.ndarray) -> np.ndarray:
    require(station.size >= 3, "too few samples to derive longitudinal speed")
    return np.gradient(station, SIMPACK_PERIOD_SECONDS, edge_order=2)


def plot_speeds(
    path: Path,
    time: np.ndarray,
    orvd: pd.DataFrame,
    simpack: pd.DataFrame,
    title: str,
) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    figure, axes = plt.subplots(1, 2, figsize=(13.2, 4.5), sharex=True)
    result: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for axis, axle in zip(axes, AXLES, strict=True):
        station_column = f"axle_{axle}_track_station_meters"
        orvd_speed = station_derived_speed(orvd[station_column].to_numpy(np.float64))
        simpack_speed = station_derived_speed(
            simpack[station_column].to_numpy(np.float64)
        )
        result[axle] = (orvd_speed, simpack_speed)
        plot_pair(axis, time, orvd_speed * 3.6, simpack_speed * 3.6)
        axis.set_title(f"Axle {axle}")
        style_axis(axis, "Station-derived speed [km/h]", xlabel=False)
    add_figure_legend(figure, axes[0])
    figure.supxlabel("Time [s]", y=0.025)
    figure.suptitle(f"{title}: longitudinal speed", y=0.995)
    figure.tight_layout(rect=(0, 0.035, 1, 0.88))
    figure.savefig(path, dpi=200, facecolor="white")
    plt.close(figure)
    return result


def plot_contact_forces(
    path: Path,
    time: np.ndarray,
    orvd: dict[str, np.ndarray],
    simpack: dict[str, np.ndarray],
    title: str,
) -> None:
    figure, axes = plt.subplots(3, 2, figsize=(13.2, 9.2), sharex=True)
    labels = {"N": "N [kN]", "Tx": "Tx [kN]", "Ty": "Ty [kN]"}
    for row, quantity in enumerate(("N", "Tx", "Ty")):
        for column, axle in enumerate(AXLES):
            wheel = LEFT_WHEELS[axle] - 1
            axis = axes[row, column]
            plot_pair(
                axis,
                time,
                orvd[quantity][:, wheel] * 1.0e-3,
                simpack[quantity][:, wheel] * 1.0e-3,
            )
            if row == 0:
                axis.set_title(f"Axle {axle} left wheel")
            style_axis(axis, labels[quantity], xlabel=row == 2)
    add_figure_legend(figure, axes[0, 0])
    figure.suptitle(
        f"{title}: wheel-side contact forces",
        y=0.997,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.93))
    figure.savefig(path, dpi=200, facecolor="white")
    plt.close(figure)


def error_metrics(left: np.ndarray, right: np.ndarray, mask: np.ndarray) -> dict[str, float]:
    require(left.shape == right.shape == mask.shape, "metric shape mismatch")
    require(bool(mask.any()), "metric window is empty")
    error = left[mask] - right[mask]
    maximum_index = int(np.argmax(np.abs(error)))
    left_selected = left[mask]
    right_selected = right[mask]
    correlation = float(np.corrcoef(left_selected, right_selected)[0, 1])
    return {
        "rms": float(np.sqrt(np.mean(np.square(error)))),
        "maximum_absolute": float(np.max(np.abs(error))),
        "maximum_absolute_window_index": maximum_index,
        "correlation": correlation,
    }


def scaled_metrics(
    left: np.ndarray,
    right: np.ndarray,
    mask: np.ndarray,
    time: np.ndarray,
    scale: float,
) -> dict[str, float]:
    raw = error_metrics(left, right, mask)
    selected_time = time[mask]
    return {
        "rms": raw["rms"] * scale,
        "maximum_absolute": raw["maximum_absolute"] * scale,
        "maximum_absolute_time_seconds": float(
            selected_time[int(raw["maximum_absolute_window_index"])]
        ),
        "correlation": raw["correlation"],
    }


def comparison_metrics(
    time: np.ndarray,
    orvd: pd.DataFrame,
    simpack: pd.DataFrame,
    speeds: dict[int, tuple[np.ndarray, np.ndarray]],
    orvd_forces: dict[str, np.ndarray],
    simpack_forces: dict[str, np.ndarray],
) -> dict[str, object]:
    result: dict[str, object] = {
        "comparison_clock_period_seconds": SIMPACK_PERIOD_SECONDS,
        "time_shift_applied_seconds": 0.0,
        "terminal_time_seconds": float(time[-1]),
        "sample_count": int(time.size),
        "yaw_sign_applied_to_simpack": 1.0,
        "simpack_normal_force_sign_applied": -1.0,
        "windows": {},
    }
    windows: dict[str, object] = result["windows"]  # type: ignore[assignment]
    for axle in AXLES:
        station_column = f"axle_{axle}_track_station_meters"
        orvd_station = orvd[station_column].to_numpy(np.float64)
        simpack_station = simpack[station_column].to_numpy(np.float64)
        masks = {
            "full_native_time": np.ones(time.size, dtype=bool),
            "both_in_100_to_600_meters": (
                (orvd_station >= 100.0)
                & (orvd_station <= 600.0)
                & (simpack_station >= 100.0)
                & (simpack_station <= 600.0)
            ),
        }
        axle_result: dict[str, object] = {}
        for name, mask in masks.items():
            lateral_column = f"axle_{axle}_lateral_displacement_meters"
            yaw_column = f"axle_{axle}_yaw_angle_radians"
            torque_column = (
                f"wheel_{LEFT_WHEELS[axle]}_actual_drive_torque_newton_metres"
            )
            wheel = LEFT_WHEELS[axle] - 1
            orvd_speed, simpack_speed = speeds[axle]
            axle_result[name] = {
                "first_time_seconds": float(time[mask][0]),
                "last_time_seconds": float(time[mask][-1]),
                "sample_count": int(mask.sum()),
                "lateral_difference_millimetres": scaled_metrics(
                    orvd[lateral_column].to_numpy(np.float64),
                    simpack[lateral_column].to_numpy(np.float64),
                    mask,
                    time,
                    1.0e3,
                ),
                "yaw_difference_milliradians": scaled_metrics(
                    orvd[yaw_column].to_numpy(np.float64),
                    simpack[yaw_column].to_numpy(np.float64),
                    mask,
                    time,
                    1.0e3,
                ),
                "left_wheel_torque_difference_newton_metres": scaled_metrics(
                    orvd[torque_column].to_numpy(np.float64),
                    simpack[torque_column].to_numpy(np.float64),
                    mask,
                    time,
                    1.0,
                ),
                "station_derived_speed_difference_kilometres_per_hour": scaled_metrics(
                    orvd_speed, simpack_speed, mask, time, 3.6
                ),
                "left_wheel_contact_force_difference_kilonewtons": {
                    quantity: scaled_metrics(
                        orvd_forces[quantity][:, wheel],
                        simpack_forces[quantity][:, wheel],
                        mask,
                        time,
                        1.0e-3,
                    )
                    for quantity in ("N", "Tx", "Ty")
                },
            }
        windows[f"axle_{axle}"] = axle_result
    return result


def compare(
    orvd_directory: Path,
    simpack_directory: Path,
    output_directory: Path,
    title: str,
) -> None:
    require(not output_directory.exists(), "comparison output directory already exists")
    require(output_directory.parent.is_dir(), "comparison output parent does not exist")
    orvd_native = read_orvd(orvd_directory)
    simpack = read_simpack(simpack_directory)
    orvd = orvd_native.iloc[::2].reset_index(drop=True)
    require(orvd.shape[0] == simpack.shape[0], "terminal sample counts differ")
    orvd_time = orvd["time_seconds"].to_numpy(np.float64)
    simpack_time = simpack["time_seconds"].to_numpy(np.float64)
    require(
        float(np.max(np.abs(orvd_time - simpack_time))) <= 1.0e-12,
        "ORVD and SIMPACK have no exact common native time grid",
    )
    require(
        bool(np.isfinite(orvd.select_dtypes(include=[np.number]).to_numpy()).all())
        and bool(np.isfinite(simpack.select_dtypes(include=[np.number]).to_numpy()).all()),
        "comparison input contains non-finite values",
    )

    output_directory.mkdir()
    time = simpack_time
    plot_kinematics(
        output_directory / "axle_1_and_3_kinematics.png",
        time,
        orvd,
        simpack,
        title,
    )
    plot_torques(
        output_directory / "axle_1_and_3_left_wheel_torque.png",
        time,
        orvd,
        simpack,
        title,
    )
    speeds = plot_speeds(
        output_directory / "axle_1_and_3_longitudinal_speed.png",
        time,
        orvd,
        simpack,
        title,
    )

    orvd_force_native = read_orvd_contact_forces(orvd_directory, orvd_native.shape[0])
    orvd_forces = {key: value[::2] for key, value in orvd_force_native.items()}
    simpack_forces = simpack_contact_forces(simpack)
    for quantity in ("N", "Tx", "Ty"):
        require(
            bool(np.isfinite(orvd_forces[quantity]).all())
            and bool(np.isfinite(simpack_forces[quantity]).all()),
            f"{quantity} comparison contains non-finite values",
        )
    plot_contact_forces(
        output_directory / "axle_1_and_3_left_wheel_contact_forces.png",
        time,
        orvd_forces,
        simpack_forces,
        title,
    )

    metrics = comparison_metrics(
        time, orvd, simpack, speeds, orvd_forces, simpack_forces
    )
    metrics["plot_title"] = title
    (output_directory / "comparison_metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (output_directory / "COMPLETE").write_text("complete\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compare one IRW single-curve guidance arm on the SIMPACK 1 ms "
            "and ORVD 0.5 ms native output clocks."
        )
    )
    parser.add_argument("orvd_directory", type=Path)
    parser.add_argument("simpack_directory", type=Path)
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("--title", required=True)
    arguments = parser.parse_args()
    compare(
        arguments.orvd_directory.resolve(strict=True),
        arguments.simpack_directory.resolve(strict=True),
        arguments.output_directory.absolute(),
        arguments.title,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
