#!/usr/bin/env python3
"""Plot the native response of an IRW guidance experiment run."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


OBSERVATION_PERIOD_NANOSECONDS = 500_000
AXLE_COLOR = {1: "#0072B2", 3: "#D55E00"}
GRID_COLOR = "#d6d6d6"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def style_axis(axis: plt.Axes, ylabel: str, xlabel: bool = True) -> None:
    axis.set_ylabel(ylabel)
    if xlabel:
        axis.set_xlabel("Time [s]")
    axis.grid(True, color=GRID_COLOR, linewidth=0.65, alpha=0.62)
    axis.margins(x=0)
    axis.spines["top"].set_color("#4a4a4a")
    axis.spines["right"].set_color("#4a4a4a")


def read_observations(run_directory: Path) -> pd.DataFrame:
    columns = [
        "sample_index",
        "time_nanoseconds",
        "time_seconds",
        "axle_1_longitudinal_speed_meters_per_second",
        "axle_3_longitudinal_speed_meters_per_second",
        "axle_1_lateral_displacement_meters",
        "axle_3_lateral_displacement_meters",
        "axle_1_yaw_angle_radians",
        "axle_3_yaw_angle_radians",
        "wheel_1_actual_drive_torque_newton_metres",
        "wheel_5_actual_drive_torque_newton_metres",
    ]
    frame = pd.read_csv(
        run_directory / "observations.tsv", sep="\t", usecols=columns
    )
    require(not frame.empty, "observations.tsv is empty")
    sample_index = frame["sample_index"].to_numpy(np.int64)
    expected_index = np.arange(frame.shape[0], dtype=np.int64)
    require(
        bool(np.array_equal(sample_index, expected_index)),
        "observations.tsv does not contain a contiguous zero-based sample clock",
    )
    time_nanoseconds = frame["time_nanoseconds"].to_numpy(np.int64)
    require(
        bool(
            np.array_equal(
                time_nanoseconds,
                expected_index * OBSERVATION_PERIOD_NANOSECONDS,
            )
        ),
        "observations.tsv does not use the native 0.5 ms clock",
    )
    values = frame.drop(columns=["sample_index", "time_nanoseconds"]).to_numpy(
        np.float64
    )
    require(bool(np.isfinite(values).all()), "observations.tsv contains non-finite data")
    return frame


def accumulate_wear_number(
    run_directory: Path, sample_count: int
) -> np.ndarray:
    wear_by_sample_and_wheel = np.zeros((sample_count, 8), dtype=np.float64)
    columns = [
        "sample_index",
        "time_nanoseconds",
        "wheel_index",
        "longitudinal_force_on_wheel_in_contact_frame_newtons",
        "lateral_force_on_wheel_in_contact_frame_newtons",
        "longitudinal_creepage",
        "lateral_creepage",
    ]
    row_count = 0
    for chunk in pd.read_csv(
        run_directory / "contact_patches.tsv",
        sep="\t",
        usecols=columns,
        chunksize=250_000,
    ):
        sample = chunk["sample_index"].to_numpy(np.int64)
        time_nanoseconds = chunk["time_nanoseconds"].to_numpy(np.int64)
        wheel = chunk["wheel_index"].to_numpy(np.int64)
        require(
            bool(
                (sample >= 0).all()
                and (sample < sample_count).all()
                and (wheel >= 0).all()
                and (wheel < 8).all()
            ),
            "contact_patches.tsv contains an invalid sample or wheel index",
        )
        require(
            bool(
                np.array_equal(
                    time_nanoseconds,
                    sample * OBSERVATION_PERIOD_NANOSECONDS,
                )
            ),
            "contact_patches.tsv does not use the native 0.5 ms clock",
        )
        longitudinal_force = chunk[
            "longitudinal_force_on_wheel_in_contact_frame_newtons"
        ].to_numpy(np.float64)
        lateral_force = chunk[
            "lateral_force_on_wheel_in_contact_frame_newtons"
        ].to_numpy(np.float64)
        longitudinal_creepage = chunk["longitudinal_creepage"].to_numpy(
            np.float64
        )
        lateral_creepage = chunk["lateral_creepage"].to_numpy(np.float64)
        require(
            bool(
                np.isfinite(longitudinal_force).all()
                and np.isfinite(lateral_force).all()
                and np.isfinite(longitudinal_creepage).all()
                and np.isfinite(lateral_creepage).all()
            ),
            "contact_patches.tsv contains non-finite force or creepage data",
        )
        wear = np.abs(longitudinal_force * longitudinal_creepage) + np.abs(
            lateral_force * lateral_creepage
        )
        np.add.at(wear_by_sample_and_wheel, (sample, wheel), wear)
        row_count += chunk.shape[0]
    require(row_count > 0, "contact_patches.tsv contains no contact patches")
    return wear_by_sample_and_wheel


def plot_axle_response(
    output_path: Path,
    observations: pd.DataFrame,
    wear: np.ndarray,
    title: str,
) -> None:
    time = observations["time_seconds"].to_numpy(np.float64)
    figure, axes = plt.subplots(2, 2, figsize=(13.2, 7.4), sharex=True)
    panels = (
        (
            axes[0, 0],
            "lateral_displacement_meters",
            1.0e3,
            "Lateral displacement [mm]",
        ),
        (axes[0, 1], "yaw_angle_radians", 1.0e3, "Yaw angle [mrad]"),
    )
    for axis, suffix, scale, ylabel in panels:
        for axle in (1, 3):
            axis.plot(
                time,
                observations[f"axle_{axle}_{suffix}"].to_numpy(np.float64)
                * scale,
                color=AXLE_COLOR[axle],
                linewidth=0.85,
                label=f"Axle {axle}",
            )
        style_axis(axis, ylabel, xlabel=False)
        axis.legend(loc="best", frameon=False)

    for axle, wheel in ((1, 1), (3, 5)):
        axes[1, 0].plot(
            time,
            observations[
                f"wheel_{wheel}_actual_drive_torque_newton_metres"
            ].to_numpy(np.float64),
            color=AXLE_COLOR[axle],
            linewidth=0.85,
            label=f"Axle {axle} left wheel",
        )
        axes[1, 1].plot(
            time,
            wear[:, wheel - 1],
            color=AXLE_COLOR[axle],
            linewidth=0.85,
            label=f"Axle {axle} left wheel",
        )
    style_axis(axes[1, 0], "Drive torque [N m]")
    style_axis(axes[1, 1], "Instantaneous wear number [N]")
    axes[1, 0].legend(loc="best", frameon=False)
    axes[1, 1].legend(loc="best", frameon=False)
    figure.suptitle(f"{title}: axle response")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(figure)


def plot_longitudinal_speed(
    output_path: Path, observations: pd.DataFrame, title: str
) -> None:
    time = observations["time_seconds"].to_numpy(np.float64)
    figure, axis = plt.subplots(figsize=(12.6, 4.4))
    for axle in (1, 3):
        speed_kilometres_per_hour = (
            observations[
                f"axle_{axle}_longitudinal_speed_meters_per_second"
            ].to_numpy(np.float64)
            * 3.6
        )
        axis.plot(
            time,
            speed_kilometres_per_hour,
            color=AXLE_COLOR[axle],
            linewidth=0.9,
            label=f"Axle {axle}",
        )
    style_axis(axis, "Longitudinal speed [km/h]")
    axis.legend(loc="best", frameon=False)
    axis.set_title(f"{title}: longitudinal speed")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(figure)


def plot_whole_vehicle_wear(
    output_path: Path,
    observations: pd.DataFrame,
    wear: np.ndarray,
    title: str,
) -> None:
    time = observations["time_seconds"].to_numpy(np.float64)
    total = np.sum(wear, axis=1)
    dt = OBSERVATION_PERIOD_NANOSECONDS * 1.0e-9
    cumulative = np.zeros_like(total)
    cumulative[1:] = np.cumsum(0.5 * (total[:-1] + total[1:]) * dt)

    figure, left_axis = plt.subplots(figsize=(12.6, 4.8))
    right_axis = left_axis.twinx()
    instantaneous_line = left_axis.plot(
        time,
        total,
        color="#0072B2",
        linewidth=0.75,
        label="Instantaneous",
    )[0]
    cumulative_line = right_axis.plot(
        time,
        cumulative,
        color="#D55E00",
        linewidth=1.05,
        label="Cumulative integral",
    )[0]
    style_axis(left_axis, "Whole-vehicle wear number [N]")
    right_axis.set_ylabel("Wear-number integral [N s]")
    left_axis.legend(
        [instantaneous_line, cumulative_line],
        [instantaneous_line.get_label(), cumulative_line.get_label()],
        loc="best",
        frameon=False,
    )
    left_axis.set_title(f"{title}: whole-vehicle wear number")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--output-directory", type=Path)
    parser.add_argument(
        "--title", default="IRW cross-line full-state guidance"
    )
    parser.add_argument("--file-prefix", default="irw_crossline")
    arguments = parser.parse_args()

    run_directory = arguments.run_directory.resolve()
    require((run_directory / "COMPLETE").is_file(), "run is not complete")
    output_directory = (
        arguments.output_directory.resolve()
        if arguments.output_directory is not None
        else run_directory.with_name(run_directory.name + "-plots")
    )
    require(
        not output_directory.exists(),
        f"plot output already exists: {output_directory}",
    )
    output_directory.mkdir(parents=False)

    observations = read_observations(run_directory)
    wear = accumulate_wear_number(run_directory, observations.shape[0])
    response_path = (
        output_directory
        / f"{arguments.file_prefix}_axle_1_and_3_response.png"
    )
    speed_path = (
        output_directory / f"{arguments.file_prefix}_longitudinal_speed.png"
    )
    wear_path = (
        output_directory / f"{arguments.file_prefix}_whole_vehicle_wear.png"
    )
    plot_axle_response(response_path, observations, wear, arguments.title)
    plot_longitudinal_speed(speed_path, observations, arguments.title)
    plot_whole_vehicle_wear(wear_path, observations, wear, arguments.title)
    for path in (response_path, speed_path, wear_path):
        print(path)


if __name__ == "__main__":
    main()
