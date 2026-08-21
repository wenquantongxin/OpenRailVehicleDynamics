#!/usr/bin/env python3
"""Extract the IRW passive direct-SLV comparison ABI from a binary64 SBR."""

from __future__ import annotations

import argparse
import csv
import ctypes as ct
import json
from pathlib import Path

import numpy as np

from irw_passive_multispeed_simpack_direct_slv_scenario_catalog import (
    require_scenario,
)


DEFAULT_LIBSBR = Path("/opt/Simpack-2021x/run/bin/linux64/libsbr.so")
AXLES = ("ff", "fr", "rf", "rr")
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
SIMPACK_WHEELS = (
    "axle01_left",
    "axle01_right",
    "axle02_left",
    "axle02_right",
    "axle03_left",
    "axle03_right",
    "axle04_left",
    "axle04_right",
)
MOTOR_CHANNELS = tuple(
    f"forceOv.S_IRWBogie_{bogie}__F_Motor_{motor}_Simat.ov_005"
    for bogie in ("Front", "Rear")
    for motor in ("A", "B", "C", "D")
)
MAXIMUM_PATCH_COUNT = 8


class SbrLibrary:
    """Small, process-local binding for the scalar parts of libsbr."""

    def __init__(self, path: Path) -> None:
        self.library = ct.CDLL(str(path))
        self._declare("SbrFileOpen", [ct.c_char_p, ct.c_char_p, ct.POINTER(ct.c_int)])
        self._declare(
            "SbrResultListGet",
            [ct.POINTER(ct.c_int), ct.POINTER(ct.POINTER(ct.c_void_p))],
        )
        self._declare("SbrResultActiveSet", [ct.c_void_p])
        self._declare("SbrResultFpnPrecisionGet", [ct.POINTER(ct.c_int)])
        self._declare("SbrResultFpnStorageSizeGet", [ct.POINTER(ct.c_int)])
        self._declare(
            "SbrResultStructChildrenGet",
            [
                ct.c_void_p,
                ct.POINTER(ct.c_int),
                ct.POINTER(ct.POINTER(ct.c_void_p)),
            ],
        )
        self._declare(
            "SbrResultStructIdNameGet", [ct.c_void_p, ct.POINTER(ct.c_char_p)]
        )
        self._declare(
            "SbrResultStructChannelGet", [ct.c_void_p, ct.POINTER(ct.c_void_p)]
        )
        self._declare("SbrChannelTypeGet", [ct.c_void_p, ct.POINTER(ct.c_int)])
        self._declare(
            "SbrChannelValueNumberGet",
            [ct.c_void_p, ct.POINTER(ct.c_size_t)],
        )
        self._declare(
            "SbrChannelDoublesGet",
            [ct.c_void_p, ct.c_size_t, ct.c_size_t, ct.POINTER(ct.c_double)],
        )
        self._declare("SbrCleanup", [])

    def _declare(self, name: str, arguments: list[object]) -> None:
        function = getattr(self.library, name)
        function.argtypes = arguments
        function.restype = ct.c_int

    @staticmethod
    def check(code: int, operation: str) -> None:
        if code != 0:
            raise RuntimeError(f"libsbr {operation} failed with error {code}")


def required_channel_paths() -> set[str]:
    result = {"time"}
    for axle in range(1, 5):
        prefix = f"yout.Y_axle{axle:02d}"
        result.update(
            {
                prefix + "_s",
                prefix + "_y",
                prefix + "_yaw",
                prefix + "_left_w",
                prefix + "_right_w",
            }
        )
    for simpack_wheel in SIMPACK_WHEELS:
        result.add(f"yout.Y_QVertical_{simpack_wheel}")
        for patch in range(1, MAXIMUM_PATCH_COUNT + 1):
            prefix = f"yout.Y_contact_{simpack_wheel}_patch{patch:02d}"
            result.update({prefix + "_tx", prefix + "_ty", prefix + "_normal"})
    result.update(MOTOR_CHANNELS)
    return result


def read_selected_channels(
    source: Path, libsbr_path: Path
) -> tuple[dict[str, np.ndarray], int, int]:
    source = source.resolve()
    libsbr_path = libsbr_path.resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    if not libsbr_path.is_file():
        raise FileNotFoundError(libsbr_path)

    required = required_channel_paths()
    channels: dict[str, np.ndarray] = {}
    api = SbrLibrary(libsbr_path)
    try:
        file_id = ct.c_int()
        api.check(
            api.library.SbrFileOpen(
                str(source).encode("utf-8"), b"r", ct.byref(file_id)
            ),
            "SbrFileOpen",
        )
        result_count = ct.c_int()
        results = ct.POINTER(ct.c_void_p)()
        api.check(
            api.library.SbrResultListGet(ct.byref(result_count), ct.byref(results)),
            "SbrResultListGet",
        )
        if result_count.value != 1:
            raise RuntimeError(
                f"expected exactly one SBR result, found {result_count.value}"
            )
        api.check(api.library.SbrResultActiveSet(results[0]), "SbrResultActiveSet")
        precision = ct.c_int()
        storage = ct.c_int()
        api.check(
            api.library.SbrResultFpnPrecisionGet(ct.byref(precision)),
            "SbrResultFpnPrecisionGet",
        )
        api.check(
            api.library.SbrResultFpnStorageSizeGet(ct.byref(storage)),
            "SbrResultFpnStorageSizeGet",
        )
        if precision.value != 8 or storage.value != 8:
            raise RuntimeError(
                "comparison SBR must use binary64 values; "
                f"precision={precision.value}, storage={storage.value}"
            )

        def walk(parent: ct.c_void_p | None, prefix: str = "") -> None:
            child_count = ct.c_int()
            children = ct.POINTER(ct.c_void_p)()
            api.check(
                api.library.SbrResultStructChildrenGet(
                    parent, ct.byref(child_count), ct.byref(children)
                ),
                "SbrResultStructChildrenGet",
            )
            for index in range(child_count.value):
                node = children[index]
                name = ct.c_char_p()
                api.check(
                    api.library.SbrResultStructIdNameGet(node, ct.byref(name)),
                    "SbrResultStructIdNameGet",
                )
                if name.value is None:
                    raise RuntimeError("SBR result node has no identifier")
                node_name = name.value.decode("utf-8", errors="strict")
                node_path = f"{prefix}.{node_name}" if prefix else node_name
                if node_path in required:
                    if node_path in channels:
                        raise RuntimeError(f"duplicate SBR channel {node_path}")
                    channel = ct.c_void_p()
                    api.check(
                        api.library.SbrResultStructChannelGet(node, ct.byref(channel)),
                        "SbrResultStructChannelGet",
                    )
                    channel_type = ct.c_int()
                    api.check(
                        api.library.SbrChannelTypeGet(channel, ct.byref(channel_type)),
                        "SbrChannelTypeGet",
                    )
                    if channel_type.value != 0:
                        raise RuntimeError(
                            f"required SBR channel {node_path} is not scalar"
                        )
                    value_count = ct.c_size_t()
                    api.check(
                        api.library.SbrChannelValueNumberGet(
                            channel, ct.byref(value_count)
                        ),
                        "SbrChannelValueNumberGet",
                    )
                    buffer = (ct.c_double * value_count.value)()
                    api.check(
                        api.library.SbrChannelDoublesGet(
                            channel, 0, value_count.value, buffer
                        ),
                        "SbrChannelDoublesGet",
                    )
                    channels[node_path] = np.array(
                        np.ctypeslib.as_array(buffer), dtype=np.float64, copy=True
                    )
                walk(node, node_path)

        walk(None)
    finally:
        api.check(api.library.SbrCleanup(), "SbrCleanup")

    missing = sorted(required - channels.keys())
    if missing:
        preview = ", ".join(missing[:8])
        raise RuntimeError(
            f"SBR is missing {len(missing)} required channels: {preview}"
        )
    return channels, precision.value, storage.value


def write_tsv(path: Path, headings: list[str], rows: list[list[object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, delimiter="\t", lineterminator="\n")
        writer.writerow(headings)
        writer.writerows(rows)


def number(value: float) -> str:
    return format(float(value), ".17g")


def extract(
    sbr_path: Path,
    output_directory: Path,
    scenario_identifier: str,
    libsbr_path: Path = DEFAULT_LIBSBR,
) -> dict[str, object]:
    scenario = require_scenario(scenario_identifier)
    if output_directory.exists():
        raise FileExistsError(output_directory)
    output_directory.mkdir(parents=True)

    channels, precision, storage = read_selected_channels(sbr_path, libsbr_path)
    time = channels["time"]
    expected_count = (
        int(round(scenario.duration_seconds * scenario.output_frequency_hertz)) + 1
    )
    expected_time = np.arange(expected_count, dtype=np.float64) / (
        scenario.output_frequency_hertz
    )
    if time.shape != expected_time.shape:
        raise RuntimeError(f"SBR has {time.size} samples, expected {expected_count}")
    if not np.isfinite(time).all() or not np.all(np.diff(time) > 0.0):
        raise RuntimeError("SBR time channel is not finite and increasing")
    if float(np.max(np.abs(time - expected_time))) > 5.0e-13:
        raise RuntimeError("SBR time channel is not the requested native clock")
    for path, values in channels.items():
        if values.shape != time.shape:
            raise RuntimeError(
                f"SBR channel {path} has {values.size} values, expected {time.size}"
            )
        if not np.isfinite(values).all():
            raise RuntimeError(f"SBR channel {path} contains non-finite values")

    motor_torques = np.column_stack([channels[path] for path in MOTOR_CHANNELS])
    if np.count_nonzero(motor_torques) != 0:
        maximum = float(np.max(np.abs(motor_torques)))
        raise RuntimeError(
            f"passive SIMPACK run contains nonzero motor torque: {maximum} N m"
        )

    patch_forces = np.zeros(
        (time.size, len(WHEELS), MAXIMUM_PATCH_COUNT, 3), dtype=np.float64
    )
    for wheel, simpack_wheel in enumerate(SIMPACK_WHEELS):
        for patch in range(MAXIMUM_PATCH_COUNT):
            prefix = f"yout.Y_contact_{simpack_wheel}_patch{patch + 1:02d}"
            patch_forces[:, wheel, patch, 0] = -channels[prefix + "_normal"]
            patch_forces[:, wheel, patch, 1] = channels[prefix + "_tx"]
            patch_forces[:, wheel, patch, 2] = channels[prefix + "_ty"]
    nonzero_patch = np.any(patch_forces != 0.0, axis=3)
    force_totals = np.sum(patch_forces, axis=2)

    headings = ["sample_index", "time_seconds"]
    for axle in AXLES:
        headings.extend(
            [
                f"axlebridge_{axle}.track_station_meters",
                f"axlebridge_{axle}.lateral_meters",
                f"axlebridge_{axle}.raw_simpack_yaw_radians",
            ]
        )
    for wheel in WHEELS:
        headings.extend(
            [
                f"{wheel}.raw_simpack_rate_radians_per_second",
                f"{wheel}.contact_patch_count",
                f"{wheel}.q_vertical_newtons",
                f"{wheel}.normal_force_newtons",
                f"{wheel}.longitudinal_force_newtons",
                f"{wheel}.lateral_force_newtons",
                f"{wheel}.applied_simpack_input_torque_newton_metres",
            ]
        )

    rows: list[list[object]] = []
    for sample in range(time.size):
        row: list[object] = [sample, number(time[sample])]
        for axle_index in range(4):
            prefix = f"yout.Y_axle{axle_index + 1:02d}"
            row.extend(
                [
                    number(channels[prefix + "_s"][sample]),
                    number(channels[prefix + "_y"][sample]),
                    number(channels[prefix + "_yaw"][sample]),
                ]
            )
        for wheel, simpack_wheel in enumerate(SIMPACK_WHEELS):
            axle = wheel // 2 + 1
            side = "left" if wheel % 2 == 0 else "right"
            row.extend(
                [
                    number(channels[f"yout.Y_axle{axle:02d}_{side}_w"][sample]),
                    int(np.count_nonzero(nonzero_patch[sample, wheel])),
                    number(-channels[f"yout.Y_QVertical_{simpack_wheel}"][sample]),
                    number(force_totals[sample, wheel, 0]),
                    number(force_totals[sample, wheel, 1]),
                    number(force_totals[sample, wheel, 2]),
                    number(motor_torques[sample, wheel]),
                ]
            )
        rows.append(row)
    write_tsv(output_directory / "observations.tsv", headings, rows)

    patch_rows: list[list[object]] = []
    for sample in range(time.size):
        for wheel, wheel_name in enumerate(WHEELS):
            for patch in range(MAXIMUM_PATCH_COUNT):
                if not nonzero_patch[sample, wheel, patch]:
                    continue
                values = patch_forces[sample, wheel, patch]
                patch_rows.append(
                    [
                        sample,
                        number(time[sample]),
                        wheel_name,
                        patch,
                        number(values[0]),
                        number(values[1]),
                        number(values[2]),
                    ]
                )
    write_tsv(
        output_directory / "contact_patches.tsv",
        [
            "sample_index",
            "time_seconds",
            "interface_name",
            "patch_ordinal",
            "normal_force_newtons",
            "longitudinal_force_newtons",
            "lateral_force_newtons",
        ],
        patch_rows,
    )

    metadata = {
        "complete": True,
        "scenario_identifier": scenario.identifier,
        "source_sbr": str(sbr_path.resolve()),
        "precision_bytes": precision,
        "storage_bytes": storage,
        "sample_count": int(time.size),
        "sample_period_seconds": 1.0 / scenario.output_frequency_hertz,
        "terminal_time_seconds": float(time[-1]),
        "contact_patch_row_count": len(patch_rows),
        "maximum_contact_patch_count_per_wheel": MAXIMUM_PATCH_COUNT,
        "simpack_yaw_sign_for_orvd_comparison": -1.0,
        "simpack_wheel_rate_sign_for_orvd_comparison": -1.0,
        "maximum_absolute_applied_motor_torque_newton_metres": 0.0,
    }
    (output_directory / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output_directory / "COMPLETE").write_text("complete\n", encoding="utf-8")
    return metadata


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sbr", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--libsbr", type=Path, default=DEFAULT_LIBSBR)
    arguments = parser.parse_args()
    metadata = extract(
        arguments.sbr, arguments.output, arguments.scenario, arguments.libsbr
    )
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
