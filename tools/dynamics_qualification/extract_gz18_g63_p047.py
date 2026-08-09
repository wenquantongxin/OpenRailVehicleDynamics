#!/usr/bin/env python3
"""Extract the G63 macro-response and contact authority from P047 binary64.

The SIMPACK SBR reader remains owned by wheel-rail-lab.  This migration-only
tool reads the frozen P047 file exactly once, selects the named channels used by
G63, converts SIMPACK Type-80 tangential forces from the rail end to the ORVD
canonical wheel end, and writes a compact binary64 archive.  It does not search
for authority data and it does not interpolate, shift, filter, or rescale any
series.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
from pathlib import Path
import sys
from types import ModuleType
from typing import Iterable

import numpy as np


SAMPLE_COUNT = 32_001
SAMPLE_PERIOD_SECONDS = 0.0005
AXLES = ("ff", "fr", "rf", "rr")
AXLE_PATH_PARTS = (("F", "F"), ("F", "R"), ("R", "F"), ("R", "R"))
SIDES = (("l", "Left", "L"), ("r", "Right", "R"))
P047_SOURCE_SBR_SHA256 = (
    "f8673e57c5956467c81c3ba95a677f4efc6422e69b1aa7fc07848726b8bb4b68"
)
EXPECTED_CHANNEL_COUNT = 5_162


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def load_sbr_module(wrl_root: Path) -> ModuleType:
    source = wrl_root / "scripts_python" / "afs_bench" / "simpack_sbr.py"
    if not source.is_file():
        raise FileNotFoundError(f"wheel-rail-lab SBR reader is absent: {source}")
    spec = importlib.util.spec_from_file_location("g63_simpack_sbr", source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load wheel-rail-lab SBR reader: {source}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wheel-rail-lab-root", type=Path, required=True)
    parser.add_argument("--p047-sbr", type=Path, required=True)
    parser.add_argument("--output-npz", type=Path, required=True)
    return parser.parse_args(list(argv))


def require_channel(
    channels: dict[str, object],
    path: str,
    description: str,
    roles: list[str],
    paths: list[str],
    descriptions: list[str],
    role: str,
) -> np.ndarray:
    channel = channels.get(path)
    if channel is None:
        raise KeyError(f"P047 SBR is missing channel: {path}")
    if channel.description != description:
        raise ValueError(
            f"P047 channel {path} has description {channel.description!r}; "
            f"expected {description!r}"
        )
    values = np.asarray(channel.values, dtype=np.float64)
    if values.shape != (SAMPLE_COUNT,) or not np.isfinite(values).all():
        raise ValueError(f"P047 channel {path} has invalid values")
    roles.append(role)
    paths.append(path)
    descriptions.append(description)
    return np.array(values, copy=True)


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    source = arguments.p047_sbr.resolve()
    output = arguments.output_npz.resolve()
    if output.exists():
        print(
            f"G63 extraction failed: output already exists: {output}", file=sys.stderr
        )
        return 1

    try:
        source_sha256 = sha256_file(source)
        if source_sha256 != P047_SOURCE_SBR_SHA256:
            raise ValueError("source SBR is not the frozen P047 binary64 authority")
        module = load_sbr_module(arguments.wheel_rail_lab_root.resolve())
        result = module.read_sbr_scalar_result(source, require_double=True)
        if (result.precision_bytes, result.storage_bytes) != (8, 8):
            raise ValueError("P047 SBR is not stored and reported as binary64")
        if len(result.channels) != EXPECTED_CHANNEL_COUNT:
            raise ValueError(
                f"P047 channel count is {len(result.channels)}, "
                f"expected {EXPECTED_CHANNEL_COUNT}"
            )
        channels = result.by_path()
        time = np.asarray(channels["time"].values, dtype=np.float64)
        expected_time = np.arange(SAMPLE_COUNT, dtype=np.float64) * np.float64(
            SAMPLE_PERIOD_SECONDS
        )
        if not np.array_equal(time, expected_time):
            raise ValueError("P047 time is not the exact 0--16 s / 0.5 ms clock")

        roles: list[str] = []
        selected_paths: list[str] = []
        selected_descriptions: list[str] = []
        macro_station = []
        macro_joint_lateral = []
        macro_joint_yaw = []
        macro_lateral_irregularity = []
        contact_station = []
        contact_patch_count = []
        contact_q = []
        contact_n = []
        contact_tx = []
        contact_ty = []
        wheel_names = []

        for axle, (bogie, wheelset) in zip(AXLES, AXLE_PATH_PARTS, strict=True):
            joint = f"jointPos.S_BF_{bogie}__S_WS_{wheelset}__J_wheelset"
            top = (
                f"RS_result.RS_S_BF_{bogie}.RS_S_WS_{wheelset}."
                f"S_BF_{bogie}__S_WS_{wheelset}"
            )
            macro_station.append(
                require_channel(
                    channels,
                    f"{top}__RS_RWT_track.ch_003",
                    "s Position along track",
                    roles,
                    selected_paths,
                    selected_descriptions,
                    f"{axle}.macro_station",
                )
            )
            macro_lateral_irregularity.append(
                require_channel(
                    channels,
                    f"{top}__RS_RWT_track.ch_007",
                    "y_irreg Lateral track excitation",
                    roles,
                    selected_paths,
                    selected_descriptions,
                    f"{axle}.macro_lateral_irregularity",
                )
            )
            macro_joint_lateral.append(
                require_channel(
                    channels,
                    f"{joint}.st_002",
                    "y  : Lateral position",
                    roles,
                    selected_paths,
                    selected_descriptions,
                    f"{axle}.joint_lateral",
                )
            )
            macro_joint_yaw.append(
                require_channel(
                    channels,
                    f"{joint}.st_005",
                    "psi: Yaw angle",
                    roles,
                    selected_paths,
                    selected_descriptions,
                    f"{axle}.joint_yaw",
                )
            )

            for side, side_word, side_letter in SIDES:
                wheel_name = f"{axle}_{side}"
                wheel_names.append(wheel_name)
                pair = (
                    f"{top}__RS_RWP_WS_{side_word}."
                    f"S_BF_{bogie}__S_WS_{wheelset}__RS_RWP_{side_letter}_Pair"
                )
                patch = (
                    f"{top}__RS_RWP_WS_{side_word}."
                    f"S_BF_{bogie}__S_WS_{wheelset}__RS_RWP_{side_letter}_1"
                )
                contact_q.append(
                    require_channel(
                        channels,
                        f"{pair}.ch_001",
                        "Q Vertical wheel force",
                        roles,
                        selected_paths,
                        selected_descriptions,
                        f"{wheel_name}.Q",
                    )
                )
                contact_station.append(
                    require_channel(
                        channels,
                        f"{pair}.ch_008",
                        "s Position along track",
                        roles,
                        selected_paths,
                        selected_descriptions,
                        f"{wheel_name}.station",
                    )
                )
                contact_patch_count.append(
                    require_channel(
                        channels,
                        f"{pair}.ch_022",
                        "np Number of contact patches",
                        roles,
                        selected_paths,
                        selected_descriptions,
                        f"{wheel_name}.patch_count",
                    )
                )
                contact_n.append(
                    require_channel(
                        channels,
                        f"{patch}.ch_010",
                        "N Normal force",
                        roles,
                        selected_paths,
                        selected_descriptions,
                        f"{wheel_name}.N",
                    )
                )
                # SIMPACK Type-80 reports Tx/Ty on the rail end.  ORVD's
                # canonical scalars act on the wheel, so negate exactly once.
                contact_tx.append(
                    -require_channel(
                        channels,
                        f"{patch}.ch_012",
                        "Tx Longitudinal creep force",
                        roles,
                        selected_paths,
                        selected_descriptions,
                        f"{wheel_name}.Tx_rail_end",
                    )
                )
                contact_ty.append(
                    -require_channel(
                        channels,
                        f"{patch}.ch_013",
                        "Ty Lateral creep force",
                        roles,
                        selected_paths,
                        selected_descriptions,
                        f"{wheel_name}.Ty_rail_end",
                    )
                )

        macro_station_array = np.column_stack(macro_station)
        contact_station_array = np.column_stack(contact_station)
        patch_count_array = np.column_stack(contact_patch_count)
        if not np.all(np.diff(macro_station_array, axis=0) > 0.0):
            raise ValueError("P047 macro stations are not strictly increasing")
        if not np.all(np.diff(contact_station_array, axis=0) > 0.0):
            raise ValueError(
                "P047 per-wheel contact stations are not strictly increasing"
            )
        if not np.array_equal(patch_count_array, np.ones_like(patch_count_array)):
            raise ValueError(
                "P047 qualified run is not eight-wheel single-patch throughout"
            )

        output.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            output,
            time_seconds=np.array(time, copy=True),
            axle_names=np.asarray(AXLES),
            wheel_names=np.asarray(wheel_names),
            macro_station_meters=macro_station_array,
            macro_joint_lateral_meters=np.column_stack(macro_joint_lateral),
            macro_joint_yaw_radians=np.column_stack(macro_joint_yaw),
            macro_lateral_irregularity_meters=np.column_stack(
                macro_lateral_irregularity
            ),
            contact_station_meters=contact_station_array,
            contact_patch_count=patch_count_array.astype(np.int64),
            contact_Q_newtons=np.column_stack(contact_q),
            contact_N_newtons=np.column_stack(contact_n),
            contact_Tx_on_wheel_newtons=np.column_stack(contact_tx),
            contact_Ty_on_wheel_newtons=np.column_stack(contact_ty),
            channel_roles=np.asarray(roles),
            channel_paths=np.asarray(selected_paths),
            channel_descriptions=np.asarray(selected_descriptions),
            source_sbr_sha256=np.asarray(source_sha256),
            source_sbr_precision_bytes=np.asarray(result.precision_bytes),
            source_sbr_channel_count=np.asarray(len(result.channels)),
        )
    except (FileNotFoundError, KeyError, OSError, RuntimeError, ValueError) as error:
        print(f"G63 extraction failed: {error}", file=sys.stderr)
        return 1

    print(f"wrote P047 G63 authority to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
