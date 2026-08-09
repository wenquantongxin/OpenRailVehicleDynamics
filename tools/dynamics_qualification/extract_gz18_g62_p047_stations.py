#!/usr/bin/env python3
"""Extract the four P047 Result Element 82 station channels from binary64 SBR.

The SBR reader remains owned by wheel-rail-lab.  This migration-only tool is
given both repositories and the source SBR explicitly; it neither searches for
an authority file nor passes the values through SIMPACK Post.
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
CHANNEL_PATHS = (
    "RS_result.RS_S_BF_F.RS_S_WS_F."
    "S_BF_F__S_WS_F__RS_RWT_track.ch_003",
    "RS_result.RS_S_BF_F.RS_S_WS_R."
    "S_BF_F__S_WS_R__RS_RWT_track.ch_003",
    "RS_result.RS_S_BF_R.RS_S_WS_F."
    "S_BF_R__S_WS_F__RS_RWT_track.ch_003",
    "RS_result.RS_S_BF_R.RS_S_WS_R."
    "S_BF_R__S_WS_R__RS_RWT_track.ch_003",
)
EXPECTED_DESCRIPTION = "s Position along track"
P047_SOURCE_SBR_SHA256 = (
    "f8673e57c5956467c81c3ba95a677f4efc6422e69b1aa7fc07848726b8bb4b68"
)


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
    spec = importlib.util.spec_from_file_location("g62_simpack_sbr", source)
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


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    source = arguments.p047_sbr.resolve()
    output = arguments.output_npz.resolve()
    if output.exists():
        print(f"G62 extraction failed: output already exists: {output}", file=sys.stderr)
        return 1
    try:
        source_sha256 = sha256_file(source)
        if source_sha256 != P047_SOURCE_SBR_SHA256:
            raise ValueError("source SBR is not the frozen P047 binary64 authority")
        module = load_sbr_module(arguments.wheel_rail_lab_root.resolve())
        result = module.read_sbr_scalar_result(source, require_double=True)
        if result.precision_bytes != 8 or result.storage_bytes != 8:
            raise ValueError("P047 SBR is not stored and reported as binary64")
        channels = result.by_path()
        time = np.array(channels["time"].values, dtype=np.float64, copy=True)
        expected_time = np.arange(SAMPLE_COUNT, dtype=np.float64) * np.float64(
            SAMPLE_PERIOD_SECONDS
        )
        if time.shape != (SAMPLE_COUNT,) or not np.array_equal(time, expected_time):
            raise ValueError("P047 time is not the exact 0--16 s / 0.5 ms clock")
        selected = []
        descriptions = []
        for path in CHANNEL_PATHS:
            channel = channels.get(path)
            if channel is None:
                raise KeyError(f"P047 SBR is missing Result Element 82 channel: {path}")
            if channel.description != EXPECTED_DESCRIPTION:
                raise ValueError(
                    f"P047 channel {path} has description {channel.description!r}"
                )
            values = np.array(channel.values, dtype=np.float64, copy=True)
            if values.shape != time.shape or not np.isfinite(values).all():
                raise ValueError(f"P047 channel {path} has invalid values")
            if not np.all(np.diff(values) > 0.0):
                raise ValueError(f"P047 channel {path} is not strictly increasing")
            selected.append(values)
            descriptions.append(channel.description)
        output.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            output,
            time_seconds=time,
            axle_names=np.asarray(AXLES),
            station_meters=np.column_stack(selected),
            channel_paths=np.asarray(CHANNEL_PATHS),
            channel_descriptions=np.asarray(descriptions),
            source_sbr_sha256=np.asarray(source_sha256),
            source_sbr_precision_bytes=np.asarray(result.precision_bytes),
        )
    except (FileNotFoundError, KeyError, OSError, RuntimeError, ValueError) as error:
        print(f"G62 extraction failed: {error}", file=sys.stderr)
        return 1
    print(f"wrote P047 Result Element 82 stations to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
