#!/usr/bin/env python3
"""Prepare a temporary model and run one single-curve guidance arm."""

from __future__ import annotations

import argparse
import math
import re
import shutil
import subprocess
import tempfile
from pathlib import Path


FIXED_ASSIGNMENTS = {
    "vehicle.applystartvel": "1",
    "slv.integ.type": "5",
    "slv.integ.meetop": "1",
    "slv.integ.fix.h": "1.00000000000000002E-03",
    "slv.integ.tend.time": "{ 45 s }",
    "slv.rt.enable": "1",
    "slv.rt.stepsize": "1.00000000000000002E-03",
    "slv.rt.log.rate": "0",
}

HISTORICAL_AAR5_EXCITATION_ASSIGNMENTS = {
    ("subvar.str", None, "$_AAR5IrregStart"): "'50'",
    ("subvar.str", None, "$_AAR5IrregEnd"): "'3000'",
    ("subvar.str", None, "$_AAR5IrregFade"): "'50'",
    ("excit.par", 2, "$E_AAR5_LAT"): "2.00000000000000000E+00",
    ("excit.par", 5, "$E_AAR5_LAT"): "6.56000000000000028E-01",
    ("excit.par", 6, "$E_AAR5_LAT"): "2.40000000000000005E-02",
    ("excit.par", 2, "$E_AAR5_VER"): "2.00000000000000000E+00",
    ("excit.par", 5, "$E_AAR5_VER"): "6.56000000000000028E-01",
    ("excit.par", 6, "$E_AAR5_VER"): "2.40000000000000005E-02",
}

SPCK_INDEXED_ASSIGNMENT = re.compile(
    r"^\s*(?P<property>[a-z.]+)\s*\(\s*"
    r"(?:(?P<index>\d+)\s*,\s*)?"
    r"(?P<object>\$[_A-Za-z0-9]+)\s*\)\s*="
)


def assignment_property(line: str, assignments: dict[str, str]) -> str | None:
    stripped = line.lstrip()
    for property_name in assignments:
        if not stripped.startswith(property_name):
            continue
        suffix = stripped[len(property_name) : len(property_name) + 1]
        if suffix in {" ", "\t", "("}:
            return property_name
    return None


def replace_assignment_value(line: str, value: str) -> str:
    equals = line.find("=")
    if equals < 0:
        raise RuntimeError("malformed SPCK assignment")
    comment = line.find("!", equals + 1)
    suffix = "" if comment < 0 else " " + line[comment:].lstrip()
    return line[: equals + 1] + " " + value + suffix


def historical_aar5_assignment_key(
    line: str,
) -> tuple[str, int | None, str] | None:
    match = SPCK_INDEXED_ASSIGNMENT.match(line)
    if match is None:
        return None
    index_text = match.group("index")
    key = (
        match.group("property"),
        None if index_text is None else int(index_text),
        match.group("object"),
    )
    return key if key in HISTORICAL_AAR5_EXCITATION_ASSIGNMENTS else None


def prepare_model(
    source: Path,
    destination: Path,
    active_track: str,
    initial_speed_kilometres_per_hour: float,
    use_historical_aar5_excitation: bool,
) -> None:
    assignments = {
        **FIXED_ASSIGNMENTS,
        "vehicle.startvel":
            f"{{ {initial_speed_kilometres_per_hour:.17g}/3.6 }}",
        "track.active": active_track,
    }
    lines = source.read_text(encoding="utf-8").splitlines()
    if not any(active_track in line for line in lines):
        raise RuntimeError(f"SIMPACK model does not define Track {active_track}")
    counts = {name: 0 for name in assignments}
    historical_counts = {
        key: 0 for key in HISTORICAL_AAR5_EXCITATION_ASSIGNMENTS
    }
    transformed: list[str] = []
    for line in lines:
        historical_key = (
            historical_aar5_assignment_key(line)
            if use_historical_aar5_excitation
            else None
        )
        if historical_key is not None:
            transformed.append(
                replace_assignment_value(
                    line,
                    HISTORICAL_AAR5_EXCITATION_ASSIGNMENTS[historical_key],
                )
            )
            historical_counts[historical_key] += 1
            continue
        property_name = assignment_property(line, assignments)
        if property_name is None:
            transformed.append(line)
            continue
        transformed.append(replace_assignment_value(line, assignments[property_name]))
        counts[property_name] += 1
    invalid = [name for name, count in counts.items() if count != 1]
    if invalid:
        raise RuntimeError(
            "expected exactly one assignment for: " + ", ".join(invalid)
        )
    invalid_historical = [
        key for key, count in historical_counts.items() if count != 1
    ]
    if use_historical_aar5_excitation and invalid_historical:
        raise RuntimeError(
            "expected exactly one historical AAR5 assignment for: "
            + ", ".join(str(key) for key in invalid_historical)
        )
    destination.write_text("\n".join(transformed) + "\n", encoding="utf-8")


def generated_sidecars(model: Path) -> tuple[Path, ...]:
    return (
        model.with_suffix(".output"),
        Path(str(model) + ".output"),
        model.with_suffix(".sbr"),
        Path(str(model) + ".sbr"),
    )


def remove_generated_path(path: Path) -> None:
    if path.is_dir():
        shutil.rmtree(path)
    elif path.exists():
        path.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run one IRW single-curve guidance "
            "SIMPACK Realtime comparison arm without changing the reference model."
        )
    )
    parser.add_argument("runner", type=Path)
    parser.add_argument("reference_model", type=Path)
    parser.add_argument("torque_conditioner", type=Path)
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("--active-track", required=True)
    parser.add_argument(
        "--initial-speed-kilometres-per-hour", required=True, type=float
    )
    parser.add_argument(
        "--historical-aar5-excitation",
        action="store_true",
        help=(
            "use the historical coupled AAR5 excitation: lateral and vertical "
            "seed 2, 0.024--0.656 cycles/m, start/end/fade 50/3000/50 m"
        ),
    )
    arguments = parser.parse_args()

    runner = arguments.runner.resolve(strict=True)
    reference_model = arguments.reference_model.resolve(strict=True)
    conditioner = arguments.torque_conditioner.resolve(strict=True)
    if (
        not arguments.active_track.startswith("$Trk_")
        or not math.isfinite(arguments.initial_speed_kilometres_per_hour)
        or arguments.initial_speed_kilometres_per_hour <= 0.0
    ):
        raise RuntimeError("active Track or initial speed is invalid")
    output_directory = arguments.output_directory.absolute()
    if output_directory.exists():
        raise RuntimeError(f"output directory already exists: {output_directory}")
    if not output_directory.parent.is_dir():
        raise RuntimeError(
            f"output parent is not an existing directory: {output_directory.parent}"
        )

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".orvd_single_curve_guidance_",
        suffix=".spck",
        dir=reference_model.parent,
    )
    temporary_model = Path(temporary_name)
    try:
        # Close the descriptor before the Realtime loader opens the model.
        import os

        os.close(descriptor)
        prepare_model(
            reference_model,
            temporary_model,
            arguments.active_track,
            arguments.initial_speed_kilometres_per_hour,
            arguments.historical_aar5_excitation,
        )
        completed = subprocess.run(
            [
                str(runner),
                str(temporary_model),
                str(conditioner),
                str(output_directory),
            ],
            check=False,
        )
        return completed.returncode
    finally:
        remove_generated_path(temporary_model)
        for sidecar in generated_sidecars(temporary_model):
            remove_generated_path(sidecar)


if __name__ == "__main__":
    raise SystemExit(main())
