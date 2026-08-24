#!/usr/bin/env python3
"""Run one qualification process and publish its external execution identity."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import runpy
import subprocess
import sys
import time
from typing import Iterable, Mapping

try:
    import resource as posix_resource
except ImportError:  # Windows keeps layout parsing importable for CTest.
    posix_resource = None


RUNNER_LAYOUTS = {
    "gz18": {8: 5, 9: 5},
    "irw-passive-scenario": {9: 6, 10: 6},
    "irw-r300-aar5-v60-100hz-full-state-guidance": {8: 6, 9: 6},
}

RUNNER_EXECUTABLE_NAMES = {
    "gz18": "orvd_gz18_dynamics_qualification",
    "irw-passive-scenario": "orvd_irw_passive_scenario",
    "irw-r300-aar5-v60-100hz-full-state-guidance": (
        "orvd_irw_r300_aar5_v60_100hz_full_state_guidance"
    ),
}

SERIAL_OPENMP_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OMP_DYNAMIC": "FALSE",
    "OMP_MAX_ACTIVE_LEVELS": "1",
    "OMP_NESTED": "FALSE",
}

TIME_COLUMNS = ("sample_index", "time_nanoseconds", "time_seconds")

CONTACT_PATCH_COLUMNS = (
    *TIME_COLUMNS,
    "interface_name",
    "patch_ordinal",
    "normal_force_newtons",
    "longitudinal_force_on_wheel_in_contact_frame_newtons",
    "lateral_force_on_wheel_in_contact_frame_newtons",
    "contact_frame_angle_radians",
    "contact_point_in_carrier_track_frame_x_meters",
    "contact_point_in_carrier_track_frame_y_meters",
    "contact_point_in_carrier_track_frame_z_meters",
    "force_on_wheel_in_carrier_track_frame_x_newtons",
    "force_on_wheel_in_carrier_track_frame_y_newtons",
    "force_on_wheel_in_carrier_track_frame_z_newtons",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def parse_affinity(text: str) -> set[int]:
    result: set[int] = set()
    for item in text.split(","):
        fields = item.strip().split("-")
        if len(fields) == 1:
            begin = end = int(fields[0])
        elif len(fields) == 2:
            begin, end = (int(field) for field in fields)
        else:
            raise ValueError(f"invalid CPU-affinity item {item!r}")
        if begin < 0 or end < begin:
            raise ValueError(f"invalid CPU-affinity range {item!r}")
        result.update(range(begin, end + 1))
    if not result:
        raise ValueError("CPU affinity is empty")
    return result


def processor_identity() -> str:
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.is_file():
        for line in cpuinfo.read_text(encoding="utf-8").splitlines():
            if line.startswith("model name") and ":" in line:
                return line.split(":", 1)[1].strip()
    return platform.processor() or platform.machine()


def write_identity(path: Path, value: dict[str, object]) -> None:
    if path.exists():
        raise FileExistsError(f"execution identity already exists: {path}")
    partial = path.with_name(path.name + ".partial")
    if partial.exists():
        raise FileExistsError(f"execution identity partial exists: {partial}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with partial.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
    partial.rename(path)


def _load_comparison_namespace() -> dict[str, object]:
    comparison_path = Path(__file__).resolve().with_name(
        "qualification_comparison.py"
    )
    return runpy.run_path(str(comparison_path))


def _comparison_options_are_complete(arguments: argparse.Namespace) -> bool:
    values = (
        arguments.comparison_manifest,
        arguments.comparison_scenario,
        arguments.comparison_case,
    )
    return all(value is not None for value in values)


def prepare_manifest_bound_execution(
    arguments: argparse.Namespace,
    environment: Mapping[str, str],
) -> dict[str, object] | None:
    """Validate and bind one manifest run before affinity or subprocess use."""

    if not _comparison_options_are_complete(arguments):
        return None
    namespace = _load_comparison_namespace()
    source_root = (
        arguments.comparison_source_root
        if arguments.comparison_source_root is not None
        else Path(__file__).resolve().parents[2]
    ).resolve(strict=True)
    manifest_path = arguments.comparison_manifest.resolve(strict=True)
    manifest = namespace["load_manifest"](  # type: ignore[operator]
        manifest_path, source_root=source_root
    )
    scenario = next(
        (
            entry
            for entry in manifest["scenarios"]
            if entry["identifier"] == arguments.comparison_scenario
        ),
        None,
    )
    if scenario is None:
        raise ValueError(
            f"unknown manifest scenario {arguments.comparison_scenario!r}"
        )
    if scenario["runner_layout"] != arguments.vehicle_recipe:
        raise ValueError(
            "comparison scenario runner layout does not match --vehicle-recipe"
        )
    if arguments.executable.name != RUNNER_EXECUTABLE_NAMES[arguments.vehicle_recipe]:
        raise ValueError(
            "qualification executable name does not match the scenario runner"
        )
    execution = manifest["required_execution"]
    if arguments.build_type != execution["build_type"]:
        raise ValueError("wrapper build type does not match the manifest")
    requested_affinity = parse_affinity(arguments.cpu_affinity)
    if len(requested_affinity) != execution["cpu_affinity_core_count"]:
        raise ValueError("CPU affinity core count does not match the manifest")

    output_index = RUNNER_LAYOUTS[arguments.vehicle_recipe][
        len(arguments.runner_arguments)
    ]
    expected_arguments = namespace["materialize_runner_arguments"](  # type: ignore[operator]
        manifest,
        scenario_identifier=arguments.comparison_scenario,
        qualification_case_identifier=arguments.comparison_case,
        source_root=source_root,
        output_directory=arguments.runner_arguments[output_index],
    )
    if arguments.runner_arguments != expected_arguments:
        raise ValueError(
            "runner arguments do not exactly match the manifest materialization"
        )

    child_environment = dict(environment)
    child_environment.update(SERIAL_OPENMP_ENVIRONMENT)
    case = next(
        entry
        for entry in manifest["qualification_cases"]
        if entry["identifier"] == arguments.comparison_case
    )
    return {
        "manifest": manifest,
        "scenario": scenario,
        "case": case,
        "source_root": source_root,
        "expected_compiler_identity": arguments.compiler_identity,
        "child_environment": child_environment,
        "identity": {
            "manifest_identifier": manifest["manifest_identifier"],
            "manifest_schema_version": manifest["schema_version"],
            "canonical_manifest_path": str(manifest_path),
            "source_root": str(source_root),
            "scenario_identifier": arguments.comparison_scenario,
            "qualification_case_identifier": arguments.comparison_case,
            "validation_status": "preflight_passed",
            "validation_errors": [],
        },
    }


def _load_json_object(path: Path) -> dict[str, object]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-finite JSON constant {value}")

    def reject_duplicate_keys(
        pairs: list[tuple[str, object]],
    ) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key {key!r}")
            result[key] = value
        return result

    with path.open("r", encoding="utf-8") as stream:
        value = json.load(
            stream,
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    if type(value) is not dict:
        raise ValueError(f"{path.name} is not a JSON object")
    return value


def _read_tsv_header(stream: object, path: Path) -> list[str]:
    line = stream.readline()  # type: ignore[attr-defined]
    if not line:
        raise ValueError(f"{path.name} has no header")
    if not line.endswith("\n"):
        raise ValueError(f"{path.name} header is truncated")
    return line[:-1].split("\t")


def _require_finite_float(text: str, detail: str) -> None:
    try:
        value = float(text)
    except ValueError as error:
        raise ValueError(f"{detail} is not numeric") from error
    if not math.isfinite(value):
        raise ValueError(f"{detail} is not finite")


def _require_clock_row(
    fields: list[str],
    *,
    expected_sample_index: int,
    sample_period_nanoseconds: int,
    path: Path,
    line_number: int,
) -> None:
    try:
        sample_index = int(fields[0])
        time_nanoseconds = int(fields[1])
    except ValueError as error:
        raise ValueError(
            f"{path.name}:{line_number} has a non-integer join key"
        ) from error
    if sample_index != expected_sample_index:
        raise ValueError(
            f"{path.name}:{line_number} sample_index is not the frozen sequence"
        )
    if time_nanoseconds != sample_index * sample_period_nanoseconds:
        raise ValueError(
            f"{path.name}:{line_number} time_nanoseconds is off the frozen clock"
        )
    _require_finite_float(
        fields[2], f"{path.name}:{line_number} time_seconds"
    )


def _validate_dense_tsv(
    path: Path,
    *,
    expected_header: tuple[str, ...],
    expected_sample_count: int,
    sample_period_nanoseconds: int,
) -> None:
    with path.open("r", encoding="utf-8") as stream:
        header = _read_tsv_header(stream, path)
        if tuple(header) != expected_header:
            raise ValueError(f"{path.name} header drifted")
        observed_rows = 0
        for observed_rows, line in enumerate(stream, start=1):
            line_number = observed_rows + 1
            if not line.endswith("\n"):
                raise ValueError(f"{path.name}:{line_number} is truncated")
            fields = line[:-1].split("\t")
            if len(fields) != len(header):
                raise ValueError(
                    f"{path.name}:{line_number} column count drifted"
                )
            _require_clock_row(
                fields,
                expected_sample_index=observed_rows - 1,
                sample_period_nanoseconds=sample_period_nanoseconds,
                path=path,
                line_number=line_number,
            )
            for column_index, text in enumerate(fields[3:], start=3):
                _require_finite_float(
                    text,
                    f"{path.name}:{line_number} column {header[column_index]}",
                )
        if observed_rows != expected_sample_count:
            raise ValueError(
                f"{path.name} has {observed_rows} rows; "
                f"expected {expected_sample_count}"
            )


def _validate_observations_tsv(
    path: Path, *, expected_sample_count: int, sample_period_nanoseconds: int
) -> tuple[tuple[str, ...], list[tuple[int, ...]]]:
    with path.open("r", encoding="utf-8") as stream:
        header = _read_tsv_header(stream, path)
        if tuple(header[:3]) != TIME_COLUMNS or len(header) <= 3:
            raise ValueError(f"{path.name} has an invalid observation header")
        if len(set(header)) != len(header):
            raise ValueError(f"{path.name} contains duplicate columns")
        patch_count_columns = tuple(
            (index, name.removesuffix(".contact_patch_count"))
            for index, name in enumerate(header)
            if name.endswith(".contact_patch_count")
        )
        if not patch_count_columns:
            raise ValueError(f"{path.name} has no contact-patch count columns")
        patch_count_indices = {
            column_index for column_index, _ in patch_count_columns
        }
        observed_rows = 0
        expected_patch_counts: list[tuple[int, ...]] = []
        for observed_rows, line in enumerate(stream, start=1):
            line_number = observed_rows + 1
            if not line.endswith("\n"):
                raise ValueError(f"{path.name}:{line_number} is truncated")
            fields = line[:-1].split("\t")
            if len(fields) != len(header):
                raise ValueError(
                    f"{path.name}:{line_number} column count drifted"
                )
            _require_clock_row(
                fields,
                expected_sample_index=observed_rows - 1,
                sample_period_nanoseconds=sample_period_nanoseconds,
                path=path,
                line_number=line_number,
            )
            sample_patch_counts: list[int] = []
            for column_index, text in enumerate(fields[3:], start=3):
                if column_index in patch_count_indices:
                    try:
                        patch_count = int(text)
                    except ValueError as error:
                        raise ValueError(
                            f"{path.name}:{line_number} column "
                            f"{header[column_index]} is not an integer"
                        ) from error
                    if patch_count < 0:
                        raise ValueError(
                            f"{path.name}:{line_number} column "
                            f"{header[column_index]} is negative"
                        )
                    sample_patch_counts.append(patch_count)
                    continue
                _require_finite_float(
                    text,
                    f"{path.name}:{line_number} column {header[column_index]}",
                )
            expected_patch_counts.append(tuple(sample_patch_counts))
        if observed_rows != expected_sample_count:
            raise ValueError(
                f"{path.name} has {observed_rows} rows; "
                f"expected {expected_sample_count}"
            )
        return (
            tuple(interface_name for _, interface_name in patch_count_columns),
            expected_patch_counts,
        )


def _validate_contact_patches_tsv(
    path: Path,
    *,
    expected_sample_count: int,
    expected_interface_names: tuple[str, ...],
    expected_patch_counts: list[tuple[int, ...]],
    sample_period_nanoseconds: int,
) -> None:
    with path.open("r", encoding="utf-8") as stream:
        header = _read_tsv_header(stream, path)
        if tuple(header) != CONTACT_PATCH_COLUMNS:
            raise ValueError(f"{path.name} header drifted")
        expected_keys = (
            (sample_index, interface_name, patch_ordinal)
            for sample_index, counts in enumerate(expected_patch_counts)
            for interface_name, patch_count in zip(
                expected_interface_names, counts, strict=True
            )
            for patch_ordinal in range(patch_count)
        )
        for line_number, line in enumerate(stream, start=2):
            if not line.endswith("\n"):
                raise ValueError(f"{path.name}:{line_number} is truncated")
            fields = line[:-1].split("\t")
            if len(fields) != len(header):
                raise ValueError(
                    f"{path.name}:{line_number} column count drifted"
                )
            try:
                sample_index = int(fields[0])
                time_nanoseconds = int(fields[1])
            except ValueError as error:
                raise ValueError(
                    f"{path.name}:{line_number} has a non-integer join key"
                ) from error
            if not 0 <= sample_index < expected_sample_count:
                raise ValueError(
                    f"{path.name}:{line_number} sample_index is out of range"
                )
            if time_nanoseconds != sample_index * sample_period_nanoseconds:
                raise ValueError(
                    f"{path.name}:{line_number} time_nanoseconds is off the frozen clock"
                )
            if not fields[3]:
                raise ValueError(
                    f"{path.name}:{line_number} has an empty interface_name"
                )
            try:
                patch_ordinal = int(fields[4])
            except ValueError as error:
                raise ValueError(
                    f"{path.name}:{line_number} has a non-integer patch_ordinal"
                ) from error
            if patch_ordinal < 0:
                raise ValueError(
                    f"{path.name}:{line_number} has a negative patch_ordinal"
                )
            try:
                expected_key = next(expected_keys)
            except StopIteration as error:
                raise ValueError(
                    f"{path.name}:{line_number} contains an undeclared patch"
                ) from error
            observed_key = (sample_index, fields[3], patch_ordinal)
            if observed_key != expected_key:
                raise ValueError(
                    f"{path.name}:{line_number} key {observed_key!r} "
                    f"does not match observations.tsv declaration {expected_key!r}"
                )
            _require_finite_float(
                fields[2], f"{path.name}:{line_number} time_seconds"
            )
            for column_index, text in enumerate(fields[5:], start=5):
                _require_finite_float(
                    text,
                    f"{path.name}:{line_number} column {header[column_index]}",
                )
        try:
            missing_key = next(expected_keys)
        except StopIteration:
            missing_key = None
        if missing_key is not None:
            raise ValueError(
                f"{path.name} is missing observations.tsv-declared patch "
                f"{missing_key!r}"
            )


def _validate_completion_and_tables(
    artifact_directory: Path,
    *,
    layout: Mapping[str, object],
    expected_sample_count: int,
    sample_period_nanoseconds: int,
) -> list[str]:
    errors: list[str] = []
    complete_path = artifact_directory / "COMPLETE"
    try:
        complete = complete_path.read_text(encoding="utf-8")
        if complete != f"{expected_sample_count} samples\n":
            errors.append("COMPLETE marker does not match the frozen sample count")
    except (OSError, UnicodeError) as error:
        errors.append(f"could not read COMPLETE marker: {error}")

    nq = int(layout["generalized_position_count"])
    nv = int(layout["generalized_velocity_count"])
    nz = int(layout["series_force_state_count"])
    continuous_header = (
        *TIME_COLUMNS,
        *(f"q.{index}" for index in range(nq)),
        *(f"v.{index}" for index in range(nv)),
        *(f"z.{index}" for index in range(nz)),
    )
    try:
        _validate_dense_tsv(
            artifact_directory / "continuous_states.tsv",
            expected_header=continuous_header,
            expected_sample_count=expected_sample_count,
            sample_period_nanoseconds=sample_period_nanoseconds,
        )
    except (OSError, UnicodeError, ValueError) as error:
        errors.append(str(error))

    expected_patch_layout: tuple[
        tuple[str, ...], list[tuple[int, ...]]
    ] | None = None
    try:
        expected_patch_layout = _validate_observations_tsv(
            artifact_directory / "observations.tsv",
            expected_sample_count=expected_sample_count,
            sample_period_nanoseconds=sample_period_nanoseconds,
        )
    except (OSError, UnicodeError, ValueError) as error:
        errors.append(str(error))

    if expected_patch_layout is not None:
        expected_interface_names, expected_patch_counts = expected_patch_layout
        try:
            _validate_contact_patches_tsv(
                artifact_directory / "contact_patches.tsv",
                expected_sample_count=expected_sample_count,
                expected_interface_names=expected_interface_names,
                expected_patch_counts=expected_patch_counts,
                sample_period_nanoseconds=sample_period_nanoseconds,
            )
        except (OSError, UnicodeError, ValueError) as error:
            errors.append(str(error))
    return errors


def validate_manifest_bound_artifact(
    artifact_directory: Path, binding: Mapping[str, object]
) -> list[str]:
    """Return every postflight identity mismatch for one completed run."""

    errors: list[str] = []

    def require(condition: bool, detail: str) -> None:
        if not condition:
            errors.append(detail)

    required_files = (
        "COMPLETE",
        "metadata.json",
        "continuous_states.tsv",
        "observations.tsv",
        "contact_patches.tsv",
        "performance.json",
    )
    for filename in required_files:
        require(
            (artifact_directory / filename).is_file(),
            f"missing qualification artifact {filename}",
        )
    if errors:
        return errors

    try:
        metadata = _load_json_object(artifact_directory / "metadata.json")
        performance = _load_json_object(artifact_directory / "performance.json")
    except (OSError, json.JSONDecodeError, ValueError) as error:
        return [f"could not parse qualification identity artifacts: {error}"]

    def nested_object(parent: Mapping[str, object], key: str) -> dict[str, object]:
        value = parent.get(key)
        return value if type(value) is dict else {}

    manifest = binding["manifest"]
    execution = manifest["required_execution"]
    scenario = binding["scenario"]
    case = binding["case"]
    source_root = binding["source_root"]
    clock = scenario["clock"]
    layout = scenario["state_layout"]
    default_numerics = scenario["scenario_default_numerics"]
    scale = case["scale"]
    expected_recipe = (
        default_numerics["integrator_recipe_identifier"]
        if case["backend"] == "scenario_default_cvode"
        else "radau5"
    )
    expected_inputs = {
        key: str((source_root / relative).resolve(strict=True))
        for key, relative in scenario["inputs"].items()
        if key != "track_irregularity_identifier"
    }

    require(metadata.get("completed") is True, "metadata is not completed")
    require(
        metadata.get("artifact_schema_identifier")
        == "orvd.passive_vehicle_qualification.v2",
        "metadata artifact schema identifier drifted",
    )
    require(metadata.get("input_paths") == expected_inputs, "input paths drifted")
    continuous_state_contract = nested_object(
        metadata, "continuous_state_observation_contract"
    )
    require(
        continuous_state_contract.get("file") == "continuous_states.tsv"
        and continuous_state_contract.get("row_join_key")
        == ["sample_index", "time_nanoseconds"]
        and continuous_state_contract.get("time_seconds_role") == "audit_only"
        and continuous_state_contract.get("state_layout") == "[q;v;z]",
        "continuous-state artifact contract drifted",
    )
    require(
        metadata.get("track_irregularity_identifier")
        == scenario["inputs"]["track_irregularity_identifier"],
        "track irregularity identity drifted",
    )
    require(
        metadata.get("sample_period_nanoseconds")
        == clock["sample_period_nanoseconds"],
        "sample period drifted",
    )
    require(
        metadata.get("terminal_time_nanoseconds")
        == clock["duration_nanoseconds"],
        "terminal clock drifted",
    )
    expected_sample_count = (
        clock["duration_nanoseconds"] // clock["sample_period_nanoseconds"] + 1
    )
    require(metadata.get("sample_count") == expected_sample_count,
            "sample count drifted")
    require(
        metadata.get("local_sample_refinement") is None,
        "unexpected local sample refinement changed the frozen clock",
    )
    assembled_layout = nested_object(metadata, "assembled_state_and_force_layout")
    require(
        assembled_layout.get("generalized_position_count")
        == layout["generalized_position_count"]
        and assembled_layout.get("generalized_velocity_count")
        == layout["generalized_velocity_count"]
        and assembled_layout.get("series_force_state_count")
        == layout["series_force_state_count"],
        "assembled q/v/z layout drifted",
    )
    numerical = nested_object(metadata, "numerical_execution_contract")
    require(
        numerical.get("qualification_case_identifier") == case["identifier"],
        "metadata qualification case drifted",
    )
    require(
        numerical.get("tolerance_tier_identifier") == case["tier"]
        and numerical.get("tolerance_scale_from_scenario_recipe") == scale,
        "metadata tolerance tier or scale drifted",
    )
    require(
        numerical.get("integrator_recipe_identifier") == expected_recipe,
        "metadata integrator recipe drifted",
    )
    floating_point = nested_object(
        numerical, "floating_point_compilation_contract"
    )
    require(
        floating_point.get("identifier")
        == execution["floating_point_semantics_identifier"]
        and floating_point.get("cmake_external_flag_audit_passed") is True
        and floating_point.get("compile_command_audit_enabled") is True
        and floating_point.get("fast_math_macro_defined") is False
        and floating_point.get("finite_math_only_enabled") is False
        and floating_point.get("build_type") == execution["build_type"]
        and type(floating_point.get("compiler_id")) is str
        and bool(floating_point.get("compiler_id"))
        and type(floating_point.get("compiler_version")) is str
        and bool(floating_point.get("compiler_version")),
        "metadata strict floating-point compilation identity drifted",
    )
    compiled_compiler_identity = (
        f"{floating_point.get('compiler_id')} "
        f"{floating_point.get('compiler_version')}"
    )
    require(
        compiled_compiler_identity == binding["expected_compiler_identity"],
        "wrapper compiler identity disagrees with the compiled runner",
    )
    for field in (
        "relative_tolerance",
        "generalized_position_absolute_tolerance",
        "generalized_velocity_absolute_tolerance",
        "series_force_absolute_tolerance_newtons",
    ):
        require(
            numerical.get(field) == default_numerics[field] * scale,
            f"metadata resolved numerical field {field} drifted",
        )
    require(
        numerical.get("openmp_dynamic_teams_enabled") is False
        and numerical.get("openmp_runtime_maximum_threads") == 1
        and numerical.get("contact_batch_requested_worker_count") == 1
        and numerical.get("contact_batch_parallel_team_probe_worker_count") == 1,
        "metadata OpenMP/contact serial identity drifted",
    )
    require(
        performance.get("qualification_case_identifier") == case["identifier"]
        and performance.get("integrator_recipe_identifier") == expected_recipe,
        "performance backend/case identity drifted",
    )
    advance_wall_seconds = performance.get("advance_wall_seconds")
    valid_advance_wall_seconds = (
        type(advance_wall_seconds) is int
        and 0 <= advance_wall_seconds <= sys.float_info.max
    ) or (
        type(advance_wall_seconds) is float
        and math.isfinite(advance_wall_seconds)
        and advance_wall_seconds >= 0.0
    )
    require(
        valid_advance_wall_seconds,
        "primary advance_wall_seconds timing is missing or invalid",
    )
    performance_statistics = nested_object(performance, "integration_statistics")
    statistics_fields = manifest["comparison_contract"]["statistics_fields"]
    for field in statistics_fields:
        value = performance_statistics.get(field)
        require(
            type(value) is int and value >= 0,
            f"performance statistic {field} is missing or invalid",
        )
    require(
        performance_statistics.get(
            "requested_dense_finite_difference_jacobian_worker_count"
        ) == 1,
        "performance Jacobian worker identity drifted",
    )
    errors.extend(
        _validate_completion_and_tables(
            artifact_directory,
            layout=layout,
            expected_sample_count=expected_sample_count,
            sample_period_nanoseconds=clock["sample_period_nanoseconds"],
        )
    )
    return errors


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--identity-output", type=Path, required=True)
    parser.add_argument("--orvd-revision", required=True)
    parser.add_argument("--build-type", choices=("Release",), required=True)
    parser.add_argument("--compiler-identity", required=True)
    parser.add_argument("--cpu-affinity", required=True)
    parser.add_argument(
        "--vehicle-recipe", choices=tuple(RUNNER_LAYOUTS), default="gz18"
    )
    parser.add_argument("--comparison-manifest", type=Path)
    parser.add_argument("--comparison-scenario")
    parser.add_argument("--comparison-case")
    parser.add_argument("--comparison-source-root", type=Path)
    parser.add_argument("runner_arguments", nargs=argparse.REMAINDER)
    arguments = parser.parse_args(list(argv))
    if arguments.runner_arguments[:1] == ["--"]:
        arguments.runner_arguments = arguments.runner_arguments[1:]
    if not arguments.runner_arguments:
        parser.error("runner arguments are empty")
    comparison_values = (
        arguments.comparison_manifest,
        arguments.comparison_scenario,
        arguments.comparison_case,
    )
    if any(value is not None for value in comparison_values) and not all(
        value is not None for value in comparison_values
    ):
        parser.error(
            "--comparison-manifest, --comparison-scenario and "
            "--comparison-case must be supplied together"
        )
    if arguments.comparison_source_root is not None and not all(
        value is not None for value in comparison_values
    ):
        parser.error("--comparison-source-root requires a complete binding")
    layouts = RUNNER_LAYOUTS[arguments.vehicle_recipe]
    if len(arguments.runner_arguments) not in layouts:
        parser.error(
            f"the {arguments.vehicle_recipe.upper()} qualification runner "
            f"requires one of {tuple(layouts)} argument counts"
        )
    return arguments


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    try:
        executable = arguments.executable.resolve(strict=True)
    except OSError as error:
        print(f"could not resolve qualification executable: {error}",
              file=sys.stderr)
        return 2
    if not executable.is_file() or not os.access(executable, os.X_OK):
        print(f"qualification executable is not executable: {executable}",
              file=sys.stderr)
        return 2
    if posix_resource is None:
        print("qualification execution metrics require POSIX resource "
              "accounting", file=sys.stderr)
        return 2
    try:
        comparison_binding = prepare_manifest_bound_execution(
            arguments, os.environ
        )
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f"comparison manifest preflight failed: {error}", file=sys.stderr)
        return 2
    try:
        affinity = parse_affinity(arguments.cpu_affinity)
        os.sched_setaffinity(0, affinity)
        applied_affinity = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError, ValueError) as error:
        print(f"could not set CPU affinity: {error}", file=sys.stderr)
        return 2
    if comparison_binding is not None and set(applied_affinity) != affinity:
        print("applied CPU affinity differs from the manifest-bound request",
              file=sys.stderr)
        return 2

    environment_keys = (
        "OMP_NUM_THREADS", "OMP_DYNAMIC", "OMP_PLACES", "OMP_PROC_BIND",
        "OMP_THREAD_LIMIT", "OMP_MAX_ACTIVE_LEVELS", "OMP_NESTED",
    )
    child_environment = (
        comparison_binding["child_environment"]
        if comparison_binding is not None
        else os.environ
    )
    openmp_environment = {
        key: child_environment[key]
        for key in environment_keys
        if key in child_environment
    }
    output_directory_argument_index = RUNNER_LAYOUTS[
        arguments.vehicle_recipe
    ][len(arguments.runner_arguments)]
    artifact_directory = Path(
        arguments.runner_arguments[output_directory_argument_index]
    ).resolve()
    before_usage = posix_resource.getrusage(posix_resource.RUSAGE_CHILDREN)
    begin = time.perf_counter()
    runner_exit_status: int | None = None
    launch_error: str | None = None
    try:
        completed = subprocess.run(
            [str(executable), *arguments.runner_arguments],
            check=False,
            env=child_environment,
        )
        runner_exit_status = completed.returncode
    except OSError as error:
        launch_error = f"could not launch qualification runner: {error}"
    wall_seconds = time.perf_counter() - begin
    after_usage = posix_resource.getrusage(posix_resource.RUSAGE_CHILDREN)
    child_user_seconds = after_usage.ru_utime - before_usage.ru_utime
    child_system_seconds = after_usage.ru_stime - before_usage.ru_stime
    postflight_errors: list[str] = []
    if launch_error is not None:
        postflight_errors.append(launch_error)
    elif comparison_binding is not None:
        if runner_exit_status == 0:
            try:
                postflight_errors = validate_manifest_bound_artifact(
                    artifact_directory, comparison_binding
                )
            except (KeyError, OSError, TypeError, UnicodeError, ValueError) as error:
                postflight_errors = [
                    f"could not validate qualification artifact: {error}"
                ]
        else:
            postflight_errors = [
                f"qualification runner exited with status {runner_exit_status}"
            ]

    provenance_errors: list[str] = []
    try:
        hardware = processor_identity()
    except (OSError, UnicodeError) as error:
        hardware = "unavailable"
        provenance_errors.append(f"could not read processor identity: {error}")
    try:
        executable_sha256: str | None = sha256_file(executable)
    except OSError as error:
        executable_sha256 = None
        provenance_errors.append(f"could not hash qualification executable: {error}")
    postflight_errors.extend(provenance_errors)

    wrapper_exit_status = (
        runner_exit_status
        if runner_exit_status not in (None, 0)
        else (2 if postflight_errors else 0)
    )
    if comparison_binding is not None:
        binding_identity = comparison_binding["identity"]
        binding_identity["validation_status"] = (
            "passed" if not postflight_errors else "failed"
        )
        binding_identity["validation_errors"] = postflight_errors
    identity = {
        "orvd_revision": arguments.orvd_revision,
        "vehicle_recipe": arguments.vehicle_recipe,
        "build_type": arguments.build_type,
        "compiler": arguments.compiler_identity,
        "hardware": hardware,
        "requested_cpu_affinity": arguments.cpu_affinity,
        "applied_cpu_affinity": applied_affinity,
        "openmp_environment": openmp_environment,
        "runner_arguments": arguments.runner_arguments,
        "qualification_artifact_directory": str(artifact_directory),
        "process_wall_seconds": wall_seconds,
        "process_user_seconds": child_user_seconds,
        "process_system_seconds": child_system_seconds,
        "process_cpu_utilization_percent": (
            100.0 * (child_user_seconds + child_system_seconds) / wall_seconds
            if wall_seconds > 0.0
            else 0.0
        ),
        "maximum_resident_set_kilobytes": after_usage.ru_maxrss,
        "executable": str(executable),
        "executable_sha256": executable_sha256,
        "runner_exit_status": runner_exit_status,
        "wrapper_exit_status": wrapper_exit_status,
        "exit_status": wrapper_exit_status,
    }
    if launch_error is not None:
        identity["launch_error"] = launch_error
    if comparison_binding is not None:
        identity["comparison_binding"] = comparison_binding["identity"]
        if postflight_errors:
            print(
                "comparison manifest postflight failed: "
                + "; ".join(postflight_errors),
                file=sys.stderr,
            )
    elif postflight_errors:
        print(
            "qualification metrics wrapper failed: "
            + "; ".join(postflight_errors),
            file=sys.stderr,
        )
    try:
        write_identity(arguments.identity_output.resolve(), identity)
    except OSError as error:
        print(f"could not publish execution identity: {error}", file=sys.stderr)
        return wrapper_exit_status if wrapper_exit_status != 0 else 2
    return wrapper_exit_status


if __name__ == "__main__":
    raise SystemExit(main())
