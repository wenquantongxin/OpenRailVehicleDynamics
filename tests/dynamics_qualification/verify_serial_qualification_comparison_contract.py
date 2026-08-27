#!/usr/bin/env python3
"""Verify the serial qualification-comparison execution contract."""

from __future__ import annotations

import copy
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace


sys.dont_write_bytecode = True

SOURCE_ROOT = Path(__file__).resolve().parents[2]
COMPARISON_MODULE = (
    SOURCE_ROOT / "tools/dynamics_qualification/qualification_comparison.py"
)
MANIFEST_PATH = (
    SOURCE_ROOT
    / "tools/dynamics_qualification/int07a_serial_comparison_manifest.json"
)
METRICS_WRAPPER = (
    SOURCE_ROOT
    / "tools/dynamics_qualification/run_qualification_with_metrics.py"
)

EXPECTED_CASES = (
    "scenario_default_cvode_coarse",
    "scenario_default_cvode_nominal",
    "scenario_default_cvode_fine",
    "scenario_default_cvode_reference",
    "radau5_coarse",
    "radau5_nominal",
    "radau5_fine",
    "radau5_reference",
)


def load_comparison_module(path: Path = COMPARISON_MODULE) -> ModuleType:
    specification = importlib.util.spec_from_file_location(
        "orvd_serial_qualification_comparison_module", path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError(f"could not load {path}")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def require(condition: bool, detail: str) -> None:
    if not condition:
        raise AssertionError(detail)


def expect_invalid(module: ModuleType, manifest: dict[str, object], detail: str) -> None:
    try:
        module.validate_manifest(manifest)
    except module.ManifestValidationError:
        return
    raise AssertionError(f"validator accepted {detail}")


def check_frozen_manifest(module: ModuleType) -> dict[str, object]:
    manifest = module.load_manifest(MANIFEST_PATH, source_root=SOURCE_ROOT)
    require(
        module.qualification_case_identifiers(manifest) == EXPECTED_CASES,
        "the manifest does not contain the ordered closed eight-case set",
    )
    require(
        manifest["performance_decision_eligible"] is False,
        "INT-07A evidence became performance-decision eligible",
    )
    execution = manifest["required_execution"]
    require(
        execution["jacobian_worker_count"] == 1
        and execution["floating_point_semantics_identifier"]
        == "orvd.strict_ieee_no_fast_math.v1"
        and execution["omp_num_threads"] == 1
        and execution["omp_dynamic"] is False
        and execution["omp_max_active_levels"] == 1
        and execution["omp_nested"] is False
        and execution["cpu_affinity_core_count"] == 1,
        "the manifest is not a one-worker serial baseline",
    )
    contract = manifest["comparison_contract"]
    require(
        contract["continuous_state_artifact_filename"]
        == "continuous_states.tsv"
        and contract["candidate_values_in_normalization_denominator"] is False
        and contract["reference_smooth_rms_limit"] == 0.1
        and contract["reference_smooth_inf_norm_limit"] == 1.0
        and contract["eligible_candidate_smooth_rms_limit"] == 1.0
        and contract["eligible_candidate_smooth_inf_norm_limit"] == 10.0
        and contract["primary_timing_metric"]
        == "performance_json.advance_wall_seconds"
        and contract["end_to_end_timing_guard_metric"]
        == "execution_identity.process_wall_seconds"
        and contract["reference_band_candidate_distance_rule"]
        == "maximum_distance_to_either_reference",
        "the reference-band or configuration-aware state contract drifted",
    )

    scenarios = {entry["identifier"]: entry for entry in manifest["scenarios"]}
    gz18 = scenarios["gz18_straight_aar6_v60_passive_20s"]
    require(
        gz18["runner_layout"] == "gz18"
        and gz18["inputs"]["track_irregularity_identifier"]
        == "aar6_irregularity"
        and gz18["clock"]["duration_nanoseconds"] == 20_000_000_000
        and gz18["clock"]["sample_period_nanoseconds"] == 500_000,
        "the GZ18 straight/AAR6 20 s at 0.5 ms scenario drifted",
    )
    irw = scenarios["irw_r300_aar5_v60_passive_30s"]
    require(
        irw["runner_layout"] == "irw-passive-scenario"
        and irw["runner_scenario_identifier"]
        == "irw_r300_aar5_v60_passive"
        and irw["inputs"]["track_irregularity_identifier"]
        == "aar5_irregularity"
        and irw["clock"]["duration_nanoseconds"] == 30_000_000_000
        and irw["clock"]["sample_period_nanoseconds"] == 500_000
        and irw["smooth_state_window"]["end_time_nanoseconds"] == 10_000_000
        and irw["smooth_state_window"]["require_constant_contact_patch_counts"]
        is True,
        "the IRW R300/AAR5 passive 30 s at 0.5 ms scenario drifted",
    )
    require(
        irw["clock"]["duration_nanoseconds"]
        // irw["clock"]["sample_period_nanoseconds"]
        + 1
        == 60_001,
        "the IRW 30 s/0.5 ms clock does not contain 60001 samples",
    )
    return manifest


def check_runner_arguments(module: ModuleType, manifest: dict[str, object]) -> None:
    expected_layouts = {
        "gz18_straight_aar6_v60_passive_20s": (9, 5),
        "irw_r300_aar5_v60_passive_30s": (10, 6),
    }
    for scenario_identifier, (argument_count, output_index) in expected_layouts.items():
        for case_identifier in EXPECTED_CASES:
            output = (
                SOURCE_ROOT
                / "tmp/serial-qualification-comparison-unexecuted"
                / scenario_identifier
                / case_identifier
            ).resolve()
            arguments = module.materialize_runner_arguments(
                manifest,
                scenario_identifier=scenario_identifier,
                qualification_case_identifier=case_identifier,
                source_root=SOURCE_ROOT,
                output_directory=output,
            )
            require(
                len(arguments) == argument_count,
                f"{scenario_identifier}/{case_identifier} has the wrong argv count",
            )
            require(
                arguments[output_index] == str(output),
                f"{scenario_identifier}/{case_identifier} has the wrong output slot",
            )
            require(
                arguments[-1] == case_identifier,
                f"{scenario_identifier}/{case_identifier} lost its closed case tail",
            )
            require(
                all(Path(argument).is_absolute() for argument in arguments[1:4])
                if scenario_identifier.startswith("irw_")
                else all(Path(argument).is_absolute() for argument in arguments[0:4]),
                f"{scenario_identifier}/{case_identifier} did not resolve inputs",
            )

    gz18_arguments = module.materialize_runner_arguments(
        manifest,
        scenario_identifier="gz18_straight_aar6_v60_passive_20s",
        qualification_case_identifier="radau5_reference",
        source_root=SOURCE_ROOT,
        output_directory=(
            SOURCE_ROOT
            / "tmp/serial-qualification-comparison-unexecuted/gz18-reference"
        ),
    )
    require(
        gz18_arguments[4] == "aar6_irregularity"
        and gz18_arguments[6:8] == ["20000000000", "500000"],
        "GZ18 argv lost its irregularity or integer clock",
    )
    irw_arguments = module.materialize_runner_arguments(
        manifest,
        scenario_identifier="irw_r300_aar5_v60_passive_30s",
        qualification_case_identifier="scenario_default_cvode_reference",
        source_root=SOURCE_ROOT,
        output_directory=(
            SOURCE_ROOT
            / "tmp/serial-qualification-comparison-unexecuted/irw-reference"
        ),
    )
    require(
        irw_arguments[0] == "irw_r300_aar5_v60_passive"
        and irw_arguments[5] == "aar5_irregularity"
        and irw_arguments[7:9] == ["30000000000", "500000"],
        "IRW argv lost its scenario, irregularity, or integer clock",
    )


def check_strict_rejections(module: ModuleType, manifest: dict[str, object]) -> None:
    unknown_field = copy.deepcopy(manifest)
    unknown_field["future_option_bag"] = {}
    expect_invalid(module, unknown_field, "an unknown top-level field")

    invalid_clock = copy.deepcopy(manifest)
    invalid_clock["scenarios"][0]["clock"]["sample_period_nanoseconds"] = 3
    expect_invalid(module, invalid_clock, "a non-integral sample clock")

    invalid_worker = copy.deepcopy(manifest)
    invalid_worker["required_execution"]["jacobian_worker_count"] = 2
    expect_invalid(module, invalid_worker, "a non-serial Jacobian worker count")

    invalid_floating_point = copy.deepcopy(manifest)
    invalid_floating_point["required_execution"][
        "floating_point_semantics_identifier"
    ] = "fast"
    expect_invalid(
        module, invalid_floating_point, "a non-strict floating-point identity"
    )

    invalid_omp_worker = copy.deepcopy(manifest)
    invalid_omp_worker["required_execution"]["omp_num_threads"] = 4
    expect_invalid(module, invalid_omp_worker, "a non-serial OpenMP worker count")

    invalid_active_levels = copy.deepcopy(manifest)
    invalid_active_levels["required_execution"]["omp_max_active_levels"] = 2
    expect_invalid(
        module, invalid_active_levels, "nested OpenMP active levels"
    )

    invalid_smooth_window = copy.deepcopy(manifest)
    invalid_smooth_window["scenarios"][1]["smooth_state_window"][
        "require_constant_contact_patch_counts"
    ] = False
    expect_invalid(module, invalid_smooth_window, "a mutable smooth-state window")

    decision_enabled = copy.deepcopy(manifest)
    decision_enabled["performance_decision_eligible"] = True
    expect_invalid(module, decision_enabled, "performance-decision eligibility")

    ranking_enabled = copy.deepcopy(manifest)
    ranking_enabled["comparison_contract"]["performance_ranking_enabled"] = True
    expect_invalid(module, ranking_enabled, "performance ranking")

    invalid_candidate_gate = copy.deepcopy(manifest)
    invalid_candidate_gate["comparison_contract"][
        "eligible_candidate_smooth_rms_limit"
    ] = 2.0
    expect_invalid(module, invalid_candidate_gate, "a drifted candidate state gate")

    invalid_primary_timing = copy.deepcopy(manifest)
    invalid_primary_timing["comparison_contract"]["primary_timing_metric"] = (
        "execution_identity.process_wall_seconds"
    )
    expect_invalid(module, invalid_primary_timing, "a drifted primary timing metric")

    duplicate_manifest_text = MANIFEST_PATH.read_text(encoding="utf-8").replace(
        '"schema_version": 2,',
        '"schema_version": 2,\n  "schema_version": 2,',
        1,
    )
    with tempfile.TemporaryDirectory(
        prefix="orvd-serial-comparison-duplicate-manifest-"
    ) as temporary:
        duplicate_path = Path(temporary) / "duplicate.json"
        duplicate_path.write_text(duplicate_manifest_text, encoding="utf-8")
        try:
            module.load_manifest(duplicate_path)
        except module.ManifestValidationError as error:
            require(
                "duplicate key" in str(error),
                "duplicate manifest key failed for an unrelated reason",
            )
        else:
            raise AssertionError("manifest loader accepted a duplicate JSON key")


def wrapper_arguments(
    runner_arguments: list[str],
    *,
    scenario_identifier: str,
    case_identifier: str,
    cpu_affinity: str = "0",
    identity_output: Path = Path(
        "/tmp/orvd-unused-serial-comparison-identity.json"
    ),
    executable: Path | None = None,
) -> list[str]:
    vehicle_recipe = (
        "gz18"
        if scenario_identifier.startswith("gz18_")
        else "irw-passive-scenario"
    )
    executable_name = (
        "orvd_gz18_dynamics_qualification"
        if vehicle_recipe == "gz18"
        else "orvd_irw_passive_scenario"
    )
    executable_path = (
        executable
        if executable is not None
        else SOURCE_ROOT
        / "build/tools/dynamics_qualification"
        / executable_name
    )
    return [
        "--executable",
        str(executable_path),
        "--identity-output",
        str(identity_output),
        "--orvd-revision",
        "test-revision",
        "--build-type",
        "Release",
        "--compiler-identity",
        "synthetic 1.0",
        "--cpu-affinity",
        cpu_affinity,
        "--vehicle-recipe",
        vehicle_recipe,
        "--comparison-manifest",
        str(MANIFEST_PATH),
        "--comparison-scenario",
        scenario_identifier,
        "--comparison-case",
        case_identifier,
        "--comparison-source-root",
        str(SOURCE_ROOT),
        "--",
        *runner_arguments,
    ]


def expect_value_error(function: object, *arguments: object) -> None:
    try:
        function(*arguments)  # type: ignore[operator]
    except ValueError:
        return
    raise AssertionError("manifest-bound wrapper accepted an invalid execution")


def write_synthetic_artifact(
    wrapper: ModuleType,
    artifact: Path,
    binding: dict[str, object],
) -> dict[str, object]:
    scenario = binding["scenario"]
    case = binding["case"]
    clock = scenario["clock"]
    layout = scenario["state_layout"]
    default_numerics = scenario["scenario_default_numerics"]
    sample_count = (
        clock["duration_nanoseconds"] // clock["sample_period_nanoseconds"] + 1
    )
    artifact.mkdir(parents=True, exist_ok=True)
    (artifact / "COMPLETE").write_text(
        f"{sample_count} samples\n", encoding="utf-8"
    )

    state_header = (
        *wrapper.TIME_COLUMNS,
        *(
            f"q.{index}"
            for index in range(layout["generalized_position_count"])
        ),
        *(
            f"v.{index}"
            for index in range(layout["generalized_velocity_count"])
        ),
        *(
            f"z.{index}"
            for index in range(layout["series_force_state_count"])
        ),
    )
    state_size = len(state_header) - 3
    state_lines = ["\t".join(state_header)]
    observation_lines = [
        "sample_index\ttime_nanoseconds\ttime_seconds"
        "\tsynthetic_interface.contact_patch_count\tprobe"
    ]
    patch_lines = ["\t".join(wrapper.CONTACT_PATCH_COLUMNS)]
    for sample_index in range(sample_count):
        time_nanoseconds = sample_index * clock["sample_period_nanoseconds"]
        time_seconds = str(time_nanoseconds * 1.0e-9)
        prefix = [str(sample_index), str(time_nanoseconds), time_seconds]
        state_lines.append("\t".join([*prefix, *(["0"] * state_size)]))
        observation_lines.append("\t".join([*prefix, "1", "0"]))
        patch_lines.append(
            "\t".join(
                [*prefix, "synthetic_interface", *(["0"] * 11)]
            )
        )
    (artifact / "continuous_states.tsv").write_text(
        "\n".join(state_lines) + "\n", encoding="utf-8"
    )
    (artifact / "observations.tsv").write_text(
        "\n".join(observation_lines) + "\n", encoding="utf-8"
    )
    (artifact / "contact_patches.tsv").write_text(
        "\n".join(patch_lines) + "\n", encoding="utf-8"
    )

    metadata = {
        "completed": True,
        "artifact_schema_identifier": "orvd.passive_vehicle_qualification.v2",
        "input_paths": {
            key: str((SOURCE_ROOT / relative).resolve())
            for key, relative in scenario["inputs"].items()
            if key != "track_irregularity_identifier"
        },
        "track_irregularity_identifier": scenario["inputs"][
            "track_irregularity_identifier"
        ],
        "continuous_state_observation_contract": {
            "file": "continuous_states.tsv",
            "row_join_key": ["sample_index", "time_nanoseconds"],
            "time_seconds_role": "audit_only",
            "state_layout": "[q;v;z]",
        },
        "sample_period_nanoseconds": clock["sample_period_nanoseconds"],
        "terminal_time_nanoseconds": clock["duration_nanoseconds"],
        "sample_count": sample_count,
        "local_sample_refinement": None,
        "assembled_state_and_force_layout": layout,
        "numerical_execution_contract": {
            "qualification_case_identifier": case["identifier"],
            "tolerance_tier_identifier": case["tier"],
            "tolerance_scale_from_scenario_recipe": case["scale"],
            "integrator_recipe_identifier": "radau5",
            "floating_point_compilation_contract": {
                "identifier": "orvd.strict_ieee_no_fast_math.v1",
                "cmake_external_flag_audit_passed": True,
                "compile_command_audit_enabled": True,
                "fast_math_macro_defined": False,
                "finite_math_only_enabled": False,
                "build_type": "Release",
                "compiler_id": "synthetic",
                "compiler_version": "1.0",
            },
            "relative_tolerance": default_numerics["relative_tolerance"]
            * case["scale"],
            "generalized_position_absolute_tolerance": default_numerics[
                "generalized_position_absolute_tolerance"
            ]
            * case["scale"],
            "generalized_velocity_absolute_tolerance": default_numerics[
                "generalized_velocity_absolute_tolerance"
            ]
            * case["scale"],
            "series_force_absolute_tolerance_newtons": default_numerics[
                "series_force_absolute_tolerance_newtons"
            ]
            * case["scale"],
            "openmp_dynamic_teams_enabled": False,
            "openmp_runtime_maximum_threads": 1,
            "contact_batch_requested_worker_count": 1,
            "contact_batch_parallel_team_probe_worker_count": 1,
        },
    }
    performance = {
        "qualification_case_identifier": case["identifier"],
        "integrator_recipe_identifier": "radau5",
        "advance_wall_seconds": 0.25,
        "integration_statistics": {
            field: 0
            for field in binding["manifest"]["comparison_contract"][
                "statistics_fields"
            ]
        },
    }
    performance["integration_statistics"][
        "requested_dense_finite_difference_jacobian_worker_count"
    ] = 1
    (artifact / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8"
    )
    (artifact / "performance.json").write_text(
        json.dumps(performance), encoding="utf-8"
    )
    return performance


def check_wrapper_main_execution(
    wrapper: ModuleType,
    runner_arguments: list[str],
    *,
    scenario_identifier: str,
    case_identifier: str,
) -> None:
    real_os = wrapper.os
    real_posix_resource = wrapper.posix_resource
    real_subprocess = wrapper.subprocess
    isolated_os = ModuleType("orvd_serial_comparison_test_os")
    isolated_os.__dict__.update(vars(real_os))
    isolated_posix_resource = ModuleType(
        "orvd_serial_comparison_test_posix_resource"
    )
    isolated_posix_resource.RUSAGE_CHILDREN = 1
    isolated_subprocess = ModuleType("orvd_serial_comparison_test_subprocess")
    isolated_subprocess.__dict__.update(vars(real_subprocess))
    original_validate = wrapper.validate_manifest_bound_artifact
    original_processor_identity = wrapper.processor_identity
    original_sha256_file = wrapper.sha256_file
    captured_environments: list[dict[str, str]] = []
    captured_affinities: list[tuple[int, set[int]]] = []

    def set_affinity(process: int, affinity: set[int]) -> None:
        captured_affinities.append((process, set(affinity)))

    def getrusage(scope: int) -> SimpleNamespace:
        require(
            scope == isolated_posix_resource.RUSAGE_CHILDREN,
            "main() requested an unexpected resource-accounting scope",
        )
        return SimpleNamespace(ru_utime=0.0, ru_stime=0.0, ru_maxrss=0)

    def successful_run(command: list[str], *, check: bool, env: dict[str, str]):
        require(check is False, "the wrapper unexpectedly enabled check=True")
        require(
            command[1:] == runner_arguments,
            "main() did not forward the exact manifest runner argv",
        )
        captured_environments.append(dict(env))
        return wrapper.subprocess.CompletedProcess(command, 0)

    try:
        wrapper.os = isolated_os
        wrapper.posix_resource = isolated_posix_resource
        wrapper.subprocess = isolated_subprocess
        wrapper.os.sched_setaffinity = set_affinity
        wrapper.os.sched_getaffinity = lambda process: {0}
        wrapper.posix_resource.getrusage = getrusage
        wrapper.subprocess.run = successful_run
        wrapper.validate_manifest_bound_artifact = lambda artifact, binding: []
        with tempfile.TemporaryDirectory(
            prefix="orvd-serial-comparison-wrapper-main-"
        ) as temporary:
            temporary_path = Path(temporary)
            executable_name = (
                "orvd_gz18_dynamics_qualification"
                if scenario_identifier.startswith("gz18_")
                else "orvd_irw_passive_scenario"
            )
            fake_executable = temporary_path / executable_name
            fake_executable.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
            fake_executable.chmod(0o700)
            resolved_fake_executable = fake_executable.resolve(strict=True)
            wrapper.os.access = lambda path, mode: (
                Path(path) == resolved_fake_executable
                and mode == wrapper.os.X_OK
            )

            def main_arguments(identity_output: Path) -> list[str]:
                return wrapper_arguments(
                    runner_arguments,
                    scenario_identifier=scenario_identifier,
                    case_identifier=case_identifier,
                    identity_output=identity_output,
                    executable=fake_executable,
                )

            success_identity = temporary_path / "success.json"
            success_code = wrapper.main(main_arguments(success_identity))
            require(success_code == 0, "main() rejected a valid serial run")
            success = json.loads(success_identity.read_text(encoding="utf-8"))
            require(
                success["runner_exit_status"] == 0
                and success["wrapper_exit_status"] == 0
                and success["exit_status"] == 0
                and success["comparison_binding"]["validation_status"]
                == "passed"
                and captured_affinities[-1] == (0, {0})
                and all(
                    captured_environments[-1][key] == value
                    for key, value in wrapper.SERIAL_OPENMP_ENVIRONMENT.items()
                ),
                "main() lost the serial child environment or success identity",
            )

            failed_identity = temporary_path / "postflight-failed.json"
            wrapper.validate_manifest_bound_artifact = (
                lambda artifact, binding: ["simulated truncated artifact"]
            )
            with contextlib.redirect_stderr(io.StringIO()):
                failed_code = wrapper.main(main_arguments(failed_identity))
            failed = json.loads(failed_identity.read_text(encoding="utf-8"))
            require(
                failed_code == 2
                and failed["runner_exit_status"] == 0
                and failed["wrapper_exit_status"] == 2
                and failed["exit_status"] == 2
                and failed["comparison_binding"]["validation_status"]
                == "failed",
                "postflight failure did not control the identity and return code",
            )

            provenance_identity = temporary_path / "provenance-failed.json"
            wrapper.validate_manifest_bound_artifact = lambda artifact, binding: []

            def failed_processor_identity() -> str:
                raise OSError("simulated processor identity failure")

            def failed_sha256(path: Path) -> str:
                raise OSError("simulated executable hash failure")

            wrapper.processor_identity = failed_processor_identity
            wrapper.sha256_file = failed_sha256
            with contextlib.redirect_stderr(io.StringIO()):
                provenance_code = wrapper.main(
                    main_arguments(provenance_identity)
                )
            provenance = json.loads(
                provenance_identity.read_text(encoding="utf-8")
            )
            require(
                provenance_code == 2
                and provenance["runner_exit_status"] == 0
                and provenance["wrapper_exit_status"] == 2
                and provenance["hardware"] == "unavailable"
                and provenance["executable_sha256"] is None
                and provenance["comparison_binding"]["validation_status"]
                == "failed",
                "provenance OSError did not fail closed with a stable identity",
            )
            wrapper.processor_identity = original_processor_identity
            wrapper.sha256_file = original_sha256_file

            runner_identity = temporary_path / "runner-failed.json"

            def failed_runner(command: list[str], *, check: bool, env: dict[str, str]):
                return wrapper.subprocess.CompletedProcess(command, 7)

            wrapper.subprocess.run = failed_runner
            with contextlib.redirect_stderr(io.StringIO()):
                runner_code = wrapper.main(main_arguments(runner_identity))
            runner = json.loads(runner_identity.read_text(encoding="utf-8"))
            require(
                runner_code == 7
                and runner["runner_exit_status"] == 7
                and runner["wrapper_exit_status"] == 7
                and runner["comparison_binding"]["validation_status"]
                == "failed",
                "a runner failure did not propagate through the failed identity",
            )

            launch_identity = temporary_path / "launch-failed.json"

            def failed_launch(command: list[str], *, check: bool, env: dict[str, str]):
                raise OSError("simulated launch failure")

            wrapper.subprocess.run = failed_launch
            with contextlib.redirect_stderr(io.StringIO()):
                launch_code = wrapper.main(main_arguments(launch_identity))
            launch = json.loads(launch_identity.read_text(encoding="utf-8"))
            require(
                launch_code == 2
                and launch["runner_exit_status"] is None
                and launch["wrapper_exit_status"] == 2
                and launch["comparison_binding"]["validation_status"]
                == "failed"
                and "simulated launch failure" in launch["launch_error"],
                "an OSError launch did not publish a controlled failed identity",
            )
    finally:
        wrapper.os = real_os
        wrapper.posix_resource = real_posix_resource
        wrapper.subprocess = real_subprocess
        wrapper.validate_manifest_bound_artifact = original_validate
        wrapper.processor_identity = original_processor_identity
        wrapper.sha256_file = original_sha256_file


def check_manifest_bound_wrapper(
    comparison: ModuleType, manifest: dict[str, object]
) -> None:
    wrapper = load_comparison_module(METRICS_WRAPPER)
    scenario_identifier = "gz18_straight_aar6_v60_passive_20s"
    case_identifier = "radau5_reference"
    output = (
        SOURCE_ROOT
        / "tmp/serial-qualification-comparison-unexecuted/wrapper-bound-gz18"
    )
    runner_arguments = comparison.materialize_runner_arguments(
        manifest,
        scenario_identifier=scenario_identifier,
        qualification_case_identifier=case_identifier,
        source_root=SOURCE_ROOT,
        output_directory=output,
    )
    parsed = wrapper.parse_arguments(
        wrapper_arguments(
            runner_arguments,
            scenario_identifier=scenario_identifier,
            case_identifier=case_identifier,
        )
    )
    binding = wrapper.prepare_manifest_bound_execution(
        parsed, {"OMP_NUM_THREADS": "99", "UNRELATED": "preserved"}
    )
    require(binding is not None, "the complete manifest binding was ignored")
    require(
        all(
            binding["child_environment"][key] == value
            for key, value in wrapper.SERIAL_OPENMP_ENVIRONMENT.items()
        )
        and binding["child_environment"]["UNRELATED"] == "preserved"
        and binding["identity"]["manifest_identifier"]
        == "int07a_serial_baseline_v2"
        and binding["identity"]["scenario_identifier"] == scenario_identifier
        and binding["identity"]["qualification_case_identifier"]
        == case_identifier,
        "the wrapper did not force or record its serial manifest identity",
    )

    wrong_arguments = list(runner_arguments)
    wrong_arguments[-2] = "499999"
    wrong_parsed = wrapper.parse_arguments(
        wrapper_arguments(
            wrong_arguments,
            scenario_identifier=scenario_identifier,
            case_identifier=case_identifier,
        )
    )
    expect_value_error(
        wrapper.prepare_manifest_bound_execution, wrong_parsed, {}
    )
    two_core_parsed = wrapper.parse_arguments(
        wrapper_arguments(
            runner_arguments,
            scenario_identifier=scenario_identifier,
            case_identifier=case_identifier,
            cpu_affinity="0,1",
        )
    )
    expect_value_error(
        wrapper.prepare_manifest_bound_execution, two_core_parsed, {}
    )

    partial_options = wrapper_arguments(
        runner_arguments,
        scenario_identifier=scenario_identifier,
        case_identifier=case_identifier,
    )
    case_option = partial_options.index("--comparison-case")
    del partial_options[case_option : case_option + 2]
    with contextlib.redirect_stderr(io.StringIO()):
        try:
            wrapper.parse_arguments(partial_options)
        except SystemExit as error:
            require(
                error.code == 2,
                "partial binding returned the wrong parser code",
            )
        else:
            raise AssertionError("the wrapper accepted a partial manifest binding")

    with tempfile.TemporaryDirectory(
        prefix="orvd-serial-comparison-wrapper-"
    ) as temporary:
        artifact = Path(temporary)
        synthetic_binding = copy.deepcopy(binding)
        synthetic_clock = synthetic_binding["scenario"]["clock"]
        synthetic_clock["duration_nanoseconds"] = 500_000
        synthetic_clock["sample_period_nanoseconds"] = 500_000
        performance = write_synthetic_artifact(
            wrapper, artifact, synthetic_binding
        )
        require(
            wrapper.validate_manifest_bound_artifact(
                artifact, synthetic_binding
            ) == [],
            "the wrapper rejected a consistent synthetic postflight identity",
        )

        wrong_compiler_binding = copy.deepcopy(synthetic_binding)
        wrong_compiler_binding["expected_compiler_identity"] = "other 9.9"
        require(
            wrapper.validate_manifest_bound_artifact(
                artifact, wrong_compiler_binding
            ),
            "the wrapper accepted a compiler identity different from the "
            "compiled runner",
        )

        metadata_text = (artifact / "metadata.json").read_text(encoding="utf-8")
        duplicate_metadata_text = metadata_text.replace(
            '"completed": true',
            '"completed": true, "completed": true',
            1,
        )
        (artifact / "metadata.json").write_text(
            duplicate_metadata_text, encoding="utf-8"
        )
        require(
            wrapper.validate_manifest_bound_artifact(artifact, synthetic_binding),
            "the wrapper accepted a duplicate metadata JSON key",
        )
        (artifact / "metadata.json").write_text(
            metadata_text, encoding="utf-8"
        )

        floating_point_metadata = json.loads(metadata_text)
        floating_point_metadata["numerical_execution_contract"][
            "floating_point_compilation_contract"
        ]["identifier"] = "fast"
        (artifact / "metadata.json").write_text(
            json.dumps(floating_point_metadata), encoding="utf-8"
        )
        require(
            wrapper.validate_manifest_bound_artifact(
                artifact, synthetic_binding
            ),
            "the wrapper accepted a non-strict floating-point artifact",
        )
        (artifact / "metadata.json").write_text(
            metadata_text, encoding="utf-8"
        )

        performance["advance_wall_seconds"] = "missing-clock"
        (artifact / "performance.json").write_text(
            json.dumps(performance), encoding="utf-8"
        )
        require(
            wrapper.validate_manifest_bound_artifact(artifact, synthetic_binding),
            "the wrapper accepted an invalid primary timing metric",
        )
        performance["advance_wall_seconds"] = 0.25
        (artifact / "performance.json").write_text(
            json.dumps(performance), encoding="utf-8"
        )
        performance["advance_wall_seconds"] = 10**400
        (artifact / "performance.json").write_text(
            json.dumps(performance), encoding="utf-8"
        )
        require(
            wrapper.validate_manifest_bound_artifact(artifact, synthetic_binding),
            "the wrapper overflowed or accepted an unrepresentable timing metric",
        )
        performance["advance_wall_seconds"] = 0.25
        (artifact / "performance.json").write_text(
            json.dumps(performance), encoding="utf-8"
        )

        complete_text = (artifact / "COMPLETE").read_text(encoding="utf-8")
        (artifact / "COMPLETE").write_text("", encoding="utf-8")
        require(
            wrapper.validate_manifest_bound_artifact(artifact, synthetic_binding),
            "the wrapper accepted an empty COMPLETE marker",
        )
        (artifact / "COMPLETE").write_text(complete_text, encoding="utf-8")

        continuous_text = (artifact / "continuous_states.tsv").read_text(
            encoding="utf-8"
        )
        continuous_header = continuous_text.splitlines()[0]
        (artifact / "continuous_states.tsv").write_text(
            continuous_header + "\n", encoding="utf-8"
        )
        require(
            wrapper.validate_manifest_bound_artifact(artifact, synthetic_binding),
            "the wrapper accepted a truncated continuous-state table",
        )
        (artifact / "continuous_states.tsv").write_text(
            continuous_text, encoding="utf-8"
        )

        patches_text = (artifact / "contact_patches.tsv").read_text(
            encoding="utf-8"
        )
        patch_lines = patches_text.splitlines()
        duplicated_key_fields = patch_lines[2].split("\t")
        duplicated_key_fields[:3] = ["0", "0", "0.0"]
        patch_lines[2] = "\t".join(duplicated_key_fields)
        (artifact / "contact_patches.tsv").write_text(
            "\n".join(patch_lines) + "\n", encoding="utf-8"
        )
        require(
            wrapper.validate_manifest_bound_artifact(artifact, synthetic_binding),
            "the wrapper accepted a duplicate patch key hiding a missing sample",
        )
        (artifact / "contact_patches.tsv").write_text(
            patches_text, encoding="utf-8"
        )

        statistics = performance["integration_statistics"]
        missing_value = statistics.pop("jacobian_evaluation_count")
        (artifact / "performance.json").write_text(
            json.dumps(performance), encoding="utf-8"
        )
        require(
            wrapper.validate_manifest_bound_artifact(artifact, synthetic_binding),
            "the wrapper accepted incomplete integration statistics",
        )
        statistics["jacobian_evaluation_count"] = missing_value
        performance["integration_statistics"][
            "requested_dense_finite_difference_jacobian_worker_count"
        ] = 2
        (artifact / "performance.json").write_text(
            json.dumps(performance), encoding="utf-8"
        )
        require(
            wrapper.validate_manifest_bound_artifact(artifact, synthetic_binding),
            "the wrapper accepted a non-serial artifact worker identity",
        )

    check_wrapper_main_execution(
        wrapper,
        runner_arguments,
        scenario_identifier=scenario_identifier,
        case_identifier=case_identifier,
    )


def main() -> int:
    module = load_comparison_module()
    manifest = check_frozen_manifest(module)
    check_runner_arguments(module, manifest)
    check_strict_rejections(module, manifest)
    check_manifest_bound_wrapper(module, manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
