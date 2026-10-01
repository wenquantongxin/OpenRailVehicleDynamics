#!/usr/bin/env python3
"""Verify closed qualification-wrapper layouts and CLI case rejection."""

from __future__ import annotations

import argparse
import builtins
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
from unittest.mock import patch


sys.dont_write_bytecode = True

EXPECTED_LAYOUTS = {
    "gz18": {8: 5, 9: 5},
    "irw-passive-scenario": {9: 6, 10: 6},
    "irw-r300-aar5-v60-100hz-full-state-guidance": {8: 6, 9: 6},
}

LEGACY_CASES = tuple(
    f"{backend}_{tier}"
    for backend in ("scenario_default_cvode", "radau5")
    for tier in ("coarse", "nominal", "fine", "reference")
)


def load_metrics_wrapper(path: Path) -> ModuleType:
    specification = importlib.util.spec_from_file_location(
        "orvd_qualification_metrics_wrapper", path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError(f"could not load metrics wrapper {path}")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def load_metrics_wrapper_without_posix_resource(path: Path) -> ModuleType:
    module = ModuleType("orvd_qualification_metrics_wrapper_without_resource")
    module.__file__ = str(path)
    original_import = builtins.__import__

    def import_without_resource(
        name: str,
        globals: object = None,
        locals: object = None,
        fromlist: tuple[str, ...] = (),
        level: int = 0,
    ) -> object:
        if name == "resource":
            raise ImportError("simulated non-POSIX Python")
        return original_import(name, globals, locals, fromlist, level)

    try:
        builtins.__import__ = import_without_resource  # type: ignore[assignment]
        exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"),
             module.__dict__)
    finally:
        builtins.__import__ = original_import
    return module


def wrapper_options(vehicle_recipe: str) -> list[str]:
    return [
        "--executable",
        "/bin/true",
        "--identity-output",
        "/tmp/orvd-unused-qualification-identity.json",
        "--build-type",
        "Release",
        "--compiler-identity",
        "test",
        "--cpu-affinity",
        "0",
        "--vehicle-recipe",
        vehicle_recipe,
        "--",
    ]


def require_parser_exit_two(function: object, arguments: list[str]) -> str:
    diagnostic = io.StringIO()
    with contextlib.redirect_stderr(diagnostic):
        try:
            function(arguments)  # type: ignore[operator]
        except SystemExit as error:
            if error.code == 2:
                return diagnostic.getvalue()
            raise AssertionError(
                f"argument parser exited with {error.code}, expected 2"
            ) from error
    raise AssertionError("argument parser accepted invalid runner arguments")


def check_wrapper_layouts(module: ModuleType) -> None:
    if module.RUNNER_LAYOUTS != EXPECTED_LAYOUTS:
        raise AssertionError(
            f"runner layouts changed: {module.RUNNER_LAYOUTS!r}"
        )
    for vehicle_recipe, layouts in EXPECTED_LAYOUTS.items():
        for argument_count, output_index in layouts.items():
            runner_arguments = [
                f"runner-argument-{index}" for index in range(argument_count)
            ]
            runner_arguments[output_index] = "qualification-artifact"
            parsed = module.parse_arguments(
                [*wrapper_options(vehicle_recipe), *runner_arguments]
            )
            if parsed.runner_arguments != runner_arguments:
                raise AssertionError(
                    f"{vehicle_recipe} did not preserve runner arguments"
                )
            if parsed.runner_positionals != runner_arguments:
                raise AssertionError("legacy positional runner layout changed")
            if parsed.runner_arguments[output_index] != "qualification-artifact":
                raise AssertionError(
                    f"{vehicle_recipe} selected the wrong output argument"
                )
        for invalid_count in (min(layouts) - 1, max(layouts) + 1):
            require_parser_exit_two(
                module.parse_arguments,
                [
                    *wrapper_options(vehicle_recipe),
                    *("unused" for _ in range(invalid_count)),
                ],
            )


def check_wrapper_configuration_options(module: ModuleType) -> None:
    for recipe, layouts in EXPECTED_LAYOUTS.items():
        count = min(layouts)
        output_index = layouts[count]
        base = [f"runner-argument-{index}" for index in range(count)]
        base[output_index] = "artifact with spaces"
        config = "configs/integration settings.json"
        for insertion in (0, output_index, len(base)):
            forwarded = [*base[:insertion], "--integration-config", config,
                         *base[insertion:]]
            if recipe == "irw-passive-scenario":
                forwarded = ["--scene-record", *forwarded, "--scene-record"]
            parsed = module.parse_arguments([*wrapper_options(recipe), *forwarded])
            if (parsed.runner_arguments != forwarded or parsed.runner_positionals != base
                    or parsed.integration_config_path != config):
                raise AssertionError(f"{recipe}: named options altered the runner argv or layout")
            if parsed.runner_positionals[output_index] != "artifact with spaces":
                raise AssertionError(f"{recipe}: named options shifted the output path")
        if recipe == "irw-passive-scenario":
            for case in (None, *LEGACY_CASES):
                positionals = [*base, *([] if case is None else [case])]
                forwarded = ["--scene-record", *positionals]
                parsed = module.parse_arguments([*wrapper_options(recipe), *forwarded])
                if (parsed.runner_arguments != forwarded or
                        parsed.runner_positionals != positionals):
                    raise AssertionError("legacy --scene-record layout or forwarding changed")
        for tail, message in (
            (["--integration-config"], "requires one non-empty path"),
            (["--integration-config", ""], "requires one non-empty path"),
            (["--integration-config", "--scene-record"], "requires one non-empty path"),
            (["--integration-config", config, "--integration-config", "other.json"],
             "may be specified only once"),
            (["radau5_fine", "--integration-config", config], "mutually exclusive"),
        ):
            diagnostic = require_parser_exit_two(
                module.parse_arguments, [*wrapper_options(recipe), *base, *tail])
            if message not in diagnostic:
                raise AssertionError(f"{recipe}: missing option diagnostic {message!r}")
        # The files do not exist: refusal must happen before manifest/executable
        # access, affinity changes or process execution.
        diagnostic = require_parser_exit_two(module.parse_arguments, [
            *wrapper_options(recipe)[:-1],
            "--comparison-manifest", "unread-manifest.json",
            "--comparison-scenario", "unused-scenario",
            "--comparison-case", "radau5_fine", "--",
            *base, "--integration-config", config,
        ])
        if "INT-07 manifest binding does not admit --integration-config" not in diagnostic:
            raise AssertionError("mechanical configuration was not rejected at the INT-07 boundary")


def check_wrapper_main_passthrough(module: ModuleType) -> None:
    with tempfile.TemporaryDirectory(prefix="orvd-config-wrapper-") as temporary:
        root = Path(temporary)
        executable = root / "unexecuted runner"
        executable.write_text("unused", encoding="utf-8")
        usage = SimpleNamespace(ru_utime=0.0, ru_stime=0.0, ru_maxrss=0)
        resource = SimpleNamespace(RUSAGE_CHILDREN=1, getrusage=lambda scope: usage)
        for recipe, layouts in EXPECTED_LAYOUTS.items():
            count = min(layouts)
            output_index = layouts[count]
            base = [f"runner-argument-{index}" for index in range(count)]
            output = root / recipe / "output with spaces"
            base[output_index] = str(output)
            forwarded = ["--integration-config", "config with spaces.json", *base]
            if recipe == "irw-passive-scenario":
                forwarded.insert(0, "--scene-record")
            identity = root / f"{recipe}.json"
            command_lines: list[list[str]] = []

            def record_run(command: list[str], *, check: bool, env: object) -> object:
                if check:
                    raise AssertionError("wrapper changed subprocess failure handling")
                command_lines.append(command)
                return subprocess.CompletedProcess(command, 0)

            with (
                patch.object(module, "posix_resource", resource),
                patch.object(module, "processor_identity", lambda: "test cpu"),
                patch.object(module.os, "sched_setaffinity", lambda pid, affinity: None, create=True),
                patch.object(module.os, "sched_getaffinity", lambda pid: {0}, create=True),
                patch.object(module.os, "access", lambda path, mode: True),
                patch.object(module.subprocess, "run", record_run),
            ):
                result = module.main([
                    "--executable", str(executable), "--identity-output", str(identity),
                    "--build-type", "Release", "--compiler-identity", "test",
                    "--cpu-affinity", "0", "--vehicle-recipe", recipe, "--", *forwarded,
                ])
            record = json.loads(identity.read_text(encoding="utf-8"))
            if (result != 0 or command_lines != [[str(executable.resolve()), *forwarded]] or
                    record["runner_arguments"] != forwarded or
                    record["qualification_artifact_directory"] != str(output.resolve())):
                raise AssertionError(f"{recipe}: main did not preserve full argv and filtered output path")


def check_cli_rejects_unknown_case(
    executable: Path, runner_arguments: list[str]
) -> None:
    completed = subprocess.run(
        [str(executable), *runner_arguments, "not_a_qualification_case"],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 2:
        raise AssertionError(
            f"{executable.name} returned {completed.returncode}, expected 2"
        )
    if "unknown time-integrator qualification case" not in completed.stderr:
        raise AssertionError(
            f"{executable.name} did not diagnose its invalid tail case"
        )


def check_cli_configuration_options(
    executable: Path, base: list[str], *, scene_record: bool = False
) -> None:
    for tail, message in (
        (["--integration-config"], "requires one non-empty path"),
        (["--integration-config", ""], "requires one non-empty path"),
        (["--integration-config", "--scene-record"], "requires one non-empty path"),
        (["--integration-config", "one.json", "--integration-config", "two.json"],
         "may be specified only once"),
        (["radau5_fine", "--integration-config", "one.json"], "mutually exclusive"),
    ):
        completed = subprocess.run([str(executable), *base, *tail], check=False,
                                   capture_output=True, text=True)
        if completed.returncode != 2 or message not in completed.stderr:
            raise AssertionError(f"{executable.name}: invalid config option did not report {message!r}")
    # Deliberately nonexistent assets keep these as parsing checks: a valid
    # command reaches the runner and fails there (1), never usage parsing (2).
    valid_commands = [base, *([*base, case] for case in LEGACY_CASES)]
    for insertion in (0, len(base) // 2, len(base)):
        command = [*base[:insertion], "--integration-config", "unread config.json",
                   *base[insertion:]]
        valid_commands.append(command)
        if scene_record:
            valid_commands.append(["--scene-record", *command])
    if scene_record:
        valid_commands.extend(["--scene-record", *base, case] for case in LEGACY_CASES)
    for command in valid_commands:
        completed = subprocess.run([str(executable), *command], check=False,
                                   capture_output=True, text=True)
        if completed.returncode != 1:
            raise AssertionError(
                f"{executable.name}: valid option layout did not reach the runner: "
                f"status={completed.returncode}, stderr={completed.stderr!r}")


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metrics-wrapper", type=Path, required=True)
    parser.add_argument("--gz18", type=Path, required=True)
    parser.add_argument("--irw-passive", type=Path, required=True)
    parser.add_argument("--irw-guidance", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    wrapper = load_metrics_wrapper(arguments.metrics_wrapper)
    check_wrapper_layouts(wrapper)
    check_wrapper_configuration_options(wrapper)
    check_wrapper_main_passthrough(wrapper)
    without_resource = load_metrics_wrapper_without_posix_resource(
        arguments.metrics_wrapper
    )
    if without_resource.posix_resource is not None:
        raise AssertionError("wrapper did not tolerate missing POSIX resource")
    check_wrapper_layouts(without_resource)
    check_wrapper_configuration_options(without_resource)
    check_cli_rejects_unknown_case(
        arguments.gz18,
        ["vehicle", "startup", "line", "data", "irregularity", "output", "1", "1"],
    )
    check_cli_configuration_options(
        arguments.gz18,
        ["vehicle", "startup", "line", "data", "irregularity", "output", "1", "1"],
    )
    check_cli_configuration_options(
        arguments.irw_passive,
        ["scenario", "vehicle", "startup", "line", "data", "none", "output", "1", "1"],
        scene_record=True,
    )
    check_cli_configuration_options(
        arguments.irw_guidance,
        ["vehicle", "startup", "line", "data", "controller", "conditioner", "output", "1"],
    )
    check_cli_rejects_unknown_case(
        arguments.irw_passive,
        [
            "scenario",
            "vehicle",
            "startup",
            "line",
            "data",
            "none",
            "output",
            "1",
            "1",
        ],
    )
    check_cli_rejects_unknown_case(
        arguments.irw_guidance,
        [
            "vehicle",
            "startup",
            "line",
            "data",
            "controller",
            "conditioner",
            "output",
            "1",
        ],
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
