#!/usr/bin/env python3
"""Verify closed qualification-wrapper layouts and CLI case rejection."""

from __future__ import annotations

import argparse
import builtins
import contextlib
import importlib.util
import io
from pathlib import Path
import subprocess
import sys
from types import ModuleType


sys.dont_write_bytecode = True

EXPECTED_LAYOUTS = {
    "gz18": {8: 5, 9: 5},
    "irw-passive-scenario": {9: 6, 10: 6},
    "irw-r300-aar5-v60-100hz-full-state-guidance": {8: 6, 9: 6},
}


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
        "--orvd-revision",
        "test",
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


def require_parser_exit_two(function: object, arguments: list[str]) -> None:
    with contextlib.redirect_stderr(io.StringIO()):
        try:
            function(arguments)  # type: ignore[operator]
        except SystemExit as error:
            if error.code == 2:
                return
            raise AssertionError(
                f"argument parser exited with {error.code}, expected 2"
            ) from error
    raise AssertionError("argument parser accepted an invalid runner count")


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


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metrics-wrapper", type=Path, required=True)
    parser.add_argument("--gz18", type=Path, required=True)
    parser.add_argument("--irw-passive", type=Path, required=True)
    parser.add_argument("--irw-guidance", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    check_wrapper_layouts(load_metrics_wrapper(arguments.metrics_wrapper))
    without_resource = load_metrics_wrapper_without_posix_resource(
        arguments.metrics_wrapper
    )
    if without_resource.posix_resource is not None:
        raise AssertionError("wrapper did not tolerate missing POSIX resource")
    check_wrapper_layouts(without_resource)
    check_cli_rejects_unknown_case(
        arguments.gz18,
        ["vehicle", "startup", "line", "data", "irregularity", "output", "1", "1"],
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
