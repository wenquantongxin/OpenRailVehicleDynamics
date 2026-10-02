#!/usr/bin/env python3
"""Verify the three vehicle CLIs' positional layouts and configuration options."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


def check_options(executable: Path, base: list[str], *, scene_record: bool = False) -> None:
    for tail, message in (
        (["--integration-config"], "requires one non-empty path"),
        (["--integration-config", ""], "requires one non-empty path"),
        (["--integration-config", "--scene-record"], "requires one non-empty path"),
        (["--integration-config", "one.json", "--integration-config", "two.json"],
         "may be specified only once"),
        (["--unknown-option"], "unknown option"),
        (["radau5_fine"], "usage:"),
        (["radau5_fine", "--integration-config", "one.json"], "usage:"),
    ):
        completed = subprocess.run([str(executable), *base, *tail], check=False,
                                   capture_output=True, text=True)
        if completed.returncode != 2 or message not in completed.stderr:
            raise AssertionError(
                f"{executable.name}: invalid arguments {tail!r} did not report {message!r}: "
                f"status={completed.returncode}, stderr={completed.stderr!r}")
    # Nonexistent assets make valid layouts fail in the runner (1), rather than
    # during command-line parsing (2). Runtime publication has its own C++ tests.
    valid_commands = [base]
    for insertion in (0, len(base) // 2, len(base)):
        command = [*base[:insertion], "--integration-config", "unread config.json",
                   *base[insertion:]]
        valid_commands.append(command)
        if scene_record:
            valid_commands.append(["--scene-record", *command])
    if scene_record:
        valid_commands.append([*base, "--scene-record"])
    for command in valid_commands:
        completed = subprocess.run([str(executable), *command], check=False,
                                   capture_output=True, text=True)
        if completed.returncode != 1:
            raise AssertionError(
                f"{executable.name}: valid option layout did not reach the runner: "
                f"status={completed.returncode}, stderr={completed.stderr!r}")
    # Every CLI ends in a positive integer time value (duration or sampling
    # period), rejected before any assets are read.
    malformed = [*base[:-1], "0"]
    completed = subprocess.run([str(executable), *malformed], check=False,
                               capture_output=True, text=True)
    if completed.returncode != 2 or "positive integer" not in completed.stderr:
        raise AssertionError(f"{executable.name}: invalid time value was not rejected")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gz18", type=Path, required=True)
    parser.add_argument("--irw-passive", type=Path, required=True)
    parser.add_argument("--irw-guidance", type=Path, required=True)
    arguments = parser.parse_args()
    check_options(arguments.gz18,
                  ["vehicle", "startup", "line", "data", "irregularity", "output", "1", "1"])
    check_options(arguments.irw_passive,
                  ["scenario", "vehicle", "startup", "line", "data", "none", "output", "1", "1"],
                  scene_record=True)
    check_options(arguments.irw_guidance,
                  ["vehicle", "startup", "line", "data", "controller", "conditioner", "output", "1"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
