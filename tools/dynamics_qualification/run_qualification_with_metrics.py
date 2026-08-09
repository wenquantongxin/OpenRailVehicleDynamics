#!/usr/bin/env python3
"""Run one qualification process and publish its external execution identity."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import subprocess
import sys
import time
from typing import Iterable


RUNNER_ARGUMENT_COUNT = 8
OUTPUT_DIRECTORY_ARGUMENT_INDEX = 5


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


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--identity-output", type=Path, required=True)
    parser.add_argument("--orvd-revision", required=True)
    parser.add_argument("--build-type", choices=("Release",), required=True)
    parser.add_argument("--compiler-identity", required=True)
    parser.add_argument("--cpu-affinity", required=True)
    parser.add_argument("runner_arguments", nargs=argparse.REMAINDER)
    arguments = parser.parse_args(list(argv))
    if arguments.runner_arguments[:1] == ["--"]:
        arguments.runner_arguments = arguments.runner_arguments[1:]
    if not arguments.runner_arguments:
        parser.error("runner arguments are empty")
    if len(arguments.runner_arguments) != RUNNER_ARGUMENT_COUNT:
        parser.error(
            "the GZ18 qualification runner requires exactly eight arguments"
        )
    return arguments


def main(argv: Iterable[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    executable = arguments.executable.resolve(strict=True)
    if not executable.is_file() or not os.access(executable, os.X_OK):
        print(f"qualification executable is not executable: {executable}",
              file=sys.stderr)
        return 2
    try:
        affinity = parse_affinity(arguments.cpu_affinity)
        os.sched_setaffinity(0, affinity)
        applied_affinity = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError, ValueError) as error:
        print(f"could not set CPU affinity: {error}", file=sys.stderr)
        return 2

    environment_keys = (
        "OMP_NUM_THREADS", "OMP_DYNAMIC", "OMP_PLACES", "OMP_PROC_BIND",
        "OMP_THREAD_LIMIT",
    )
    openmp_environment = {
        key: os.environ[key] for key in environment_keys if key in os.environ
    }
    before_usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    begin = time.perf_counter()
    completed = subprocess.run(
        [str(executable), *arguments.runner_arguments], check=False
    )
    wall_seconds = time.perf_counter() - begin
    after_usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    child_user_seconds = after_usage.ru_utime - before_usage.ru_utime
    child_system_seconds = after_usage.ru_stime - before_usage.ru_stime
    artifact_directory = Path(
        arguments.runner_arguments[OUTPUT_DIRECTORY_ARGUMENT_INDEX]
    ).resolve()
    identity = {
        "schema_version": 1,
        "orvd_revision": arguments.orvd_revision,
        "build_type": arguments.build_type,
        "compiler": arguments.compiler_identity,
        "hardware": processor_identity(),
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
        ),
        "maximum_resident_set_kilobytes": after_usage.ru_maxrss,
        "executable": str(executable),
        "executable_sha256": sha256_file(executable),
        "exit_status": completed.returncode,
    }
    try:
        write_identity(arguments.identity_output.resolve(), identity)
    except OSError as error:
        print(f"could not publish execution identity: {error}", file=sys.stderr)
        return 2 if completed.returncode == 0 else completed.returncode
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
