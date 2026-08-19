#!/usr/bin/env python3
"""Assembles the offline, CMake-only ORVD source bundle.

This is a developer packaging tool. The resulting bundle does not need Python
or Git: its root CMake superbuild consumes only the four copied local archives.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

sys.dont_write_bytecode = True

from dependency_sources import DependencySource, load_dependency_sources


SCRIPT_PATH = Path(__file__).resolve()
DEFAULT_SOURCE_ROOT = SCRIPT_PATH.parents[2]
TEMPLATE_DIRECTORY = DEFAULT_SOURCE_ROOT / "distribution" / "dependencies"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--eigen-archive", type=Path, required=True)
    parser.add_argument("--fmt-archive", type=Path, required=True)
    parser.add_argument("--nlohmann-json-archive", type=Path, required=True)
    parser.add_argument("--sundials-archive", type=Path, required=True)
    parser.add_argument("--cmake-executable", default="cmake")
    parser.add_argument("--git-executable", default="git")
    return parser.parse_args()


def require_archive(path: Path, record: DependencySource) -> None:
    if not path.is_file():
        raise FileNotFoundError(
            f"{record.name} archive does not exist: {path}"
        )


def copy_license_materials(
    archive: Path, record: DependencySource, destination: Path
) -> None:
    destination.mkdir(parents=True)
    with tarfile.open(archive, mode="r:*") as source:
        for relative_path in record.license_paths:
            member_name = f"{record.source_directory}/{relative_path}"
            try:
                member = source.getmember(member_name)
            except KeyError as error:
                raise ValueError(
                    f"{archive} has no declared licence file {member_name}"
                ) from error
            if not member.isfile():
                raise ValueError(f"declared licence is not a file: {member_name}")
            extracted = source.extractfile(member)
            if extracted is None:
                raise ValueError(f"could not read declared licence: {member_name}")
            with (destination / Path(relative_path).name).open("wb") as output:
                shutil.copyfileobj(extracted, output)


def tracked_source_files(
    source_root: Path, git_executable: str
) -> list[Path]:
    try:
        checkout_prefix = subprocess.run(
            [
                git_executable,
                "-C",
                str(source_root),
                "rev-parse",
                "--show-prefix",
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        status = subprocess.run(
            [
                git_executable,
                "-C",
                str(source_root),
                "status",
                "--porcelain=v1",
                "--untracked-files=all",
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        tracked = subprocess.run(
            [
                git_executable,
                "-C",
                str(source_root),
                "ls-files",
                "-z",
                "--cached",
            ],
            check=True,
            capture_output=True,
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        raise ValueError(
            "the developer source-bundle tool requires a Git checkout and a "
            "working git executable"
        ) from error

    # Git implementations on Windows can spell the same root as either
    # `H:/checkout` or `/h/checkout`.  Ask Git whether -C selected its root
    # instead of comparing those host-specific path dialects in Python.
    if checkout_prefix:
        raise ValueError(
            f"source root {source_root} is inside the Git checkout at prefix "
            f"{checkout_prefix!r}, not at its root"
        )
    if status:
        raise ValueError(
            "refusing to package a dirty Git checkout; commit or remove every "
            "tracked and untracked change first:\n"
            f"{status.rstrip()}"
        )

    relative_paths = [
        Path(os.fsdecode(item)) for item in tracked.split(b"\0") if item
    ]
    if not relative_paths:
        raise ValueError(f"the Git checkout at {source_root} has no tracked files")
    return relative_paths


def copy_tracked_source(
    source_root: Path, destination: Path, git_executable: str
) -> None:
    destination.mkdir()
    for relative_path in tracked_source_files(source_root, git_executable):
        source = source_root / relative_path
        output = destination / relative_path
        output.parent.mkdir(parents=True, exist_ok=True)
        if source.is_symlink():
            output.symlink_to(os.readlink(source))
        elif source.is_file():
            shutil.copy2(source, output)
        else:
            raise ValueError(
                f"tracked source entry is neither a file nor a symlink: {source}"
            )


def assemble_bundle(arguments: argparse.Namespace) -> None:
    source_root = arguments.source_root.resolve()
    output_directory = arguments.output_directory.resolve()
    template_directory = source_root / "distribution" / "dependencies"
    dependency_sources = load_dependency_sources(
        template_directory / "dependency_sources.cmake",
        cmake_executable=arguments.cmake_executable,
    )
    archives = {
        "eigen": arguments.eigen_archive.resolve(),
        "fmt": arguments.fmt_archive.resolve(),
        "nlohmann_json": arguments.nlohmann_json_archive.resolve(),
        "sundials": arguments.sundials_archive.resolve(),
    }

    if not (source_root / "CMakeLists.txt").is_file():
        raise ValueError(f"not an ORVD source root: {source_root}")
    if output_directory == source_root or source_root in output_directory.parents:
        raise ValueError("the bundle output directory must be outside the source tree")
    if output_directory.exists():
        raise FileExistsError(
            f"refusing to replace existing bundle output: {output_directory}"
        )

    records = dependency_sources.records_by_key()
    for key, archive in archives.items():
        require_archive(archive, records[key])

    output_directory.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(
            prefix=f".{output_directory.name}.staging-",
            dir=output_directory.parent,
        )
    )
    try:
        shutil.copy2(template_directory / "CMakeLists.txt", staging / "CMakeLists.txt")
        shutil.copy2(
            template_directory / "OrvdDependencySources.cmake",
            staging / "OrvdDependencySources.cmake",
        )
        shutil.copy2(
            template_directory / "dependency_sources.cmake",
            staging / "dependency_sources.cmake",
        )
        shutil.copy2(template_directory / "README.md", staging / "README.md")

        archive_directory = staging / "dependencies"
        archive_directory.mkdir()
        license_root = staging / "dependency-licenses"
        for key, record in records.items():
            shutil.copy2(archives[key], archive_directory / record.archive)
            copy_license_materials(
                archives[key],
                record,
                license_root / f"{key}-{record.version}",
            )

        copy_tracked_source(
            source_root,
            staging / "OpenRailVehicleDynamics",
            arguments.git_executable,
        )
        staging.rename(output_directory)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    assemble_bundle(parse_arguments())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
