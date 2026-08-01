#!/usr/bin/env python3
"""Assembles the offline, CMake-only ORVD source bundle.

This is a developer packaging tool. The resulting bundle does not need Python
or Git: its root CMake superbuild consumes only the three copied local archives.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile


SCRIPT_PATH = Path(__file__).resolve()
DEFAULT_SOURCE_ROOT = SCRIPT_PATH.parents[2]
TEMPLATE_DIRECTORY = DEFAULT_SOURCE_ROOT / "distribution" / "dependencies"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--eigen-archive", type=Path, required=True)
    parser.add_argument("--fmt-archive", type=Path, required=True)
    parser.add_argument("--sundials-archive", type=Path, required=True)
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_manifest(template_directory: Path) -> dict:
    manifest_path = template_directory / "dependency_sources.json"
    with manifest_path.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    if manifest.get("schema_version") != 1:
        raise ValueError(
            f"unsupported dependency manifest schema in {manifest_path}"
        )
    records = manifest.get("dependencies")
    if not isinstance(records, list) or {item.get("key") for item in records} != {
        "eigen",
        "fmt",
        "sundials",
    }:
        raise ValueError(
            f"{manifest_path} must contain exactly eigen, fmt and sundials"
        )
    return manifest


def validate_archive(path: Path, record: dict) -> None:
    if not path.is_file():
        raise FileNotFoundError(
            f"{record['name']} archive does not exist: {path}"
        )
    actual = sha256(path)
    expected = record["sha256"]
    if actual != expected:
        raise ValueError(
            f"{record['name']} {record['version']} archive failed its transport "
            f"integrity check: expected SHA-256 {expected}, got {actual}"
        )


def copy_license_materials(archive: Path, record: dict, destination: Path) -> None:
    destination.mkdir(parents=True)
    with tarfile.open(archive, mode="r:*") as source:
        for relative_path in record["license_paths"]:
            member_name = f"{record['source_directory']}/{relative_path}"
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


def tracked_source_files(source_root: Path) -> list[Path]:
    try:
        top_level = subprocess.run(
            ["git", "-C", str(source_root), "rev-parse", "--show-toplevel"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        status = subprocess.run(
            [
                "git",
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
            ["git", "-C", str(source_root), "ls-files", "-z", "--cached"],
            check=True,
            capture_output=True,
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        raise ValueError(
            "the developer source-bundle tool requires a Git checkout and a "
            "working git executable"
        ) from error

    if Path(top_level).resolve() != source_root:
        raise ValueError(
            f"source root {source_root} is not the Git checkout root {top_level}"
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


def copy_tracked_source(source_root: Path, destination: Path) -> None:
    destination.mkdir()
    for relative_path in tracked_source_files(source_root):
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
    manifest = load_manifest(template_directory)
    archives = {
        "eigen": arguments.eigen_archive.resolve(),
        "fmt": arguments.fmt_archive.resolve(),
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

    records = {record["key"]: record for record in manifest["dependencies"]}
    for key, archive in archives.items():
        validate_archive(archive, records[key])

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
            template_directory / "dependency_sources.json",
            staging / "dependency_sources.json",
        )
        shutil.copy2(template_directory / "README.md", staging / "README.md")

        archive_directory = staging / "dependencies"
        archive_directory.mkdir()
        license_root = staging / "dependency-licenses"
        for key, record in records.items():
            shutil.copy2(archives[key], archive_directory / record["archive"])
            copy_license_materials(
                archives[key],
                record,
                license_root / f"{key}-{record['version']}",
            )

        copy_tracked_source(
            source_root, staging / "OpenRailVehicleDynamics"
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
