#!/usr/bin/env python3
"""Verifies the single CMake dependency-source declaration and validator."""

from __future__ import annotations

import argparse
import io
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

sys.dont_write_bytecode = True

from dependency_sources import DependencySource, load_dependency_sources


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cmake", required=True)
    parser.add_argument("--git-executable", required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    return parser.parse_args()


def render_record(record: DependencySource) -> str:
    license_lines = "\n".join(
        f"    LICENSE_PATH {path}" for path in record.license_paths
    )
    return f"""orvd_declare_dependency(
    KEY {record.key}
    NAME {record.name}
    VERSION {record.version}
    ARCHIVE {record.archive}
    SOURCE_DIRECTORY {record.source_directory}
    SOURCE_URL {record.source_url}
{license_lines}
)"""


def render_manifest(records: tuple[DependencySource, ...]) -> str:
    return "\n\n".join(render_record(record) for record in records) + "\n"


def replace_once(text: str, old: str, new: str) -> str:
    if text.count(old) != 1:
        raise AssertionError(f"test mutation expected one occurrence of {old!r}")
    return text.replace(old, new, 1)


def require_rejected(
    name: str,
    manifest_text: str,
    *,
    cmake: str,
    module: Path,
    export_script: Path,
    directory: Path,
) -> None:
    manifest = directory / f"{name}.cmake"
    manifest.write_text(manifest_text, encoding="utf-8")
    try:
        load_dependency_sources(
            manifest,
            cmake_executable=cmake,
            source_module=module,
            export_script=export_script,
        )
    except ValueError:
        return
    raise AssertionError(f"invalid dependency declaration was accepted: {name}")


def verify_rejections(
    records: tuple[DependencySource, ...],
    *,
    cmake: str,
    module: Path,
    export_script: Path,
    directory: Path,
) -> None:
    valid = render_manifest(records)
    first = records[0]
    second = records[1]
    first_license_block = "\n".join(
        f"    LICENSE_PATH {path}" for path in first.license_paths
    )
    mutations = {
        "missing_name": replace_once(valid, f"    NAME {first.name}\n", ""),
        "duplicate_version": replace_once(
            valid,
            f"    VERSION {first.version}\n",
            f"    VERSION {first.version}\n    VERSION {first.version}\n",
        ),
        "unknown_field": replace_once(
            valid,
            f"    VERSION {first.version}\n",
            f"    VERSION {first.version}\n    CHECKSUM absent\n",
        ),
        "empty_name": replace_once(
            valid, f"    NAME {first.name}\n", "    NAME \"\"\n"
        ),
        "multiple_name_values": replace_once(
            valid,
            f"    NAME {first.name}\n",
            f"    NAME {first.name} unexpected\n",
        ),
        "empty_license_list": replace_once(
            valid,
            first_license_block + "\n",
            "",
        ),
        "license_path_with_two_values": replace_once(
            valid,
            f"    LICENSE_PATH {first.license_paths[0]}\n",
            f"    LICENSE_PATH {first.license_paths[0]} unexpected\n",
        ),
        "unknown_field_after_licenses": replace_once(
            valid,
            f"    LICENSE_PATH {first.license_paths[-1]}\n",
            f"    LICENSE_PATH {first.license_paths[-1]}\n"
            "    CHECKSUM absent\n",
        ),
        "duplicate_dependency_key": replace_once(
            valid, f"    KEY {second.key}\n", f"    KEY {first.key}\n"
        ),
        "missing_dependency": render_manifest(records[:-1]),
        "unknown_dependency": replace_once(
            valid, f"    KEY {records[-1].key}\n", "    KEY boost\n"
        ),
        "archive_traversal": replace_once(
            valid,
            f"    ARCHIVE {first.archive}\n",
            f"    ARCHIVE ../{first.archive}\n",
        ),
        "version_traversal": replace_once(
            valid,
            f"    VERSION {first.version}\n",
            f"    VERSION ../{first.version}\n",
        ),
        "source_directory_traversal": replace_once(
            valid,
            f"    SOURCE_DIRECTORY {first.source_directory}\n",
            "    SOURCE_DIRECTORY ../source\n",
        ),
        "license_traversal": replace_once(
            valid,
            f"    LICENSE_PATH {first.license_paths[0]}\n",
            "    LICENSE_PATH ../LICENSE\n",
        ),
        "duplicate_archive": replace_once(
            valid,
            f"    ARCHIVE {second.archive}\n",
            f"    ARCHIVE {first.archive}\n",
        ),
        "duplicate_license_path": replace_once(
            valid,
            f"    LICENSE_PATH {first.license_paths[0]}\n",
            f"    LICENSE_PATH {first.license_paths[0]}\n"
            f"    LICENSE_PATH {first.license_paths[0]}\n",
        ),
        "license_output_collision": replace_once(
            valid,
            f"    LICENSE_PATH {first.license_paths[0]}\n",
            "    LICENSE_PATH legal/LICENSE\n"
            "    LICENSE_PATH notices/LICENSE\n",
        ),
    }
    for name, manifest_text in mutations.items():
        require_rejected(
            name,
            manifest_text,
            cmake=cmake,
            module=module,
            export_script=export_script,
            directory=directory,
        )


def run_checked(command: list[str], failure: str) -> None:
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        diagnostic = (completed.stderr or completed.stdout).strip()
        raise AssertionError(f"{failure}:\n{diagnostic}")


def write_test_archive(path: Path, record: DependencySource) -> None:
    mode = "w:xz" if path.name.endswith(".tar.xz") else "w:gz"
    with tarfile.open(path, mode=mode) as archive:
        for relative_path in record.license_paths:
            payload = f"licence fixture for {record.key}: {relative_path}\n".encode()
            member = tarfile.TarInfo(
                f"{record.source_directory}/{relative_path}"
            )
            member.size = len(payload)
            archive.addfile(member, io.BytesIO(payload))


def verify_real_source_bundle_assembly(
    source_root: Path,
    cmake: str,
    git_executable: str,
    records: tuple[DependencySource, ...],
    directory: Path,
) -> None:
    template = source_root / "distribution" / "dependencies"
    fixture_source = directory / "source-checkout"
    fixture_template = fixture_source / "distribution" / "dependencies"
    fixture_tools = fixture_source / "tools" / "package_distribution"
    fixture_template.mkdir(parents=True)
    fixture_tools.mkdir(parents=True)
    for name in (
        "CMakeLists.txt",
        "README.md",
        "OrvdDependencySources.cmake",
        "dependency_sources.cmake",
        "export_dependency_sources.cmake",
    ):
        shutil.copy2(template / name, fixture_template / name)
    for name in ("assemble_source_bundle.py", "dependency_sources.py"):
        shutil.copy2(
            source_root / "tools" / "package_distribution" / name,
            fixture_tools / name,
        )
    (fixture_source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.24)\nproject(placeholder LANGUAGES NONE)\n",
        encoding="utf-8",
    )
    run_checked(
        [git_executable, "init", "--quiet", str(fixture_source)],
        "could not initialise the clean source fixture",
    )
    run_checked(
        [git_executable, "-C", str(fixture_source), "add", "."],
        "could not stage the clean source fixture",
    )
    run_checked(
        [
            git_executable,
            "-C",
            str(fixture_source),
            "-c",
            "user.name=ORVD verifier",
            "-c",
            "user.email=orvd-verifier@example.invalid",
            "commit",
            "--quiet",
            "-m",
            "fixture",
        ],
        "could not commit the clean source fixture",
    )

    archive_directory = directory / "input-archives"
    archive_directory.mkdir()
    archives: dict[str, Path] = {}
    for record in records:
        archive = archive_directory / record.archive
        write_test_archive(archive, record)
        archives[record.key] = archive

    bundle = directory / "assembled-bundle"
    run_checked(
        [
            sys.executable,
            str(fixture_tools / "assemble_source_bundle.py"),
            "--source-root",
            str(fixture_source),
            "--output-directory",
            str(bundle),
            "--eigen-archive",
            str(archives["eigen"]),
            "--fmt-archive",
            str(archives["fmt"]),
            "--nlohmann-json-archive",
            str(archives["nlohmann_json"]),
            "--sundials-archive",
            str(archives["sundials"]),
            "--cmake-executable",
            cmake,
            "--git-executable",
            git_executable,
        ],
        "the real source-bundle assembler rejected valid inputs",
    )
    if (bundle / "dependency_sources.json").exists():
        raise AssertionError("the assembled bundle resurrected the removed JSON format")
    for record in records:
        for relative_path in record.license_paths:
            installed_license = (
                bundle
                / "dependency-licenses"
                / f"{record.key}-{record.version}"
                / Path(relative_path).name
            )
            if not installed_license.is_file():
                raise AssertionError(
                    f"the assembled bundle omitted licence {installed_license}"
                )
    run_checked(
        [cmake, "-S", str(bundle), "-B", str(directory / "bundle-build")],
        "the generated offline superbuild rejected its shared declaration",
    )


def main() -> int:
    arguments = parse_arguments()
    source_root = arguments.source_root.resolve()
    template = source_root / "distribution" / "dependencies"
    manifest = template / "dependency_sources.cmake"
    module = template / "OrvdDependencySources.cmake"
    export_script = template / "export_dependency_sources.cmake"
    production = load_dependency_sources(
        manifest,
        cmake_executable=arguments.cmake,
        source_module=module,
        export_script=export_script,
    )
    with tempfile.TemporaryDirectory(prefix="orvd-dependency-contract-") as temporary:
        directory = Path(temporary)
        verify_rejections(
            production.records,
            cmake=arguments.cmake,
            module=module,
            export_script=export_script,
            directory=directory,
        )
        verify_real_source_bundle_assembly(
            source_root,
            arguments.cmake,
            arguments.git_executable,
            production.records,
            directory,
        )
    print("dependency source declaration contract verified")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, ValueError) as error:
        print(f"dependency source verification failed: {error}", file=sys.stderr)
        raise SystemExit(1) from error
