"""Loads the one current dependency-source declaration through CMake.

The human-maintained format is ``dependency_sources.cmake``.  Both the offline
superbuild and this developer tool execute the same declaration validator; this
module only turns its trusted, private export into immutable Python values.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
import tempfile


EXPECTED_DEPENDENCY_KEYS = ("eigen", "fmt", "nlohmann_json", "sundials")


@dataclass(frozen=True)
class DependencySource:
    key: str
    name: str
    version: str
    archive: str
    source_directory: str
    source_url: str
    license_paths: tuple[str, ...]


@dataclass(frozen=True)
class DependencySources:
    records: tuple[DependencySource, ...]

    def records_by_key(self) -> dict[str, DependencySource]:
        return {record.key: record for record in self.records}


def _read_scalar(path: Path) -> str:
    try:
        value = path.read_text(encoding="utf-8")
    except OSError as error:
        raise ValueError(f"dependency export is missing {path}") from error
    if not value:
        raise ValueError(f"dependency export field is empty: {path}")
    return value


def _read_export(directory: Path) -> DependencySources:
    if not (directory / "COMPLETE").is_file():
        raise ValueError("CMake did not publish a complete dependency export")
    keys = tuple((directory / "keys.txt").read_text(encoding="utf-8").splitlines())
    if set(keys) != set(EXPECTED_DEPENDENCY_KEYS) or len(keys) != len(
        EXPECTED_DEPENDENCY_KEYS
    ):
        raise ValueError(f"dependency export has the wrong key set: {keys}")

    records: list[DependencySource] = []
    for key in keys:
        record_directory = directory / key
        license_paths = tuple(
            (record_directory / "license_paths.txt")
            .read_text(encoding="utf-8")
            .splitlines()
        )
        if not license_paths or any(not path for path in license_paths):
            raise ValueError(f"dependency export has invalid licence paths for {key}")
        record = DependencySource(
            key=_read_scalar(record_directory / "key.txt"),
            name=_read_scalar(record_directory / "name.txt"),
            version=_read_scalar(record_directory / "version.txt"),
            archive=_read_scalar(record_directory / "archive.txt"),
            source_directory=_read_scalar(record_directory / "source_directory.txt"),
            source_url=_read_scalar(record_directory / "source_url.txt"),
            license_paths=license_paths,
        )
        if record.key != key:
            raise ValueError(
                f"dependency export directory {key!r} contains key {record.key!r}"
            )
        records.append(record)
    return DependencySources(tuple(records))


def load_dependency_sources(
    manifest_path: Path,
    *,
    cmake_executable: str = "cmake",
    source_module: Path | None = None,
    export_script: Path | None = None,
) -> DependencySources:
    manifest_path = manifest_path.resolve()
    template_directory = manifest_path.parent
    source_module = (
        template_directory / "OrvdDependencySources.cmake"
        if source_module is None
        else source_module.resolve()
    )
    export_script = (
        template_directory / "export_dependency_sources.cmake"
        if export_script is None
        else export_script.resolve()
    )
    for required_path in (manifest_path, source_module, export_script):
        if not required_path.is_file():
            raise ValueError(f"dependency source input does not exist: {required_path}")

    with tempfile.TemporaryDirectory(prefix="orvd-dependency-sources-") as temporary:
        export_directory = Path(temporary) / "export"
        command = [
            cmake_executable,
            f"-DORVD_DEPENDENCY_SOURCE_MODULE={source_module.as_posix()}",
            f"-DORVD_DEPENDENCY_SOURCE_MANIFEST={manifest_path.as_posix()}",
            f"-DORVD_DEPENDENCY_EXPORT_DIRECTORY={export_directory.as_posix()}",
            "-P",
            str(export_script),
        ]
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
            )
        except OSError as error:
            raise ValueError(
                f"could not execute CMake dependency-source validator: {cmake_executable}"
            ) from error
        if completed.returncode != 0:
            diagnostic = (completed.stderr or completed.stdout).strip()
            raise ValueError(
                "dependency source declaration was rejected by the shared CMake "
                f"validator:\n{diagnostic}"
            )
        return _read_export(export_directory)
