#!/usr/bin/env python3
"""Compiles the admitted Drake translation units and links whatever objects result.

Reading includes says which files the boundary reaches. It does not say whether
those files compile, and it never says whether the result links. This asks a
compiler and a linker instead.

Staging is the point. Every admitted file — and nothing else — is copied into a
temporary tree, and that tree is the only Drake include directory the compiler
gets. A run against the upstream clone would let a declined file satisfy an
include and report success the boundary has not earned.

Failures do not stop the compilation pass: each translation unit is attempted,
so one broken file cannot hide the state of the remaining translation units.
The compiler's own output is kept verbatim rather than parsed into categories,
because a regular expression that classifies diagnostics is one compiler
release away from lying.

The link step exists because compiling proves less than it appears to. Every
object that was produced goes on the link line together with a temporary `main`
that provides nothing but an entry point, with no static archive, no
link-time optimisation and no section garbage collection — an unreferenced
function whose body calls a missing symbol must still break the link. That link
covers only the subset that compiled today, and this tool says so; proving that
the whole boundary compiles and links is G28's job.

Nothing survives the run: objects, logs and the staged tree are temporary, and
the report is for the decision being made now. No symbol table, count or
allowlist is written down, because a number recorded today becomes a gate that
passes for the wrong reason tomorrow.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

# Importing the sibling analyzer must not leave bytecode in the source tree.
sys.dont_write_bytecode = True

from calculate_required_drake_source_closure import (
    ADMITTED_DISPOSITIONS,
    HEADER_SUFFIXES,
    IMPLEMENTATION_SUFFIXES,
    LedgerError,
    parse_ledger,
    verify_source_commit,
)

# The product is C++23; compiling the boundary under an older standard would
# answer a question nobody asked.
LANGUAGE_STANDARD = "c++23"


@dataclass
class StageOutcome:
    staged_paths: list[str] = field(default_factory=list)
    missing_paths: list[str] = field(default_factory=list)


@dataclass
class TranslationUnitCompilationOutcome:
    translation_unit_path: str
    succeeded: bool
    object_path: Path | None
    diagnostics: str


def sanitized_compiler_environment() -> dict[str, str]:
    """Removes implicit include and library paths from the compiler environment."""
    environment = os.environ.copy()
    for variable_name in (
        "CPATH",
        "CPLUS_INCLUDE_PATH",
        "C_INCLUDE_PATH",
        "OBJC_INCLUDE_PATH",
        "LIBRARY_PATH",
    ):
        environment.pop(variable_name, None)
    return environment


def compiler_default_search_exposes_drake_headers(
    compiler: str,
    classified_paths: list[str],
    compiler_environment: dict[str, str],
) -> bool:
    """Checks whether built-in compiler paths expose a Drake installation."""
    header_paths = [path for path in classified_paths if path.endswith(HEADER_SUFFIXES)]
    visibility_checks = "".join(
        f'#if __has_include("drake/{path}")\n'
        "#error ORVD_UNSTAGED_DRAKE_HEADER_VISIBLE\n"
        "#endif\n"
        for path in header_paths
    )
    compiler_process = subprocess.run(
        [compiler, f"-std={LANGUAGE_STANDARD}", "-E", "-x", "c++", "-"],
        input=visibility_checks,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
        env=compiler_environment,
    )
    return "ORVD_UNSTAGED_DRAKE_HEADER_VISIBLE" in compiler_process.stdout


def stage_admitted_files(
    admitted_paths: list[str], source_root: Path, staging_root: Path
) -> StageOutcome:
    """Copies exactly the admitted files into an otherwise empty Drake tree.

    A file the ledger admits but the source tree lacks is a staging failure, not
    a compile failure: nothing downstream can be trusted once the input set is
    wrong.
    """
    outcome = StageOutcome()
    drake_root = staging_root / "drake"
    for path in admitted_paths:
        source_file = source_root / path
        if not source_file.is_file():
            outcome.missing_paths.append(path)
            continue
        destination = drake_root / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source_file, destination)
        outcome.staged_paths.append(path)
    return outcome


def compile_translation_unit(
    relative_path: str,
    staging_root: Path,
    object_directory: Path,
    compiler: str,
    third_party_include_directories: list[Path],
    compiler_environment: dict[str, str],
) -> TranslationUnitCompilationOutcome:
    # Preserve the source directory structure. Flattening with underscores makes
    # e.g. a/b_c.cc and a_b/c.cc overwrite the same object.
    object_path = object_directory / f"{relative_path}.o"
    object_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        compiler,
        f"-std={LANGUAGE_STANDARD}",
        "-c",
        str(staging_root / "drake" / relative_path),
        "-o",
        str(object_path),
        "-I",
        str(staging_root),
    ]
    for directory in third_party_include_directories:
        command += ["-I", str(directory)]
    # The first-party warning policy deliberately does not apply here: this is
    # upstream source compiled as found, and -Werror would turn an upstream
    # style choice into a boundary failure.
    compiler_process = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
        env=compiler_environment,
    )
    object_was_produced = object_path.is_file() and object_path.stat().st_size > 0
    succeeded = compiler_process.returncode == 0 and object_was_produced
    if compiler_process.returncode == 0 and not object_was_produced:
        diagnostics = f"{compiler} reported success but produced no object file"
    else:
        diagnostics = compiler_process.stdout
    return TranslationUnitCompilationOutcome(
        relative_path, succeeded, object_path if succeeded else None, diagnostics
    )


def link_generated_objects(
    object_paths: list[Path],
    link_workspace: Path,
    compiler: str,
    third_party_library_arguments: list[str],
    compiler_environment: dict[str, str],
) -> tuple[bool, str]:
    """Links every object produced, with an entry point and nothing else.

    No archive is built, so nothing can be dropped for being unreferenced, and
    section garbage collection stays off for the same reason: a function nobody
    calls must still drag its undefined symbols into the link.
    """
    entry_point_source = link_workspace / "provide_entry_point_only.cc"
    entry_point_source.write_text(
        "// Supplies an entry point so the objects under test can be linked.\n"
        "// It calls nothing: what must be resolved is what those objects\n"
        "// reference, not what this file exercises.\n"
        "int main() { return 0; }\n",
        encoding="utf-8",
    )
    executable = link_workspace / "linked_admitted_objects"
    command = [
        compiler,
        f"-std={LANGUAGE_STANDARD}",
        str(entry_point_source),
        *[str(path) for path in object_paths],
        "-o",
        str(executable),
    ]
    command += third_party_library_arguments
    linker_process = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
        env=compiler_environment,
    )
    return linker_process.returncode == 0, linker_process.stdout


def compile_and_link_admitted_translation_units(
    ledger_path: Path,
    source_root: Path,
    compiler: str,
    third_party_include_directories: list[Path],
    third_party_library_arguments: list[str],
    show_diagnostics: bool,
) -> int:
    try:
        ledger = parse_ledger(ledger_path.read_text(encoding="utf-8"))
    except (OSError, LedgerError) as error:
        print(f"STAGING: ledger unusable: {error}", file=sys.stderr)
        return 2
    if not source_root.is_dir():
        print(f"STAGING: not a directory: {source_root}", file=sys.stderr)
        return 2
    revision_mismatch = verify_source_commit(source_root, ledger.source_commit)
    if revision_mismatch is not None:
        print(
            f"STAGING: source revision mismatch: {revision_mismatch}", file=sys.stderr
        )
        return 2
    compiler_environment = sanitized_compiler_environment()
    if compiler_default_search_exposes_drake_headers(
        compiler,
        sorted(ledger.dispositions),
        compiler_environment,
    ):
        print(
            "STAGING: compiler default include search exposes a Drake tree",
            file=sys.stderr,
        )
        return 2
    for include_directory in third_party_include_directories:
        if (include_directory / "drake").exists():
            print(
                "STAGING: third-party include directory exposes a Drake tree: "
                f"{include_directory}",
                file=sys.stderr,
            )
            return 2

    admitted_paths = sorted(
        path
        for path, disposition in ledger.dispositions.items()
        if disposition in ADMITTED_DISPOSITIONS
    )
    if not admitted_paths:
        print("STAGING: the ledger admits no file", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary_workspace = Path(temporary_directory)
        staging_root = temporary_workspace / "staged"
        object_directory = temporary_workspace / "objects"
        object_directory.mkdir(parents=True, exist_ok=True)

        staging_outcome = stage_admitted_files(
            admitted_paths, source_root, staging_root
        )
        for path in staging_outcome.missing_paths:
            print(
                f"STAGING: {path} is admitted but absent from the source tree",
                file=sys.stderr,
            )
        print(f"admitted files:        {len(admitted_paths)}")
        print(f"staged:                {len(staging_outcome.staged_paths)}")
        if staging_outcome.missing_paths:
            return 1

        translation_units = [
            path
            for path in staging_outcome.staged_paths
            if path.endswith(IMPLEMENTATION_SUFFIXES)
        ]
        if not translation_units:
            print(
                "COMPILE: the ledger admits no translation unit",
                file=sys.stderr,
            )
            return 1

        compilation_outcomes = [
            compile_translation_unit(
                path,
                staging_root,
                object_directory,
                compiler,
                third_party_include_directories,
                compiler_environment,
            )
            for path in translation_units
        ]
        successful_compilations = [
            outcome for outcome in compilation_outcomes if outcome.succeeded
        ]
        failed_compilations = [
            outcome for outcome in compilation_outcomes if not outcome.succeeded
        ]

        for outcome in failed_compilations:
            print(
                f"COMPILE: {outcome.translation_unit_path} failed",
                file=sys.stderr,
            )
            if show_diagnostics:
                print(outcome.diagnostics, file=sys.stderr)

        print(f"translation units:     {len(translation_units)}")
        print(f"compiled:              {len(successful_compilations)}")
        print(f"compile failures:      {len(failed_compilations)}")

        link_succeeded = False
        link_diagnostics = ""
        if successful_compilations:
            link_workspace = temporary_workspace / "link"
            link_workspace.mkdir(parents=True, exist_ok=True)
            link_succeeded, link_diagnostics = link_generated_objects(
                [
                    outcome.object_path
                    for outcome in successful_compilations
                    if outcome.object_path
                ],
                link_workspace,
                compiler,
                third_party_library_arguments,
                compiler_environment,
            )
            # Said plainly because the number invites the wrong reading: this
            # link covers only what compiled in this run. A clean result here is
            # not a statement about the boundary as a whole.
            print(
                f"link of the {len(successful_compilations)} object(s) that compiled: "
                f"{'succeeded' if link_succeeded else 'FAILED'}"
            )
            if not link_succeeded:
                print("LINK: generated objects did not link", file=sys.stderr)
                if show_diagnostics:
                    print(link_diagnostics, file=sys.stderr)
        else:
            print("link skipped: no object was produced")

        if failed_compilations or (successful_compilations and not link_succeeded):
            return 1
        return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Compile the admitted Drake translation units and link the "
        "objects that result."
    )
    parser.add_argument("--drake-source-root", required=True, type=Path)
    parser.add_argument("--disposition-ledger", required=True, type=Path)
    parser.add_argument(
        "--compiler",
        default="c++",
        help="GNU-compatible compiler executable to invoke",
    )
    parser.add_argument(
        "--third-party-include-directory",
        action="append",
        default=[],
        type=Path,
        help="an include directory for a dependency outside the staged tree "
        "(Eigen, for instance); repeatable",
    )
    parser.add_argument(
        "--third-party-library-argument",
        action="append",
        default=[],
        help="a library-selection argument appended to the link command; repeatable",
    )
    parser.add_argument(
        "--show-diagnostics",
        action="store_true",
        help="print the compiler and linker output verbatim",
    )
    arguments = parser.parse_args(argv)
    return compile_and_link_admitted_translation_units(
        arguments.disposition_ledger,
        arguments.drake_source_root,
        arguments.compiler,
        arguments.third_party_include_directory,
        arguments.third_party_library_argument,
        arguments.show_diagnostics,
    )


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
