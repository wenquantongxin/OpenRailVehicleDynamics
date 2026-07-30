#!/usr/bin/env python3
"""Compiles the landed double-only source and reports what still depends on a runtime.

This is not the G11 probe. That one stages files out of a pinned upstream clone to
ask what the *admission ledger* would compile; this one compiles the source that
actually landed in the repository, after the double-only surgery, to ask where the
real compile frontier is. The two must stay separate: relaxing the G11 probe's
pinned-revision check so it could also read the landed tree would let one tool
confuse two source identities, and neither answer would be trustworthy again.

The landed tree is the only Drake include directory the compiler gets. Any other
Drake tree in scope — through an argument, through the environment, or through the
compiler's own defaults — would let a header this boundary deliberately does not
have satisfy an include, and the run would report a frontier the boundary has not
earned. That is checked by asking the compiler what it can see, with
`__has_include` on headers known to be absent from the landed tree, rather than by
matching path strings: a Drake installation somewhere unexpected is exactly the
case a string match misses.

Failures do not stop the pass. Every translation unit is attempted so that one
blocked file cannot hide the state of the rest, and the compiler's own text is
kept verbatim rather than sorted into categories by a regular expression that a
compiler release could invalidate.

While the objects still exist, they are checked for symbols that the double-only
boundary must not contain. The check uses qualified names: a bare search for
`symbolic` would match `Eigen::symbolic::SymbolExpr`, which is Eigen's own index
expression machinery and has nothing to do with Drake's symbolic scalar. Both
defined and undefined symbols are examined, because a translation unit that merely
*calls* something forbidden is as much a violation as one that defines it.
Constructs that never reach the symbol table — a `default_scalars.h` include, a
`scalar_predicate` branch — are caught by a separate source scan.

Until G20-G28 replace the runtime, the complete landed tree has translation units
that cannot compile and the tool returns non-zero. That is not hidden behind an
expected-failure wrapper, because a test that is allowed to fail stops being read.

Nothing is written down. No pass count, no symbol list, no allowlist: a number
recorded today becomes a gate that passes for the wrong reason tomorrow. Objects
and logs live in the temporary directory the environment provides and are gone
when the run ends.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

sys.dont_write_bytecode = True

from calculate_required_drake_source_closure import source_lines_without_comments

LANGUAGE_STANDARD = "c++23"

# Headers that upstream Drake has and this boundary deliberately does not. If the
# compiler can see any of them, some other Drake tree is in scope.
ABSENT_HEADER_CANARIES = (
    "drake/common/default_scalars.h",
    "drake/common/autodiff.h",
    "drake/systems/framework/leaf_system.h",
    "drake/multibody/plant/multibody_plant.h",
)

# Qualified names. `drake::symbolic::` and not `symbolic`, because Eigen has a
# `symbolic` namespace of its own for `Eigen::last` and `Eigen::seq` index
# expressions; `CloneToScalar` covers `DoCloneToScalar` and
# `TemplatedDoCloneToScalar` without naming them.
FORBIDDEN_SYMBOL_NAMES = (
    "drake::symbolic::",
    "Eigen::AutoDiffScalar<",
    "AutoDiffXd",
    "CloneToScalar",
    "ToAutoDiffXd",
    "ToSymbolic",
    "SystemScalarConverter",
    "ScalarConversionTraits",
    "ExtractDouble",
    "DiscardGradient",
)

# Scalar machinery that resolves at compile time and so never reaches an object's
# symbol table. Searched in the source instead.
FORBIDDEN_SOURCE_TOKENS = (
    "default_scalars.h",
    "scalar_predicate",
    "scalar_conversion",
    "@tparam_default_scalar",
    "@tparam_nonsymbolic_scalar",
)

DRAKE_INCLUDE_PATTERN = re.compile(
    r'^\s*#\s*include\s*"(drake/[^"]+)"'
)
SYSTEMS_TYPE_PATTERN = re.compile(r"\bsystems::([A-Za-z_][A-Za-z0-9_]*)")


@dataclass
class TranslationUnitOutcome:
    relative_path: str
    succeeded: bool
    object_path: Path | None
    diagnostics: str


def sanitized_compiler_environment() -> dict[str, str]:
    """Drops the environment variables that add include and library paths."""
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


def drake_headers_visible_with(
    compiler: str,
    include_arguments: list[str],
    compiler_environment: dict[str, str],
    headers_expected_absent: tuple[str, ...],
) -> bool:
    """Asks the compiler whether an absent Drake header is reachable anyway."""
    probe_source = "".join(
        f'#if __has_include("{header}")\n'
        "#error ORVD_FOREIGN_DRAKE_HEADER_VISIBLE\n"
        "#endif\n"
        for header in headers_expected_absent
    )
    compiler_process = subprocess.run(
        [compiler, f"-std={LANGUAGE_STANDARD}", *include_arguments, "-E", "-x", "c++", "-"],
        input=probe_source,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
        env=compiler_environment,
    )
    if "ORVD_FOREIGN_DRAKE_HEADER_VISIBLE" in compiler_process.stdout:
        return True
    if compiler_process.returncode != 0:
        raise RuntimeError(
            "the compiler could not perform the foreign-Drake preflight:\n"
            + compiler_process.stdout.strip()
        )
    return False


def landed_translation_units(landed_root: Path) -> list[Path]:
    return sorted(path for path in landed_root.rglob("*.cc") if path.is_file())


def landed_sources(landed_root: Path) -> list[Path]:
    return sorted(
        path
        for path in landed_root.rglob("*")
        if path.is_file() and path.suffix in (".h", ".cc")
    )


def compile_translation_unit(
    compiler: str,
    translation_unit: Path,
    object_path: Path,
    include_arguments: list[str],
    compiler_environment: dict[str, str],
) -> tuple[bool, str]:
    compiler_process = subprocess.run(
        [
            compiler,
            f"-std={LANGUAGE_STANDARD}",
            *include_arguments,
            "-c",
            str(translation_unit),
            "-o",
            str(object_path),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
        env=compiler_environment,
    )
    if compiler_process.returncode != 0:
        return False, compiler_process.stdout
    if not object_path.is_file() or object_path.stat().st_size == 0:
        return False, (
            compiler_process.stdout
            + "the compiler reported success but produced no object content\n"
        )
    return True, compiler_process.stdout


def read_demangled_symbols(object_paths: list[Path]) -> dict[Path, str]:
    """Demangled symbol text per object, defined and undefined alike."""
    symbol_reader = shutil.which("nm")
    if symbol_reader is None:
        raise RuntimeError(
            "nm was not found; the symbol check cannot run and will not be skipped"
        )
    listings: dict[Path, str] = {}
    for object_path in object_paths:
        reader_process = subprocess.run(
            [symbol_reader, "--demangle", str(object_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        if reader_process.returncode != 0:
            raise RuntimeError(
                f"nm failed on {object_path.name}: {reader_process.stdout.strip()}"
            )
        listings[object_path] = reader_process.stdout
    return listings


def source_level_runtime_dependency_surface(
    landed_root: Path,
) -> tuple[list[tuple[str, int, str]], dict[str, list[tuple[str, int]]]]:
    """Calculates missing Drake includes and named systems types from source."""
    missing_includes: list[tuple[str, int, str]] = []
    systems_types: dict[str, list[tuple[str, int]]] = {}
    for source_path in landed_sources(landed_root):
        relative_path = source_path.relative_to(landed_root).as_posix()
        source_text = source_path.read_text()
        include_lines = source_lines_without_comments(
            source_text, preserve_literal_contents=True
        )
        code_only_lines = source_lines_without_comments(
            source_text, preserve_literal_contents=False
        )
        for line_number, (include_line, code_only_line) in enumerate(
            zip(include_lines, code_only_lines), start=1
        ):
            include_match = DRAKE_INCLUDE_PATTERN.search(include_line)
            if include_match is not None:
                included = include_match.group(1)
                if not (landed_root / included).is_file():
                    missing_includes.append((relative_path, line_number, included))
            for type_match in SYSTEMS_TYPE_PATTERN.finditer(code_only_line):
                systems_types.setdefault(type_match.group(1), []).append(
                    (relative_path, line_number)
                )
    return missing_includes, systems_types


def report_runtime_dependency_surface(
    missing_includes: list[tuple[str, int, str]],
    systems_types: dict[str, list[tuple[str, int]]],
) -> None:
    """Prints what the landed source still needs from a runtime it does not have.

    Recomputed from source on every run and printed, never stored: this is the
    source-level surface, not a proven ABI or a minimum implementation contract,
    and freezing it into a file would turn today's reading into tomorrow's gate.
    """
    print("\nSOURCE-LEVEL RUNTIME DEPENDENCY SURFACE")
    print("  (recomputed from source; not an ABI or an implementation contract)")
    print("\n  headers included but not landed:")
    if not missing_includes:
        print("    none")
    for relative_path, line_number, included in missing_includes:
        print(f"    {included}  <- {relative_path}:{line_number}")
    print("\n  systems:: types named by the landed source:")
    if not systems_types:
        print("    none")
    for type_name in sorted(systems_types):
        sites = systems_types[type_name]
        first_path, first_line = sites[0]
        print(f"    systems::{type_name}  first at {first_path}:{first_line}")


def report_forbidden_symbols(
    listings: dict[Path, str], source_path_by_object: dict[Path, str]
) -> bool:
    """Reports symbols the double-only boundary must not contain."""
    violations: list[tuple[str, str, str]] = []
    for object_path, listing in listings.items():
        for line in listing.splitlines():
            for forbidden_name in FORBIDDEN_SYMBOL_NAMES:
                if forbidden_name in line:
                    violations.append(
                        (
                            source_path_by_object[object_path],
                            forbidden_name,
                            line.strip(),
                        )
                    )
    print("\nSYMBOL CHECK")
    if not violations:
        print("  no forbidden scalar symbol in the objects produced")
        return True
    for source_path, forbidden_name, line in violations:
        print(f"  FORBIDDEN SYMBOL {source_path}: matched '{forbidden_name}' in {line}")
    return False


def report_forbidden_source_tokens(landed_root: Path) -> bool:
    """Reports scalar machinery that resolves before it can reach a symbol table."""
    violations: list[tuple[str, int, str]] = []
    for source_path in landed_sources(landed_root):
        relative_path = source_path.relative_to(landed_root).as_posix()
        text = source_path.read_text()
        code = source_lines_without_comments(
            text, preserve_literal_contents=False
        )
        for line_number, (raw_line, code_line) in enumerate(
            zip(text.splitlines(), code), start=1
        ):
            for token in FORBIDDEN_SOURCE_TOKENS:
                # An include or a construct counts wherever it appears in code; a
                # doxygen tag counts even inside a comment, because that is the
                # only place it can appear.
                haystack = raw_line if token.startswith("@tparam_") else code_line
                if token in haystack:
                    violations.append((relative_path, line_number, token))
    print("\nSOURCE RESIDUE CHECK")
    if not violations:
        print("  no compile-time scalar machinery in the landed source")
        return True
    for relative_path, line_number, token in violations:
        print(f"  FORBIDDEN SOURCE TOKEN {relative_path}:{line_number}: {token}")
    return False


def compile_landed_translation_units(
    landed_root: Path,
    compiler: str,
    third_party_include_directories: list[str],
    show_diagnostics: bool,
) -> int:
    if not landed_root.is_dir():
        print(f"LANDED: not a directory: {landed_root}", file=sys.stderr)
        return 2
    if not (landed_root / "drake").is_dir():
        print(
            f"LANDED: {landed_root} does not contain a drake/ directory, so it is"
            " not a landed source root",
            file=sys.stderr,
        )
        return 2

    missing_includes, systems_types = source_level_runtime_dependency_surface(
        landed_root
    )
    headers_expected_absent = tuple(
        sorted(
            set(ABSENT_HEADER_CANARIES)
            | {included for _, _, included in missing_includes}
        )
    )
    compiler_environment = sanitized_compiler_environment()
    if drake_headers_visible_with(
        compiler, [], compiler_environment, headers_expected_absent
    ):
        print(
            "LANDED: the compiler's default include search reaches a Drake tree"
            " this boundary does not have",
            file=sys.stderr,
        )
        return 2
    for include_directory in third_party_include_directories:
        if drake_headers_visible_with(
            compiler,
            ["-I", include_directory],
            compiler_environment,
            headers_expected_absent,
        ):
            print(
                "LANDED: third-party include directory reaches a Drake tree this"
                f" boundary does not have: {include_directory}",
                file=sys.stderr,
            )
            return 2

    translation_units = landed_translation_units(landed_root)
    if not translation_units:
        print(f"LANDED: no translation unit under {landed_root}", file=sys.stderr)
        return 2

    include_arguments = ["-I", str(landed_root)]
    for include_directory in third_party_include_directories:
        include_arguments += ["-isystem", include_directory]

    outcomes: list[TranslationUnitOutcome] = []
    with tempfile.TemporaryDirectory(prefix="orvd_landed_frontier.") as work_directory:
        work_root = Path(work_directory)
        for index, translation_unit in enumerate(translation_units):
            relative_path = translation_unit.relative_to(landed_root).as_posix()
            object_path = work_root / f"{index:04d}.o"
            succeeded, diagnostics = compile_translation_unit(
                compiler,
                translation_unit,
                object_path,
                include_arguments,
                compiler_environment,
            )
            outcomes.append(
                TranslationUnitOutcome(
                    relative_path=relative_path,
                    succeeded=succeeded,
                    object_path=object_path if succeeded else None,
                    diagnostics=diagnostics,
                )
            )

        print("COMPILE FRONTIER")
        for outcome in outcomes:
            marker = "compiled" if outcome.succeeded else "BLOCKED "
            print(f"  {marker}  {outcome.relative_path}")
            if not outcome.succeeded or show_diagnostics:
                for line in outcome.diagnostics.splitlines():
                    print(f"      | {line}")

        source_path_by_object = {
            outcome.object_path: outcome.relative_path
            for outcome in outcomes
            if outcome.object_path is not None
        }
        symbols_clean = report_forbidden_symbols(
            read_demangled_symbols(list(source_path_by_object)),
            source_path_by_object,
        )

    source_clean = report_forbidden_source_tokens(landed_root)
    report_runtime_dependency_surface(missing_includes, systems_types)

    blocked = [outcome for outcome in outcomes if not outcome.succeeded]
    print("\nRESULT")
    if blocked:
        print(
            f"  {len(blocked)} translation unit(s) cannot compile against the landed"
            " boundary alone. Read the diagnostics above; this tool deliberately"
            " does not classify compiler failures."
        )
    else:
        print("  every landed translation unit produced an object")
    if not symbols_clean:
        print("  the symbol check found forbidden scalar symbols")
    if not source_clean:
        print("  the source scan found compile-time scalar machinery")
    if blocked or not symbols_clean or not source_clean:
        return 1
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Compile the landed double-only Drake source and report what"
        " still depends on a runtime."
    )
    parser.add_argument(
        "--landed-root",
        required=True,
        help="directory containing the landed drake/ tree",
    )
    parser.add_argument(
        "--compiler", default="c++", help="GNU-compatible compiler executable"
    )
    parser.add_argument(
        "--third-party-include-directory",
        action="append",
        default=[],
        help="an include directory for an admitted dependency (Eigen, fmt);"
        " repeatable. It must not expose a Drake tree.",
    )
    parser.add_argument(
        "--show-diagnostics",
        action="store_true",
        help="print the compiler output for successful units as well",
    )
    arguments = parser.parse_args(argv)
    try:
        return compile_landed_translation_units(
            Path(arguments.landed_root),
            arguments.compiler,
            arguments.third_party_include_directory,
            arguments.show_diagnostics,
        )
    except RuntimeError as error:
        print(f"LANDED: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
