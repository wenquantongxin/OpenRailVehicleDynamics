#!/usr/bin/env python3
"""Computes, from source, which Drake files the ORVD vendor boundary actually needs.

The closure starts at admitted files in the ledger's candidate directories,
follows literal Drake include edges, and adds each header's implementation
translation unit. Anything the closure reaches that the ledger does not classify
is an error, and so is anything it reaches that the ledger forbids.

Boundary failures are reported separately because they need different fixes. An
unclassified edge has no decision, a forbidden edge violates a decision, an unmet
dependency contradicts one, and an unreached admission is unnecessary baggage.

What this deliberately does not do: it never invokes a compiler, a linker or nm,
never resolves symbols, and never claims the admitted set links. Reaching a
header is not the same as needing every symbol in it; proving link completeness
is a separate question answered by compiling, not by reading includes.

Include resolution is literal on purpose. Drake writes workspace-relative
includes (``#include "drake/multibody/tree/frame.h"``), so a path is a path. No
preprocessor is run: conditional compilation is not evaluated, which makes
recognized literal includes an over-approximation. A macro or otherwise
unrecognized include operand is rejected instead of being silently omitted.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

# Loose on form: leading whitespace, spacing after `#`, quote style, and trailing
# content are all tolerated.
INCLUDE_DIRECTIVE_PATTERN = re.compile(r"^\s*#\s*include\b")
INCLUDE_PATH_PATTERN = re.compile(
    r'^\s*#\s*include\s*(?:"([^"]+)"|<([^>]+)>)'
)

HEADER_SUFFIXES = (".h", ".hpp")
IMPLEMENTATION_SUFFIXES = (".cc", ".cpp")

ADMITTED_DISPOSITIONS = frozenset({"vendor"})
FORBIDDEN_DISPOSITIONS = frozenset({"forbidden"})
ALL_DISPOSITIONS = frozenset(
    {"vendor", "first_party", "reference_only", "discard", "forbidden"}
)


class LedgerError(Exception):
    """The ledger cannot be used as written."""


@dataclass
class Ledger:
    """Everything the disposition file says, in the form the closure needs."""

    source_commit: str = ""
    source_tag: str = ""
    license_spdx: str = ""
    license_file: str = ""
    candidate_directories: list[str] = field(default_factory=list)
    # path relative to the Drake source root -> disposition
    dispositions: dict[str, str] = field(default_factory=dict)
    dispositions_reasons: dict[str, str] = field(default_factory=dict)
    forbidden_prefixes: dict[str, str] = field(default_factory=dict)
    # header path -> implementation translation units declared for it, for the
    # cases where a header's implementation does not share its stem
    declared_implementations: dict[str, list[str]] = field(default_factory=dict)
    # headers that no source tree contains because the build system generates them
    generated_headers: dict[str, str] = field(default_factory=dict)
    file_licenses: dict[str, str] = field(default_factory=dict)

    def admitted_roots(self, source_root: Path) -> list[str]:
        roots: list[str] = []
        for directory in self.candidate_directories:
            candidate_directory = source_root / directory
            if not candidate_directory.is_dir():
                raise LedgerError(f"candidate directory does not exist: {directory}")
            for candidate in candidate_directory.iterdir():
                if not candidate.is_file():
                    continue
                if candidate.suffix not in HEADER_SUFFIXES + IMPLEMENTATION_SUFFIXES:
                    continue
                path = candidate.relative_to(source_root).as_posix()
                disposition = self.dispositions.get(path)
                if disposition is None:
                    raise LedgerError(f"candidate file is unclassified: {path}")
                if disposition in ADMITTED_DISPOSITIONS:
                    roots.append(path)
        if not roots:
            raise LedgerError("candidate directories contain no admitted vendor file")
        return sorted(roots)

    def forbidden_prefix_for(self, path: str) -> str | None:
        for prefix in self.forbidden_prefixes:
            if path.startswith(prefix):
                return prefix
        return None


def parse_ledger(text: str) -> Ledger:
    """Reads the disposition file.

    Loose on form: blank lines, full-line comments, trailing comments and unknown
    metadata keys are all accepted, and fields may appear in any order. Strict on
    substance: a directive whose payload is missing or whose disposition is not a
    known one is an error, because the closure cannot guess what was meant.
    """
    ledger = Ledger()
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        directive = fields[0]
        payload = fields[1:]

        def require(count: int, shape: str) -> list[str]:
            if len(payload) < count:
                raise LedgerError(
                    f"line {line_number}: {directive} needs {shape}, got: {raw_line.strip()}"
                )
            return payload

        if directive == "source_commit":
            ledger.source_commit = require(1, "a commit")[0]
        elif directive == "source_tag":
            ledger.source_tag = require(1, "a tag")[0]
        elif directive == "license_spdx":
            ledger.license_spdx = require(1, "an SPDX identifier")[0]
        elif directive == "license_file":
            ledger.license_file = require(1, "a path")[0]
        elif directive == "candidate_directory":
            ledger.candidate_directories.append(require(1, "a path")[0])
        elif directive == "file_license":
            path, license_spdx = require(2, "a path and an SPDX identifier")[:2]
            ledger.file_licenses[path] = license_spdx
        elif directive in ALL_DISPOSITIONS:
            path = require(1, "a path")[0]
            previous = ledger.dispositions.get(path)
            if previous is not None and previous != directive:
                raise LedgerError(
                    f"line {line_number}: {path} is already classified as {previous}"
                )
            ledger.dispositions[path] = directive
            # The reason is everything after the path on the original line, which
            # is where the trailing comment lives.
            _, _, trailing = raw_line.partition("#")
            ledger.dispositions_reasons[path] = trailing.strip()
        elif directive == "forbidden_prefix":
            prefix = require(1, "a path prefix")[0]
            _, _, trailing = raw_line.partition("#")
            ledger.forbidden_prefixes[prefix] = trailing.strip()
        elif directive == "implementation":
            header, implementation = require(2, "a header and a translation unit")[:2]
            ledger.declared_implementations.setdefault(header, []).append(implementation)
        elif directive == "generated_header":
            path = require(1, "a path")[0]
            _, _, trailing = raw_line.partition("#")
            ledger.generated_headers[path] = trailing.strip()
        else:
            # Unknown directives are ignored rather than rejected: this file is
            # trusted internal input, and refusing to run because it grew a field
            # this tool does not read would be brittle for no gain.
            continue

    if not ledger.source_commit:
        raise LedgerError("the ledger declares no source_commit")
    if re.fullmatch(r"[0-9a-f]{40}", ledger.source_commit) is None:
        raise LedgerError("source_commit must be a complete 40-digit lowercase hex commit")
    if not ledger.source_tag:
        raise LedgerError("the ledger declares no source_tag")
    if not ledger.license_spdx:
        raise LedgerError("the ledger declares no license_spdx")
    if not ledger.license_file:
        raise LedgerError("the ledger declares no license_file")
    if not ledger.candidate_directories:
        raise LedgerError("the ledger declares no candidate_directory")
    for path, disposition in ledger.dispositions.items():
        if not ledger.dispositions_reasons.get(path):
            raise LedgerError(f"{disposition} {path} carries no reason")
    for prefix, reason in ledger.forbidden_prefixes.items():
        if not reason:
            raise LedgerError(f"forbidden_prefix {prefix} carries no reason")
    for path in ledger.file_licenses:
        if ledger.dispositions.get(path) != "vendor":
            raise LedgerError(f"file_license refers to a non-vendored file: {path}")
    return ledger


def read_drake_includes(file_path: Path) -> list[str]:
    """Every `drake/...` include in a file, in order, with the `drake/` prefix removed."""
    includes = []
    for line_number, line in enumerate(
        file_path.read_text(encoding="utf-8", errors="replace").splitlines(), start=1
    ):
        if not INCLUDE_DIRECTIVE_PATTERN.match(line):
            continue
        match = INCLUDE_PATH_PATTERN.match(line)
        if match is None:
            raise LedgerError(
                f"{file_path}:{line_number}: unsupported include operand: {line.strip()}"
            )
        included_path = match.group(1) or match.group(2)
        if included_path.startswith("drake/"):
            includes.append(included_path[len("drake/") :])
    return includes


def implementation_units_for(header: str, ledger: Ledger, source_root: Path) -> list[str]:
    """The translation units that implement a header.

    Two relations, and only two: the unit sharing the header's stem, and any unit
    the ledger declares explicitly for headers whose implementation is split or
    renamed. Whether those units are needed to *link* is not decided here.
    """
    units = []
    stem = header
    for suffix in HEADER_SUFFIXES:
        if header.endswith(suffix):
            stem = header[: -len(suffix)]
            break
    for suffix in IMPLEMENTATION_SUFFIXES:
        candidate = stem + suffix
        if (source_root / candidate).is_file():
            units.append(candidate)
    units.extend(ledger.declared_implementations.get(header, []))
    return units


@dataclass
class Violation:
    # Three distinct situations, kept distinct because the fix differs:
    #   forbidden         the boundary is drawn and something crossed it.
    #   unclassified      the boundary was never drawn here; someone must decide.
    #   unmet_dependency  the boundary is drawn — this file is deliberately not
    #                     vendored — yet admitted code still needs it. Either the
    #                     decision or the admitted set is wrong.
    kind: str
    path: str
    reached_by: list[str]
    detail: str = ""


@dataclass
class ClosureResult:
    admitted_roots: list[str]
    reached: dict[str, list[str]]  # path -> the chain that first reached it
    violations: list[Violation]
    missing_from_source: list[str]


def compute_closure(ledger: Ledger, source_root: Path) -> ClosureResult:
    """Walks include and implementation edges from every admitted file."""
    reached: dict[str, list[str]] = {}
    violations: list[Violation] = []
    missing_from_source: list[str] = []

    roots = ledger.admitted_roots(source_root)
    # Each entry carries how it was reached. An include edge means admitted code
    # literally names the file; an implementation edge only means "a header with
    # this stem exists", which is a much weaker claim and must not be reported
    # the same way.
    pending: list[tuple[str, list[str], bool]] = [(root, [root], True) for root in roots]
    reached_by_include: dict[str, bool] = {}

    while pending:
        path, chain, arrived_by_include = pending.pop()
        was_reached = path in reached
        was_reached_by_include = reached_by_include.get(path, False)
        if was_reached and (was_reached_by_include or not arrived_by_include):
            continue
        reached[path] = chain
        reached_by_include[path] = was_reached_by_include or arrived_by_include

        forbidden_prefix = ledger.forbidden_prefix_for(path)
        disposition = ledger.dispositions.get(path)
        absolute = source_root / path

        if forbidden_prefix is not None:
            if not was_reached:
                violations.append(
                    Violation(
                        "forbidden",
                        path,
                        chain,
                        f"under forbidden prefix {forbidden_prefix}",
                    )
                )
            continue
        if disposition in FORBIDDEN_DISPOSITIONS:
            if not was_reached:
                violations.append(
                    Violation("forbidden", path, chain, f"classified {disposition}")
                )
            continue
        if disposition is None and path in ledger.generated_headers:
            if absolute.is_file():
                if not was_reached:
                    violations.append(
                        Violation(
                            "unclassified",
                            path,
                            chain,
                            "declared generated_header but present in the source tree",
                        )
                    )
            # A genuine generated header has no source file to walk into.
            continue
        if disposition is None:
            if not was_reached:
                violations.append(Violation("unclassified", path, chain, ""))
            continue
        if disposition not in ADMITTED_DISPOSITIONS:
            # Reached a file the ledger deliberately does not vendor. Reporting
            # this as "unclassified" would be wrong: the decision was made, and
            # what is broken is that admitted code contradicts it.
            #
            # Only an include edge is that contradiction. Arriving purely through
            # the implementation relation says the ledger declined a translation
            # unit it was offered, which is a decision already on record — many
            # of Drake's `.cc` files exist only to prove their header is
            # self-contained and define no symbol at all. Reporting those would
            # bury the real unmet dependencies in noise.
            if arrived_by_include and not was_reached_by_include:
                violations.append(
                    Violation(
                        "unmet_dependency", path, chain, f"classified {disposition}"
                    )
                )
            continue

        if was_reached:
            continue
        if not absolute.is_file():
            missing_from_source.append(path)
            continue

        for include in read_drake_includes(absolute):
            if include not in reached or not reached_by_include.get(include, False):
                pending.append((include, chain + [include], True))
        if path.endswith(HEADER_SUFFIXES):
            for unit in implementation_units_for(path, ledger, source_root):
                if unit not in reached:
                    pending.append((unit, chain + [unit], False))

    return ClosureResult(roots, reached, violations, missing_from_source)


def verify_source_commit(source_root: Path, expected_commit: str) -> str | None:
    """Confirms the clone is the revision the ledger was written against.

    Read straight from the git directory rather than by running git, so the check
    works without a git binary and cannot be confused by the caller's environment.
    """
    head_file = source_root / ".git" / "HEAD"
    if not head_file.is_file():
        return f"{source_root} has no .git/HEAD, so its revision cannot be confirmed"
    head = head_file.read_text(encoding="utf-8").strip()
    if head.startswith("ref:"):
        reference = head.split(":", 1)[1].strip()
        reference_file = source_root / ".git" / reference
        if reference_file.is_file():
            head = reference_file.read_text(encoding="utf-8").strip()
        else:
            packed = source_root / ".git" / "packed-refs"
            head = ""
            if packed.is_file():
                for line in packed.read_text(encoding="utf-8").splitlines():
                    if line.endswith(" " + reference):
                        head = line.split(" ", 1)[0]
                        break
    if not head:
        return f"{source_root} HEAD could not be resolved"
    if head != expected_commit:
        return (
            f"{source_root} is at {head}, but the ledger was written against "
            f"{expected_commit}"
        )
    return None


def format_chain(chain: list[str]) -> str:
    return " -> ".join(chain)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Compute the Drake source closure the ORVD vendor boundary needs."
    )
    parser.add_argument(
        "--drake-source-root",
        required=True,
        type=Path,
        help="root of a Drake source clone (needs .cc files; an installed include "
        "tree is not enough)",
    )
    parser.add_argument(
        "--disposition-ledger",
        required=True,
        type=Path,
        help="path to SOURCE_DISPOSITION.txt",
    )
    parser.add_argument(
        "--require-source-commit",
        action="store_true",
        help="fail unless the clone's HEAD matches the ledger's source_commit",
    )
    parser.add_argument(
        "--list-closure",
        action="store_true",
        help="print every reached file instead of only the violations",
    )
    arguments = parser.parse_args(argv)

    try:
        ledger = parse_ledger(
            arguments.disposition_ledger.read_text(encoding="utf-8")
        )
    except (OSError, LedgerError) as error:
        print(f"ledger unusable: {error}", file=sys.stderr)
        return 2

    source_root = arguments.drake_source_root
    if not source_root.is_dir():
        print(f"not a directory: {source_root}", file=sys.stderr)
        return 2

    if arguments.require_source_commit:
        mismatch = verify_source_commit(source_root, ledger.source_commit)
        if mismatch is not None:
            print(f"source revision mismatch: {mismatch}", file=sys.stderr)
            return 2

    license_path = source_root / ledger.license_file
    if not license_path.is_file():
        print(
            f"ledger license_file is absent from the source tree: {ledger.license_file}",
            file=sys.stderr,
        )
        return 2

    try:
        result = compute_closure(ledger, source_root)
    except LedgerError as error:
        print(f"source boundary unusable: {error}", file=sys.stderr)
        return 2

    print(f"admitted roots:   {len(result.admitted_roots)}")
    print(f"files reached:    {len(result.reached)}")
    if arguments.list_closure:
        for path in sorted(result.reached):
            print(f"  reached {path}")

    for path in sorted(result.missing_from_source):
        print(
            f"MISSING: {path} is admitted but absent from the source tree "
            f"(reached by {format_chain(result.reached[path])})",
            file=sys.stderr,
        )

    admitted_files = {
        path
        for path, disposition in ledger.dispositions.items()
        if disposition in ADMITTED_DISPOSITIONS
    }
    unreached_admissions = sorted(admitted_files - set(result.reached))
    for path in unreached_admissions:
        print(
            f"UNREACHED ADMISSION: {path} is classified vendor but no candidate "
            "requires it",
            file=sys.stderr,
        )

    forbidden = [v for v in result.violations if v.kind == "forbidden"]
    unclassified = [v for v in result.violations if v.kind == "unclassified"]
    unmet = [v for v in result.violations if v.kind == "unmet_dependency"]

    for label, group in (
        ("FORBIDDEN EDGE", forbidden),
        ("UNCLASSIFIED EDGE", unclassified),
        ("UNMET DEPENDENCY", unmet),
    ):
        for violation in sorted(group, key=lambda v: v.path):
            detail = f" ({violation.detail})" if violation.detail else ""
            print(
                f"{label}: {violation.path}{detail}\n"
                f"  reached by {format_chain(violation.reached_by)}",
                file=sys.stderr,
            )

    print(f"forbidden edges:   {len(forbidden)}")
    print(f"unclassified:      {len(unclassified)}")
    print(f"unmet dependency:  {len(unmet)}")
    print(f"missing sources:   {len(result.missing_from_source)}")
    print(f"unreached vendor:  {len(unreached_admissions)}")

    if result.violations or result.missing_from_source or unreached_admissions:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
