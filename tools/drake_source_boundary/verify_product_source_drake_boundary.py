#!/usr/bin/env python3
"""Checks that no product source lands or reaches a forbidden part of Drake.

The disposition ledger distinguishes two kinds of "not here". `discard` means we
do not need this today and a later goal may admit it after a fresh decision.
`forbidden` means the file violates the runtime boundary — geometry, FEM, plant,
contact solvers, meshcat, solvers — and reaching it is a defect, not a decision
to revisit. This tool only enforces the second kind.

That restraint is the point. An unclassified header, a `discard`, or a runtime
header outside the landed tree are not `forbidden` boundary crossings. The
compile frontier and the admission closure report those distinct conditions if
they appear; repeating them here would mean three tools disagreeing about which
one is authoritative. G26-G28 have removed the runtime-header gap from the
current product, but that does not change this tool's deliberately narrower
question.

A `forbidden_prefix` outranks a per-file disposition. If someone writes
`vendor geometry/query_object.h`, the answer is not "vendored, therefore fine" —
it is that the ledger now contradicts itself on an architectural boundary, and
the boundary wins.

Product source only: `external/drake_mbtree/` covers both the landed Drake subset
and ORVD's own implementations beside it, and `libs/` covers the first-party
runtime as it appears from G20 onwards. `tests/` is deliberately excluded — the
Drake-backed reference tests are supposed to reach Drake, that is what makes them
a reference.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.dont_write_bytecode = True

from calculate_required_drake_source_closure import (
    HEADER_SUFFIXES,
    IMPLEMENTATION_SUFFIXES,
    LedgerError,
    parse_ledger,
    read_drake_includes,
)

# The upstream closure only needs the suffixes Drake uses. Product source is a
# wider boundary: first-party C++ may use any of these conventional header,
# template-implementation, or translation-unit suffixes, and every one must be
# inspected rather than silently falling outside the gate.
PRODUCT_SOURCE_SUFFIXES = HEADER_SUFFIXES + IMPLEMENTATION_SUFFIXES + (
    ".hh",
    ".hxx",
    ".inl",
    ".inc",
    ".ipp",
    ".tcc",
    ".cxx",
)


class ProductSourceBoundaryInputError(Exception):
    """The requested product source roots cannot be checked completely."""


def product_source_files(product_roots: list[Path]) -> list[tuple[Path, Path]]:
    """Return `(root, file)` for every source file under the product roots."""
    found: list[tuple[Path, Path]] = []
    for root in product_roots:
        if root.is_symlink():
            raise ProductSourceBoundaryInputError(
                f"product source root is a symbolic link: {root}"
            )
        for path in sorted(root.rglob("*")):
            if path.is_symlink() and (
                path.is_dir() or path.suffix in PRODUCT_SOURCE_SUFFIXES
            ):
                raise ProductSourceBoundaryInputError(
                    f"product source traversal reaches a symbolic link: {path}"
                )
            if path.is_file() and path.suffix in PRODUCT_SOURCE_SUFFIXES:
                found.append((root, path))
    return found


def forbidden_reason(ledger, drake_relative_path: str) -> str | None:
    """Why this Drake path is forbidden, or None if it is not.

    The prefix rule is consulted first: an architectural boundary is not
    negotiable by a per-file line elsewhere in the same ledger.
    """
    prefix = ledger.forbidden_prefix_for(drake_relative_path)
    if prefix is not None:
        return f"it is under the forbidden prefix '{prefix}'"
    if ledger.dispositions.get(drake_relative_path) == "forbidden":
        return "the ledger marks it forbidden"
    return None


def verify_product_source_boundary(
    repository_root: Path, ledger_path: Path, product_relative_roots: list[str]
) -> int:
    try:
        ledger = parse_ledger(ledger_path.read_text(encoding="utf-8"))
    except (OSError, LedgerError) as error:
        print(f"BOUNDARY: ledger unusable: {error}", file=sys.stderr)
        return 2

    product_roots = [repository_root / relative for relative in product_relative_roots]
    missing_roots = [
        relative
        for relative, root in zip(product_relative_roots, product_roots)
        if not root.is_dir()
    ]
    if missing_roots:
        print(
            "BOUNDARY: product source root does not exist: "
            + ", ".join(missing_roots),
            file=sys.stderr,
        )
        return 2
    try:
        source_files = product_source_files(product_roots)
    except ProductSourceBoundaryInputError as error:
        print(f"BOUNDARY: {error}", file=sys.stderr)
        return 2
    if not source_files:
        print(
            "BOUNDARY: product source roots contain no supported source files",
            file=sys.stderr,
        )
        return 2

    violations: list[str] = []
    input_errors: list[str] = []

    print("PRODUCT SOURCE BOUNDARY")
    for relative in product_relative_roots:
        print(f"  scanning {relative}")

    landed_drake_root = repository_root / "external" / "drake_mbtree" / "drake"
    if landed_drake_root.is_dir():
        for path in sorted(landed_drake_root.rglob("*")):
            if not path.is_file():
                continue
            drake_relative_path = path.relative_to(landed_drake_root).as_posix()
            reason = forbidden_reason(ledger, drake_relative_path)
            if reason is not None:
                violations.append(
                    f"{path.relative_to(repository_root).as_posix()} is landed, but"
                    f" {reason}"
                )

    for _root, path in source_files:
        display_path = path.relative_to(repository_root).as_posix()
        try:
            includes = read_drake_includes(path)
        except LedgerError as error:
            input_errors.append(f"{display_path}: {error}")
            continue
        for included in includes:
            reason = forbidden_reason(ledger, included)
            if reason is not None:
                violations.append(
                    f"{display_path} includes drake/{included}, but {reason}"
                )

    print("\nRESULT")
    if input_errors:
        for input_error in input_errors:
            print(f"  BOUNDARY INPUT ERROR: {input_error}")
        return 2
    if violations:
        for violation in violations:
            print(f"  FORBIDDEN BOUNDARY CROSSING: {violation}")
        return 1
    print(
        "  no product source is a forbidden Drake file, and none includes one;"
        " this checks the architectural boundary only, not admission or the"
        " compile frontier"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Check that product source neither is nor reaches forbidden Drake."
    )
    parser.add_argument(
        "--repository-root", required=True, help="the ORVD repository root"
    )
    parser.add_argument(
        "--disposition-ledger", required=True, help="path to SOURCE_DISPOSITION.txt"
    )
    parser.add_argument(
        "--product-source-root",
        action="append",
        default=None,
        help="a product source root relative to the repository root; repeatable."
        " Defaults to external/drake_mbtree and libs.",
    )
    arguments = parser.parse_args(argv)
    product_relative_roots = arguments.product_source_root or [
        "external/drake_mbtree",
        "libs",
    ]
    return verify_product_source_boundary(
        Path(arguments.repository_root),
        Path(arguments.disposition_ledger),
        product_relative_roots,
    )


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
