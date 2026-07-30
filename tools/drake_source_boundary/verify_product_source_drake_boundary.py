#!/usr/bin/env python3
"""Checks that no product source lands or reaches a forbidden part of Drake.

The disposition ledger distinguishes two kinds of "not here". `discard` means we
do not need this today and a later goal may admit it after a fresh decision.
`forbidden` means the file violates the runtime boundary — geometry, FEM, plant,
contact solvers, meshcat, solvers — and reaching it is a defect, not a decision
to revisit. This tool only enforces the second kind.

That restraint is the point. An unclassified header, a `discard`, or one of the
runtime headers G20-G28 has not replaced yet would all fail a broader check, and
all three are things the project currently expects to see. A gate that fires on
expected conditions is a gate somebody turns off. The compile frontier and the
admission closure already report those; repeating them here would mean three
tools disagreeing about which one is authoritative.

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

SOURCE_SUFFIXES = HEADER_SUFFIXES + IMPLEMENTATION_SUFFIXES


def product_source_files(product_roots: list[Path]) -> list[tuple[Path, Path]]:
    """(root, file) for every source file under the product roots that exist."""
    found: list[tuple[Path, Path]] = []
    for root in product_roots:
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*")):
            if path.is_file() and path.suffix in SOURCE_SUFFIXES:
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
    if not any(root.is_dir() for root in product_roots):
        print(
            "BOUNDARY: none of the product source roots exist: "
            + ", ".join(product_relative_roots),
            file=sys.stderr,
        )
        return 2

    violations: list[str] = []

    print("PRODUCT SOURCE BOUNDARY")
    for relative in product_relative_roots:
        present = "" if (repository_root / relative).is_dir() else "  (not present yet)"
        print(f"  scanning {relative}{present}")

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

    for root, path in product_source_files(product_roots):
        display_path = path.relative_to(repository_root).as_posix()
        try:
            includes = read_drake_includes(path)
        except LedgerError as error:
            violations.append(f"{display_path}: {error}")
            continue
        for included in includes:
            reason = forbidden_reason(ledger, included)
            if reason is not None:
                violations.append(
                    f"{display_path} includes drake/{included}, but {reason}"
                )

    print("\nRESULT")
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
