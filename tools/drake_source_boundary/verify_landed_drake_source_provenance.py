#!/usr/bin/env python3
"""Checks that every landed Drake file can be traced back to the pinned upstream.

Redistributing someone else's source carries obligations, and the one this checks
is traceability: a reader must be able to take any file under the landed tree and
say which upstream repository, which revision and which path it came from. The
disposition ledger holds those three facts, so this tool asks whether the ledger,
the landed tree and the upstream clone still agree.

Identity is by path, never by hash. A hash says two files are byte-identical; it
does not say where either came from, it changes the moment we exercise the
modification right the licence grants, and a hash list committed today becomes a
gate that fails for the right reason on the wrong day. Paths survive
modification, which is exactly the property provenance needs.

Divergence from upstream is expected and is not an error here: the landed tree
has been through a documented double-only surgery. What is checked is that the
files which diverge and carry a licence requiring a modification notice actually
carry one. Apache-2.0 section 4(b) obliges a modified file to say it was
modified; BSD-3-Clause has no such clause, so a modified BSD file needs the
licence text shipped and nothing in the file itself.

The upstream clone is named on the command line and nowhere else. It is large,
it is a working copy of someone else's repository, and where a given machine
keeps it is not a fact about this project.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True

from calculate_required_drake_source_closure import (
    LedgerError,
    parse_ledger,
)

# The notice Apache-2.0 section 4(b) requires of us, matched loosely enough that
# rewording it is allowed and strictly enough that its absence is visible.
MODIFICATION_NOTICE_PATTERN = re.compile(
    r"modified by the OpenRailVehicleDynamics project", re.IGNORECASE
)
LICENSE_REQUIRING_MODIFICATION_NOTICE = "Apache-2.0"


def read_pinned_revision(clone_root: Path) -> str | None:
    """Returns the clone's HEAD commit, or None if it cannot be determined."""
    head_process = subprocess.run(
        ["git", "-C", str(clone_root), "rev-parse", "HEAD"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if head_process.returncode != 0:
        return None
    return head_process.stdout.strip()


def audit_provenance(
    landed_root: Path, upstream_clone: Path, ledger_path: Path
) -> int:
    try:
        ledger = parse_ledger(ledger_path.read_text())
    except (OSError, LedgerError) as error:
        print(f"PROVENANCE: ledger unusable: {error}", file=sys.stderr)
        return 2
    if not landed_root.is_dir():
        print(f"PROVENANCE: not a directory: {landed_root}", file=sys.stderr)
        return 2
    if not upstream_clone.is_dir():
        print(f"PROVENANCE: not a directory: {upstream_clone}", file=sys.stderr)
        return 2

    violations: list[str] = []

    print("PROVENANCE")
    print(f"  upstream repository  {ledger.source_repository}")
    print(f"  pinned revision      {ledger.source_commit}  ({ledger.source_tag})")
    print(f"  upstream clone       {upstream_clone}")
    print(f"  landed tree          {landed_root}")

    # The clone must be the revision the ledger names. Comparing a landed file
    # against some other revision would answer a question nobody asked.
    clone_revision = read_pinned_revision(upstream_clone)
    if clone_revision is None:
        violations.append(
            f"the clone at {upstream_clone} has no readable git revision, so it"
            " cannot be confirmed to be the pinned one"
        )
    elif clone_revision != ledger.source_commit:
        violations.append(
            f"the clone is at {clone_revision}, but the ledger pins"
            f" {ledger.source_commit}"
        )

    vendored_paths = {
        path for path, disposition in ledger.dispositions.items() if disposition == "vendor"
    }
    landed_paths = {
        path.relative_to(landed_root / "drake").as_posix()
        for path in (landed_root / "drake").rglob("*")
        if path.is_file()
    }

    # Three-way agreement: what the ledger admits, what is actually here, and what
    # upstream has at that path.
    for path in sorted(vendored_paths - landed_paths):
        violations.append(f"the ledger admits {path} but it is not in the landed tree")
    for path in sorted(landed_paths - vendored_paths):
        violations.append(f"{path} is in the landed tree but the ledger does not admit it")
    for path in sorted(vendored_paths & landed_paths):
        if not (upstream_clone / path).is_file():
            violations.append(
                f"{path} is landed but does not exist upstream at that path, so its"
                " provenance cannot be stated"
            )

    # The licence texts we are obliged to ship.
    for relative_license_path in (ledger.license_file, "LICENSE.Apache-2.0.txt"):
        if not (landed_root / relative_license_path).is_file():
            violations.append(
                f"the licence text {relative_license_path} is not present beside the"
                " source it covers"
            )

    # Files whose own licence requires us to say we changed them.
    print("\n  files carrying a per-file licence:")
    if not ledger.file_licenses:
        print("    none")
    for path in sorted(ledger.file_licenses):
        license_spdx = ledger.file_licenses[path]
        landed_file = landed_root / "drake" / path
        upstream_file = upstream_clone / path
        if not landed_file.is_file():
            print(f"    {path}  {license_spdx}  (not landed)")
            continue
        modified = (
            not upstream_file.is_file()
            or landed_file.read_bytes() != upstream_file.read_bytes()
        )
        if not modified:
            print(f"    {path}  {license_spdx}  unmodified")
            continue
        has_notice = MODIFICATION_NOTICE_PATTERN.search(landed_file.read_text()) is not None
        print(
            f"    {path}  {license_spdx}  modified,"
            f" {'notice present' if has_notice else 'NOTICE MISSING'}"
        )
        if license_spdx == LICENSE_REQUIRING_MODIFICATION_NOTICE and not has_notice:
            violations.append(
                f"{path} is {license_spdx} and has been modified, but carries no"
                " notice saying so, which section 4(b) requires"
            )

    print("\nRESULT")
    if violations:
        for violation in violations:
            print(f"  PROVENANCE VIOLATION: {violation}")
        return 1
    print(
        "  every landed file is admitted by the ledger and exists upstream at the"
        " same path under the pinned revision; the licence texts are present and"
        " every modified file whose licence requires a modification notice has one"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Audit the provenance of the landed Drake source."
    )
    parser.add_argument(
        "--landed-root",
        required=True,
        help="directory containing the landed drake/ tree and its licence texts",
    )
    parser.add_argument(
        "--upstream-clone",
        required=True,
        help="a clone of the upstream repository at the pinned revision",
    )
    parser.add_argument(
        "--disposition-ledger", required=True, help="path to SOURCE_DISPOSITION.txt"
    )
    arguments = parser.parse_args(argv)
    return audit_provenance(
        Path(arguments.landed_root),
        Path(arguments.upstream_clone),
        Path(arguments.disposition_ledger),
    )


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
