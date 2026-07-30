#!/usr/bin/env python3
"""Checks that the provenance audit reports what it must and hides nothing.

Each case builds a synthetic landed tree, ledger and upstream clone whose right
answer is known by construction, so the audit is exercised on something other
than whatever the real tree happens to contain today.

The controls each target a distinct way provenance could be claimed without being
earned: a clone at the wrong revision, an admitted file that never landed, a
landed file nobody admitted, a landed file with no upstream path to point at, a
missing licence text, and a modified Apache-2.0 file with no notice saying it was
modified. The last case has a matching negative control — an unmodified
Apache-2.0 file must not be asked for a notice it does not owe.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True

PROVENANCE_AUDIT = (
    Path(__file__).resolve().parent / "verify_landed_drake_source_provenance.py"
)

PINNED_COMMIT = "0123456789abcdef0123456789abcdef01234567"

BASE_LEDGER = f"""\
source_repository https://example.invalid/synthetic/upstream.git
source_commit {PINNED_COMMIT}
source_tag v0.0.0
license_spdx BSD-3-Clause
license_file LICENSE.TXT
candidate_directory candidate

vendor candidate/root.h  # the candidate under audit
vendor support/shared.h  # support the candidate includes
vendor support/licensed.cc  # support carrying its own per-file licence
file_license support/licensed.cc Apache-2.0
"""

UPSTREAM_FILES = {
    "candidate/root.h": "#pragma once\n// root\n",
    "support/shared.h": "#pragma once\n// shared\n",
    "support/licensed.cc": (
        "/* Portions copyright (c) 2014 Somebody Else.\n"
        "Licensed under the Apache License, Version 2.0. */\n"
        "int Licensed() { return 1; }\n"
    ),
}

failure_count = 0


def record_failure_unless(condition: bool, failure_description: str) -> None:
    global failure_count
    if not condition:
        print(f"FAILED: {failure_description}", file=sys.stderr)
        failure_count += 1


def write_files(root: Path, files: dict[str, str]) -> None:
    for relative_path, content in files.items():
        path = root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")


def make_upstream_clone(root: Path, files: dict[str, str], revision: str) -> Path:
    write_files(root, files)
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    subprocess.run(
        ["git", "-C", str(root), "config", "user.email", "synthetic@example.invalid"],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(root), "config", "user.name", "Synthetic Upstream"], check=True
    )
    subprocess.run(["git", "-C", str(root), "add", "-A"], check=True)
    subprocess.run(
        ["git", "-C", str(root), "commit", "-q", "-m", "synthetic upstream"], check=True
    )
    # The audit reads HEAD, so point it at whatever revision the case wants to
    # pretend the clone is at.
    (root / ".git" / "HEAD").write_text(revision + "\n", encoding="utf-8")
    return root


def make_landed_tree(root: Path, files: dict[str, str], with_licenses: bool = True) -> Path:
    write_files(root / "drake", files)
    if with_licenses:
        (root / "LICENSE.TXT").write_text("synthetic BSD text\n", encoding="utf-8")
        (root / "LICENSE.Apache-2.0.txt").write_text(
            "synthetic Apache text\n", encoding="utf-8"
        )
    return root


def run_audit(landed_root: Path, upstream_clone: Path, ledger: Path):
    return subprocess.run(
        [
            sys.executable,
            str(PROVENANCE_AUDIT),
            "--landed-root",
            str(landed_root),
            "--upstream-clone",
            str(upstream_clone),
            "--disposition-ledger",
            str(ledger),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )


def build_case(work: Path, name: str) -> tuple[Path, Path, Path]:
    case_root = work / name
    clone = make_upstream_clone(case_root / "upstream", UPSTREAM_FILES, PINNED_COMMIT)
    landed = make_landed_tree(case_root / "landed", UPSTREAM_FILES)
    ledger = case_root / "ledger.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    return landed, clone, ledger


def case_matching_tree_passes(work: Path) -> None:
    landed, clone, ledger = build_case(work, "matching")
    result = run_audit(landed, clone, ledger)
    record_failure_unless(
        result.returncode == 0,
        "a landed tree matching the ledger and upstream must pass\n" + result.stdout,
    )
    record_failure_unless(
        "unmodified" in result.stdout,
        "an unmodified Apache-2.0 file must be reported as unmodified\n" + result.stdout,
    )


def case_clone_at_wrong_revision_fails(work: Path) -> None:
    case_root = work / "wrong_revision"
    clone = make_upstream_clone(
        case_root / "upstream", UPSTREAM_FILES, "f" * 40
    )
    landed = make_landed_tree(case_root / "landed", UPSTREAM_FILES)
    ledger = case_root / "ledger.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    result = run_audit(landed, clone, ledger)
    record_failure_unless(
        result.returncode == 1 and "but the ledger pins" in result.stdout,
        "a clone at another revision must not certify provenance\n" + result.stdout,
    )


def case_admitted_but_not_landed_fails(work: Path) -> None:
    case_root = work / "not_landed"
    clone = make_upstream_clone(case_root / "upstream", UPSTREAM_FILES, PINNED_COMMIT)
    incomplete = {k: v for k, v in UPSTREAM_FILES.items() if k != "support/shared.h"}
    landed = make_landed_tree(case_root / "landed", incomplete)
    ledger = case_root / "ledger.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    result = run_audit(landed, clone, ledger)
    record_failure_unless(
        result.returncode == 1 and "is not in the landed tree" in result.stdout,
        "an admitted file missing from the landed tree must be reported\n"
        + result.stdout,
    )


def case_landed_but_not_admitted_fails(work: Path) -> None:
    case_root = work / "not_admitted"
    clone = make_upstream_clone(case_root / "upstream", UPSTREAM_FILES, PINNED_COMMIT)
    extra = dict(UPSTREAM_FILES)
    extra["support/stowaway.h"] = "#pragma once\n"
    landed = make_landed_tree(case_root / "landed", extra)
    ledger = case_root / "ledger.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    result = run_audit(landed, clone, ledger)
    record_failure_unless(
        result.returncode == 1 and "the ledger does not admit it" in result.stdout,
        "a landed file nobody admitted must be reported\n" + result.stdout,
    )


def case_landed_file_absent_upstream_fails(work: Path) -> None:
    case_root = work / "absent_upstream"
    thinner_upstream = {
        k: v for k, v in UPSTREAM_FILES.items() if k != "support/shared.h"
    }
    clone = make_upstream_clone(case_root / "upstream", thinner_upstream, PINNED_COMMIT)
    landed = make_landed_tree(case_root / "landed", UPSTREAM_FILES)
    ledger = case_root / "ledger.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    result = run_audit(landed, clone, ledger)
    record_failure_unless(
        result.returncode == 1 and "does not exist upstream at that path" in result.stdout,
        "a landed file with no upstream path must not be claimed as provenanced\n"
        + result.stdout,
    )


def case_missing_licence_text_fails(work: Path) -> None:
    case_root = work / "missing_licence"
    clone = make_upstream_clone(case_root / "upstream", UPSTREAM_FILES, PINNED_COMMIT)
    landed = make_landed_tree(case_root / "landed", UPSTREAM_FILES, with_licenses=False)
    ledger = case_root / "ledger.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    result = run_audit(landed, clone, ledger)
    record_failure_unless(
        result.returncode == 1 and "is not present beside the source it covers" in result.stdout,
        "a missing licence text must be reported\n" + result.stdout,
    )


def case_modified_apache_file_without_notice_fails(work: Path) -> None:
    case_root = work / "modified_no_notice"
    clone = make_upstream_clone(case_root / "upstream", UPSTREAM_FILES, PINNED_COMMIT)
    modified = dict(UPSTREAM_FILES)
    modified["support/licensed.cc"] = UPSTREAM_FILES["support/licensed.cc"].replace(
        "return 1;", "return 2;"
    )
    landed = make_landed_tree(case_root / "landed", modified)
    ledger = case_root / "ledger.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    result = run_audit(landed, clone, ledger)
    record_failure_unless(
        result.returncode == 1 and "section 4(b) requires" in result.stdout,
        "a modified Apache-2.0 file with no modification notice must be reported\n"
        + result.stdout,
    )


def case_modified_apache_file_with_notice_passes(work: Path) -> None:
    case_root = work / "modified_with_notice"
    clone = make_upstream_clone(case_root / "upstream", UPSTREAM_FILES, PINNED_COMMIT)
    modified = dict(UPSTREAM_FILES)
    modified["support/licensed.cc"] = UPSTREAM_FILES["support/licensed.cc"].replace(
        "return 1;", "return 2;"
    ) + (
        "\n/* This file has been modified by the OpenRailVehicleDynamics project. */\n"
    )
    landed = make_landed_tree(case_root / "landed", modified)
    ledger = case_root / "ledger.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    result = run_audit(landed, clone, ledger)
    record_failure_unless(
        result.returncode == 0,
        "a modified Apache-2.0 file that carries its notice must pass\n" + result.stdout,
    )
    record_failure_unless(
        "modified, notice present" in result.stdout,
        "the report must say the notice is present\n" + result.stdout,
    )


def case_modified_bsd_file_needs_no_notice(work: Path) -> None:
    # BSD-3-Clause has no modification-notice clause. Demanding one would be this
    # tool inventing an obligation, which is its own kind of wrong answer.
    case_root = work / "modified_bsd"
    clone = make_upstream_clone(case_root / "upstream", UPSTREAM_FILES, PINNED_COMMIT)
    modified = dict(UPSTREAM_FILES)
    modified["candidate/root.h"] = "#pragma once\n// root, changed by us\n"
    landed = make_landed_tree(case_root / "landed", modified)
    ledger = case_root / "ledger.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    result = run_audit(landed, clone, ledger)
    record_failure_unless(
        result.returncode == 0,
        "a modified BSD-3-Clause file must not be asked for a notice it does not owe\n"
        + result.stdout,
    )


def main() -> int:
    cases = (
        case_matching_tree_passes,
        case_clone_at_wrong_revision_fails,
        case_admitted_but_not_landed_fails,
        case_landed_but_not_admitted_fails,
        case_landed_file_absent_upstream_fails,
        case_missing_licence_text_fails,
        case_modified_apache_file_without_notice_fails,
        case_modified_apache_file_with_notice_passes,
        case_modified_bsd_file_needs_no_notice,
    )
    with tempfile.TemporaryDirectory(prefix="orvd_verify_provenance.") as work_directory:
        work = Path(work_directory)
        for case in cases:
            case(work)
    if failure_count > 0:
        print(f"{failure_count} provenance audit check(s) failed", file=sys.stderr)
        return 1
    print(f"provenance audit verified across {len(cases)} cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
