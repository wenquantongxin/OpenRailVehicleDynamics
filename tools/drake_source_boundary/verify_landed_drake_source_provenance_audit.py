#!/usr/bin/env python3
"""Exercise the provenance audit against synthetic Git repositories."""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True

PROVENANCE_AUDIT = (
    Path(__file__).resolve().parent
    / "verify_landed_drake_source_provenance.py"
)

SYNTHETIC_BSD_LICENSE = """\
Synthetic BSD-3-Clause licence text used only by this test.
"""

SYNTHETIC_APACHE_LICENSE = """\
Apache License
Version 2.0, January 2004
TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION
1. Definitions.
2. Grant of Copyright License.
3. Grant of Patent License.
4. Redistribution.
You must cause any modified files to carry prominent notices
stating that You changed the files;
5. Submission of Contributions.
6. Trademarks.
7. Disclaimer of Warranty.
8. Limitation of Liability.
9. Accepting Warranty or Additional Liability.
END OF TERMS AND CONDITIONS
"""

APACHE_SOURCE_HEADER = """\
/* Portions copyright (c) 2014 Somebody Else.

Licensed under the Apache License, Version 2.0. */
"""

ORVD_MODIFICATION_NOTICE = """\
/* This file has been modified by the OpenRailVehicleDynamics project.
   See external/drake_mbtree/DRAKE_SOURCE_MODIFICATIONS.md for details. */
"""

UPSTREAM_VENDOR_FILES = {
    "candidate/root.h": "#pragma once\n// root\n",
    "support/shared.h": "#pragma once\n// shared\n",
    "support/licensed.cc": (
        APACHE_SOURCE_HEADER
        + "\n#include <cstddef>\n\nint Licensed() { return 1; }\n"
    ),
}

failure_count = 0


def record_failure_unless(condition: bool, failure_description: str) -> None:
    global failure_count
    if not condition:
        print(f"FAILED: {failure_description}", file=sys.stderr)
        failure_count += 1


def run_git(
    git_executable: str, repository_root: Path, arguments: list[str]
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [git_executable, "-C", str(repository_root), *arguments],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )


def require_git_success(
    git_executable: str, repository_root: Path, arguments: list[str]
) -> str:
    result = run_git(git_executable, repository_root, arguments)
    if result.returncode != 0:
        raise RuntimeError(
            f"Git command failed: {' '.join(arguments)}\n{result.stdout}"
        )
    return result.stdout.strip()


def write_files(root: Path, files: dict[str, str]) -> None:
    for relative_path, content in files.items():
        path = root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")


def make_upstream_repository(root: Path, git_executable: str) -> str:
    root.mkdir(parents=True)
    write_files(root, UPSTREAM_VENDOR_FILES)
    (root / "LICENSE.TXT").write_text(
        SYNTHETIC_BSD_LICENSE, encoding="utf-8"
    )
    require_git_success(git_executable, root, ["init", "-q"])
    require_git_success(
        git_executable,
        root,
        ["config", "user.email", "synthetic@example.invalid"],
    )
    require_git_success(
        git_executable, root, ["config", "user.name", "Synthetic Upstream"]
    )
    require_git_success(git_executable, root, ["add", "-A"])
    require_git_success(
        git_executable,
        root,
        ["-c", "commit.gpgSign=false", "commit", "-q", "-m", "synthetic upstream"],
    )
    pinned_commit = require_git_success(
        git_executable, root, ["rev-parse", "HEAD"]
    )
    require_git_success(
        git_executable,
        root,
        ["-c", "tag.gpgSign=false", "tag", "v0.0.0"],
    )
    return pinned_commit


def make_landed_tree(root: Path, files: dict[str, str]) -> Path:
    write_files(root / "drake", files)
    (root / "LICENSE.TXT").write_text(
        SYNTHETIC_BSD_LICENSE, encoding="utf-8"
    )
    (root / "LICENSE.Apache-2.0.txt").write_text(
        SYNTHETIC_APACHE_LICENSE, encoding="utf-8"
    )
    return root


def ledger_text(
    pinned_commit: str,
    *,
    include_apache_record: bool = True,
    extra_vendor_path: str | None = None,
) -> str:
    lines = [
        "source_repository https://example.invalid/synthetic/upstream.git",
        f"source_commit {pinned_commit}",
        "source_tag v0.0.0",
        "license_spdx BSD-3-Clause",
        "license_file LICENSE.TXT",
        "candidate_directory candidate",
        "vendor candidate/root.h  # the candidate under audit",
        "vendor support/shared.h  # support the candidate includes",
        "vendor support/licensed.cc  # support carrying its own per-file licence",
    ]
    if extra_vendor_path is not None:
        lines.append(f"vendor {extra_vendor_path}  # synthetic absent upstream")
    if include_apache_record:
        lines.append("file_license support/licensed.cc Apache-2.0")
    return "\n".join(lines) + "\n"


def run_audit(
    landed_root: Path,
    upstream_clone: Path,
    ledger_path: Path,
    git_executable: str,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(PROVENANCE_AUDIT),
            "--landed-root",
            str(landed_root),
            "--upstream-clone",
            str(upstream_clone),
            "--disposition-ledger",
            str(ledger_path),
            "--git-executable",
            git_executable,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )


def build_case(
    work: Path, name: str, git_executable: str
) -> tuple[Path, Path, Path, str]:
    case_root = work / name
    clone = case_root / "upstream"
    pinned_commit = make_upstream_repository(clone, git_executable)
    landed = make_landed_tree(case_root / "landed", UPSTREAM_VENDOR_FILES)
    ledger = case_root / "ledger.txt"
    ledger.write_text(ledger_text(pinned_commit), encoding="utf-8")
    return landed, clone, ledger, pinned_commit


def commit_worktree_change(
    clone: Path, git_executable: str, relative_path: str, content: str
) -> str:
    path = clone / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    require_git_success(git_executable, clone, ["add", relative_path])
    require_git_success(
        git_executable,
        clone,
        ["-c", "commit.gpgSign=false", "commit", "-q", "-m", "later change"],
    )
    return require_git_success(git_executable, clone, ["rev-parse", "HEAD"])


def case_matching_tree_passes(work: Path, git_executable: str) -> None:
    landed, clone, ledger, _ = build_case(work, "matching", git_executable)
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 0,
        "a matching landed tree must pass\n" + result.stdout,
    )


def case_matching_tree_with_crlf_passes(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(work, "matching_crlf", git_executable)
    for path in landed.rglob("*"):
        if path.is_file():
            # Path.write_text() already uses the host text convention.  First
            # recover one canonical LF stream so this fixture creates CRLF,
            # rather than CRCRLF, when the test itself runs on Windows.
            content = path.read_bytes()
            content = content.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
            content = content.replace(b"\n", b"\r\n")
            path.write_bytes(content)
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 0,
        "line-ending conversion must not change source provenance\n" + result.stdout,
    )


def case_clone_head_at_another_commit_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(work, "wrong_head", git_executable)
    commit_worktree_change(
        clone,
        git_executable,
        "unrelated.txt",
        "a later unrelated commit\n",
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "clone HEAD is" in result.stdout,
        "a clone checked out at another commit must be reported\n" + result.stdout,
    )


def case_tag_at_another_commit_fails(work: Path, git_executable: str) -> None:
    landed, clone, ledger, pinned_commit = build_case(
        work, "wrong_tag", git_executable
    )
    later_commit = commit_worktree_change(
        clone, git_executable, "unrelated.txt", "a later unrelated commit\n"
    )
    require_git_success(
        git_executable,
        clone,
        ["update-ref", "refs/tags/v0.0.0", later_commit],
    )
    require_git_success(
        git_executable, clone, ["checkout", "-q", "--detach", pinned_commit]
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "tag v0.0.0 resolves to" in result.stdout,
        "a tag pointing away from the pinned commit must be reported\n"
        + result.stdout,
    )


def case_admitted_but_not_landed_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(work, "not_landed", git_executable)
    (landed / "drake/support/shared.h").unlink()
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "is not in the landed tree" in result.stdout,
        "an admitted file missing from the landed tree must be reported\n"
        + result.stdout,
    )


def case_landed_but_not_admitted_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(work, "not_admitted", git_executable)
    write_files(landed / "drake", {"support/stowaway.h": "#pragma once\n"})
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "the ledger does not admit it" in result.stdout,
        "a landed file nobody admitted must be reported\n" + result.stdout,
    )


def case_landed_file_absent_from_pinned_commit_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, pinned_commit = build_case(
        work, "absent_upstream", git_executable
    )
    write_files(landed / "drake", {"support/missing.h": "#pragma once\n"})
    ledger.write_text(
        ledger_text(pinned_commit, extra_vendor_path="support/missing.h"),
        encoding="utf-8",
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "does not exist at that path" in result.stdout,
        "a landed file absent from the pinned object tree must be reported\n"
        + result.stdout,
    )


def case_dirty_clone_cannot_hide_missing_notice(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(work, "dirty_clone", git_executable)
    modified = UPSTREAM_VENDOR_FILES["support/licensed.cc"].replace(
        "return 1;", "return 2;"
    )
    (landed / "drake/support/licensed.cc").write_text(
        modified, encoding="utf-8"
    )
    # Make the mutable working copy match landed exactly. The audit must still
    # compare landed with the pinned Git blob and demand the missing notice.
    (clone / "support/licensed.cc").write_text(modified, encoding="utf-8")
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "source header carries no" in result.stdout,
        "uncommitted upstream edits must not change the audit answer\n"
        + result.stdout,
    )


def case_missing_apache_metadata_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, pinned_commit = build_case(
        work, "missing_apache_metadata", git_executable
    )
    ledger.write_text(
        ledger_text(pinned_commit, include_apache_record=False),
        encoding="utf-8",
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "no matching file_license record" in result.stdout,
        "removing Apache metadata must not silence the Apache checks\n"
        + result.stdout,
    )


def case_altered_repository_license_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(
        work, "altered_repository_license", git_executable
    )
    (landed / "LICENSE.TXT").write_text("not the pinned text\n", encoding="utf-8")
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "does not match the licence text" in result.stdout,
        "an altered repository-level licence must be reported\n" + result.stdout,
    )


def case_incomplete_apache_license_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(
        work, "incomplete_apache_license", git_executable
    )
    (landed / "LICENSE.Apache-2.0.txt").write_text(
        "Apache License\n", encoding="utf-8"
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "is missing the Apache-2.0 title" in result.stdout,
        "an incomplete Apache-2.0 text must be reported\n" + result.stdout,
    )


def case_notice_after_code_fails(work: Path, git_executable: str) -> None:
    landed, clone, ledger, _ = build_case(
        work, "notice_after_code", git_executable
    )
    modified = UPSTREAM_VENDOR_FILES["support/licensed.cc"].replace(
        "return 1;", "return 2;"
    )
    (landed / "drake/support/licensed.cc").write_text(
        modified + "\n" + ORVD_MODIFICATION_NOTICE,
        encoding="utf-8",
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "source header carries no" in result.stdout,
        "a notice hidden after the code must not satisfy the header obligation\n"
        + result.stdout,
    )


def case_notice_text_outside_comment_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(
        work, "notice_outside_comment", git_executable
    )
    modified = UPSTREAM_VENDOR_FILES["support/licensed.cc"].replace(
        "return 1;", "return 2;"
    ).replace(
        "#include <cstddef>",
        (
            'constexpr char kMisleadingText[] = R"NOTICE('
            "/* This file has been modified by the OpenRailVehicleDynamics project.\n"
            "   See external/drake_mbtree/DRAKE_SOURCE_MODIFICATIONS.md for details. */"
            ')NOTICE";\n'
            "#include <cstddef>"
        ),
    )
    (landed / "drake/support/licensed.cc").write_text(
        modified, encoding="utf-8"
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "source header carries no" in result.stdout,
        "notice words outside a comment must not satisfy the obligation\n"
        + result.stdout,
    )


def case_modified_apache_file_with_header_notice_passes(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(
        work, "modified_with_notice", git_executable
    )
    modified = UPSTREAM_VENDOR_FILES["support/licensed.cc"].replace(
        "return 1;", "return 2;"
    ).replace(
        "#include <cstddef>",
        ORVD_MODIFICATION_NOTICE + "\n\n#include <cstddef>",
    )
    (landed / "drake/support/licensed.cc").write_text(
        modified, encoding="utf-8"
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 0 and "header notice present" in result.stdout,
        "a modified Apache file preserving its header and adding the notice must pass\n"
        + result.stdout,
    )


def case_modified_apache_header_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(
        work, "modified_apache_header", git_executable
    )
    modified = UPSTREAM_VENDOR_FILES["support/licensed.cc"].replace(
        "Somebody Else", "A Different Attribution"
    ).replace(
        "#include <cstddef>",
        ORVD_MODIFICATION_NOTICE + "\n\n#include <cstddef>",
    )
    (landed / "drake/support/licensed.cc").write_text(
        modified, encoding="utf-8"
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1 and "does not preserve its original" in result.stdout,
        "changing the original Apache header must be reported\n" + result.stdout,
    )


def case_original_apache_block_after_code_fails(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(
        work, "apache_block_after_code", git_executable
    )
    original = UPSTREAM_VENDOR_FILES["support/licensed.cc"]
    body_without_header = original.removeprefix(APACHE_SOURCE_HEADER + "\n")
    modified = (
        ORVD_MODIFICATION_NOTICE
        + "\n\n"
        + body_without_header.replace("return 1;", "return 2;")
        + "\n"
        + APACHE_SOURCE_HEADER
    )
    (landed / "drake/support/licensed.cc").write_text(
        modified, encoding="utf-8"
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 1
        and "does not preserve its original Apache-2.0 block" in result.stdout,
        "moving the original Apache block after code must be reported\n"
        + result.stdout,
    )


def case_modified_bsd_file_needs_no_notice(
    work: Path, git_executable: str
) -> None:
    landed, clone, ledger, _ = build_case(work, "modified_bsd", git_executable)
    (landed / "drake/candidate/root.h").write_text(
        "#pragma once\n// root changed by ORVD\n", encoding="utf-8"
    )
    result = run_audit(landed, clone, ledger, git_executable)
    record_failure_unless(
        result.returncode == 0,
        "a modified BSD file must not be asked for an Apache notice\n"
        + result.stdout,
    )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--git-executable",
        default="git",
        help="Git executable used to build and inspect the synthetic repositories",
    )
    arguments = parser.parse_args(argv)
    cases = (
        case_matching_tree_passes,
        case_matching_tree_with_crlf_passes,
        case_clone_head_at_another_commit_fails,
        case_tag_at_another_commit_fails,
        case_admitted_but_not_landed_fails,
        case_landed_but_not_admitted_fails,
        case_landed_file_absent_from_pinned_commit_fails,
        case_dirty_clone_cannot_hide_missing_notice,
        case_missing_apache_metadata_fails,
        case_altered_repository_license_fails,
        case_incomplete_apache_license_fails,
        case_notice_after_code_fails,
        case_notice_text_outside_comment_fails,
        case_modified_apache_file_with_header_notice_passes,
        case_modified_apache_header_fails,
        case_original_apache_block_after_code_fails,
        case_modified_bsd_file_needs_no_notice,
    )
    with tempfile.TemporaryDirectory(
        prefix="orvd_verify_provenance."
    ) as work_directory:
        work = Path(work_directory)
        for case in cases:
            case(work, arguments.git_executable)
    if failure_count > 0:
        print(f"{failure_count} provenance audit check(s) failed", file=sys.stderr)
        return 1
    print(f"provenance audit verified across {len(cases)} cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
