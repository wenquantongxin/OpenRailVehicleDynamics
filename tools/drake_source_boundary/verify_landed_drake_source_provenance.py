#!/usr/bin/env python3
"""Audit the landed Drake sources against one pinned Git object tree.

The disposition ledger names an upstream repository, commit, tag, and path for
every vendored file. This tool checks those facts against the commit objects in a
local clone and the source that ORVD actually lands. It deliberately ignores the
clone's mutable working tree: uncommitted edits must not be able to change the
answer.

Identity is repository + commit + path, never a frozen file hash. Divergence from
upstream is expected because the landed tree has documented double-only changes.
For files whose own Apache-2.0 header requires a modification notice, the audit
also checks that the original header remains and a notice appears in the source
header rather than somewhere after the code.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True

from calculate_required_drake_source_closure import LedgerError, parse_ledger

APACHE_LICENSE_IDENTIFIER = "Apache-2.0"
APACHE_HEADER_MARKER = b"Licensed under the Apache License, Version 2.0"
ORVD_MODIFICATION_NOTICE_BLOCK = (
    b"/* This file has been modified by the OpenRailVehicleDynamics project.\n"
    b"   See external/drake_mbtree/DRAKE_SOURCE_MODIFICATIONS.md for details. */"
)
INCLUDE_DIRECTIVE_PATTERN = re.compile(br"(?m)^\s*#\s*include\b")
BLOCK_COMMENT_PATTERN = re.compile(br"/\*.*?\*/", re.DOTALL)
APACHE_LICENSE_REQUIRED_FRAGMENTS = (
    b"Apache License",
    b"Version 2.0, January 2004",
    b"TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION",
    b"1. Definitions.",
    b"2. Grant of Copyright License.",
    b"3. Grant of Patent License.",
    b"4. Redistribution.",
    b"You must cause any modified files to carry prominent notices",
    b"stating that You changed the files;",
    b"5. Submission of Contributions.",
    b"6. Trademarks.",
    b"7. Disclaimer of Warranty.",
    b"8. Limitation of Liability.",
    b"9. Accepting Warranty or Additional Liability.",
    b"END OF TERMS AND CONDITIONS",
)


def run_git(
    git_executable: str, repository_root: Path, arguments: list[str]
) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        [git_executable, "-C", str(repository_root), *arguments],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def resolve_commit(
    git_executable: str, repository_root: Path, revision: str
) -> str | None:
    result = run_git(
        git_executable,
        repository_root,
        ["rev-parse", "--verify", f"{revision}^{{commit}}"],
    )
    if result.returncode != 0:
        return None
    return result.stdout.decode("ascii", errors="replace").strip()


def list_paths_at_commit(
    git_executable: str, repository_root: Path, commit: str
) -> set[str] | None:
    result = run_git(
        git_executable,
        repository_root,
        ["ls-tree", "-r", "-z", "--name-only", commit],
    )
    if result.returncode != 0:
        return None
    return {
        encoded_path.decode("utf-8")
        for encoded_path in result.stdout.split(b"\0")
        if encoded_path
    }


def read_file_at_commit(
    git_executable: str, repository_root: Path, commit: str, path: str
) -> bytes | None:
    result = run_git(
        git_executable, repository_root, ["show", f"{commit}:{path}"]
    )
    if result.returncode != 0:
        return None
    return result.stdout


def extract_comment_containing(content: bytes, marker: bytes) -> bytes | None:
    for comment_match in BLOCK_COMMENT_PATTERN.finditer(content):
        if marker in comment_match.group():
            return comment_match.group()
    return None


def normalize_text_line_endings(content: bytes) -> bytes:
    return content.replace(b"\r\n", b"\n").replace(b"\r", b"\n")


def source_header_before_first_include(content: bytes) -> bytes:
    first_include = INCLUDE_DIRECTIVE_PATTERN.search(content)
    header_end = first_include.start() if first_include is not None else len(content)
    return content[:header_end]


def modification_notice_follows_license_block(
    content: bytes, original_license_block: bytes
) -> bool:
    source_header = source_header_before_first_include(content)
    license_offset = source_header.find(original_license_block)
    if license_offset < 0:
        return False
    after_license = source_header[
        license_offset + len(original_license_block) :
    ].lstrip()
    return after_license.startswith(ORVD_MODIFICATION_NOTICE_BLOCK)


def apache_license_text_has_required_sections(content: bytes) -> bool:
    return all(fragment in content for fragment in APACHE_LICENSE_REQUIRED_FRAGMENTS)


def audit_provenance(
    landed_root: Path,
    upstream_clone: Path,
    ledger_path: Path,
    git_executable: str,
) -> int:
    try:
        ledger = parse_ledger(ledger_path.read_text(encoding="utf-8"))
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

    try:
        clone_head = resolve_commit(git_executable, upstream_clone, "HEAD")
        pinned_commit = resolve_commit(
            git_executable, upstream_clone, ledger.source_commit
        )
        tagged_commit = resolve_commit(
            git_executable,
            upstream_clone,
            f"refs/tags/{ledger.source_tag}",
        )
    except OSError as error:
        print(f"PROVENANCE: cannot run Git: {error}", file=sys.stderr)
        return 2

    if clone_head is None:
        violations.append(
            f"the clone at {upstream_clone} has no readable Git HEAD"
        )
    elif clone_head != ledger.source_commit:
        violations.append(
            f"the clone HEAD is {clone_head}, but the ledger pins"
            f" {ledger.source_commit}"
        )
    if pinned_commit is None:
        violations.append(
            f"the clone does not contain the pinned commit {ledger.source_commit}"
        )
    elif pinned_commit != ledger.source_commit:
        violations.append(
            f"the ledger revision resolves to {pinned_commit}, not"
            f" {ledger.source_commit}"
        )
    if tagged_commit is None:
        violations.append(
            f"the clone has no readable tag refs/tags/{ledger.source_tag}"
        )
    elif tagged_commit != ledger.source_commit:
        violations.append(
            f"tag {ledger.source_tag} resolves to {tagged_commit}, not the pinned"
            f" commit {ledger.source_commit}"
        )

    vendored_paths = {
        path
        for path, disposition in ledger.dispositions.items()
        if disposition == "vendor"
    }
    landed_drake_root = landed_root / "drake"
    landed_paths = (
        {
            path.relative_to(landed_drake_root).as_posix()
            for path in landed_drake_root.rglob("*")
            if path.is_file()
        }
        if landed_drake_root.is_dir()
        else set()
    )

    for path in sorted(vendored_paths - landed_paths):
        violations.append(f"the ledger admits {path} but it is not in the landed tree")
    for path in sorted(landed_paths - vendored_paths):
        violations.append(f"{path} is in the landed tree but the ledger does not admit it")

    upstream_paths: set[str] = set()
    upstream_contents: dict[str, bytes] = {}
    if pinned_commit is not None:
        listed_paths = list_paths_at_commit(
            git_executable, upstream_clone, pinned_commit
        )
        if listed_paths is None:
            violations.append(
                f"the pinned commit {pinned_commit} cannot be read as a Git tree"
            )
        else:
            upstream_paths = listed_paths
            for path in sorted(vendored_paths):
                if path not in upstream_paths:
                    violations.append(
                        f"{path} is landed but does not exist at that path in the"
                        f" pinned commit {pinned_commit}"
                    )
                    continue
                content = read_file_at_commit(
                    git_executable, upstream_clone, pinned_commit, path
                )
                if content is None:
                    violations.append(
                        f"{path} exists in the pinned commit but its contents cannot"
                        " be read"
                    )
                    continue
                upstream_contents[path] = content

    # Drake's repository-level BSD text must be the text at the pinned commit,
    # not merely a file with the expected name.
    landed_repository_license = landed_root / ledger.license_file
    if not landed_repository_license.is_file():
        violations.append(
            f"the licence text {ledger.license_file} is not present beside the"
            " source it covers"
        )
    elif pinned_commit is not None:
        upstream_repository_license = read_file_at_commit(
            git_executable,
            upstream_clone,
            pinned_commit,
            ledger.license_file,
        )
        if upstream_repository_license is None:
            violations.append(
                f"the pinned commit does not contain {ledger.license_file}"
            )
        elif normalize_text_line_endings(
            landed_repository_license.read_bytes()
        ) != normalize_text_line_endings(upstream_repository_license):
            violations.append(
                f"{ledger.license_file} does not match the licence text at the"
                " pinned commit"
            )

    apache_license_path = landed_root / "LICENSE.Apache-2.0.txt"
    if not apache_license_path.is_file():
        violations.append(
            "the licence text LICENSE.Apache-2.0.txt is not present beside the"
            " source it covers"
        )
    elif not apache_license_text_has_required_sections(
        apache_license_path.read_bytes()
    ):
        violations.append(
            "LICENSE.Apache-2.0.txt is missing the Apache-2.0 title, a numbered"
            " section, or the section 4(b) modification-notice clause"
        )

    # Derive the Apache file set from the pinned source itself, then require the
    # ledger to describe exactly that set. Otherwise deleting a metadata line
    # could silence the only check that depends on it.
    upstream_apache_paths = {
        path
        for path, content in upstream_contents.items()
        if APACHE_HEADER_MARKER in content
    }
    ledger_apache_paths = {
        path
        for path, license_identifier in ledger.file_licenses.items()
        if license_identifier == APACHE_LICENSE_IDENTIFIER
    }
    for path in sorted(upstream_apache_paths - ledger_apache_paths):
        violations.append(
            f"{path} carries an Apache-2.0 header in the pinned source but has no"
            " matching file_license record"
        )
    for path in sorted(ledger_apache_paths - upstream_apache_paths):
        violations.append(
            f"{path} is labelled Apache-2.0 in the ledger but the pinned source"
            " carries no matching header"
        )

    print("\n  files carrying an Apache-2.0 header:")
    if not upstream_apache_paths:
        print("    none")
    for path in sorted(upstream_apache_paths):
        landed_file = landed_drake_root / path
        upstream_content = upstream_contents[path]
        if not landed_file.is_file():
            print(f"    {path}  (not landed)")
            continue
        landed_content = normalize_text_line_endings(landed_file.read_bytes())
        upstream_content = normalize_text_line_endings(upstream_content)
        original_license_block = extract_comment_containing(
            upstream_content, APACHE_HEADER_MARKER
        )
        if (
            original_license_block is None
            or original_license_block
            not in source_header_before_first_include(landed_content)
        ):
            violations.append(
                f"{path} does not preserve its original Apache-2.0 block in the"
                " source header"
            )
        modified = landed_content != upstream_content
        if not modified:
            print(f"    {path}  unmodified")
            continue
        has_header_notice = (
            original_license_block is not None
            and modification_notice_follows_license_block(
                landed_content, original_license_block
            )
        )
        print(
            f"    {path}  modified,"
            f" {'header notice present' if has_header_notice else 'HEADER NOTICE MISSING'}"
        )
        if not has_header_notice:
            violations.append(
                f"{path} is Apache-2.0 and has been modified, but its source"
                " header carries no prominent notice saying so"
            )

    print("\nRESULT")
    if violations:
        for violation in violations:
            print(f"  PROVENANCE VIOLATION: {violation}")
        return 1
    print(
        "  every landed file is admitted by the ledger and exists at the same"
        " path in the pinned Git object tree; the pinned tag and licence texts"
        " agree, and every modified Apache-2.0 file preserves its original"
        " header and carries an ORVD modification notice there"
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
        help="a local Git clone containing the pinned upstream commit and tag",
    )
    parser.add_argument(
        "--disposition-ledger",
        required=True,
        help="path to SOURCE_DISPOSITION.txt",
    )
    parser.add_argument(
        "--git-executable",
        default="git",
        help="Git executable used to read the pinned object tree",
    )
    arguments = parser.parse_args(argv)
    return audit_provenance(
        Path(arguments.landed_root),
        Path(arguments.upstream_clone),
        Path(arguments.disposition_ledger),
        arguments.git_executable,
    )


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
