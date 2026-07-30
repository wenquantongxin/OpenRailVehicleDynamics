#!/usr/bin/env python3
"""Checks that the product source boundary distinguishes crossings from valid source.

Half the cases here are negative controls, and they are the more important half.
A boundary check that also fails on `discard`, on an unclassified header, or on a
runtime header G20-G28 has not replaced yet would fail on conditions the project
currently expects, and a gate that fires on expected conditions gets turned off.
Those three cases are therefore asserted to pass, not merely left untested.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True

BOUNDARY_TOOL = (
    Path(__file__).resolve().parent / "verify_product_source_drake_boundary.py"
)

BASE_LEDGER = """\
source_repository https://example.invalid/synthetic/upstream.git
source_commit 0123456789abcdef0123456789abcdef01234567
source_tag v0.0.0
license_spdx BSD-3-Clause
license_file LICENSE.TXT
candidate_directory candidate

forbidden_prefix geometry/  # geometry is outside the rigid runtime boundary
forbidden multibody/tree/deformable_body.h  # deformable is outside it too

vendor candidate/root.h  # the candidate under test
vendor support/shared.h  # support the candidate includes
discard support/unused.h  # not needed today; a later goal may admit it
"""

failure_count = 0


def record_failure_unless(condition: bool, failure_description: str) -> None:
    global failure_count
    if not condition:
        print(f"FAILED: {failure_description}", file=sys.stderr)
        failure_count += 1


def build_repository(
    root: Path,
    drake_files: dict[str, str],
    first_party_files: dict[str, str] | None = None,
    runtime_files: dict[str, str] | None = None,
    ledger_text: str = BASE_LEDGER,
) -> Path:
    for relative_path, content in drake_files.items():
        path = root / "external" / "drake_mbtree" / "drake" / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    for relative_path, content in (first_party_files or {}).items():
        path = root / "external" / "drake_mbtree" / "orvd_implementations" / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    runtime_root = root / "libs"
    runtime_root.mkdir(parents=True, exist_ok=True)
    for relative_path, content in (runtime_files or {}).items():
        path = runtime_root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    ledger = root / "SOURCE_DISPOSITION.txt"
    ledger.write_text(ledger_text, encoding="utf-8")
    return ledger


def run_tool(repository_root: Path, ledger: Path):
    return subprocess.run(
        [
            sys.executable,
            str(BOUNDARY_TOOL),
            "--repository-root",
            str(repository_root),
            "--disposition-ledger",
            str(ledger),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )


CLEAN_TREE = {
    "candidate/root.h": '#pragma once\n#include "drake/support/shared.h"\n',
    "support/shared.h": "#pragma once\n// shared\n",
}


def case_clean_product_passes(work: Path) -> None:
    root = work / "clean"
    ledger = build_repository(root, CLEAN_TREE)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 0,
        "a product that crosses no boundary must pass\n" + result.stdout,
    )


def case_exact_forbidden_include_fails(work: Path) -> None:
    root = work / "exact_forbidden"
    tree = dict(CLEAN_TREE)
    tree["candidate/root.h"] = (
        '#pragma once\n#include "drake/multibody/tree/deformable_body.h"\n'
    )
    ledger = build_repository(root, tree)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 1 and "the ledger marks it forbidden" in result.stdout,
        "including a file the ledger marks forbidden must fail\n" + result.stdout,
    )


def case_forbidden_prefix_include_fails(work: Path) -> None:
    root = work / "prefix_forbidden"
    tree = dict(CLEAN_TREE)
    tree["candidate/root.h"] = '#pragma once\n#include "drake/geometry/query_object.h"\n'
    ledger = build_repository(root, tree)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 1
        and "under the forbidden prefix 'geometry/'" in result.stdout,
        "including a file under a forbidden prefix must fail\n" + result.stdout,
    )


def case_landed_forbidden_file_fails(work: Path) -> None:
    root = work / "landed_forbidden"
    tree = dict(CLEAN_TREE)
    # Nobody includes it; it simply exists in the landed tree, which is enough.
    tree["multibody/tree/deformable_body.h"] = "#pragma once\n"
    ledger = build_repository(root, tree)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 1 and "is landed, but" in result.stdout,
        "a forbidden file that landed must fail even if nothing includes it\n"
        + result.stdout,
    )


def case_first_party_implementation_crossing_fails(work: Path) -> None:
    root = work / "first_party_crossing"
    ledger = build_repository(
        root,
        CLEAN_TREE,
        {"our_own.cc": '#include "drake/geometry/query_object.h"\nint Ours() { return 1; }\n'},
    )
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 1 and "our_own.cc" in result.stdout,
        "ORVD's own implementation crossing the boundary must fail\n" + result.stdout,
    )


def case_runtime_source_crossing_fails(work: Path) -> None:
    root = work / "runtime_source_crossing"
    ledger = build_repository(
        root,
        CLEAN_TREE,
        runtime_files={
            "multibody_runtime/native_context.cc": (
                '#include "drake/geometry/query_object.h"\n'
                "int NativeContext() { return 1; }\n"
            )
        },
    )
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 1 and "libs/multibody_runtime" in result.stdout,
        "first-party runtime source under libs must be scanned\n" + result.stdout,
    )


def case_common_cpp_source_suffixes_are_scanned(work: Path) -> None:
    for suffix in (".hh", ".hxx", ".inl", ".inc", ".ipp", ".tcc", ".cxx"):
        root = work / f"runtime_source_suffix_{suffix[1:]}"
        crossing_path = f"multibody_runtime/forbidden_crossing{suffix}"
        ledger = build_repository(
            root,
            CLEAN_TREE,
            runtime_files={
                crossing_path: '#include "drake/geometry/query_object.h"\n'
            },
        )
        result = run_tool(root, ledger)
        record_failure_unless(
            result.returncode == 1 and crossing_path in result.stdout,
            f"first-party C++ source with suffix {suffix} must be scanned\n"
            + result.stdout,
        )


def case_vendor_cannot_override_forbidden_prefix(work: Path) -> None:
    # A per-file line must not be able to license an architectural crossing.
    root = work / "vendor_override"
    tree = dict(CLEAN_TREE)
    tree["candidate/root.h"] = '#pragma once\n#include "drake/geometry/query_object.h"\n'
    ledger_text = BASE_LEDGER + "vendor geometry/query_object.h  # attempted override\n"
    ledger = build_repository(root, tree, ledger_text=ledger_text)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 1
        and "under the forbidden prefix 'geometry/'" in result.stdout,
        "a vendor line must not override a forbidden prefix\n" + result.stdout,
    )


def case_discarded_include_is_not_a_crossing(work: Path) -> None:
    # Negative control: `discard` means "not needed today", not "must never be
    # here". Failing on it would re-implement the admission closure.
    root = work / "discarded"
    tree = dict(CLEAN_TREE)
    tree["candidate/root.h"] = '#pragma once\n#include "drake/support/unused.h"\n'
    tree["support/unused.h"] = "#pragma once\n"
    ledger = build_repository(root, tree)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 0,
        "including a discarded file is an admission question, not a boundary"
        " crossing\n" + result.stdout,
    )


def case_unclassified_include_is_not_a_crossing(work: Path) -> None:
    # Negative control: the admission closure reports unclassified edges. This
    # tool answering the same question would make two tools authoritative.
    root = work / "unclassified"
    tree = dict(CLEAN_TREE)
    tree["candidate/root.h"] = '#pragma once\n#include "drake/support/nobody_classified.h"\n'
    ledger = build_repository(root, tree)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 0,
        "an unclassified include is the closure analyzer's business, not this"
        " tool's\n" + result.stdout,
    )


def case_unreplaced_runtime_include_is_not_a_crossing(work: Path) -> None:
    # Negative control: the tree still includes the runtime headers G20-G28
    # replaces. Failing on those would make this gate red for the whole of
    # subgoals 8 to 10, which is when it most needs to be readable.
    root = work / "runtime_header"
    tree = dict(CLEAN_TREE)
    tree["candidate/root.h"] = (
        '#pragma once\n#include "drake/systems/framework/context.h"\n'
    )
    ledger = build_repository(root, tree)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 0,
        "a runtime header awaiting replacement is not a forbidden crossing\n"
        + result.stdout,
    )


def case_commented_forbidden_include_is_not_source(work: Path) -> None:
    root = work / "commented_include"
    tree = dict(CLEAN_TREE)
    tree["candidate/root.h"] = (
        "#pragma once\n"
        "/*\n"
        '#include "drake/geometry/query_object.h"\n'
        "*/\n"
        '#include "drake/support/shared.h"\n'
    )
    ledger = build_repository(root, tree)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 0,
        "an include inside a block comment is not a source dependency\n"
        + result.stdout,
    )


def case_missing_product_root_is_rejected(work: Path) -> None:
    root = work / "missing_root"
    ledger = build_repository(root, CLEAN_TREE)
    (root / "libs").rmdir()
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 2
        and "product source root does not exist: libs" in result.stdout,
        "every requested product source root must exist\n" + result.stdout,
    )


def case_empty_product_roots_are_rejected(work: Path) -> None:
    root = work / "empty_roots"
    (root / "external" / "drake_mbtree").mkdir(parents=True)
    (root / "libs").mkdir()
    ledger = root / "SOURCE_DISPOSITION.txt"
    ledger.write_text(BASE_LEDGER, encoding="utf-8")
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 2
        and "contain no supported source files" in result.stdout,
        "an empty scan must not certify a source boundary\n" + result.stdout,
    )


def case_symbolic_linked_source_is_rejected(work: Path) -> None:
    root = work / "linked_source"
    ledger = build_repository(root, CLEAN_TREE)
    hidden_source = work / "source_outside_product_roots"
    hidden_source.mkdir()
    (hidden_source / "hidden.cc").write_text(
        '#include "drake/geometry/query_object.h"\n', encoding="utf-8"
    )
    try:
        (root / "libs" / "linked_source").symlink_to(
            hidden_source, target_is_directory=True
        )
    except OSError as error:
        # A Windows host without Developer Mode or symbolic-link privilege
        # cannot construct this negative fixture. The production check still
        # rejects links when they exist; only this one discrimination case is
        # unavailable on that host, and the remaining cases continue to run.
        if sys.platform == "win32" and getattr(error, "winerror", None) == 1314:
            print("SKIPPED: symbolic-link fixture requires Windows link privilege")
            return
        raise
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 2 and "symbolic link" in result.stdout,
        "a directory link must not place source outside the traversal silently\n"
        + result.stdout,
    )


def case_unsupported_include_operand_is_an_input_error(work: Path) -> None:
    root = work / "unsupported_include"
    tree = dict(CLEAN_TREE)
    tree["candidate/root.h"] = "#pragma once\n#include DRAKE_SELECTED_HEADER\n"
    ledger = build_repository(root, tree)
    result = run_tool(root, ledger)
    record_failure_unless(
        result.returncode == 2 and "BOUNDARY INPUT ERROR" in result.stdout,
        "an include the scanner cannot interpret must be an input error, not a"
        " forbidden crossing\n" + result.stdout,
    )


def main() -> int:
    cases = (
        case_clean_product_passes,
        case_exact_forbidden_include_fails,
        case_forbidden_prefix_include_fails,
        case_landed_forbidden_file_fails,
        case_first_party_implementation_crossing_fails,
        case_runtime_source_crossing_fails,
        case_common_cpp_source_suffixes_are_scanned,
        case_vendor_cannot_override_forbidden_prefix,
        case_discarded_include_is_not_a_crossing,
        case_unclassified_include_is_not_a_crossing,
        case_unreplaced_runtime_include_is_not_a_crossing,
        case_commented_forbidden_include_is_not_source,
        case_missing_product_root_is_rejected,
        case_empty_product_roots_are_rejected,
        case_symbolic_linked_source_is_rejected,
        case_unsupported_include_operand_is_an_input_error,
    )
    with tempfile.TemporaryDirectory(prefix="orvd_verify_product_boundary.") as work:
        for case in cases:
            case(Path(work))
    if failure_count > 0:
        print(f"{failure_count} product source boundary check(s) failed", file=sys.stderr)
        return 1
    print(f"product source boundary verified across {len(cases)} cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
