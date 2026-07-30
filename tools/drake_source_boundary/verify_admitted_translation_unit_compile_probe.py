#!/usr/bin/env python3
"""Checks that the compile-and-link probe reports what it must and hides nothing.

Each case builds a synthetic source tree and ledger, so the probe is exercised on
inputs whose right answer is known by construction rather than by whatever the
Drake clone happens to contain.

The controls each target a distinct false success: incomplete staging, an
implicit or explicit include path exposing unadmitted Drake headers, one failure
masking the rest, object-path collisions, a compiler that produces no object, an
empty compilation set, or a linker that discards the missing symbol.
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path

COMPILE_AND_LINK_PROBE = (
    Path(__file__).resolve().parent
    / "compile_admitted_drake_translation_units_and_link_generated_objects.py"
)

failure_count = 0


def record_failure_unless(condition: bool, failure_description: str) -> None:
    global failure_count
    if not condition:
        print(f"FAILED: {failure_description}", file=sys.stderr)
        failure_count += 1


def write_source_tree(root: Path, files: dict[str, str]) -> None:
    for relative_path, content in files.items():
        path = root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    (root / "LICENSE.TXT").write_text("synthetic license\n", encoding="utf-8")
    git_directory = root / ".git"
    git_directory.mkdir(parents=True, exist_ok=True)
    (git_directory / "HEAD").write_text(
        "0123456789abcdef0123456789abcdef01234567\n",
        encoding="utf-8",
    )


compiler_under_test = "c++"


def run_probe(
    source_root: Path,
    ledger_path: Path,
    *,
    compiler: str | None = None,
    additional_arguments: list[str] | None = None,
    environment: dict[str, str] | None = None,
) -> subprocess.CompletedProcess:
    command = [
        sys.executable,
        str(COMPILE_AND_LINK_PROBE),
        "--drake-source-root",
        str(source_root),
        "--disposition-ledger",
        str(ledger_path),
        "--compiler",
        compiler or compiler_under_test,
        "--show-diagnostics",
    ]
    if additional_arguments:
        command.extend(additional_arguments)
    return subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=False,
        env=environment,
    )


LEDGER_HEADER = """\
source_repository https://example.invalid/synthetic/upstream.git
source_commit 0123456789abcdef0123456789abcdef01234567
source_tag v0.0.0
license_spdx BSD-3-Clause
license_file LICENSE.TXT
candidate_directory candidate
candidate_directory candidate_alpha
"""

# Two self-contained units that compile and link with nothing but the staged
# tree. Their paths deliberately collided under the probe's former underscore
# flattening, so the ordinary success case also proves distinct object paths.
ACCEPTED_TREE = {
    "candidate/alpha_beta.h": "int first_value();\n",
    "candidate/alpha_beta.cc": (
        '#include "drake/candidate/alpha_beta.h"\n' "int first_value() { return 1; }\n"
    ),
    "candidate_alpha/beta.h": "int second_value();\n",
    "candidate_alpha/beta.cc": (
        '#include "drake/candidate_alpha/beta.h"\n' "int second_value() { return 2; }\n"
    ),
}

ACCEPTED_LEDGER = (
    LEDGER_HEADER
    + """\
vendor candidate/alpha_beta.h  # declares the first unit
vendor candidate/alpha_beta.cc  # defines it
vendor candidate_alpha/beta.h  # declares the second unit
vendor candidate_alpha/beta.cc  # defines it
"""
)


def case_accepted_set_compiles_and_links(temporary_workspace: Path) -> None:
    source_root = temporary_workspace / "accepted_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = temporary_workspace / "accepted_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    result = run_probe(source_root, ledger)
    record_failure_unless(
        result.returncode == 0,
        f"a self-contained admitted set must compile and link; got "
        f"{result.returncode}\n{result.stdout}{result.stderr}",
    )
    record_failure_unless(
        "compiled:              2" in result.stdout
        and "link of the 2 object(s) that compiled: succeeded" in result.stdout,
        f"both units must compile and the link must be reported; got:\n{result.stdout}",
    )


def case_admitted_file_absent_from_source_fails_at_staging(
    temporary_workspace: Path,
) -> None:
    source_root = temporary_workspace / "absent_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = temporary_workspace / "absent_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER + "vendor candidate/never_written.cc  # admitted but absent\n",
        encoding="utf-8",
    )

    result = run_probe(source_root, ledger)
    record_failure_unless(
        result.returncode != 0,
        "an admitted file missing from the source tree must fail",
    )
    record_failure_unless(
        "STAGING: candidate/never_written.cc is admitted but absent" in result.stderr,
        f"the staging failure must name the file and the stage; got:\n{result.stderr}",
    )
    record_failure_unless(
        "translation units:" not in result.stdout and "LINK:" not in result.stderr,
        "a staging failure must stop before compilation and linking",
    )


def case_unadmitted_header_is_not_staged(temporary_workspace: Path) -> None:
    """A file present upstream but not admitted must not satisfy an include.

    This is the control that makes staging worth doing: run against the clone
    instead and this compiles, reporting a boundary the ledger never granted.
    """
    source_root = temporary_workspace / "unadmitted_source"
    tree = dict(ACCEPTED_TREE)
    tree["candidate/alpha_beta.cc"] = (
        '#include "drake/candidate/alpha_beta.h"\n'
        '#include "drake/support/not_admitted.h"\n'
        "int first_value() { return kNotAdmitted; }\n"
    )
    tree["support/not_admitted.h"] = "constexpr int kNotAdmitted = 7;\n"
    write_source_tree(source_root, tree)
    ledger = temporary_workspace / "unadmitted_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    implicit_include_root = temporary_workspace / "implicit_include_root"
    implicit_header = implicit_include_root / "drake/support/not_admitted.h"
    implicit_header.parent.mkdir(parents=True, exist_ok=True)
    implicit_header.write_text("constexpr int kNotAdmitted = 7;\n", encoding="utf-8")
    implicit_admitted_header = implicit_include_root / "drake/candidate/alpha_beta.h"
    implicit_admitted_header.parent.mkdir(parents=True, exist_ok=True)
    implicit_admitted_header.write_text("int first_value();\n", encoding="utf-8")
    polluted_environment = os.environ.copy()
    polluted_environment["CPATH"] = str(implicit_include_root)

    result = run_probe(source_root, ledger, environment=polluted_environment)
    record_failure_unless(
        result.returncode != 0,
        "an implicit include path must not expose an unadmitted Drake header",
    )
    record_failure_unless(
        "COMPILE: candidate/alpha_beta.cc failed" in result.stderr,
        f"the compile failure must name the unit and the stage; got:\n{result.stderr}",
    )
    record_failure_unless(
        "drake/support/not_admitted.h" in result.stderr,
        f"the compiler's own diagnostic must be preserved; got:\n{result.stderr}",
    )

    explicit_result = run_probe(
        source_root,
        ledger,
        additional_arguments=[
            "--third-party-include-directory",
            str(implicit_include_root),
        ],
    )
    record_failure_unless(
        explicit_result.returncode != 0
        and "third-party include directory exposes a Drake tree"
        in explicit_result.stderr,
        "an explicit third-party include root must not expose a Drake tree",
    )

    compiler_with_implicit_drake = temporary_workspace / "compiler_with_implicit_drake"
    compiler_with_implicit_drake.write_text(
        "#!/bin/sh\n"
        f"exec {shlex.quote(compiler_under_test)} "
        f'-I{shlex.quote(str(implicit_include_root))} "$@"\n',
        encoding="utf-8",
    )
    compiler_with_implicit_drake.chmod(0o755)
    default_search_result = run_probe(
        source_root,
        ledger,
        compiler=str(compiler_with_implicit_drake),
    )
    record_failure_unless(
        default_search_result.returncode != 0
        and "compiler default include search exposes a Drake tree"
        in default_search_result.stderr,
        "a compiler's default include search must not expose a Drake tree",
    )


def case_one_failure_does_not_mask_the_others(temporary_workspace: Path) -> None:
    source_root = temporary_workspace / "continue_source"
    tree = dict(ACCEPTED_TREE)
    tree["candidate/alpha_beta.cc"] = (
        '#include "drake/candidate/alpha_beta.h"\n'
        "int first_value() { return this_name_does_not_exist(); }\n"
    )
    write_source_tree(source_root, tree)
    ledger = temporary_workspace / "continue_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    result = run_probe(source_root, ledger)
    record_failure_unless(
        result.returncode != 0, "a translation unit that does not compile must fail"
    )
    # The point of the case: the healthy unit is still attempted and still
    # produces an object, so one broken file cannot hide the rest of the state.
    record_failure_unless(
        "compiled:              1" in result.stdout
        and "compile failures:      1" in result.stdout
        and "COMPILE: candidate/alpha_beta.cc failed" in result.stderr,
        f"the surviving unit must still compile; got:\n{result.stdout}",
    )


def case_unreferenced_undefined_symbol_still_breaks_the_link(
    temporary_workspace: Path,
) -> None:
    """Nothing calls the offending function, and the link must still fail.

    With an archive, or with section garbage collection on, the object would be
    dropped and the link would come out clean — reporting a boundary that only
    holds because the missing symbol was never reached.
    """
    source_root = temporary_workspace / "gc_source"
    tree = dict(ACCEPTED_TREE)
    tree["candidate_alpha/beta.cc"] = (
        '#include "drake/candidate_alpha/beta.h"\n'
        "int a_symbol_no_one_defines();\n"
        "// Never called from main, and never called from the other unit.\n"
        "int nobody_calls_this() { return a_symbol_no_one_defines(); }\n"
        "int second_value() { return 2; }\n"
    )
    write_source_tree(source_root, tree)
    ledger = temporary_workspace / "gc_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    result = run_probe(source_root, ledger)
    record_failure_unless(
        result.returncode != 0,
        "an undefined symbol reached only from an uncalled function must break "
        "the link",
    )
    record_failure_unless(
        "compile failures:      0" in result.stdout,
        f"the defect must surface at link, not at compile; got:\n{result.stdout}",
    )
    record_failure_unless(
        "LINK: generated objects did not link" in result.stderr
        and "a_symbol_no_one_defines" in result.stderr,
        f"the link failure must name the stage and the symbol; got:\n{result.stderr}",
    )


def case_ledger_without_translation_units_fails(temporary_workspace: Path) -> None:
    source_root = temporary_workspace / "header_only_source"
    write_source_tree(source_root, {"candidate/only_header.h": "int value();\n"})
    ledger = temporary_workspace / "header_only_ledger.txt"
    ledger.write_text(
        LEDGER_HEADER
        + "vendor candidate/only_header.h  # no implementation was admitted\n",
        encoding="utf-8",
    )

    result = run_probe(source_root, ledger)
    record_failure_unless(
        result.returncode != 0
        and "COMPILE: the ledger admits no translation unit" in result.stderr,
        "a ledger with no translation unit must not report success",
    )


def case_compiler_success_without_object_fails(temporary_workspace: Path) -> None:
    source_root = temporary_workspace / "no_object_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = temporary_workspace / "no_object_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    compiler_that_produces_no_object = (
        temporary_workspace / "compiler_that_produces_no_object"
    )
    compiler_that_produces_no_object.write_text(
        "#!/bin/sh\nexit 0\n",
        encoding="utf-8",
    )
    compiler_that_produces_no_object.chmod(0o755)

    result = run_probe(
        source_root,
        ledger,
        compiler=str(compiler_that_produces_no_object),
    )
    record_failure_unless(
        result.returncode != 0
        and "reported success but produced no object file" in result.stderr,
        "compiler success without a nonempty object must fail",
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check the admitted translation unit compile probe."
    )
    parser.add_argument(
        "--compiler",
        default="c++",
        help="compiler the probe should invoke; defaults to the platform default "
        "so the check runs standalone",
    )
    global compiler_under_test
    compiler_under_test = parser.parse_args().compiler

    cases = [
        case_accepted_set_compiles_and_links,
        case_admitted_file_absent_from_source_fails_at_staging,
        case_unadmitted_header_is_not_staged,
        case_one_failure_does_not_mask_the_others,
        case_unreferenced_undefined_symbol_still_breaks_the_link,
        case_ledger_without_translation_units_fails,
        case_compiler_success_without_object_fails,
    ]
    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary_workspace = Path(temporary_directory)
        for case in cases:
            case(temporary_workspace)

    if failure_count > 0:
        print(f"{failure_count} compile probe check(s) failed", file=sys.stderr)
        return 1
    print(f"admitted translation unit compile probe verified across {len(cases)} cases")
    return 0


if __name__ == "__main__":
    sys.exit(main())
