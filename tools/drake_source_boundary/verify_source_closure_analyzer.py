#!/usr/bin/env python3
"""Checks that the closure analyzer accepts what it should and rejects what it must.

Every case builds a small synthetic source tree in a temporary directory and runs
the analyzer against it. Nothing here reads Drake: an analyzer whose only proof of
correctness is a run against one real tree has been tested on one input, and a
gate that cannot be shown to fail is not a gate.

The cases are deliberately adversarial. Each rejection case starts from the
accepted tree and introduces exactly one defect, so a failure names that defect
rather than a difference in the fixture.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

ANALYZER = Path(__file__).resolve().parent / "calculate_required_drake_source_closure.py"

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


def run_analyzer(source_root: Path, ledger_path: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            sys.executable,
            str(ANALYZER),
            "--drake-source-root",
            str(source_root),
            "--disposition-ledger",
            str(ledger_path),
        ],
        capture_output=True,
        text=True,
        check=False,
    )


# A tiny stand-in for the real boundary: an admitted root that includes a second
# admitted header, whose implementation is split into a differently named unit,
# plus a generated header and a forbidden neighbour that nothing admitted reaches.
ACCEPTED_TREE = {
    "candidate/root.h": '#include "drake/multibody/tree/joint.h"\n',
    "multibody/tree/joint.h": (
        '#include "drake/multibody/tree/frame.h"\n'
        '#include "drake/common/autodiff.h"\n'
    ),
    "multibody/tree/joint.cc": '#include "drake/multibody/tree/joint.h"\n',
    "multibody/tree/frame.h": "// no further edges\n",
    "multibody/tree/frame.cc": '#include "drake/multibody/tree/frame.h"\n',
    # Implementation whose name does not share the header's stem, so it is only
    # found through an explicit `implementation` declaration.
    "multibody/tree/frame_extra_impl.cc": '#include "drake/multibody/tree/frame.h"\n',
    "common/autodiff.h": '#include "drake/common/autodiff_config.h"\n',
    "geometry/shape.h": "// forbidden neighbour, not reachable from the roots\n",
}

ACCEPTED_LEDGER = """\
source_repository https://example.invalid/synthetic/upstream.git
source_commit 0123456789abcdef0123456789abcdef01234567
source_tag v0.0.0
license_spdx BSD-3-Clause
license_file LICENSE.TXT
candidate_directory candidate
some_future_field that this analyzer does not read

vendor candidate/root.h  # the only closure seed
vendor multibody/tree/joint.h  # the joint interface under test
vendor multibody/tree/joint.cc  # its implementation
vendor multibody/tree/frame.h  # reached from joint.h
vendor multibody/tree/frame.cc  # frame's own implementation
vendor multibody/tree/frame_extra_impl.cc  # split implementation of frame.h
vendor common/autodiff.h  # reached from joint.h

implementation multibody/tree/frame.h multibody/tree/frame_extra_impl.cc

generated_header common/autodiff_config.h  # produced by the upstream build

forbidden_prefix geometry/  # geometry is outside the runtime boundary
"""


def case_accepted_closure(work: Path) -> None:
    source_root = work / "accepted_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = work / "accepted_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode == 0,
        f"a fully classified closure must be accepted; got {result.returncode}\n"
        f"{result.stdout}{result.stderr}",
    )
    record_failure_unless(
        "admitted roots:   1" in result.stdout
        and "files reached:    8" in result.stdout,
        f"the one candidate root must reach all seven support files; got:\n"
        f"{result.stdout}",
    )


def case_unclassified_edge(work: Path) -> None:
    source_root = work / "unclassified_source"
    tree = dict(ACCEPTED_TREE)
    tree["multibody/tree/frame.h"] = '#include "drake/math/rigid_transform.h"\n'
    tree["math/rigid_transform.h"] = "// nobody classified this\n"
    write_source_tree(source_root, tree)
    ledger = work / "unclassified_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0,
        "an edge to an unclassified file must fail",
    )
    record_failure_unless(
        "UNCLASSIFIED EDGE: math/rigid_transform.h" in result.stderr,
        f"the unclassified file must be named; got:\n{result.stderr}",
    )
    # The chain is what makes the report actionable: knowing a file is unclassified
    # is useless without knowing what pulled it in.
    record_failure_unless(
        "multibody/tree/frame.h -> math/rigid_transform.h" in result.stderr,
        f"the reaching chain must be reported; got:\n{result.stderr}",
    )


def case_forbidden_edge_by_prefix(work: Path) -> None:
    source_root = work / "forbidden_prefix_source"
    tree = dict(ACCEPTED_TREE)
    tree["multibody/tree/frame.h"] = '#include "drake/geometry/shape.h"\n'
    write_source_tree(source_root, tree)
    ledger = work / "forbidden_prefix_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0, "an edge under a forbidden prefix must fail"
    )
    record_failure_unless(
        "FORBIDDEN EDGE: geometry/shape.h" in result.stderr,
        f"the forbidden file must be named; got:\n{result.stderr}",
    )
    # Forbidden and unclassified must not be reported as the same thing: one means
    # the boundary was violated, the other that it was never drawn.
    record_failure_unless(
        "UNCLASSIFIED EDGE: geometry/shape.h" not in result.stderr,
        "a forbidden edge must not also be reported as unclassified",
    )


def case_forbidden_edge_by_file(work: Path) -> None:
    source_root = work / "forbidden_file_source"
    tree = dict(ACCEPTED_TREE)
    tree["multibody/tree/frame.h"] = (
        '#include "drake/multibody/tree/deformable_body.h"\n'
    )
    tree["multibody/tree/deformable_body.h"] = "// deformable bodies are out of scope\n"
    write_source_tree(source_root, tree)
    ledger = work / "forbidden_file_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER
        + "forbidden multibody/tree/deformable_body.h  # deformable bodies are FEM\n",
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0, "an edge to a file classified forbidden must fail"
    )
    record_failure_unless(
        "FORBIDDEN EDGE: multibody/tree/deformable_body.h" in result.stderr,
        f"the forbidden file must be named; got:\n{result.stderr}",
    )


def case_explicit_vendor_cannot_override_forbidden_prefix(work: Path) -> None:
    source_root = work / "forbidden_override_source"
    tree = dict(ACCEPTED_TREE)
    tree["candidate/root.h"] = '#include "drake/geometry/shape.h"\n'
    write_source_tree(source_root, tree)
    ledger = work / "forbidden_override_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER
        + "vendor geometry/shape.h  # an invalid attempt to override the boundary\n",
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0
        and "FORBIDDEN EDGE: geometry/shape.h" in result.stderr,
        f"an explicit disposition must not override a forbidden prefix; got:\n"
        f"{result.stderr}",
    )


def case_include_edge_supersedes_implementation_edge(work: Path) -> None:
    source_root = work / "arrival_order_source"
    write_source_tree(
        source_root,
        {
            "candidate/root.h": (
                '#include "drake/support/a_includer.h"\n'
                '#include "drake/support/z_header.h"\n'
            ),
            "support/a_includer.h": '#include "drake/support/z_header.cc"\n',
            "support/z_header.h": "// has a same-stem implementation\n",
            "support/z_header.cc": "// deliberately declined\n",
        },
    )
    ledger = work / "arrival_order_ledger.txt"
    ledger.write_text(
        """\
source_repository https://example.invalid/synthetic/upstream.git
source_commit 0123456789abcdef0123456789abcdef01234567
source_tag v0.0.0
license_spdx BSD-3-Clause
license_file LICENSE.TXT
candidate_directory candidate
vendor candidate/root.h  # the only closure seed
vendor support/a_includer.h  # includes the declined implementation
vendor support/z_header.h  # reaches its implementation by stem first
discard support/z_header.cc  # must still fail when a real include reaches it
""",
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0
        and "UNMET DEPENDENCY: support/z_header.cc" in result.stderr,
        f"a later include must not be hidden by an earlier implementation edge; got:\n"
        f"{result.stderr}",
    )


def case_forgotten_implementation_unit_is_caught(work: Path) -> None:
    """Vendoring a header but forgetting its implementation must not pass.

    The implementation is withheld from the ledger, so it is not a root and the
    stem relation is the only thing that can reach it. If that relation is not
    followed the file is never seen and the run wrongly succeeds.
    """
    source_root = work / "forgotten_implementation_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = work / "forgotten_implementation_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER.replace(
            "vendor multibody/tree/frame.cc  # frame's own implementation\n", ""
        ),
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0,
        "a vendored header whose implementation is unclassified must fail",
    )
    record_failure_unless(
        "UNCLASSIFIED EDGE: multibody/tree/frame.cc" in result.stderr,
        f"the forgotten implementation must be named; got:\n{result.stderr}",
    )
    record_failure_unless(
        "multibody/tree/frame.h -> multibody/tree/frame.cc" in result.stderr,
        f"the stem relation must appear in the chain; got:\n{result.stderr}",
    )


def case_forgotten_split_implementation_is_caught(work: Path) -> None:
    """The declared split-implementation relation must be walked as well.

    Same construction: the declared unit is withheld from the ledger, so only the
    explicit `implementation` line can reach it.
    """
    source_root = work / "forgotten_split_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = work / "forgotten_split_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER.replace(
            "vendor multibody/tree/frame_extra_impl.cc  # split implementation of frame.h\n",
            "",
        ),
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0,
        "a declared split implementation that is unclassified must fail",
    )
    record_failure_unless(
        "UNCLASSIFIED EDGE: multibody/tree/frame_extra_impl.cc" in result.stderr,
        f"the forgotten split implementation must be named; got:\n{result.stderr}",
    )
    record_failure_unless(
        "multibody/tree/frame.h -> multibody/tree/frame_extra_impl.cc" in result.stderr,
        f"the declared relation must appear in the chain; got:\n{result.stderr}",
    )


def case_generated_header_terminates_a_branch(work: Path) -> None:
    """A declared generated header is a decision, not a missing file."""
    source_root = work / "generated_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = work / "generated_ledger.txt"
    # Remove the generated_header declaration and the same tree must now fail,
    # because autodiff_config.h exists in no source tree.
    ledger.write_text(
        ACCEPTED_LEDGER.replace(
            "generated_header common/autodiff_config.h  # produced by the upstream build\n",
            "",
        ),
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0,
        "an undeclared header that exists in no source tree must fail",
    )
    record_failure_unless(
        "common/autodiff_config.h" in result.stderr,
        f"the undeclared generated header must be named; got:\n{result.stderr}",
    )


def case_generated_header_must_be_absent_from_source(work: Path) -> None:
    source_root = work / "false_generated_source"
    tree = dict(ACCEPTED_TREE)
    tree["candidate/root.h"] = '#include "drake/common/generated.h"\n'
    tree["common/generated.h"] = '#include "drake/geometry/shape.h"\n'
    write_source_tree(source_root, tree)
    ledger = work / "false_generated_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER
        + "generated_header common/generated.h  # falsely declared generated\n",
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0
        and "declared generated_header but present in the source tree"
        in result.stderr,
        f"a generated-header declaration must not hide real source; got:\n"
        f"{result.stderr}",
    )


def case_unclassified_candidate_is_rejected(work: Path) -> None:
    source_root = work / "unclassified_candidate_source"
    tree = dict(ACCEPTED_TREE)
    tree["candidate/forgotten.cc"] = "// a candidate omitted from the ledger\n"
    write_source_tree(source_root, tree)
    ledger = work / "unclassified_candidate_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode == 2
        and "candidate file is unclassified: candidate/forgotten.cc"
        in result.stderr,
        f"every file in a candidate directory must be classified; got:\n"
        f"{result.stderr}",
    )


def case_empty_admission_is_rejected(work: Path) -> None:
    source_root = work / "empty_admission_source"
    write_source_tree(source_root, {"candidate/root.h": "// deliberately declined\n"})
    ledger = work / "empty_admission_ledger.txt"
    ledger.write_text(
        """\
source_repository https://example.invalid/synthetic/upstream.git
source_commit 0123456789abcdef0123456789abcdef01234567
source_tag v0.0.0
license_spdx BSD-3-Clause
license_file LICENSE.TXT
candidate_directory candidate
discard candidate/root.h  # no admitted implementation remains
""",
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode == 2
        and "candidate directories contain no admitted vendor file" in result.stderr,
        f"an empty vendor boundary must not pass as a valid closure; got:\n"
        f"{result.stderr}",
    )


def case_unrecognized_include_operand_is_rejected(work: Path) -> None:
    source_root = work / "macro_include_source"
    tree = dict(ACCEPTED_TREE)
    tree["candidate/root.h"] = "#include DRAKE_GENERATED_INCLUDE\n"
    write_source_tree(source_root, tree)
    ledger = work / "macro_include_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode == 2 and "unsupported include operand" in result.stderr,
        f"an include the analyzer cannot resolve must fail loudly; got:\n"
        f"{result.stderr}",
    )


def case_reason_is_required(work: Path) -> None:
    """A disposition without a reason is not a decision, it is a placeholder."""
    source_root = work / "reasonless_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = work / "reasonless_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER.replace(
            "vendor multibody/tree/frame.h  # reached from joint.h",
            "vendor multibody/tree/frame.h",
        ),
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode == 2,
        f"a disposition with no reason must make the ledger unusable; "
        f"got {result.returncode}",
    )
    record_failure_unless(
        "carries no reason" in result.stderr,
        f"the reasonless entry must be named; got:\n{result.stderr}",
    )


def case_source_repository_is_required(work: Path) -> None:
    # A commit without a repository is not provenance: the same forty digits
    # exist in every fork and mirror, and none of them is named.
    source_root = work / "repository_metadata_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = work / "repository_metadata_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER.replace(
            "source_repository https://example.invalid/synthetic/upstream.git\n", ""
        ),
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode == 2 and "declares no source_repository" in result.stderr,
        f"a ledger without a source repository must be unusable; got:\n"
        f"{result.stderr}",
    )


def case_license_file_is_required(work: Path) -> None:
    source_root = work / "license_metadata_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = work / "license_metadata_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER.replace("license_file LICENSE.TXT\n", ""),
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode == 2 and "declares no license_file" in result.stderr,
        f"missing license metadata must make the ledger unusable; got:\n"
        f"{result.stderr}",
    )


def case_unreached_vendor_is_rejected(work: Path) -> None:
    source_root = work / "unreached_vendor_source"
    tree = dict(ACCEPTED_TREE)
    tree["support/unneeded.h"] = "// no candidate needs this\n"
    write_source_tree(source_root, tree)
    ledger = work / "unreached_vendor_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER
        + "vendor support/unneeded.h  # deliberately unnecessary admission\n",
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0
        and "UNREACHED ADMISSION: support/unneeded.h" in result.stderr,
        f"a vendor support file must be reached from a candidate; got:\n"
        f"{result.stderr}",
    )


def case_conflicting_disposition_is_rejected(work: Path) -> None:
    source_root = work / "conflict_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = work / "conflict_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER + "discard multibody/tree/frame.h  # contradicts the vendor line\n",
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode == 2,
        f"a file classified twice must make the ledger unusable; got {result.returncode}",
    )
    record_failure_unless(
        "already classified as vendor" in result.stderr,
        f"the conflict must be named; got:\n{result.stderr}",
    )


def case_non_vendor_dependency_is_reported(work: Path) -> None:
    """Something admitted needing a file we chose not to vendor is a real gap."""
    source_root = work / "non_vendor_source"
    tree = dict(ACCEPTED_TREE)
    tree["multibody/tree/frame.h"] = '#include "drake/multibody/tree/rewritten.h"\n'
    tree["multibody/tree/rewritten.h"] = "// ORVD will reimplement this\n"
    write_source_tree(source_root, tree)
    ledger = work / "non_vendor_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER
        + "first_party multibody/tree/rewritten.h  # ORVD reimplements this interface\n",
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode != 0,
        "an admitted file depending on a non-vendored file must be reported",
    )
    record_failure_unless(
        "UNMET DEPENDENCY: multibody/tree/rewritten.h (classified first_party)"
        in result.stderr,
        f"the non-vendored dependency must name its disposition; got:\n{result.stderr}",
    )
    # A decided-but-unvendored file is not an undecided one. Reporting both the
    # same way would hide which of the two a reader has to go fix.
    record_failure_unless(
        "UNCLASSIFIED EDGE: multibody/tree/rewritten.h" not in result.stderr,
        "an unmet dependency must not also be reported as unclassified",
    )


def case_declined_implementation_is_not_an_unmet_dependency(work: Path) -> None:
    """Declining a translation unit is a decision, not a contradiction.

    Many upstream `.cc` files exist only to prove their header is self-contained
    and define no symbol. Reaching one through the stem relation says nothing
    about whether admitted code needs it, so it must not be reported alongside
    the files admitted code actually includes.
    """
    source_root = work / "declined_implementation_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    ledger = work / "declined_implementation_ledger.txt"
    ledger.write_text(
        ACCEPTED_LEDGER.replace(
            "vendor multibody/tree/frame.cc  # frame's own implementation",
            "discard multibody/tree/frame.cc  # defines no symbol; self-containment check only",
        ),
        encoding="utf-8",
    )

    result = run_analyzer(source_root, ledger)
    record_failure_unless(
        result.returncode == 0,
        f"declining an implementation unit must not fail the run; got "
        f"{result.returncode}\n{result.stdout}{result.stderr}",
    )
    record_failure_unless(
        "multibody/tree/frame.cc" not in result.stderr,
        f"a declined implementation unit must not be reported; got:\n{result.stderr}",
    )
    # An include edge to the same declined file is a different matter and must
    # still be reported, so the rule above cannot be used to silence real gaps.
    tree = dict(ACCEPTED_TREE)
    tree["multibody/tree/joint.h"] = (
        '#include "drake/multibody/tree/frame.h"\n'
        '#include "drake/common/autodiff.h"\n'
        '#include "drake/multibody/tree/frame.cc"\n'
    )
    included_root = work / "declined_but_included_source"
    write_source_tree(included_root, tree)
    included_result = run_analyzer(included_root, ledger)
    record_failure_unless(
        included_result.returncode != 0
        and "UNMET DEPENDENCY: multibody/tree/frame.cc" in included_result.stderr,
        f"an include edge to a declined file must still be reported; got:\n"
        f"{included_result.stderr}",
    )


def case_source_commit_mismatch(work: Path) -> None:
    source_root = work / "commit_source"
    write_source_tree(source_root, ACCEPTED_TREE)
    (source_root / ".git").mkdir(parents=True, exist_ok=True)
    (source_root / ".git" / "HEAD").write_text(
        "2\n", encoding="utf-8"
    )
    ledger = work / "commit_ledger.txt"
    ledger.write_text(ACCEPTED_LEDGER, encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(ANALYZER),
            "--drake-source-root",
            str(source_root),
            "--disposition-ledger",
            str(ledger),
            "--require-source-commit",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    record_failure_unless(
        result.returncode == 2,
        f"a clone at the wrong revision must be refused; got {result.returncode}",
    )
    record_failure_unless(
        "source revision mismatch" in result.stderr,
        f"the revision mismatch must be named; got:\n{result.stderr}",
    )


def main() -> int:
    cases = [
        case_accepted_closure,
        case_unclassified_edge,
        case_forbidden_edge_by_prefix,
        case_forbidden_edge_by_file,
        case_explicit_vendor_cannot_override_forbidden_prefix,
        case_include_edge_supersedes_implementation_edge,
        case_forgotten_implementation_unit_is_caught,
        case_forgotten_split_implementation_is_caught,
        case_generated_header_terminates_a_branch,
        case_generated_header_must_be_absent_from_source,
        case_unclassified_candidate_is_rejected,
        case_empty_admission_is_rejected,
        case_unrecognized_include_operand_is_rejected,
        case_reason_is_required,
        case_source_repository_is_required,
        case_license_file_is_required,
        case_unreached_vendor_is_rejected,
        case_conflicting_disposition_is_rejected,
        case_non_vendor_dependency_is_reported,
        case_declined_implementation_is_not_an_unmet_dependency,
        case_source_commit_mismatch,
    ]
    with tempfile.TemporaryDirectory() as temporary_directory:
        work = Path(temporary_directory)
        for case in cases:
            case(work)

    if failure_count > 0:
        print(f"{failure_count} analyzer check(s) failed", file=sys.stderr)
        return 1
    print(f"source closure analyzer verified across {len(cases)} cases")
    return 0


if __name__ == "__main__":
    sys.exit(main())
