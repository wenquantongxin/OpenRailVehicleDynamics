#!/usr/bin/env python3
"""Checks that the landed compile frontier tool reports what it must and hides nothing.

Every case builds a synthetic landed tree whose right answer is known by
construction, so the tool is exercised against something other than whatever the
real landed tree happens to contain today.

Each control targets a distinct way the tool could report a frontier that was not
earned: a foreign Drake tree satisfying an include, one blocked unit masking the
rest, a compiler that reports success without producing an object, a forbidden
scalar symbol going unnoticed, and — the reverse mistake — Eigen's own `symbolic`
namespace being mistaken for Drake's symbolic scalar.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True

LANDED_FRONTIER_TOOL = (
    Path(__file__).resolve().parent
    / "compile_landed_double_multibody_translation_units.py"
)

failure_count = 0
compiler_under_test = "c++"


def record_failure_unless(condition: bool, failure_description: str) -> None:
    global failure_count
    if not condition:
        print(f"FAILED: {failure_description}", file=sys.stderr)
        failure_count += 1


def write_tree(root: Path, files: dict[str, str]) -> None:
    for relative_path, content in files.items():
        path = root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")


def run_tool(
    landed_root: Path, admitted_include_directories: list[Path] | None = None
) -> subprocess.CompletedProcess[str]:
    command = [
        sys.executable,
        str(LANDED_FRONTIER_TOOL),
        "--landed-root",
        str(landed_root),
        "--compiler",
        compiler_under_test,
    ]
    for include_directory in admitted_include_directories or []:
        command += ["--admitted-include-directory", str(include_directory)]
    return subprocess.run(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False
    )


def case_self_contained_tree_compiles(workspace: Path) -> None:
    landed_root = workspace / "self_contained"
    write_tree(
        landed_root,
        {
            "drake/common/kept.h": "#pragma once\nint Kept();\n",
            "drake/common/kept.cc": '#include "drake/common/kept.h"\nint Kept() { return 1; }\n',
        },
    )
    result = run_tool(landed_root)
    record_failure_unless(
        result.returncode == 0,
        "a self-contained landed tree must exit zero\n" + result.stdout,
    )
    record_failure_unless(
        "every landed translation unit produced an object" in result.stdout,
        "a self-contained tree must be reported as fully compiled\n" + result.stdout,
    )


def case_foreign_drake_include_directory_is_refused(workspace: Path) -> None:
    landed_root = workspace / "with_foreign_include"
    write_tree(
        landed_root,
        {
            "drake/common/uses_runtime.cc": (
                '#include "drake/multibody/tree/multibody_tree_system.h"\n'
                "int UsesRuntime() { return 1; }\n"
            ),
        },
    )
    # The foreign directory provides exactly the header the landed source misses,
    # not one of the fixed category canaries. The preflight must derive this name
    # from the landed source or the compile would report an unearned success.
    foreign_root = workspace / "foreign_drake_prefix"
    write_tree(
        foreign_root,
        {
            "drake/multibody/tree/multibody_tree_system.h": "#pragma once\n",
        },
    )
    result = run_tool(landed_root, [foreign_root])
    record_failure_unless(
        result.returncode == 2,
        "an admitted include directory exposing a Drake tree must be refused\n"
        + result.stdout,
    )
    record_failure_unless(
        "reaches a Drake tree" in result.stdout,
        "the refusal must say that a foreign Drake tree is reachable\n" + result.stdout,
    )


def case_preflight_failure_is_not_treated_as_clean(workspace: Path) -> None:
    landed_root = workspace / "failed_preflight"
    write_tree(
        landed_root,
        {"drake/common/kept.cc": "int Kept() { return 1; }\n"},
    )
    failing_preprocessor = workspace / "compiler_with_failed_preflight.py"
    failing_preprocessor.write_text(
        "#!/usr/bin/env python3\n"
        "import sys\n"
        "print('preprocessor intentionally failed')\n"
        "sys.exit(7)\n",
        encoding="utf-8",
    )
    failing_preprocessor.chmod(0o755)
    result = subprocess.run(
        [
            sys.executable,
            str(LANDED_FRONTIER_TOOL),
            "--landed-root",
            str(landed_root),
            "--compiler",
            str(failing_preprocessor),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    record_failure_unless(
        result.returncode == 2,
        "a failed isolation preflight must be an infrastructure error\n"
        + result.stdout,
    )
    record_failure_unless(
        "could not perform the foreign-Drake preflight" in result.stdout,
        "the preflight failure must be reported by name\n" + result.stdout,
    )


def case_one_blocked_unit_does_not_mask_the_others(workspace: Path) -> None:
    landed_root = workspace / "one_blocked"
    write_tree(
        landed_root,
        {
            "drake/common/first.cc": "int First() { return 1; }\n",
            "drake/common/blocked.cc": '#include "drake/absent/runtime.h"\nint Blocked();\n',
            "drake/common/last.cc": "int Last() { return 2; }\n",
        },
    )
    result = run_tool(landed_root)
    record_failure_unless(
        result.returncode == 1,
        "a tree with a blocked unit must exit one\n" + result.stdout,
    )
    for expected_line in (
        "compiled  drake/common/first.cc",
        "BLOCKED   drake/common/blocked.cc",
        "compiled  drake/common/last.cc",
    ):
        record_failure_unless(
            expected_line in result.stdout,
            f"the report must contain '{expected_line}' so one failure cannot hide"
            " the rest\n" + result.stdout,
        )
    record_failure_unless(
        "drake/absent/runtime.h" in result.stdout,
        "the blocked unit's own diagnostics must be shown verbatim\n" + result.stdout,
    )
    record_failure_unless(
        "drake/absent/runtime.h  <- drake/common/blocked.cc:1" in result.stdout,
        "the missing header must be listed with the site that includes it\n"
        + result.stdout,
    )
    record_failure_unless(
        "does not classify compiler failures" in result.stdout,
        "the tool must not label every compiler failure as a runtime dependency\n"
        + result.stdout,
    )


def case_compiler_success_without_object_fails(workspace: Path) -> None:
    landed_root = workspace / "empty_object"
    write_tree(
        landed_root,
        {"drake/common/kept.cc": "int Kept() { return 1; }\n"},
    )
    # A compiler shim that reports success and writes nothing where the object was
    # asked for. Compiling is not the claim being made; producing an object is.
    shim = workspace / "compiler_shim_without_object.py"
    shim.write_text(
        "#!/usr/bin/env python3\n"
        "import sys\n"
        "arguments = sys.argv[1:]\n"
        "if '-o' in arguments:\n"
        "    open(arguments[arguments.index('-o') + 1], 'w').close()\n"
        "sys.exit(0)\n",
        encoding="utf-8",
    )
    shim.chmod(0o755)
    result = subprocess.run(
        [
            sys.executable,
            str(LANDED_FRONTIER_TOOL),
            "--landed-root",
            str(landed_root),
            "--compiler",
            str(shim),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    record_failure_unless(
        result.returncode == 1,
        "a compiler that reports success without an object must not pass\n"
        + result.stdout,
    )
    record_failure_unless(
        "produced no object content" in result.stdout,
        "the report must say the object had no content\n" + result.stdout,
    )


def case_forbidden_scalar_symbol_is_detected(workspace: Path) -> None:
    landed_root = workspace / "forbidden_symbol"
    write_tree(
        landed_root,
        {
            # A type named AutoDiffXd in the drake namespace, defined and used, so
            # the name reaches the symbol table.
            "drake/common/scalar.h": (
                "#pragma once\n"
                "namespace drake {\n"
                "class AutoDiffXd {\n"
                " public:\n"
                "  double Value() const;\n"
                "};\n"
                "}  // namespace drake\n"
            ),
            "drake/common/scalar.cc": (
                '#include "drake/common/scalar.h"\n'
                "namespace drake {\n"
                "double AutoDiffXd::Value() const { return 0.0; }\n"
                "}  // namespace drake\n"
            ),
        },
    )
    result = run_tool(landed_root)
    record_failure_unless(
        result.returncode == 1,
        "a forbidden scalar symbol must not pass\n" + result.stdout,
    )
    record_failure_unless(
        "FORBIDDEN SYMBOL" in result.stdout and "AutoDiffXd" in result.stdout,
        "the report must name the forbidden symbol it matched\n" + result.stdout,
    )
    record_failure_unless(
        "FORBIDDEN SYMBOL drake/common/scalar.cc" in result.stdout,
        "a forbidden symbol report must name its source translation unit\n"
        + result.stdout,
    )


def case_eigen_symbolic_namespace_is_not_a_violation(workspace: Path) -> None:
    landed_root = workspace / "eigen_symbolic"
    write_tree(
        landed_root,
        {
            # Eigen's own `symbolic` namespace, which its index expressions use.
            # A bare search for `symbolic` would call this a violation; a search
            # for `drake::symbolic::` must not.
            "drake/common/index_expression.h": (
                "#pragma once\n"
                "namespace Eigen {\n"
                "namespace symbolic {\n"
                "class SymbolExpr {\n"
                " public:\n"
                "  int Value() const;\n"
                "};\n"
                "}  // namespace symbolic\n"
                "}  // namespace Eigen\n"
            ),
            "drake/common/index_expression.cc": (
                '#include "drake/common/index_expression.h"\n'
                "namespace Eigen {\n"
                "namespace symbolic {\n"
                "int SymbolExpr::Value() const { return 0; }\n"
                "}  // namespace symbolic\n"
                "}  // namespace Eigen\n"
            ),
        },
    )
    result = run_tool(landed_root)
    record_failure_unless(
        result.returncode == 0,
        "Eigen's own symbolic namespace must not be reported as a violation\n"
        + result.stdout,
    )
    record_failure_unless(
        "FORBIDDEN SYMBOL" not in result.stdout,
        "Eigen::symbolic must not match the forbidden-symbol check\n" + result.stdout,
    )


def case_compile_time_scalar_machinery_is_detected(workspace: Path) -> None:
    landed_root = workspace / "source_residue"
    write_tree(
        landed_root,
        {
            # A construct that resolves before it could reach a symbol table, so
            # only a source scan can see it.
            "drake/common/kept.h": (
                "#pragma once\n"
                "namespace drake {\n"
                "template <typename T>\n"
                "struct scalar_predicate {\n"
                "  static constexpr bool is_bool = true;\n"
                "};\n"
                "}  // namespace drake\n"
            ),
            "drake/common/kept.cc": '#include "drake/common/kept.h"\nint Kept() { return 1; }\n',
        },
    )
    result = run_tool(landed_root)
    record_failure_unless(
        result.returncode == 1,
        "compile-time scalar machinery must not pass\n" + result.stdout,
    )
    record_failure_unless(
        "FORBIDDEN SOURCE TOKEN" in result.stdout
        and "scalar_predicate" in result.stdout,
        "the report must name the source token it matched\n" + result.stdout,
    )


def case_a_name_in_a_comment_is_not_a_dependency(workspace: Path) -> None:
    landed_root = workspace / "comment_only"
    write_tree(
        landed_root,
        {
            # A doxygen block whose continuation lines carry no marker, mentioning
            # a systems:: name and an include of a header that does not exist.
            "drake/common/documented.h": (
                "#pragma once\n"
                "/** Explains something.\n"
                " Consider drake::systems::Imaginary<T> and how one might write\n"
                ' #include "drake/systems/framework/imaginary.h"\n'
                " in an example. */\n"
                "int Documented();\n"
            ),
            "drake/common/documented.cc": (
                '#include "drake/common/documented.h"\nint Documented() { return 1; }\n'
            ),
        },
    )
    result = run_tool(landed_root)
    record_failure_unless(
        result.returncode == 0,
        "names mentioned only inside comments must not block the run\n"
        + result.stdout,
    )
    record_failure_unless(
        "systems::Imaginary" not in result.stdout,
        "a systems:: name inside a comment must not be reported as a dependency\n"
        + result.stdout,
    )
    record_failure_unless(
        "imaginary.h" not in result.stdout,
        "an include inside a comment must not be reported as a missing header\n"
        + result.stdout,
    )


def case_a_name_in_a_string_is_not_source_machinery(workspace: Path) -> None:
    landed_root = workspace / "string_only"
    write_tree(
        landed_root,
        {
            "drake/common/described.cc": (
                "const char* Describe() {\n"
                '  return "systems::Imaginary scalar_predicate";\n'
                "}\n"
            ),
        },
    )
    result = run_tool(landed_root)
    record_failure_unless(
        result.returncode == 0,
        "names inside a string literal must not become source dependencies\n"
        + result.stdout,
    )
    record_failure_unless(
        "systems::Imaginary" not in result.stdout,
        "a systems:: name inside a string must not be reported\n" + result.stdout,
    )
    record_failure_unless(
        "FORBIDDEN SOURCE TOKEN" not in result.stdout,
        "a forbidden token inside a string must not fail the source scan\n"
        + result.stdout,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Self-check for the landed compile frontier tool."
    )
    parser.add_argument(
        "--compiler", default="c++", help="GNU-compatible compiler executable"
    )
    arguments = parser.parse_args()
    global compiler_under_test
    compiler_under_test = arguments.compiler

    cases = (
        case_self_contained_tree_compiles,
        case_foreign_drake_include_directory_is_refused,
        case_preflight_failure_is_not_treated_as_clean,
        case_one_blocked_unit_does_not_mask_the_others,
        case_compiler_success_without_object_fails,
        case_forbidden_scalar_symbol_is_detected,
        case_eigen_symbolic_namespace_is_not_a_violation,
        case_compile_time_scalar_machinery_is_detected,
        case_a_name_in_a_comment_is_not_a_dependency,
        case_a_name_in_a_string_is_not_source_machinery,
    )
    with tempfile.TemporaryDirectory(
        prefix="orvd_verify_landed_frontier."
    ) as workspace_directory:
        workspace = Path(workspace_directory)
        for case in cases:
            case(workspace)

    if failure_count > 0:
        print(f"{failure_count} landed frontier check(s) failed", file=sys.stderr)
        return 1
    print(f"{len(cases)} landed frontier checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
