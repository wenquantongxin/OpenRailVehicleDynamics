#!/usr/bin/env python3
"""Checks that the product boundary gate fails on a Drake dependency and only on one.

Each case configures a synthetic CMake project that includes the repository's own
gate module — not a copy of it — so what is exercised is the gate the build
actually uses. The Drake targets are fabricated: the gate reads the target graph,
so whether a real Drake is installed on this machine is beside the point, and
depending on one would make the check unrunnable wherever it is not.

The cases cover direct and wrapped target edges, imported artefacts, link-driver
options, nested product directories, and directory-scoped imported targets. Two
controls give the rest their meaning: a target outside the product directories
may link Drake, while a clean product must configure.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True

GATE_MODULE_DIRECTORY = Path(__file__).resolve().parents[2] / "cmake"

failure_count = 0
cmake_executable = "cmake"


def record_failure_unless(condition: bool, failure_description: str) -> None:
    global failure_count
    if not condition:
        print(f"FAILED: {failure_description}", file=sys.stderr)
        failure_count += 1


PROJECT_PREAMBLE = """\
cmake_minimum_required(VERSION 3.24)
project(product_boundary_gate_case LANGUAGES CXX)
list(APPEND CMAKE_MODULE_PATH "@GATE_MODULE_DIRECTORY@")
include(OrvdProductBoundaryGate)

# A fabricated Drake, so the gate is exercised on the target graph rather than on
# whatever happens to be installed here.
add_library(fabricated_drake UNKNOWN IMPORTED)
set_target_properties(fabricated_drake PROPERTIES
    IMPORTED_LOCATION "@WORK@/libdrake.so")
add_library(drake::drake ALIAS fabricated_drake)
"""


def write_case(
    work: Path,
    name: str,
    product_body: str,
    extra_body: str = "",
    nested_product_body: str | None = None,
) -> Path:
    case_root = work / name
    product_directory = case_root / "product_module"
    product_directory.mkdir(parents=True, exist_ok=True)
    (product_directory / "product.cc").write_text(
        "int Product() { return 1; }\n", encoding="utf-8"
    )
    (product_directory / "CMakeLists.txt").write_text(product_body, encoding="utf-8")
    if nested_product_body is not None:
        nested_directory = product_directory / "nested"
        nested_directory.mkdir()
        (nested_directory / "nested.cc").write_text(
            "int NestedProduct() { return 1; }\n", encoding="utf-8"
        )
        (nested_directory / "CMakeLists.txt").write_text(
            nested_product_body, encoding="utf-8"
        )

    other_directory = case_root / "not_the_product"
    other_directory.mkdir(parents=True, exist_ok=True)
    (other_directory / "other.cc").write_text(
        "int Other() { return 1; }\n", encoding="utf-8"
    )
    (other_directory / "CMakeLists.txt").write_text(
        extra_body or "# nothing here\n", encoding="utf-8"
    )

    preamble = PROJECT_PREAMBLE.replace(
        "@GATE_MODULE_DIRECTORY@", GATE_MODULE_DIRECTORY.as_posix()
    ).replace("@WORK@", case_root.as_posix())
    (case_root / "CMakeLists.txt").write_text(
        preamble
        + "\nadd_subdirectory(product_module)\n"
        + "add_subdirectory(not_the_product)\n"
        + "orvd_verify_product_targets_have_no_drake_dependency(product_module)\n",
        encoding="utf-8",
    )
    return case_root


def configure(case_root: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [cmake_executable, "-S", str(case_root), "-B", str(case_root / "build")],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )


def case_clean_product_configures(work: Path) -> None:
    case_root = write_case(
        work, "clean", "add_library(product_library STATIC product.cc)\n"
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode == 0,
        "a product target with no Drake dependency must configure\n" + result.stdout,
    )
    record_failure_unless(
        "no Drake dependency in product_library" in result.stdout,
        "the gate must say which targets it checked\n" + result.stdout,
    )


def case_drake_through_interface_wrapper_fails(work: Path) -> None:
    # The dependency nobody notices: the product links something reasonable, and
    # that something carries Drake in its interface.
    case_root = write_case(
        work,
        "interface_wrapper",
        "add_library(innocent_wrapper INTERFACE)\n"
        "target_link_libraries(innocent_wrapper INTERFACE drake::drake)\n"
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE innocent_wrapper)\n",
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode != 0 and "boundary violated" in result.stdout,
        "Drake reached through an interface wrapper must fail\n" + result.stdout,
    )
    record_failure_unless(
        "product_library" in result.stdout,
        "the report must name the product target that carries the dependency\n"
        + result.stdout,
    )

    # A one-element property is still a list even when CMake reads its contents
    # as false. If the gate asks `if(edge_values)` instead of counting the
    # elements, this Drake-named edge is mistaken for an empty property.
    false_like_edge_root = write_case(
        work,
        "false_like_interface_edge",
        "add_library(innocent_wrapper INTERFACE IMPORTED GLOBAL)\n"
        "set_target_properties(innocent_wrapper PROPERTIES\n"
        "    INTERFACE_LINK_LIBRARIES drake-NOTFOUND)\n"
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE innocent_wrapper)\n",
    )
    false_like_edge_result = configure(false_like_edge_root)
    record_failure_unless(
        false_like_edge_result.returncode != 0
        and "is a Drake library" in false_like_edge_result.stdout,
        "a false-like one-element link edge is not an empty edge list\n"
        + false_like_edge_result.stdout,
    )


def case_imported_drake_artefact_fails(work: Path) -> None:
    # No `drake::` in sight; the target is called something else entirely and only
    # its artefact gives it away.
    case_root = write_case(
        work,
        "imported_artefact",
        # GLOBAL, because that is what the product build produces: the top-level
        # CMakeLists sets CMAKE_FIND_PACKAGE_TARGETS_GLOBAL so that imported
        # targets are visible where the gate runs.
        "add_library(third_party_thing SHARED IMPORTED GLOBAL)\n"
        "set_target_properties(third_party_thing PROPERTIES\n"
        '    IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/libdrake_marker.so")\n'
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE third_party_thing)\n",
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode != 0 and "resolves to a Drake library" in result.stdout,
        "an imported target whose artefact is a Drake library must fail\n"
        + result.stdout,
    )

    # `get_target_property` uses `<variable>-NOTFOUND` for an unset property, but
    # a real property value can also end in `-NOTFOUND`. Property presence and
    # list length, not CMake truth, decide whether there is text to inspect.
    false_like_artefact_root = write_case(
        work,
        "false_like_imported_artefact",
        "add_library(third_party_thing SHARED IMPORTED GLOBAL)\n"
        "set_target_properties(third_party_thing PROPERTIES\n"
        '    IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/libdrake-NOTFOUND")\n'
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE third_party_thing)\n",
    )
    false_like_artefact_result = configure(false_like_artefact_root)
    record_failure_unless(
        false_like_artefact_result.returncode != 0
        and "resolves to a Drake library" in false_like_artefact_result.stdout,
        "a false-like imported artefact value is not an unset artefact\n"
        + false_like_artefact_result.stdout,
    )


def case_drake_link_option_fails(work: Path) -> None:
    case_root = write_case(
        work,
        "link_option",
        "add_library(product_library STATIC product.cc)\n"
        "target_link_options(product_library PRIVATE -ldrake)\n",
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode != 0 and "selects a Drake library" in result.stdout,
        "a link option naming Drake must fail\n" + result.stdout,
    )


def case_bare_drake_library_path_fails(work: Path) -> None:
    case_root = write_case(
        work,
        "bare_path",
        "add_library(product_library STATIC product.cc)\n"
        'target_link_libraries(product_library PRIVATE "/somewhere/else/libdrake.so")\n',
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode != 0 and "is a Drake library" in result.stdout,
        "a bare path to a Drake library must fail\n" + result.stdout,
    )


def case_non_product_target_may_link_drake(work: Path) -> None:
    # The control that gives the rest their meaning. The Drake-backed reference
    # comparison lives outside the product directories and is supposed to link
    # Drake; a gate that forbade that would have to be switched off to run it.
    case_root = write_case(
        work,
        "reference_target",
        "add_library(product_library STATIC product.cc)\n",
        extra_body="add_library(reference_library STATIC other.cc)\n"
        "target_link_libraries(reference_library PRIVATE drake::drake)\n",
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode == 0,
        "a target outside the product directories must be free to link Drake\n"
        + result.stdout,
    )


def case_directory_membership_needs_no_registration(work: Path) -> None:
    # A second target appears in the product directory and nobody registers it
    # anywhere. Membership is by directory, so it is checked regardless.
    case_root = write_case(
        work,
        "unregistered_target",
        "add_library(product_library STATIC product.cc)\n"
        "add_library(product_library_added_later STATIC product.cc)\n"
        "target_link_libraries(product_library_added_later PRIVATE drake::drake)\n",
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode != 0 and "product_library_added_later" in result.stdout,
        "a target nobody registered must still be checked, because membership is"
        " by directory\n" + result.stdout,
    )

    # Target names are list contents too. `OFF` is legal here and false to
    # `if(directory_targets)`, so a truth test would report that the directory had
    # no targets at all.
    false_like_target_root = write_case(
        work,
        "false_like_product_target",
        "add_library(OFF STATIC product.cc)\n"
        "target_link_libraries(OFF PRIVATE drake::drake)\n",
    )
    false_like_target_result = configure(false_like_target_root)
    record_failure_unless(
        false_like_target_result.returncode != 0
        and "boundary violated" in false_like_target_result.stdout
        and "OFF: it depends on the Drake target 'drake::drake'"
        in false_like_target_result.stdout,
        "a product target whose name looks false must still be collected\n"
        + false_like_target_result.stdout,
    )


def case_nested_product_directory_is_checked(work: Path) -> None:
    case_root = write_case(
        work,
        "nested_product",
        "add_subdirectory(nested)\n",
        nested_product_body=(
            "add_library(nested_product_library STATIC nested.cc)\n"
            "target_link_libraries(nested_product_library PRIVATE drake::drake)\n"
        ),
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode != 0 and "nested_product_library" in result.stdout,
        "a product target in a nested directory must be checked\n" + result.stdout,
    )


def case_directory_scoped_imported_target_is_refused(work: Path) -> None:
    case_root = write_case(
        work,
        "directory_scoped_import",
        "add_library(innocent_name SHARED IMPORTED)\n"
        "set_target_properties(innocent_name PROPERTIES\n"
        '    IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/libdrake_hidden.so")\n'
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE innocent_name)\n",
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode != 0
        and "directory-scoped imported target 'innocent_name'" in result.stdout,
        "an imported target the top-level gate cannot inspect must fail loudly\n"
        + result.stdout,
    )


def case_imported_configuration_and_libname_are_checked(work: Path) -> None:
    configured_artefact_root = write_case(
        work,
        "configured_imported_artefact",
        "add_library(configured_dependency SHARED IMPORTED GLOBAL)\n"
        "set_target_properties(configured_dependency PROPERTIES\n"
        '    IMPORTED_CONFIGURATIONS DEBUG\n'
        '    IMPORTED_LOCATION_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/libdrake_debug.so"\n'
        "    MAP_IMPORTED_CONFIG_RELEASE DEBUG)\n"
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE configured_dependency)\n",
    )
    configured_result = configure(configured_artefact_root)
    record_failure_unless(
        configured_result.returncode != 0
        and "resolves to a Drake library" in configured_result.stdout,
        "every declared imported configuration must be inspected\n"
        + configured_result.stdout,
    )

    false_like_configuration_root = write_case(
        work,
        "false_like_imported_configuration",
        "add_library(configured_dependency SHARED IMPORTED GLOBAL)\n"
        "set_target_properties(configured_dependency PROPERTIES\n"
        "    IMPORTED_CONFIGURATIONS OFF\n"
        '    IMPORTED_LOCATION_OFF "${CMAKE_CURRENT_BINARY_DIR}/libdrake.so")\n'
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE configured_dependency)\n",
    )
    false_like_configuration_result = configure(false_like_configuration_root)
    record_failure_unless(
        false_like_configuration_result.returncode != 0
        and "resolves to a Drake library" in false_like_configuration_result.stdout,
        "an imported configuration named OFF is one configuration, not none\n"
        + false_like_configuration_result.stdout,
    )

    imported_libname_root = write_case(
        work,
        "imported_libname",
        "add_library(named_dependency INTERFACE IMPORTED GLOBAL)\n"
        "set_target_properties(named_dependency PROPERTIES IMPORTED_LIBNAME drake)\n"
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE named_dependency)\n",
    )
    libname_result = configure(imported_libname_root)
    record_failure_unless(
        libname_result.returncode != 0
        and "resolves to a Drake library" in libname_result.stdout,
        "IMPORTED_LIBNAME must be inspected as a library identity\n"
        + libname_result.stdout,
    )


def case_wrapped_and_configuration_link_options_are_checked(work: Path) -> None:
    option_declarations = {
        "compiler_driver_wrapper": (
            "target_link_options(product_library PRIVATE -Wl,-ldrake)\n"
        ),
        "cmake_linker_wrapper": (
            "target_link_options(product_library PRIVATE LINKER:-l,drake)\n"
        ),
        "gnu_long_library_selector": (
            "target_link_options(product_library PRIVATE --library=drake)\n"
        ),
        "windows_default_library": (
            "target_link_options(product_library PRIVATE /DEFAULTLIB:Drake.lib)\n"
        ),
        "compiler_driver_whole_archive": (
            'target_link_options(product_library PRIVATE '
            '"-Wl,--whole-archive,/somewhere/libdrake.a,--no-whole-archive")\n'
        ),
        "windows_whole_archive": (
            "target_link_options(product_library PRIVATE "
            "/WHOLEARCHIVE:drake.lib)\n"
        ),
        "compound_link_flags": (
            'set_property(TARGET product_library PROPERTY LINK_FLAGS "-pthread -ldrake")\n'
        ),
        "configuration_link_flags": (
            "set(CMAKE_BUILD_TYPE Release PARENT_SCOPE)\n"
            'set_property(TARGET product_library PROPERTY LINK_FLAGS_RELEASE "-ldrake")\n'
        ),
        "false_like_configuration_link_flags": (
            "set(CMAKE_BUILD_TYPE OFF PARENT_SCOPE)\n"
            'set_property(TARGET product_library PROPERTY LINK_FLAGS_OFF "-ldrake")\n'
        ),
        "false_like_library_option": (
            "target_link_options(product_library PRIVATE -ldrake-NOTFOUND)\n"
        ),
    }
    for case_name, option_declaration in option_declarations.items():
        case_root = write_case(
            work,
            case_name,
            "add_library(product_library STATIC product.cc)\n"
            + option_declaration,
        )
        result = configure(case_root)
        record_failure_unless(
            result.returncode != 0
            and (
                "selects a Drake library" in result.stdout
                or "names a Drake library file" in result.stdout
            ),
            f"{case_name} must not hide a Drake link option\n" + result.stdout,
        )


def case_directory_link_options_do_not_name_a_library(work: Path) -> None:
    case_root = write_case(
        work,
        "directory_link_options",
        "add_library(product_library STATIC product.cc)\n"
        'target_link_options(product_library PRIVATE "-L/opt/drake")\n'
        'target_link_options(product_library PRIVATE "-Wl,-rpath,/opt/drake")\n'
        'target_link_options(product_library PRIVATE "/LIBPATH:C:/third_party/drake")\n',
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode == 0,
        "library search and runtime paths are machine locations, not Drake "
        "library identities\n" + result.stdout,
    )


def case_unrelated_target_namespace_configures(work: Path) -> None:
    case_root = write_case(
        work,
        "unrelated_target_namespace",
        "add_library(mandrake_core INTERFACE IMPORTED GLOBAL)\n"
        "add_library(mandrake::core ALIAS mandrake_core)\n"
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE mandrake::core)\n",
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode == 0,
        "an unrelated target whose namespace merely contains 'drake' must "
        "configure\n" + result.stdout,
    )


def case_imported_generator_expressions_cannot_hide_drake(work: Path) -> None:
    link_edge_root = write_case(
        work,
        "imported_conditional_link_edge",
        "add_library(imported_wrapper INTERFACE IMPORTED GLOBAL)\n"
        "set_target_properties(imported_wrapper PROPERTIES\n"
        '    INTERFACE_LINK_LIBRARIES "$<$<CONFIG:Release>:drake::drake>")\n'
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE imported_wrapper)\n",
    )
    edge_result = configure(link_edge_root)
    record_failure_unless(
        edge_result.returncode != 0 and "Drake target" in edge_result.stdout,
        "an imported generator expression must not hide a Drake target\n"
        + edge_result.stdout,
    )

    link_option_root = write_case(
        work,
        "imported_conditional_link_option",
        "add_library(imported_wrapper INTERFACE IMPORTED GLOBAL)\n"
        "set_target_properties(imported_wrapper PROPERTIES\n"
        '    INTERFACE_LINK_OPTIONS "$<$<CONFIG:Release>:-ldrake>")\n'
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE imported_wrapper)\n",
    )
    option_result = configure(link_option_root)
    record_failure_unless(
        option_result.returncode != 0
        and "selects a Drake library" in option_result.stdout,
        "an imported generator expression must not hide a Drake link option\n"
        + option_result.stdout,
    )

    dependent_library_root = write_case(
        work,
        "imported_dependent_library",
        "add_library(imported_wrapper SHARED IMPORTED GLOBAL)\n"
        "set_target_properties(imported_wrapper PROPERTIES\n"
        '    IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/libinnocent.so"\n'
        "    IMPORTED_LINK_DEPENDENT_LIBRARIES drake::drake)\n"
        "add_library(product_library STATIC product.cc)\n"
        "target_link_libraries(product_library PRIVATE imported_wrapper)\n",
    )
    dependent_result = configure(dependent_library_root)
    record_failure_unless(
        dependent_result.returncode != 0 and "Drake target" in dependent_result.stdout,
        "an imported target must not hide a dependent Drake library\n"
        + dependent_result.stdout,
    )


def case_split_library_selection_options_are_caught(work: Path) -> None:
    """A selector and the library it selects, arriving apart.

    `-ldrake` is one token and easy. `-l` followed by `drake` — as two list
    elements, as one element with a space, or wrapped in a driver prefix — is the
    same instruction written differently, and scanning tokens one at a time sees
    only a selector and only a neutral name.
    """
    for label, option in (
        ("split_elements", '"-l" "drake"'),
        ("spaced_element", '"-l drake"'),
        ("long_form", '"--library" "drake"'),
        ("linker_driver", '"LINKER:SHELL:-l drake"'),
        ("framework", '"-framework" "Drake"'),
    ):
        case_root = write_case(
            work,
            f"split_option_{label}",
            "add_library(product_library STATIC product.cc)\n"
            f"target_link_options(product_library PUBLIC {option})\n",
        )
        result = configure(case_root)
        record_failure_unless(
            result.returncode != 0 and "which is Drake" in result.stdout,
            f"a split library selection ({label}) must fail\n" + result.stdout,
        )


def case_imported_artefact_behind_a_generator_expression_is_refused(
    work: Path,
) -> None:
    """An imported target whose location is decided later.

    The one place an imported target says which file it resolves to is exactly
    where an unevaluable expression must not be tolerated: it can resolve to
    Drake in the configuration nobody checked.
    """
    case_root = write_case(
        work,
        "imported_artefact_genex",
        "add_library(product_library STATIC product.cc)\n"
        "add_library(sneaky SHARED IMPORTED GLOBAL)\n"
        "set_target_properties(sneaky PROPERTIES IMPORTED_LOCATION"
        ' "$<$<CONFIG:Release>:/opt/drake/lib/libdrake.so>")\n'
        "target_link_libraries(product_library PUBLIC sneaky)\n",
    )
    result = configure(case_root)
    record_failure_unless(
        result.returncode != 0 and "cannot evaluate" in result.stdout,
        "an imported artefact hidden behind a generator expression must fail\n"
        + result.stdout,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Self-check for the product boundary gate."
    )
    parser.add_argument("--cmake", default="cmake", help="the cmake executable to use")
    arguments = parser.parse_args()
    global cmake_executable
    cmake_executable = arguments.cmake
    if shutil.which(cmake_executable) is None:
        print(f"cmake not found: {cmake_executable}", file=sys.stderr)
        return 2

    cases = (
        case_clean_product_configures,
        case_drake_through_interface_wrapper_fails,
        case_imported_drake_artefact_fails,
        case_drake_link_option_fails,
        case_bare_drake_library_path_fails,
        case_non_product_target_may_link_drake,
        case_directory_membership_needs_no_registration,
        case_nested_product_directory_is_checked,
        case_directory_scoped_imported_target_is_refused,
        case_imported_configuration_and_libname_are_checked,
        case_wrapped_and_configuration_link_options_are_checked,
        case_directory_link_options_do_not_name_a_library,
        case_unrelated_target_namespace_configures,
        case_imported_generator_expressions_cannot_hide_drake,
        case_split_library_selection_options_are_caught,
        case_imported_artefact_behind_a_generator_expression_is_refused,
    )
    with tempfile.TemporaryDirectory(prefix="orvd_verify_boundary_gate.") as work:
        for case in cases:
            case(Path(work))
    if failure_count > 0:
        print(f"{failure_count} product boundary gate check(s) failed", file=sys.stderr)
        return 1
    print(f"product boundary gate verified across {len(cases)} cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
