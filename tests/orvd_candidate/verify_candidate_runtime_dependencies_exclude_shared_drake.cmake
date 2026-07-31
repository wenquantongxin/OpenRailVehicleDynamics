# Checks a built candidate's declared dynamic load-time closure for shared Drake.
#
# On the built artefact, not only on the CMake graph. A graph check has to model
# every way CMake can be told about a dependency — a library selection option in
# any of its spellings, a generator expression evaluated per configuration, an
# imported target declared in a scope where it is invisible, a link edge written
# from a different directory — and each is a separate thing to get right.
# Deferring a graph check can move it later, but still does not turn the
# configure-time representation into the built program's resolved load-time
# closure.
#
# CMake resolves the linked file's declared dynamic dependencies recursively
# using the platform rules available in this test environment. Whatever CMake
# spelling produced them, a shared `libdrake` either appears in that resolved
# closure or it does not. This is not a claim about static archives, `dlopen`,
# `LoadLibrary`, `LD_PRELOAD`, a changed loader search environment, or a library
# deliberately renamed to hide its identity. The product graph gate rejects
# ordinary static and shared Drake inputs early; provenance and object ownership
# say what the product embeds statically. This check has the narrower job of
# keeping a shared Drake out of this candidate's declared runtime closure.
#
# What this permits, and must: the candidate links the landed tree statically.
# That is vendored Drake source compiled into ORVD's own archives, and it is what
# the product is. What it must not do is load the installed shared library as
# well — that library and the vendored copy export the same `drake::` symbols, so
# both in one address space is an ODR violation whose likely symptom is a
# comparison that appears to pass.
cmake_minimum_required(VERSION 3.24)

if(NOT DEFINED CANDIDATE_EXECUTABLE)
    message(FATAL_ERROR "CANDIDATE_EXECUTABLE was not given")
endif()
if(NOT EXISTS "${CANDIDATE_EXECUTABLE}")
    message(FATAL_ERROR
        "the candidate '${CANDIDATE_EXECUTABLE}' does not exist, so this check "
        "would pass by having nothing to look at")
endif()

file(GET_RUNTIME_DEPENDENCIES
     EXECUTABLES "${CANDIDATE_EXECUTABLE}"
     RESOLVED_DEPENDENCIES_VAR resolved_dependencies
     UNRESOLVED_DEPENDENCIES_VAR unresolved_dependencies)

# A dependency that could not be resolved is not a dependency that was cleared.
# Resolution is how the closure is walked: an unresolved node is one whose own
# dependencies were never read, so a Drake sitting behind a neutrally named
# wrapper would go unmentioned. A runtime environment may still make that wrapper
# loadable without changing this executable. There is no answer to give about
# such a file, so none is given.
# Counted, not tested for truth. `if(<list>)` asks whether the list's *contents*
# look true, and CMake reads `OFF`, `0`, `NOTFOUND` and anything ending in
# `-NOTFOUND` as false — so a dependency whose name happens to be one of those
# would make this branch decide there were none. What is being asked is how many
# there are.
list(LENGTH unresolved_dependencies unresolved_count)
if(unresolved_count GREATER 0)
    list(JOIN unresolved_dependencies "\n  " unresolved_report)
    message(FATAL_ERROR
        "the candidate's runtime dependencies could not all be resolved, so "
        "what lies behind them was never read:\n  ${unresolved_report}\n"
        "This check reports nothing rather than reporting a closure it did not "
        "finish walking.")
endif()

set(drake_dependencies "")
foreach(dependency IN LISTS resolved_dependencies)
    get_filename_component(dependency_name "${dependency}" NAME)
    string(TOLOWER "${dependency_name}" normalized_dependency_name)
    # The library's own name, not the directory it sits in. An installation in an
    # unexpected place is exactly what a path match misses, and `/opt/drake/lib/
    # libfmt.so` is not Drake.
    if(normalized_dependency_name MATCHES "^(lib)?drake([._-]|$)")
        list(APPEND drake_dependencies "${dependency}")
    endif()
endforeach()

list(LENGTH drake_dependencies drake_count)
if(drake_count GREATER 0)
    list(JOIN drake_dependencies "\n  " report)
    message(FATAL_ERROR
        "the candidate's runtime dependency closure contains shared Drake:\n  "
        "${report}\n"
        "It exists to produce numbers that are compared against Drake's; if it "
        "loaded Drake those numbers would be Drake's.")
endif()

list(LENGTH resolved_dependencies resolved_count)
message(STATUS
    "the candidate's runtime dependency closure contains no shared Drake: "
    "${resolved_count} dependencies, all resolved")
