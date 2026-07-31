# Checks a built candidate's actual runtime dependencies for an installed Drake.
#
# On the built artefact, not on the CMake graph. A graph check has to model every
# way CMake can be told about a dependency — a library selection option in any of
# its spellings, a generator expression evaluated per configuration, an imported
# target declared in a scope where it is invisible, a link edge written from a
# different directory — and each of those is a separate thing to get right. Worse,
# `cmake_language(DEFER)` can change the graph after any configure-time check has
# run, so such a check cannot be the last word even in principle.
#
# The linked file's declared load-time closure has one answer. Whatever spelling
# produced it, an installed `libdrake` either is reachable through that closure
# or is not. What this does not see is a library loaded by name at run time —
# `dlopen`, `LoadLibrary`, `LD_PRELOAD` — because none of those is recorded in
# the file. Nothing in the candidate does that, and if something ever does, this
# check will not be the thing that notices.
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
        "the candidate loads Drake at runtime:\n  ${report}\n"
        "It exists to produce numbers that are compared against Drake's; if it "
        "loaded Drake those numbers would be Drake's.")
endif()

list(LENGTH resolved_dependencies resolved_count)
message(STATUS
    "the candidate loads no Drake: ${resolved_count} runtime dependencies, all "
    "of them resolved, none of them Drake")
