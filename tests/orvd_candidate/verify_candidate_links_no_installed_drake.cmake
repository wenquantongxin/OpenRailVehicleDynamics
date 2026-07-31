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
# The linked file has one answer. Whatever spelling produced it, an installed
# `libdrake` either is among its runtime dependencies or is not.
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

set(drake_dependencies "")
foreach(dependency IN LISTS resolved_dependencies unresolved_dependencies)
    get_filename_component(dependency_name "${dependency}" NAME)
    string(TOLOWER "${dependency_name}" normalized_dependency_name)
    # The library's own name, not the directory it sits in. An installation in an
    # unexpected place is exactly what a path match misses, and `/opt/drake/lib/
    # libfmt.so` is not Drake.
    if(normalized_dependency_name MATCHES "^(lib)?drake([._-]|$)")
        list(APPEND drake_dependencies "${dependency}")
    endif()
endforeach()

if(drake_dependencies)
    list(JOIN drake_dependencies "\n  " report)
    message(FATAL_ERROR
        "the candidate loads Drake at runtime:\n  ${report}\n"
        "It exists to produce numbers that are compared against Drake's; if it "
        "loaded Drake those numbers would be Drake's.")
endif()

list(LENGTH resolved_dependencies resolved_count)
message(STATUS
    "the candidate loads no Drake: ${resolved_count} runtime dependencies, none "
    "of them Drake")
