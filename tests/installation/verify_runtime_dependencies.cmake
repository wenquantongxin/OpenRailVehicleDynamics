cmake_minimum_required(VERSION 3.24)

if(NOT DEFINED CHECKED_EXECUTABLE)
    message(FATAL_ERROR "CHECKED_EXECUTABLE was not given")
endif()
if(NOT EXISTS "${CHECKED_EXECUTABLE}")
    message(FATAL_ERROR
        "the checked executable '${CHECKED_EXECUTABLE}' does not exist")
endif()

set(runtime_dependency_arguments
    EXECUTABLES "${CHECKED_EXECUTABLE}"
    RESOLVED_DEPENDENCIES_VAR resolved_dependencies
    UNRESOLVED_DEPENDENCIES_VAR unresolved_dependencies
    CONFLICTING_DEPENDENCIES_PREFIX runtime_conflicts)
if(DEFINED RUNTIME_SEARCH_DIRECTORIES AND RUNTIME_SEARCH_DIRECTORIES)
    list(APPEND runtime_dependency_arguments
         DIRECTORIES ${RUNTIME_SEARCH_DIRECTORIES})
endif()

file(GET_RUNTIME_DEPENDENCIES ${runtime_dependency_arguments})

# More than one loader search root can contain the same C/C++ runtime basename
# (for example a toolchain prefix and the host system).  That is not an
# unresolved edge, and the executable has already proved which candidate its
# native loader selects by running successfully.  Keep every reported candidate
# in the identity scan below so a conflicting Drake basename cannot disappear
# behind CMake's selected path.
foreach(conflicting_name IN LISTS runtime_conflicts_FILENAMES)
    set(conflicting_variable "runtime_conflicts_${conflicting_name}")
    list(APPEND resolved_dependencies ${${conflicting_variable}})
endforeach()
list(REMOVE_DUPLICATES resolved_dependencies)

list(LENGTH unresolved_dependencies unresolved_count)
if(unresolved_count GREATER 0)
    list(JOIN unresolved_dependencies "\n  " unresolved_report)
    message(FATAL_ERROR
        "the executable's runtime dependency closure could not be resolved; "
        "dependencies behind these nodes were not inspected:\n  "
        "${unresolved_report}")
endif()

set(drake_dependencies "")
foreach(dependency IN LISTS resolved_dependencies)
    get_filename_component(dependency_name "${dependency}" NAME)
    string(TOLOWER "${dependency_name}" normalized_dependency_name)
    if(normalized_dependency_name MATCHES "^(lib)?drake([._-]|$)")
        list(APPEND drake_dependencies "${dependency}")
    endif()
endforeach()

list(LENGTH drake_dependencies drake_count)
if(drake_count GREATER 0)
    list(JOIN drake_dependencies "\n  " drake_report)
    message(FATAL_ERROR
        "the executable's runtime dependency closure contains shared Drake:\n  "
        "${drake_report}")
endif()

list(LENGTH resolved_dependencies resolved_count)
message(STATUS
    "runtime dependency closure is complete and contains no shared Drake: "
    "${resolved_count} resolved dependencies")
