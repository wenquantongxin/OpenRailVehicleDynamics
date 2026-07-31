# Configure-time gate: no product target may depend on Drake.
#
# The product vendors Drake source. It must not link Drake. Those two facts are
# easy to keep straight while someone is looking and easy to lose the moment a
# new target picks up a convenient dependency, so this is checked by the build
# system rather than by a habit.
#
# Which targets are product targets is decided by directory, not by a target list
# someone has to remember to update: every non-imported target defined in a listed
# product module directory or one of its subdirectories is one. A new top-level
# product module still has to be added to ORVD_PRODUCT_MODULE_DIRECTORIES.
#
# The check is on the target graph, at configure time. That is the earliest point
# where the answer is knowable, and it is the same answer on every platform. It
# is deliberately not `ldd` or `readelf`: those would report only what one
# operating system's loader does with one build, and passing them would create a
# cross-platform confidence the evidence does not support. What a release package
# actually carries is a separate question, settled at G50.
#
# Imported targets must also remain inspectable. The build sets
# CMAKE_FIND_PACKAGE_TARGETS_GLOBAL so find_package targets are visible here. A
# hand-written imported target in a product directory must likewise be GLOBAL;
# otherwise configuration fails instead of silently treating an opaque target as
# an innocent bare library name.
#
# Machine paths are not evidence. A dependency is Drake because of the library's
# own identity — a `drake::` target, a library file named `libdrake…`, an
# `-ldrake` option — not because it happens to live under a directory with
# "drake" in the name. An installation somewhere unexpected is exactly the case a
# path match misses.

# A library file belongs to Drake if its own name says so. `liborvd_vendored_…`
# does not match: our archives are named for us, not for the source we vendored.
set(ORVD_DRAKE_LIBRARY_BASENAME_PATTERN "^(lib)?drake([._-]|$)")

# Library-selection options, as opposed to directory options: `-ldrake` names a
# library, `-L/opt/drake/lib` names a place to look and is not a dependency.
set(ORVD_DRAKE_LINK_OPTION_PATTERN
    "(^|[,;: =<>])(-l(:|,)?|--library(=|,)|/defaultlib:)(lib)?drake([._-]|[,;: =<>]|$)")

# A file name may be wrapped in a generator expression or a linker-driver option.
# Match a path component, not an installation directory: `/some/drake/libfmt.so`
# is not Drake, while `/somewhere/libdrake.so` is.
set(ORVD_DRAKE_LIBRARY_TEXT_PATTERN
    "(^|[/\\\\,;:=<> ])(lib)?drake([._-][^/\\\\,;:=<> ]*)?([,;:=<> ]|$)")

# Link options may also carry literal library files, for example as an argument
# to `--whole-archive` or `-force_load`. Requiring a library suffix keeps search
# and runtime paths such as `-L/opt/drake` from being mistaken for dependencies.
set(ORVD_DRAKE_LIBRARY_FILE_TEXT_PATTERN
    "(^|[/\\\\,;:=<> ])(lib)?drake([^/\\\\,;:=<> ]*)\\.(a|so(\\.[0-9]+)*|dylib|lib|dll)([,;:=<> ]|$)")

# A target namespace must start at a target-token boundary. In particular,
# `mandrake::core` is unrelated, while a target inside a generator expression
# still has `:` immediately before `drake::`.
set(ORVD_DRAKE_TARGET_TEXT_PATTERN "(^|[,;: =<>])drake::")

function(_orvd_record_boundary_violation product_target detail)
    set_property(GLOBAL APPEND PROPERTY ORVD_PRODUCT_BOUNDARY_VIOLATIONS
        "${product_target}: ${detail}")
endfunction()

# True when a link item, read as a library file path or bare library name, is a
# Drake library.
function(_orvd_link_item_is_drake_library item output_variable)
    string(TOLOWER "${item}" normalized_item)
    get_filename_component(item_basename "${normalized_item}" NAME)
    if(item_basename MATCHES "${ORVD_DRAKE_LIBRARY_BASENAME_PATTERN}" OR
       normalized_item MATCHES "${ORVD_DRAKE_LIBRARY_TEXT_PATTERN}")
        set(${output_variable} TRUE PARENT_SCOPE)
    else()
        set(${output_variable} FALSE PARENT_SCOPE)
    endif()
endfunction()

function(_orvd_imported_configuration_names target output_variable)
    get_property(configuration_names GLOBAL PROPERTY ORVD_BOUNDARY_CONFIGURATION_NAMES)
    get_target_property(imported_configuration_names
        ${target} IMPORTED_CONFIGURATIONS)
    if(imported_configuration_names)
        list(APPEND configuration_names ${imported_configuration_names})
    endif()
    foreach(configuration_name IN LISTS configuration_names)
        string(TOUPPER "${configuration_name}" upper_configuration_name)
        get_target_property(mapped_configuration_names
            ${target} "MAP_IMPORTED_CONFIG_${upper_configuration_name}")
        if(mapped_configuration_names)
            list(APPEND configuration_names ${mapped_configuration_names})
        endif()
    endforeach()
    list(REMOVE_DUPLICATES configuration_names)
    set(${output_variable} "${configuration_names}" PARENT_SCOPE)
endfunction()

# Checks an imported target's own artefact, which is where a third-party package
# records the library it actually resolves to.
function(_orvd_imported_artefact_is_drake target output_variable)
    set(${output_variable} FALSE PARENT_SCOPE)
    set(artefact_properties
        IMPORTED_LOCATION IMPORTED_IMPLIB IMPORTED_LIBNAME)
    _orvd_imported_configuration_names(${target} configuration_names)
    foreach(configuration_name IN LISTS configuration_names)
        string(TOUPPER "${configuration_name}" upper_configuration_name)
        list(APPEND artefact_properties
            "IMPORTED_LOCATION_${upper_configuration_name}"
            "IMPORTED_IMPLIB_${upper_configuration_name}"
            "IMPORTED_LIBNAME_${upper_configuration_name}")
    endforeach()
    foreach(artefact_property IN LISTS artefact_properties)
        get_target_property(artefact ${target} ${artefact_property})
        if(artefact)
            # A generator expression here is not a third-party habit to be
            # tolerated: it is the one place an imported target says which file
            # it resolves to, and an expression this gate cannot evaluate can
            # resolve to Drake in exactly the configuration nobody checked. The
            # identity match below runs on the raw text, so a Drake named inside
            # the expression is still caught — what escapes is a neutral name
            # that resolves to Drake, which is why this fails closed instead.
            if(artefact MATCHES "\\$<")
                set(${output_variable} TRUE PARENT_SCOPE)
                _orvd_record_boundary_violation("${target}"
                    "its ${artefact_property} is the generator expression '${artefact}', which this gate cannot evaluate; an imported target must name the file it resolves to")
                return()
            endif()
            _orvd_link_item_is_drake_library("${artefact}" artefact_is_drake)
            if(artefact_is_drake)
                set(${output_variable} TRUE PARENT_SCOPE)
                return()
            endif()
        endif()
    endforeach()
endfunction()

function(_orvd_check_link_options product_target target is_first_party)
    set(option_properties
        LINK_OPTIONS INTERFACE_LINK_OPTIONS LINK_FLAGS STATIC_LIBRARY_OPTIONS)
    get_property(configuration_names GLOBAL PROPERTY ORVD_BOUNDARY_CONFIGURATION_NAMES)
    foreach(configuration_name IN LISTS configuration_names)
        string(TOUPPER "${configuration_name}" upper_configuration_name)
        list(APPEND option_properties "LINK_FLAGS_${upper_configuration_name}")
    endforeach()
    foreach(option_property IN LISTS option_properties)
        get_target_property(options ${target} ${option_property})
        if(NOT options)
            continue()
        endif()
        # Flatten first. A library selector and the library it selects may arrive
        # as two list elements (`-l;drake`), as one element with a space
        # (`-l drake`), or wrapped in a driver prefix (`LINKER:SHELL:-l drake`).
        # Scanning elements one at a time sees `-l` and `drake` separately, and
        # neither of them alone says Drake.
        set(option_tokens "")
        foreach(option IN LISTS options)
            set(unwrapped_option "${option}")
            string(REGEX REPLACE "^(LINKER|SHELL):" "" unwrapped_option
                   "${unwrapped_option}")
            string(REGEX REPLACE "^(LINKER|SHELL):" "" unwrapped_option
                   "${unwrapped_option}")
            string(REPLACE " " ";" unwrapped_option "${unwrapped_option}")
            string(REPLACE "," ";" unwrapped_option "${unwrapped_option}")
            list(APPEND option_tokens ${unwrapped_option})
        endforeach()
        set(selector_pending FALSE)
        foreach(option_token IN LISTS option_tokens)
            string(TOLOWER "${option_token}" normalized_token)
            if(selector_pending)
                set(selector_pending FALSE)
                if(normalized_token MATCHES
                   "${ORVD_DRAKE_LIBRARY_BASENAME_PATTERN}")
                    _orvd_record_boundary_violation("${product_target}"
                        "${target} carries a link option selecting the library '${option_token}', which is Drake")
                    continue()
                endif()
            endif()
            if(normalized_token MATCHES "^(-l|--library|-framework|/defaultlib:?)$")
                # A bare selector: the library it selects is the next token.
                set(selector_pending TRUE)
            endif()
        endforeach()
        foreach(option IN LISTS options)
            string(TOLOWER "${option}" normalized_option)
            if(normalized_option MATCHES "${ORVD_DRAKE_LINK_OPTION_PATTERN}")
                _orvd_record_boundary_violation("${product_target}"
                    "${target} carries the link option '${option}', which selects a Drake library")
            elseif(normalized_option MATCHES
                    "${ORVD_DRAKE_LIBRARY_FILE_TEXT_PATTERN}")
                _orvd_record_boundary_violation("${product_target}"
                    "${target} carries the link option '${option}', which names a Drake library file")
            elseif(option MATCHES "\\$<" AND is_first_party)
                _orvd_record_boundary_violation("${product_target}"
                    "${target} has a generator expression in ${option_property} ('${option}') that this gate cannot evaluate; a first-party product target must not hide its link options from the boundary check")
            endif()
        endforeach()
    endforeach()
endfunction()

# Walks one product target's transitive link closure. Recursion, with a visited
# set, because the graph is a DAG with repeated nodes: fmt appears under almost
# everything.
function(_orvd_walk_link_closure product_target item is_first_party)
    get_property(visited GLOBAL PROPERTY ORVD_BOUNDARY_VISITED)
    if("${product_target}|${item}" IN_LIST visited)
        return()
    endif()
    set_property(GLOBAL APPEND PROPERTY ORVD_BOUNDARY_VISITED "${product_target}|${item}")

    # CMake wraps some link items itself. `$<LINK_ONLY:X>` is how a static or
    # object library's PRIVATE dependencies appear in its INTERFACE_LINK_LIBRARIES;
    # the build-interface wrappers come from export sets. These are not
    # dependencies being hidden from the check, they are dependencies CMake
    # spelled differently, so unwrap and keep walking. Anything else containing a
    # generator expression really is unevaluable here, and for a first-party
    # product target that is a failure rather than a shrug.
    if(item MATCHES "^\\$<(LINK_ONLY|BUILD_INTERFACE|INSTALL_INTERFACE|COMPILE_ONLY):(.+)>$")
        _orvd_walk_link_closure(
            "${product_target}" "${CMAKE_MATCH_2}" ${is_first_party})
        return()
    endif()

    # Drake's identity is in the item's own text, so read it before asking whether
    # it resolves to a target. This also catches an identity nested inside an
    # imported target's conditional generator expression.
    string(TOLOWER "${item}" normalized_item)
    if(normalized_item MATCHES "${ORVD_DRAKE_TARGET_TEXT_PATTERN}" OR
       normalized_item STREQUAL "drake")
        _orvd_record_boundary_violation("${product_target}"
            "it depends on the Drake target '${item}'")
        return()
    endif()
    if(normalized_item MATCHES "${ORVD_DRAKE_LINK_OPTION_PATTERN}")
        _orvd_record_boundary_violation("${product_target}"
            "the link item '${item}' selects a Drake library")
        return()
    endif()
    _orvd_link_item_is_drake_library("${item}" item_is_drake)
    if(item_is_drake)
        _orvd_record_boundary_violation("${product_target}"
            "the link item '${item}' is a Drake library")
        return()
    endif()

    if(NOT TARGET "${item}")
        # A bare link item: a library path, a bare library name, or a link option
        # that arrived through target_link_libraries. None of them is Drake, or the
        # checks above would have said so.
        if(item MATCHES "\\$<" AND is_first_party)
            _orvd_record_boundary_violation("${product_target}"
                "the link item '${item}' is a generator expression this gate cannot evaluate; a first-party product target must not hide a dependency from the boundary check")
        endif()
        return()
    endif()

    get_target_property(is_imported ${item} IMPORTED)
    if(is_imported)
        _orvd_imported_artefact_is_drake(${item} artefact_is_drake)
        if(artefact_is_drake)
            _orvd_record_boundary_violation("${product_target}"
                "the imported target '${item}' resolves to a Drake library")
            return()
        endif()
    endif()

    # An imported third-party target may legitimately carry generator expressions
    # we cannot evaluate; a first-party product target may not. Drake named inside
    # such an expression fails either way, because the pattern match above runs on
    # the raw text.
    set(item_is_first_party FALSE)
    if(NOT is_imported)
        set(item_is_first_party TRUE)
        get_target_property(item_source_directory ${item} SOURCE_DIR)
        if(item_source_directory)
            set_property(GLOBAL APPEND PROPERTY ORVD_BOUNDARY_CLOSURE_DIRECTORIES
                         "${item_source_directory}")
        endif()
    endif()
    _orvd_check_link_options("${product_target}" ${item} ${item_is_first_party})

    set(edges "")
    set(edge_properties
        LINK_LIBRARIES INTERFACE_LINK_LIBRARIES INTERFACE_LINK_LIBRARIES_DIRECT)
    if(is_imported)
        list(APPEND edge_properties
            IMPORTED_LINK_INTERFACE_LIBRARIES
            IMPORTED_LINK_DEPENDENT_LIBRARIES)
        _orvd_imported_configuration_names(${item} imported_configuration_names)
        foreach(configuration_name IN LISTS imported_configuration_names)
            string(TOUPPER "${configuration_name}" upper_configuration_name)
            list(APPEND edge_properties
                "IMPORTED_LINK_INTERFACE_LIBRARIES_${upper_configuration_name}"
                "IMPORTED_LINK_DEPENDENT_LIBRARIES_${upper_configuration_name}")
        endforeach()
    endif()
    foreach(edge_property IN LISTS edge_properties)
        get_target_property(edge_values ${item} ${edge_property})
        if(edge_values)
            list(APPEND edges ${edge_values})
        endif()
    endforeach()
    if(NOT edges)
        return()
    endif()
    list(REMOVE_DUPLICATES edges)
    foreach(edge IN LISTS edges)
        _orvd_walk_link_closure("${product_target}" "${edge}" ${item_is_first_party})
    endforeach()
endfunction()

# Refuses a directory that declares an imported target this gate cannot see.
#
# IMPORTED_TARGETS lists targets created in a directory even when their directory
# scope makes them invisible from where the check runs. An invisible target
# cannot be inspected, so accepting its neutral name would turn it into the
# escape hatch.
function(_orvd_audit_directory_imported_targets subject directory)
    get_property(directory_imported_targets
        DIRECTORY "${directory}" PROPERTY IMPORTED_TARGETS)
    foreach(imported_target IN LISTS directory_imported_targets)
        if(NOT TARGET "${imported_target}")
            _orvd_record_boundary_violation("${subject}"
                "its link closure passes through '${directory}', which defines the directory-scoped imported target '${imported_target}'; an imported target in that closure must be GLOBAL so the boundary gate can inspect it")
        endif()
    endforeach()
endfunction()

function(_orvd_collect_product_directory product_module_directory)
    get_property(visited_directories GLOBAL
        PROPERTY ORVD_PRODUCT_BOUNDARY_VISITED_DIRECTORIES)
    if(product_module_directory IN_LIST visited_directories)
        return()
    endif()
    set_property(GLOBAL APPEND PROPERTY
        ORVD_PRODUCT_BOUNDARY_VISITED_DIRECTORIES "${product_module_directory}")

    get_property(directory_targets
        DIRECTORY "${product_module_directory}" PROPERTY BUILDSYSTEM_TARGETS)
    if(directory_targets)
        set_property(GLOBAL APPEND PROPERTY
            ORVD_PRODUCT_BOUNDARY_TARGETS ${directory_targets})
    endif()

    _orvd_audit_directory_imported_targets(
        "${product_module_directory}" "${product_module_directory}")

    get_property(product_subdirectories
        DIRECTORY "${product_module_directory}" PROPERTY SUBDIRECTORIES)
    foreach(product_subdirectory IN LISTS product_subdirectories)
        _orvd_collect_product_directory("${product_subdirectory}")
    endforeach()
endfunction()

# Every non-imported target defined under the given directories or their
# descendants is a product target. Call after every add_subdirectory(), including
# the test tree: targets defined later must not escape the gate.
function(orvd_verify_product_targets_have_no_drake_dependency)
    set(product_module_relative_paths ${ARGN})
    set_property(GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_VIOLATIONS "")
    set_property(GLOBAL PROPERTY ORVD_BOUNDARY_VISITED "")
    set_property(GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_VISITED_DIRECTORIES "")
    set_property(GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_TARGETS "")
    set_property(GLOBAL PROPERTY ORVD_BOUNDARY_CLOSURE_DIRECTORIES "")

    set(configuration_names "")
    if(CMAKE_CONFIGURATION_TYPES)
        set(configuration_names ${CMAKE_CONFIGURATION_TYPES})
    elseif(CMAKE_BUILD_TYPE)
        set(configuration_names ${CMAKE_BUILD_TYPE})
    endif()
    set_property(GLOBAL PROPERTY ORVD_BOUNDARY_CONFIGURATION_NAMES "${configuration_names}")

    foreach(product_module_relative_path IN LISTS product_module_relative_paths)
        set(product_module_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/${product_module_relative_path}")
        if(NOT EXISTS "${product_module_directory}/CMakeLists.txt")
            continue()
        endif()
        _orvd_collect_product_directory("${product_module_directory}")
    endforeach()

    get_property(product_targets GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_TARGETS)
    if(product_targets)
        list(REMOVE_DUPLICATES product_targets)
    endif()
    get_property(violations GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_VIOLATIONS)
    if(NOT product_targets AND NOT violations)
        message(STATUS
            "ORVD product boundary: no product target exists yet; nothing to check")
        return()
    endif()

    foreach(product_target IN LISTS product_targets)
        _orvd_walk_link_closure("${product_target}" "${product_target}" TRUE)
    endforeach()

    get_property(violations GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_VIOLATIONS)
    if(violations)
        list(JOIN violations "\n  " violation_report)
        message(FATAL_ERROR
            "ORVD product boundary violated. The product vendors Drake source and"
            " must not link Drake:\n  ${violation_report}")
    endif()
    list(JOIN product_targets ", " checked_report)
    message(STATUS
        "ORVD product boundary: no Drake dependency in ${checked_report}")
endfunction()

# The same check, on targets named outright rather than found by directory.
#
# A cross-process candidate is not a product module — it is a test program — but
# it carries the same prohibition for a sharper reason: it exists to produce
# numbers that are compared against Drake's, and if it linked the installed
# library those numbers would be Drake's. The directory-based entry point cannot
# reach it, because it is declared beside the tests that consume it rather than
# in a module of its own.
#
# Call after the targets exist. Names that are not targets are a failure rather
# than a skip: a gate that silently checks nothing is worse than no gate, because
# it is written down as if it did something.
function(orvd_verify_targets_have_no_drake_dependency)
    set(named_targets ${ARGN})
    if(NOT named_targets)
        message(FATAL_ERROR
            "orvd_verify_targets_have_no_drake_dependency was called with no "
            "target, which cannot be what the caller meant")
    endif()

    set_property(GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_VIOLATIONS "")
    set_property(GLOBAL PROPERTY ORVD_BOUNDARY_VISITED "")
    set_property(GLOBAL PROPERTY ORVD_BOUNDARY_CLOSURE_DIRECTORIES "")

    set(configuration_names "")
    if(CMAKE_CONFIGURATION_TYPES)
        set(configuration_names ${CMAKE_CONFIGURATION_TYPES})
    elseif(CMAKE_BUILD_TYPE)
        set(configuration_names ${CMAKE_BUILD_TYPE})
    endif()
    set_property(GLOBAL PROPERTY ORVD_BOUNDARY_CONFIGURATION_NAMES
                 "${configuration_names}")

    set_property(GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_VISITED_DIRECTORIES "")
    foreach(named_target IN LISTS named_targets)
        if(NOT TARGET ${named_target})
            message(FATAL_ERROR
                "ORVD boundary: '${named_target}' is not a target, so the "
                "prohibition written down for it checks nothing")
        endif()
        _orvd_walk_link_closure("${named_target}" "${named_target}" TRUE)

        # The same audit the directory entry performs, over the directories this
        # target's own closure was declared in. A directory-scoped imported
        # target is invisible from here, so the walk above sees a neutral name
        # and a bare link item — which is exactly what an escape looks like.
        get_property(closure_directories GLOBAL
            PROPERTY ORVD_BOUNDARY_CLOSURE_DIRECTORIES)
        foreach(closure_directory IN LISTS closure_directories)
            _orvd_audit_directory_imported_targets(
                "${named_target}" "${closure_directory}")
        endforeach()
    endforeach()

    get_property(violations GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_VIOLATIONS)
    if(violations)
        list(JOIN violations "\n  " violation_report)
        message(FATAL_ERROR
            "ORVD boundary violated. These targets must not link Drake:\n  "
            "${violation_report}")
    endif()
    list(JOIN named_targets ", " checked_targets)
    message(STATUS
        "ORVD boundary: ${checked_targets} link no Drake")
endfunction()
