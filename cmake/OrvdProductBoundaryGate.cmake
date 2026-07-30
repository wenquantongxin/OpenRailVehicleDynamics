# Configure-time gate: no product target may depend on Drake.
#
# The product vendors Drake source. It must not link Drake. Those two facts are
# easy to keep straight while someone is looking and easy to lose the moment a
# new target picks up a convenient dependency, so this is checked by the build
# system rather than by a habit.
#
# Which targets are product targets is decided by directory, not by a list
# someone has to remember to update: every non-imported target defined under the
# product module directories is one. A registration call would fail silently the
# first time somebody forgot it, and the failure mode of a boundary check that
# silently stops checking is the worst one available.
#
# The check is on the target graph, at configure time. That is the earliest point
# where the answer is knowable, and it is the same answer on every platform. It
# is deliberately not `ldd` or `readelf`: those would report only what one
# operating system's loader does with one build, and passing them would create a
# cross-platform confidence the evidence does not support. What a release package
# actually carries is a separate question, settled at G50.
#
# What the gate can see: the top-level scope. Imported targets are directory
# scoped by default, so the build sets CMAKE_FIND_PACKAGE_TARGETS_GLOBAL and every
# find_package target — which is how a real Drake arrives — is visible here. A
# hand-written `add_library(... IMPORTED)` without GLOBAL inside a product
# directory would not be, and the gate reads such an item as the bare library name
# it would otherwise become on the link line. That residue is a deliberate act
# rather than an accident, and the identity checks below still catch it whenever
# the name or the option says Drake.
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
set(ORVD_DRAKE_LINK_OPTION_PATTERN "^(-l:?|/DEFAULTLIB:|/defaultlib:)(lib)?drake([._-]|$)")

function(_orvd_record_boundary_violation product_target detail)
    set_property(GLOBAL APPEND PROPERTY ORVD_PRODUCT_BOUNDARY_VIOLATIONS
        "${product_target}: ${detail}")
endfunction()

# True when a link item, read as a library file path or bare library name, is a
# Drake library.
function(_orvd_link_item_is_drake_library item output_variable)
    get_filename_component(item_basename "${item}" NAME)
    if(item_basename MATCHES "${ORVD_DRAKE_LIBRARY_BASENAME_PATTERN}")
        set(${output_variable} TRUE PARENT_SCOPE)
    else()
        set(${output_variable} FALSE PARENT_SCOPE)
    endif()
endfunction()

# Checks an imported target's own artefact, which is where a third-party package
# records the library it actually resolves to.
function(_orvd_imported_artefact_is_drake target output_variable)
    set(${output_variable} FALSE PARENT_SCOPE)
    set(artefact_properties IMPORTED_LOCATION IMPORTED_IMPLIB)
    get_property(configuration_names GLOBAL PROPERTY ORVD_BOUNDARY_CONFIGURATION_NAMES)
    foreach(configuration_name IN LISTS configuration_names)
        string(TOUPPER "${configuration_name}" upper_configuration_name)
        list(APPEND artefact_properties
            "IMPORTED_LOCATION_${upper_configuration_name}"
            "IMPORTED_IMPLIB_${upper_configuration_name}")
    endforeach()
    foreach(artefact_property IN LISTS artefact_properties)
        get_target_property(artefact ${target} ${artefact_property})
        if(artefact)
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
    foreach(option_property IN LISTS option_properties)
        get_target_property(options ${target} ${option_property})
        if(NOT options)
            continue()
        endif()
        foreach(option IN LISTS options)
            if(option MATCHES "${ORVD_DRAKE_LINK_OPTION_PATTERN}")
                _orvd_record_boundary_violation("${product_target}"
                    "${target} carries the link option '${option}', which selects a Drake library")
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

    # Drake's identity is in the item's own text, so it is read before asking
    # whether the item resolves to a target here. `find_package` creates
    # directory-scoped targets by default, so a name that is a target where it was
    # linked can arrive at this scope as a plain string; a check that only looked
    # at resolved targets would wave it through.
    if(item MATCHES "^drake::" OR item STREQUAL "drake")
        _orvd_record_boundary_violation("${product_target}"
            "it depends on the Drake target '${item}'")
        return()
    endif()
    if(item MATCHES "${ORVD_DRAKE_LINK_OPTION_PATTERN}")
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
    endif()
    _orvd_check_link_options("${product_target}" ${item} ${item_is_first_party})

    set(edges "")
    foreach(edge_property IN ITEMS
            LINK_LIBRARIES INTERFACE_LINK_LIBRARIES INTERFACE_LINK_LIBRARIES_DIRECT)
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

# Every non-imported target defined under the given directories is a product
# target. Call after every add_subdirectory(), including the test tree: targets
# defined later must not escape the gate by being defined later.
function(orvd_verify_product_targets_have_no_drake_dependency)
    set(product_module_relative_paths ${ARGN})
    set_property(GLOBAL PROPERTY ORVD_PRODUCT_BOUNDARY_VIOLATIONS "")
    set_property(GLOBAL PROPERTY ORVD_BOUNDARY_VISITED "")

    set(configuration_names "")
    if(CMAKE_CONFIGURATION_TYPES)
        set(configuration_names ${CMAKE_CONFIGURATION_TYPES})
    elseif(CMAKE_BUILD_TYPE)
        set(configuration_names ${CMAKE_BUILD_TYPE})
    endif()
    set_property(GLOBAL PROPERTY ORVD_BOUNDARY_CONFIGURATION_NAMES "${configuration_names}")

    set(product_targets "")
    foreach(product_module_relative_path IN LISTS product_module_relative_paths)
        set(product_module_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/${product_module_relative_path}")
        if(NOT EXISTS "${product_module_directory}/CMakeLists.txt")
            continue()
        endif()
        get_property(directory_targets
            DIRECTORY "${product_module_directory}" PROPERTY BUILDSYSTEM_TARGETS)
        foreach(directory_target IN LISTS directory_targets)
            get_target_property(is_imported ${directory_target} IMPORTED)
            if(NOT is_imported)
                list(APPEND product_targets ${directory_target})
            endif()
        endforeach()
    endforeach()

    if(NOT product_targets)
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
