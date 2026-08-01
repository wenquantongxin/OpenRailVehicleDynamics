# Target-scoped compile policy for ORVD first-party targets.
#
# Everything here is applied PER TARGET. There is deliberately no global
# add_compile_options(), no mutation of CMAKE_CXX_FLAGS and no directory-scoped
# warning flag: a global flag reaches vendored Drake sources and external
# packages as well, and warnings we cannot act on are warnings that get ignored,
# which then hides the ones we could have acted on.
#
# Apply this to first-party targets only. Vendored and third-party targets are
# left alone. CMake treats include directories from imported targets as SYSTEM
# by default, so third-party diagnostics stay out of our build output.

# Warnings are informative, not fatal: -Werror in the build system turns an
# upstream compiler upgrade into a build outage on code that did not change.
# A project that wants errors can add them in CI on top of this.
set(ORVD_FIRST_PARTY_WARNINGS_GNU_LIKE -Wall -Wextra -Wpedantic)
set(ORVD_FIRST_PARTY_OPTIONS_MSVC
    /W4
    /permissive-
    /Zc:__cplusplus
    /Zc:preprocessor)

function(orvd_configure_first_party_target target_name)
    if(NOT TARGET ${target_name})
        message(FATAL_ERROR
            "orvd_configure_first_party_target: '${target_name}' is not a target")
    endif()

    get_target_property(first_party_target_type ${target_name} TYPE)
    if(first_party_target_type STREQUAL "INTERFACE_LIBRARY")
        message(FATAL_ERROR
            "orvd_configure_first_party_target requires a compiled target")
    endif()

    if(first_party_target_type STREQUAL "EXECUTABLE" OR
       first_party_target_type STREQUAL "MODULE_LIBRARY")
        set(language_standard_scope PRIVATE)
    else()
        # Public first-party headers are allowed to use C++23, so compiled
        # library targets must carry that requirement to their consumers.
        set(language_standard_scope PUBLIC)
    endif()

    target_compile_features(
        ${target_name} ${language_standard_scope} cxx_std_23)
    set_target_properties(${target_name} PROPERTIES CXX_EXTENSIONS OFF)

    if(MSVC OR CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
        target_compile_options(
            ${target_name} PRIVATE ${ORVD_FIRST_PARTY_OPTIONS_MSVC})
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
        target_compile_options(
            ${target_name} PRIVATE ${ORVD_FIRST_PARTY_WARNINGS_GNU_LIKE})
    else()
        message(FATAL_ERROR
            "Unsupported first-party C++ compiler: '${CMAKE_CXX_COMPILER_ID}'")
    endif()
endfunction()
