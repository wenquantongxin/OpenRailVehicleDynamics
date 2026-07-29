# Target-scoped compile policy for ORVD first-party targets.
#
# Everything here is applied PER TARGET. There is deliberately no global
# add_compile_options(), no mutation of CMAKE_CXX_FLAGS and no directory-scoped
# warning flag: a global flag reaches vendored Drake sources and external
# packages as well, and warnings we cannot act on are warnings that get ignored,
# which then hides the ones we could have acted on.
#
# Apply this to first-party targets only. Vendored and third-party targets are
# left alone; third-party headers are consumed through imported targets that
# mark their include directories SYSTEM, so their diagnostics stay out of our
# build output.

# Warnings are informative, not fatal: -Werror in the build system turns an
# upstream compiler upgrade into a build outage on code that did not change.
# A project that wants errors can add them in CI on top of this.
set(ORVD_FIRST_PARTY_WARNINGS_GNU_LIKE -Wall -Wextra -Wpedantic)
set(ORVD_FIRST_PARTY_WARNINGS_MSVC /W4 /permissive- /Zc:__cplusplus)

function(orvd_configure_first_party_target target_name)
    if(NOT TARGET ${target_name})
        message(FATAL_ERROR
            "orvd_configure_first_party_target: '${target_name}' is not a target")
    endif()

    get_target_property(target_type ${target_name} TYPE)
    if(target_type STREQUAL "INTERFACE_LIBRARY")
        set(scope INTERFACE)
    else()
        set(scope PRIVATE)
    endif()

    # The language standard is declared on the target rather than through the
    # CMAKE_CXX_STANDARD variable, so a consumer that sets its own standard
    # cannot silently downgrade ours.
    target_compile_features(${target_name} ${scope} cxx_std_23)

    if(MSVC)
        target_compile_options(${target_name} ${scope} ${ORVD_FIRST_PARTY_WARNINGS_MSVC})
    else()
        target_compile_options(${target_name} ${scope} ${ORVD_FIRST_PARTY_WARNINGS_GNU_LIKE})
    endif()
endfunction()
