# Fail-closed audit for the floating-point semantics used by ORVD numerical
# qualification.  The compiler-macro guards in the Radau5 core and artifact
# writers remain the final check; this layer also catches Clang options such as
# -ffp-model=fast that do not expose a complete predefined-macro signal.
#
# The v1 identifier is a floating-point safety category, not a cross-build
# bit-reproduction identity. It excludes fast/finite-only assumptions but does
# not freeze contraction: compiler defaults, -ffp-contract=fast/off, and
# explicit fused multiply-add evaluation remain admissible. A qualification
# artifact must therefore record its compiler/build identity separately.

set(ORVD_STRICT_FLOATING_POINT_SEMANTICS_IDENTIFIER
    "orvd.strict_ieee_no_fast_math.v1")

function(orvd_require_strict_floating_point_qualification)
    # Audit every configured C++ flag variable, not merely the currently
    # selected single-config build.  The compile launcher below remains the
    # authoritative check of the final command after directory, target,
    # imported-target and generator-expression expansion.
    get_cmake_property(cmake_variables VARIABLES)
    set(flag_variables)
    foreach(cmake_variable IN LISTS cmake_variables)
        if(cmake_variable MATCHES "^CMAKE_CXX_FLAGS($|_)")
            list(APPEND flag_variables "${cmake_variable}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES flag_variables)

    foreach(flag_variable IN LISTS flag_variables)
        if(NOT DEFINED ${flag_variable} OR
           "${${flag_variable}}" STREQUAL "")
            continue()
        endif()
        separate_arguments(flag_tokens NATIVE_COMMAND "${${flag_variable}}")
        foreach(flag_token IN LISTS flag_tokens)
            set(forbidden FALSE)
            # Response files are audited after their actual path is resolved by
            # the compile launcher.  Rejecting every @ token here would also
            # reject readable, qualification-safe response files.
            if(flag_token MATCHES "^-Ofast($|=)" OR
               flag_token MATCHES "^-ffast-math($|=)" OR
               flag_token MATCHES "^-ffinite-math-only($|=)" OR
               flag_token STREQUAL "-funsafe-math-optimizations" OR
               flag_token STREQUAL "-fassociative-math" OR
               flag_token STREQUAL "-freciprocal-math" OR
               flag_token STREQUAL "-fno-signed-zeros" OR
               flag_token STREQUAL "-fno-trapping-math" OR
               flag_token STREQUAL "-fapprox-func" OR
               flag_token STREQUAL "-fno-honor-nans" OR
               flag_token STREQUAL "-fno-honor-infinities" OR
               flag_token STREQUAL "-menable-no-nans" OR
               flag_token STREQUAL "-menable-no-infs" OR
               flag_token STREQUAL "-ffp-model=fast" OR
               flag_token STREQUAL "-ffp-model=aggressive" OR
               flag_token STREQUAL "-fcx-limited-range" OR
               flag_token STREQUAL "/fp:fast")
                set(forbidden TRUE)
            endif()
            if(forbidden)
                message(FATAL_ERROR
                    "ORVD strict floating-point qualification rejected "
                    "${flag_variable} token '${flag_token}'. Remove every "
                    "unsafe token instead of countermanding it later.")
            endif()
        endforeach()
    endforeach()

    set(ORVD_STRICT_FLOATING_POINT_FLAG_AUDIT_PASSED TRUE PARENT_SCOPE)
endfunction()

function(orvd_enable_strict_floating_point_compile_launcher)
    if(NOT CMAKE_GENERATOR MATCHES
       "^(Ninja|Ninja Multi-Config|.* Makefiles|.* Makefiles JOM)$")
        message(FATAL_ERROR
            "ORVD strict floating-point qualification requires a Ninja or "
            "Makefile generator that executes CMAKE_CXX_COMPILER_LAUNCHER; "
            "generator '${CMAKE_GENERATOR}' cannot provide that proof")
    endif()
    if(CMAKE_CXX_COMPILER_LAUNCHER)
        message(FATAL_ERROR
            "ORVD strict floating-point qualification rejects a pre-existing "
            "CMAKE_CXX_COMPILER_LAUNCHER because it could rewrite arguments "
            "outside the final audit")
    endif()

    set(launcher
        "${CMAKE_COMMAND}"
        "-DORVD_STRICT_FLOATING_POINT_CXX_COMPILER_ID:STRING=${CMAKE_CXX_COMPILER_ID}"
        "-P"
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/OrvdStrictFloatingPointCompileLauncher.cmake"
        "--")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${launcher}" PARENT_SCOPE)
    set(ORVD_STRICT_FLOATING_POINT_COMPILE_COMMAND_AUDIT_ENABLED
        TRUE PARENT_SCOPE)
endfunction()
