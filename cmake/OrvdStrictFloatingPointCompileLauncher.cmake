# Inspect the exact C++ compile command after CMake has expanded directory,
# target, imported-target and configuration-specific options.  This script is
# installed as the sole compiler launcher for every target in the source build.
# After a successful audit it injects the proof macro itself, so a qualification
# artifact cannot claim that the launcher ran when its generator ignored it.
# This launcher enforces the v1 safety category, not cross-build bit identity:
# it deliberately permits compiler-default contraction, -ffp-contract=fast/off,
# and explicit fused multiply-add evaluation while rejecting broader unsafe
# fast/finite-only semantics.

function(orvd_check_strict_floating_point_token argument)
    if(argument MATCHES "^@")
        string(SUBSTRING "${argument}" 1 -1 response_file)
        if(NOT IS_ABSOLUTE "${response_file}")
            set(response_file "${CMAKE_CURRENT_BINARY_DIR}/${response_file}")
        endif()
        if(NOT EXISTS "${response_file}")
            message(FATAL_ERROR
                "ORVD strict floating-point compile command rejected opaque "
                "response token '${argument}'")
        endif()
        file(READ "${response_file}" response_contents)
        separate_arguments(response_tokens NATIVE_COMMAND
                           "${response_contents}")
        foreach(response_token IN LISTS response_tokens)
            if(response_token MATCHES "^@")
                message(FATAL_ERROR
                    "ORVD strict floating-point compile command rejected "
                    "nested response token '${response_token}'")
            endif()
        endforeach()
        orvd_check_strict_floating_point_tokens(${response_tokens})
        return()
    endif()

    if(argument STREQUAL "--config" OR
       argument MATCHES "^--config=" OR
       argument STREQUAL "-mllvm" OR
       argument MATCHES "^-mllvm=" OR
       argument STREQUAL "-specs" OR
       argument MATCHES "^-specs=" OR
       argument STREQUAL "--specs" OR
       argument MATCHES "^--specs=" OR
       argument STREQUAL "-wrapper" OR
       argument MATCHES "^-wrapper=" OR
       argument MATCHES "^-Xclang($|=)" OR
       argument STREQUAL "-fplugin" OR
       argument MATCHES "^-fplugin(=|-arg-)" OR
       argument MATCHES "^-include" OR
       argument MATCHES "^-imacros" OR
       argument MATCHES "^-Ofast($|=)" OR
       argument MATCHES "^-ffast-math($|=)" OR
       argument MATCHES "^-ffinite-math-only($|=)" OR
       argument STREQUAL "-funsafe-math-optimizations" OR
       argument STREQUAL "-fassociative-math" OR
       argument STREQUAL "-freciprocal-math" OR
       argument STREQUAL "-fno-signed-zeros" OR
       argument STREQUAL "-fno-trapping-math" OR
       argument STREQUAL "-fapprox-func" OR
       argument STREQUAL "-fno-honor-nans" OR
       argument STREQUAL "-fno-honor-infinities" OR
       argument STREQUAL "-menable-no-nans" OR
       argument STREQUAL "-menable-no-infs" OR
       argument STREQUAL "-ffp-model=fast" OR
       argument STREQUAL "-ffp-model=aggressive" OR
       argument STREQUAL "-fcx-limited-range" OR
       argument STREQUAL "/fp:fast")
        message(FATAL_ERROR
            "ORVD strict floating-point compile command rejected token "
            "'${argument}'. Remove every unsafe token instead of "
            "countermanding it later.")
    endif()
endfunction()

function(orvd_check_strict_floating_point_tokens)
    set(tokens ${ARGN})
    list(LENGTH tokens token_count)
    set(index 0)
    while(index LESS token_count)
        list(GET tokens ${index} token)
        if(token STREQUAL "-Xclang")
            math(EXPR forwarded_index "${index} + 1")
            if(NOT ORVD_STRICT_FLOATING_POINT_CXX_COMPILER_ID STREQUAL
                   "AppleClang" OR
               forwarded_index GREATER_EQUAL token_count)
                orvd_check_strict_floating_point_token("${token}")
            endif()
            list(GET tokens ${forwarded_index} forwarded_token)
            if(NOT forwarded_token STREQUAL "-fopenmp")
                orvd_check_strict_floating_point_token("${token}")
            endif()
            math(EXPR index "${index} + 2")
        else()
            orvd_check_strict_floating_point_token("${token}")
            math(EXPR index "${index} + 1")
        endif()
    endwhile()
endfunction()

set(command)
set(found_separator FALSE)
set(last_argument -1)
if(CMAKE_ARGC GREATER 0)
    math(EXPR last_argument "${CMAKE_ARGC} - 1")
endif()

foreach(index RANGE 0 ${last_argument})
    set(argument "${CMAKE_ARGV${index}}")
    if(NOT found_separator)
        if(argument STREQUAL "--")
            set(found_separator TRUE)
        endif()
        continue()
    endif()

    list(APPEND command "${argument}")
endforeach()

if(NOT found_separator OR NOT command)
    message(FATAL_ERROR
        "ORVD strict floating-point compile launcher received no command")
endif()

orvd_check_strict_floating_point_tokens(${command})

list(APPEND command
     "-DORVD_STRICT_FLOATING_POINT_COMPILE_COMMAND_AUDIT_PASSED=1")
execute_process(COMMAND ${command} RESULT_VARIABLE compile_result)
if(NOT compile_result EQUAL 0)
    message(FATAL_ERROR
        "ORVD C++ compile command failed with status ${compile_result}")
endif()
