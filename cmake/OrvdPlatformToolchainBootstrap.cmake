include_guard(GLOBAL)

# Establish native macOS defaults before project() enables a compiler. Homebrew
# GCC cannot link its compiler probe without an Apple SDK, while an explicit
# toolchain file must remain authoritative over every target-platform choice.
function(orvd_initialize_native_macos_toolchain)
    if(NOT CMAKE_HOST_APPLE)
        return()
    endif()
    if(DEFINED CMAKE_SYSTEM_NAME AND
       NOT "${CMAKE_SYSTEM_NAME}" STREQUAL "" AND
       NOT CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        return()
    endif()
    if(DEFINED CMAKE_TOOLCHAIN_FILE AND
       NOT "${CMAKE_TOOLCHAIN_FILE}" STREQUAL "")
        return()
    endif()

    if(NOT DEFINED CMAKE_OSX_ARCHITECTURES OR
       "${CMAKE_OSX_ARCHITECTURES}" STREQUAL "")
        set(CMAKE_OSX_ARCHITECTURES arm64 CACHE STRING
            "ORVD macOS target architecture" FORCE)
    endif()

    if(DEFINED CMAKE_OSX_SYSROOT AND
       NOT "${CMAKE_OSX_SYSROOT}" STREQUAL "")
        return()
    endif()
    if(DEFINED ENV{SDKROOT} AND NOT "$ENV{SDKROOT}" STREQUAL "")
        set(CMAKE_OSX_SYSROOT "$ENV{SDKROOT}" CACHE STRING
            "ORVD native macOS SDK from SDKROOT" FORCE)
        return()
    endif()

    find_program(_orvd_xcrun_executable NAMES xcrun NO_CACHE)
    if(NOT _orvd_xcrun_executable)
        message(FATAL_ERROR
            "a native macOS ORVD build requires xcrun from the Apple Command "
            "Line Tools so CMake can select the platform SDK")
    endif()
    execute_process(
        COMMAND "${_orvd_xcrun_executable}" --sdk macosx --show-sdk-path
        RESULT_VARIABLE _orvd_xcrun_result
        OUTPUT_VARIABLE _orvd_macos_sdk
        ERROR_VARIABLE _orvd_xcrun_error
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _orvd_xcrun_result EQUAL 0 OR
       NOT IS_DIRECTORY "${_orvd_macos_sdk}")
        message(FATAL_ERROR
            "xcrun could not resolve the native macOS SDK: "
            "${_orvd_xcrun_error}")
    endif()
    set(CMAKE_OSX_SYSROOT "${_orvd_macos_sdk}" CACHE PATH
        "ORVD native macOS SDK" FORCE)
endfunction()

# Reject configurations outside the qualified macOS source-support boundary.
# An explicit user value is honoured by the bootstrap and then validated here.
function(orvd_require_supported_macos_target)
    if(NOT APPLE)
        return()
    endif()
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        message(FATAL_ERROR
            "ORVD Apple-platform source support is limited to macOS")
    endif()
    if(NOT CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64")
        message(FATAL_ERROR
            "ORVD macOS source support requires an Apple silicon host")
    endif()

    if(NOT "${CMAKE_OSX_ARCHITECTURES}" STREQUAL "")
        set(_orvd_macos_architectures ${CMAKE_OSX_ARCHITECTURES})
        list(LENGTH _orvd_macos_architectures _orvd_architecture_count)
        if(NOT _orvd_architecture_count EQUAL 1)
            message(FATAL_ERROR
                "ORVD macOS source support requires the single arm64 "
                "architecture; universal builds are not supported")
        endif()
        list(GET _orvd_macos_architectures 0 _orvd_macos_architecture)
        if(NOT _orvd_macos_architecture STREQUAL "arm64")
            message(FATAL_ERROR
                "ORVD macOS source support requires the arm64 architecture")
        endif()
    elseif(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
        message(FATAL_ERROR
            "ORVD macOS source support requires the arm64 architecture")
    endif()
endfunction()

# AppleClang does not ship an OpenMP runtime. Resolve the supported Homebrew
# formula as a fallback search prefix without writing compiler flags, include
# paths, or runtime-library filenames into the project configuration.
function(orvd_resolve_appleclang_openmp_prefix output_variable)
    set(${output_variable} "" PARENT_SCOPE)
    if(NOT APPLE OR
       NOT CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
        return()
    endif()

    find_program(_orvd_brew_executable NAMES brew NO_CACHE)
    if(NOT _orvd_brew_executable)
        return()
    endif()
    execute_process(
        COMMAND "${_orvd_brew_executable}" --prefix libomp
        RESULT_VARIABLE _orvd_brew_result
        OUTPUT_VARIABLE _orvd_libomp_prefix
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _orvd_brew_result EQUAL 0 OR
       NOT IS_DIRECTORY "${_orvd_libomp_prefix}" OR
       NOT EXISTS "${_orvd_libomp_prefix}/include/omp.h" OR
       (NOT EXISTS "${_orvd_libomp_prefix}/lib/libomp.dylib" AND
        NOT EXISTS "${_orvd_libomp_prefix}/lib/libomp.a"))
        return()
    endif()
    set(${output_variable} "${_orvd_libomp_prefix}" PARENT_SCOPE)
endfunction()
