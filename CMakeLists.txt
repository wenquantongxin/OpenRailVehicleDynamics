# OpenRailVehicleDynamics (ORVD) — top-level build
#
# STATUS: infrastructure stage. No product library or executable exists yet.
# With BUILD_TESTING enabled, the build contains only model-neutral self-checks.
# Product modules are wired in only when they gain a CMakeLists.txt; their order
# is set by docs/planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md.
#
# The first-party compile policy lives in cmake/OrvdFirstPartyTargetPolicy.cmake
# and is applied per target. Eigen and the CTest entry point are configured below.

cmake_minimum_required(VERSION 3.24)

project(OpenRailVehicleDynamics
    VERSION 0.0.0
    DESCRIPTION "Model-neutral, double-only C++ multibody runtime with no Drake runtime dependency"
    LANGUAGES CXX)

# The C++ standard and extension mode are declared per target by
# orvd_configure_first_party_target. Global CMAKE_CXX_* defaults would also
# affect vendored and third-party targets.

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(OrvdFirstPartyTargetPolicy)

# --- External dependencies ---
# Eigen is required outright: every multibody pass is Eigen arithmetic, so a
# build without it cannot produce anything useful and should fail at configure
# time rather than at the first compile error. CMake treats include directories
# from the imported Eigen3::Eigen target as SYSTEM by default, which keeps its
# diagnostics out of first-party output.
# SUNDIALS CVODE and Ceres have no consumer yet and are therefore not searched
# for, not made optional, and given no target.
find_package(Eigen3 3.4 CONFIG REQUIRED)

# --- Module graph, wired in as each module is implemented ---
# Dependency direction: external -> core -> {systems, forces} -> integrators
#   -> equilibrium ; tests last.
foreach(_dir
        external/drake_mbtree
        libs/core
        libs/systems
        libs/forces
        libs/integrators
        libs/equilibrium)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/CMakeLists.txt")
        add_subdirectory(${_dir})
    endif()
endforeach()

# include(CTest) defines BUILD_TESTING (default ON) and configures CTest.
include(CTest)
option(ORVD_BUILD_DRAKE_REFERENCE_TESTS
       "Build the Drake-backed reference emitter and comparison tests" OFF)
if(ORVD_BUILD_DRAKE_REFERENCE_TESTS AND NOT BUILD_TESTING)
    message(FATAL_ERROR
        "ORVD_BUILD_DRAKE_REFERENCE_TESTS requires BUILD_TESTING=ON")
endif()
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

message(STATUS "ORVD ${PROJECT_VERSION} — infrastructure stage")
