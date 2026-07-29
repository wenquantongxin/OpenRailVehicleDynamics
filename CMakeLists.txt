# OpenRailVehicleDynamics (ORVD) — top-level build
#
# STATUS: infrastructure stage. This file declares the project, the language
# standard and the intended module graph, but produces NO targets yet: every
# module src/ directory is still empty, and add_subdirectory() is gated on a
# module actually having a CMakeLists.txt. Wire each module in as it is
# implemented; the order is set by
# docs/planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md.
#
# The build entry point itself (compile policy, warning policy, preset schema)
# is established by G04; the dependency and test foundation by G05.

cmake_minimum_required(VERSION 3.24)

project(OpenRailVehicleDynamics
    VERSION 0.0.0
    DESCRIPTION "Model-neutral, double-only C++ multibody runtime with no Drake runtime dependency"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- External dependencies ---
# This skeleton configures without third-party packages. G05 introduces Eigen
# with its first real consumer; SUNDIALS CVODE and Ceres follow only when their
# first real consumers are implemented.

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

message(STATUS "ORVD ${PROJECT_VERSION} — infrastructure stage; C++${CMAKE_CXX_STANDARD}")
