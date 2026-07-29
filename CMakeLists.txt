# OpenRailVehicleDynamics (ORVD) — top-level build
#
# STATUS: pre-implementation skeleton. This file declares the project, language
# standard, options and the intended module graph, but produces NO targets yet
# (module src/ directories are empty). add_subdirectory() calls are gated on the
# presence of a CMakeLists.txt in each module so configuration stays clean until
# code lands. Wire each module in as it is implemented (see docs/planning/PROJECT_PHASES.md).

cmake_minimum_required(VERSION 3.24)

project(OpenRailVehicleDynamics
    VERSION 0.0.0
    DESCRIPTION "Zero-Drake, self-contained C++ rail-vehicle dynamics core (GZ18)"
    LANGUAGES CXX)

# --- Language standard (matches wheel-rail-lab) ---
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- Options ---
# Parity builds MUST keep these OFF: -ffast-math / -march=native destroy bit-parity
# for non-physical reasons (see docs/design/DESIGN_BASIS.md §6). They exist only so a
# separate perf build can opt in; never enable them on a parity/CI configuration.
option(ORVD_FAST_MATH       "Enable -ffast-math (BREAKS parity — perf builds only)" OFF)
option(ORVD_MARCH_NATIVE    "Enable -march=native (BREAKS parity — perf builds only)" OFF)
option(ORVD_BUILD_TESTS     "Build unit + parity tests" ON)
option(ORVD_BUILD_APPS      "Build executables under apps/" ON)
# During M0/M1 the Drake differential oracle is still linkable for parity tests.
# After C1 (M2), production targets must not link Drake; only tests/parity may.
option(ORVD_ENABLE_DRAKE_ORACLE "Link Drake in tests/parity as the differential oracle" OFF)

# --- External dependencies (cross-platform, permissive; brought in per-module) ---
#   Eigen3            REQUIRED  (linear-algebra substrate)
#   SUNDIALS CVODE    REQUIRED  (integrator; pin 7.7.0 / double / int32)
#   Ceres             REQUIRED  (static equilibrium; pin dense backend)
# find_package() calls live in the modules that need them, not here, so the
# skeleton configures without them present.

# --- Module graph (wired in as each is implemented) ---
# Order reflects dependency direction: external -> core -> {systems, forces} -> integrators
#   -> vehicles -> equilibrium -> apps ; tests last.
foreach(_dir
        external/drake_mbtree
        libs/core
        libs/systems
        libs/forces
        libs/integrators
        libs/vehicles
        libs/equilibrium
        apps/gz18_sim)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/CMakeLists.txt")
        add_subdirectory(${_dir})
    endif()
endforeach()

if(ORVD_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    enable_testing()
    add_subdirectory(tests)
endif()

message(STATUS "ORVD ${PROJECT_VERSION} — pre-implementation skeleton; C++${CMAKE_CXX_STANDARD}")
