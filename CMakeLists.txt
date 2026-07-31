# OpenRailVehicleDynamics (ORVD) — top-level build
#
# Vendored common support, rigid topology, double pose math and the complete
# double-only rigid tree are buildable. The first-party runtime owns multibody
# state, versions and the typed cache workspace used by that internal tree
# target. BUILD_TESTING adds model-neutral self-checks. Product modules are wired
# in only when they gain a CMakeLists.txt; their order is set by the
# implementation roadmap.
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

# Imported targets from find_package are directory-scoped by default, which would
# leave the product boundary gate below unable to see a dependency that was
# perfectly visible where it was linked. A boundary check that can only inspect
# part of the graph reports a boundary it has not actually checked.
set(CMAKE_FIND_PACKAGE_TARGETS_GLOBAL ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(OrvdFirstPartyTargetPolicy)
include(OrvdProductBoundaryGate)

# --- External dependencies ---
# Eigen is required outright: every multibody pass is Eigen arithmetic, so a
# build without it cannot produce anything useful and should fail at configure
# time rather than at the first compile error. CMake treats include directories
# from the imported Eigen3::Eigen target as SYSTEM by default, which keeps its
# diagnostics out of first-party output.
# SUNDIALS CVODE and Ceres have no consumer yet and are therefore not searched
# for, not made optional, and given no target.
find_package(Eigen3 3.4 CONFIG REQUIRED)

# fmt is required by the vendored Drake source: its public headers include
# fmt/format.h and specialise fmt::formatter. No version is pinned: real source
# compilation verifies the configured package, while a minimum version would
# require evidence from a specific API.
#
# PATH-derived prefixes are excluded for this search. An unrelated toolchain on
# PATH — a conda environment, say — otherwise supplies its own fmt, whose
# directory lands in every consumer's RUNPATH and takes the C++ runtime with it.
# The result is an executable that links cleanly and cannot start, which is worse
# than a build failure because it survives every check that stops at linking.
find_package(fmt CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH)

# --- Module graph, wired in as each module is implemented ---
# Dependency direction: external -> multibody_runtime
#   -> {system_assembly, forces}
#   -> integrators
#   -> equilibrium ; tests last.
# These listed directories are the product module roots. Every non-imported
# target defined in one of them or a descendant is subject to the Drake boundary
# gate below. A new target inside a listed module needs no registration; a new
# top-level product module must be added to this list.
# tests/ is deliberately absent: the Drake-backed reference targets live there and
# are supposed to link Drake.
set(ORVD_PRODUCT_MODULE_DIRECTORIES
    external/drake_mbtree
    libs/multibody_runtime
    libs/multibody_model
    libs/system_assembly
    libs/forces
    libs/integrators
    libs/equilibrium)
foreach(product_module_relative_path IN LISTS ORVD_PRODUCT_MODULE_DIRECTORIES)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${product_module_relative_path}/CMakeLists.txt")
        add_subdirectory(${product_module_relative_path})
    endif()
endforeach()

# include(CTest) defines BUILD_TESTING (default ON) and configures CTest.
include(CTest)
option(ORVD_BUILD_DRAKE_REFERENCE_TESTS
       "Build the Drake-backed reference tests" OFF)
if(ORVD_BUILD_DRAKE_REFERENCE_TESTS AND NOT BUILD_TESTING)
    message(FATAL_ERROR
        "ORVD_BUILD_DRAKE_REFERENCE_TESTS requires BUILD_TESTING=ON")
endif()
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# After every add_subdirectory(), including the test tree, so that a target
# defined in a later file is still seen. Not a proof that none can escape:
# cmake_language(DEFER) can change the graph after this has run, and a
# directory-scoped imported target is invisible from here. What carries a
# guarantee is the built-artefact check in tests/orvd_candidate/. Unconditional,
# and in
# particular independent of ORVD_BUILD_DRAKE_REFERENCE_TESTS — that option is
# exactly when Drake is in the graph, which is exactly when this must be checked.
orvd_verify_product_targets_have_no_drake_dependency(
    ${ORVD_PRODUCT_MODULE_DIRECTORIES})


message(STATUS
    "ORVD ${PROJECT_VERSION} — product targets configured")
