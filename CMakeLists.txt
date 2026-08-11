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
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(OrvdFirstPartyTargetPolicy)
include(OrvdProductBoundaryGate)

# --- External dependencies ---
# Eigen is required outright: every multibody pass is Eigen arithmetic, so a
# build without it cannot produce anything useful and should fail at configure
# time rather than at the first compile error. CMake treats include directories
# from the imported Eigen3::Eigen target as SYSTEM by default, which keeps its
# diagnostics out of first-party output.
# G44 is the first SUNDIALS consumer. The dependency is exact and required:
# silently omitting the only admitted numerical backend would leave a build
# that advertises an advancer it cannot construct. Its installation prefix is
# supplied by the build environment and is never recorded in the source tree.
# Ceres still has no consumer and is therefore not searched or given a target.
find_package(Eigen3 3.4 CONFIG REQUIRED)
find_package(SUNDIALS 7.7.0 EXACT CONFIG REQUIRED
    COMPONENTS cvode nvecserial sunmatrixdense sunlinsoldense core)
# G59 admits deterministic parallel evaluation of the eight independent GZ18
# wheel-rail interfaces. OpenMP is a required product dependency: silently
# compiling a different serial product would make the numerical execution
# identity depend on an accidental configure-time discovery.
find_package(OpenMP REQUIRED COMPONENTS CXX)

# fmt is required by the vendored Drake source: its public headers include
# fmt/format.h and specialise fmt::formatter. G46 pins 9.1.0 to the version
# carried by the offline source bundle, so development and installed-package
# modes consume the same declared interface.
#
# PATH-derived prefixes are excluded for this search. An unrelated toolchain on
# PATH — a conda environment, say — otherwise supplies its own fmt, whose
# directory lands in every consumer's RUNPATH and takes the C++ runtime with it.
# The result is an executable that links cleanly and cannot start, which is worse
# than a build failure because it survives every check that stops at linking.
find_package(fmt 9.1.0 CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH)
find_package(nlohmann_json 3.12.0 EXACT CONFIG REQUIRED
    NO_SYSTEM_ENVIRONMENT_PATH)

# --- Module graph, wired in as each module is implemented ---
# Dependency direction: multibody_runtime + internal rigid tree
#   -> multibody_model -> forces -> system_assembly -> integrators.
# Control owns state-explicit sampled algorithms, while actuation owns
# state-explicit command-conditioning laws. Both are independent of vehicle
# topology; configuration is their first typed asset consumer.
# Track geometry and the wheel-rail contact core also feed the typed force
# plans; configuration consumes those lower layers and equilibrium/tests come
# last.
# These listed directories are the product module roots. Every non-imported
# target defined in one of them or a descendant is subject to the Drake boundary
# gate below. A new target inside a listed module needs no registration; a new
# top-level product module must be added to this list.
# tests/ is deliberately absent: the Drake-backed reference targets live there and
# are supposed to link Drake.
set(ORVD_PRODUCT_MODULE_DIRECTORIES
    external/drake_mbtree
    libs/control
    libs/actuation
    libs/track_geometry
    libs/wheel_rail_contact
    libs/multibody_runtime
    libs/multibody_model
    libs/forces
    libs/system_assembly
    libs/integrators
    libs/configuration
    libs/equilibrium)
foreach(product_module_relative_path IN LISTS ORVD_PRODUCT_MODULE_DIRECTORIES)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${product_module_relative_path}/CMakeLists.txt")
        add_subdirectory(${product_module_relative_path})
    endif()
endforeach()

# Development-time tooling. Not a product module: nothing here is installed or
# exported, and no product target links it, so it is deliberately absent from
# the list above and from the boundary gate that list drives.
add_subdirectory(tools/profile_conversion)
add_subdirectory(tools/dynamics_qualification)

# --- Installable package ---
# Eleven first-party targets form the documented surface. The rigid-tree archive
# is exported only because a static modelling archive must retain its private
# implementation link edge; no vendored or adapter header is installed.
set(ORVD_INSTALL_TARGETS
    orvd_control
    orvd_actuation
    orvd_track_geometry
    orvd_wheel_rail_contact
    orvd_configuration
    orvd_multibody_runtime
    orvd_multibody_model
    orvd_forces
    orvd_system_assembly
    orvd_integrators
    orvd_rigid_multibody_tree)
install(TARGETS ${ORVD_INSTALL_TARGETS}
        EXPORT OpenRailVehicleDynamicsTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

foreach(orvd_public_module
        control actuation track_geometry wheel_rail_contact configuration
        multibody_runtime multibody_model forces system_assembly integrators)
    install(DIRECTORY
            "${CMAKE_CURRENT_SOURCE_DIR}/libs/${orvd_public_module}/include/"
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
            FILES_MATCHING PATTERN "*.h")
endforeach()

install(DIRECTORY
        "${CMAKE_CURRENT_SOURCE_DIR}/track_library/geometries/"
        DESTINATION
        "${CMAKE_INSTALL_DATADIR}/OpenRailVehicleDynamics/track_library/geometries"
        FILES_MATCHING PATTERN "*.json")

install(DIRECTORY
        "${CMAKE_CURRENT_SOURCE_DIR}/track_library/rail_profiles/"
        DESTINATION
        "${CMAKE_INSTALL_DATADIR}/OpenRailVehicleDynamics/track_library/rail_profiles"
        FILES_MATCHING PATTERN "*.json")

install(DIRECTORY
        "${CMAKE_CURRENT_SOURCE_DIR}/track_library/irregularities/"
        DESTINATION
        "${CMAKE_INSTALL_DATADIR}/OpenRailVehicleDynamics/track_library/irregularities"
        FILES_MATCHING PATTERN "*.json")

install(DIRECTORY
        "${CMAKE_CURRENT_SOURCE_DIR}/vehicle_library/"
        DESTINATION
        "${CMAKE_INSTALL_DATADIR}/OpenRailVehicleDynamics/vehicle_library"
        FILES_MATCHING PATTERN "*.json")

install(DIRECTORY
        "${CMAKE_CURRENT_SOURCE_DIR}/controller_library/"
        DESTINATION
        "${CMAKE_INSTALL_DATADIR}/OpenRailVehicleDynamics/controller_library"
        FILES_MATCHING PATTERN "*.json")

set(ORVD_INSTALL_CMAKEDIR
    "${CMAKE_INSTALL_LIBDIR}/cmake/OpenRailVehicleDynamics")
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/OpenRailVehicleDynamicsConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/OpenRailVehicleDynamicsConfig.cmake"
    INSTALL_DESTINATION "${ORVD_INSTALL_CMAKEDIR}")
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/OpenRailVehicleDynamicsConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY ExactVersion)
install(EXPORT OpenRailVehicleDynamicsTargets
        FILE OpenRailVehicleDynamicsTargets.cmake
        NAMESPACE ORVD::
        DESTINATION "${ORVD_INSTALL_CMAKEDIR}")
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/OpenRailVehicleDynamicsConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenRailVehicleDynamicsConfigVersion.cmake"
        DESTINATION "${ORVD_INSTALL_CMAKEDIR}")
install(FILES
        NOTICE
        external/drake_mbtree/LICENSE.TXT
        external/drake_mbtree/LICENSE.Apache-2.0.txt
        external/drake_mbtree/DRAKE_SOURCE_MODIFICATIONS.md
        DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/OpenRailVehicleDynamics")

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
