cmake_minimum_required(VERSION 3.24)

set(_orvd_product_source
    "${CMAKE_CURRENT_LIST_DIR}/OpenRailVehicleDynamics")
set(_orvd_platform_toolchain_module
    "${_orvd_product_source}/cmake/OrvdPlatformToolchainBootstrap.cmake")
if(NOT EXISTS "${_orvd_platform_toolchain_module}")
    message(FATAL_ERROR
        "the offline source bundle is missing its platform toolchain module")
endif()
include("${_orvd_platform_toolchain_module}")
orvd_initialize_native_macos_toolchain()

project(ORVDSourceBundle LANGUAGES NONE)

orvd_require_supported_macos_target()

include(ExternalProject)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install"
        CACHE PATH "Installation prefix for the complete ORVD source bundle" FORCE)
endif()

set(_orvd_bundle_root "${CMAKE_CURRENT_LIST_DIR}")
set(_orvd_archives "${_orvd_bundle_root}/dependencies")
set(_orvd_licenses "${_orvd_bundle_root}/dependency-licenses")
set(_orvd_dependency_source_module
    "${_orvd_bundle_root}/OrvdDependencySources.cmake")
set(_orvd_dependency_source_manifest
    "${_orvd_bundle_root}/dependency_sources.cmake")

foreach(required_path IN ITEMS
        "${_orvd_dependency_source_module}"
        "${_orvd_dependency_source_manifest}"
        "${_orvd_product_source}/CMakeLists.txt"
        "${_orvd_licenses}")
    if(NOT EXISTS "${required_path}")
        message(FATAL_ERROR
            "the offline source bundle is incomplete: '${required_path}' is missing")
    endif()
endforeach()

include("${_orvd_dependency_source_module}")
orvd_begin_dependency_sources()
include("${_orvd_dependency_source_manifest}")
orvd_finalize_dependency_sources()

foreach(key IN LISTS ORVD_DEPENDENCY_KEYS)
    string(TOUPPER "${key}" upper_key)
    set(archive "${ORVD_${upper_key}_ARCHIVE}")
    set("ORVD_${upper_key}_ARCHIVE" "${_orvd_archives}/${archive}")
    if(NOT EXISTS "${_orvd_archives}/${archive}")
        message(FATAL_ERROR
            "the offline source bundle is missing dependency archive "
            "'${_orvd_archives}/${archive}'")
    endif()
endforeach()

# Every nested project uses the same toolchain and installs into this bundle's
# isolated prefix. Package locations passed to ORVD below are exact, so the
# normal product find_package calls remain the only dependency-consumption path.
set(_orvd_common_cache_args
    "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
    "-DCMAKE_INSTALL_LIBDIR:STRING=lib"
    "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON")

foreach(variable IN ITEMS
        CMAKE_BUILD_TYPE
        CMAKE_TOOLCHAIN_FILE
        CMAKE_MSVC_RUNTIME_LIBRARY
        CMAKE_OSX_ARCHITECTURES
        CMAKE_OSX_DEPLOYMENT_TARGET
        CMAKE_OSX_SYSROOT)
    if(DEFINED ${variable} AND NOT "${${variable}}" STREQUAL "")
        list(APPEND _orvd_common_cache_args
            "-D${variable}:STRING=${${variable}}")
    endif()
endforeach()

set(_orvd_c_cache_args ${_orvd_common_cache_args})
if(DEFINED CMAKE_C_COMPILER AND NOT "${CMAKE_C_COMPILER}" STREQUAL "")
    list(APPEND _orvd_c_cache_args
        "-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}")
endif()

set(_orvd_cxx_cache_args ${_orvd_common_cache_args})
if(DEFINED CMAKE_CXX_COMPILER AND NOT "${CMAKE_CXX_COMPILER}" STREQUAL "")
    list(APPEND _orvd_cxx_cache_args
        "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}")
endif()

# The caller may provide product-only package prefixes, for example a custom
# toolchain or a non-default LLVM installation. The four bundled dependencies
# must remain isolated from unrelated packages installed in those prefixes.
# Native AppleClang resolves Homebrew libomp inside the nested product itself.
set(_orvd_product_cache_args ${_orvd_cxx_cache_args})
if(DEFINED CMAKE_PREFIX_PATH AND NOT "${CMAKE_PREFIX_PATH}" STREQUAL "")
    string(REPLACE ";" "|" _orvd_product_prefix_path
           "${CMAKE_PREFIX_PATH}")
    list(APPEND _orvd_product_cache_args
        "-DCMAKE_PREFIX_PATH:STRING=${_orvd_product_prefix_path}")
endif()

ExternalProject_Add(orvd_dependency_eigen
    URL "${ORVD_EIGEN_ARCHIVE}"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CMAKE_ARGS
        ${_orvd_cxx_cache_args}
        -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY:BOOL=ON
        -DBUILD_TESTING:BOOL=OFF
        -DEIGEN_BUILD_DOC:BOOL=OFF
        -DEIGEN_BUILD_PKGCONFIG:BOOL=OFF)

ExternalProject_Add(orvd_dependency_fmt
    URL "${ORVD_FMT_ARCHIVE}"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CMAKE_ARGS
        ${_orvd_cxx_cache_args}
        -DBUILD_SHARED_LIBS:BOOL=OFF
        -DFMT_INSTALL:BOOL=ON
        -DFMT_DOC:BOOL=OFF
        -DFMT_TEST:BOOL=OFF
        -DFMT_FUZZ:BOOL=OFF)

ExternalProject_Add(orvd_dependency_nlohmann_json
    URL "${ORVD_NLOHMANN_JSON_ARCHIVE}"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CMAKE_ARGS
        ${_orvd_cxx_cache_args}
        -DJSON_BuildTests:BOOL=OFF
        -DJSON_Install:BOOL=ON
        -DJSON_MultipleHeaders:BOOL=ON)

ExternalProject_Add(orvd_dependency_sundials
    URL "${ORVD_SUNDIALS_ARCHIVE}"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CMAKE_ARGS
        ${_orvd_c_cache_args}
        -DBUILD_STATIC_LIBS:BOOL=ON
        -DBUILD_SHARED_LIBS:BOOL=OFF
        -DBUILD_TESTING:BOOL=OFF
        -DSUNDIALS_PRECISION:STRING=DOUBLE
        -DSUNDIALS_INDEX_SIZE:STRING=32
        -DSUNDIALS_ENABLE_ARKODE:BOOL=OFF
        -DSUNDIALS_ENABLE_CVODE:BOOL=ON
        -DSUNDIALS_ENABLE_CVODES:BOOL=OFF
        -DSUNDIALS_ENABLE_IDA:BOOL=OFF
        -DSUNDIALS_ENABLE_IDAS:BOOL=OFF
        -DSUNDIALS_ENABLE_KINSOL:BOOL=OFF
        -DSUNDIALS_ENABLE_FORTRAN:BOOL=OFF
        -DSUNDIALS_ENABLE_C_EXAMPLES:BOOL=OFF
        -DSUNDIALS_ENABLE_CXX_EXAMPLES:BOOL=OFF
        -DSUNDIALS_ENABLE_EXAMPLES_INSTALL:BOOL=OFF
        -DSUNDIALS_ENABLE_NVECTOR_MANYVECTOR:BOOL=OFF
        -DSUNDIALS_ENABLE_MPI:BOOL=OFF
        -DSUNDIALS_ENABLE_OPENMP:BOOL=OFF
        -DSUNDIALS_ENABLE_OPENMP_DEVICE:BOOL=OFF
        -DSUNDIALS_ENABLE_PTHREAD:BOOL=OFF
        -DSUNDIALS_ENABLE_CUDA:BOOL=OFF
        -DSUNDIALS_ENABLE_HIP:BOOL=OFF
        -DSUNDIALS_ENABLE_SYCL:BOOL=OFF
        -DSUNDIALS_ENABLE_LAPACK:BOOL=OFF
        -DSUNDIALS_ENABLE_KLU:BOOL=OFF
        -DSUNDIALS_LOGGING_LEVEL:STRING=0)

set(_orvd_license_destination
    "${CMAKE_INSTALL_PREFIX}/share/licenses/orvd-dependencies")
add_custom_command(
    OUTPUT "${CMAKE_BINARY_DIR}/dependency-licenses.stamp"
    COMMAND "${CMAKE_COMMAND}" -E make_directory
            "${_orvd_license_destination}"
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
            "${_orvd_licenses}" "${_orvd_license_destination}"
    COMMAND "${CMAKE_COMMAND}" -E touch
            "${CMAKE_BINARY_DIR}/dependency-licenses.stamp"
    DEPENDS "${_orvd_dependency_source_manifest}"
    COMMENT "Installing bundled dependency licence materials"
    VERBATIM)
add_custom_target(orvd_dependency_licenses ALL
    DEPENDS "${CMAKE_BINARY_DIR}/dependency-licenses.stamp")

ExternalProject_Add(orvd_product
    LIST_SEPARATOR "|"
    SOURCE_DIR "${_orvd_product_source}"
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    DEPENDS
        orvd_dependency_eigen
        orvd_dependency_fmt
        orvd_dependency_nlohmann_json
        orvd_dependency_sundials
        orvd_dependency_licenses
    CMAKE_ARGS
        ${_orvd_product_cache_args}
        -DBUILD_TESTING:BOOL=OFF
        -DORVD_BUILD_DRAKE_REFERENCE_TESTS:BOOL=OFF
        "-DEigen3_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake"
        "-Dfmt_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/fmt"
        "-Dnlohmann_json_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/cmake/nlohmann_json"
        "-DSUNDIALS_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/sundials")

message(STATUS
    "ORVD offline source bundle will install into '${CMAKE_INSTALL_PREFIX}'")
