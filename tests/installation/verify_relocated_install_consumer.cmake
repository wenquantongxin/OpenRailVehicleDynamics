cmake_minimum_required(VERSION 3.24)

foreach(required_variable
        ORVD_BUILD_DIR
        ORVD_SOURCE_DIR
        INSTALLATION_SOURCE_DIR
        WORK_ROOT
        CMAKE_GENERATOR_NAME
        CMAKE_CXX_COMPILER_PATH
        BUILD_CONFIG
        EXECUTABLE_SUFFIX
        INSTALL_DATADIR)
    if(NOT DEFINED ${required_variable})
        message(FATAL_ERROR "${required_variable} was not given")
    endif()
endforeach()

function(run_checked label)
    execute_process(
        COMMAND ${ARGN}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE errors)
    if(NOT "${result}" STREQUAL "0")
        message(FATAL_ERROR
            "${label} failed with exit code ${result}:\n${output}${errors}")
    endif()
endfunction()

function(find_single_executable output_variable basename search_root)
    file(GLOB_RECURSE candidates LIST_DIRECTORIES FALSE
         "${search_root}/${basename}${EXECUTABLE_SUFFIX}")
    list(LENGTH candidates candidate_count)
    if(NOT candidate_count EQUAL 1)
        message(FATAL_ERROR
            "expected one '${basename}${EXECUTABLE_SUFFIX}' below "
            "'${search_root}', found ${candidate_count}: ${candidates}")
    endif()
    list(GET candidates 0 executable)
    set(${output_variable} "${executable}" PARENT_SCOPE)
endfunction()

if("${WORK_ROOT}" STREQUAL "${ORVD_BUILD_DIR}" OR
   "${WORK_ROOT}" STREQUAL "${ORVD_SOURCE_DIR}")
    message(FATAL_ERROR "WORK_ROOT must be dedicated installation-test storage")
endif()

file(REMOVE_RECURSE "${WORK_ROOT}")
file(MAKE_DIRECTORY "${WORK_ROOT}")
set(original_prefix "${WORK_ROOT}/stage-original")
set(relocated_prefix "${WORK_ROOT}/stage-relocated")
set(consumer_source "${WORK_ROOT}/consumer-source")
set(consumer_build "${WORK_ROOT}/consumer-build")

run_checked(
    "installing ORVD"
    "${CMAKE_COMMAND}" --install "${ORVD_BUILD_DIR}"
    --prefix "${original_prefix}" --config "${BUILD_CONFIG}")
if(NOT EXISTS "${original_prefix}")
    message(FATAL_ERROR "ORVD installation produced no prefix")
endif()
file(RENAME "${original_prefix}" "${relocated_prefix}")

set(installed_track_geometry_directory
    "${relocated_prefix}/${INSTALL_DATADIR}/OpenRailVehicleDynamics/track_library/geometries")
set(installed_track_geometry
    "${installed_track_geometry_directory}/straight_level_2000m.json")
if(NOT EXISTS "${installed_track_geometry}")
    message(FATAL_ERROR
        "the relocated prefix has no installed track geometry record")
endif()

set(installed_vehicle_definition
    "${relocated_prefix}/${INSTALL_DATADIR}/OpenRailVehicleDynamics/vehicle_library/gz18/vehicle_definition.json")
if(NOT EXISTS "${installed_vehicle_definition}")
    message(FATAL_ERROR
        "the relocated prefix has no installed vehicle definition record")
endif()

# The start-up state sits in a subdirectory of the vehicle library, so it is
# carried by the same recursive install rule. The consumer below loads it for
# real rather than this script only asserting the file arrived.
set(installed_startup_state
    "${relocated_prefix}/${INSTALL_DATADIR}/OpenRailVehicleDynamics/vehicle_library/gz18/startup_states/moving_startup_60kmh.json")
set(installed_data_root
    "${relocated_prefix}/${INSTALL_DATADIR}/OpenRailVehicleDynamics")
set(installed_wheel_profile
    "${installed_data_root}/vehicle_library/gz18/wheel_profiles/gz18_reference_wheel_profile.json")
set(installed_rail_profile
    "${installed_data_root}/track_library/rail_profiles/uic60_rail_profile.json")
set(installed_track_irregularity
    "${installed_data_root}/track_library/irregularities/gz18_aar6_reference_irregularity.json")
set(installed_lateral_irregularity
    "${installed_data_root}/track_library/irregularities/series/gz18_aar6_reference_lateral.json")
set(installed_vertical_irregularity
    "${installed_data_root}/track_library/irregularities/series/gz18_aar6_reference_vertical.json")
foreach(installed_profile IN ITEMS
        "${installed_wheel_profile}" "${installed_rail_profile}")
    if(NOT EXISTS "${installed_profile}")
        message(FATAL_ERROR
            "the relocated prefix has no installed profile '${installed_profile}'")
    endif()
endforeach()
foreach(installed_irregularity IN ITEMS
        "${installed_track_irregularity}"
        "${installed_lateral_irregularity}"
        "${installed_vertical_irregularity}")
    if(NOT EXISTS "${installed_irregularity}")
        message(FATAL_ERROR
            "the relocated prefix has no installed track-irregularity asset "
            "'${installed_irregularity}'")
    endif()
endforeach()

file(GLOB_RECURSE package_configs LIST_DIRECTORIES FALSE
     "${relocated_prefix}/*OpenRailVehicleDynamicsConfig.cmake")
list(LENGTH package_configs package_config_count)
if(NOT package_config_count EQUAL 1)
    message(FATAL_ERROR
        "the relocated prefix contains ${package_config_count} package config "
        "files instead of one: ${package_configs}")
endif()
list(GET package_configs 0 installed_package_config)
file(READ "${installed_package_config}" installed_package_config_text)
string(FIND "${installed_package_config_text}"
       "find_dependency(fmt 9.1.0 CONFIG NO_SYSTEM_ENVIRONMENT_PATH)"
       isolated_fmt_dependency_position)
if(isolated_fmt_dependency_position EQUAL -1)
    message(FATAL_ERROR
        "the installed package does not isolate fmt lookup from PATH-provided "
        "toolchains")
endif()
string(FIND "${installed_package_config_text}"
       "find_dependency(OpenMP COMPONENTS CXX)"
       openmp_dependency_position)
if(openmp_dependency_position EQUAL -1)
    message(FATAL_ERROR
        "the installed package does not restore its OpenMP dependency")
endif()

# Relocation is a property of installed CMake metadata, not of debug strings in
# compiled archives.  Inspect only the files a future find_package() reads.
file(GLOB_RECURSE installed_cmake_files LIST_DIRECTORIES FALSE
     "${relocated_prefix}/*.cmake")
foreach(installed_cmake_file IN LISTS installed_cmake_files)
    file(READ "${installed_cmake_file}" installed_cmake_text)
    foreach(forbidden_text
            "${original_prefix}"
            "${ORVD_SOURCE_DIR}"
            "/opt/")
        if(NOT forbidden_text STREQUAL "")
            string(FIND "${installed_cmake_text}" "${forbidden_text}"
                   forbidden_position)
            if(NOT forbidden_position EQUAL -1)
                message(FATAL_ERROR
                    "installed package metadata '${installed_cmake_file}' "
                    "contains the forbidden path '${forbidden_text}'")
            endif()
        endif()
    endforeach()
endforeach()

file(COPY "${INSTALLATION_SOURCE_DIR}/consumer/"
     DESTINATION "${consumer_source}")

set(prefix_path "${relocated_prefix}")
if(DEFINED DEPENDENCY_PREFIX_PATH AND NOT DEPENDENCY_PREFIX_PATH STREQUAL "")
    list(APPEND prefix_path ${DEPENDENCY_PREFIX_PATH})
endif()
list(JOIN prefix_path ";" prefix_path_joined)
string(REPLACE ";" "\\;" prefix_path_argument "${prefix_path_joined}")

set(configure_command
    "${CMAKE_COMMAND}"
    -S "${consumer_source}"
    -B "${consumer_build}"
    -G "${CMAKE_GENERATOR_NAME}"
    "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER_PATH}"
    "-DCMAKE_PREFIX_PATH=${prefix_path_argument}"
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF
    -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF)
if(DEFINED CMAKE_GENERATOR_PLATFORM_NAME AND
   NOT CMAKE_GENERATOR_PLATFORM_NAME STREQUAL "")
    list(APPEND configure_command -A "${CMAKE_GENERATOR_PLATFORM_NAME}")
endif()
if(DEFINED CMAKE_GENERATOR_TOOLSET_NAME AND
   NOT CMAKE_GENERATOR_TOOLSET_NAME STREQUAL "")
    list(APPEND configure_command -T "${CMAKE_GENERATOR_TOOLSET_NAME}")
endif()
foreach(package_hint Eigen3 fmt SUNDIALS)
    string(TOUPPER "${package_hint}" package_hint_upper)
    set(package_hint_variable "${package_hint_upper}_CONFIG_DIR")
    if(DEFINED ${package_hint_variable} AND
       NOT "${${package_hint_variable}}" STREQUAL "")
        list(APPEND configure_command
             "-D${package_hint}_DIR=${${package_hint_variable}}")
    endif()
endforeach()
if(DEFINED CMAKE_MAKE_PROGRAM_PATH AND
   NOT CMAKE_MAKE_PROGRAM_PATH STREQUAL "")
    list(APPEND configure_command
         "-DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM_PATH}")
endif()
if(NOT DEFINED MULTI_CONFIG OR NOT MULTI_CONFIG)
    list(APPEND configure_command "-DCMAKE_BUILD_TYPE=${BUILD_CONFIG}")
endif()
run_checked("configuring the relocated independent consumer"
            ${configure_command})
run_checked(
    "building the relocated independent consumer"
    "${CMAKE_COMMAND}" --build "${consumer_build}" --config "${BUILD_CONFIG}")

find_single_executable(smoke_executable orvd_installation_smoke
                       "${consumer_build}")
find_single_executable(configuration_smoke_executable
                       configuration_installation_smoke "${consumer_build}")
find_single_executable(track_geometry_smoke_executable
                       track_geometry_installation_smoke "${consumer_build}")
find_single_executable(wheel_rail_contact_smoke_executable
                       wheel_rail_contact_installation_smoke "${consumer_build}")
find_single_executable(drake_probe_executable drake_loading_probe
                       "${consumer_build}")
get_filename_component(smoke_runtime_directory "${smoke_executable}" DIRECTORY)
get_filename_component(drake_probe_runtime_directory
                       "${drake_probe_executable}" DIRECTORY)

run_checked("running the relocated independent consumer" "${smoke_executable}")
run_checked("running the configuration-only installed consumer"
            "${configuration_smoke_executable}" "${installed_track_geometry}"
            "${installed_vehicle_definition}" "${installed_startup_state}"
            "${installed_data_root}")
run_checked("running the track-geometry-only installed consumer"
            "${track_geometry_smoke_executable}")
run_checked("running the wheel-rail-contact-only installed consumer"
            "${wheel_rail_contact_smoke_executable}")
run_checked("running the Drake-marker positive control"
            "${drake_probe_executable}")
run_checked(
    "checking the installed consumer runtime dependency closure"
    "${CMAKE_COMMAND}"
    "-DCHECKED_EXECUTABLE=${smoke_executable}"
    "-DRUNTIME_SEARCH_DIRECTORIES=${smoke_runtime_directory}"
    -P "${INSTALLATION_SOURCE_DIR}/verify_runtime_dependencies.cmake")

execute_process(
    COMMAND "${CMAKE_COMMAND}"
            "-DCHECKED_EXECUTABLE=${drake_probe_executable}"
            "-DRUNTIME_SEARCH_DIRECTORIES=${drake_probe_runtime_directory}"
            -P "${INSTALLATION_SOURCE_DIR}/verify_runtime_dependencies.cmake"
    RESULT_VARIABLE positive_control_result
    OUTPUT_VARIABLE positive_control_output
    ERROR_VARIABLE positive_control_errors)
if("${positive_control_result}" STREQUAL "0")
    message(FATAL_ERROR
        "the runtime dependency checker accepted a running executable that "
        "loads a Drake-identified shared library")
endif()
if(NOT "${positive_control_output}${positive_control_errors}" MATCHES
       "runtime dependency closure contains shared Drake")
    message(FATAL_ERROR
        "the Drake-marker positive control failed for the wrong reason:\n"
        "${positive_control_output}${positive_control_errors}")
endif()

message(STATUS
    "relocated find_package consumer, real CVODE smoke and runtime dependency "
    "discrimination all passed")
