cmake_minimum_required(VERSION 3.24)

project(orvd_irw_crossline_full_state_guidance_experiment LANGUAGES CXX)

# This experiment is deliberately an installed-package consumer.  It is not
# entered by the ORVD root build, installed with ORVD, or registered in CTest.
find_package(OpenRailVehicleDynamics CONFIG REQUIRED)

add_library(irw_guidance_control_transaction STATIC
    irw_guidance_control_transaction.cc)
target_compile_features(irw_guidance_control_transaction PUBLIC cxx_std_23)
set_target_properties(
    irw_guidance_control_transaction PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(
    irw_guidance_control_transaction PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(
    irw_guidance_control_transaction PUBLIC ORVD::configuration)

add_library(irw_guidance_operating_point_schedules STATIC
    irw_crossline_operating_point_schedule.cc
    irw_single_curve_full_state_guidance_schedule.cc
    irw_single_curve_normal_differential_wheel_speed_schedule.cc)

target_compile_features(
    irw_guidance_operating_point_schedules PUBLIC cxx_std_23)
set_target_properties(
    irw_guidance_operating_point_schedules PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(
    irw_guidance_operating_point_schedules
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(
    irw_guidance_operating_point_schedules PUBLIC ORVD::control)

add_library(irw_guidance_control_event_session STATIC
    irw_guidance_control_event_session.cc)
target_compile_features(
    irw_guidance_control_event_session PUBLIC cxx_std_23)
set_target_properties(
    irw_guidance_control_event_session PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(
    irw_guidance_control_event_session
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(
    irw_guidance_control_event_session
    PUBLIC irw_guidance_control_transaction
           irw_guidance_operating_point_schedules
           ORVD::configuration)

add_library(irw_guidance_experiment_run STATIC
    irw_guidance_experiment_run.cc)
target_compile_features(irw_guidance_experiment_run PUBLIC cxx_std_23)
set_target_properties(
    irw_guidance_experiment_run PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(
    irw_guidance_experiment_run PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(
    irw_guidance_experiment_run
    PUBLIC irw_guidance_control_event_session
           ORVD::configuration ORVD::integrators)

add_executable(
    orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
    run_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance.cc)
target_compile_features(
    orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
    PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
    PRIVATE irw_guidance_experiment_run)

add_executable(
    orvd_irw_r300_aar5_v60_normal_differential_wheel_speed
    run_irw_r300_aar5_v60_normal_differential_wheel_speed.cc)
target_compile_features(
    orvd_irw_r300_aar5_v60_normal_differential_wheel_speed PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_r300_aar5_v60_normal_differential_wheel_speed
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_r300_aar5_v60_normal_differential_wheel_speed
    PRIVATE irw_guidance_experiment_run)

add_executable(
    orvd_irw_r600_aar5_v80_normal_differential_wheel_speed
    run_irw_r600_aar5_v80_normal_differential_wheel_speed.cc)
target_compile_features(
    orvd_irw_r600_aar5_v80_normal_differential_wheel_speed PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_r600_aar5_v80_normal_differential_wheel_speed
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_r600_aar5_v80_normal_differential_wheel_speed
    PRIVATE irw_guidance_experiment_run)

add_executable(
    orvd_irw_r800_aar5_v100_normal_differential_wheel_speed
    run_irw_r800_aar5_v100_normal_differential_wheel_speed.cc)
target_compile_features(
    orvd_irw_r800_aar5_v100_normal_differential_wheel_speed PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_r800_aar5_v100_normal_differential_wheel_speed
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_r800_aar5_v100_normal_differential_wheel_speed
    PRIVATE irw_guidance_experiment_run)

add_executable(
    orvd_irw_r300_aar5_v60_better2_full_state_guidance
    run_irw_r300_aar5_v60_better2_full_state_guidance.cc)
target_compile_features(
    orvd_irw_r300_aar5_v60_better2_full_state_guidance PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_r300_aar5_v60_better2_full_state_guidance
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_r300_aar5_v60_better2_full_state_guidance
    PRIVATE irw_guidance_experiment_run)

add_executable(
    orvd_irw_r600_aar5_v80_better2_full_state_guidance
    run_irw_r600_aar5_v80_better2_full_state_guidance.cc)
target_compile_features(
    orvd_irw_r600_aar5_v80_better2_full_state_guidance PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_r600_aar5_v80_better2_full_state_guidance
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_r600_aar5_v80_better2_full_state_guidance
    PRIVATE irw_guidance_experiment_run)

add_executable(
    orvd_irw_r800_aar5_v100_better2_full_state_guidance
    run_irw_r800_aar5_v100_better2_full_state_guidance.cc)
target_compile_features(
    orvd_irw_r800_aar5_v100_better2_full_state_guidance PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_r800_aar5_v100_better2_full_state_guidance
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_r800_aar5_v100_better2_full_state_guidance
    PRIVATE irw_guidance_experiment_run)

add_executable(
    orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_full_state_guidance
    run_irw_r300_aar5_v60_scbp_recorded_wear_champion_full_state_guidance.cc)
target_compile_features(
    orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_full_state_guidance
    PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_full_state_guidance
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_full_state_guidance
    PRIVATE irw_guidance_experiment_run)

add_executable(
    orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_full_state_guidance
    run_irw_r600_aar5_v80_scbp_recorded_wear_champion_full_state_guidance.cc)
target_compile_features(
    orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_full_state_guidance
    PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_full_state_guidance
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_full_state_guidance
    PRIVATE irw_guidance_experiment_run)

add_executable(
    orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_full_state_guidance
    run_irw_r800_aar5_v100_scbp_recorded_wear_champion_full_state_guidance.cc)
target_compile_features(
    orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_full_state_guidance
    PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_full_state_guidance
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_full_state_guidance
    PRIVATE irw_guidance_experiment_run)

option(ORVD_ENABLE_SIMPACK_REALTIME_COMPARISON
    "Build the proprietary SIMPACK Realtime comparison arm" OFF)
set(ORVD_SIMPACK_ROOT "/opt/Simpack-2021x" CACHE PATH
    "SIMPACK installation used by the optional Realtime comparison arm")

if(ORVD_ENABLE_SIMPACK_REALTIME_COMPARISON)
    if(NOT UNIX OR APPLE)
        message(FATAL_ERROR
            "The current SIMPACK Realtime direct-call adapter is Linux-only")
    endif()
    find_package(Threads REQUIRED)
    find_path(ORVD_SIMPACK_REALTIME_INCLUDE_DIRECTORY spck_rt.h
        HINTS "${ORVD_SIMPACK_ROOT}/run/realtime"
        NO_DEFAULT_PATH REQUIRED)
    find_library(ORVD_SIMPACK_REALTIME_LIBRARY spck_rt
        HINTS "${ORVD_SIMPACK_ROOT}/run/realtime/linux64"
        NO_DEFAULT_PATH REQUIRED)
    get_filename_component(ORVD_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.."
        ABSOLUTE)
    set(ORVD_SIMPACK_REALTIME_TOOL_DIRECTORY
        "${ORVD_SOURCE_ROOT}/tools/simpack_realtime")

    add_library(irw_simpack_realtime_direct_call STATIC
        "${ORVD_SIMPACK_REALTIME_TOOL_DIRECTORY}/simpack_realtime_instance.cc")
    target_compile_features(irw_simpack_realtime_direct_call PUBLIC cxx_std_23)
    set_target_properties(irw_simpack_realtime_direct_call
        PROPERTIES CXX_EXTENSIONS OFF)
    target_include_directories(irw_simpack_realtime_direct_call
        PUBLIC "${ORVD_SIMPACK_REALTIME_TOOL_DIRECTORY}")
    target_include_directories(irw_simpack_realtime_direct_call SYSTEM
        PRIVATE "${ORVD_SIMPACK_REALTIME_INCLUDE_DIRECTORY}")
    target_compile_definitions(irw_simpack_realtime_direct_call
        PRIVATE SPCK_RT_API=2)
    target_link_libraries(irw_simpack_realtime_direct_call
        PRIVATE "${ORVD_SIMPACK_REALTIME_LIBRARY}" Threads::Threads rt m)

    add_library(irw_single_curve_guidance_simpack_realtime_run STATIC
        irw_single_curve_guidance_simpack_realtime_run.cc)
    target_compile_features(
        irw_single_curve_guidance_simpack_realtime_run PUBLIC cxx_std_23)
    set_target_properties(irw_single_curve_guidance_simpack_realtime_run
        PROPERTIES CXX_EXTENSIONS OFF)
    target_include_directories(irw_single_curve_guidance_simpack_realtime_run
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(irw_single_curve_guidance_simpack_realtime_run
        PUBLIC irw_guidance_control_transaction
               irw_guidance_operating_point_schedules
               irw_simpack_realtime_direct_call
               ORVD::configuration)

    add_executable(orvd_irw_r300_aar5_v60_normal_simpack_realtime
        run_irw_r300_aar5_v60_normal_differential_wheel_speed_simpack_realtime.cc)
    target_compile_features(
        orvd_irw_r300_aar5_v60_normal_simpack_realtime PRIVATE cxx_std_23)
    set_target_properties(orvd_irw_r300_aar5_v60_normal_simpack_realtime
        PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_definitions(
        orvd_irw_r300_aar5_v60_normal_simpack_realtime
        PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
    target_link_libraries(
        orvd_irw_r300_aar5_v60_normal_simpack_realtime
        PRIVATE irw_single_curve_guidance_simpack_realtime_run)

    add_executable(orvd_irw_r600_aar5_v80_normal_simpack_realtime
        run_irw_r600_aar5_v80_normal_differential_wheel_speed_simpack_realtime.cc)
    target_compile_features(
        orvd_irw_r600_aar5_v80_normal_simpack_realtime PRIVATE cxx_std_23)
    set_target_properties(orvd_irw_r600_aar5_v80_normal_simpack_realtime
        PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_definitions(
        orvd_irw_r600_aar5_v80_normal_simpack_realtime
        PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
    target_link_libraries(
        orvd_irw_r600_aar5_v80_normal_simpack_realtime
        PRIVATE irw_single_curve_guidance_simpack_realtime_run)

    add_executable(orvd_irw_r800_aar5_v100_normal_simpack_realtime
        run_irw_r800_aar5_v100_normal_differential_wheel_speed_simpack_realtime.cc)
    target_compile_features(
        orvd_irw_r800_aar5_v100_normal_simpack_realtime PRIVATE cxx_std_23)
    set_target_properties(orvd_irw_r800_aar5_v100_normal_simpack_realtime
        PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_definitions(
        orvd_irw_r800_aar5_v100_normal_simpack_realtime
        PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
    target_link_libraries(
        orvd_irw_r800_aar5_v100_normal_simpack_realtime
        PRIVATE irw_single_curve_guidance_simpack_realtime_run)

    add_executable(orvd_irw_r300_aar5_v60_better2_simpack_realtime
        run_irw_r300_aar5_v60_better2_full_state_guidance_simpack_realtime.cc)
    target_compile_features(
        orvd_irw_r300_aar5_v60_better2_simpack_realtime PRIVATE cxx_std_23)
    set_target_properties(orvd_irw_r300_aar5_v60_better2_simpack_realtime
        PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_definitions(
        orvd_irw_r300_aar5_v60_better2_simpack_realtime
        PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
    target_link_libraries(
        orvd_irw_r300_aar5_v60_better2_simpack_realtime
        PRIVATE irw_single_curve_guidance_simpack_realtime_run)

    add_executable(orvd_irw_r600_aar5_v80_better2_simpack_realtime
        run_irw_r600_aar5_v80_better2_full_state_guidance_simpack_realtime.cc)
    target_compile_features(
        orvd_irw_r600_aar5_v80_better2_simpack_realtime PRIVATE cxx_std_23)
    set_target_properties(orvd_irw_r600_aar5_v80_better2_simpack_realtime
        PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_definitions(
        orvd_irw_r600_aar5_v80_better2_simpack_realtime
        PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
    target_link_libraries(
        orvd_irw_r600_aar5_v80_better2_simpack_realtime
        PRIVATE irw_single_curve_guidance_simpack_realtime_run)

    add_executable(orvd_irw_r800_aar5_v100_better2_simpack_realtime
        run_irw_r800_aar5_v100_better2_full_state_guidance_simpack_realtime.cc)
    target_compile_features(
        orvd_irw_r800_aar5_v100_better2_simpack_realtime PRIVATE cxx_std_23)
    set_target_properties(orvd_irw_r800_aar5_v100_better2_simpack_realtime
        PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_definitions(
        orvd_irw_r800_aar5_v100_better2_simpack_realtime
        PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
    target_link_libraries(
        orvd_irw_r800_aar5_v100_better2_simpack_realtime
        PRIVATE irw_single_curve_guidance_simpack_realtime_run)

    add_executable(
        orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_simpack_realtime
        run_irw_r300_aar5_v60_scbp_recorded_wear_champion_full_state_guidance_simpack_realtime.cc)
    target_compile_features(
        orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_simpack_realtime
        PRIVATE cxx_std_23)
    set_target_properties(
        orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_simpack_realtime
        PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_definitions(
        orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_simpack_realtime
        PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
    target_link_libraries(
        orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_simpack_realtime
        PRIVATE irw_single_curve_guidance_simpack_realtime_run)

    add_executable(
        orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_simpack_realtime
        run_irw_r600_aar5_v80_scbp_recorded_wear_champion_full_state_guidance_simpack_realtime.cc)
    target_compile_features(
        orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_simpack_realtime
        PRIVATE cxx_std_23)
    set_target_properties(
        orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_simpack_realtime
        PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_definitions(
        orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_simpack_realtime
        PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
    target_link_libraries(
        orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_simpack_realtime
        PRIVATE irw_single_curve_guidance_simpack_realtime_run)

    add_executable(
        orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_simpack_realtime
        run_irw_r800_aar5_v100_scbp_recorded_wear_champion_full_state_guidance_simpack_realtime.cc)
    target_compile_features(
        orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_simpack_realtime
        PRIVATE cxx_std_23)
    set_target_properties(
        orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_simpack_realtime
        PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_definitions(
        orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_simpack_realtime
        PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
    target_link_libraries(
        orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_simpack_realtime
        PRIVATE irw_single_curve_guidance_simpack_realtime_run)
endif()

if(MSVC OR CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
    target_compile_options(
        irw_guidance_control_transaction PRIVATE
        /W4 /permissive- /Zc:__cplusplus)
    target_compile_options(
        irw_guidance_operating_point_schedules PRIVATE
        /W4 /permissive- /Zc:__cplusplus)
    target_compile_options(
        irw_guidance_control_event_session PRIVATE
        /W4 /permissive- /Zc:__cplusplus)
    target_compile_options(
        irw_guidance_experiment_run PRIVATE
        /W4 /permissive- /Zc:__cplusplus)
    target_compile_options(
        orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
        PRIVATE /W4 /permissive- /Zc:__cplusplus)
    target_compile_options(
        orvd_irw_r300_aar5_v60_normal_differential_wheel_speed
        PRIVATE /W4 /permissive- /Zc:__cplusplus)
    target_compile_options(
        orvd_irw_r600_aar5_v80_normal_differential_wheel_speed
        PRIVATE /W4 /permissive- /Zc:__cplusplus)
    target_compile_options(
        orvd_irw_r800_aar5_v100_normal_differential_wheel_speed
        PRIVATE /W4 /permissive- /Zc:__cplusplus)
    foreach(target IN ITEMS
            orvd_irw_r300_aar5_v60_better2_full_state_guidance
            orvd_irw_r600_aar5_v80_better2_full_state_guidance
            orvd_irw_r800_aar5_v100_better2_full_state_guidance
            orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_full_state_guidance
            orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_full_state_guidance
            orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_full_state_guidance)
        target_compile_options(${target} PRIVATE
            /W4 /permissive- /Zc:__cplusplus)
    endforeach()
    if(ORVD_ENABLE_SIMPACK_REALTIME_COMPARISON)
        target_compile_options(irw_simpack_realtime_direct_call PRIVATE
            /W4 /permissive- /Zc:__cplusplus)
        target_compile_options(irw_single_curve_guidance_simpack_realtime_run PRIVATE
            /W4 /permissive- /Zc:__cplusplus)
        target_compile_options(
            orvd_irw_r300_aar5_v60_normal_simpack_realtime PRIVATE
            /W4 /permissive- /Zc:__cplusplus)
        target_compile_options(
            orvd_irw_r600_aar5_v80_normal_simpack_realtime PRIVATE
            /W4 /permissive- /Zc:__cplusplus)
        target_compile_options(
            orvd_irw_r800_aar5_v100_normal_simpack_realtime PRIVATE
            /W4 /permissive- /Zc:__cplusplus)
        foreach(target IN ITEMS
                orvd_irw_r300_aar5_v60_better2_simpack_realtime
                orvd_irw_r600_aar5_v80_better2_simpack_realtime
                orvd_irw_r800_aar5_v100_better2_simpack_realtime
                orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_simpack_realtime
                orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_simpack_realtime
                orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_simpack_realtime)
            target_compile_options(${target} PRIVATE
                /W4 /permissive- /Zc:__cplusplus)
        endforeach()
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
    target_compile_options(
        irw_guidance_control_transaction PRIVATE
        -Wall -Wextra -Wpedantic)
    target_compile_options(
        irw_guidance_operating_point_schedules PRIVATE
        -Wall -Wextra -Wpedantic)
    target_compile_options(
        irw_guidance_control_event_session PRIVATE
        -Wall -Wextra -Wpedantic)
    target_compile_options(
        irw_guidance_experiment_run PRIVATE
        -Wall -Wextra -Wpedantic)
    target_compile_options(
        orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
        PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(
        orvd_irw_r300_aar5_v60_normal_differential_wheel_speed
        PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(
        orvd_irw_r600_aar5_v80_normal_differential_wheel_speed
        PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(
        orvd_irw_r800_aar5_v100_normal_differential_wheel_speed
        PRIVATE -Wall -Wextra -Wpedantic)
    foreach(target IN ITEMS
            orvd_irw_r300_aar5_v60_better2_full_state_guidance
            orvd_irw_r600_aar5_v80_better2_full_state_guidance
            orvd_irw_r800_aar5_v100_better2_full_state_guidance
            orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_full_state_guidance
            orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_full_state_guidance
            orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_full_state_guidance)
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic)
    endforeach()
    if(ORVD_ENABLE_SIMPACK_REALTIME_COMPARISON)
        target_compile_options(irw_simpack_realtime_direct_call PRIVATE
            -Wall -Wextra -Wpedantic)
        target_compile_options(irw_single_curve_guidance_simpack_realtime_run PRIVATE
            -Wall -Wextra -Wpedantic)
        target_compile_options(
            orvd_irw_r300_aar5_v60_normal_simpack_realtime PRIVATE
            -Wall -Wextra -Wpedantic)
        target_compile_options(
            orvd_irw_r600_aar5_v80_normal_simpack_realtime PRIVATE
            -Wall -Wextra -Wpedantic)
        target_compile_options(
            orvd_irw_r800_aar5_v100_normal_simpack_realtime PRIVATE
            -Wall -Wextra -Wpedantic)
        foreach(target IN ITEMS
                orvd_irw_r300_aar5_v60_better2_simpack_realtime
                orvd_irw_r600_aar5_v80_better2_simpack_realtime
                orvd_irw_r800_aar5_v100_better2_simpack_realtime
                orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_simpack_realtime
                orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_simpack_realtime
                orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_simpack_realtime)
            target_compile_options(${target} PRIVATE
                -Wall -Wextra -Wpedantic)
        endforeach()
    endif()
else()
    message(FATAL_ERROR
        "Unsupported C++ compiler: ${CMAKE_CXX_COMPILER_ID}")
endif()
