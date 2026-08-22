cmake_minimum_required(VERSION 3.24)

project(orvd_irw_crossline_full_state_guidance_experiment LANGUAGES CXX)

# This experiment is deliberately an installed-package consumer.  It is not
# entered by the ORVD root build, installed with ORVD, or registered in CTest.
find_package(OpenRailVehicleDynamics CONFIG REQUIRED)

add_library(irw_crossline_operating_point_schedule STATIC
    irw_crossline_operating_point_schedule.cc)

target_compile_features(
    irw_crossline_operating_point_schedule PUBLIC cxx_std_23)
set_target_properties(
    irw_crossline_operating_point_schedule PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(
    irw_crossline_operating_point_schedule
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(
    irw_crossline_operating_point_schedule PUBLIC ORVD::control)

add_library(irw_crossline_control_event_session STATIC
    irw_crossline_control_event_session.cc)
target_compile_features(
    irw_crossline_control_event_session PUBLIC cxx_std_23)
set_target_properties(
    irw_crossline_control_event_session PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(
    irw_crossline_control_event_session
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(
    irw_crossline_control_event_session
    PUBLIC irw_crossline_operating_point_schedule ORVD::configuration)

add_executable(
    orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
    run_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance.cc
    irw_crossline_full_state_guidance_run.cc)
target_compile_features(
    orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
    PRIVATE cxx_std_23)
set_target_properties(
    orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(
    orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
    PRIVATE irw_crossline_control_event_session
            ORVD::configuration ORVD::integrators)

if(MSVC OR CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
    target_compile_options(
        irw_crossline_operating_point_schedule PRIVATE
        /W4 /permissive- /Zc:__cplusplus)
    target_compile_options(
        irw_crossline_control_event_session PRIVATE
        /W4 /permissive- /Zc:__cplusplus)
    target_compile_options(
        orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
        PRIVATE /W4 /permissive- /Zc:__cplusplus)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
    target_compile_options(
        irw_crossline_operating_point_schedule PRIVATE
        -Wall -Wextra -Wpedantic)
    target_compile_options(
        irw_crossline_control_event_session PRIVATE
        -Wall -Wextra -Wpedantic)
    target_compile_options(
        orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
        PRIVATE -Wall -Wextra -Wpedantic)
else()
    message(FATAL_ERROR
        "Unsupported C++ compiler: ${CMAKE_CXX_COMPILER_ID}")
endif()
