# Sampled control algorithms with explicit caller-owned memory.
#
# This layer is independent of vehicle topology, event scheduling and actuator
# command conditioning.  It maps one observation sample and one previous state
# into a request, observations and the next state.

add_library(orvd_control STATIC
    src/irw_full_state_wheel_speed_guidance_controller.cc
    src/sampled_filtered_pi.cc
    src/sampled_longitudinal_cruise_controller.cc)

orvd_configure_first_party_target(orvd_control)

target_include_directories(orvd_control
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(orvd_control PROPERTIES
    EXPORT_NAME control)
add_library(ORVD::control ALIAS orvd_control)
