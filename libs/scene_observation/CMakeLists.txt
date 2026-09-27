# Renderer-independent scene observation. It reads the finalized multibody
# model and the frozen wheel-rail force plan through their public queries and
# produces plain values: rigid-body identity, world-frame body states, wheel
# placement derived from the contact constants, and a sampled track table.
# It owns no integrator, clock, file, socket or rendering object, and no
# dynamics module depends on it.

add_library(orvd_scene_observation STATIC
    src/body_state.cc
    src/scene_topology.cc
    src/track_sampling.cc
    src/wheel_spin_sampling.cc)

orvd_configure_first_party_target(orvd_scene_observation)

target_include_directories(orvd_scene_observation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(orvd_scene_observation
    PUBLIC
        orvd_forces
        orvd_multibody_model
        orvd_track_geometry
        orvd_wheel_rail_contact)

set_target_properties(orvd_scene_observation PROPERTIES
    EXPORT_NAME scene_observation)
add_library(ORVD::scene_observation ALIAS orvd_scene_observation)
