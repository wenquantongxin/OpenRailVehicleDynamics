# Strict, one-shot loading of human-authored product configuration. JSON is an
# implementation detail: installed consumers receive typed ORVD objects and do
# not need the parser or its headers.

add_library(orvd_configuration STATIC
    src/assemble_resolved_initial_context.cc
    src/assemble_vehicle_multibody_model.cc
    src/assembled_gz18_contact_scenario.cc
    src/assembled_vehicle_system.cc
    src/gz18_wheel_rail_contact.cc
    src/load_profile_points.cc
    src/load_resolved_startup_state.cc
    src/load_track_irregularity_field.cc
    src/load_track_geometry.cc
    src/load_vehicle_definition.cc
    src/resolved_startup_state_invariants.cc
    src/strict_json.cc
    src/vehicle_definition_invariants.cc
    src/vehicle_definition_inertia.cc)

orvd_configure_first_party_target(orvd_configuration)

target_include_directories(orvd_configuration
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(orvd_configuration
    PUBLIC orvd_forces orvd_multibody_model orvd_system_assembly
           orvd_track_geometry orvd_wheel_rail_contact
    PRIVATE $<BUILD_INTERFACE:nlohmann_json::nlohmann_json>)

set_target_properties(orvd_configuration PROPERTIES
    EXPORT_NAME configuration)
add_library(ORVD::configuration ALIAS orvd_configuration)
