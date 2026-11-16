# The document a graphical modelling tool edits for one rail vehicle: its
# mechanical definition, its wheel-rail contact record once provided, free-body
# placements and the names bodies' shapes are stated under, both for drawing,
# and a tree of groups that only organises. It generates the configuration
# records a vehicle assembly consumes; it reads and writes no files.

add_library(orvd_vehicle_project STATIC
    src/generate_assembly_inputs_from_vehicle_project.cc
    src/make_vehicle_project_from_vehicle_definition.cc
    src/vehicle_project_free_body_placements.cc
    src/vehicle_project_invariants.cc)

orvd_configure_first_party_target(orvd_vehicle_project)

target_include_directories(orvd_vehicle_project
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(orvd_vehicle_project
    PUBLIC orvd_configuration)

set_target_properties(orvd_vehicle_project PROPERTIES
    EXPORT_NAME vehicle_project)
add_library(ORVD::vehicle_project ALIAS orvd_vehicle_project)
