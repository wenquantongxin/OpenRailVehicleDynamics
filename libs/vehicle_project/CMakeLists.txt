# The document a graphical modelling tool edits for one rail vehicle: its
# mechanical definition, its wheel-rail contact record once provided, the
# gravity it is checked under, free-body placements, the names bodies' shapes
# are stated under and the visual definition they are drawn with, and a tree
# of groups that only organises. It generates the configuration records a
# vehicle assembly consumes, takes a visual definition file in as the text a
# project carries, and writes and reads a project as one file of a directory
# of projects. JSON stays inside: the public headers hold none, and the
# configuration library's encoders and strict parser are reached through its
# internal directory, which is not installed.

add_library(orvd_vehicle_project STATIC
    src/generate_assembly_inputs_from_vehicle_project.cc
    src/make_vehicle_project_from_vehicle_definition.cc
    src/read_visual_definition_document_from_file.cc
    src/vehicle_project_file_reading_and_writing.cc
    src/vehicle_project_files_in_directory.cc
    src/vehicle_project_free_body_placements.cc
    src/vehicle_project_invariants.cc
    src/vehicle_project_json_reading_and_writing.cc)

orvd_configure_first_party_target(orvd_vehicle_project)

target_include_directories(orvd_vehicle_project
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/libs/configuration/internal>)

target_link_libraries(orvd_vehicle_project
    PUBLIC orvd_configuration
    PRIVATE $<BUILD_INTERFACE:nlohmann_json::nlohmann_json>)

set_target_properties(orvd_vehicle_project PROPERTIES
    EXPORT_NAME vehicle_project)
add_library(ORVD::vehicle_project ALIAS orvd_vehicle_project)
