# A portable scene record on disk: the scene topology and layout as JSON, the
# frames as a flat little-endian binary64 table, and the parametric visual
# definition copied alongside. It consumes public scene data only; it knows
# neither the integrator nor any transport.

add_library(orvd_scene_record STATIC
    src/scene_record_reader.cc
    src/scene_record_writer.cc)

orvd_configure_first_party_target(orvd_scene_record)

target_include_directories(orvd_scene_record
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(orvd_scene_record
    PUBLIC orvd_scene_observation
    PRIVATE $<BUILD_INTERFACE:nlohmann_json::nlohmann_json>)

set_target_properties(orvd_scene_record PROPERTIES
    EXPORT_NAME scene_record)
add_library(ORVD::scene_record ALIAS orvd_scene_record)
