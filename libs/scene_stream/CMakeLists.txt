add_library(orvd_scene_stream STATIC src/scene_stream.cc)
orvd_configure_first_party_target(orvd_scene_stream)
target_include_directories(orvd_scene_stream PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(orvd_scene_stream PUBLIC orvd_scene_observation
    PRIVATE $<BUILD_INTERFACE:nlohmann_json::nlohmann_json>)
set_target_properties(orvd_scene_stream PROPERTIES EXPORT_NAME scene_stream)
add_library(ORVD::scene_stream ALIAS orvd_scene_stream)
