# Strict, one-shot loading of human-authored product configuration. JSON is an
# implementation detail: installed consumers receive typed ORVD objects and do
# not need the parser or its headers.

add_library(orvd_configuration STATIC
    src/load_track_geometry.cc)

orvd_configure_first_party_target(orvd_configuration)

target_include_directories(orvd_configuration
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(orvd_configuration
    PUBLIC orvd_track_geometry
    PRIVATE $<BUILD_INTERFACE:nlohmann_json::nlohmann_json>)

set_target_properties(orvd_configuration PROPERTIES
    EXPORT_NAME configuration)
add_library(ORVD::configuration ALIAS orvd_configuration)
