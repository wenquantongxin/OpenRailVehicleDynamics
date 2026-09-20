add_library(orvd_state_stream STATIC state_stream.cc)
orvd_configure_first_party_target(orvd_state_stream)
target_include_directories(orvd_state_stream PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(orvd_state_stream PUBLIC orvd_multibody_model
    PRIVATE $<BUILD_INTERFACE:nlohmann_json::nlohmann_json>)
set_target_properties(orvd_state_stream PROPERTIES EXPORT_NAME state_stream)
add_library(ORVD::state_stream ALIAS orvd_state_stream)
