add_library(orvd_state_stream STATIC state_stream.cc)
orvd_configure_first_party_target(orvd_state_stream)
target_include_directories(orvd_state_stream PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(orvd_state_stream PUBLIC orvd_scene_stream)
set_target_properties(orvd_state_stream PROPERTIES EXPORT_NAME state_stream)
add_library(ORVD::state_stream ALIAS orvd_state_stream)

find_package(Threads REQUIRED)
add_executable(orvd_replay_scene_record replay_scene_record.cc)
orvd_configure_first_party_target(orvd_replay_scene_record)
target_link_libraries(orvd_replay_scene_record
    PRIVATE orvd_scene_record orvd_state_stream Threads::Threads)
