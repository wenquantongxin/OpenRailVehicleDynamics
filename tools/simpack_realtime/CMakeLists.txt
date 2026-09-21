# Optional adapter for the proprietary SIMPACK Realtime direct-call interface.
# Both the owning direct-call session and the high-level runner are installed
# when the top-level option is enabled. The vendor ABI stays private.

add_library(orvd_simpack_realtime_direct_call STATIC
    simpack_realtime_instance.cc)
orvd_configure_first_party_target(orvd_simpack_realtime_direct_call)
target_include_directories(orvd_simpack_realtime_direct_call
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(orvd_simpack_realtime_direct_call SYSTEM
    PRIVATE "${ORVD_SIMPACK_REALTIME_INCLUDE_DIRECTORY}")
target_compile_definitions(orvd_simpack_realtime_direct_call
    PRIVATE SPCK_RT_API=2)
target_link_libraries(orvd_simpack_realtime_direct_call
    PRIVATE SIMPACKRealtime::spck_rt Threads::Threads rt m)
set_target_properties(orvd_simpack_realtime_direct_call PROPERTIES
    EXPORT_NAME simpack_realtime_direct_call)
add_library(ORVD::simpack_realtime_direct_call ALIAS orvd_simpack_realtime_direct_call)

add_library(orvd_irw_simpack_realtime_runner STATIC
    irw_simpack_realtime_runner.cc)
orvd_configure_first_party_target(orvd_irw_simpack_realtime_runner)
target_include_directories(orvd_irw_simpack_realtime_runner
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(orvd_irw_simpack_realtime_runner
    PRIVATE orvd_configuration orvd_simpack_realtime_direct_call)
set_target_properties(orvd_irw_simpack_realtime_runner PROPERTIES
    EXPORT_NAME simpack_realtime)
add_library(ORVD::simpack_realtime ALIAS orvd_irw_simpack_realtime_runner)

add_executable(orvd_irw_simpack_realtime_cruise
    run_irw_simpack_realtime_cruise.cc)
orvd_configure_first_party_target(orvd_irw_simpack_realtime_cruise)
target_compile_definitions(orvd_irw_simpack_realtime_cruise
    PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
target_link_libraries(orvd_irw_simpack_realtime_cruise
    PRIVATE orvd_irw_simpack_realtime_runner)
