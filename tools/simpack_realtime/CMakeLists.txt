# Local adapter for the proprietary SIMPACK Realtime direct-call interface.
# The top-level option is OFF by default. Nothing here is installed or exported.

add_library(orvd_simpack_realtime_direct_call STATIC
    simpack_realtime_instance.cc)
orvd_configure_first_party_target(orvd_simpack_realtime_direct_call)
target_include_directories(orvd_simpack_realtime_direct_call
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_include_directories(orvd_simpack_realtime_direct_call SYSTEM
    PRIVATE "${ORVD_SIMPACK_REALTIME_INCLUDE_DIRECTORY}")
target_compile_definitions(orvd_simpack_realtime_direct_call
    PRIVATE SPCK_RT_API=2)
target_link_libraries(orvd_simpack_realtime_direct_call
    PRIVATE "${ORVD_SIMPACK_REALTIME_LIBRARY}" Threads::Threads rt m)

add_library(orvd_irw_simpack_realtime_runner STATIC
    irw_simpack_realtime_runner.cc)
orvd_configure_first_party_target(orvd_irw_simpack_realtime_runner)
target_include_directories(orvd_irw_simpack_realtime_runner
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(orvd_irw_simpack_realtime_runner
    PUBLIC orvd_simpack_realtime_direct_call
    PRIVATE orvd_configuration)

add_executable(orvd_irw_simpack_realtime_cruise
    run_irw_simpack_realtime_cruise.cc)
orvd_configure_first_party_target(orvd_irw_simpack_realtime_cruise)
target_compile_definitions(orvd_irw_simpack_realtime_cruise
    PRIVATE ORVD_SIMPACK_ROOT="${ORVD_SIMPACK_ROOT}")
target_link_libraries(orvd_irw_simpack_realtime_cruise
    PRIVATE orvd_irw_simpack_realtime_runner)
