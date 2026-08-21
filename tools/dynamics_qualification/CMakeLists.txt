# Internal long-window qualification tooling. Nothing in this directory is
# installed or exported; it consumes the product modules and never becomes one.

add_library(orvd_dynamics_qualification_support STATIC
    atomic_qualification_directory.cc
    gz18_qualification_runner.cc
    irw_passive_scenario_recipe_dispatch.cc
    irw_r300_aar5_v60_100hz_full_state_guidance_runner.cc
    qualification_sample_clock.cc
    station_series_projection.cc
    vehicle_qualification_runner.cc)

orvd_configure_first_party_target(orvd_dynamics_qualification_support)

target_include_directories(orvd_dynamics_qualification_support
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    PRIVATE ${PROJECT_SOURCE_DIR}/libs/integrators/src)

target_link_libraries(orvd_dynamics_qualification_support
    PUBLIC orvd_configuration orvd_integrators
    PRIVATE OpenMP::OpenMP_CXX nlohmann_json::nlohmann_json)

add_executable(orvd_gz18_dynamics_qualification
    run_gz18_qualification.cc)
orvd_configure_first_party_target(orvd_gz18_dynamics_qualification)
target_link_libraries(orvd_gz18_dynamics_qualification
    PRIVATE orvd_dynamics_qualification_support)

add_executable(orvd_irw_passive_scenario
    run_irw_passive_scenario.cc)
orvd_configure_first_party_target(orvd_irw_passive_scenario)
target_link_libraries(orvd_irw_passive_scenario
    PRIVATE orvd_dynamics_qualification_support)

add_executable(orvd_irw_r300_aar5_v60_100hz_full_state_guidance
    run_irw_r300_aar5_v60_100hz_full_state_guidance.cc)
orvd_configure_first_party_target(
    orvd_irw_r300_aar5_v60_100hz_full_state_guidance)
target_link_libraries(orvd_irw_r300_aar5_v60_100hz_full_state_guidance
    PRIVATE orvd_dynamics_qualification_support)
