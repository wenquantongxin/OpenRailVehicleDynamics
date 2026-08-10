# Internal long-window qualification tooling. Nothing in this directory is
# installed or exported; it consumes the product modules and never becomes one.

add_library(orvd_dynamics_qualification_support STATIC
    atomic_qualification_directory.cc
    gz18_qualification_runner.cc
    irw_qualification_runner.cc
    qualification_sample_clock.cc
    station_series_projection.cc
    vehicle_qualification_runner.cc)

orvd_configure_first_party_target(orvd_dynamics_qualification_support)

target_include_directories(orvd_dynamics_qualification_support
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(orvd_dynamics_qualification_support
    PUBLIC orvd_configuration orvd_integrators
    PRIVATE OpenMP::OpenMP_CXX)

add_executable(orvd_gz18_dynamics_qualification
    run_gz18_qualification.cc)
orvd_configure_first_party_target(orvd_gz18_dynamics_qualification)
target_link_libraries(orvd_gz18_dynamics_qualification
    PRIVATE orvd_dynamics_qualification_support)

add_executable(orvd_irw_dynamics_qualification
    run_irw_qualification.cc)
orvd_configure_first_party_target(orvd_irw_dynamics_qualification)
target_link_libraries(orvd_irw_dynamics_qualification
    PRIVATE orvd_dynamics_qualification_support)
