add_library(orvd_integrators STATIC
    src/continuous_state_advancer.cc
    src/system_rhs_bridge.cc)

orvd_configure_first_party_target(orvd_integrators)

target_include_directories(orvd_integrators
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(orvd_integrators
    PUBLIC Eigen3::Eigen orvd_system_assembly)

# SUNDIALS is intentionally absent.  G44 owns the CVODE backend; G43 exposes a
# contiguous Eigen view that can bind N_Vector storage without including or
# linking that third-party library here.
