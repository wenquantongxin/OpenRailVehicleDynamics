add_library(orvd_integrators STATIC
    src/continuous_state_advancer.cc
    src/cvode_continuous_state_advancer.cc
    src/system_rhs_bridge.cc)

orvd_configure_first_party_target(orvd_integrators)

target_include_directories(orvd_integrators
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(orvd_integrators
    PUBLIC Eigen3::Eigen orvd_system_assembly)

# CVODE is an implementation dependency. No SUNDIALS type appears in a public
# ORVD header; static-library consumers receive only the link edge needed to
# resolve this object's private implementation.
target_link_libraries(orvd_integrators PRIVATE
    SUNDIALS::cvode
    SUNDIALS::nvecserial
    SUNDIALS::sunmatrixdense
    SUNDIALS::sunlinsoldense)
