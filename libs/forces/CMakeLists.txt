# The vehicle's own force elements, named by real constitutive law. The module
# knows the multibody facade and nothing above it: a force element states a
# wrench, and who integrates that wrench is not its concern.

add_library(orvd_forces STATIC
    src/independent_wheel_active_torque_plan.cc
    src/vehicle_force_plan.cc
    src/wheel_rail_contact_force_plan.cc)

orvd_configure_first_party_target(orvd_forces)

target_include_directories(orvd_forces
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(orvd_forces
    PUBLIC
        orvd_multibody_model
        orvd_track_geometry
        orvd_wheel_rail_contact
    PRIVATE
        OpenMP::OpenMP_CXX)

set_target_properties(orvd_forces PROPERTIES
    EXPORT_NAME forces)
add_library(ORVD::forces ALIAS orvd_forces)
