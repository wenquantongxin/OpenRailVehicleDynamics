# The wheel-rail layer: the profile assets themselves, the interpolants they are
# read through, and the preparations a vehicle type binds.
#
# A compiled library, not an interface target. It links Eigen and the standard
# library and nothing else: it does not know about the line, the multibody tree,
# the force elements or the integrators, and the dependency runs from those
# toward here. The product boundary gate in the top-level CMakeLists picks this
# target up automatically because it is defined under a listed product module
# directory.

add_library(orvd_wheel_rail_contact STATIC
    src/contact_creepage.cc
    src/contact_geometry.cc
    src/contact_wrench.cc
    src/kalker_coefficient_table.cc
    src/monotone_cubic_interpolant.cc
    src/natural_cubic_spline.cc
    src/normal_contact_force.cc
    src/profile_arc_length.cc
    src/profile_points.cc
    src/profile_track_roll_transport.cc
    src/rail_gauge_datum.cc
    src/tangential_contact_force.cc
    src/track_irregularity_field.cc
    src/wheel_profile_preprocessing.cc
    src/wheel_rail_contact_model.cc
    src/wheel_rail_contact_runtime_personality.cc
    src/wheel_rail_pose.cc)

orvd_configure_first_party_target(orvd_wheel_rail_contact)

target_include_directories(orvd_wheel_rail_contact
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# PUBLIC: the roll transport carries Eigen types in its interface, so a consumer
# compiling against these headers needs Eigen's include directories.
target_link_libraries(orvd_wheel_rail_contact PUBLIC Eigen3::Eigen)

set_target_properties(orvd_wheel_rail_contact PROPERTIES
    EXPORT_NAME wheel_rail_contact)
add_library(ORVD::wheel_rail_contact ALIAS orvd_wheel_rail_contact)
