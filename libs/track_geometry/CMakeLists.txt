# The line layer: the track inertial frame, the scalar profiles laid along the
# line, the frames they induce and the projection of a point onto the
# centerline.
#
# A compiled library, not an interface target. It links Eigen and the standard
# library and nothing else: it does not know about multibody, contact or the
# integrators, and the dependency runs from those toward here. The product
# boundary gate in the top-level CMakeLists picks this target up automatically
# because it is defined under a listed product module directory.

add_library(orvd_track_geometry STATIC
    src/track_geometry.cc
    src/track_geometry_segments.cc
    src/track_inertial_frame.cc)

orvd_configure_first_party_target(orvd_track_geometry)

target_include_directories(orvd_track_geometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# PUBLIC: the public headers carry Eigen types in their interface, so a consumer
# compiling against them needs Eigen's include directories.
target_link_libraries(orvd_track_geometry PUBLIC Eigen3::Eigen)

set_target_properties(orvd_track_geometry PROPERTIES
    EXPORT_NAME track_geometry)
add_library(ORVD::track_geometry ALIAS orvd_track_geometry)
