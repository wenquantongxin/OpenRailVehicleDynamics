# The runtime base layer: state, parameters, and later versions and cache slots.
#
# A compiled library, not an interface target. An interface library would let the
# module claim to exist without anything having compiled, and the first real
# consumer would discover the difference.
#
# It links Eigen and the standard library and nothing else. In particular it does
# not link the vendored Drake targets: the dependency runs the other way, and the
# adapter that owns the rigid tree is what reaches down to here. The product
# boundary gate in the top-level CMakeLists picks this target up automatically
# because it is defined under a product module directory.

add_library(orvd_multibody_runtime STATIC
    src/multibody_state_instance.cc
    src/multibody_state_layout.cc)

orvd_configure_first_party_target(orvd_multibody_runtime)

target_include_directories(orvd_multibody_runtime
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# PUBLIC: the public headers carry Eigen types in their interface, so a consumer
# compiling against them needs Eigen's include directories.
target_link_libraries(orvd_multibody_runtime PUBLIC Eigen3::Eigen)

# Deliberately not installed or exported. Installation is a delivery concern with
# its own set of decisions about what is public; those belong to G29 and G46, and
# exporting a target before then would freeze an interface still being built.
