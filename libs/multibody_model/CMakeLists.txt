# The public modelling layer: what a caller writes against to describe a model.
#
# The dependency on the rigid tree is PRIVATE, and that is the point of this
# target. The public headers name no vendored type, so a consumer compiling
# against them never sees `drake/` — and cannot come to depend on how the tree
# arranges what it is given, because it cannot see the arrangement.
#
# Deliberately not installed and not exported. What is public at delivery is
# G46's question; exporting now would freeze an interface still being built.

add_library(orvd_multibody_model STATIC
    src/multibody_evaluation_context.cc
    src/multibody_model.cc)

orvd_configure_first_party_target(orvd_multibody_model)

target_include_directories(orvd_multibody_model
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# PUBLIC on the runtime: the public headers carry its typed physical parameter
# records, which is first-party and is the vocabulary this layer speaks.
# PRIVATE on the tree: it is an implementation, and a consumer must not inherit
# an include path that would let `drake/...` resolve.
target_link_libraries(orvd_multibody_model
    PUBLIC orvd_multibody_runtime Eigen3::Eigen
    PRIVATE orvd_rigid_multibody_tree)
