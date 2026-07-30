# Vendored Drake source. The upstream tree lives under drake/ with its original
# layout so that every `#include "drake/..."` line is the one upstream wrote:
# rewriting those includes would be a mechanical edit of every file and a fresh
# one on every upstream sync, for no gain.
#
# The first-party warning policy is deliberately not applied to vendored source.
# This is upstream code compiled as found, and turning its style choices into our
# diagnostics would produce noise nobody is allowed to fix. ORVD's own source in
# orvd_implementations/ is a different matter and does carry the policy; see the
# object target below.
#
# Source ownership is single: every translation unit belongs to exactly one
# target. Two archives defining the same object would make link order decide
# which definition a consumer gets.

# The support translation units that both the topology target and the double
# pose math target need. They are here, and not duplicated into either consumer,
# because a vendored .cc compiled twice is two definitions of the same symbol.
set(ORVD_VENDORED_DRAKE_COMMON_SUPPORT_SOURCES
    drake/common/drake_assert.cc
    drake/common/fmt.cc
    drake/common/nice_type_name.cc
    drake/common/nice_type_name_override.cc)

add_library(orvd_vendored_drake_common_support STATIC
    ${ORVD_VENDORED_DRAKE_COMMON_SUPPORT_SOURCES})

# SYSTEM so that consumers do not inherit upstream diagnostics either.
target_include_directories(orvd_vendored_drake_common_support
    SYSTEM PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_compile_features(orvd_vendored_drake_common_support PUBLIC cxx_std_23)
set_target_properties(orvd_vendored_drake_common_support
    PROPERTIES CXX_EXTENSIONS OFF)

# PUBLIC, not PRIVATE: the vendored public headers include fmt headers and
# specialise fmt::formatter, so a consumer needs fmt's include directories and
# its library, not just this archive.
target_link_libraries(orvd_vendored_drake_common_support PUBLIC fmt::fmt)

# Eigen is deliberately absent from this target. Measured: these four
# translation units compile with no Eigen include directory at all. Declaring it
# would hand an Eigen dependency to the topology target, which does not have one.

# --- Vendored multibody topology ---
# type_safe_index.cc lives here rather than in the common support target because
# topology is its only consumer: measured, the double pose math target links
# without it.
set(ORVD_VENDORED_DRAKE_MULTIBODY_TOPOLOGY_SOURCES
    drake/common/type_safe_index.cc
    drake/multibody/topology/link_joint_graph.cc
    drake/multibody/topology/spanning_forest.cc
    drake/multibody/topology/spanning_forest_debug.cc
    drake/multibody/topology/spanning_forest_mobod.cc)

add_library(orvd_vendored_drake_multibody_topology STATIC
    ${ORVD_VENDORED_DRAKE_MULTIBODY_TOPOLOGY_SOURCES})

target_compile_features(orvd_vendored_drake_multibody_topology PUBLIC cxx_std_23)
set_target_properties(orvd_vendored_drake_multibody_topology
    PROPERTIES CXX_EXTENSIONS OFF)

# PUBLIC: the include directory and fmt both reach consumers through the support
# target's own PUBLIC interface.
target_link_libraries(orvd_vendored_drake_multibody_topology
    PUBLIC orvd_vendored_drake_common_support)

# --- ORVD's own double pose composition ---
# This is first-party source satisfying a vendored declaration, so it carries the
# first-party warning policy while the vendored math sources beside it do not. An
# object library keeps that distinction target-scoped and still lets the objects
# land in the same archive as the sources that declare them.
add_library(orvd_double_pose_composition OBJECT
    orvd_implementations/double_rotation_and_rigid_transform_composition.cc)
orvd_configure_first_party_target(orvd_double_pose_composition)
target_include_directories(orvd_double_pose_composition
    SYSTEM PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(orvd_double_pose_composition PRIVATE Eigen3::Eigen)

# --- Vendored double pose math ---
# The rotation, rigid transform, roll-pitch-yaw and unit vector translation
# units, plus the first-party composition that math/fast_pose_composition_functions.h
# declares. This is a self-contained module, not a partial tree: it owns every
# definition its own public headers declare, and none of the tree sources that
# still wait on the runtime replacement of G20-G28.
#
# fmt_eigen.cc belongs to this target rather than to the common support target
# because this is its only consumer, and because fmt_eigen.h includes Eigen: in
# the support target it would give the topology target an Eigen dependency it
# does not have.
set(ORVD_VENDORED_DRAKE_DOUBLE_POSE_MATH_SOURCES
    drake/common/fmt_eigen.cc
    drake/math/rigid_transform.cc
    drake/math/roll_pitch_yaw.cc
    drake/math/rotation_matrix.cc
    drake/math/unit_vector.cc)

add_library(orvd_vendored_drake_double_pose_math STATIC
    ${ORVD_VENDORED_DRAKE_DOUBLE_POSE_MATH_SOURCES}
    $<TARGET_OBJECTS:orvd_double_pose_composition>)

target_compile_features(orvd_vendored_drake_double_pose_math PUBLIC cxx_std_23)
set_target_properties(orvd_vendored_drake_double_pose_math
    PROPERTIES CXX_EXTENSIONS OFF)

# Eigen is PUBLIC here: math/rigid_transform.h and math/rotation_matrix.h carry
# Eigen types in their public interface, so a consumer compiling against them
# needs Eigen's include directories. fmt reaches consumers through the support
# target.
target_link_libraries(orvd_vendored_drake_double_pose_math
    PUBLIC orvd_vendored_drake_common_support Eigen3::Eigen)
