# Vendored Drake source. The upstream tree lives under drake/ with its original
# layout so that every `#include "drake/..."` line is the one upstream wrote:
# rewriting those includes would be a mechanical edit of every file and a fresh
# one on every upstream sync, for no gain.
#
# The first-party warning policy is deliberately not applied to vendored source.
# This is upstream code compiled as found, and turning its style choices into our
# diagnostics would produce noise nobody is allowed to fix. ORVD's own source in
# orvd_implementations/ does carry the policy, and the first-party adapter headers
# live under include/orvd rather than the SYSTEM-qualified vendored include root.
# Because compilers inherit system-header identity through a vendored includer,
# a compile-only first-party target under tests/ also includes every adapter
# header directly before any Drake header. That target, rather than path spelling
# alone, is what guarantees warning coverage for the adapter headers.
#
# Compilation ownership is single: every translation unit is compiled by
# exactly one OBJECT target. The resulting object may be packaged both in its
# narrow component archive and in the complete rigid-tree archive; it is still
# the same compiled object, not a second definition produced with another set of
# flags. Consumers use either a component archive or the complete archive, not
# both.

# The support translation units that both the topology target and the double
# pose math target need. They are here, and not duplicated into either consumer,
# because a vendored .cc compiled twice is two definitions of the same symbol.
set(ORVD_VENDORED_DRAKE_COMMON_SUPPORT_SOURCES
    drake/common/drake_assert.cc
    drake/common/fmt.cc
    drake/common/nice_type_name.cc
    drake/common/nice_type_name_override.cc)

# The objects are owned by an OBJECT library and the archive is packed from it.
# That is what lets the rigid tree archive contain these objects too without
# compiling them a second time: one owner, two archives, one set of objects.
add_library(orvd_vendored_drake_common_support_objects OBJECT
    ${ORVD_VENDORED_DRAKE_COMMON_SUPPORT_SOURCES})
target_include_directories(orvd_vendored_drake_common_support_objects
    SYSTEM PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_compile_features(orvd_vendored_drake_common_support_objects
    PUBLIC cxx_std_23)
set_target_properties(orvd_vendored_drake_common_support_objects
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(orvd_vendored_drake_common_support_objects PUBLIC fmt::fmt)

add_library(orvd_vendored_drake_common_support STATIC)
target_link_libraries(orvd_vendored_drake_common_support
    PUBLIC orvd_vendored_drake_common_support_objects)


# PUBLIC, not PRIVATE: the vendored public headers include fmt headers and
# specialise fmt::formatter, so a consumer needs fmt's include directories and
# its library, not just this archive.
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

add_library(orvd_vendored_drake_multibody_topology_objects OBJECT
    ${ORVD_VENDORED_DRAKE_MULTIBODY_TOPOLOGY_SOURCES})
target_compile_features(orvd_vendored_drake_multibody_topology_objects
    PUBLIC cxx_std_23)
set_target_properties(orvd_vendored_drake_multibody_topology_objects
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(orvd_vendored_drake_multibody_topology_objects
    PUBLIC orvd_vendored_drake_common_support_objects)

add_library(orvd_vendored_drake_multibody_topology STATIC)
# The object library supplies this archive's own members; the support archive is
# a separate archive a consumer must also link. Linking an OBJECT library does
# not carry the objects of the OBJECT libraries it in turn links, so common
# support does not accidentally become a member of this narrow topology
# archive. The archive-level dependency still has to be stated here.
target_link_libraries(orvd_vendored_drake_multibody_topology
    PUBLIC orvd_vendored_drake_multibody_topology_objects
           orvd_vendored_drake_common_support)


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
target_link_libraries(orvd_double_pose_composition
    PRIVATE Eigen3::Eigen fmt::fmt)

# --- Vendored double pose math ---
# The rotation, rigid transform, roll-pitch-yaw and unit vector translation
# units, plus the first-party composition that math/fast_pose_composition_functions.h
# declares. This is a self-contained module, not a partial tree: it owns every
# definition its own public headers declare, and none of the tree sources that
# the rigid tree target below now owns.
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

add_library(orvd_vendored_drake_double_pose_math_objects OBJECT
    ${ORVD_VENDORED_DRAKE_DOUBLE_POSE_MATH_SOURCES})
target_compile_features(orvd_vendored_drake_double_pose_math_objects
    PUBLIC cxx_std_23)
set_target_properties(orvd_vendored_drake_double_pose_math_objects
    PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(orvd_vendored_drake_double_pose_math_objects
    PUBLIC orvd_vendored_drake_common_support_objects Eigen3::Eigen)

add_library(orvd_vendored_drake_double_pose_math STATIC)
target_link_libraries(orvd_vendored_drake_double_pose_math
    PUBLIC orvd_vendored_drake_double_pose_math_objects
           orvd_double_pose_composition
           orvd_vendored_drake_common_support)

# --- The rigid multibody tree ---
# The remaining landed translation units: the tree itself and the trajectory
# support one of its mobilizers needs. Together with the four object owners
# above these account for every landed .cc exactly once, which the check at the
# end of this file verifies rather than asserts.
set(ORVD_VENDORED_DRAKE_RIGID_TREE_SOURCES
    drake/common/trajectories/piecewise_constant_curvature_trajectory.cc
    drake/common/trajectories/piecewise_trajectory.cc
    drake/common/trajectories/trajectory.cc
    drake/multibody/tree/acceleration_kinematics_cache.cc
    drake/multibody/tree/articulated_body_force_cache.cc
    drake/multibody/tree/articulated_body_inertia.cc
    drake/multibody/tree/articulated_body_inertia_cache.cc
    drake/multibody/tree/ball_rpy_joint.cc
    drake/multibody/tree/body_node.cc
    drake/multibody/tree/body_node_impl.cc
    drake/multibody/tree/body_node_impl_mass_matrix.cc
    drake/multibody/tree/curvilinear_joint.cc
    drake/multibody/tree/curvilinear_mobilizer.cc
    drake/multibody/tree/element_collection.cc
    drake/multibody/tree/fixed_offset_frame.cc
    drake/multibody/tree/force_element.cc
    drake/multibody/tree/frame.cc
    drake/multibody/tree/frame_body_pose_cache.cc
    drake/multibody/tree/joint.cc
    drake/multibody/tree/joint_actuator.cc
    drake/multibody/tree/linear_bushing_roll_pitch_yaw.cc
    drake/multibody/tree/linear_spring_damper.cc
    drake/multibody/tree/mobilizer.cc
    drake/multibody/tree/mobilizer_impl.cc
    drake/multibody/tree/model_instance.cc
    drake/multibody/tree/multibody_element.cc
    drake/multibody/tree/multibody_forces.cc
    drake/multibody/tree/multibody_tree.cc
    drake/multibody/tree/planar_joint.cc
    drake/multibody/tree/planar_mobilizer.cc
    drake/multibody/tree/position_kinematics_cache.cc
    drake/multibody/tree/prismatic_joint.cc
    drake/multibody/tree/prismatic_mobilizer.cc
    drake/multibody/tree/prismatic_spring.cc
    drake/multibody/tree/quaternion_floating_joint.cc
    drake/multibody/tree/quaternion_floating_mobilizer.cc
    drake/multibody/tree/revolute_joint.cc
    drake/multibody/tree/revolute_mobilizer.cc
    drake/multibody/tree/revolute_spring.cc
    drake/multibody/tree/rigid_body.cc
    drake/multibody/tree/rotational_inertia.cc
    drake/multibody/tree/rpy_ball_mobilizer.cc
    drake/multibody/tree/rpy_floating_joint.cc
    drake/multibody/tree/rpy_floating_mobilizer.cc
    drake/multibody/tree/scoped_name.cc
    drake/multibody/tree/screw_joint.cc
    drake/multibody/tree/screw_mobilizer.cc
    drake/multibody/tree/spatial_inertia.cc
    drake/multibody/tree/uniform_gravity_field_element.cc
    drake/multibody/tree/unit_inertia.cc
    drake/multibody/tree/universal_joint.cc
    drake/multibody/tree/universal_mobilizer.cc
    drake/multibody/tree/velocity_kinematics_cache.cc
    drake/multibody/tree/weld_joint.cc
    drake/multibody/tree/weld_mobilizer.cc)

add_library(orvd_vendored_drake_rigid_tree_objects OBJECT
    ${ORVD_VENDORED_DRAKE_RIGID_TREE_SOURCES})
target_compile_features(orvd_vendored_drake_rigid_tree_objects PUBLIC cxx_std_23)
set_target_properties(orvd_vendored_drake_rigid_tree_objects
    PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(orvd_vendored_drake_rigid_tree_objects
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# The tree reaches down to the first-party runtime for state, versions and cache
# slots. That is the intended direction and the only one: the runtime never
# reaches back, which is why orvd_multibody_runtime links nothing from here.
target_link_libraries(orvd_vendored_drake_rigid_tree_objects
    PUBLIC orvd_vendored_drake_multibody_topology_objects
           orvd_vendored_drake_double_pose_math_objects
           orvd_multibody_runtime)

# The five compilation owners are also the single source of truth for both the
# complete archive and the all-object link test. Keeping that set in one place
# prevents the verifier from silently omitting an owner that the archive gained,
# or from verifying an owner the archive no longer packages.
set(ORVD_RIGID_MULTIBODY_TREE_OBJECT_OWNER_TARGETS
    orvd_vendored_drake_rigid_tree_objects
    orvd_vendored_drake_multibody_topology_objects
    orvd_vendored_drake_double_pose_math_objects
    orvd_vendored_drake_common_support_objects
    orvd_double_pose_composition)
set(ORVD_RIGID_MULTIBODY_TREE_OBJECT_OWNER_TARGETS
    ${ORVD_RIGID_MULTIBODY_TREE_OBJECT_OWNER_TARGETS}
    PARENT_SCOPE)

# The internal product target: one archive holding every landed object.
#
# It is an archive of the objects themselves, not a target that merely links the
# four sub-archives. A consumer of a static library gets only the members that
# resolve a symbol it already needs, so a chain of archives cannot be made to
# yield every object; putting the objects in makes "this archive is the tree"
# true rather than approximately true.
#
# G46 installs this archive only as an implementation dependency of the public
# modelling target.  Its exported name is explicitly internal, and none of its
# vendored or adapter headers is installed.
add_library(orvd_rigid_multibody_tree STATIC)
foreach(orvd_rigid_tree_object_owner
        IN LISTS ORVD_RIGID_MULTIBODY_TREE_OBJECT_OWNER_TARGETS)
    target_link_libraries(orvd_rigid_multibody_tree
        PUBLIC "$<BUILD_INTERFACE:${orvd_rigid_tree_object_owner}>")
endforeach()
# The object owners vanish from the installed link interface, but their archive
# still contains unresolved fmt calls and runtime references.  Keep those real
# link dependencies explicit for an installed static consumer.
target_link_libraries(orvd_rigid_multibody_tree
    PUBLIC orvd_multibody_runtime
    PRIVATE fmt::fmt)
set_target_properties(orvd_rigid_multibody_tree PROPERTIES
    EXPORT_NAME _internal_rigid_multibody_tree)

# --- Single source ownership, checked ---
# Every landed .cc must belong to exactly one of the object owners that form the
# complete archive. Read each target's actual SOURCES property rather than
# rebuilding that fact from a second hand-written list: changing an owner target
# must change the archive, the all-object test and this check together.
#
# The check compares sets, not counts: freezing a number would make an added
# file look like a violation and a removed one look fine.
set(_orvd_owned_sources "")
foreach(object_owner_target
        IN LISTS ORVD_RIGID_MULTIBODY_TREE_OBJECT_OWNER_TARGETS)
    get_target_property(object_owner_sources ${object_owner_target} SOURCES)
    if(NOT object_owner_sources)
        message(FATAL_ERROR
            "vendored source ownership: object owner "
            "'${object_owner_target}' has no translation unit")
    endif()
    foreach(object_owner_source IN LISTS object_owner_sources)
        if(object_owner_source MATCHES "\\$<")
            message(FATAL_ERROR
                "vendored source ownership: object owner "
                "'${object_owner_target}' uses the generator-expression source "
                "'${object_owner_source}', whose identity cannot be checked at "
                "configure time")
        endif()
        cmake_path(ABSOLUTE_PATH object_owner_source
            BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            NORMALIZE OUTPUT_VARIABLE absolute_object_owner_source)
        cmake_path(RELATIVE_PATH absolute_object_owner_source
            BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE relative_object_owner_source)
        if(relative_object_owner_source MATCHES "^\\.\\.")
            message(FATAL_ERROR
                "vendored source ownership: object owner "
                "'${object_owner_target}' compiles the source "
                "'${object_owner_source}' outside this landed source root")
        endif()
        list(APPEND _orvd_owned_sources "${relative_object_owner_source}")
    endforeach()
endforeach()

set(_orvd_duplicated_sources ${_orvd_owned_sources})
list(REMOVE_DUPLICATES _orvd_duplicated_sources)
list(LENGTH _orvd_owned_sources _orvd_owned_count)
list(LENGTH _orvd_duplicated_sources _orvd_unique_count)
if(NOT _orvd_owned_count EQUAL _orvd_unique_count)
    message(FATAL_ERROR
        "vendored source ownership: a translation unit is claimed by more than "
        "one object owner, so one archive would hold two definitions of the "
        "same symbol")
endif()

file(GLOB_RECURSE _orvd_landed_sources
     RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
     CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
set(_orvd_unowned_sources ${_orvd_landed_sources})
list(REMOVE_ITEM _orvd_unowned_sources ${_orvd_owned_sources})
if(_orvd_unowned_sources)
    message(FATAL_ERROR
        "vendored source ownership: these landed translation units belong to no "
        "target, so nothing compiles them and a missing definition would only "
        "appear at a consumer's link: ${_orvd_unowned_sources}")
endif()

set(_orvd_missing_sources ${_orvd_owned_sources})
list(REMOVE_ITEM _orvd_missing_sources ${_orvd_landed_sources})
if(_orvd_missing_sources)
    message(FATAL_ERROR
        "vendored source ownership: these sources are claimed by a target but "
        "are not landed: ${_orvd_missing_sources}")
endif()
