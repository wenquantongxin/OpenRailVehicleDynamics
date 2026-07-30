# Vendored Drake source. The upstream tree lives under drake/ with its original
# layout so that every `#include "drake/..."` line is the one upstream wrote:
# rewriting those includes would be a mechanical edit of every file and a fresh
# one on every upstream sync, for no gain.
#
# The first-party warning policy is deliberately not applied. This is upstream
# source compiled as found, and turning its style choices into our diagnostics
# would produce noise nobody is allowed to fix.

set(ORVD_VENDORED_DRAKE_MULTIBODY_TOPOLOGY_SOURCES
    drake/common/drake_assert.cc
    drake/common/fmt.cc
    drake/common/nice_type_name.cc
    drake/common/nice_type_name_override.cc
    drake/common/type_safe_index.cc
    drake/multibody/topology/link_joint_graph.cc
    drake/multibody/topology/spanning_forest.cc
    drake/multibody/topology/spanning_forest_debug.cc
    drake/multibody/topology/spanning_forest_mobod.cc)

add_library(orvd_vendored_drake_multibody_topology STATIC
    ${ORVD_VENDORED_DRAKE_MULTIBODY_TOPOLOGY_SOURCES})

# SYSTEM so that consumers do not inherit upstream diagnostics either.
target_include_directories(orvd_vendored_drake_multibody_topology
    SYSTEM PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_compile_features(orvd_vendored_drake_multibody_topology PUBLIC cxx_std_23)
set_target_properties(orvd_vendored_drake_multibody_topology
    PROPERTIES CXX_EXTENSIONS OFF)

# PUBLIC, not PRIVATE: the vendored public headers include fmt headers and
# specialise fmt::formatter, so a consumer needs fmt's include directories and
# its library, not just this archive.
target_link_libraries(orvd_vendored_drake_multibody_topology PUBLIC fmt::fmt)

# Eigen is deliberately absent. Measured: all nine translation units compile with
# no Eigen include directory at all. Linking it would declare a dependency this
# target does not have.
