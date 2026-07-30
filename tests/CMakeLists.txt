# The source-boundary analyzer is a development tool, not a shipped artifact, so
# Python is only ever required when testing is enabled. It builds its own
# synthetic source trees, so this check needs neither Drake nor a Drake clone.
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
find_package(Git REQUIRED)
add_test(NAME verify_source_closure_analyzer
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_SOURCE_DIR}/tools/drake_source_boundary/verify_source_closure_analyzer.py)

# The source-boundary probe is a development tool with a GNU-style compiler
# command line, not a product target. Its self-check uses the same native
# compiler executable as this build. Cross builds and MSVC-style frontends are
# left to the CMake-native product build exercised later in the roadmap.
if(UNIX AND NOT CMAKE_CROSSCOMPILING AND
   CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
    add_test(NAME verify_admitted_translation_unit_compile_probe
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_SOURCE_DIR}/tools/drake_source_boundary/verify_admitted_translation_unit_compile_probe.py
                     --compiler ${CMAKE_CXX_COMPILER})
endif()

# The architectural boundary: no product source is a forbidden Drake file, and
# none includes one. Both the synthetic discrimination check and a resident run
# against the real product source are registered — the synthetic one proves the
# tool still fires, the real one proves the boundary still holds.
add_test(NAME verify_product_source_drake_boundary_discrimination
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_SOURCE_DIR}/tools/drake_source_boundary/verify_product_source_drake_boundary_discrimination.py)
add_test(NAME verify_product_source_drake_boundary
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_SOURCE_DIR}/tools/drake_source_boundary/verify_product_source_drake_boundary.py
                 --repository-root ${CMAKE_SOURCE_DIR}
                 --disposition-ledger
                 ${CMAKE_SOURCE_DIR}/external/drake_mbtree/SOURCE_DISPOSITION.txt)

# The link boundary gate's own discrimination check. It configures synthetic
# projects that include this repository's gate module, so what it exercises is the
# gate this build uses rather than a copy of it. The gate itself has already run
# by the time any test does: it fails at configure time, which is why there is no
# test here for the real product targets.
add_test(NAME verify_product_boundary_gate
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_SOURCE_DIR}/tools/product_boundary_gate/verify_product_boundary_gate.py
                 --cmake ${CMAKE_COMMAND})

# The provenance audit. Only its synthetic self-check is registered: the real
# audit needs a clone of the pinned upstream, which is a large working copy of
# someone else's repository whose location is a fact about a machine, not about
# this project. Its path is passed on the command line; see
# tools/drake_source_boundary/README.md.
add_test(NAME verify_landed_drake_source_provenance_audit
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_SOURCE_DIR}/tools/drake_source_boundary/verify_landed_drake_source_provenance_audit.py
                 --git-executable ${GIT_EXECUTABLE})

# The landed compile frontier tool. Only its synthetic self-check is registered:
# running the tool against the real landed tree exits non-zero today, because the
# translation units that still need the runtime cannot compile until G20-G28
# replace it. Registering that as a test would mean either a permanently red suite
# or a WILL_FAIL wrapper, and a test allowed to fail stops being read. The real run
# is a development command; see tools/drake_source_boundary/README.md.
if(UNIX AND NOT CMAKE_CROSSCOMPILING AND
   CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
    add_test(NAME verify_landed_double_compile_frontier
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_SOURCE_DIR}/tools/drake_source_boundary/verify_landed_double_compile_frontier.py
                     --compiler ${CMAKE_CXX_COMPILER})
endif()

# The index and ordering contract the vendored topology provides. It links only
# the vendored target: this is a structural contract, not a comparison against
# Drake, so libdrake must not be anywhere near it.
add_executable(verify_vendored_drake_multibody_topology_index_assignment
    topology/verify_vendored_drake_multibody_topology_index_assignment.cc)
orvd_configure_first_party_target(
    verify_vendored_drake_multibody_topology_index_assignment)
target_link_libraries(verify_vendored_drake_multibody_topology_index_assignment
    PRIVATE orvd_vendored_drake_multibody_topology)
add_test(NAME verify_vendored_drake_multibody_topology_index_assignment
         COMMAND verify_vendored_drake_multibody_topology_index_assignment)

# The algebra and aliasing contract of the four double pose compositions. It
# links the real drake::math::internal::Compose* symbols out of the pose math
# target, so it checks the shipped implementation rather than a copy. libdrake
# is nowhere near it: the point is that ORVD's own definitions are correct.
add_executable(verify_double_pose_composition_contract
    math/verify_double_pose_composition_contract.cc)
orvd_configure_first_party_target(verify_double_pose_composition_contract)
target_link_libraries(verify_double_pose_composition_contract
    PRIVATE orvd_vendored_drake_double_pose_math)
add_test(NAME verify_double_pose_composition_contract
         COMMAND verify_double_pose_composition_contract)

# What the runtime state store owns, what it refuses, and what it leaves alone
# when it refuses. It links only the runtime base layer: this layer knows nothing
# about the rigid tree, and a test that reached for one would be testing a
# dependency the design forbids.
add_executable(verify_multibody_state_instance
    multibody_runtime/verify_multibody_state_instance.cc)
orvd_configure_first_party_target(verify_multibody_state_instance)
target_link_libraries(verify_multibody_state_instance
    PRIVATE orvd_multibody_runtime)
add_test(NAME verify_multibody_state_instance
         COMMAND verify_multibody_state_instance)

# When a cache slot will hand out its value and when it refuses. The value type
# here is synthetic on purpose: the base layer must not name the rigid tree's
# cache values, so what is under test is the slot mechanism rather than any
# particular thing to put in one.
add_executable(verify_multibody_cache_slot
    multibody_runtime/verify_multibody_cache_slot.cc)
orvd_configure_first_party_target(verify_multibody_cache_slot)
target_link_libraries(verify_multibody_cache_slot
    PRIVATE orvd_multibody_runtime)
add_test(NAME verify_multibody_cache_slot
         COMMAND verify_multibody_cache_slot)

# The runtime contract's dependency policy, made executable: eight declared
# dependency subsets against five state sources. The expectations are transcribed
# from the contract by hand rather than derived from the slots' own template
# parameters — an expectation computed the same way the slot computes freshness
# would agree with any declaration, and the declaration is what is under test.
add_executable(verify_multibody_cache_dependency_matrix
    multibody_runtime/verify_multibody_cache_dependency_matrix.cc)
orvd_configure_first_party_target(verify_multibody_cache_dependency_matrix)
target_link_libraries(verify_multibody_cache_dependency_matrix
    PRIVATE orvd_multibody_runtime)
add_test(NAME verify_multibody_cache_dependency_matrix
         COMMAND verify_multibody_cache_dependency_matrix)

# The toolchain self-check is the project's first real first-party target, and
# so also the first consumer of the target-scoped compile policy.
add_executable(verify_eigen_and_cxx23_toolchain
    toolchain/verify_eigen_and_cxx23_toolchain.cc)
orvd_configure_first_party_target(verify_eigen_and_cxx23_toolchain)
target_link_libraries(verify_eigen_and_cxx23_toolchain PRIVATE Eigen3::Eigen)
add_test(NAME verify_eigen_and_cxx23_toolchain
         COMMAND verify_eigen_and_cxx23_toolchain)

# The scenario and observation contract both sides of a cross-process comparison
# are driven from. It is a test contract, not a product library, so it lives here
# rather than under libs/.
add_library(orvd_test_contract STATIC
    contract/observation_stream.cc
    contract/scenario_definition.cc)
orvd_configure_first_party_target(orvd_test_contract)
target_include_directories(orvd_test_contract PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(orvd_test_contract PUBLIC Eigen3::Eigen)

add_executable(verify_scenario_and_observation_contract
    contract/verify_scenario_and_observation_contract.cc)
orvd_configure_first_party_target(verify_scenario_and_observation_contract)
target_link_libraries(verify_scenario_and_observation_contract
    PRIVATE orvd_test_contract)
add_test(NAME verify_scenario_and_observation_contract
         COMMAND verify_scenario_and_observation_contract)

# The comparison judge: it holds the required observation set and the tolerance
# rule, so that neither side of a comparison can certify itself.
add_library(orvd_test_comparison STATIC
    comparison/observation_comparator.cc
    comparison/required_observations.cc)
orvd_configure_first_party_target(orvd_test_comparison)
target_link_libraries(orvd_test_comparison PUBLIC orvd_test_contract)

add_executable(verify_observation_comparator
    comparison/verify_observation_comparator.cc)
orvd_configure_first_party_target(verify_observation_comparator)
target_link_libraries(verify_observation_comparator
    PRIVATE orvd_test_comparison)
add_test(NAME verify_observation_comparator
         COMMAND verify_observation_comparator)

# Drake-backed reference. Off by default: Drake is a comparison reference for the
# migration, not a dependency of anything this project ships. Turning it on must
# really find Drake, really build the targets and really register the test.
if(ORVD_BUILD_DRAKE_REFERENCE_TESTS)
    if(NOT UNIX)
        message(FATAL_ERROR
            "Drake reference tests currently require POSIX process primitives")
    endif()
    # Location comes from standard CMake package search; callers may provide
    # CMAKE_PREFIX_PATH or drake_DIR. No machine path is written into the
    # repository.
    #
    # PATH-derived prefixes are excluded for this search: an unrelated toolchain
    # on PATH (a conda environment, say) otherwise supplies its own build of a
    # dependency Drake also needs, whose directory then lands in the executable's
    # RUNPATH and takes the C++ runtime with it. The result is a binary that
    # links cleanly and cannot start.
    # Function scope, not set/unset around the call. Two reasons, both measured:
    # NO_SYSTEM_ENVIRONMENT_PATH as a find_package keyword constrains only the
    # outer search — drake-config.cmake's own find_dependency(fmt) and
    # find_dependency(spdlog) do not inherit it and resolve to whatever conda
    # environment is on PATH, which is how the C++ runtime got swapped out in the
    # first place. And unset() deletes the variable rather than restoring it, so a
    # caller who had set it deliberately would silently lose their setting.
    # Inside a function the assignment is local, transitive finds still see OFF,
    # and the caller's value is intact on return.
    function(orvd_find_drake_reference_package)
        set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH OFF)
        find_package(drake CONFIG REQUIRED)
        # Not `1.54.0 EXACT`: the installed Drake ships no drake-config-version
        # file, so the version only becomes known after the config file has been
        # loaded.
        if(NOT drake_VERSION VERSION_EQUAL "1.54.0")
            message(FATAL_ERROR
                "Drake reference tests require Drake 1.54.0; found ${drake_VERSION}")
        endif()
    endfunction()
    orvd_find_drake_reference_package()

    add_executable(emit_drake_reference_observations
        drake_reference/emit_drake_reference_observations.cc)
    orvd_configure_first_party_target(emit_drake_reference_observations)
    target_link_libraries(emit_drake_reference_observations
        PRIVATE orvd_test_contract drake::drake)

    add_executable(verify_cross_process_observation_comparison
        drake_reference/verify_cross_process_observation_comparison.cc)
    orvd_configure_first_party_target(verify_cross_process_observation_comparison)
    # The comparison never links Drake: it starts the reference as a separate
    # process and reads what it wrote.
    target_link_libraries(verify_cross_process_observation_comparison
        PRIVATE orvd_test_comparison)

    add_test(NAME verify_cross_process_observation_comparison
             COMMAND verify_cross_process_observation_comparison
                     $<TARGET_FILE:emit_drake_reference_observations>)

    # Measures the reference's kinematics cache invalidation semantics on the
    # pinned Drake. Reference-only probe: the cache-entry accessors are Drake
    # public API but must not shape any ORVD interface.
    add_executable(verify_drake_multibody_kinematics_cache_invalidation
        drake_reference/verify_drake_multibody_kinematics_cache_invalidation.cc)
    orvd_configure_first_party_target(
        verify_drake_multibody_kinematics_cache_invalidation)
    target_link_libraries(verify_drake_multibody_kinematics_cache_invalidation
        PRIVATE drake::drake)
    add_test(NAME verify_drake_multibody_kinematics_cache_invalidation
             COMMAND verify_drake_multibody_kinematics_cache_invalidation)
endif()
