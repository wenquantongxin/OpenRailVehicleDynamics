# The source-boundary analyzer is a development tool, not a shipped artifact, so
# Python is only ever required when testing is enabled. It builds its own
# synthetic source trees, so this check needs neither Drake nor a Drake clone.
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
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
