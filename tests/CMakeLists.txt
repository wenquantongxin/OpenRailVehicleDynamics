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
# test here for the real product targets. It is an architectural check on the
# CMake graph and best-effort by construction; see the module head for what it
# does not reach.
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

# The landed compile-frontier tool's synthetic self-check stays in the default
# suite. Its real-tree mode deliberately remains a development command: it
# recompiles every translation unit with its own fixed command line, so running
# it once per CMake configuration repeats the same long check without exercising
# that configuration's actual flags. The product OBJECT targets compile every
# landed translation unit with the configured flags, and the G28 all-object
# executable forces every resulting object through a final link. The real-tree
# tool still provides the independent sanitized-include and forbidden-symbol
# audit when invoked explicitly; see tools/drake_source_boundary/README.md.
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

# When the lazy evaluator calculates and when it does not, and what a failed
# calculation leaves behind. The failure paths carry the weight: a half-written
# buffer and a value computed from a state the calculator moved both look like
# ordinary success from the call site if they are handled wrongly.
add_executable(verify_multibody_cache_evaluation
    multibody_runtime/verify_multibody_cache_evaluation.cc)
orvd_configure_first_party_target(verify_multibody_cache_evaluation)
target_link_libraries(verify_multibody_cache_evaluation
    PRIVATE orvd_multibody_runtime)
add_test(NAME verify_multibody_cache_evaluation
         COMMAND verify_multibody_cache_evaluation)

# These five adapter headers are first-party code but are normally reached
# through vendored Drake headers. GCC and Clang inherit a system-header identity
# along that include chain, so merely placing the files on a normal `-I` root
# does not apply the first-party warning policy to them. Each translation unit
# includes one adapter header first: this both checks that the header is
# self-contained and makes its diagnostics first-party. It is a compile-only
# target, not an empty runtime test and not a member of the product archive.
add_library(verify_rigid_multibody_tree_adapter_headers_compile OBJECT
    header_diagnostics/compile_multibody_parameter_slot_allocator_header.cc
    header_diagnostics/compile_rigid_multibody_tree_cache_workspace_header.cc
    header_diagnostics/compile_rigid_multibody_tree_evaluation_context_header.cc
    header_diagnostics/compile_rigid_multibody_tree_evaluation_context_forward_declaration_header.cc
    header_diagnostics/compile_spatial_inertia_parameter_conversion_header.cc)
orvd_configure_first_party_target(
    verify_rigid_multibody_tree_adapter_headers_compile)
target_link_libraries(verify_rigid_multibody_tree_adapter_headers_compile
    PRIVATE orvd_rigid_multibody_tree)


# The tree's own caches, bound to real state and driven on a real model. An
# ordinary consumer: it links the product archive and takes from it only what it
# needs, which is how the archive will actually be used.
add_executable(verify_rigid_tree_cache_binding
    multibody_runtime/verify_rigid_tree_cache_binding.cc)
orvd_configure_first_party_target(verify_rigid_tree_cache_binding)
target_link_libraries(verify_rigid_tree_cache_binding
    PRIVATE orvd_rigid_multibody_tree)
add_test(NAME verify_rigid_tree_cache_binding
         COMMAND verify_rigid_tree_cache_binding)

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
                     $<TARGET_FILE:emit_drake_reference_observations>
                     $<TARGET_FILE:emit_orvd_kinematics_observations>)
    add_test(NAME verify_cross_process_comparison_rejects_same_emitter
             COMMAND ${CMAKE_COMMAND}
                     -DCOMPARISON_EXECUTABLE=$<TARGET_FILE:verify_cross_process_observation_comparison>
                     -DEMITTER_EXECUTABLE=$<TARGET_FILE:emit_drake_reference_observations>
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/drake_reference/verify_cross_process_comparison_rejects_same_emitter.cmake)

    # A check that only ever runs against something clean has never been shown to
    # fail. This runs it against four things it must refuse and asserts that
    # each refusal is the one expected: `WILL_FAIL` alone would take any non-zero
    # exit, so a typo that aborted the script early would read as success.
    #
    # The unresolvable fixture is a program the build linker can produce, but its
    # dependency has no recorded runtime search path. Neither the closure check
    # nor the default loader resolves it. A runtime environment could make that
    # neutral wrapper loadable and reveal a Drake behind it, which is why the
    # check refuses rather than reporting a walk it did not finish.
    add_library(orvd_neutral_wrapper SHARED orvd_candidate/neutral_wrapper.cc)
    orvd_configure_first_party_target(orvd_neutral_wrapper)
    add_executable(orvd_unresolvable_consumer
        orvd_candidate/unresolvable_consumer.cc)
    orvd_configure_first_party_target(orvd_unresolvable_consumer)
    target_link_libraries(orvd_unresolvable_consumer
        PRIVATE orvd_neutral_wrapper)
    set_target_properties(orvd_unresolvable_consumer
        PROPERTIES SKIP_BUILD_RPATH TRUE BUILD_RPATH "" INSTALL_RPATH "")

    # And the same thing wearing a name CMake reads as false. `if(<list>)` looks
    # at the contents, and `NOTFOUND` is one of the words that makes a list of
    # one item test false — so a check written that way would decide there were
    # no unresolved dependencies at all. An ordinary name cannot show that.
    add_library(orvd_false_like_wrapper SHARED orvd_candidate/neutral_wrapper.cc)
    orvd_configure_first_party_target(orvd_false_like_wrapper)
    set_target_properties(orvd_false_like_wrapper
        PROPERTIES PREFIX "" SUFFIX "" OUTPUT_NAME "NOTFOUND")
    add_executable(orvd_false_like_consumer
        orvd_candidate/unresolvable_consumer.cc)
    orvd_configure_first_party_target(orvd_false_like_consumer)
    target_link_libraries(orvd_false_like_consumer
        PRIVATE orvd_false_like_wrapper)
    set_target_properties(orvd_false_like_consumer
        PROPERTIES SKIP_BUILD_RPATH TRUE BUILD_RPATH "" INSTALL_RPATH "")

    add_test(NAME verify_candidate_runtime_dependency_check_discriminates
             COMMAND ${CMAKE_COMMAND}
                     -DCHECK_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/orvd_candidate/verify_candidate_runtime_dependencies_exclude_shared_drake.cmake
                     -DDRAKE_LOADING_EXECUTABLE=$<TARGET_FILE:emit_drake_reference_observations>
                     -DUNRESOLVABLE_EXECUTABLE=$<TARGET_FILE:orvd_unresolvable_consumer>
                     -DFALSE_LIKE_EXECUTABLE=$<TARGET_FILE:orvd_false_like_consumer>
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/orvd_candidate/verify_candidate_runtime_dependency_check_discriminates.cmake)

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

# Every object of the landed tree, forced into one link.
#
# A consumer of a static archive gets only the members that resolve a symbol it
# already needs, so the ordinary consumer above cannot show that every object
# links — a translation unit with an unresolved symbol nobody happens to call
# would sit in the archive unnoticed until some later consumer did call it. This
# target consumes the object libraries directly, so every object is in the link
# whether the program refers to it or not.
#
# It uses CMake's own object-library mechanism rather than a linker option:
# `--whole-archive` is GNU-specific, and a check that only fires on one toolchain
# is not a check on the others.
add_executable(verify_landed_tree_objects_all_link
    multibody_runtime/verify_landed_tree_objects_all_link.cc)
orvd_configure_first_party_target(verify_landed_tree_objects_all_link)
if(NOT ORVD_RIGID_MULTIBODY_TREE_OBJECT_OWNER_TARGETS)
    message(FATAL_ERROR
        "the rigid-tree archive did not publish its object-owner set, so the "
        "all-object link test cannot prove that archive's complete contents")
endif()
target_link_libraries(verify_landed_tree_objects_all_link
    PRIVATE ${ORVD_RIGID_MULTIBODY_TREE_OBJECT_OWNER_TARGETS}
            orvd_multibody_runtime)
add_test(NAME verify_landed_tree_objects_all_link
         COMMAND verify_landed_tree_objects_all_link)

# The smallest program that uses the tree the way a consumer will: build a
# model, finalize it, make its own evaluation context, put it in a configuration
# and ask for the kinematics. It links only the product archive.
add_executable(verify_minimal_rigid_tree_model
    multibody_runtime/verify_minimal_rigid_tree_model.cc)
orvd_configure_first_party_target(verify_minimal_rigid_tree_model)
target_link_libraries(verify_minimal_rigid_tree_model
    PRIVATE orvd_rigid_multibody_tree)
add_test(NAME verify_minimal_rigid_tree_model
         COMMAND verify_minimal_rigid_tree_model)

# What the modelling facade takes and what it refuses. It links only
# `orvd_multibody_model`: that target's dependency on the rigid tree is PRIVATE,
# so if this test compiles then the public headers really do name no vendored
# type — the check and the boundary are the same fact.
add_executable(verify_multibody_model_building
    multibody_model/verify_multibody_model_building.cc)
orvd_configure_first_party_target(verify_multibody_model_building)
target_link_libraries(verify_multibody_model_building
    PRIVATE orvd_multibody_model)
add_test(NAME verify_multibody_model_building
         COMMAND verify_multibody_model_building)

# Finalization and what a finalized model answers. Same linkage and the same
# reason: everything it asks for is asked through the public headers, so the
# coordinate ranges it checks are ones a consumer can actually obtain.
add_executable(verify_multibody_model_finalization
    multibody_model/verify_multibody_model_finalization.cc)
orvd_configure_first_party_target(verify_multibody_model_finalization)
target_link_libraries(verify_multibody_model_finalization
    PRIVATE orvd_multibody_model)
add_test(NAME verify_multibody_model_finalization
         COMMAND verify_multibody_model_finalization)

# Where the model says things are, against where the stated relations put them.
# Same linkage again: every expectation is written against the public headers,
# so a pose this test can obtain is a pose a consumer can obtain.
add_executable(verify_multibody_model_position_kinematics
    multibody_model/verify_multibody_model_position_kinematics.cc)
orvd_configure_first_party_target(verify_multibody_model_position_kinematics)
target_link_libraries(verify_multibody_model_position_kinematics
    PRIVATE orvd_multibody_model)
add_test(NAME verify_multibody_model_position_kinematics
         COMMAND verify_multibody_model_position_kinematics)

# Angular and frame-origin translational velocities, including point shifts,
# relative measurement, expression frames, branches, free bodies and reversed
# relations. Every expectation is built from elementary rigid-body formulas
# through the public first-party API.
add_executable(verify_multibody_model_spatial_kinematics
    multibody_model/verify_multibody_model_spatial_kinematics.cc)
orvd_configure_first_party_target(verify_multibody_model_spatial_kinematics)
target_link_libraries(verify_multibody_model_spatial_kinematics
    PRIVATE orvd_multibody_model)
add_test(NAME verify_multibody_model_spatial_kinematics
         COMMAND verify_multibody_model_spatial_kinematics)

# The candidate side of the cross-process comparison. It links the modelling
# facade and the shared scenario contract, which brings in the landed tree
# statically — that is the product. What it must never link is the installed
# `libdrake.so`: that library and the vendored copy export the same `drake::`
# symbols, so having both in one address space is an ODR violation whose likely
# symptom is a comparison that appears to pass. What checks that is the test
# below, on the built program: a configure-time check on the target graph would
# have to model every spelling of a dependency and still could not be final,
# because cmake_language(DEFER) can change the graph after it has run.
add_executable(emit_orvd_kinematics_observations
    orvd_candidate/emit_orvd_kinematics_observations.cc)
orvd_configure_first_party_target(emit_orvd_kinematics_observations)
target_link_libraries(emit_orvd_kinematics_observations
    PRIVATE orvd_multibody_model orvd_test_contract)
# The declared dynamic runtime closure of the built program. Anything it could
# not finish walking is a refusal, not a clean result. Static object provenance
# and ordinary link inputs are guarded separately.
add_test(NAME verify_candidate_runtime_dependencies_exclude_shared_drake
         COMMAND ${CMAKE_COMMAND}
                 -DCANDIDATE_EXECUTABLE=$<TARGET_FILE:emit_orvd_kinematics_observations>
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/orvd_candidate/verify_candidate_runtime_dependencies_exclude_shared_drake.cmake)
