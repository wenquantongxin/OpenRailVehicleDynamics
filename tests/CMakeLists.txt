# The source-boundary analyzer is a development tool, not a shipped artifact, so
# Python is only ever required when testing is enabled. It builds its own
# synthetic source trees, so this check needs neither Drake nor a Drake clone.
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
find_package(Git REQUIRED)
get_filename_component(orvd_test_toolchain_runtime_directory
                       "${CMAKE_CXX_COMPILER}" DIRECTORY)
add_test(NAME verify_dependency_source_declaration
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_SOURCE_DIR}/tools/package_distribution/verify_dependency_sources.py
                 --cmake ${CMAKE_COMMAND}
                 --git-executable ${GIT_EXECUTABLE}
                 --source-root ${CMAKE_SOURCE_DIR})
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

# Proves the OpenMP pragmas were compiled into real parallel semantics and the
# linked runtime forms real teams, using the product's two pragma shapes. A
# toolchain that drops the pragmas while linking an OpenMP runtime (Clang's
# -fopenmp=libgomp) passes every numerical test in this suite — the physics is
# bit-identical across team sizes — and fails only here.
add_executable(verify_openmp_parallel_execution
    toolchain/verify_openmp_parallel_execution.cc)
orvd_configure_first_party_target(verify_openmp_parallel_execution)
target_link_libraries(verify_openmp_parallel_execution
    PRIVATE OpenMP::OpenMP_CXX)
add_test(NAME verify_openmp_parallel_execution
         COMMAND verify_openmp_parallel_execution)

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
                     $<TARGET_FILE:emit_orvd_candidate_observations>)
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
                     "-DDRAKE_RUNTIME_DIRECTORIES=$<TARGET_FILE_DIR:emit_drake_reference_observations>;${orvd_test_toolchain_runtime_directory}"
                     -DTOOLCHAIN_RUNTIME_DIRECTORY=${orvd_test_toolchain_runtime_directory}
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

# The model-neutral Ball-RPY entry point in the same topology that first
# consumes it: a free axle bridge with a massive longitudinal bar outboard.
# Expectations reconstruct Rz*Ry*Rx, qdot=N(q)v and virtual power without
# exposing or calling a vendored joint type.
add_executable(verify_multibody_model_ball_rpy_joint
    multibody_model/verify_multibody_model_ball_rpy_joint.cc)
orvd_configure_first_party_target(verify_multibody_model_ball_rpy_joint)
target_link_libraries(verify_multibody_model_ball_rpy_joint
    PRIVATE orvd_multibody_model)
add_test(NAME verify_multibody_model_ball_rpy_joint
         COMMAND verify_multibody_model_ball_rpy_joint)

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

# qdot/v mappings, caller-owned spatial accelerations and the deliberately
# narrow kV body-point Jacobian. The Jacobian is checked column by column
# against central differences of the public pose query; acceleration is checked
# against its independent affine relation to that Jacobian.
add_executable(verify_multibody_model_differential_kinematics
    multibody_model/verify_multibody_model_differential_kinematics.cc)
orvd_configure_first_party_target(
    verify_multibody_model_differential_kinematics)
target_link_libraries(verify_multibody_model_differential_kinematics
    PRIVATE orvd_multibody_model)
add_test(NAME verify_multibody_model_differential_kinematics
         COMMAND verify_multibody_model_differential_kinematics)

# The public generalized mass matrix. Its complete operator is recovered from
# independently assembled rigid-body kinetic energy, with a separate analytic
# one-degree-of-freedom case and refusal atomicity at the public boundary.
add_executable(verify_multibody_model_mass_matrix
    multibody_model/verify_multibody_model_mass_matrix.cc)
orvd_configure_first_party_target(verify_multibody_model_mass_matrix)
target_link_libraries(verify_multibody_model_mass_matrix
    PRIVATE orvd_multibody_model)
add_test(NAME verify_multibody_model_mass_matrix
         COMMAND verify_multibody_model_mass_matrix)

add_executable(verify_multibody_model_inverse_dynamics
    multibody_model/verify_multibody_model_inverse_dynamics.cc)
orvd_configure_first_party_target(verify_multibody_model_inverse_dynamics)
target_link_libraries(verify_multibody_model_inverse_dynamics
    PRIVATE orvd_multibody_model)
add_test(NAME verify_multibody_model_inverse_dynamics
         COMMAND verify_multibody_model_inverse_dynamics)

add_executable(verify_multibody_model_forward_dynamics
    multibody_model/verify_multibody_model_forward_dynamics.cc)
orvd_configure_first_party_target(verify_multibody_model_forward_dynamics)
target_link_libraries(verify_multibody_model_forward_dynamics
    PRIVATE orvd_multibody_model)
add_test(NAME verify_multibody_model_forward_dynamics
         COMMAND verify_multibody_model_forward_dynamics)

# G40's immutable assembly description.  It links only the first-party
# modelling facade and proves that description construction observes finalized
# topology without creating a runtime context or workspace.
add_executable(verify_system_assembly_description
    system_assembly/verify_system_assembly_description.cc)
orvd_configure_first_party_target(verify_system_assembly_description)
target_link_libraries(verify_system_assembly_description
    PRIVATE orvd_forces orvd_system_assembly)
add_test(NAME verify_system_assembly_description
         COMMAND verify_system_assembly_description)

# G41's executable system: frozen ranges and component identity, exactly one
# authoritative multibody state per root context, and a colocated preallocated
# forward-dynamics workspace.  The same test closes the deferred context-local
# joint-damping write contract.
add_executable(verify_system_instance
    system_assembly/verify_system_instance.cc)
orvd_configure_first_party_target(verify_system_instance)
target_link_libraries(verify_system_instance PRIVATE orvd_system_assembly)
add_test(NAME verify_system_instance COMMAND verify_system_instance)

# G42's single static evaluation order.  The current admitted graph has no
# event-producing component, so unsupported event forms have no API to test;
# this executable verifies the real derivative operation and that both success
# and failure leave the accepted physical state untouched.
add_executable(verify_compiled_system_plan
    system_assembly/verify_compiled_system_plan.cc)
orvd_configure_first_party_target(verify_compiled_system_plan)
target_link_libraries(verify_compiled_system_plan PRIVATE orvd_system_assembly)
add_test(NAME verify_compiled_system_plan COMMAND verify_compiled_system_plan)

# G43's backend-neutral state/RHS seam. This does not instantiate or step a
# numerical backend; it verifies the dynamically sized contiguous mapping,
# component tolerances and the dedicated trial-context boundary used by G44.
add_executable(verify_system_rhs_bridge
    integrators/continuous_state_advancer_header_self_containment.cc
    integrators/system_rhs_bridge_header_self_containment.cc
    integrators/verify_system_rhs_bridge.cc)
orvd_configure_first_party_target(verify_system_rhs_bridge)
target_link_libraries(verify_system_rhs_bridge PRIVATE orvd_integrators)
add_test(NAME verify_system_rhs_bridge COMMAND verify_system_rhs_bridge)

# G44's real CVODE backend. Analytic systems exercise the public second-order
# default and the source-tree-only fifth-order identity, including actual CVODE
# order and reinitialization; the focused failure case verifies that a C
# callback never leaks a C++ exception or publishes a partial endpoint.
add_executable(verify_cvode_continuous_state_advancer
    integrators/cvode_continuous_state_advancer_header_self_containment.cc
    integrators/verify_cvode_continuous_state_advancer.cc)
orvd_configure_first_party_target(verify_cvode_continuous_state_advancer)
target_include_directories(verify_cvode_continuous_state_advancer
    PRIVATE ${PROJECT_SOURCE_DIR}/libs/integrators/src)
target_link_libraries(verify_cvode_continuous_state_advancer
    PRIVATE orvd_integrators)
add_test(NAME verify_cvode_continuous_state_advancer
         COMMAND verify_cvode_continuous_state_advancer)

# G45's accepted/trial transaction boundary.  A real damped multibody system
# runs through CVODE, a singular articulated inertia exercises post-RHS failure,
# and the system-level atomic setter covers an invalid final acceptance without
# introducing a fake numerical backend or a production fault hook.
add_executable(verify_system_continuous_state_advancer
    integrators/system_continuous_state_advancer_header_self_containment.cc
    integrators/verify_system_continuous_state_advancer.cc)
orvd_configure_first_party_target(verify_system_continuous_state_advancer)
target_link_libraries(verify_system_continuous_state_advancer
    PRIVATE orvd_integrators)
add_test(NAME verify_system_continuous_state_advancer
         COMMAND verify_system_continuous_state_advancer)

# G46's installed-package qualification.  This test moves an actual installation
# to a second prefix, configures an independent public-header-only consumer,
# advances a damped rotor with the real CVODE backend, and then walks the
# resulting executable's runtime dependency closure.  The sibling marker
# executable proves that the same closure check rejects a real shared library
# whose file identity is Drake.  It is a native execution test, not a cross-build
# substitute for running on the target platform.
if(NOT CMAKE_CROSSCOMPILING)
    if(CMAKE_CONFIGURATION_TYPES)
        set(orvd_installation_multi_config TRUE)
        set(orvd_installation_build_config "$<CONFIG>")
    else()
        set(orvd_installation_multi_config FALSE)
        set(orvd_installation_build_config "${CMAKE_BUILD_TYPE}")
    endif()
    string(REPLACE ";" "\\;" orvd_installation_dependency_prefix_path
           "${CMAKE_PREFIX_PATH}")
    set(orvd_installation_openmp_initial_cache_content "")
    foreach(openmp_string_variable
            OpenMP_CXX_FLAGS
            OpenMP_CXX_LIB_NAMES)
        if(DEFINED ${openmp_string_variable} AND
           NOT "${${openmp_string_variable}}" STREQUAL "")
            string(APPEND orvd_installation_openmp_initial_cache_content
                   "set(${openmp_string_variable} "
                   "[==[${${openmp_string_variable}}]==] "
                   "CACHE STRING \"\" FORCE)\n")
        endif()
    endforeach()
    if(DEFINED OpenMP_CXX_INCLUDE_DIR AND
       NOT "${OpenMP_CXX_INCLUDE_DIR}" STREQUAL "")
        string(APPEND orvd_installation_openmp_initial_cache_content
               "set(OpenMP_CXX_INCLUDE_DIR "
               "[==[${OpenMP_CXX_INCLUDE_DIR}]==] "
               "CACHE PATH \"\" FORCE)\n")
    endif()
    foreach(openmp_library IN LISTS OpenMP_CXX_LIB_NAMES)
        set(openmp_library_variable "OpenMP_${openmp_library}_LIBRARY")
        if(DEFINED ${openmp_library_variable} AND
           NOT "${${openmp_library_variable}}" STREQUAL "")
            string(APPEND orvd_installation_openmp_initial_cache_content
                   "set(${openmp_library_variable} "
                   "[==[${${openmp_library_variable}}]==] "
                   "CACHE FILEPATH \"\" FORCE)\n")
        endif()
    endforeach()
    set(orvd_installation_openmp_initial_cache
        "${CMAKE_CURRENT_BINARY_DIR}/installation/openmp-initial-cache.cmake")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/installation")
    file(WRITE "${orvd_installation_openmp_initial_cache}"
         "${orvd_installation_openmp_initial_cache_content}")
    add_test(NAME verify_relocated_install_consumer
        COMMAND ${CMAKE_COMMAND}
            -DORVD_BUILD_DIR=${CMAKE_BINARY_DIR}
            -DORVD_SOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DINSTALLATION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/installation
            -DWORK_ROOT=${CMAKE_CURRENT_BINARY_DIR}/installation/$<CONFIG>
            -DCMAKE_GENERATOR_NAME=${CMAKE_GENERATOR}
            -DCMAKE_GENERATOR_PLATFORM_NAME=${CMAKE_GENERATOR_PLATFORM}
            -DCMAKE_GENERATOR_TOOLSET_NAME=${CMAKE_GENERATOR_TOOLSET}
            -DCMAKE_CXX_COMPILER_PATH=${CMAKE_CXX_COMPILER}
            -DCMAKE_MAKE_PROGRAM_PATH=${CMAKE_MAKE_PROGRAM}
            -DBUILD_CONFIG=${orvd_installation_build_config}
            -DEXECUTABLE_SUFFIX=${CMAKE_EXECUTABLE_SUFFIX}
            -DINSTALL_DATADIR=${CMAKE_INSTALL_DATADIR}
            -DMULTI_CONFIG=${orvd_installation_multi_config}
            -DDEPENDENCY_PREFIX_PATH=${orvd_installation_dependency_prefix_path}
            -DEIGEN3_CONFIG_DIR=${Eigen3_DIR}
            -DFMT_CONFIG_DIR=${fmt_DIR}
            -DSUNDIALS_CONFIG_DIR=${SUNDIALS_DIR}
            -DOPENMP_INITIAL_CACHE=${orvd_installation_openmp_initial_cache}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/installation/verify_relocated_install_consumer.cmake)
    set_tests_properties(verify_relocated_install_consumer
        PROPERTIES RUN_SERIAL TRUE)
endif()

# The candidate side of the cross-process comparison. It links the modelling
# facade and the shared scenario contract, which brings in the landed tree
# statically — that is the product. What it must never link is the installed
# `libdrake.so`: that library and the vendored copy export the same `drake::`
# symbols, so having both in one address space is an ODR violation whose likely
# symptom is a comparison that appears to pass. What checks that is the test
# below, on the built program: a configure-time check on the target graph would
# have to model every spelling of a dependency and still could not be final,
# because cmake_language(DEFER) can change the graph after it has run.
add_executable(emit_orvd_candidate_observations
    orvd_candidate/emit_orvd_candidate_observations.cc)
orvd_configure_first_party_target(emit_orvd_candidate_observations)
target_link_libraries(emit_orvd_candidate_observations
    PRIVATE orvd_multibody_model orvd_test_contract)
# The declared dynamic runtime closure of the built program. Anything it could
# not finish walking is a refusal, not a clean result. Static object provenance
# and ordinary link inputs are guarded separately.
add_test(NAME verify_candidate_runtime_dependencies_exclude_shared_drake
         COMMAND ${CMAKE_COMMAND}
                 -DCANDIDATE_EXECUTABLE=$<TARGET_FILE:emit_orvd_candidate_observations>
                 "-DRUNTIME_SEARCH_DIRECTORIES=$<TARGET_FILE_DIR:emit_orvd_candidate_observations>;${orvd_test_toolchain_runtime_directory}"
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/orvd_candidate/verify_candidate_runtime_dependencies_exclude_shared_drake.cmake)

# G47's original line layer uses three executables, one per group of completion
# gates, each also compiling the public headers it owns as standalone
# translation units. The dedicated vertical-profile value object adds one
# focused executable without widening the older geometry fixtures.
# The orientation and derivative identities use dedicated non-degenerate
# fixtures carrying non-zero curvature, superelevation, grade and heading at
# the same station. Refusal and projection fixtures intentionally vary those
# quantities to isolate their own contracts.
add_executable(verify_track_geometry_invariants
    track_geometry/track_geometry_header_self_containment.cc
    track_geometry/track_geometry_segments_header_self_containment.cc
    track_geometry/track_inertial_frame_header_self_containment.cc
    track_geometry/verify_track_geometry_invariants.cc)
orvd_configure_first_party_target(verify_track_geometry_invariants)
target_include_directories(verify_track_geometry_invariants
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_track_geometry_invariants
    PRIVATE orvd_track_geometry)
add_test(NAME verify_track_geometry_invariants
         COMMAND verify_track_geometry_invariants)

add_executable(verify_track_vertical_profile
    track_geometry/track_vertical_profile_header_self_containment.cc
    track_geometry/verify_track_vertical_profile.cc)
orvd_configure_first_party_target(verify_track_vertical_profile)
target_link_libraries(verify_track_vertical_profile
    PRIVATE orvd_track_geometry)
add_test(NAME verify_track_vertical_profile
         COMMAND verify_track_vertical_profile)

add_executable(verify_track_frame_orientation
    track_geometry/track_frame_pose_header_self_containment.cc
    track_geometry/verify_track_frame_orientation.cc)
orvd_configure_first_party_target(verify_track_frame_orientation)
target_include_directories(verify_track_frame_orientation
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_track_frame_orientation
    PRIVATE orvd_track_geometry)
add_test(NAME verify_track_frame_orientation
         COMMAND verify_track_frame_orientation)

add_executable(verify_track_station_projection
    track_geometry/track_station_projection_header_self_containment.cc
    track_geometry/verify_track_station_projection.cc)
orvd_configure_first_party_target(verify_track_station_projection)
target_include_directories(verify_track_station_projection
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_track_station_projection
    PRIVATE orvd_track_geometry)
add_test(NAME verify_track_station_projection
         COMMAND verify_track_station_projection)

# G48's strict human-authored configuration boundary. One non-degenerate record
# exercises both admitted segment shapes and a seam; the refusal table changes
# one property at a time. The JSON parser remains private to the product library.
add_executable(verify_track_geometry_json_loading
    configuration/load_track_geometry_header_self_containment.cc
    configuration/verify_track_geometry_json_loading.cc)
orvd_configure_first_party_target(verify_track_geometry_json_loading)
target_link_libraries(verify_track_geometry_json_loading
    PRIVATE orvd_configuration)
add_test(NAME verify_track_geometry_json_loading
         COMMAND verify_track_geometry_json_loading
            ${CMAKE_SOURCE_DIR}/track_library/geometries/straight_level_2000m.json
            ${CMAKE_SOURCE_DIR}/track_library/geometries/r300_centerline_superelevation_1100m.json
            ${CMAKE_SOURCE_DIR}/track_library/geometries/chunshen_station.json
            ${CMAKE_SOURCE_DIR}/track_library/geometries/r600_centerline_superelevation_1100m.json
            ${CMAKE_SOURCE_DIR}/track_library/geometries/r800_centerline_superelevation_1100m.json
            ${CMAKE_SOURCE_DIR}/track_library/geometries/r1000_centerline_superelevation_300m.json
            ${CMAKE_SOURCE_DIR}/track_library/geometries/straight_level_1100m.json
            ${CMAKE_CURRENT_BINARY_DIR}/configuration-fixtures)

# G52a's profile layer: the two interpolants a surface is read through, the
# qualified equal-arc wheel-profile preparation, and the shared roll-yaw-pitch
# kinematics. Every gate is a property the implementation claims, not a
# remembered table of numbers.
add_executable(verify_profile_interpolants
    wheel_rail_contact/natural_cubic_spline_header_self_containment.cc
    wheel_rail_contact/monotone_cubic_interpolant_header_self_containment.cc
    wheel_rail_contact/verify_profile_interpolants.cc)
orvd_configure_first_party_target(verify_profile_interpolants)
target_link_libraries(verify_profile_interpolants PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_profile_interpolants COMMAND verify_profile_interpolants)

# G56's immutable two-channel station field. The channels deliberately use
# different non-uniform grids and domains, so the test reaches all four typed
# values without turning the qualified AAR6 input into a numerical gold table.
add_executable(verify_track_irregularity_field
    wheel_rail_contact/track_irregularity_field_header_self_containment.cc
    wheel_rail_contact/verify_track_irregularity_field.cc)
orvd_configure_first_party_target(verify_track_irregularity_field)
target_link_libraries(verify_track_irregularity_field
    PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_track_irregularity_field
         COMMAND verify_track_irregularity_field)

# The model-neutral AAR generator is checked by formula properties, finite
# placement invariants and a direct DFT projection of full-amplitude samples.
# The wheel-rail field edge proves the required crop-before-spline boundary.
add_executable(verify_aar_track_irregularity_generator
    track_irregularity/aar_track_irregularity_generator_header_self_containment.cc
    track_irregularity/verify_aar_track_irregularity_generator.cc)
orvd_configure_first_party_target(verify_aar_track_irregularity_generator)
target_link_libraries(verify_aar_track_irregularity_generator
    PRIVATE orvd_track_irregularity orvd_wheel_rail_contact)
add_test(NAME verify_aar_track_irregularity_generator
         COMMAND verify_aar_track_irregularity_generator)

add_executable(verify_wheel_profile_preprocessing
    wheel_rail_contact/profile_points_header_self_containment.cc
    wheel_rail_contact/profile_arc_length_header_self_containment.cc
    wheel_rail_contact/wheel_profile_preprocessing_header_self_containment.cc
    wheel_rail_contact/verify_wheel_profile_preprocessing.cc)
orvd_configure_first_party_target(verify_wheel_profile_preprocessing)
target_link_libraries(verify_wheel_profile_preprocessing
    PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_wheel_profile_preprocessing
         COMMAND verify_wheel_profile_preprocessing)

add_executable(verify_wheel_rail_pose
    wheel_rail_contact/rail_gauge_datum_header_self_containment.cc
    wheel_rail_contact/wheel_rail_pose_header_self_containment.cc
    wheel_rail_contact/verify_wheel_rail_pose.cc)
orvd_configure_first_party_target(verify_wheel_rail_pose)
target_link_libraries(verify_wheel_rail_pose PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_wheel_rail_pose COMMAND verify_wheel_rail_pose)

add_executable(verify_contact_wrench
    wheel_rail_contact/contact_wrench_header_self_containment.cc
    wheel_rail_contact/verify_contact_wrench.cc)
orvd_configure_first_party_target(verify_contact_wrench)
target_link_libraries(verify_contact_wrench PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_contact_wrench COMMAND verify_contact_wrench)

add_executable(verify_kalker_coefficient_table
    wheel_rail_contact/kalker_coefficient_table_header_self_containment.cc
    wheel_rail_contact/verify_kalker_coefficient_table.cc)
orvd_configure_first_party_target(verify_kalker_coefficient_table)
target_link_libraries(verify_kalker_coefficient_table
    PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_kalker_coefficient_table
         COMMAND verify_kalker_coefficient_table)

add_executable(verify_contact_geometry
    wheel_rail_contact/contact_geometry_header_self_containment.cc
    wheel_rail_contact/verify_contact_geometry.cc)
orvd_configure_first_party_target(verify_contact_geometry)
target_link_libraries(verify_contact_geometry PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_contact_geometry COMMAND verify_contact_geometry)

add_executable(verify_normal_contact_force
    wheel_rail_contact/normal_contact_force_header_self_containment.cc
    wheel_rail_contact/verify_normal_contact_force.cc)
orvd_configure_first_party_target(verify_normal_contact_force)
target_link_libraries(verify_normal_contact_force PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_normal_contact_force COMMAND verify_normal_contact_force)

add_executable(verify_contact_creepage
    wheel_rail_contact/contact_creepage_header_self_containment.cc
    wheel_rail_contact/verify_contact_creepage.cc)
orvd_configure_first_party_target(verify_contact_creepage)
target_link_libraries(verify_contact_creepage PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_contact_creepage COMMAND verify_contact_creepage)

add_executable(verify_tangential_contact_force
    wheel_rail_contact/tangential_contact_force_header_self_containment.cc
    wheel_rail_contact/verify_tangential_contact_force.cc)
orvd_configure_first_party_target(verify_tangential_contact_force)
target_link_libraries(verify_tangential_contact_force PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_tangential_contact_force COMMAND verify_tangential_contact_force)

add_executable(verify_wheel_rail_contact_model
    wheel_rail_contact/wheel_rail_contact_model_header_self_containment.cc
    wheel_rail_contact/verify_wheel_rail_contact_model.cc)
orvd_configure_first_party_target(verify_wheel_rail_contact_model)
target_link_libraries(verify_wheel_rail_contact_model PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_wheel_rail_contact_model
         COMMAND verify_wheel_rail_contact_model)

add_executable(verify_roll_yaw_pitch
    wheel_rail_contact/roll_yaw_pitch_header_self_containment.cc
    wheel_rail_contact/verify_roll_yaw_pitch.cc)
orvd_configure_first_party_target(verify_roll_yaw_pitch)
target_link_libraries(verify_roll_yaw_pitch
    PRIVATE orvd_wheel_rail_contact)
add_test(NAME verify_roll_yaw_pitch COMMAND verify_roll_yaw_pitch)

# The product record and the local interoperability path meet here: a profile
# must survive a trip out to the reference format and back with its meaning
# intact, and every preprocessing step the reader does not implement must be
# refused rather than ignored.
add_executable(verify_profile_asset_interoperability
    configuration/load_profile_points_header_self_containment.cc
    configuration/verify_profile_asset_interoperability.cc)
orvd_configure_first_party_target(verify_profile_asset_interoperability)
target_link_libraries(verify_profile_asset_interoperability
    PRIVATE orvd_configuration orvd_profile_conversion)
add_test(NAME verify_profile_asset_interoperability
         COMMAND verify_profile_asset_interoperability
             ${CMAKE_CURRENT_BINARY_DIR}/profile-fixtures)

# G56's strict logical binding: one field manifest resolves two independently
# gridded series from an explicit data root and returns an object that no longer
# depends on the files. The real installed AAR6 asset is exercised by the
# relocated consumer below rather than copied into this rejection table.
add_executable(verify_track_irregularity_field_loading
    configuration/load_track_irregularity_field_header_self_containment.cc
    configuration/verify_track_irregularity_field_loading.cc)
orvd_configure_first_party_target(verify_track_irregularity_field_loading)
target_link_libraries(verify_track_irregularity_field_loading
    PRIVATE orvd_configuration)
add_test(NAME verify_track_irregularity_field_loading
         COMMAND verify_track_irregularity_field_loading
             ${CMAKE_SOURCE_DIR}
             ${CMAKE_CURRENT_BINARY_DIR}/track-irregularity-fixtures)

# DEC-043's closed source choice: the historical loader remains unchanged,
# while a generated PSD realization is cropped to its placement boundaries and
# converted to the same immutable field consumed by both vehicle factories.
add_executable(verify_track_irregularity_field_source_resolution
    configuration/resolve_track_irregularity_field_header_self_containment.cc
    configuration/verify_track_irregularity_field_source_resolution.cc)
orvd_configure_first_party_target(
    verify_track_irregularity_field_source_resolution)
target_link_libraries(verify_track_irregularity_field_source_resolution
    PRIVATE orvd_configuration)
add_test(NAME verify_track_irregularity_field_source_resolution
         COMMAND verify_track_irregularity_field_source_resolution
             ${CMAKE_SOURCE_DIR})

# G52c's real asset and personality boundary. The test loads the same JSON a
# relocated consumer receives, checks the closed logical identities and derived
# gauge geometry, then evaluates both sides after preparing their workspaces.
add_executable(verify_gz18_wheel_rail_contact_binding
    configuration/gz18_wheel_rail_contact_header_self_containment.cc
    configuration/verify_gz18_wheel_rail_contact_binding.cc)
orvd_configure_first_party_target(verify_gz18_wheel_rail_contact_binding)
target_include_directories(verify_gz18_wheel_rail_contact_binding
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_gz18_wheel_rail_contact_binding
    PRIVATE orvd_configuration)
add_test(NAME verify_gz18_wheel_rail_contact_binding
         COMMAND verify_gz18_wheel_rail_contact_binding
             ${CMAKE_SOURCE_DIR}
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/startup_states/moving_startup_60kmh.json
             ${CMAKE_CURRENT_BINARY_DIR}/gz18-contact-fixtures)

# G68 resolves the IRW logical identities to its S1002 wheel and the shared
# UIC60 rail, checks the common equal-arc personality, then reaches a real
# static contact without inventing G69's independent-wheel body kinematics.
add_executable(verify_irw_wheel_rail_contact_binding
    configuration/irw_wheel_rail_contact_header_self_containment.cc
    configuration/verify_irw_wheel_rail_contact_binding.cc)
orvd_configure_first_party_target(verify_irw_wheel_rail_contact_binding)
target_include_directories(verify_irw_wheel_rail_contact_binding
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_irw_wheel_rail_contact_binding
    PRIVATE orvd_configuration)
add_test(NAME verify_irw_wheel_rail_contact_binding
         COMMAND verify_irw_wheel_rail_contact_binding
             ${CMAKE_SOURCE_DIR}
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/startup_states/moving_startup_60kmh.json
             ${CMAKE_CURRENT_BINARY_DIR}/irw-contact-fixtures)

# G69/G73 connect the real H3 IRW state to four axle-bridge projection
# carriers, eight independently rotating wheel bodies and eight held active
# torque channels. One direct-state gate checks the complete 157-state RHS,
# fixed wheel targets, per-wheel independence, accepted-to-trial local-data
# synchronization and deterministic 1/2/4/8-worker execution without advancing
# time.
add_executable(verify_irw_contact_rhs
    configuration/verify_irw_contact_rhs.cc)
orvd_configure_first_party_target(verify_irw_contact_rhs)
target_include_directories(verify_irw_contact_rhs
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_irw_contact_rhs
    PRIVATE orvd_configuration orvd_integrators OpenMP::OpenMP_CXX)
add_test(NAME verify_irw_contact_rhs
         COMMAND verify_irw_contact_rhs
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/vehicle_definition.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/startup_states/moving_startup_60kmh.json
             ${CMAKE_SOURCE_DIR}/track_library/geometries/r300_centerline_superelevation_1100m.json
             ${CMAKE_SOURCE_DIR})
set_tests_properties(verify_irw_contact_rhs PROPERTIES PROCESSORS 8)

# G76 binds the same closed IRW topology to the 100 Hz full-state controller
# and torque conditioner. The real H3/R300/AAR5 gate performs the start-up
# double update before backend construction, then executes one synchronized
# non-terminal event and one terminal audit without a generic event framework.
add_executable(verify_irw_full_state_control_event_session
    configuration/irw_full_state_control_event_session_header_self_containment.cc
    configuration/verify_irw_full_state_control_event_session.cc)
orvd_configure_first_party_target(
    verify_irw_full_state_control_event_session)
target_include_directories(verify_irw_full_state_control_event_session
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_irw_full_state_control_event_session
    PRIVATE orvd_configuration orvd_integrators OpenMP::OpenMP_CXX)
add_test(NAME verify_irw_full_state_control_event_session
         COMMAND verify_irw_full_state_control_event_session
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/vehicle_definition.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/startup_states/moving_startup_60kmh.json
             ${CMAKE_SOURCE_DIR}/track_library/geometries/r300_centerline_superelevation_1100m.json
             ${CMAKE_SOURCE_DIR}
             ${CMAKE_SOURCE_DIR}/controller_library/irw/irw_r300_v60_full_state_wheel_speed_guidance_controller.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/drive_torque_conditioners/irw_reference_wheel_drive_torque_conditioner.json)
set_tests_properties(verify_irw_full_state_control_event_session PROPERTIES
    PROCESSORS 8)

# The longitudinal cruise binding observes one forward circumferential-speed
# mean from the closed eight-wheel IRW topology and commits one common PI
# request through the existing conditioner at each 100 Hz event. A 100 ms real
# assembled consumer checks the two explicit wheel-rate sign conventions,
# equal requests, atomic U0, backend synchronization and a finite terminal RHS.
add_executable(verify_irw_longitudinal_cruise_event_session
    configuration/irw_longitudinal_cruise_event_session_header_self_containment.cc
    configuration/verify_irw_longitudinal_cruise_event_session.cc)
orvd_configure_first_party_target(
    verify_irw_longitudinal_cruise_event_session)
target_include_directories(verify_irw_longitudinal_cruise_event_session
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_irw_longitudinal_cruise_event_session
    PRIVATE orvd_configuration orvd_integrators OpenMP::OpenMP_CXX)
add_test(NAME verify_irw_longitudinal_cruise_event_session
         COMMAND verify_irw_longitudinal_cruise_event_session
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/vehicle_definition.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/startup_states/moving_startup_60kmh.json
             ${CMAKE_SOURCE_DIR}/track_library/geometries/straight_level_2000m.json
             ${CMAKE_SOURCE_DIR}
             ${CMAKE_SOURCE_DIR}/controller_library/irw/irw_v60_common_wheel_speed_cruise_controller.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/drive_torque_conditioners/irw_reference_wheel_drive_torque_conditioner.json)
set_tests_properties(verify_irw_longitudinal_cruise_event_session PROPERTIES
    PROCESSORS 8)

# G53's complete GZ18 direct RHS and G54's accepted-step projection history.
# The same installed assets drive both: four wheelsets share four local branch
# hints across eight interfaces, and CVODE advances those hints at its internal
# accepted endpoints without a second synthetic vehicle.
add_executable(verify_gz18_contact_rhs
    configuration/assembled_vehicle_contact_scenario_header_self_containment.cc
    forces/wheel_rail_contact_force_plan_header_self_containment.cc
    wheel_rail_contact/wheel_rail_contact_runtime_personality_header_self_containment.cc
    configuration/verify_gz18_contact_rhs.cc)
orvd_configure_first_party_target(verify_gz18_contact_rhs)
target_include_directories(verify_gz18_contact_rhs
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_gz18_contact_rhs
    PRIVATE orvd_configuration orvd_integrators OpenMP::OpenMP_CXX)
add_test(NAME verify_gz18_contact_rhs
         COMMAND verify_gz18_contact_rhs
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/vehicle_definition.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/startup_states/moving_startup_60kmh.json
             ${CMAKE_SOURCE_DIR}/track_library/geometries/straight_level_2000m.json
             ${CMAKE_SOURCE_DIR})
set_tests_properties(verify_gz18_contact_rhs PROPERTIES PROCESSORS 8)

# G55's first complete passive longitudinal slice. One public CVODE advance
# returns the P035-aligned 101-state batch; a private observation context then
# replays those states through the same real eight-interface contact plan.
add_executable(verify_gz18_passive_short_window
    configuration/verify_gz18_passive_short_window.cc)
orvd_configure_first_party_target(verify_gz18_passive_short_window)
target_include_directories(verify_gz18_passive_short_window
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_gz18_passive_short_window
    PRIVATE orvd_configuration orvd_integrators)
add_test(NAME verify_gz18_passive_short_window
         COMMAND verify_gz18_passive_short_window
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/vehicle_definition.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/startup_states/moving_startup_60kmh.json
             ${CMAKE_SOURCE_DIR}/track_library/geometries/straight_level_2000m.json
             ${CMAKE_SOURCE_DIR})

# G58's non-installed qualification leaf. A 30 ms real GZ18 run with only its
# two endpoints exercises multiple dense Jacobians, observation projection across a
# sample displacement larger than the RHS's one-centimetre local window, the
# formal s_ref=0 boundary summary and directory-level publication. The 10/20 s
# science windows remain explicit local qualifications and never enter the
# default CTest suite.
add_executable(
    verify_gz18_straight_aar6_v60_passive_dense_output_and_publication
    dynamics_qualification/verify_gz18_straight_aar6_v60_passive_dense_output_and_publication.cc)
orvd_configure_first_party_target(
    verify_gz18_straight_aar6_v60_passive_dense_output_and_publication)
target_link_libraries(
    verify_gz18_straight_aar6_v60_passive_dense_output_and_publication
    PRIVATE orvd_dynamics_qualification_support OpenMP::OpenMP_CXX
            nlohmann_json::nlohmann_json)
add_test(
    NAME verify_gz18_straight_aar6_v60_passive_dense_output_and_publication
    COMMAND verify_gz18_straight_aar6_v60_passive_dense_output_and_publication
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/vehicle_definition.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/startup_states/moving_startup_60kmh.json
             ${CMAKE_SOURCE_DIR}/track_library/geometries/straight_level_1100m.json
             ${CMAKE_SOURCE_DIR}
             aar6_irregularity
             ${CMAKE_CURRENT_BINARY_DIR}/gz18-straight-aar6-v60-passive-artifact-fixtures)
set_tests_properties(
    verify_gz18_straight_aar6_v60_passive_dense_output_and_publication
    PROPERTIES
    PROCESSORS 16)

add_executable(verify_irw_dynamics_qualification
    dynamics_qualification/verify_irw_dynamics_qualification.cc)
orvd_configure_first_party_target(verify_irw_dynamics_qualification)
target_link_libraries(verify_irw_dynamics_qualification
    PRIVATE orvd_dynamics_qualification_support OpenMP::OpenMP_CXX
            nlohmann_json::nlohmann_json)
add_test(NAME verify_irw_dynamics_qualification
         COMMAND verify_irw_dynamics_qualification
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/vehicle_definition.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/startup_states/moving_startup_60kmh.json
             ${CMAKE_SOURCE_DIR}/track_library/geometries/r300_centerline_superelevation_1100m.json
             ${CMAKE_SOURCE_DIR}
             ${CMAKE_SOURCE_DIR}/controller_library/irw/irw_r300_v60_full_state_wheel_speed_guidance_controller.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/drive_torque_conditioners/irw_reference_wheel_drive_torque_conditioner.json
             ${CMAKE_CURRENT_BINARY_DIR}/irw-dynamics-qualification-fixtures)
set_tests_properties(verify_irw_dynamics_qualification PROPERTIES
    PROCESSORS 16)

add_executable(verify_vehicle_definition_loading
    configuration/vehicle_definition_header_self_containment.cc
    configuration/load_vehicle_definition_header_self_containment.cc
    configuration/verify_vehicle_definition_loading.cc)
orvd_configure_first_party_target(verify_vehicle_definition_loading)
target_link_libraries(verify_vehicle_definition_loading
    PRIVATE orvd_configuration)
add_test(NAME verify_vehicle_definition_loading
         COMMAND verify_vehicle_definition_loading
             ${CMAKE_CURRENT_BINARY_DIR}/vehicle-definition-fixtures)

add_executable(verify_gz18_vehicle_assembly
    vehicle_library/assemble_vehicle_multibody_model_header_self_containment.cc
    vehicle_library/verify_gz18_vehicle_assembly.cc)
orvd_configure_first_party_target(verify_gz18_vehicle_assembly)
target_link_libraries(verify_gz18_vehicle_assembly
    PRIVATE orvd_configuration orvd_system_assembly)
add_test(NAME verify_gz18_vehicle_assembly
         COMMAND verify_gz18_vehicle_assembly
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/vehicle_definition.json)

add_executable(verify_irw_vehicle_assembly
    vehicle_library/verify_irw_vehicle_assembly.cc)
orvd_configure_first_party_target(verify_irw_vehicle_assembly)
target_link_libraries(verify_irw_vehicle_assembly
    PRIVATE orvd_configuration orvd_system_assembly)
add_test(NAME verify_irw_vehicle_assembly
         COMMAND verify_irw_vehicle_assembly
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/vehicle_definition.json)

add_executable(verify_vehicle_force_elements
    forces/vehicle_force_elements_header_self_containment.cc
    forces/vehicle_force_plan_header_self_containment.cc
    forces/verify_vehicle_force_elements.cc)
orvd_configure_first_party_target(verify_vehicle_force_elements)
target_link_libraries(verify_vehicle_force_elements
    PRIVATE orvd_forces)
add_test(NAME verify_vehicle_force_elements
         COMMAND verify_vehicle_force_elements)

add_executable(verify_half_angle_midpoint_roll_pitch_yaw_bushing
    forces/verify_half_angle_midpoint_roll_pitch_yaw_bushing.cc)
orvd_configure_first_party_target(
    verify_half_angle_midpoint_roll_pitch_yaw_bushing)
target_link_libraries(verify_half_angle_midpoint_roll_pitch_yaw_bushing
    PRIVATE orvd_forces)
add_test(NAME verify_half_angle_midpoint_roll_pitch_yaw_bushing
         COMMAND verify_half_angle_midpoint_roll_pitch_yaw_bushing)

add_executable(verify_independent_wheel_active_torque_plan
    forces/independent_wheel_active_torque_plan_header_self_containment.cc
    forces/verify_independent_wheel_active_torque_plan.cc)
orvd_configure_first_party_target(
    verify_independent_wheel_active_torque_plan)
target_link_libraries(verify_independent_wheel_active_torque_plan
    PRIVATE orvd_forces)
add_test(NAME verify_independent_wheel_active_torque_plan
         COMMAND verify_independent_wheel_active_torque_plan)

# G74's pure eight-channel command conditioner and its strict first-party
# asset. One analytic batch distinguishes both source direction tables,
# interpolation, deadband, low/high-speed handling, magnitude limiting and
# signed cross-zero slew memory without introducing an event scheduler.
add_executable(verify_wheel_drive_torque_command_conditioner
    actuation/wheel_drive_torque_command_conditioner_header_self_containment.cc
    configuration/load_wheel_drive_torque_command_conditioner_header_self_containment.cc
    actuation/verify_wheel_drive_torque_command_conditioner.cc)
orvd_configure_first_party_target(
    verify_wheel_drive_torque_command_conditioner)
target_include_directories(verify_wheel_drive_torque_command_conditioner
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(verify_wheel_drive_torque_command_conditioner
    PRIVATE orvd_configuration)
add_test(NAME verify_wheel_drive_torque_command_conditioner
         COMMAND verify_wheel_drive_torque_command_conditioner
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/drive_torque_conditioners/irw_reference_wheel_drive_torque_conditioner.json
             ${CMAKE_CURRENT_BINARY_DIR}/wheel-drive-torque-conditioner-fixture.json)

# G75's state-explicit 100 Hz IRW controller. Analytic fixtures distinguish
# first and subsequent derivative estimates, both guidance-state resets, the
# pre-clamp outer-filter memory and all eight filtered PI channels; the same
# target also loads the one strict first-party controller asset.
add_executable(verify_irw_full_state_wheel_speed_guidance_controller
    control/sampled_filtered_pi_header_self_containment.cc
    control/irw_full_state_wheel_speed_guidance_controller_header_self_containment.cc
    configuration/load_irw_full_state_wheel_speed_guidance_controller_header_self_containment.cc
    control/verify_irw_full_state_wheel_speed_guidance_controller.cc)
orvd_configure_first_party_target(
    verify_irw_full_state_wheel_speed_guidance_controller)
target_link_libraries(
    verify_irw_full_state_wheel_speed_guidance_controller
    PRIVATE orvd_configuration)
add_test(NAME verify_irw_full_state_wheel_speed_guidance_controller
         COMMAND verify_irw_full_state_wheel_speed_guidance_controller
             ${CMAKE_SOURCE_DIR}/controller_library/irw/irw_r300_v60_full_state_wheel_speed_guidance_controller.json)

# One topology-independent sampled cruise PI and the strict IRW asset carrying
# its target, nominal radius and raw-joint-rate signs. The scalar test checks
# target-minus-measurement feedback and caller-owned state without repeating
# the already-qualified generic PI clamp and filter matrix.
add_executable(verify_sampled_longitudinal_cruise_controller
    control/sampled_longitudinal_cruise_controller_header_self_containment.cc
    configuration/load_irw_longitudinal_cruise_controller_header_self_containment.cc
    control/verify_sampled_longitudinal_cruise_controller.cc)
orvd_configure_first_party_target(
    verify_sampled_longitudinal_cruise_controller)
target_link_libraries(verify_sampled_longitudinal_cruise_controller
    PRIVATE orvd_configuration)
add_test(NAME verify_sampled_longitudinal_cruise_controller
         COMMAND verify_sampled_longitudinal_cruise_controller
             ${CMAKE_SOURCE_DIR}/controller_library/irw/irw_v60_common_wheel_speed_cruise_controller.json
             ${CMAKE_CURRENT_BINARY_DIR}/irw-longitudinal-cruise-controller-fixture.json)

add_executable(verify_gz18_startup_assembly
    configuration/resolved_startup_state_header_self_containment.cc
    configuration/load_resolved_startup_state_header_self_containment.cc
    configuration/assembled_vehicle_system_header_self_containment.cc
    configuration/assemble_resolved_initial_context_header_self_containment.cc
    configuration/verify_gz18_startup_assembly.cc)
orvd_configure_first_party_target(verify_gz18_startup_assembly)
target_link_libraries(verify_gz18_startup_assembly
    PRIVATE orvd_configuration)
add_test(NAME verify_gz18_startup_assembly
         COMMAND verify_gz18_startup_assembly
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/vehicle_definition.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/gz18/startup_states/moving_startup_60kmh.json
             ${CMAKE_SOURCE_DIR}/track_library/geometries/straight_level_2000m.json)

add_executable(verify_irw_startup_assembly
    configuration/verify_irw_startup_assembly.cc)
orvd_configure_first_party_target(verify_irw_startup_assembly)
target_link_libraries(verify_irw_startup_assembly
    PRIVATE orvd_configuration)
add_test(NAME verify_irw_startup_assembly
         COMMAND verify_irw_startup_assembly
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/vehicle_definition.json
             ${CMAKE_SOURCE_DIR}/vehicle_library/irw/startup_states/moving_startup_60kmh.json
             ${CMAKE_SOURCE_DIR}/track_library/geometries/r300_centerline_superelevation_1100m.json)

add_executable(verify_resolved_startup_state_loading
    configuration/verify_resolved_startup_state_loading.cc)
orvd_configure_first_party_target(verify_resolved_startup_state_loading)
target_link_libraries(verify_resolved_startup_state_loading
    PRIVATE orvd_configuration)
add_test(NAME verify_resolved_startup_state_loading
         COMMAND verify_resolved_startup_state_loading
             ${CMAKE_CURRENT_BINARY_DIR}/resolved-startup-state-fixtures)
