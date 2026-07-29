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
