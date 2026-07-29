# The toolchain self-check is the first real first-party target in the project,
# so it is also the first consumer of the target-scoped compile policy.
add_executable(verify_eigen_and_cxx23_toolchain
    toolchain/verify_eigen_and_cxx23_toolchain.cc)
orvd_configure_first_party_target(verify_eigen_and_cxx23_toolchain)
target_link_libraries(verify_eigen_and_cxx23_toolchain PRIVATE Eigen3::Eigen)

add_test(NAME verify_eigen_and_cxx23_toolchain
         COMMAND verify_eigen_and_cxx23_toolchain)
