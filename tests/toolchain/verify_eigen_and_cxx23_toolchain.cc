// Verifies that the toolchain this project is configured with actually provides
// the two things every later target depends on: a working Eigen, and the C++23
// library features we intend to use.
//
// This is not a placeholder. If it fails, nothing built after it can be trusted,
// because every multibody pass is Eigen arithmetic written in C++23.
#include <cstdio>
#include <expected>
#include <string_view>

#include <Eigen/Dense>

namespace {

// std::expected is C++23 and is how this project reports a failed check: the
// error carries what went wrong, so the caller cannot ignore it by accident the
// way an ignored bool return would be.
std::expected<void, std::string_view> VerifyDenseSolveRecoversKnownSolution() {
    Eigen::Matrix3d coefficients;
    coefficients << 2.0, -1.0, 0.0,
                    -1.0, 2.0, -1.0,
                    0.0, -1.0, 2.0;
    const Eigen::Vector3d expected_solution(1.0, 2.0, 3.0);
    const Eigen::Vector3d right_hand_side = coefficients * expected_solution;

    const Eigen::Vector3d computed_solution =
        coefficients.colPivHouseholderQr().solve(right_hand_side);

    // The matrix is well conditioned and the scale is order one, so a solve that
    // is working at all lands far inside this bound; a solve that is broken
    // misses it by orders of magnitude.
    constexpr double kSolveTolerance = 1e-12;
    if ((computed_solution - expected_solution).cwiseAbs().maxCoeff() > kSolveTolerance)
        return std::unexpected("dense linear solve did not recover the known solution");
    return {};
}

std::expected<void, std::string_view> VerifySelfAdjointEigenvaluesOfKnownMatrix() {
    Eigen::Matrix2d symmetric;
    symmetric << 2.0, 1.0,
                 1.0, 2.0;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(symmetric);
    if (solver.info() != Eigen::Success)
        return std::unexpected("self-adjoint eigen decomposition reported failure");

    // Eigenvalues of [[2,1],[1,2]] are exactly 1 and 3.
    constexpr double kEigenvalueTolerance = 1e-12;
    if (std::abs(solver.eigenvalues()(0) - 1.0) > kEigenvalueTolerance ||
        std::abs(solver.eigenvalues()(1) - 3.0) > kEigenvalueTolerance)
        return std::unexpected("self-adjoint eigenvalues are wrong");
    return {};
}

}  // namespace

int main() {
    for (const auto& [check_name, check] :
         {std::pair{std::string_view{"dense linear solve"},
                    &VerifyDenseSolveRecoversKnownSolution},
          std::pair{std::string_view{"self-adjoint eigenvalues"},
                    &VerifySelfAdjointEigenvaluesOfKnownMatrix}}) {
        if (const auto result = check(); !result) {
            std::fprintf(stderr, "%.*s FAILED: %.*s\n",
                         static_cast<int>(check_name.size()), check_name.data(),
                         static_cast<int>(result.error().size()), result.error().data());
            return 1;
        }
    }
    std::printf("Eigen %d.%d.%d and C++%ld toolchain verified\n",
                EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION,
                static_cast<long>(__cplusplus));
    return 0;
}
