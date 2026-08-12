#pragma once

// Private, non-installed bridge between the backend-neutral system evaluator
// and CVODE's dense Jacobian callback. No SUNDIALS type crosses this boundary.

#include <cstdint>
#include <exception>

#include <Eigen/Core>

namespace orvd::integrators {

class CvodeContinuousStateAdvancer;

namespace internal {

struct DenseFiniteDifferenceJacobianBatchResult final {
    std::int64_t attempted_right_hand_side_evaluation_count{};
    std::exception_ptr lowest_column_failure;
};

class DenseFiniteDifferenceJacobianProvider {
   public:
    virtual ~DenseFiniteDifferenceJacobianProvider() = default;

    [[nodiscard]] virtual int continuous_state_size() const noexcept = 0;
    [[nodiscard]] virtual int requested_worker_count() const noexcept = 0;

    // Evaluates f(t, y + increment[j] e_j) for every state column. The output
    // is preallocated as state_size by state_size, with one derivative vector
    // per column. The baseline derivative and differencing arithmetic remain
    // owned by CVODE's adapter so this provider contains no backend formula.
    // A failed column is returned rather than thrown so the adapter can first
    // account for every RHS evaluation actually attempted by the batch.
    [[nodiscard]] virtual DenseFiniteDifferenceJacobianBatchResult
    CalcPerturbedDerivatives(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
        const Eigen::Ref<const Eigen::VectorXd>& increments,
        Eigen::MatrixXd& perturbed_derivatives) = 0;

    [[nodiscard]] virtual bool IsRecoverableFailure(
        const std::exception_ptr& failure) const noexcept = 0;
};

// The public CVODE wrapper does not expose a Jacobian-policy constructor.
// SystemContinuousStateAdvancer alone reaches this private registration seam.
class DenseFiniteDifferenceJacobianRegistration final {
   public:
    static void Attach(
        CvodeContinuousStateAdvancer& advancer,
        DenseFiniteDifferenceJacobianProvider& provider);
};

}  // namespace internal
}  // namespace orvd::integrators
