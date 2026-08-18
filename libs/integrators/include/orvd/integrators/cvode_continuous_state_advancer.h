#pragma once

/// @file
/// The SUNDIALS CVODE implementation of the continuous-state advancer.

#include <memory>

#include <Eigen/Dense>

#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::integrators {

namespace internal {
class BdfIntegrationAccess;
class DenseFiniteDifferenceJacobianRegistration;
enum class MaximumBdfOrder : int;
}

/// Advances one positive-dimensional continuous state with SUNDIALS CVODE.
///
/// The numerical method is fixed to the admitted first-backend configuration:
/// double precision, a serial vector, BDF, and a dense finite-difference
/// Jacobian. `rhs` is borrowed and must outlive this object. SUNDIALS types and
/// storage do not enter the public interface.
class CvodeContinuousStateAdvancer final : public ContinuousStateAdvancer {
   public:
    CvodeContinuousStateAdvancer(
        ContinuousStateRhs& rhs,
        double initial_time_seconds,
        Eigen::VectorXd initial_continuous_state,
        ContinuousStateErrorTolerances tolerances);
    ~CvodeContinuousStateAdvancer() override;

    CvodeContinuousStateAdvancer(const CvodeContinuousStateAdvancer&) = delete;
    CvodeContinuousStateAdvancer& operator=(
        const CvodeContinuousStateAdvancer&) = delete;
    CvodeContinuousStateAdvancer(CvodeContinuousStateAdvancer&&) = delete;
    CvodeContinuousStateAdvancer& operator=(
        CvodeContinuousStateAdvancer&&) = delete;

    [[nodiscard]] int continuous_state_size() const override;
    [[nodiscard]] double current_time_seconds() const override;
    [[nodiscard]] ContinuousStateIntegrationStatistics
    integration_statistics() const override;
    void CopyCurrentState(
        Eigen::Ref<Eigen::VectorXd> continuous_state) const override;
    [[nodiscard]] ContinuousStateInternalStep AdvanceOneInternalStepToward(
        double stop_time_seconds,
        Eigen::Ref<Eigen::VectorXd> endpoint_continuous_state) override;
    void ReinitializeAfterExternalChange(
        double committed_time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& committed_continuous_state)
        override;
    [[nodiscard]] std::optional<ContinuousStateDenseOutputInterval>
    dense_output_interval() const override;
    void CopyDenseState(
        double time_seconds,
        Eigen::Ref<Eigen::VectorXd> continuous_state) const override;

   private:
    friend class internal::BdfIntegrationAccess;
    friend class internal::DenseFiniteDifferenceJacobianRegistration;

    CvodeContinuousStateAdvancer(
        ContinuousStateRhs& rhs,
        double initial_time_seconds,
        Eigen::VectorXd initial_continuous_state,
        ContinuousStateErrorTolerances tolerances,
        internal::MaximumBdfOrder maximum_bdf_order);

    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators
