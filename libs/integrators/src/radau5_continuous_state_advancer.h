#pragma once

/// @file
/// Source-tree-only ContinuousStateAdvancer adapter for the Radau5 core.

#include <memory>

#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::integrators {

/// Research adapter for the pure-C++ Radau5 numerical core.
///
/// This declaration deliberately remains below `src/`; it is not installed and
/// does not create a public backend-selection commitment.
class Radau5ContinuousStateAdvancer final : public ContinuousStateAdvancer {
   public:
    Radau5ContinuousStateAdvancer(
        ContinuousStateRhs& rhs,
        double initial_time_seconds,
        Eigen::VectorXd initial_continuous_state,
        ContinuousStateErrorTolerances tolerances);
    ~Radau5ContinuousStateAdvancer() override;

    Radau5ContinuousStateAdvancer(const Radau5ContinuousStateAdvancer&) =
        delete;
    Radau5ContinuousStateAdvancer& operator=(
        const Radau5ContinuousStateAdvancer&) = delete;
    Radau5ContinuousStateAdvancer(Radau5ContinuousStateAdvancer&&) = delete;
    Radau5ContinuousStateAdvancer& operator=(
        Radau5ContinuousStateAdvancer&&) = delete;

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

    /// Source-private notification for accepted numerical RHS history that is
    /// not part of the continuous state.  This is deliberately narrower than
    /// reinitialization and is not an installed backend-selection API.
    void InvalidateLinearizationAfterNumericalRhsHistoryChange() noexcept;

    [[nodiscard]] std::optional<ContinuousStateDenseOutputInterval>
    dense_output_interval() const override;

    void CopyDenseState(
        double time_seconds,
        Eigen::Ref<Eigen::VectorXd> continuous_state) const override;

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators
