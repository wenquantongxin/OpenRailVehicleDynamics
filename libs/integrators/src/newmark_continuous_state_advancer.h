#pragma once

#include <memory>

#include "basic_mechanical_integration_configuration.h"
#include "coordinate_second_order_problem.h"
#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::integrators::internal {

class SystemCoordinateProblem;

// Real fixed-step Newmark core adapted to the shared physical-state contract.
// Borrowed problem must outlive the adapter. Dense values use linear physical
// storage interpolation with the bridge's quaternion representation adaptation;
// observations never change integration history or invoke the RHS.
class NewmarkContinuousStateAdvancer final : public ContinuousStateAdvancer {
   public:
    NewmarkContinuousStateAdvancer(SystemCoordinateProblem& problem,
             double initial_time_seconds, const Eigen::VectorXd& initial_physical_state,
             NewmarkConfiguration configuration);
    NewmarkContinuousStateAdvancer(SystemCoordinateProblem&&, double,
             const Eigen::VectorXd&, NewmarkConfiguration) = delete;
    ~NewmarkContinuousStateAdvancer() override;

    NewmarkContinuousStateAdvancer(const NewmarkContinuousStateAdvancer&) = delete;
    NewmarkContinuousStateAdvancer& operator=(const NewmarkContinuousStateAdvancer&) = delete;
    NewmarkContinuousStateAdvancer(NewmarkContinuousStateAdvancer&&) = delete;
    NewmarkContinuousStateAdvancer& operator=(NewmarkContinuousStateAdvancer&&) = delete;

    [[nodiscard]] int continuous_state_size() const override;
    [[nodiscard]] double current_time_seconds() const override;
    [[nodiscard]] ContinuousStateIntegrationStatistics integration_statistics() const override;
    [[nodiscard]] CoordinateIntegrationDiagnostics diagnostics() const;
    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd>) const override;
    [[nodiscard]] ContinuousStateInternalStep AdvanceOneInternalStepToward(
        double stop_time_seconds, Eigen::Ref<Eigen::VectorXd> endpoint) override;
    void ReinitializeAfterExternalChange(
        double time_seconds, const Eigen::Ref<const Eigen::VectorXd>& physical_state) override;
    [[nodiscard]] std::optional<ContinuousStateDenseOutputInterval>
    dense_output_interval() const override;
    void CopyDenseState(double time_seconds, Eigen::Ref<Eigen::VectorXd>) const override;

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators::internal
