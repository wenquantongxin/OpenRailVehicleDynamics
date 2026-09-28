#pragma once

#include <memory>

#include "basic_mechanical_integration_configuration.h"
#include "coordinate_second_order_problem.h"
#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::integrators::internal {

class SystemCoordinateProblem;

// Real fixed-step Zhai core adapted to the shared physical-state contract.
// Borrowed problem must outlive the adapter. Dense values use linear physical
// storage interpolation with the bridge's quaternion representation adaptation;
// observations never change integration history or invoke the RHS.
class ZhaiContinuousStateAdvancer final : public ContinuousStateAdvancer {
   public:
    ZhaiContinuousStateAdvancer(SystemCoordinateProblem& problem,
             double initial_time_seconds, const Eigen::VectorXd& initial_physical_state,
             ZhaiConfiguration configuration);
    ZhaiContinuousStateAdvancer(SystemCoordinateProblem&&, double,
             const Eigen::VectorXd&, ZhaiConfiguration) = delete;
    ~ZhaiContinuousStateAdvancer() override;

    ZhaiContinuousStateAdvancer(const ZhaiContinuousStateAdvancer&) = delete;
    ZhaiContinuousStateAdvancer& operator=(const ZhaiContinuousStateAdvancer&) = delete;
    ZhaiContinuousStateAdvancer(ZhaiContinuousStateAdvancer&&) = delete;
    ZhaiContinuousStateAdvancer& operator=(ZhaiContinuousStateAdvancer&&) = delete;

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
