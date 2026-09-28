#pragma once

#include <memory>

#include "basic_mechanical_integration_configuration.h"
#include "coordinate_second_order_problem.h"
#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::integrators::internal {

/// Basic coordinate Zhai method with phi = psi = 1/2 on equal steps.
///
/// The first step and each change of step length use the original
/// phi = psi = 0 start (and forward Euler for z). Repeating those starts
/// frequently does not promise global second-order accuracy. A problem owns
/// any coordinate projection; the core always evaluates the accepted
/// acceleration and internal-state derivative after that projection.
class ZhaiCore final {
   public:
    ZhaiCore(CoordinateSecondOrderProblem& problem,
             ZhaiConfiguration configuration,
             const CoordinateState& initial_state);
    ZhaiCore(CoordinateSecondOrderProblem&&,
             ZhaiConfiguration,
             const CoordinateState&) = delete;
    ~ZhaiCore();

    ZhaiCore(const ZhaiCore&) = delete;
    ZhaiCore& operator=(const ZhaiCore&) = delete;
    ZhaiCore(ZhaiCore&&) = delete;
    ZhaiCore& operator=(ZhaiCore&&) = delete;

    void AdvanceOneStep();
    void AdvanceOneStep(double step_size_seconds);
    // Only the endpoint clock may be rounded; the specified h remains the
    // exact interval identity used by the two-step history comparison.
    void AdvanceOneStep(double step_size_seconds, double endpoint_time_seconds);
    void Reinitialize(const CoordinateState& state);

    [[nodiscard]] double current_time_seconds() const;
    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd> q,
                          Eigen::Ref<Eigen::VectorXd> s,
                          Eigen::Ref<Eigen::VectorXd> z) const;

    [[nodiscard]] ContinuousStateIntegrationStatistics
    integration_statistics() const;
    [[nodiscard]] CoordinateIntegrationDiagnostics diagnostics() const;

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators::internal
