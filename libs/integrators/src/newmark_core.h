#pragma once

#include <memory>

#include "orvd/integrators/continuous_state_advancer.h"

#include "basic_mechanical_integration_configuration.h"
#include "coordinate_second_order_problem.h"

namespace orvd::integrators::internal {

// Basic coordinate Newmark: beta=1/4, gamma=1/2, trapezoidal z. This source-
// private core has no system context, stop-time scheduler or dense output.
// The borrowed problem must outlive the core. Initialization evaluates B/G
// once, without projecting the supplied state.
class NewmarkCore final {
   public:
    NewmarkCore(CoordinateSecondOrderProblem& problem,
                NewmarkConfiguration configuration,
                const CoordinateState& initial_state);
    NewmarkCore(CoordinateSecondOrderProblem&&, NewmarkConfiguration,
                const CoordinateState&) = delete;
    ~NewmarkCore();

    NewmarkCore(const NewmarkCore&) = delete;
    NewmarkCore& operator=(const NewmarkCore&) = delete;
    NewmarkCore(NewmarkCore&&) = delete;
    NewmarkCore& operator=(NewmarkCore&&) = delete;

    [[nodiscard]] double current_time_seconds() const;
    [[nodiscard]] ContinuousStateIntegrationStatistics integration_statistics()
        const;
    [[nodiscard]] CoordinateIntegrationDiagnostics diagnostics() const;

    // All output dimensions are checked before any output entry is written.
    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd> q,
                          Eigen::Ref<Eigen::VectorXd> s,
                          Eigen::Ref<Eigen::VectorXd> z) const;

    void AdvanceOneStep();
    // Explicit boundary adaptation, 0 < h <= configured nominal step.
    void AdvanceOneStep(double step_size_seconds);
    // The explicit endpoint may differ from current_time+h only by the shared
    // clock-rounding allowance. Formula coefficients still use h. Invalid
    // endpoint input is rejected before changing state or failure status.
    void AdvanceOneStep(double step_size_seconds, double endpoint_time_seconds);

    // A successful reinitialization resets work counters (including its one
    // initial B/G evaluation) and replaces the projection reference. Invalid
    // input leaves the core usable; an evaluation failure requires retrying
    // reinitialization. Neither failure replaces the last accepted state.
    void Reinitialize(const CoordinateState& initial_state);

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators::internal
