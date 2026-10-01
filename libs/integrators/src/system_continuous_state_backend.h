#pragma once

/// @file
/// Source-private owner of one concrete system integration backend.

#include <cstdint>
#include <memory>
#include <optional>
#include <type_traits>

#include <Eigen/Dense>

#include "orvd/integrators/continuous_state_advancer.h"
#include "orvd/integrators/system_rhs_bridge.h"

#include "orvd/integrators/system_integration_configuration.h"
#include <string_view>

namespace orvd::system_assembly {
class CompiledSystemPlan;
class SystemInstance;
class SystemRuntimeContext;
}  // namespace orvd::system_assembly

namespace orvd::integrators::internal {

class SystemContinuousStateBackend final {
   public:
    SystemContinuousStateBackend(
        SystemIntegrationMethodConfiguration configuration,
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& candidate_context,
        const system_assembly::SystemRuntimeContext& accepted_context,
        const Eigen::VectorXd& initial_continuous_state,
        NoCallTimeAppliedForces no_call_time_applied_forces);
    template <class System, class Plan>
        requires (!std::is_lvalue_reference_v<System&&> ||
                  !std::is_lvalue_reference_v<Plan&&>)
    SystemContinuousStateBackend(
        SystemIntegrationMethodConfiguration, System&&, Plan&&,
        system_assembly::SystemRuntimeContext&,
        const system_assembly::SystemRuntimeContext&, const Eigen::VectorXd&,
        NoCallTimeAppliedForces) = delete;

    ~SystemContinuousStateBackend();

    SystemContinuousStateBackend(const SystemContinuousStateBackend&) =
        delete;
    SystemContinuousStateBackend& operator=(
        const SystemContinuousStateBackend&) = delete;
    SystemContinuousStateBackend(SystemContinuousStateBackend&&) = delete;
    SystemContinuousStateBackend& operator=(
        SystemContinuousStateBackend&&) = delete;

    [[nodiscard]] ContinuousStateAdvancer& advancer();
    [[nodiscard]] const ContinuousStateAdvancer& advancer() const;
    [[nodiscard]] std::string_view method_identifier() const noexcept;
    void SynchronizeContextLocalDataFrom(
        const system_assembly::SystemRuntimeContext& accepted_context);

    /// Notifies the selected backend that accepted wheel--rail projection
    /// history changed outside the continuous state.  Backends that retain a
    /// linearization may invalidate it without resetting accepted numerical
    /// history.
    void NotifyAcceptedProjectionHistoryChange();

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators::internal
