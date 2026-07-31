#pragma once

/// @file
/// The contiguous ODE-state bridge for the compiled ORVD system.

#include <memory>

#include <Eigen/Dense>

#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::system_assembly {
class CompiledSystemPlan;
class SystemInstance;
class SystemRuntimeContext;
}  // namespace orvd::system_assembly

namespace orvd::integrators {

/// States explicitly that no call-time body or joint force is supplied.
/// Model gravity, damping and admitted internal force elements remain active.
struct NoCallTimeAppliedForces final {};

/// Evaluates the compiled system in a dedicated trial context.
///
/// The caller creates and configures the trial context, then transfers its sole
/// ownership here.  An accepted context is intentionally absent from this
/// type, so an RHS call cannot modify one and cannot require write-then-rollback
/// semantics.  Context-local parameters are not silently mirrored; explicit
/// accepted/trial synchronization belongs to the G45 transaction boundary.
/// Construction also requires `NoCallTimeAppliedForces`, making the current
/// graph's empty call-time force input an explicit choice rather than a silent
/// loss of the three typed G42 inputs.
/// `system` and `plan` are borrowed and must outlive the bridge.
class SystemRhsBridge final : public ContinuousStateRhs {
   public:
    SystemRhsBridge(
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        std::unique_ptr<system_assembly::SystemRuntimeContext> trial_context,
        NoCallTimeAppliedForces);
    ~SystemRhsBridge() override;
    SystemRhsBridge(system_assembly::SystemInstance&&,
                    const system_assembly::CompiledSystemPlan&,
                    std::unique_ptr<system_assembly::SystemRuntimeContext>,
                    NoCallTimeAppliedForces) = delete;
    SystemRhsBridge(const system_assembly::SystemInstance&&,
                    const system_assembly::CompiledSystemPlan&,
                    std::unique_ptr<system_assembly::SystemRuntimeContext>,
                    NoCallTimeAppliedForces) = delete;
    SystemRhsBridge(const system_assembly::SystemInstance&,
                    system_assembly::CompiledSystemPlan&&,
                    std::unique_ptr<system_assembly::SystemRuntimeContext>,
                    NoCallTimeAppliedForces) = delete;
    SystemRhsBridge(const system_assembly::SystemInstance&,
                    const system_assembly::CompiledSystemPlan&&,
                    std::unique_ptr<system_assembly::SystemRuntimeContext>,
                    NoCallTimeAppliedForces) = delete;

    SystemRhsBridge(const SystemRhsBridge&) = delete;
    SystemRhsBridge& operator=(const SystemRhsBridge&) = delete;
    SystemRhsBridge(SystemRhsBridge&&) = delete;
    SystemRhsBridge& operator=(SystemRhsBridge&&) = delete;

    [[nodiscard]] int continuous_state_size() const override;
    void CalcTimeDerivatives(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
        Eigen::Ref<Eigen::VectorXd> state_time_derivatives) override;

   private:
    const system_assembly::SystemInstance* system_;
    const system_assembly::CompiledSystemPlan* plan_;
    std::unique_ptr<system_assembly::SystemRuntimeContext> trial_context_;
    Eigen::VectorXd derivative_buffer_;
};

}  // namespace orvd::integrators
