#pragma once

#include <memory>

#include "orvd/integrators/system_rhs_bridge.h"

#include "coordinate_second_order_problem.h"

namespace orvd::multibody_model {
class MultibodyModel;
}

namespace orvd::integrators::internal {

// Source-private coordinate view of the same compiled physics as SystemRhsBridge.
// The caller owns system, plan and a dedicated physics trial context; all must
// outlive this object. No accepted context is retained. A separate geometry
// context isolates validation, conversion and projection from physical trials.
// Like a runtime context, one instance is for sequential use, not concurrent calls.
class SystemCoordinateProblem final : public CoordinateSecondOrderProblem {
   public:
    SystemCoordinateProblem(
        const system_assembly::SystemInstance& system,
        const system_assembly::CompiledSystemPlan& plan,
        system_assembly::SystemRuntimeContext& trial_context,
        NoCallTimeAppliedForces);
    ~SystemCoordinateProblem() override;
    SystemCoordinateProblem(system_assembly::SystemInstance&&,
                            const system_assembly::CompiledSystemPlan&,
                            system_assembly::SystemRuntimeContext&,
                            NoCallTimeAppliedForces) = delete;
    SystemCoordinateProblem(const system_assembly::SystemInstance&&,
                            const system_assembly::CompiledSystemPlan&,
                            system_assembly::SystemRuntimeContext&,
                            NoCallTimeAppliedForces) = delete;
    SystemCoordinateProblem(const system_assembly::SystemInstance&,
                            system_assembly::CompiledSystemPlan&&,
                            system_assembly::SystemRuntimeContext&,
                            NoCallTimeAppliedForces) = delete;
    SystemCoordinateProblem(const system_assembly::SystemInstance&,
                            const system_assembly::CompiledSystemPlan&&,
                            system_assembly::SystemRuntimeContext&,
                            NoCallTimeAppliedForces) = delete;
    SystemCoordinateProblem(const SystemCoordinateProblem&) = delete;
    SystemCoordinateProblem& operator=(const SystemCoordinateProblem&) = delete;
    SystemCoordinateProblem(SystemCoordinateProblem&&) = delete;
    SystemCoordinateProblem& operator=(SystemCoordinateProblem&&) = delete;

    [[nodiscard]] const multibody_model::MultibodyModel& model() const;
    [[nodiscard]] int coordinate_size() const override;
    [[nodiscard]] int internal_state_size() const override;
    [[nodiscard]] int physical_state_size() const;

    // Neither conversion evaluates forces or changes the physics trial context.
    // q and z are preserved; s=N(q)v on import and v=N+(q)s on export. Export
    // discards any radial quaternion rate, just as the trial extension does.
    [[nodiscard]] CoordinateState MakeCoordinateState(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& physical_state) const;
    void CopyPhysicalState(const CoordinateState& coordinate_state,
                           Eigen::Ref<Eigen::VectorXd> physical_state) const;

    // Observation-only interpolation of physical [q,v,z] storage. Ordinary
    // entries are linear; free quaternions use shortest-sign normalized linear
    // interpolation of their directions, scaled to the reference norm. Exact
    // endpoints retain their original bits, including quaternion sign and norm.
    // No force or coordinate-rate mapping is evaluated, so a finite RPY sample
    // may lie at a rate-map singularity. Inputs may alias the output; failure
    // leaves the whole output unchanged and never changes the physics context.
    void CopyLinearlyInterpolatedPhysicalState(
        const Eigen::Ref<const Eigen::VectorXd>& reference_q,
        const Eigen::Ref<const Eigen::VectorXd>& start_physical,
        const Eigen::Ref<const Eigen::VectorXd>& end_physical, double fraction,
        Eigen::Ref<Eigen::VectorXd> physical_state) const;

    // Copies only the existing context-local inputs. Time, physical state,
    // projection hints and integrator history remain the caller's responsibility.
    void SynchronizeContextLocalDataFrom(
        const system_assembly::SystemRuntimeContext& source_context);

    void ValidateInitialState(
        double time_seconds, const Eigen::Ref<const Eigen::VectorXd>& q,
        const Eigen::Ref<const Eigen::VectorXd>& s,
        const Eigen::Ref<const Eigen::VectorXd>& z) const override;

    // One complete physical RHS, followed by a -> b on that same physical state.
    // Raw Newton s may have a radial component: the extension uses v=N+(q)s
    // and the model's qdot=N(q)v, not a chain rule along arbitrary raw s.
    // Outputs must be disjoint. Both are written only after the entire call succeeds.
    void Evaluate(double time_seconds,
                  const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>& s,
                  const Eigen::Ref<const Eigen::VectorXd>& z,
                  Eigen::Ref<Eigen::VectorXd> b,
                  Eigen::Ref<Eigen::VectorXd> g) override;

    // Restore each free quaternion's reference norm and reconstruct its tangent
    // rate while preserving physical velocity. All other entries remain exact.
    // Reference data are supplied by the core, never stored as bridge history.
    bool ProjectEndpoint(const Eigen::Ref<const Eigen::VectorXd>& q_reference,
                         Eigen::Ref<Eigen::VectorXd> q,
                         Eigen::Ref<Eigen::VectorXd> s) override;

   private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::integrators::internal
