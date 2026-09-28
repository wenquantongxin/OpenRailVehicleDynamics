#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Dense>

namespace orvd::integrators::internal {

// Source-private coordinates. s is qdot, not a multibody generalized velocity.
struct CoordinateState final {
    double time_seconds{};
    Eigen::VectorXd q;
    Eigen::VectorXd s;
    Eigen::VectorXd z;
};

class CoordinateIntegrationFailure final : public std::runtime_error {
   public:
    enum class Reason {
        kStepSizeUnderflow,
        kNonFiniteState,
        kNonFiniteEvaluation,
        kNonFiniteLinearSystem,
        kSingularJacobian,
        kNonlinearConvergenceFailure,
    };

    CoordinateIntegrationFailure(Reason reason, std::string diagnostic)
        : std::runtime_error(std::move(diagnostic)), reason_(reason) {}

    [[nodiscard]] Reason reason() const noexcept { return reason_; }

   private:
    Reason reason_;
};

struct CoordinateIntegrationDiagnostics final {
    // Successful startup steps, not attempted steps.
    std::uint64_t startup_step_count{};
    // Projection work is retained even when a subsequent evaluation fails.
    std::uint64_t endpoint_projection_evaluation_count{};
    std::uint64_t endpoint_projection_change_count{};
};

// q'=s, s'=b(t,q,s,z), z'=g(t,q,s,z). All three mechanical vectors have nq
// entries. Dimensions are fixed for the lifetime of a borrowed problem.
class CoordinateSecondOrderProblem {
   public:
    virtual ~CoordinateSecondOrderProblem() = default;

    [[nodiscard]] virtual int coordinate_size() const = 0;
    [[nodiscard]] virtual int internal_state_size() const = 0;

    // Called before initializing or reinitializing. The core checks dimensions
    // and finite values first. Reject invalid geometry; do not repair input or
    // change a projection reference. Callback exceptions retain their type.
    virtual void ValidateInitialState(
        double, const Eigen::Ref<const Eigen::VectorXd>&,
        const Eigen::Ref<const Eigen::VectorXd>&,
        const Eigen::Ref<const Eigen::VectorXd>&) const {}

    // Write every entry of both pre-sized outputs. A trial may update private
    // evaluation scratch, but must never commit externally accepted state.
    virtual void Evaluate(
        double time_seconds, const Eigen::Ref<const Eigen::VectorXd>& q,
        const Eigen::Ref<const Eigen::VectorXd>& s,
        const Eigen::Ref<const Eigen::VectorXd>& z,
        Eigen::Ref<Eigen::VectorXd> b, Eigen::Ref<Eigen::VectorXd> g) = 0;

    // Called on the completed candidate, never inside the Newton residual or
    // finite differences. q_reference is the last successful initialization's
    // q, owned by the core. Return whether q or s actually changed. The default
    // is suitable for unconstrained coordinates. z is deliberately absent.
    virtual bool ProjectEndpoint(
        const Eigen::Ref<const Eigen::VectorXd>&,
        Eigen::Ref<Eigen::VectorXd>, Eigen::Ref<Eigen::VectorXd>) {
        return false;
    }
};

}  // namespace orvd::integrators::internal
