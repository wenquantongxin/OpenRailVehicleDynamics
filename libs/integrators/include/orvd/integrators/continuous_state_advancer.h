#pragma once

/// @file
/// The backend-neutral contracts shared by the continuous-state advancer and
/// its right-hand side.

#include <Eigen/Dense>

namespace orvd::integrators {

/// Numerical error tolerances for one dynamically sized continuous state.
class ContinuousStateErrorTolerances {
   public:
    ContinuousStateErrorTolerances(
        double relative_tolerance,
        Eigen::VectorXd component_absolute_tolerances);

    [[nodiscard]] double relative_tolerance() const {
        return relative_tolerance_;
    }
    [[nodiscard]] const Eigen::VectorXd& component_absolute_tolerances() const {
        return component_absolute_tolerances_;
    }

   private:
    double relative_tolerance_;
    Eigen::VectorXd component_absolute_tolerances_;
};

/// A contiguous, dynamically sized right-hand side for an ODE backend.
class ContinuousStateRhs {
   public:
    virtual ~ContinuousStateRhs() = default;

    [[nodiscard]] virtual int continuous_state_size() const = 0;

    /// Evaluates one trial state into caller-owned, already-sized storage.
    /// Implementations must not commit the trial to an accepted state.
    virtual void CalcTimeDerivatives(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& continuous_state,
        Eigen::Ref<Eigen::VectorXd> state_time_derivatives) = 0;
};

/// The numerical-backend-independent advancement surface.
///
/// G44 supplies the first implementation, CVODE.  No Drake integrator subtype
/// or compatibility route is present behind this interface.
class ContinuousStateAdvancer {
   public:
    virtual ~ContinuousStateAdvancer() = default;

    [[nodiscard]] virtual double current_time_seconds() const = 0;
    virtual void AdvanceTo(double target_time_seconds) = 0;

    /// Rebuilds backend history after an external state, parameter or input
    /// change.  The concrete backend reads the already-committed state.
    virtual void ReinitializeAfterExternalChange() = 0;
};

}  // namespace orvd::integrators
