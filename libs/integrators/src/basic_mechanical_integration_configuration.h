#pragma once

#include <Eigen/Dense>

#include "orvd/integrators/mechanical_integration_configuration.h"

namespace orvd::integrators::internal {

// Positive dimensional absolute scales for one successful initialization epoch.
// They remain fixed during nonlinear solves and ordinary advances. A successful
// explicit reinitialization may replace them together with the coordinate
// reference; a failed reinitialization replaces neither. These are not ODE
// local-error tolerances.
struct NewmarkNewtonSolverConfiguration final {
    int maximum_iterations{12};
    Eigen::VectorXd position_correction_scales;       // nq; units of q
    Eigen::VectorXd velocity_correction_scales;       // nq; units of qdot
    Eigen::VectorXd internal_state_correction_scales; // nz; units of z
    Eigen::VectorXd acceleration_residual_scales;     // nq; units of qddot
    Eigen::VectorXd internal_state_residual_scales;   // nz; units of z
    Eigen::VectorXd unknown_reference_scales;         // nq+nz; units of (b,z)
};

struct NewmarkCoreConfiguration final {
    double step_size_seconds{};
    NewmarkNewtonSolverConfiguration nonlinear_solver;
};


}  // namespace orvd::integrators::internal
