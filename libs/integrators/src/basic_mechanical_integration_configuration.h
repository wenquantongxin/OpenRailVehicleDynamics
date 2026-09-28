#pragma once

#include <Eigen/Dense>

namespace orvd::integrators::internal {

// Positive, dimensional absolute scales supplied explicitly by the caller.
// They are fixed throughout a solve; none is an ODE local-error tolerance.
struct NewmarkNewtonConfiguration final {
    int maximum_iterations{12};
    Eigen::VectorXd position_correction_scales;       // nq; units of q
    Eigen::VectorXd velocity_correction_scales;       // nq; units of qdot
    Eigen::VectorXd internal_state_correction_scales; // nz; units of z
    Eigen::VectorXd acceleration_residual_scales;     // nq; units of qddot
    Eigen::VectorXd internal_state_residual_scales;   // nz; units of z
    Eigen::VectorXd unknown_reference_scales;         // nq+nz; units of (b,z)
};

struct NewmarkConfiguration final {
    double step_size_seconds{};
    NewmarkNewtonConfiguration nonlinear_solver;
};

struct ZhaiConfiguration final {
    double step_size_seconds{};
};

}  // namespace orvd::integrators::internal
