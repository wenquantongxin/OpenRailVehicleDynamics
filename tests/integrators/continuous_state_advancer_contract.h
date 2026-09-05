#pragma once

#include <memory>

#include <Eigen/Dense>

#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::integrators::test {

using ContinuousStateAdvancerFactory =
    std::unique_ptr<ContinuousStateAdvancer> (*)(
        ContinuousStateRhs& rhs,
        double initial_time_seconds,
        Eigen::VectorXd initial_continuous_state,
        ContinuousStateErrorTolerances tolerances);

/// Runs the observable contract shared by every continuous-state backend.
///
/// The factory must create a production backend that borrows `rhs`. The caller
/// retains that RHS for the lifetime of every returned advancer.
[[nodiscard]] int RunContinuousStateAdvancerContract(
    ContinuousStateAdvancerFactory factory);

/// Runs failure semantics shared by every backend while allowing the concrete
/// adapter to supply its expected non-finite-RHS classification and raw code.
[[nodiscard]] int RunContinuousStateFailureContract(
    ContinuousStateAdvancerFactory factory,
    ContinuousStateNumericalFailure::Reason expected_nonfinite_rhs_reason,
    int expected_backend_code);

}  // namespace orvd::integrators::test
