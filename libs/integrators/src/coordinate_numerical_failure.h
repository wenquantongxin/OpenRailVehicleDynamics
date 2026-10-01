#pragma once

#include "coordinate_second_order_problem.h"
#include "orvd/integrators/continuous_state_advancer.h"

namespace orvd::integrators::internal {

[[noreturn]] inline void RethrowCoordinateNumericalFailure(
    const CoordinateIntegrationFailure& failure) {
    using Source = CoordinateIntegrationFailure::Reason;
    using Target = ContinuousStateNumericalFailure::Reason;
    Target reason;
    switch (failure.reason()) {
        case Source::kStepSizeUnderflow: reason = Target::kStepSizeUnderflow; break;
        case Source::kNonFiniteState: reason = Target::kNonFiniteState; break;
        case Source::kNonFiniteEvaluation: reason = Target::kNonFiniteRightHandSide; break;
        case Source::kNonFiniteLinearSystem: reason = Target::kNonFiniteLinearSystem; break;
        case Source::kSingularJacobian: reason = Target::kSingularLinearSystem; break;
        case Source::kNonlinearConvergenceFailure: reason = Target::kNonlinearConvergenceFailure; break;
        default: throw std::logic_error("unknown coordinate numerical failure classification");
    }
    throw ContinuousStateNumericalFailure(
        reason, static_cast<int>(failure.reason()), failure.what());
}

}  // namespace orvd::integrators::internal
