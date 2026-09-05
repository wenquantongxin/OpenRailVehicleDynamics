#include "continuous_state_advancer_contract.h"

#include <cstdio>
#include <memory>
#include <utility>

#include <Eigen/Dense>

#include "orvd/radau5/radau5_core.h"
#include "radau5_continuous_state_advancer.h"

namespace {

std::unique_ptr<orvd::integrators::ContinuousStateAdvancer>
MakeRadau5Advancer(
    orvd::integrators::ContinuousStateRhs& rhs,
    double initial_time_seconds,
    Eigen::VectorXd initial_continuous_state,
    orvd::integrators::ContinuousStateErrorTolerances tolerances) {
    return std::make_unique<
        orvd::integrators::Radau5ContinuousStateAdvancer>(
        rhs, initial_time_seconds, std::move(initial_continuous_state),
        std::move(tolerances));
}

}  // namespace

int main() {
    const int failure_count =
        orvd::integrators::test::RunContinuousStateAdvancerContract(
            &MakeRadau5Advancer) +
        orvd::integrators::test::RunContinuousStateFailureContract(
            &MakeRadau5Advancer,
            orvd::integrators::ContinuousStateNumericalFailure::Reason::
                kNonFiniteRightHandSide,
            static_cast<int>(
                orvd::radau5::Failure::Reason::kNonFiniteRhs));
    if (failure_count != 0) {
        std::printf(
            "Radau5 continuous-state advancer contract checks failed: %d\n",
            failure_count);
        return 1;
    }
    std::printf(
        "Radau5 continuous-state advancer contract checks passed\n");
    return 0;
}
