#include "continuous_state_advancer_contract.h"

#include <cstdio>
#include <memory>
#include <utility>

#include <Eigen/Dense>

#include "orvd/integrators/cvode_continuous_state_advancer.h"

namespace {

std::unique_ptr<orvd::integrators::ContinuousStateAdvancer>
MakePublicCvodeAdvancer(
    orvd::integrators::ContinuousStateRhs& rhs,
    double initial_time_seconds,
    Eigen::VectorXd initial_continuous_state,
    orvd::integrators::ContinuousStateErrorTolerances tolerances) {
    return std::make_unique<
        orvd::integrators::CvodeContinuousStateAdvancer>(
        rhs, initial_time_seconds, std::move(initial_continuous_state),
        std::move(tolerances));
}

}  // namespace

int main() {
    const int failure_count =
        orvd::integrators::test::RunContinuousStateAdvancerContract(
            &MakePublicCvodeAdvancer);
    if (failure_count != 0) {
        std::printf(
            "CVODE continuous-state advancer contract checks failed: %d\n",
            failure_count);
        return 1;
    }
    std::printf("CVODE continuous-state advancer contract checks passed\n");
    return 0;
}
