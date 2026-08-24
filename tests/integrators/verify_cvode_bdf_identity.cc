#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>

#include <Eigen/Dense>

#include "bdf_integration_access.h"
#include "orvd/integrators/cvode_continuous_state_advancer.h"

namespace {

using orvd::integrators::ContinuousStateErrorTolerances;
using orvd::integrators::ContinuousStateRhs;
using orvd::integrators::CvodeContinuousStateAdvancer;
using orvd::integrators::internal::BdfIntegrationAccess;

int failure_count = 0;

void Expect(bool condition, const char* description) {
    if (!condition) {
        std::printf("FAIL %s\n", description);
        ++failure_count;
    }
}

void ExpectNear(double actual, double expected, double tolerance,
                const char* description) {
    if (!std::isfinite(actual) || !std::isfinite(expected) ||
        !(std::abs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: actual=%.17g expected=%.17g tolerance=%.3g\n",
                    description, actual, expected, tolerance);
        ++failure_count;
    }
}

ContinuousStateErrorTolerances MakeTolerances() {
    return ContinuousStateErrorTolerances(
        1.0e-11, Eigen::VectorXd::Constant(2, 1.0e-13));
}

class LinearOscillatorRhs final : public ContinuousStateRhs {
   public:
    explicit LinearOscillatorRhs(double angular_frequency)
        : angular_frequency_(angular_frequency) {}

    int continuous_state_size() const override { return 2; }

    void CalcTimeDerivatives(
        double, const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) override {
        derivatives[0] = state[1];
        derivatives[1] =
            -angular_frequency_ * angular_frequency_ * state[0];
    }

   private:
    double angular_frequency_;
};

Eigen::Vector2d OscillatorState(double initial_time,
                                const Eigen::Vector2d& initial,
                                double angular_frequency, double time) {
    const double phase = angular_frequency * (time - initial_time);
    return Eigen::Vector2d(
        initial[0] * std::cos(phase) +
            initial[1] * std::sin(phase) / angular_frequency,
        -initial[0] * angular_frequency * std::sin(phase) +
            initial[1] * std::cos(phase));
}

void ExpectStateNear(const Eigen::VectorXd& actual,
                     const Eigen::Vector2d& expected,
                     const char* description) {
    const double scale = std::max(1.0, expected.cwiseAbs().maxCoeff());
    const double tolerance = 1.0e-8 * scale;
    ExpectNear(actual[0], expected[0], tolerance, description);
    ExpectNear(actual[1], expected[1], tolerance, description);
}

int AdvanceFullyAndObserveMaximumOrder(
    CvodeContinuousStateAdvancer& advancer, double stop_time_seconds) {
    constexpr std::size_t kMaximumSteps = 1'000'000;
    Eigen::VectorXd endpoint(advancer.continuous_state_size());
    int maximum_observed_order = 0;
    for (std::size_t step = 0; step < kMaximumSteps; ++step) {
        if (!(advancer.current_time_seconds() < stop_time_seconds)) {
            return maximum_observed_order;
        }
        const auto result = advancer.AdvanceOneInternalStepToward(
            stop_time_seconds, endpoint);
        maximum_observed_order = std::max(
            maximum_observed_order,
            BdfIntegrationAccess::LastBdfOrder(advancer));
        if (result.reached_stop) return maximum_observed_order;
    }
    Expect(false, "the BDF identity test exceeded its internal-step guard");
    return maximum_observed_order;
}

void CheckPerInstanceBdfOrderIdentity() {
    constexpr double kInitialTime = 0.0;
    constexpr double kFirstTargetTime = 2.0;
    constexpr double kSecondTargetTime = 4.0;
    constexpr double kAngularFrequency = 2.3;
    const Eigen::Vector2d initial_state(0.8, -0.25);

    LinearOscillatorRhs default_rhs(kAngularFrequency);
    CvodeContinuousStateAdvancer default_advancer(
        default_rhs, kInitialTime, initial_state, MakeTolerances());
    Expect(default_advancer.integration_statistics()
                   .requested_dense_finite_difference_jacobian_worker_count ==
               1,
           "the public CVODE recipe retains its serial Jacobian identity");
    Expect(BdfIntegrationAccess::ConfiguredMaximumBdfOrder(
               default_advancer) == 2,
           "the public CVODE construction retains the second-order identity");
    const int maximum_default_order = AdvanceFullyAndObserveMaximumOrder(
        default_advancer, kFirstTargetTime);
    Expect(maximum_default_order > 0 && maximum_default_order <= 2,
           "the public CVODE instance never exceeds second order");
    Eigen::VectorXd default_first_endpoint(2);
    default_advancer.CopyCurrentState(default_first_endpoint);
    default_advancer.ReinitializeAfterExternalChange(
        kFirstTargetTime, default_first_endpoint);
    Expect(BdfIntegrationAccess::ConfiguredMaximumBdfOrder(
               default_advancer) == 2,
           "CVODE reinitialization preserves the second-order identity");
    const int maximum_reinitialized_default_order =
        AdvanceFullyAndObserveMaximumOrder(default_advancer,
                                           kSecondTargetTime);
    Expect(maximum_reinitialized_default_order > 0 &&
               maximum_reinitialized_default_order <= 2,
           "the reinitialized public CVODE instance remains second order");

    LinearOscillatorRhs fifth_order_rhs(kAngularFrequency);
    std::unique_ptr<CvodeContinuousStateAdvancer> fifth_order_advancer =
        BdfIntegrationAccess::MakeFifthOrderCvodeContinuousStateAdvancer(
            fifth_order_rhs, kInitialTime, initial_state, MakeTolerances());
    Expect(fifth_order_advancer->integration_statistics()
                   .requested_dense_finite_difference_jacobian_worker_count ==
               1,
           "the private CVODE recipe retains its serial Jacobian identity");
    Expect(BdfIntegrationAccess::ConfiguredMaximumBdfOrder(
               *fifth_order_advancer) == 5,
           "the private CVODE construction retains the fifth-order identity");
    const int maximum_fifth_order = AdvanceFullyAndObserveMaximumOrder(
        *fifth_order_advancer, kFirstTargetTime);
    Expect(maximum_fifth_order >= 3 && maximum_fifth_order <= 5,
           "the fifth-order CVODE instance actually advances above order two");

    Eigen::VectorXd first_endpoint(2);
    fifth_order_advancer->CopyCurrentState(first_endpoint);
    ExpectStateNear(first_endpoint,
                    OscillatorState(kInitialTime, initial_state,
                                    kAngularFrequency, kFirstTargetTime),
                    "the fifth-order endpoint matches the analytic solution");

    fifth_order_advancer->ReinitializeAfterExternalChange(
        kFirstTargetTime, first_endpoint);
    Expect(BdfIntegrationAccess::ConfiguredMaximumBdfOrder(
               *fifth_order_advancer) == 5,
           "CVODE reinitialization preserves the fifth-order identity");
    const int maximum_reinitialized_order = AdvanceFullyAndObserveMaximumOrder(
        *fifth_order_advancer, kSecondTargetTime);
    Expect(maximum_reinitialized_order >= 3 &&
               maximum_reinitialized_order <= 5,
           "the reinitialized fifth-order instance advances above order two");

    Eigen::VectorXd second_endpoint(2);
    fifth_order_advancer->CopyCurrentState(second_endpoint);
    ExpectStateNear(second_endpoint,
                    OscillatorState(kInitialTime, initial_state,
                                    kAngularFrequency, kSecondTargetTime),
                    "the reinitialized fifth-order endpoint remains accurate");
}

}  // namespace

int main() {
    CheckPerInstanceBdfOrderIdentity();
    if (failure_count != 0) {
        std::printf("CVODE BDF identity checks failed: %d\n", failure_count);
        return 1;
    }
    std::printf("CVODE BDF identity checks passed\n");
    return 0;
}
