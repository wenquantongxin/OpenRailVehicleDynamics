#include "continuous_state_advancer_contract.h"

#include <cstdio>
#include <exception>
#include <memory>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>

#include "radau5_continuous_state_advancer.h"

namespace {

class DeliberateRecoverableStageFailure final : public std::runtime_error {
   public:
    DeliberateRecoverableStageFailure()
        : std::runtime_error("deliberate recoverable Radau5 stage failure") {}
};

class PersistentRecoverableStageRhs final
    : public orvd::integrators::ContinuousStateRhs {
   public:
    [[nodiscard]] int continuous_state_size() const override { return 1; }

    void CalcTimeDerivatives(
        double time_seconds,
        const Eigen::Ref<const Eigen::VectorXd>& state,
        Eigen::Ref<Eigen::VectorXd> derivatives) override {
        if (enabled_ && time_seconds > accepted_time_seconds_) {
            throw DeliberateRecoverableStageFailure();
        }
        derivatives[0] = -state[0];
    }

    [[nodiscard]] bool IsRecoverableFailure(
        const std::exception_ptr& failure) const noexcept override {
        if (failure == nullptr) return false;
        try {
            std::rethrow_exception(failure);
        } catch (const DeliberateRecoverableStageFailure&) {
            return true;
        } catch (...) {
            return false;
        }
    }

    void DisableAndMoveAcceptedTime(double accepted_time_seconds) noexcept {
        enabled_ = false;
        accepted_time_seconds_ = accepted_time_seconds;
    }

   private:
    bool enabled_{true};
    double accepted_time_seconds_{};
};

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

int VerifyRecoverableExceptionCausality() {
    int failure_count = 0;
    const auto expect = [&](bool condition, const char* message) {
        if (!condition) {
            std::printf("FAIL %s\n", message);
            ++failure_count;
        }
    };

    PersistentRecoverableStageRhs rhs;
    Eigen::VectorXd initial(1);
    initial[0] = 1.0;
    auto advancer = MakeRadau5Advancer(
        rhs, 0.0, initial,
        orvd::integrators::ContinuousStateErrorTolerances(
            1.0e-7, Eigen::VectorXd::Constant(1, 1.0e-9)));
    Eigen::VectorXd output = Eigen::VectorXd::Constant(1, 91.0);
    bool original_type_rethrown = false;
    try {
        static_cast<void>(
            advancer->AdvanceOneInternalStepToward(0.1, output));
    } catch (const DeliberateRecoverableStageFailure&) {
        original_type_rethrown = true;
    } catch (...) {
    }
    expect(original_type_rethrown,
           "persistent recoverable stage failure did not retain its "
           "original C++ exception type");
    expect(output[0] == 91.0 && advancer->current_time_seconds() == 0.0 &&
               !advancer->dense_output_interval().has_value() &&
               advancer->integration_statistics()
                       .successful_internal_step_count == 0,
           "recoverable exhaustion violated endpoint, output, dense, or "
           "statistics atomicity");

    Eigen::VectorXd poisoned = Eigen::VectorXd::Constant(1, 92.0);
    bool poisoned_until_reinitialize = false;
    try {
        static_cast<void>(
            advancer->AdvanceOneInternalStepToward(0.1, poisoned));
    } catch (const std::logic_error&) {
        poisoned_until_reinitialize = true;
    }
    expect(poisoned_until_reinitialize && poisoned[0] == 92.0,
           "recoverable exhaustion did not poison reuse atomically");

    rhs.DisableAndMoveAcceptedTime(0.0);
    advancer->ReinitializeAfterExternalChange(0.0, initial);
    const auto recovered =
        advancer->AdvanceOneInternalStepToward(1.0e-6, output);
    expect(recovered.reached_stop && recovered.end_time_seconds == 1.0e-6,
           "successful reinitialization did not reopen advancement after "
           "recoverable exhaustion");
    return failure_count;
}

}  // namespace

int main() {
    const int failure_count =
        orvd::integrators::test::RunContinuousStateAdvancerContract(
            &MakeRadau5Advancer) +
        VerifyRecoverableExceptionCausality();
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
