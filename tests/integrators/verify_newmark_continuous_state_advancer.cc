#include "basic_coordinate_advancer_contract.h"
#include "basic_coordinate_advancer.h"

#include <deque>
#include <vector>

namespace {
using namespace orvd::integrators;
using namespace orvd::integrators::internal;
using namespace orvd::integrators::internal::test;

// Script only the core outcome to isolate clock/publication boundaries. Real
// Newton recovery is exercised by the core and real-system contract tests.
enum class Action { kAccept, kReject, kCallbackFailure, kBadExport };
struct Attempts {
    std::deque<Action> actions;
    std::vector<double> h;
    std::vector<double> end;
};
struct ScriptConfiguration {
    double step_size_seconds;
    std::shared_ptr<Attempts> attempts;
};
class ScriptCore {
   public:
    using StepResult = NewmarkCore::StepResult;
    ScriptCore(SystemCoordinateProblem&, ScriptConfiguration configuration,
               const CoordinateState& initial)
        : attempts_(std::move(configuration.attempts)), state_(initial) { Reinitialize(initial); }
    StepResult AdvanceOneStep(double h, double end) {
        attempts_->h.push_back(h);
        attempts_->end.push_back(end);
        ++work_.right_hand_side_evaluation_count;
        const Action action = attempts_->actions.empty() ? Action::kAccept : attempts_->actions.front();
        if (!attempts_->actions.empty()) attempts_->actions.pop_front();
        if (action == Action::kCallbackFailure) {
            throw CoordinateIntegrationFailure(CoordinateIntegrationFailure::Reason::kNonlinearConvergenceFailure,
                                               "callback origin");
        }
        if (action == Action::kReject) {
            ++work_.nonlinear_solver_convergence_failure_count;
            return StepResult::kIterationLimit;
        }
        state_.q += h * state_.s;
        state_.time_seconds = end;
        if (action == Action::kBadExport) state_.s[0] = std::numeric_limits<double>::quiet_NaN();
        ++work_.successful_internal_step_count;
        return StepResult::kAccepted;
    }
    void Reinitialize(const CoordinateState& initial) {
        state_ = initial;
        work_ = {};
        work_.right_hand_side_evaluation_count = 1;
    }
    double current_time_seconds() const { return state_.time_seconds; }
    ContinuousStateIntegrationStatistics integration_statistics() const { return work_; }
    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd> q, Eigen::Ref<Eigen::VectorXd> s,
                          Eigen::Ref<Eigen::VectorXd> z) const {
        q = state_.q; s = state_.s; z = state_.z;
    }
   private:
    std::shared_ptr<Attempts> attempts_;
    CoordinateState state_;
    ContinuousStateIntegrationStatistics work_;
};
using Adapter = BasicCoordinateAdvancerImplementation<ScriptCore, ScriptConfiguration,
                                                     NewmarkRecoveryStepPolicy>;

void CheckRecoveryClockAndPublication() {
    SimpleSystem fixture;
    auto attempts = std::make_shared<Attempts>();
    attempts->actions = {Action::kReject, Action::kAccept, Action::kAccept};
    Adapter adapter(*fixture.problem, 0.0, Eigen::Vector2d(0.0, 1.0), {0.1, attempts}, "test");
    Eigen::VectorXd output(2);
    auto step = adapter.Advance(1.0, output);
    Require(step.end_time_seconds == 0.05 && output[0] == 0.05 &&
                attempts->h == std::vector<double>({0.1, 0.05}) &&
                adapter.dense_output_interval()->start_time_seconds == 0.0,
            "a rejected decimal step publishes exactly one reduced step");
    step = adapter.Advance(1.0, output);
    Require(step.end_time_seconds == 0.1, "the second accepted step still uses the reduced interval");
    step = adapter.Advance(1.0, output);
    Require(step.end_time_seconds == 0.2 && attempts->h.back() == 0.1,
            "two publications restore H and reanchor the clock");
    const auto before_dense = adapter.integration_statistics();
    adapter.CopyDenseState(0.15, output);
    Near(output[0], 0.15, 1e-15, "dense interpolation covers only the last accepted substep");
    Require(SameStatistics(before_dense, adapter.integration_statistics()), "sampling adds no retry or RHS work");

    const double short_stop = 0.2 + 0.1 / 4096.0;
    step = adapter.Advance(short_stop, output);
    Require(step.end_time_seconds == short_stop && attempts->h.back() < 0.1 / 1024.0,
            "a first stop-limited attempt below the recovery floor is legal");
    step = adapter.Advance(1.0, output);
    Require(attempts->h.back() == 0.1, "a successful short boundary does not reduce the plan");

    // Each rejection below belongs to a different call: the plan must persist
    // and the intervening single success must not prematurely double it.
    for (double h : {0.05, 0.025, 0.0125, 0.00625, 0.003125}) {
        attempts->actions = {Action::kReject, Action::kAccept};
        step = adapter.Advance(1.0, output);
        Require(attempts->h.back() == h, "cross-call rejections preserve the reduced plan");
    }
    adapter.Reinitialize(0.0, Eigen::Vector2d(0.0, 1.0));
    Require(!adapter.dense_output_interval() && adapter.integration_statistics().right_hand_side_evaluation_count == 1,
            "successful synchronization resets interval and statistics epoch");
    step = adapter.Advance(std::nextafter(0.1, 1.0), output);
    Require(step.reached_stop && attempts->h.back() == 0.1 &&
                attempts->end.back() == std::nextafter(0.1, 1.0),
            "synchronization restores H and snap keeps formula h while setting the RHS endpoint");
    const double boundary_h = 0.025;
    const double start = adapter.current_time_seconds();
    const double stop = start + boundary_h;
    attempts->actions = {Action::kReject, Action::kAccept};
    step = adapter.Advance(stop, output);
    Require(attempts->h.back() == 0.5 * (stop - start) &&
                step.end_time_seconds == start + 0.5 * (stop - start),
            "a rejected short boundary halves its actual interval, not nominal H");
}

void CheckTerminalFailures() {
    using Reason = ContinuousStateNumericalFailure::Reason;
    for (int mode = 0; mode < 6; ++mode) {
        SimpleSystem fixture;
        auto attempts = std::make_shared<Attempts>();
        attempts->actions.assign(12, Action::kReject);
        if (mode == 2) attempts->actions = {Action::kReject, Action::kCallbackFailure};
        if (mode == 3) attempts->actions = {Action::kReject, Action::kBadExport};
        const double h = mode == 4 ? std::numeric_limits<double>::denorm_min() : 1.0;
        const double start = mode == 5 ? std::ldexp(1.0, 52) : 0.0;
        Adapter adapter(*fixture.problem, start, Eigen::Vector2d::Zero(), {h, attempts}, "test");
        Eigen::VectorXd output = Eigen::VectorXd::Constant(2, 91.0);
        const double stop = start + (mode == 1 ? h / 2048.0 : h);
        bool caught = false;
        try { (void)adapter.Advance(stop, output); }
        catch (const ContinuousStateNumericalFailure& failure) {
            const Reason expected = mode == 0 ? Reason::kRepeatedNonlinearConvergenceFailure :
                                    mode >= 4 ? Reason::kStepSizeUnderflow : Reason::kNonlinearConvergenceFailure;
            caught = mode != 3 && failure.reason() == expected;
            if (mode == 2) Require(std::string(failure.what()) == "callback origin", "callback cause is preserved");
        } catch (const std::invalid_argument&) { caught = mode == 3; }
        Require(caught, "terminal retry/export/time failure classification");
        Require(attempts->h.size() == (mode == 0 ? 11u : mode == 2 || mode == 3 ? 2u : 1u),
                "floor bounds attempts; callbacks and export failures never retry");
        Require(adapter.current_time_seconds() == start && (output.array() == 91.0).all() &&
                    !adapter.dense_output_interval() && adapter.integration_statistics().successful_internal_step_count == 0,
                "failed publication preserves output, public endpoint and successful-step accounting");
        Throws<std::logic_error>([&] { (void)adapter.Advance(stop, output); }, "terminal adapter failure requires synchronization");
    }
    NewmarkRecoveryStepPolicy policy(std::numeric_limits<double>::max());
    Require(policy.ReduceAfterRejection(policy.step_size()), "large finite H can be halved");
    Require(!policy.AcceptedStep() && policy.AcceptedStep() &&
                policy.step_size() == std::numeric_limits<double>::max(), "growth cannot overflow before capping at H");
}
}  // namespace

int main() {
    if (test::RunBasicCoordinateAdvancerContract<NewmarkContinuousStateAdvancer>() != 0) return 1;
    try {
        CheckRecoveryClockAndPublication();
        CheckTerminalFailures();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Newmark recovery adapter: " << error.what() << '\n';
        return 1;
    }
}
