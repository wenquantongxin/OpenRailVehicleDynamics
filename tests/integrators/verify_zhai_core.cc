#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "basic_coordinate_problem_fixtures.h"
#include "zhai_core.h"

namespace {

using orvd::integrators::internal::CoordinateIntegrationFailure;
using orvd::integrators::internal::CoordinateSecondOrderProblem;
using orvd::integrators::internal::CoordinateState;
using orvd::integrators::ZhaiConfiguration;
using orvd::integrators::internal::ZhaiCore;
namespace fixtures = orvd::integrators::internal::testing;

void Expect(bool condition, const std::string& description) {
    if (!condition) throw std::runtime_error(description);
}

void ExpectNear(double actual, double expected, double tolerance,
                const std::string& description) {
    if (!std::isfinite(actual) ||
        !(std::abs(actual - expected) <= tolerance)) {
        throw std::runtime_error(description + ": actual=" +
                                 std::to_string(actual) + ", expected=" +
                                 std::to_string(expected));
    }
}

CoordinateState ScalarState(double time, double q, double s, double z) {
    return {time, Eigen::VectorXd::Constant(1, q),
            Eigen::VectorXd::Constant(1, s), Eigen::VectorXd::Constant(1, z)};
}

CoordinateState CopyState(const ZhaiCore& core, int nq, int nz) {
    CoordinateState result{core.current_time_seconds(), Eigen::VectorXd(nq),
                           Eigen::VectorXd(nq), Eigen::VectorXd(nz)};
    core.CopyCurrentState(result.q, result.s, result.z);
    return result;
}

void ExpectSameState(const CoordinateState& actual,
                     const CoordinateState& expected,
                     const std::string& description) {
    Expect(actual.time_seconds == expected.time_seconds &&
               actual.q == expected.q && actual.s == expected.s &&
               actual.z == expected.z,
           description);
}

class TrialFailure final : public std::runtime_error {
   public:
    TrialFailure() : std::runtime_error("armed coordinate trial failure") {}
};

class ProbeProblem final : public CoordinateSecondOrderProblem {
   public:
    int coordinate_size() const override { return 1; }
    int internal_state_size() const override { return 1; }

    void Evaluate(double time, const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>& s,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  Eigen::Ref<Eigen::VectorXd> b,
                  Eigen::Ref<Eigen::VectorXd> g) override {
        ++evaluation_count;
        if (throw_evaluation) throw TrialFailure();
        b[0] = state_dependent ? q[0] : 2.0 + time;
        g[0] = state_dependent ? s[0] : 4.0 - 2.0 * time;
        if (nonfinite_evaluation) b[0] = std::numeric_limits<double>::infinity();
    }

    bool ProjectEndpoint(const Eigen::Ref<const Eigen::VectorXd>&,
                         Eigen::Ref<Eigen::VectorXd> q,
                         Eigen::Ref<Eigen::VectorXd> s) override {
        if (throw_projection) {
            q[0] += 100.0;
            throw TrialFailure();
        }
        if (!project) return false;
        q[0] += 0.25;
        s[0] -= 0.5;
        return true;
    }

    int evaluation_count{};
    bool state_dependent{};
    bool throw_evaluation{};
    bool nonfinite_evaluation{};
    bool throw_projection{};
    bool project{};
};

void VerifyFormulaAndStepHistory() {
    ProbeProblem problem;
    ZhaiCore core(problem, ZhaiConfiguration{0.2}, ScalarState(0, 1, 2, 3));
    Expect(core.integration_statistics().right_hand_side_evaluation_count == 1,
           "initial endpoint is evaluated once");
    core.AdvanceOneStep();
    auto actual = CopyState(core, 1, 1);
    ExpectNear(actual.q[0], 1.44, 2e-15, "startup position");
    ExpectNear(actual.s[0], 2.4, 2e-15, "startup coordinate velocity");
    ExpectNear(actual.z[0], 3.8, 2e-15, "startup internal state uses Euler");
    core.AdvanceOneStep();
    actual = CopyState(core, 1, 1);
    ExpectNear(actual.q[0], 1.968, 2e-15, "second step uses Zhai position");
    ExpectNear(actual.s[0], 2.86, 2e-15, "second step uses Zhai velocity");
    ExpectNear(actual.z[0], 4.48, 2e-15, "second step uses AB2 internal state");
    Expect(core.diagnostics().startup_step_count == 1,
           "the second equal step already has usable history");

    core.AdvanceOneStep(0.05);
    actual = CopyState(core, 1, 1);
    ExpectNear(actual.q[0], 2.114, 3e-15, "short step uses startup position");
    ExpectNear(actual.s[0], 2.98, 3e-15, "short step uses startup velocity");
    ExpectNear(actual.z[0], 4.64, 3e-15, "short step uses Euler internal state");
    core.AdvanceOneStep(0.05);
    Expect(core.diagnostics().startup_step_count == 2,
           "a repeated short length can use normal history");
    core.AdvanceOneStep();
    Expect(core.diagnostics().startup_step_count == 3,
           "returning to the nominal length restarts once");
    core.AdvanceOneStep();
    Expect(core.diagnostics().startup_step_count == 3,
           "normal history resumes after the returning step");
    Expect(core.integration_statistics().successful_internal_step_count == 6 &&
               core.integration_statistics().right_hand_side_evaluation_count ==
                   7,
           "one new endpoint evaluation per successful explicit step");

    const double adjacent_length = std::nextafter(0.2, 0.0);
    core.AdvanceOneStep(adjacent_length);
    Expect(core.diagnostics().startup_step_count == 4,
           "different requested lengths are not merged by a loose tolerance");
    core.AdvanceOneStep(adjacent_length);
    Expect(core.diagnostics().startup_step_count == 4,
           "the repeated adjacent length has valid history");
}

void VerifyProjectionAndFreshEndpoint() {
    ProbeProblem problem;
    problem.state_dependent = true;
    problem.project = true;
    ZhaiCore core(problem, ZhaiConfiguration{0.1}, ScalarState(0, 2, 3, 4));
    core.AdvanceOneStep();
    auto actual = CopyState(core, 1, 1);
    ExpectNear(actual.q[0], 2.56, 2e-15, "first projected position");
    ExpectNear(actual.s[0], 2.7, 2e-15, "first projected velocity");
    core.AdvanceOneStep();
    actual = CopyState(core, 1, 1);
    ExpectNear(actual.q[0], 3.0956, 3e-15,
               "history acceleration belongs to the projected endpoint");
    ExpectNear(actual.s[0], 2.484, 3e-15,
               "normal velocity uses projected endpoint acceleration");
    ExpectNear(actual.z[0], 4.555, 3e-15,
               "internal derivative also belongs to the projected endpoint");
    Expect(core.diagnostics().endpoint_projection_evaluation_count == 2 &&
               core.diagnostics().endpoint_projection_change_count == 2,
           "projection attempts and changes are counted");
    Expect(problem.evaluation_count == 3,
           "Zhai needs no discarded evaluation before endpoint projection");
}

void VerifyFailureTransactionAndReinitialization() {
    ProbeProblem problem;
    ZhaiCore core(problem, ZhaiConfiguration{0.1}, ScalarState(0, 1, 2, 3));
    core.AdvanceOneStep();
    const auto accepted = CopyState(core, 1, 1);
    const auto before = core.integration_statistics();
    problem.throw_evaluation = true;
    bool original_exception = false;
    try {
        core.AdvanceOneStep();
    } catch (const TrialFailure&) {
        original_exception = true;
    }
    Expect(original_exception, "problem exception retains its original type");
    ExpectSameState(CopyState(core, 1, 1), accepted,
                    "failed evaluation cannot commit a candidate");
    Expect(core.integration_statistics().successful_internal_step_count ==
                   before.successful_internal_step_count &&
               core.integration_statistics().right_hand_side_evaluation_count ==
                   before.right_hand_side_evaluation_count + 1,
           "failed trial work is counted without accepting a step");
    Expect(core.diagnostics().startup_step_count == 1,
           "failed steps do not count as successful starts");
    problem.throw_evaluation = false;
    bool blocked = false;
    try {
        core.AdvanceOneStep();
    } catch (const std::logic_error&) {
        blocked = true;
    }
    Expect(blocked, "failure requires explicit reinitialization");

    const auto restart = ScalarState(2.0, 0.5, -0.5, 1.0);
    core.Reinitialize(restart);
    ExpectSameState(CopyState(core, 1, 1), restart,
                    "reinitialization installs the complete endpoint");
    Expect(core.integration_statistics().successful_internal_step_count == 0 &&
               core.integration_statistics().right_hand_side_evaluation_count ==
                   1 &&
               core.diagnostics().startup_step_count == 0,
           "successful reinitialization resets counters and evaluates once");
    core.AdvanceOneStep();
    Expect(core.diagnostics().startup_step_count == 1,
           "reinitialization clears acceleration and internal-state history");

    const auto after_restart = CopyState(core, 1, 1);
    problem.throw_projection = true;
    original_exception = false;
    try {
        core.AdvanceOneStep();
    } catch (const TrialFailure&) {
        original_exception = true;
    }
    Expect(original_exception, "projection exception retains its original type");
    ExpectSameState(CopyState(core, 1, 1), after_restart,
                    "partially modified projection candidate cannot commit");
    problem.throw_projection = false;
    core.Reinitialize(after_restart);
    problem.nonfinite_evaluation = true;
    bool classified = false;
    try {
        core.AdvanceOneStep();
    } catch (const CoordinateIntegrationFailure& failure) {
        classified = failure.reason() ==
                     CoordinateIntegrationFailure::Reason::kNonFiniteEvaluation;
    }
    Expect(classified, "non-finite coordinate acceleration is classified");
    ExpectSameState(CopyState(core, 1, 1), after_restart,
                    "non-finite trial cannot commit");
}

void VerifyCallerErrors() {
    ProbeProblem problem;
    for (double h : {0.0, -0.1, std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            ZhaiCore invalid(problem, ZhaiConfiguration{h},
                             ScalarState(0, 1, 2, 3));
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        Expect(rejected, "invalid nominal step is rejected");
    }
    ZhaiCore core(problem, ZhaiConfiguration{0.1}, ScalarState(0, 1, 2, 3));
    const auto accepted = CopyState(core, 1, 1);
    const auto evaluations = core.integration_statistics().right_hand_side_evaluation_count;
    for (double h : {0.0, -0.1, 0.10001,
                     std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try {
            core.AdvanceOneStep(h);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        Expect(rejected, "invalid requested step is rejected");
        ExpectSameState(CopyState(core, 1, 1), accepted,
                        "caller error preserves the current state");
    }
    Expect(core.integration_statistics().right_hand_side_evaluation_count ==
               evaluations,
           "caller errors perform no trial evaluations");
    Eigen::VectorXd q = Eigen::VectorXd::Constant(1, 17);
    Eigen::VectorXd s = Eigen::VectorXd::Constant(2, 19);
    Eigen::VectorXd z = Eigen::VectorXd::Constant(1, 23);
    bool rejected = false;
    try {
        core.CopyCurrentState(q, s, z);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Expect(rejected && q[0] == 17 && s[0] == 19 && s[1] == 19 && z[0] == 23,
           "copy shape error leaves every output untouched");
    auto invalid_state = accepted;
    invalid_state.z.resize(0);
    rejected = false;
    try {
        core.Reinitialize(invalid_state);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    Expect(rejected, "reinitialization rejects incorrect state shape");
    ExpectSameState(CopyState(core, 1, 1), accepted,
                    "invalid reinitialization preserves the current state");
    core.AdvanceOneStep();
    Expect(core.integration_statistics().successful_internal_step_count == 1,
           "caller errors do not poison the core");
}

class RelaxationProblem final : public CoordinateSecondOrderProblem {
   public:
    explicit RelaxationProblem(double rate) : rate_(rate) {}
    int coordinate_size() const override { return 1; }
    int internal_state_size() const override { return 1; }
    void Evaluate(double, const Eigen::Ref<const Eigen::VectorXd>&,
                  const Eigen::Ref<const Eigen::VectorXd>&,
                  const Eigen::Ref<const Eigen::VectorXd>& z,
                  Eigen::Ref<Eigen::VectorXd> b,
                  Eigen::Ref<Eigen::VectorXd> g) override {
        b.setZero();
        g[0] = -rate_ * z[0];
    }

   private:
    double rate_;
};

void VerifyRelaxationStabilityLimit() {
    RelaxationProblem problem(10.0);
    ZhaiCore stable(problem, ZhaiConfiguration{0.09}, ScalarState(0, 0, 0, 1));
    ZhaiCore unstable(problem, ZhaiConfiguration{0.11}, ScalarState(0, 0, 0, 1));
    stable.AdvanceOneStep();
    unstable.AdvanceOneStep();
    ExpectNear(CopyState(stable, 1, 1).z[0], 0.1, 2e-15,
               "Maxwell start retains the Euler response");
    ExpectNear(CopyState(unstable, 1, 1).z[0], -0.1, 2e-15,
               "unstable Maxwell start is not replaced by exponential decay");
    for (int i = 1; i < 120; ++i) {
        stable.AdvanceOneStep();
        unstable.AdvanceOneStep();
    }
    Expect(std::abs(CopyState(stable, 1, 1).z[0]) < 1e-6,
           "AB2 relaxation decays inside h lambda < 1");
    Expect(std::abs(CopyState(unstable, 1, 1).z[0]) > 100,
           "AB2 relaxation grows outside h lambda < 1");
}

double StateError(const CoordinateState& actual, const CoordinateState& exact) {
    return (actual.q - exact.q).norm() + (actual.s - exact.s).norm() +
           (actual.z - exact.z).norm();
}

template <class Problem>
CoordinateState Integrate(Problem& problem, int steps, double duration) {
    ZhaiCore core(problem, ZhaiConfiguration{duration / steps},
                  problem.InitialState());
    for (int i = 0; i < steps; ++i) core.AdvanceOneStep();
    Expect(core.diagnostics().startup_step_count == 1,
           "constant requested intervals retain history despite time rounding");
    Expect(core.integration_statistics().right_hand_side_evaluation_count ==
               static_cast<std::uint64_t>(steps + 1),
           "equal-step integration performs one evaluation per endpoint");
    return CopyState(core, problem.coordinate_size(), problem.internal_state_size());
}

template <class Problem>
void VerifySecondOrder(Problem& problem, const std::string& description) {
    double errors[3]{};
    const int step_counts[3]{80, 160, 320};
    for (int i = 0; i < 3; ++i) {
        const auto state = Integrate(problem, step_counts[i], 1.0);
        errors[i] = StateError(state, problem.ExactState(state.time_seconds));
    }
    const double first_order = std::log2(errors[0] / errors[1]);
    const double second_order = std::log2(errors[1] / errors[2]);
    std::printf("%s: errors %.6g, %.6g, %.6g; orders %.6g, %.6g\n",
                description.c_str(), errors[0], errors[1], errors[2],
                first_order, second_order);
    Expect(first_order >= 1.8 && first_order <= 2.2 &&
               second_order >= 1.8 && second_order <= 2.2,
           description + " has order 1.8 to 2.2 on both refinements");
}

void VerifyOscillatorsAndCoupling() {
    fixtures::HarmonicOscillatorProblem oscillator;
    VerifySecondOrder(oscillator, "harmonic oscillator");
    fixtures::NonlinearScalarProblem nonlinear;
    VerifySecondOrder(nonlinear, "nonlinear mechanical problem");
    fixtures::MaxwellCoupledProblem maxwell;
    VerifySecondOrder(maxwell, "coupled mechanical/Maxwell problem");

    ZhaiCore stable(oscillator, ZhaiConfiguration{1.99}, oscillator.InitialState());
    ZhaiCore unstable(oscillator, ZhaiConfiguration{2.01}, oscillator.InitialState());
    double maximum_stable_position = 0;
    for (int i = 0; i < 200; ++i) {
        stable.AdvanceOneStep();
        unstable.AdvanceOneStep();
        maximum_stable_position =
            std::max(maximum_stable_position, std::abs(CopyState(stable, 1, 0).q[0]));
    }
    Expect(maximum_stable_position < 2,
           "undamped mechanical motion remains bounded below h omega = 2");
    Expect(std::abs(CopyState(unstable, 1, 0).q[0]) > 1e10,
           "undamped motion grows above h omega = 2");
}

void VerifyCurvilinearCoordinates() {
    fixtures::RpyRotationProblem rpy;
    VerifySecondOrder(rpy, "noncommuting RPY rotation");
    fixtures::QuaternionRotationProblem quaternion;
    VerifySecondOrder(quaternion, "projected quaternion rotation");

    fixtures::RpyRotationProblem missing_convective_term(true);
    const auto bad_coarse = Integrate(missing_convective_term, 80, 1.0);
    const auto bad_fine = Integrate(missing_convective_term, 160, 1.0);
    const auto good = Integrate(rpy, 160, 1.0);
    const double coarse_error = StateError(
        bad_coarse, missing_convective_term.ExactState(bad_coarse.time_seconds));
    const double fine_error = StateError(
        bad_fine, missing_convective_term.ExactState(bad_fine.time_seconds));
    const double good_error = StateError(good, rpy.ExactState(good.time_seconds));
    Expect(fine_error > 100 * good_error && fine_error > 0.8 * coarse_error,
           "omitting the coordinate acceleration bias converges to a wrong ODE");

    for (double sign : {-1.0, 1.0}) {
        auto initial = quaternion.InitialState();
        initial.q *= sign;
        initial.s *= sign;
        ZhaiCore core(quaternion, ZhaiConfiguration{0.01}, initial);
        for (int i = 0; i < 100; ++i) core.AdvanceOneStep();
        const auto final = CopyState(core, 4, 0);
        const auto exact = quaternion.ExactState(final.time_seconds);
        ExpectNear(final.q.norm(), initial.q.norm(), 2e-14,
                   "projection preserves the initialization norm");
        ExpectNear(final.q.dot(final.s), 0.0, 2e-14,
                   "projection keeps coordinate velocity tangent");
        Expect(final.q.dot(initial.q) > 0,
               "projection does not switch the quaternion representative");
        Expect(fixtures::RotationAngleError(quaternion.Rotation(final.q),
                                            quaternion.Rotation(exact.q)) < 1e-4,
               "both quaternion signs reproduce the physical orientation");
        Expect((quaternion.PhysicalAngularVelocity(final.q, final.s) -
                quaternion.PhysicalAngularVelocity(exact.q, exact.s)).norm() < 1e-4,
               "physical angular velocity comes from the evolved q and s");
        Expect(core.diagnostics().endpoint_projection_change_count > 0,
               "quaternion test actually exercises projection");

        auto rescaled = exact;
        rescaled.q *= -2.0;
        rescaled.s *= -2.0;
        core.Reinitialize(rescaled);
        core.AdvanceOneStep();
        ExpectNear(CopyState(core, 4, 0).q.norm(), rescaled.q.norm(), 3e-14,
                   "successful reinitialization installs a new projection reference");
    }
}

}  // namespace

int main() {
    try {
        VerifyFormulaAndStepHistory();
        VerifyProjectionAndFreshEndpoint();
        VerifyFailureTransactionAndReinitialization();
        VerifyCallerErrors();
        VerifyRelaxationStabilityLimit();
        VerifyOscillatorsAndCoupling();
        VerifyCurvilinearCoordinates();
        std::puts("Zhai core verification passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "Zhai core verification failed: %s\n", error.what());
        return 1;
    }
}
