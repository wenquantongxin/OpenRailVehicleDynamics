#include "newmark_core.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "basic_coordinate_problem_fixtures.h"

namespace {

using orvd::integrators::internal::CoordinateIntegrationFailure;
using orvd::integrators::internal::CoordinateSecondOrderProblem;
using orvd::integrators::internal::CoordinateState;
using orvd::integrators::internal::NewmarkCoreConfiguration;
using orvd::integrators::internal::NewmarkCore;
using namespace orvd::integrators::internal::testing;
using ConstVector = const Eigen::Ref<const Eigen::VectorXd>&;
using MutableVector = Eigen::Ref<Eigen::VectorXd>;

void Expect(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void ExpectNear(double actual, double expected, double tolerance,
                const std::string& message) {
    Expect(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
           message + ": actual=" + std::to_string(actual) +
               ", expected=" + std::to_string(expected));
}

template <typename Exception, typename Operation>
void ExpectThrows(Operation&& operation, const std::string& message) {
    bool caught = false;
    try {
        operation();
    } catch (const Exception&) {
        caught = true;
    }
    Expect(caught, message);
}

template <typename Operation>
void ExpectFailure(Operation&& operation,
                   CoordinateIntegrationFailure::Reason reason,
                   const std::string& message) {
    bool caught = false;
    try {
        operation();
    } catch (const CoordinateIntegrationFailure& failure) {
        Expect(failure.reason() == reason, message + ": wrong failure reason");
        caught = true;
    }
    Expect(caught, message);
}

NewmarkCoreConfiguration Configuration(const CoordinateSecondOrderProblem& problem,
                                    double step, double tolerance = 1e-11) {
    NewmarkCoreConfiguration result;
    result.step_size_seconds = step;
    auto& solver = result.nonlinear_solver;
    solver.position_correction_scales =
        Eigen::VectorXd::Constant(problem.coordinate_size(), tolerance);
    solver.velocity_correction_scales = solver.position_correction_scales;
    solver.internal_state_correction_scales =
        Eigen::VectorXd::Constant(problem.internal_state_size(), tolerance);
    solver.acceleration_residual_scales = solver.position_correction_scales;
    solver.internal_state_residual_scales =
        solver.internal_state_correction_scales;
    solver.unknown_reference_scales = Eigen::VectorXd::Ones(
        problem.coordinate_size() + problem.internal_state_size());
    return result;
}

CoordinateState ReadState(const NewmarkCore& core,
                          const CoordinateSecondOrderProblem& problem) {
    CoordinateState result;
    result.time_seconds = core.current_time_seconds();
    result.q.resize(problem.coordinate_size());
    result.s.resize(problem.coordinate_size());
    result.z.resize(problem.internal_state_size());
    core.CopyCurrentState(result.q, result.s, result.z);
    return result;
}

void ExpectStateUnchanged(const CoordinateState& actual,
                          const CoordinateState& expected,
                          const std::string& message) {
    Expect(actual.time_seconds == expected.time_seconds &&
               (actual.q.array() == expected.q.array()).all() &&
               (actual.s.array() == expected.s.array()).all() &&
               (actual.z.array() == expected.z.array()).all(),
           message);
}

class CallbackProblem final : public CoordinateSecondOrderProblem {
   public:
    using Evaluation = std::function<void(double, ConstVector, ConstVector,
                                         ConstVector, MutableVector, MutableVector)>;
    using Projection =
        std::function<bool(ConstVector, MutableVector, MutableVector)>;

    CallbackProblem(int nq, int nz, Evaluation evaluate)
        : evaluate_(std::move(evaluate)), nq_(nq), nz_(nz) {}
    int coordinate_size() const override { return nq_; }
    int internal_state_size() const override { return nz_; }
    void Evaluate(double time, ConstVector q, ConstVector s, ConstVector z,
                  MutableVector b, MutableVector g) override {
        ++evaluation_count;
        last_evaluated_q = q;
        last_evaluated_s = s;
        last_evaluated_z = z;
        evaluate_(time, q, s, z, b, g);
    }
    bool ProjectEndpoint(ConstVector reference, MutableVector q,
                         MutableVector s) override {
        ++projection_count;
        return project ? project(reference, q, s) : false;
    }
    CoordinateState InitialState(double q = 0.0, double s = 0.0,
                                 double z = 0.0) const {
        return {0.0, Eigen::VectorXd::Constant(nq_, q),
                Eigen::VectorXd::Constant(nq_, s),
                Eigen::VectorXd::Constant(nz_, z)};
    }

    Evaluation evaluate_;
    Projection project;
    int evaluation_count{};
    int projection_count{};
    Eigen::VectorXd last_evaluated_q;
    Eigen::VectorXd last_evaluated_s;
    Eigen::VectorXd last_evaluated_z;

   private:
    int nq_;
    int nz_;
};

void CheckOrders(const std::vector<double>& errors, const std::string& name) {
    Expect(errors.size() == 4, "four convergence levels are required");
    for (std::size_t level = 2; level < errors.size(); ++level) {
        const double order = std::log2(errors[level - 1] / errors[level]);
        Expect(std::isfinite(order) && order >= 1.8 && order <= 2.2,
               name + ": observed order=" + std::to_string(order));
    }
}

template <typename Problem>
void CheckCoordinateConvergence(Problem& problem, const std::string& name) {
    std::vector<double> q_errors, s_errors, z_errors;
    constexpr double end_time = 0.8;
    for (const int steps : {20, 40, 80, 160}) {
        NewmarkCore core(problem, Configuration(problem, end_time / steps),
                         problem.InitialState());
        for (int i = 0; i < steps; ++i) core.AdvanceOneStep();
        const auto actual = ReadState(core, problem);
        const auto expected = problem.ExactState(end_time);
        q_errors.push_back((actual.q - expected.q).norm());
        s_errors.push_back((actual.s - expected.s).norm());
        if (problem.internal_state_size() != 0) {
            z_errors.push_back((actual.z - expected.z).norm());
        }
        ExpectNear(actual.time_seconds, end_time, 1e-14, name + ": time");
        Expect(core.integration_statistics().successful_internal_step_count ==
                   static_cast<std::uint64_t>(steps),
               name + ": successful steps");
    }
    CheckOrders(q_errors, name + " position");
    CheckOrders(s_errors, name + " coordinate rate");
    if (!z_errors.empty()) CheckOrders(z_errors, name + " internal state");
}

template <typename Problem>
void CheckRotationConvergence(Problem& problem, const std::string& name,
                              bool quaternion) {
    std::vector<double> angle_errors, velocity_errors;
    constexpr double end_time = 0.8;
    for (const int steps : {20, 40, 80, 160}) {
        const auto initial = problem.InitialState();
        NewmarkCore core(problem, Configuration(problem, end_time / steps), initial);
        Expect(core.diagnostics().endpoint_projection_evaluation_count == 0,
               name + ": initialization must not project");
        for (int i = 0; i < steps; ++i) {
            core.AdvanceOneStep();
            if (quaternion) {
                const auto current = ReadState(core, problem);
                ExpectNear(current.q.norm(), initial.q.norm(), 2e-14,
                           name + ": accepted quaternion norm");
                ExpectNear(current.q.dot(current.s), 0.0, 2e-14,
                           name + ": accepted tangent coordinate rate");
            }
        }
        const auto actual = ReadState(core, problem);
        const auto expected = problem.ExactState(end_time);
        angle_errors.push_back(RotationAngleError(problem.Rotation(actual.q),
                                                  problem.Rotation(expected.q)));
        velocity_errors.push_back((problem.PhysicalAngularVelocity(actual.q, actual.s) -
                                   problem.PhysicalAngularVelocity(expected.q, expected.s))
                                      .norm());
        Expect(core.diagnostics().endpoint_projection_evaluation_count ==
                   static_cast<std::uint64_t>(steps),
               name + ": one projection invocation per converged endpoint");
    }
    CheckOrders(angle_errors, name + " attitude");
    CheckOrders(velocity_errors, name + " physical angular velocity");
}

void VerifyBasicFormulasAndConvergence() {
    HarmonicOscillatorProblem oscillator(2.0);
    NewmarkCore core(oscillator, Configuration(oscillator, 0.1),
                     oscillator.InitialState());
    CoordinateState expected = oscillator.InitialState();
    // Average acceleration on this linear oscillator is exactly trapezoidal.
    // Include a short boundary step, followed by a normal step.
    for (const double h : {0.1, 0.037, 0.1}) {
        const double q1 = ((1.0 - h * h) * expected.q[0] + h * expected.s[0]) /
                          (1.0 + h * h);
        const double s1 = expected.s[0] - 2.0 * h * (expected.q[0] + q1);
        core.AdvanceOneStep(h);
        const auto actual = ReadState(core, oscillator);
        ExpectNear(actual.q[0], q1, 5e-12, "average acceleration position");
        ExpectNear(actual.s[0], s1, 5e-12, "average acceleration velocity");
        expected.q[0] = q1;
        expected.s[0] = s1;
    }

    HarmonicOscillatorProblem smooth_oscillator;
    NonlinearScalarProblem nonlinear;
    MaxwellCoupledProblem coupled;
    CheckCoordinateConvergence(smooth_oscillator, "linear oscillator");
    CheckCoordinateConvergence(nonlinear, "nonlinear oscillator");
    CheckCoordinateConvergence(coupled, "mechanical-Maxwell coupling");
    RpyRotationProblem rpy;
    QuaternionRotationProblem quaternion;
    CheckRotationConvergence(rpy, "noncommuting Ball-RPY", false);
    CheckRotationConvergence(quaternion, "nonunit quaternion", true);

    // Removing the curvilinear acceleration term integrates a different ODE.
    // Refinement must not disguise that error as an implementation of q''.
    RpyRotationProblem missing_bias(true);
    std::array<double, 2> wrong_errors{};
    for (int level = 0; level != 2; ++level) {
        const int steps = level == 0 ? 80 : 160;
        NewmarkCore wrong(missing_bias, Configuration(missing_bias, 0.8 / steps),
                          missing_bias.InitialState());
        for (int i = 0; i < steps; ++i) wrong.AdvanceOneStep();
        const auto actual = ReadState(wrong, missing_bias);
        wrong_errors[level] = RotationAngleError(
            missing_bias.Rotation(actual.q), missing_bias.Rotation(missing_bias.ExactState(0.8).q));
    }
    Expect(wrong_errors[1] > 1e-3 && wrong_errors[1] / wrong_errors[0] > 0.95,
           "missing RPY convective term must leave an error plateau");
}

void VerifyTrapezoidalInternalState() {
    CallbackProblem relaxation(1, 1,
        [](double, ConstVector, ConstVector, ConstVector z, MutableVector b, MutableVector g) {
            b[0] = 0.0;
            g[0] = -1000.0 * z[0];
        });
    NewmarkCore core(relaxation, Configuration(relaxation, 0.1),
                     relaxation.InitialState(0.0, 0.0, 1.0));
    double force = 1.0;
    for (int i = 0; i < 3; ++i) {
        core.AdvanceOneStep();
        force *= -49.0 / 51.0;
        ExpectNear(ReadState(core, relaxation).z[0], force, 1e-12,
                   "basic trapezoidal z retains its stiff alternating mode");
    }
}

void VerifyAcceptedDerivativeAndProjectionRefresh() {
    CallbackProblem problem(1, 1,
        [](double, ConstVector q, ConstVector, ConstVector z, MutableVector b, MutableVector g) {
            b[0] = -q[0];
            g[0] = z[0];
        });
    // Deliberately permissive nonlinear thresholds accept the initial guess.
    // The next step must nevertheless use B/G evaluated at that accepted state,
    // not the old acceleration guess or the unknown used by the residual.
    NewmarkCore core(problem, Configuration(problem, 0.5, 10.0),
                     problem.InitialState(1.0, 0.0, 1.0));
    core.AdvanceOneStep();
    auto state = ReadState(core, problem);
    ExpectNear(state.q[0], 0.875, 0.0, "permissive first position");
    ExpectNear(state.s[0], -0.5, 0.0, "permissive first rate");
    ExpectNear(state.z[0], 1.5, 0.0, "permissive first internal state");
    core.AdvanceOneStep();
    state = ReadState(core, problem);
    ExpectNear(state.q[0], 0.515625, 0.0, "accepted B seeds the next position");
    ExpectNear(state.s[0], -0.9375, 0.0, "accepted B seeds the next rate");
    ExpectNear(state.z[0], 2.25, 0.0, "accepted G seeds the next internal state");
    Expect(core.integration_statistics().jacobian_evaluation_count == 0,
           "an already acceptable residual needs no Newton correction");

    CallbackProblem projected(1, 1,
        [](double, ConstVector q, ConstVector, ConstVector z, MutableVector b, MutableVector g) {
            b[0] = q[0];
            g[0] = q[0] + z[0];
        });
    projected.project = [](ConstVector, MutableVector q, MutableVector) {
        q[0] += 1.0;
        return true;
    };
    auto expected = projected.InitialState(0.5, 0.25, 0.4);
    NewmarkCore projected_core(projected, Configuration(projected, 0.1), expected);
    for (int i = 0; i < 2; ++i) {
        constexpr double h = 0.1;
        const double raw_q =
            (expected.q[0] + h * expected.s[0] + 0.25 * h * h * expected.q[0]) /
            (1.0 - 0.25 * h * h);
        const double raw_s = expected.s[0] + 0.5 * h * (expected.q[0] + raw_q);
        const double raw_z =
            (expected.z[0] + 0.5 * h * (expected.q[0] + expected.z[0] + raw_q)) /
            (1.0 - 0.5 * h);
        projected_core.AdvanceOneStep();
        const auto actual = ReadState(projected_core, projected);
        ExpectNear(actual.q[0], raw_q + 1.0, 1e-11, "projected endpoint position");
        ExpectNear(actual.s[0], raw_s, 1e-11, "projected endpoint rate");
        ExpectNear(actual.z[0], raw_z, 1e-11, "projected endpoint internal state");
        ExpectNear(projected.last_evaluated_q[0], actual.q[0], 0.0,
                   "projection must refresh the physical endpoint evaluation");
        expected = actual;
    }
    const auto stats = projected_core.integration_statistics();
    Expect(projected.projection_count == 2 &&
               projected_core.diagnostics().endpoint_projection_change_count == 2,
           "Newton and finite-difference trials must not invoke projection");
    Expect(stats.right_hand_side_evaluation_count +
                   stats.linear_solver_right_hand_side_evaluation_count ==
               static_cast<std::uint64_t>(projected.evaluation_count),
           "ordinary, Jacobian and projection evaluations must all be counted");
    Expect(stats.linear_solver_right_hand_side_evaluation_count ==
               2 * stats.jacobian_evaluation_count,
           "the complete coupled residual needs nq+nz difference columns");
}

void VerifyFullQuaternionResidualBeforeProjection() {
    QuaternionRotationProblem quaternion;
    CallbackProblem observed(4, 0,
        [&](double time, ConstVector q, ConstVector s, ConstVector z,
            MutableVector b, MutableVector g) {
            quaternion.Evaluate(time, q, s, z, b, g);
        });
    constexpr double h = 0.05;
    auto previous = quaternion.InitialState();
    Eigen::VectorXd previous_b(4), previous_g(0);
    quaternion.Evaluate(previous.time_seconds, previous.q, previous.s, previous.z,
                        previous_b, previous_g);
    int checked_residuals = 0;
    observed.project = [&](ConstVector reference, MutableVector q, MutableVector s) {
        // This callback sees the converged candidate before storage adaptation.
        // Reconstruct the acceleration unknown from the velocity equation,
        // then test all four residual components, including the radial one.
        const Eigen::VectorXd unknown_b = 2.0 * (s - previous.s) / h - previous_b;
        Eigen::VectorXd physical_b(4), physical_g(0);
        quaternion.Evaluate(previous.time_seconds + h, q, s, previous.z,
                            physical_b, physical_g);
        const Eigen::VectorXd residual = unknown_b - physical_b;
        ExpectNear(residual.norm(), 0.0, 3e-11,
                   "full quaternion acceleration residual before projection");
        ExpectNear(q.dot(residual) / q.norm(), 0.0, 3e-11,
                   "quaternion radial acceleration residual before projection");
        Expect(q.dot(physical_b) < -1e-3,
               "the raw quaternion test must exercise nonzero radial acceleration");
        ++checked_residuals;
        return quaternion.ProjectEndpoint(reference, q, s);
    };
    NewmarkCore core(observed, Configuration(observed, h), previous);
    for (int step = 0; step < 6; ++step) {
        core.AdvanceOneStep();
        previous = ReadState(core, observed);
        quaternion.Evaluate(previous.time_seconds, previous.q, previous.s,
                            previous.z, previous_b, previous_g);
    }
    Expect(checked_residuals == 6,
           "every successful quaternion solve must satisfy the full raw residual");
}

void VerifyFailedReinitializationTransaction() {
    QuaternionRotationProblem quaternion;
    bool fail_evaluation = false;
    struct InitialEvaluationFailure final : std::runtime_error {
        InitialEvaluationFailure() : std::runtime_error("deliberate initialization RHS failure") {}
    };
    CallbackProblem observed(4, 0,
        [&](double time, ConstVector q, ConstVector s, ConstVector z,
            MutableVector b, MutableVector g) {
            if (fail_evaluation) throw InitialEvaluationFailure();
            quaternion.Evaluate(time, q, s, z, b, g);
        });
    double observed_reference_norm = 0.0;
    observed.project = [&](ConstVector reference, MutableVector q, MutableVector s) {
        observed_reference_norm = reference.norm();
        return quaternion.ProjectEndpoint(reference, q, s);
    };
    NewmarkCore core(observed, Configuration(observed, 0.01), quaternion.InitialState());
    core.AdvanceOneStep();
    const auto accepted = ReadState(core, observed);
    const auto prior_work = core.integration_statistics();
    const auto prior_projection = core.diagnostics();
    auto replacement = quaternion.ExactState(0.37);
    replacement.q *= 2.4 / replacement.q.norm();
    replacement.s *= 2.4 / quaternion.InitialState().q.norm();
    auto replacement_configuration = Configuration(observed, 0.01, 3e-11);
    replacement_configuration.nonlinear_solver.unknown_reference_scales.setConstant(7.0);
    const auto old_configuration = core.configuration();
    fail_evaluation = true;
    ExpectThrows<InitialEvaluationFailure>([&] { core.Reinitialize(replacement, replacement_configuration); },
                                           "initialization RHS exception must propagate");
    Expect((core.configuration().nonlinear_solver.position_correction_scales.array() ==
            old_configuration.nonlinear_solver.position_correction_scales.array()).all() &&
           (core.configuration().nonlinear_solver.unknown_reference_scales.array() ==
            old_configuration.nonlinear_solver.unknown_reference_scales.array()).all(),
           "failed initialization must preserve Newton scales and difference references");
    ExpectStateUnchanged(ReadState(core, observed), accepted,
                         "failed reinitialization preserves the accepted state");
    const auto failed_work = core.integration_statistics();
    Expect(failed_work.right_hand_side_evaluation_count ==
                   prior_work.right_hand_side_evaluation_count + 1 &&
               failed_work.successful_internal_step_count == prior_work.successful_internal_step_count &&
               failed_work.linear_solver_right_hand_side_evaluation_count ==
                   prior_work.linear_solver_right_hand_side_evaluation_count &&
               failed_work.jacobian_evaluation_count == prior_work.jacobian_evaluation_count,
           "failed reinitialization retains prior work and counts the failing evaluation");
    Expect(core.diagnostics().endpoint_projection_evaluation_count ==
               prior_projection.endpoint_projection_evaluation_count,
           "a failed reinitialization must not project a candidate");
    ExpectThrows<std::logic_error>([&] { core.AdvanceOneStep(); },
                                   "failed reinitialization blocks further advancement");
    ExpectNear(observed_reference_norm, accepted.q.norm(), 1e-14,
               "no new projection reference may be consumed before successful reinitialization");

    fail_evaluation = false;
    // The reference is deliberately not exposed by the core.  Recovery with the
    // old accepted state and then a new initialization checks its public lifecycle.
    core.Reinitialize(accepted);
    Expect(core.integration_statistics().right_hand_side_evaluation_count == 1 &&
               core.integration_statistics().successful_internal_step_count == 0,
           "successful recovery alone resets accumulated work");
    core.AdvanceOneStep();
    ExpectNear(observed_reference_norm, accepted.q.norm(), 1e-14,
               "recovery with the old state retains its norm convention");
    core.Reinitialize(replacement, replacement_configuration);
    Expect((core.configuration().nonlinear_solver.position_correction_scales.array() ==
            replacement_configuration.nonlinear_solver.position_correction_scales.array()).all() &&
           (core.configuration().nonlinear_solver.unknown_reference_scales.array() ==
            replacement_configuration.nonlinear_solver.unknown_reference_scales.array()).all(),
           "successful initialization must commit the prepared Newton configuration");
    core.AdvanceOneStep();
    ExpectNear(observed_reference_norm, 2.4, 1e-14,
               "successful new initialization publishes the new projection reference");
    ExpectNear(ReadState(core, observed).q.norm(), 2.4, 1e-14,
               "the new projection reference governs subsequent accepted states");
}

void VerifyFailureTransactions() {
    NonlinearScalarProblem nonlinear;
    auto limited = Configuration(nonlinear, 0.5, 1e-13);
    limited.nonlinear_solver.maximum_iterations = 1;
    NewmarkCore limited_core(nonlinear, limited, nonlinear.InitialState());
    const auto before = ReadState(limited_core, nonlinear);
    ExpectFailure([&] { limited_core.AdvanceOneStep(); },
                  CoordinateIntegrationFailure::Reason::kNonlinearConvergenceFailure,
                  "iteration limit must fail");
    ExpectStateUnchanged(ReadState(limited_core, nonlinear), before,
                         "failed Newton must not commit trial state");
    Expect(limited_core.integration_statistics().successful_internal_step_count == 0 &&
               limited_core.integration_statistics().nonlinear_solver_convergence_failure_count == 1 &&
               limited_core.integration_statistics().linear_solver_right_hand_side_evaluation_count > 0,
           "failed Newton work must remain visible");
    ExpectThrows<std::logic_error>([&] { limited_core.AdvanceOneStep(); },
                                   "continuation after failure must require reinitialization");
    limited_core.Reinitialize(before);
    Expect(limited_core.integration_statistics().right_hand_side_evaluation_count == 1 &&
               limited_core.integration_statistics().nonlinear_solver_iteration_count == 0,
           "successful reinitialization resets work and evaluates once");

    CallbackProblem singular(1, 0,
        [](double, ConstVector q, ConstVector, ConstVector, MutableVector b, MutableVector) {
            b[0] = 4.0 * q[0] + 1.0;
        });
    const auto zero = singular.InitialState();
    NewmarkCore singular_core(singular, Configuration(singular, 1.0), zero);
    // With h=1 the residual is the nonzero constant -2: no endpoint exists.
    ExpectFailure([&] { singular_core.AdvanceOneStep(); },
                  CoordinateIntegrationFailure::Reason::kSingularJacobian,
                  "singular endpoint Jacobian must be reported");
    ExpectStateUnchanged(ReadState(singular_core, singular), zero,
                         "singular solve must not commit");

    bool fail_evaluation = true;
    CallbackProblem nonfinite(1, 0,
        [&](double time, ConstVector, ConstVector, ConstVector, MutableVector b, MutableVector) {
            b[0] = fail_evaluation && time > 0.0
                       ? std::numeric_limits<double>::quiet_NaN() : 1.0;
        });
    NewmarkCore bad_core(nonfinite, Configuration(nonfinite, 0.1), nonfinite.InitialState());
    const auto bad_before = ReadState(bad_core, nonfinite);
    ExpectFailure([&] { bad_core.AdvanceOneStep(); },
                  CoordinateIntegrationFailure::Reason::kNonFiniteEvaluation,
                  "non-finite trial evaluation must fail");
    ExpectStateUnchanged(ReadState(bad_core, nonfinite), bad_before,
                         "non-finite evaluation must not commit");
    fail_evaluation = false;
    bad_core.Reinitialize(bad_before);
    bad_core.AdvanceOneStep();
    ExpectNear(ReadState(bad_core, nonfinite).q[0], 0.005, 1e-16,
               "reinitialization restores usable acceleration history");

    struct ProjectionFailure final : std::runtime_error {
        ProjectionFailure() : std::runtime_error("deliberate projection failure") {}
    };
    CallbackProblem projection(1, 0,
        [](double, ConstVector, ConstVector, ConstVector, MutableVector b, MutableVector) {
            b[0] = 1.0;
        });
    projection.project = [](ConstVector, MutableVector q, MutableVector) -> bool {
        q[0] += 100.0;
        throw ProjectionFailure();
    };
    NewmarkCore projection_core(projection, Configuration(projection, 0.1),
                                projection.InitialState());
    const auto projection_before = ReadState(projection_core, projection);
    ExpectThrows<ProjectionFailure>([&] { projection_core.AdvanceOneStep(); },
                                     "the projection exception must propagate");
    ExpectStateUnchanged(ReadState(projection_core, projection), projection_before,
                         "projection mutation before failure must not leak");
    Expect(projection_core.diagnostics().endpoint_projection_evaluation_count == 1 &&
               projection_core.integration_statistics().successful_internal_step_count == 0,
           "failed projection attempt must be counted without accepting a step");

    HarmonicOscillatorProblem oscillator;
    auto distant = oscillator.InitialState();
    distant.time_seconds = 1e20;
    NewmarkCore underflow(oscillator, Configuration(oscillator, 0.1), distant);
    ExpectFailure([&] { underflow.AdvanceOneStep(); },
                  CoordinateIntegrationFailure::Reason::kStepSizeUnderflow,
                  "a step must advance representable time");
    ExpectStateUnchanged(ReadState(underflow, oscillator), distant,
                         "time underflow must not commit");
}

void VerifyValidationAndQuaternionRestart() {
    HarmonicOscillatorProblem oscillator;
    auto configuration = Configuration(oscillator, 0.1);
    auto invalid = configuration;
    invalid.nonlinear_solver.acceleration_residual_scales[0] = 0.0;
    ExpectThrows<std::invalid_argument>(
        [&] { NewmarkCore core(oscillator, invalid, oscillator.InitialState()); },
        "zero residual scales must be rejected");
    invalid = configuration;
    invalid.nonlinear_solver.unknown_reference_scales.resize(0);
    ExpectThrows<std::invalid_argument>(
        [&] { NewmarkCore core(oscillator, invalid, oscillator.InitialState()); },
        "unknown reference scale dimensions must be checked");
    NewmarkCore core(oscillator, configuration, oscillator.InitialState());
    for (const double h : {0.0, -0.1, 0.2, std::numeric_limits<double>::quiet_NaN()}) {
        ExpectThrows<std::invalid_argument>([&] { core.AdvanceOneStep(h); },
                                            "invalid step must be rejected before entry");
    }
    Expect(core.integration_statistics().right_hand_side_evaluation_count == 1,
           "invalid step requests must not evaluate the problem");
    Eigen::VectorXd q = Eigen::VectorXd::Constant(1, 12.0);
    Eigen::VectorXd s = Eigen::VectorXd::Constant(2, 13.0);
    Eigen::VectorXd z(0);
    ExpectThrows<std::invalid_argument>([&] { core.CopyCurrentState(q, s, z); },
                                        "invalid copy dimensions must be rejected");
    Expect(q[0] == 12.0 && (s.array() == 13.0).all(),
           "copy validates every output before writing any output");
    auto malformed = oscillator.InitialState();
    malformed.s.resize(0);
    ExpectThrows<std::invalid_argument>([&] { core.Reinitialize(malformed); },
                                        "invalid initial state dimensions must be rejected");
    core.AdvanceOneStep();
    Expect(core.integration_statistics().successful_internal_step_count == 1,
           "invalid caller input must leave the core usable");

    QuaternionRotationProblem quaternion;
    const auto initial = quaternion.InitialState();
    NewmarkCore rotation(quaternion, Configuration(quaternion, 0.01), initial);
    auto non_tangent = initial;
    non_tangent.s += 0.01 * non_tangent.q;
    ExpectThrows<std::invalid_argument>([&] { rotation.Reinitialize(non_tangent); },
                                        "initial radial rate must be rejected, not repaired");
    ExpectStateUnchanged(ReadState(rotation, quaternion), initial,
                         "rejected initial tangent constraint must preserve accepted state");
    rotation.AdvanceOneStep();
    auto restarted = quaternion.ExactState(0.37);
    restarted.q *= -2.4 / initial.q.norm();
    restarted.s *= -2.4 / initial.q.norm();
    rotation.Reinitialize(restarted);
    Expect(rotation.diagnostics().endpoint_projection_evaluation_count == 0,
           "reinitialization records the norm without endpoint projection");
    rotation.AdvanceOneStep();
    const auto actual = ReadState(rotation, quaternion);
    ExpectNear(actual.q.norm(), 2.4, 2e-14, "reinitialization replaces the recorded norm");
    ExpectNear(actual.q.dot(actual.s), 0.0, 2e-14, "restarted accepted rate remains tangent");
    Expect(actual.q.dot(restarted.q) > 0.0,
           "endpoint projection preserves the local quaternion sign");

    // The projection itself preserves physical angular velocity, including
    // a trial with norm drift and a radial component in q'.
    Eigen::VectorXd trial_q = 1.1 * initial.q;
    Eigen::VectorXd trial_s = initial.s + 0.02 * initial.q;
    const Eigen::Vector3d omega = quaternion.PhysicalAngularVelocity(trial_q, trial_s);
    Expect(quaternion.ProjectEndpoint(initial.q, trial_q, trial_s),
           "non-tangent trial must report a projection change");
    ExpectNear((quaternion.PhysicalAngularVelocity(trial_q, trial_s) - omega).norm(),
               0.0, 1e-14, "paired quaternion projection preserves physical velocity");
    ExpectNear(trial_q.norm(), initial.q.norm(), 1e-14, "paired projection norm");
    ExpectNear(trial_q.dot(trial_s), 0.0, 1e-14, "paired projection tangency");
}

}  // namespace

int main() {
    try {
        VerifyBasicFormulasAndConvergence();
        VerifyTrapezoidalInternalState();
        VerifyAcceptedDerivativeAndProjectionRefresh();
        VerifyFullQuaternionResidualBeforeProjection();
        VerifyFailedReinitializationTransaction();
        VerifyFailureTransactions();
        VerifyValidationAndQuaternionRestart();
        std::cout << "Newmark core verification passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Newmark core verification failed: " << error.what() << '\n';
        return 1;
    }
}
