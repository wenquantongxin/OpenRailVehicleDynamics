#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "coordinate_step_time.h"
#include "newmark_continuous_state_advancer.h"
#include "newmark_core.h"
#include "newmark_coordinate_layout.h"
#include "system_coordinate_problem.h"
#include "zhai_continuous_state_advancer.h"
#include "zhai_core.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"

namespace orvd::integrators::internal::test {

inline void Require(bool condition, const std::string& description) {
    if (!condition) throw std::runtime_error(description);
}
inline void Near(double value, double expected, double tolerance, const std::string& description) {
    Require(std::isfinite(value) && std::abs(value - expected) <= tolerance,
            description + ": actual=" + std::to_string(value) + ", expected=" + std::to_string(expected));
}
template <typename Exception = std::invalid_argument, typename Operation>
void Throws(Operation&& operation, const std::string& description) {
    bool caught = false;
    try { operation(); } catch (const Exception&) { caught = true; }
    Require(caught, description);
}
template <typename Core, typename... Args>
void RequireAcceptedCoreStep(Core& core, Args... args) {
    if constexpr (std::is_same_v<Core, NewmarkCore>) {
        Require(core.AdvanceOneStep(args...) == NewmarkCore::StepResult::kAccepted,
                "Newmark core step must converge");
    } else {
        core.AdvanceOneStep(args...);
    }
}
inline bool SameStatistics(const ContinuousStateIntegrationStatistics& a,
                           const ContinuousStateIntegrationStatistics& b) {
    return a.successful_internal_step_count == b.successful_internal_step_count &&
           a.right_hand_side_evaluation_count == b.right_hand_side_evaluation_count &&
           a.linear_solver_right_hand_side_evaluation_count == b.linear_solver_right_hand_side_evaluation_count &&
           a.error_test_failure_count == b.error_test_failure_count &&
           a.nonlinear_solver_iteration_count == b.nonlinear_solver_iteration_count &&
           a.nonlinear_solver_convergence_failure_count == b.nonlinear_solver_convergence_failure_count &&
           a.linear_solver_setup_count == b.linear_solver_setup_count &&
           a.jacobian_evaluation_count == b.jacobian_evaluation_count &&
           a.requested_dense_finite_difference_jacobian_worker_count ==
               b.requested_dense_finite_difference_jacobian_worker_count;
}
inline bool SameInterval(const std::optional<ContinuousStateDenseOutputInterval>& a,
                         const std::optional<ContinuousStateDenseOutputInterval>& b) {
    return a.has_value() == b.has_value() &&
           (!a || (a->start_time_seconds == b->start_time_seconds &&
                   a->end_time_seconds == b->end_time_seconds));
}

struct SimpleSystem {
    multibody_model::MultibodyModel model;
    multibody_model::JointHandle joint;
    std::unique_ptr<system_assembly::SystemInstance> system;
    std::unique_ptr<system_assembly::CompiledSystemPlan> plan;
    std::unique_ptr<system_assembly::SystemRuntimeContext> trial;
    std::unique_ptr<SystemCoordinateProblem> problem;

    explicit SimpleSystem(bool ball = false, double gravity = 1.0) {
        multibody_runtime::RigidBodyInertiaParameters inertia;
        inertia.mass_kilograms = 1.0;
        inertia.center_of_mass_in_body_frame.setZero();
        inertia.unit_inertia_moments.setConstant(0.3);
        inertia.unit_inertia_products.setZero();
        const auto body = model.AddRigidBody("body", inertia);
        if (ball) {
            joint = model.AddBallRpyJoint("ball", model.world_frame(), model.body_frame(body),
                                          Eigen::Vector3d::Zero());
        } else {
            joint = model.AddPrismaticJoint("slide", model.world_frame(), model.body_frame(body),
                                             Eigen::Vector3d::UnitX(), 0.0);
        }
        model.SetGravityVector(Eigen::Vector3d(gravity, 0.0, 0.0));
        model.Finalize();
        const system_assembly::SystemAssemblyDescription description(model);
        system = std::make_unique<system_assembly::SystemInstance>(description);
        plan = std::make_unique<system_assembly::CompiledSystemPlan>(*system);
        trial = system->CreateDefaultRuntimeContext(0.0);
        problem = std::make_unique<SystemCoordinateProblem>(*system, *plan, *trial,
                                                             NoCallTimeAppliedForces{});
    }
};

template <typename Adapter>
auto ConfigurationFor(const SystemCoordinateProblem& problem, double h) {
    if constexpr (std::is_same_v<Adapter, NewmarkContinuousStateAdvancer>) {
        static_cast<void>(problem);
        return NewmarkConfiguration{h, {12, {1e-12, 1e-12, 1e-12, 1.0},
            {1e-12, 1e-12, 1e-12, 1.0}, {1e-12, 1e-12, 1e-12, 1.0},
            {1e-12, 1e-12, 1.0}}};
    } else {
        return ZhaiConfiguration{h};
    }
}

inline Eigen::VectorXd CurrentState(const ContinuousStateAdvancer& adapter) {
    Eigen::VectorXd result(adapter.continuous_state_size());
    adapter.CopyCurrentState(result);
    return result;
}

inline void AdvanceFully(ContinuousStateAdvancer& adapter, double stop,
                         Eigen::VectorXd* last_start_state = nullptr) {
    Eigen::VectorXd endpoint(adapter.continuous_state_size());
    for (int count = 0; count < 10000 && adapter.current_time_seconds() < stop; ++count) {
        const double before = adapter.current_time_seconds();
        const auto work_before = adapter.integration_statistics();
        if (last_start_state != nullptr) *last_start_state = CurrentState(adapter);
        const auto step = adapter.AdvanceOneInternalStepToward(stop, endpoint);
        Require(step.start_time_seconds == before && step.end_time_seconds > before &&
                    step.end_time_seconds <= stop && step.end_time_seconds == adapter.current_time_seconds() &&
                    step.reached_stop == (step.end_time_seconds == stop), "exact internal endpoint/stop metadata");
        Require(adapter.integration_statistics().successful_internal_step_count ==
                    work_before.successful_internal_step_count + 1, "one call publishes exactly one successful step");
        Require((CurrentState(adapter).array() == endpoint.array()).all(), "endpoint copy matches the public state");
        const auto dense = adapter.dense_output_interval();
        Require(dense && dense->start_time_seconds == before && dense->end_time_seconds == step.end_time_seconds,
                "dense interval is exactly the real successful step");
    }
    Require(adapter.current_time_seconds() == stop, "stop reached within the test work bound");
}

template <typename Adapter>
void CheckObservableContract() {
    SimpleSystem fixture;
    const Eigen::VectorXd initial = (Eigen::Vector2d() << 0.3, -0.2).finished();
    Adapter adapter(*fixture.problem, 0.2, initial, ConfigurationFor<Adapter>(*fixture.problem, 0.1));
    const auto initial_work = adapter.integration_statistics();
    constexpr bool newmark = std::is_same_v<Adapter, NewmarkContinuousStateAdvancer>;
    Require(initial_work.right_hand_side_evaluation_count == 1 &&
                initial_work.successful_internal_step_count == 0 &&
                initial_work.linear_solver_right_hand_side_evaluation_count == 0 &&
                initial_work.requested_dense_finite_difference_jacobian_worker_count == (newmark ? 1 : 0),
            "initial mechanical evaluation and actual Jacobian worker identity are reported honestly");
    Require(!adapter.dense_output_interval(), "initialization exposes no dense interval");
    Require((CurrentState(adapter).array() == initial.array()).all(), "initial physical storage is preserved exactly");
    Eigen::VectorXd output = Eigen::VectorXd::Constant(2, 19.0);
    Throws<std::logic_error>([&] { adapter.CopyDenseState(0.2, output); }, "initial dense query is unavailable");
    Require((output.array() == 19.0).all(), "unavailable dense query preserves output");
    Eigen::VectorXd start(2);
    AdvanceFully(adapter, 0.5, &start);
    Require(adapter.integration_statistics().successful_internal_step_count == 3,
            "decimal stop uses three nominal steps, without a rounding sliver");
    if constexpr (!newmark) Require(adapter.diagnostics().startup_step_count == 1,
                                    "rounding endpoints do not restart Zhai");
    const auto accepted = CurrentState(adapter);
    Near(accepted[0], 0.3 - 0.2 * 0.3 + 0.5 * 0.3 * 0.3, 2e-14, "constant-acceleration endpoint position");
    Near(accepted[1], 0.1, 2e-14, "constant-acceleration endpoint velocity");
    const auto interval = adapter.dense_output_interval();
    const auto before_queries = adapter.integration_statistics();
    const auto before_diagnostics = adapter.diagnostics();
    adapter.CopyDenseState(interval->start_time_seconds, output);
    Require((output.array() == start.array()).all(), "dense left endpoint copies frozen storage exactly");
    adapter.CopyDenseState(interval->end_time_seconds, output);
    Require((output.array() == accepted.array()).all(), "dense right endpoint copies frozen storage exactly");
    const double midpoint = interval->start_time_seconds +
                            0.5 * (interval->end_time_seconds - interval->start_time_seconds);
    adapter.CopyDenseState(midpoint, output);
    Near(output[0], 0.5 * (start[0] + accepted[0]), 2e-15, "dense position is declared linear storage interpolation");
    const double elapsed = midpoint - 0.2;
    const double exact_q = initial[0] + initial[1] * elapsed + 0.5 * elapsed * elapsed;
    const double interval_h = interval->end_time_seconds - interval->start_time_seconds;
    Near(output[0] - exact_q, interval_h * interval_h / 8.0, 2e-14,
         "linear dense output retains the constant-acceleration a*h^2/8 midpoint error");
    for (const double time : {midpoint, interval->start_time_seconds, midpoint, interval->end_time_seconds}) {
        adapter.CopyDenseState(time, output);
    }
    Require(SameStatistics(before_queries, adapter.integration_statistics()) &&
                before_diagnostics.startup_step_count == adapter.diagnostics().startup_step_count &&
                before_diagnostics.endpoint_projection_evaluation_count ==
                    adapter.diagnostics().endpoint_projection_evaluation_count,
            "dense queries do not evaluate physics, project endpoints or alter method history");

    output.setConstant(23.0);
    Throws([&] { adapter.CopyDenseState(std::nextafter(interval->start_time_seconds,
                                                      -std::numeric_limits<double>::infinity()), output); },
           "dense bounds are not extended by clock snap tolerance");
    Require((output.array() == 23.0).all(), "outside dense query preserves output");
    Eigen::VectorXd wrong = Eigen::VectorXd::Constant(1, 29.0);
    Throws([&] { adapter.CopyCurrentState(wrong); }, "wrong current output size");
    Throws([&] { adapter.CopyDenseState(midpoint, wrong); }, "wrong dense output size");
    Throws([&] { (void)adapter.AdvanceOneInternalStepToward(0.6, wrong); }, "wrong endpoint output size");
    Throws([&] { (void)adapter.AdvanceOneInternalStepToward(0.49, output); }, "backward stop refused before entry");
    Throws([&] { (void)adapter.AdvanceOneInternalStepToward(std::numeric_limits<double>::quiet_NaN(), output); },
           "non-finite stop refused before entry");
    Eigen::VectorXd invalid = initial;
    invalid[1] = std::numeric_limits<double>::quiet_NaN();
    Throws([&] { adapter.ReinitializeAfterExternalChange(1.0, invalid); }, "invalid reinitialization state");
    Throws([&] { adapter.ReinitializeAfterExternalChange(1.0, wrong); }, "invalid reinitialization dimension");
    Require(SameStatistics(before_queries, adapter.integration_statistics()) &&
                SameInterval(interval, adapter.dense_output_interval()) &&
                (CurrentState(adapter).array() == accepted.array()).all(),
            "caller refusals preserve endpoint, dense state, work and usability");
    const auto noop = adapter.AdvanceOneInternalStepToward(0.5, output);
    Require(noop.start_time_seconds == 0.5 && noop.end_time_seconds == 0.5 && noop.reached_stop &&
                SameStatistics(before_queries, adapter.integration_statistics()) &&
                SameInterval(interval, adapter.dense_output_interval()) &&
                (output.array() == accepted.array()).all(), "same-time request is an exact preserving no-op");
    AdvanceFully(adapter, 0.8);
    Require(adapter.integration_statistics().successful_internal_step_count == 6,
            "stopping at a nominal boundary does not change the continuing method grid");
    if constexpr (!newmark) Require(adapter.diagnostics().startup_step_count == 1,
                                    "successive aligned public stops preserve Zhai history");
    adapter.ReinitializeAfterExternalChange(-0.2, initial);
    Require(!adapter.dense_output_interval() && adapter.integration_statistics().right_hand_side_evaluation_count == 1 &&
                adapter.integration_statistics().successful_internal_step_count == 0,
            "successful reinitialization clears dense/history and reports its one initial RHS");
    AdvanceFully(adapter, 0.1);
    Require(adapter.integration_statistics().successful_internal_step_count == 3,
            "negative initial-time grid reaches its exact requested stop");
}

template <typename Adapter>
void CheckDenseConvergence() {
    SimpleSystem fixture;
    const Eigen::VectorXd initial = Eigen::Vector2d::Zero();
    double previous_error = 0.0;
    for (const double h : {0.2, 0.1, 0.05, 0.025}) {
        Adapter adapter(*fixture.problem, 0.0, initial, ConfigurationFor<Adapter>(*fixture.problem, h));
        AdvanceFully(adapter, 1.0);
        const auto interval = adapter.dense_output_interval();
        const double sample_time = interval->start_time_seconds +
            0.5 * (interval->end_time_seconds - interval->start_time_seconds);
        Eigen::VectorXd sample(2);
        adapter.CopyDenseState(sample_time, sample);
        const double error = std::abs(sample[0] - 0.5 * sample_time * sample_time);
        if (previous_error > 0.0) {
            const double order = std::log2(previous_error / error);
            Require(std::isfinite(order) && order >= 1.8 && order <= 2.2,
                    "smooth interior physical-state samples converge at second order");
        }
        previous_error = error;
    }
}

template <typename Adapter>
void CheckGridAndTrueShortSteps() {
    SimpleSystem fixture;
    const Eigen::VectorXd initial = Eigen::Vector2d::Zero();
    for (const double stop : {0.3, std::nextafter(0.3, 0.0),
                              std::nextafter(0.3, 1.0)}) {
        Adapter adapter(*fixture.problem, 0.0, initial, ConfigurationFor<Adapter>(*fixture.problem, 0.1));
        AdvanceFully(adapter, stop);
        Require(adapter.integration_statistics().successful_internal_step_count == 3,
                "neighboring floating-point stops are represented by three nominal formula steps");
        if constexpr (std::is_same_v<Adapter, ZhaiContinuousStateAdvancer>) {
            Require(adapter.diagnostics().startup_step_count == 1, "one-ULP stop rounding does not cause startup");
        }
    }
    Adapter long_run(*fixture.problem, 0.0, initial, ConfigurationFor<Adapter>(*fixture.problem, 0.001));
    AdvanceFully(long_run, 1.0);
    Require(long_run.integration_statistics().successful_internal_step_count == 1000,
            "integer nominal grid avoids a cumulative-time final sliver");
    if constexpr (std::is_same_v<Adapter, ZhaiContinuousStateAdvancer>) {
        Require(long_run.diagnostics().startup_step_count == 1, "all long-run formula intervals retain exact identity");
    }
    Adapter short_run(*fixture.problem, 0.0, initial, ConfigurationFor<Adapter>(*fixture.problem, 0.1));
    AdvanceFully(short_run, 0.25);
    AdvanceFully(short_run, 0.45);
    Require(short_run.integration_statistics().successful_internal_step_count == 5,
            "real short step is taken once and the nominal grid continues from its endpoint");
    if constexpr (std::is_same_v<Adapter, ZhaiContinuousStateAdvancer>) {
        Require(short_run.diagnostics().startup_step_count == 3,
                "Zhai starts initially, on the real short step, and on return to the nominal interval");
    }
}

template <typename Adapter>
void CheckExplicitCoreClock() {
    using Core = std::conditional_t<std::is_same_v<Adapter, NewmarkContinuousStateAdvancer>, NewmarkCore, ZhaiCore>;
    SimpleSystem fixture;
    const auto initial = fixture.problem->MakeCoordinateState(0.0, Eigen::Vector2d::Zero());
    const auto core_configuration = [&] {
        if constexpr (std::is_same_v<Core, NewmarkCore>) {
            return NewmarkCoordinateLayout(fixture.problem->model(), fixture.problem->internal_state_size())
                .Expand(ConfigurationFor<Adapter>(*fixture.problem, 0.1), initial.q);
        } else {
            return ConfigurationFor<Adapter>(*fixture.problem, 0.1);
        }
    }();
    Core core(*fixture.problem, core_configuration, initial);
    RequireAcceptedCoreStep(core);
    RequireAcceptedCoreStep(core);
    RequireAcceptedCoreStep(core, 0.1, 0.3);
    Require(core.current_time_seconds() == 0.3, "explicit core endpoint is used by accepted state");
    Near(fixture.trial->time_seconds(), 0.3, 0.0, "the actual RHS receives the snapped endpoint time");
    const auto before = core.integration_statistics();
    Throws([&] { RequireAcceptedCoreStep(core, 0.1, 0.45); }, "out-of-window explicit endpoint is a caller error");
    Require(core.current_time_seconds() == 0.3 && SameStatistics(before, core.integration_statistics()),
            "illegal explicit endpoint leaves core clock, work and availability unchanged");
    RequireAcceptedCoreStep(core, 0.1, 0.4);
    if constexpr (std::is_same_v<Core, ZhaiCore>) {
        Require(core.diagnostics().startup_step_count == 1, "explicit endpoint clock does not alter the Zhai h identity");
        RequireAcceptedCoreStep(core, std::nextafter(0.1, 0.0));
        Require(core.diagnostics().startup_step_count == 2,
                "a genuinely different explicit h still restarts the unchanged basic core");
    }
    const double one_ulp = CoordinateTimeUlp(0.3);
    Require(CoordinateEndpointTimeIsCompatible(0.2, 0.1, 0.3 + 9.0 * one_ulp),
            "core rounding allowance covers the grid-rounding plus stop-snap budget");
}

template <typename Adapter>
void CheckNumericalFailureTransactions() {
    SimpleSystem fixture;
    const Eigen::VectorXd initial = Eigen::Vector2d::Zero();
    Adapter underflow(*fixture.problem, 1e20, initial, ConfigurationFor<Adapter>(*fixture.problem, 0.1));
    Eigen::VectorXd output = Eigen::VectorXd::Constant(2, 31.0);
    bool classified = false;
    try {
        (void)underflow.AdvanceOneInternalStepToward(std::nextafter(1e20, std::numeric_limits<double>::infinity()), output);
    } catch (const ContinuousStateNumericalFailure& error) {
        classified = error.reason() == ContinuousStateNumericalFailure::Reason::kStepSizeUnderflow &&
                     error.backend_code() == static_cast<int>(CoordinateIntegrationFailure::Reason::kStepSizeUnderflow);
    }
    Require(classified && (output.array() == 31.0).all() && underflow.current_time_seconds() == 1e20,
            "unrepresentable time is classified without publishing a zero step");
    Throws<std::logic_error>([&] { (void)underflow.AdvanceOneInternalStepToward(1e20, output); },
                              "same-time request cannot bypass failure poison");

    Adapter overflow(*fixture.problem, 0.0, initial, ConfigurationFor<Adapter>(*fixture.problem, 1e308));
    AdvanceFully(overflow, 1.0);
    const auto accepted = CurrentState(overflow);
    const auto work = overflow.integration_statistics();
    output.setConstant(37.0);
    classified = false;
    try { (void)overflow.AdvanceOneInternalStepToward(1e308, output); }
    catch (const ContinuousStateNumericalFailure& error) {
        classified = error.reason() == ContinuousStateNumericalFailure::Reason::kNonFiniteState &&
                     error.backend_code() == static_cast<int>(CoordinateIntegrationFailure::Reason::kNonFiniteState);
    }
    Require(classified && (output.array() == 37.0).all() &&
                (CurrentState(overflow).array() == accepted.array()).all() &&
                overflow.current_time_seconds() == 1.0 && !overflow.dense_output_interval() &&
                overflow.integration_statistics().successful_internal_step_count == work.successful_internal_step_count,
            "non-finite candidate state has its own classification and preserves the published transaction");
    overflow.ReinitializeAfterExternalChange(0.0, initial);
    AdvanceFully(overflow, 0.5);
    Near(CurrentState(overflow)[0], 0.125, 1e-14, "successful reinitialization recovers after numeric failure");
}

template <typename Adapter>
void CheckPhysicalExceptionAndRestart() {
    SimpleSystem fixture(true, 0.0);
    Eigen::VectorXd initial = Eigen::VectorXd::Zero(6);
    initial[1] = std::acos(-1.0) / 2.0 - 0.2;
    initial[4] = 1.0;
    Adapter adapter(*fixture.problem, 0.0, initial, ConfigurationFor<Adapter>(*fixture.problem, 0.1));
    AdvanceFully(adapter, 0.1);
    const auto accepted = CurrentState(adapter);
    Eigen::VectorXd output = Eigen::VectorXd::Constant(6, 41.0);
    bool physical_type = false;
    try { (void)adapter.AdvanceOneInternalStepToward(0.2, output); }
    catch (const ContinuousStateNumericalFailure&) { physical_type = false; }
    catch (const std::runtime_error& error) {
        physical_type = std::string(error.what()).find("singularity") != std::string::npos;
    }
    Require(physical_type && (output.array() == 41.0).all() && !adapter.dense_output_interval() &&
                (CurrentState(adapter).array() == accepted.array()).all() && adapter.current_time_seconds() == 0.1,
            "a real Ball-RPY physical refusal retains its original exception category and last public endpoint");
    Throws<std::logic_error>([&] { (void)adapter.AdvanceOneInternalStepToward(0.1, output); },
                              "physical failure blocks same-time requests");
    initial[1] = 0.0;
    adapter.ReinitializeAfterExternalChange(0.0, initial);
    AdvanceFully(adapter, 0.1);
    const auto dense = adapter.dense_output_interval();
    const auto before = adapter.integration_statistics();
    auto invalid = initial;
    invalid[1] = std::acos(-1.0) / 2.0;
    Throws<std::runtime_error>([&] { adapter.ReinitializeAfterExternalChange(0.0, invalid); },
                               "invalid initial geometry is refused before reinitialization entry");
    Require(SameInterval(dense, adapter.dense_output_interval()) && SameStatistics(before, adapter.integration_statistics()),
            "geometry preflight failure preserves valid dense output, counters and continuation");
    AdvanceFully(adapter, 0.2);
}

inline void CheckNewmarkSingularFailureClassification() {
    multibody_model::MultibodyModel model;
    multibody_runtime::RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 1.0;
    inertia.center_of_mass_in_body_frame.setZero();
    inertia.unit_inertia_moments.setConstant(0.3);
    inertia.unit_inertia_products.setZero();
    for (const std::string name : {"first", "second"}) {
        const auto body = model.AddRigidBody(name, inertia);
        if (name == "first") {
            model.AddPrismaticJoint(name + "_slide", model.world_frame(), model.body_frame(body),
                                    Eigen::Vector3d::UnitX(), 1.0);
        } else {
            model.AddRevoluteJoint(name + "_hinge", model.world_frame(), model.body_frame(body),
                                   Eigen::Vector3d::UnitX(), 1.0);
        }
    }
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();
    const system_assembly::SystemAssemblyDescription description(model);
    const system_assembly::SystemInstance system(description);
    const system_assembly::CompiledSystemPlan plan(system);
    auto trial = system.CreateDefaultRuntimeContext(0.0);
    SystemCoordinateProblem problem(system, plan, *trial, NoCallTimeAppliedForces{});
    auto configuration = ConfigurationFor<NewmarkContinuousStateAdvancer>(problem, 0.1);
    // The physical Jacobian is diagonal with finite positive entries. These
    // positive finite scales deliberately make its scaled LU lose numerical
    // rank, exercising the real factorization failure without a test hook.
    configuration.nonlinear_solver.angle.acceleration_residual = 1e20;
    const Eigen::VectorXd initial = (Eigen::Vector4d() << 0.0, 0.0, 1.0, 1.0).finished();
    NewmarkContinuousStateAdvancer adapter(problem, 0.0, initial, configuration);
    Eigen::VectorXd output = Eigen::VectorXd::Constant(4, 43.0);
    bool classified = false;
    try { (void)adapter.AdvanceOneInternalStepToward(0.1, output); }
    catch (const ContinuousStateNumericalFailure& error) {
        classified = error.reason() == ContinuousStateNumericalFailure::Reason::kSingularLinearSystem &&
                     error.backend_code() == static_cast<int>(CoordinateIntegrationFailure::Reason::kSingularJacobian);
    }
    const auto work = adapter.integration_statistics();
    Require(classified && work.linear_solver_setup_count == 1 &&
                work.nonlinear_solver_convergence_failure_count == 1 &&
                work.successful_internal_step_count == 0 && !adapter.dense_output_interval() &&
                adapter.current_time_seconds() == 0.0 && (output.array() == 43.0).all() &&
                (CurrentState(adapter).array() == initial.array()).all(),
            "a real scaled LU rank failure is classified once and does not publish or retry");
    Throws<std::logic_error>([&] { (void)adapter.AdvanceOneInternalStepToward(0.1, output); },
                              "singular factorization requires reinitialization before continuing");
}

template <typename Adapter>
int RunBasicCoordinateAdvancerContract() {
    try {
        CheckObservableContract<Adapter>();
        CheckDenseConvergence<Adapter>();
        CheckGridAndTrueShortSteps<Adapter>();
        CheckExplicitCoreClock<Adapter>();
        CheckNumericalFailureTransactions<Adapter>();
        CheckPhysicalExceptionAndRestart<Adapter>();
        if constexpr (std::is_same_v<Adapter, NewmarkContinuousStateAdvancer>) {
            CheckNewmarkSingularFailureClassification();
        }
        std::cout << "basic coordinate advancer contract verified\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "basic coordinate advancer contract failed: " << error.what() << '\n';
        return 1;
    }
}

}  // namespace orvd::integrators::internal::test
