#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string_view>

#include <Eigen/Dense>

#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"

namespace {

using orvd::integrators::ContinuousStateErrorTolerances;
using orvd::integrators::NoCallTimeAppliedForces;
using orvd::integrators::SystemContinuousStateAdvancer;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::CompiledSystemPlan;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;

int failure_count = 0;

void Expect(bool condition, const char* description) {
    if (!condition) {
        std::printf("FAIL %s\n", description);
        ++failure_count;
    }
}

bool Near(double measured, double expected) {
    return std::abs(measured - expected) <=
           1.0e-8 * std::max({1.0, std::abs(measured), std::abs(expected)});
}

template <typename Function>
void ExpectInvalidArgument(Function&& function, const char* description) {
    bool refused = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused, description);
}

template <typename Function>
void ExpectLogicErrorContaining(Function&& function, std::string_view fragment,
                                const char* description) {
    bool refused = false;
    try {
        function();
    } catch (const std::invalid_argument&) {
        refused = false;
    } catch (const std::logic_error& error) {
        refused = std::string_view(error.what()).find(fragment) !=
                  std::string_view::npos;
    }
    Expect(refused, description);
}

template <typename Function>
void ExpectExceptionContaining(Function&& function, std::string_view fragment,
                               const char* description) {
    bool refused = false;
    try {
        function();
    } catch (const std::exception& error) {
        refused = std::string_view(error.what()).find(fragment) !=
                  std::string_view::npos;
    }
    Expect(refused, description);
}

RigidBodyInertiaParameters MakeInertia(double mass, double unit_izz) {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass;
    inertia.center_of_mass_in_body_frame.setZero();
    inertia.unit_inertia_moments =
        Eigen::Vector3d(unit_izz, unit_izz, unit_izz);
    inertia.unit_inertia_products.setZero();
    return inertia;
}

ContinuousStateErrorTolerances MakeTolerances(int state_size) {
    return ContinuousStateErrorTolerances(
        1.0e-12, Eigen::VectorXd::Constant(state_size, 1.0e-14));
}

struct DampedRotorFixture {
    static constexpr double kMass = 2.0;
    static constexpr double kUnitIzz = 0.5;
    static constexpr double kInitialDamping = 0.4;

    MultibodyModel model;
    JointHandle joint;

    DampedRotorFixture() {
        const auto body =
            model.AddRigidBody("rotor", MakeInertia(kMass, kUnitIzz));
        joint = model.AddRevoluteJoint(
            "bearing", model.world_frame(), model.body_frame(body),
            Eigen::Vector3d::UnitZ(), kInitialDamping);
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();
    }
};

void CheckAtomicAcceptedTimeAndState() {
    MultibodyModel model;
    const auto body = model.AddRigidBody("free", MakeInertia(2.0, 0.4));
    model.DeclareFreeBody(body);
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();
    const SystemAssemblyDescription description(model);
    const SystemInstance system(description);

    constexpr double kInitialTime = 1.25;
    auto context = system.CreateDefaultRuntimeContext(kInitialTime);
    Eigen::VectorXd accepted(system.continuous_state_size());
    system.CopyContinuousState(*context, accepted);

    Eigen::VectorXd invalid = accepted;
    const auto q_range = model.GetFreeBodyPositionRange(body);
    invalid.segment(q_range.start(), 4).setZero();
    invalid.tail(model.num_generalized_velocities()).setConstant(3.0);
    ExpectInvalidArgument(
        [&] { system.SetTimeAndContinuousState(*context, 2.0, invalid); },
        "an invalid complete state is refused by the accepted transaction");
    Eigen::VectorXd observed(system.continuous_state_size());
    system.CopyContinuousState(*context, observed);
    Expect(context->time_seconds() == kInitialTime && observed == accepted,
           "a refused state leaves accepted time, q and v unchanged");

    Eigen::VectorXd late_invalid = accepted;
    late_invalid[q_range.start() + 4] += 0.75;
    late_invalid[late_invalid.size() - 1] =
        std::numeric_limits<double>::quiet_NaN();
    ExpectInvalidArgument(
        [&] {
            system.SetTimeAndContinuousState(*context, 2.5, late_invalid);
        },
        "a late velocity failure is refused before changed q is committed");
    system.CopyContinuousState(*context, observed);
    Expect(context->time_seconds() == kInitialTime && observed == accepted,
           "a late state refusal preserves the whole accepted transaction");

    Eigen::VectorXd valid_but_different = accepted;
    valid_but_different[q_range.start() + 4] += 1.25;
    valid_but_different.tail(model.num_generalized_velocities())
        .setConstant(-2.0);
    ExpectInvalidArgument(
        [&] {
            system.SetTimeAndContinuousState(
                *context, std::numeric_limits<double>::quiet_NaN(),
                valid_but_different);
        },
        "a non-finite accepted time is refused before the state is written");
    system.CopyContinuousState(*context, observed);
    Expect(context->time_seconds() == kInitialTime && observed == accepted,
           "a refused time also leaves the accepted transaction unchanged");

    ExpectInvalidArgument(
        [&] {
            (void)system.CreateDefaultRuntimeContext(
                std::numeric_limits<double>::infinity());
        },
        "a runtime context requires a finite initial time");
}

void CheckRealCvodeCommitAndDampingSynchronization() {
    DampedRotorFixture fixture;
    const SystemAssemblyDescription description(fixture.model);
    const SystemInstance system(description);
    const CompiledSystemPlan plan(system);

    constexpr double kInitialTime = 0.3;
    auto accepted = system.CreateDefaultRuntimeContext(kInitialTime);
    auto untouched = system.CreateDefaultRuntimeContext(kInitialTime);
    Eigen::Vector2d initial;
    initial << 0.2, 1.4;
    system.SetContinuousState(*accepted, initial);
    system.SetContinuousState(*untouched, initial);
    constexpr double kInitialAcceptedDamping = 0.9;
    auto accepted_component = system.GetMultibodyComponentView(
        *accepted, system.multibody_component());
    fixture.model.SetRevoluteJointDampingCoefficient(
        &accepted_component.context(), fixture.joint,
        kInitialAcceptedDamping);

    SystemContinuousStateAdvancer advancer(
        system, plan, *accepted, MakeTolerances(2),
        NoCallTimeAppliedForces{});
    advancer.AdvanceTo(kInitialTime);
    Eigen::VectorXd observed(2);
    system.CopyContinuousState(*accepted, observed);
    Expect(accepted->time_seconds() == kInitialTime && observed == initial,
           "a same-time system advance is a complete no-op");

    constexpr double kFirstTarget = 0.55;
    constexpr double kInertia =
        DampedRotorFixture::kMass * DampedRotorFixture::kUnitIzz;
    const double first_rate = kInitialAcceptedDamping / kInertia;
    const double first_dt = kFirstTarget - kInitialTime;
    const double expected_v1 = initial[1] * std::exp(-first_rate * first_dt);
    const double expected_q1 =
        initial[0] + initial[1] * (1.0 - std::exp(-first_rate * first_dt)) /
                         first_rate;
    advancer.AdvanceTo(kFirstTarget);
    system.CopyContinuousState(*accepted, observed);
    Expect(accepted->time_seconds() == kFirstTarget,
           "a successful public advance commits the target time");
    Expect(Near(observed[0], expected_q1) && Near(observed[1], expected_v1),
           "the accepted endpoint follows the damped-rotor analytic solution");

    constexpr double kNewDamping = 1.6;
    Eigen::Vector2d externally_stated;
    externally_stated << observed[0] + 0.05, observed[1] * 0.75;
    system.SetContinuousState(*accepted, externally_stated);
    fixture.model.SetRevoluteJointDampingCoefficient(
        &accepted_component.context(), fixture.joint, kNewDamping);
    advancer.SynchronizeAfterAcceptedContextChange();

    constexpr double kSecondTarget = 0.8;
    const double second_rate = kNewDamping / kInertia;
    const double second_dt = kSecondTarget - kFirstTarget;
    const double expected_v2 =
        externally_stated[1] * std::exp(-second_rate * second_dt);
    const double expected_q2 =
        externally_stated[0] +
        externally_stated[1] *
            (1.0 - std::exp(-second_rate * second_dt)) / second_rate;
    advancer.AdvanceTo(kSecondTarget);
    system.CopyContinuousState(*accepted, observed);
    const bool synchronized_damping_applied =
        accepted->time_seconds() == kSecondTarget &&
        Near(observed[0], expected_q2) && Near(observed[1], expected_v2);
    if (!synchronized_damping_applied) {
        std::printf(
            "damping synchronization measured q=% .17g v=% .17g, expected "
            "q=% .17g v=% .17g\n",
            observed[0], observed[1], expected_q2, expected_v2);
    }
    Expect(synchronized_damping_applied,
           "explicit synchronization uses the new accepted state and damping");

    Eigen::VectorXd untouched_state(2);
    system.CopyContinuousState(*untouched, untouched_state);
    Expect(untouched->time_seconds() == kInitialTime &&
               untouched_state == initial,
           "advancing and synchronizing one context leaves another untouched");
    auto untouched_component = system.GetMultibodyComponentView(
        *untouched, system.multibody_component());
    Eigen::VectorXd damping_force(1);
    fixture.model.CalcJointDampingAppliedGeneralizedForces(
        untouched_component.context(), damping_force);
    Expect(Near(damping_force[0],
                -DampedRotorFixture::kInitialDamping * initial[1]),
           "the other context retains its original damping parameter");
}

void CheckRealForcePlanCvodeAndNominalForceSynchronization() {
    constexpr double kSliderMass = 2.0;
    constexpr double kSeriesStiffness = 8.0;
    constexpr double kSeriesDamping = 2.0;
    constexpr double kInitialSeriesForce = 120.0;

    MultibodyModel model;
    const auto anchor =
        model.AddRigidBody("force_anchor", MakeInertia(1.0, 1.0));
    const auto series_anchor =
        model.AddRigidBody("series_anchor", MakeInertia(1.0, 1.0));
    const auto slider =
        model.AddRigidBody("force_slider", MakeInertia(kSliderMass, 1.0));
    model.AddWeldJoint("force_anchor_weld", model.world_frame(),
                       model.body_frame(anchor));
    model.AddWeldJoint("series_anchor_weld", model.world_frame(),
                       model.body_frame(series_anchor));
    model.AddPrismaticJoint("slider_joint", model.body_frame(anchor),
                            model.body_frame(slider),
                            Eigen::Vector3d::UnitX(), 0.0);
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();

    orvd::forces::VehicleForcePlan force_plan(
        model,
        {orvd::forces::TranslationalSpringDamper{
            "nominal_driver",
            orvd::forces::ForceElementEnd{model.body_frame(anchor)},
            orvd::forces::ForceElementEnd{model.body_frame(slider)},
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}},
        {},
        {orvd::forces::SeriesSpringViscousDamper{
            "series_decay",
            orvd::forces::ForceElementEnd{model.body_frame(anchor)},
            orvd::forces::ForceElementEnd{model.body_frame(series_anchor)},
            orvd::forces::ForceElementAxis::kLongitudinal,
            kSeriesStiffness, kSeriesDamping}},
        {});
    const SystemAssemblyDescription description(model, force_plan);
    const SystemInstance system(description);
    const CompiledSystemPlan plan(system);
    auto accepted = system.CreateDefaultRuntimeContext(0.0);
    const auto nominal_slot =
        system.GetTranslationalSpringDamperIndexByName("nominal_driver");
    const auto series_range = system.series_spring_damper_force_state_range(
        system.GetSeriesSpringViscousDamperIndexByName("series_decay"));

    Eigen::Vector3d state;
    state << 0.25, 0.4, kInitialSeriesForce;
    system.SetContinuousState(*accepted, state);
    constexpr double kFirstNominalForce = 6.0;
    system.SetNominalForce(
        *accepted, nominal_slot,
        Eigen::Vector3d(kFirstNominalForce, 0.0, 0.0));

    Eigen::VectorXd absolute_tolerances(3);
    absolute_tolerances << 1.0e-12, 1.0e-12, 1.0e-7;
    SystemContinuousStateAdvancer advancer(
        system, plan, *accepted,
        ContinuousStateErrorTolerances(1.0e-11, absolute_tolerances),
        NoCallTimeAppliedForces{});

    constexpr double kFirstTarget = 0.2;
    const double first_acceleration = -kFirstNominalForce / kSliderMass;
    const double expected_q1 =
        state[0] + state[1] * kFirstTarget +
        0.5 * first_acceleration * kFirstTarget * kFirstTarget;
    const double expected_v1 = state[1] + first_acceleration * kFirstTarget;
    const double expected_z1 =
        kInitialSeriesForce *
        std::exp(-(kSeriesStiffness / kSeriesDamping) * kFirstTarget);
    advancer.AdvanceTo(kFirstTarget);
    Eigen::VectorXd observed(3);
    system.CopyContinuousState(*accepted, observed);
    const bool first_segment_matches =
        Near(observed[0], expected_q1) && Near(observed[1], expected_v1) &&
        std::abs(observed[series_range.start()] - expected_z1) <=
            1.0e-6 * std::max(1.0, std::abs(expected_z1));
    if (!first_segment_matches) {
        std::printf("force-plan CVODE first segment measured [% .17g, % .17g, "
                    "% .17g], expected [% .17g, % .17g, % .17g]\n",
                    observed[0], observed[1], observed[series_range.start()],
                    expected_q1, expected_v1, expected_z1);
    }
    Expect(first_segment_matches,
           "the real force plan, RHS bridge and CVODE reproduce constant "
           "nominal-force acceleration and the Maxwell exponential state");

    constexpr double kRestartedSeriesForce = -75.0;
    Eigen::VectorXd restarted_state = observed;
    restarted_state[series_range.start()] = kRestartedSeriesForce;
    system.SetContinuousState(*accepted, restarted_state);
    constexpr double kSecondNominalForce = -4.0;
    system.SetNominalForce(
        *accepted, nominal_slot,
        Eigen::Vector3d(kSecondNominalForce, 0.0, 0.0));
    advancer.SynchronizeAfterAcceptedContextChange();
    constexpr double kSecondTarget = 0.35;
    const double second_dt = kSecondTarget - kFirstTarget;
    const double second_acceleration = -kSecondNominalForce / kSliderMass;
    const double expected_q2 =
        restarted_state[0] + restarted_state[1] * second_dt +
        0.5 * second_acceleration * second_dt * second_dt;
    const double expected_v2 =
        restarted_state[1] + second_acceleration * second_dt;
    const double expected_z2 =
        kRestartedSeriesForce *
        std::exp(-(kSeriesStiffness / kSeriesDamping) * second_dt);
    advancer.AdvanceTo(kSecondTarget);
    system.CopyContinuousState(*accepted, observed);
    const bool second_segment_matches =
        Near(observed[0], expected_q2) && Near(observed[1], expected_v2) &&
        std::abs(observed[series_range.start()] - expected_z2) <=
            1.0e-6 * std::max(1.0, std::abs(expected_z2));
    if (!second_segment_matches) {
        std::printf("force-plan CVODE second segment measured [% .17g, % .17g, "
                    "% .17g], expected [% .17g, % .17g, % .17g]\n",
                    observed[0], observed[1], observed[series_range.start()],
                    expected_q2, expected_v2, expected_z2);
    }
    Expect(second_segment_matches,
           "explicit accepted-context synchronization updates nominal force "
           "and restarts CVODE from the externally replaced Maxwell state");
}

void CheckRealRhsFailureRequiresSynchronization() {
    MultibodyModel model;
    const auto massless = model.AddRigidBody("massless", MakeInertia(0.0, 0.0));
    model.AddRevoluteJoint("singular", model.world_frame(),
                           model.body_frame(massless),
                           Eigen::Vector3d::UnitZ(), 0.0);
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();
    const SystemAssemblyDescription description(model);
    const SystemInstance system(description);
    const CompiledSystemPlan plan(system);
    auto accepted = system.CreateDefaultRuntimeContext(0.0);
    Eigen::VectorXd before(2);
    system.CopyContinuousState(*accepted, before);

    SystemContinuousStateAdvancer advancer(
        system, plan, *accepted, MakeTolerances(2),
        NoCallTimeAppliedForces{});
    ExpectExceptionContaining(
        [&] { advancer.AdvanceTo(0.1); }, "positive-definite",
        "a singular articulated inertia fails after the real RHS is entered");
    Eigen::VectorXd after(2);
    system.CopyContinuousState(*accepted, after);
    Expect(accepted->time_seconds() == 0.0 && after == before,
           "an RHS failure cannot publish trial time or state");

    ExpectLogicErrorContaining(
        [&] { advancer.AdvanceTo(0.1); }, "synchronize",
        "a failed system advance blocks reuse before explicit synchronization");
    advancer.SynchronizeAfterAcceptedContextChange();
    ExpectExceptionContaining(
        [&] { advancer.AdvanceTo(0.1); }, "positive-definite",
        "synchronization reopens the backend even when the model remains "
        "singular");
    system.CopyContinuousState(*accepted, after);
    Expect(accepted->time_seconds() == 0.0 && after == before,
           "a repeated physical failure still leaves the accepted endpoint");
}

void CheckForeignAcceptedContextIsRejected() {
    DampedRotorFixture first_fixture;
    DampedRotorFixture second_fixture;
    const SystemAssemblyDescription first_description(first_fixture.model);
    const SystemAssemblyDescription second_description(second_fixture.model);
    const SystemInstance first(first_description);
    const SystemInstance second(second_description);
    const CompiledSystemPlan first_plan(first);
    auto foreign_context = second.CreateDefaultRuntimeContext(0.0);

    ExpectInvalidArgument(
        [&] {
            (void)SystemContinuousStateAdvancer(
                first, first_plan, *foreign_context, MakeTolerances(2),
                NoCallTimeAppliedForces{});
        },
        "a system advancer refuses an accepted context from another system");
}

}  // namespace

int main() {
    CheckAtomicAcceptedTimeAndState();
    CheckRealCvodeCommitAndDampingSynchronization();
    CheckRealForcePlanCvodeAndNominalForceSynchronization();
    CheckRealRhsFailureRequiresSynchronization();
    CheckForeignAcceptedContextIsRejected();
    if (failure_count != 0) {
        std::printf("%d system continuous-state assertion(s) failed\n",
                    failure_count);
        return 1;
    }
    std::printf("system continuous-state advancement contract verified\n");
    return 0;
}
