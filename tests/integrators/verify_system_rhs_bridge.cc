#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>

#include <Eigen/Dense>

#include "orvd/integrators/continuous_state_advancer.h"
#include "orvd/integrators/system_rhs_bridge.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/system_assembly/compiled_system_plan.h"
#include "orvd/system_assembly/system_assembly_description.h"

namespace {

using orvd::integrators::ContinuousStateAdvancer;
using orvd::integrators::ContinuousStateDenseOutputInterval;
using orvd::integrators::ContinuousStateErrorTolerances;
using orvd::integrators::ContinuousStateRhs;
using orvd::integrators::NoCallTimeAppliedForces;
using orvd::integrators::SystemRhsBridge;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::system_assembly::CompiledSystemPlan;
using orvd::system_assembly::SystemAssemblyDescription;
using orvd::system_assembly::SystemInstance;
using orvd::system_assembly::SystemRuntimeContext;

static_assert(std::is_abstract_v<ContinuousStateAdvancer>);
static_assert(std::is_abstract_v<ContinuousStateRhs>);
static_assert(std::is_base_of_v<ContinuousStateRhs, SystemRhsBridge>);
static_assert(!std::is_copy_constructible_v<SystemRhsBridge>);
static_assert(!std::is_move_constructible_v<SystemRhsBridge>);

class CompleteAdvancerContract final : public ContinuousStateAdvancer {
   public:
    int continuous_state_size() const override { return 0; }
    double current_time_seconds() const override { return 0.0; }
    void CopyCurrentState(Eigen::Ref<Eigen::VectorXd>) const override {}
    void AdvanceTo(double) override {}
    void ReinitializeAfterExternalChange(
        double, const Eigen::Ref<const Eigen::VectorXd>&) override {}
    std::optional<ContinuousStateDenseOutputInterval> dense_output_interval()
        const override {
        return std::nullopt;
    }
    void CopyDenseState(double,
                        Eigen::Ref<Eigen::VectorXd>) const override {}
};

static_assert(!std::is_abstract_v<CompleteAdvancerContract>);

int failure_count = 0;

void Expect(bool condition, const char* description) {
    if (!condition) {
        std::printf("FAIL %s\n", description);
        ++failure_count;
    }
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

RigidBodyInertiaParameters MakeInertia(double mass) {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass;
    inertia.center_of_mass_in_body_frame = Eigen::Vector3d(0.1, 0.0, 0.0);
    inertia.unit_inertia_moments = Eigen::Vector3d(0.3, 0.4, 0.5);
    inertia.unit_inertia_products.setZero();
    return inertia;
}

struct OneDofFixture {
    MultibodyModel model;
    orvd::multibody_model::JointHandle joint;

    OneDofFixture() {
        const auto body = model.AddRigidBody("body", MakeInertia(2.0));
        joint = model.AddRevoluteJoint(
            "joint", model.world_frame(), model.body_frame(body),
            Eigen::Vector3d::UnitZ(), 0.3);
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();
    }
};

struct TwoDofFixture {
    MultibodyModel model;

    TwoDofFixture() {
        const auto rotor = model.AddRigidBody("rotor", MakeInertia(2.0));
        const auto slider = model.AddRigidBody("slider", MakeInertia(1.0));
        model.AddRevoluteJoint(
            "revolute", model.world_frame(), model.body_frame(rotor),
            Eigen::Vector3d::UnitZ(), 0.2);
        model.AddPrismaticJoint(
            "prismatic", model.body_frame(rotor), model.body_frame(slider),
            Eigen::Vector3d::UnitX(), 0.4);
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();
    }
};

void CheckDynamicStateMappingAndAtomicRefusal() {
    TwoDofFixture fixture;
    const SystemAssemblyDescription description(fixture.model);
    const SystemInstance system(description);
    auto source = system.CreateDefaultRuntimeContext(0.0);
    auto destination = system.CreateDefaultRuntimeContext(0.0);

    Eigen::VectorXd stated(4);
    stated << 0.25, -0.5, 1.75, -2.25;
    system.SetContinuousState(*source, stated);
    Eigen::VectorXd copied(4);
    copied.setConstant(19.0);
    system.CopyContinuousState(*source, copied);
    Expect(copied == stated,
           "the dynamic [q; v] layout copies every component in order");

    system.SetContinuousState(*destination, copied);
    Eigen::VectorXd round_trip(4);
    system.CopyContinuousState(*destination, round_trip);
    Expect(round_trip == stated,
           "the contiguous state and runtime context mapping is reversible");

    const Eigen::VectorXd before = round_trip;
    Eigen::VectorXd invalid = stated;
    invalid[3] = std::numeric_limits<double>::quiet_NaN();
    ExpectInvalidArgument(
        [&] { system.SetContinuousState(*destination, invalid); },
        "a non-finite velocity is refused before q or v is changed");
    system.CopyContinuousState(*destination, round_trip);
    Expect(round_trip == before,
           "a refused whole-state load leaves both q and v unchanged");

    Eigen::VectorXd wrong_size(3);
    wrong_size.setZero();
    ExpectInvalidArgument(
        [&] { system.SetContinuousState(*destination, wrong_size); },
        "a wrong-sized contiguous state is refused explicitly");
}

void CheckComponentwiseTolerances() {
    Eigen::VectorXd absolute(4);
    absolute << 1.0e-12, 2.0e-11, 3.0e-10, 4.0e-9;
    const ContinuousStateErrorTolerances tolerances(1.0e-8, absolute);
    Expect(tolerances.relative_tolerance() == 1.0e-8,
           "the relative tolerance is retained exactly");
    Expect(tolerances.component_absolute_tolerances() == absolute,
           "distinct component tolerances retain their order and values");

    Eigen::VectorXd invalid = absolute;
    invalid[2] = 0.0;
    ExpectInvalidArgument(
        [&] { (void)ContinuousStateErrorTolerances(1.0e-8, invalid); },
        "a zero component tolerance is refused");
    invalid = absolute;
    invalid[1] = std::numeric_limits<double>::quiet_NaN();
    ExpectInvalidArgument(
        [&] { (void)ContinuousStateErrorTolerances(1.0e-8, invalid); },
        "a non-finite component tolerance is refused");
    ExpectInvalidArgument(
        [&] { (void)ContinuousStateErrorTolerances(-1.0, absolute); },
        "a negative relative tolerance is refused");
}

void CheckModelAwareQuaternionGate() {
    MultibodyModel model;
    const auto body = model.AddRigidBody("free", MakeInertia(2.0));
    model.DeclareFreeBody(body);
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();
    const SystemAssemblyDescription description(model);
    const SystemInstance system(description);
    auto context = system.CreateDefaultRuntimeContext(0.0);
    Eigen::VectorXd before(system.continuous_state_size());
    system.CopyContinuousState(*context, before);

    Eigen::VectorXd invalid = before;
    const auto q_range = model.GetFreeBodyPositionRange(body);
    invalid.segment(q_range.start(), 4).setZero();
    invalid.tail(model.num_generalized_velocities()).setConstant(3.0);
    ExpectInvalidArgument(
        [&] { system.SetContinuousState(*context, invalid); },
        "the contiguous bridge preserves the free-body quaternion gate");
    Eigen::VectorXd after(system.continuous_state_size());
    system.CopyContinuousState(*context, after);
    Expect(after == before,
           "a rejected quaternion leaves both q and v at their accepted values");
}

void CheckDedicatedTrialRhs() {
    OneDofFixture fixture;
    const SystemAssemblyDescription description(fixture.model);
    const SystemInstance system(description);
    const CompiledSystemPlan plan(system);
    auto accepted = system.CreateDefaultRuntimeContext(0.0);
    auto trial = system.CreateDefaultRuntimeContext(0.0);
    auto expected_context = system.CreateDefaultRuntimeContext(0.0);

    Eigen::VectorXd accepted_state(2);
    accepted_state << -0.7, -1.25;
    system.SetContinuousState(*accepted, accepted_state);

    const auto trial_component = system.GetMultibodyComponentView(
        *trial, system.multibody_component());
    const auto expected_component = system.GetMultibodyComponentView(
        *expected_context, system.multibody_component());
    fixture.model.SetRevoluteJointDampingCoefficient(
        &trial_component.context(), fixture.joint, 0.6);
    fixture.model.SetRevoluteJointDampingCoefficient(
        &expected_component.context(), fixture.joint, 0.6);

    Eigen::VectorXd trial_state(2);
    trial_state << 0.4, 2.0;
    system.SetContinuousState(*expected_context, trial_state);
    Eigen::VectorXd expected(2);
    plan.CalcStateTimeDerivatives(*expected_context, {}, {}, {}, expected);

    SystemRuntimeContext* const trial_observer = trial.get();
    SystemRhsBridge bridge(system, plan, std::move(trial),
                           NoCallTimeAppliedForces{});
    Expect(bridge.continuous_state_size() == 2,
           "the RHS size comes from the finalized one-DOF system");
    Eigen::VectorXd actual(2);
    bridge.CalcTimeDerivatives(1.25, trial_state, actual);
    Expect(actual == expected,
           "the bridge evaluates the configured trial context through G42");
    Expect(trial_observer->time_seconds() == 1.25,
           "the bridge writes the backend trial time into its trial context");

    Eigen::VectorXd observed_accepted(2);
    system.CopyContinuousState(*accepted, observed_accepted);
    Expect(observed_accepted == accepted_state,
           "an RHS trial cannot reach or change the accepted context");

    Eigen::VectorXd invalid = trial_state;
    invalid[1] = std::numeric_limits<double>::infinity();
    actual.setConstant(23.0);
    ExpectInvalidArgument(
        [&] { bridge.CalcTimeDerivatives(1.5, invalid, actual); },
        "an invalid trial state is refused");
    Expect(actual == Eigen::VectorXd::Constant(2, 23.0),
           "a refused trial leaves the caller's derivative output unchanged");
    Eigen::VectorXd observed_trial(2);
    system.CopyContinuousState(*trial_observer, observed_trial);
    Expect(trial_observer->time_seconds() == 1.25 &&
               observed_trial == trial_state,
           "a refused trial changes neither trial time nor trial state");
    system.CopyContinuousState(*accepted, observed_accepted);
    Expect(observed_accepted == accepted_state,
           "a refused RHS trial also cannot change the accepted context");

    Eigen::VectorXd wrong_output(1);
    wrong_output[0] = 31.0;
    ExpectInvalidArgument(
        [&] { bridge.CalcTimeDerivatives(1.5, trial_state, wrong_output); },
        "a wrong-sized derivative output is refused before evaluation");
    Expect(wrong_output[0] == 31.0,
           "a wrong-sized derivative output remains untouched");
}

void CheckPlanAndTrialBelongToTheSameSystem() {
    OneDofFixture fixture;
    const SystemAssemblyDescription first_description(fixture.model);
    const SystemAssemblyDescription second_description(fixture.model);
    const SystemInstance first(first_description);
    const SystemInstance second(second_description);
    const CompiledSystemPlan foreign_plan(second);
    auto first_trial = first.CreateDefaultRuntimeContext(0.0);
    ExpectInvalidArgument(
        [&] {
            (void)SystemRhsBridge(first, foreign_plan, std::move(first_trial),
                                  NoCallTimeAppliedForces{});
        },
        "the RHS plan and trial context must belong to the same system");
}

}  // namespace

int main() {
    CheckDynamicStateMappingAndAtomicRefusal();
    CheckComponentwiseTolerances();
    CheckModelAwareQuaternionGate();
    CheckDedicatedTrialRhs();
    CheckPlanAndTrialBelongToTheSameSystem();
    if (failure_count != 0) {
        std::printf("%d system RHS bridge checks failed\n", failure_count);
        return 1;
    }
    std::printf("continuous-state RHS bridge verified\n");
    return 0;
}
