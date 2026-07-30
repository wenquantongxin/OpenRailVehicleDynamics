// What a cache slot promises: a value can be read only when the state it was
// computed from has not moved since, and a recomputation that did not finish
// leaves nothing readable behind.
//
// The refusals carry the weight again. A slot that returned its buffer when cold
// would hand out a value nobody computed; one that returned it when stale would
// hand out a value computed from state that has since changed; one that returned
// it after an abandoned recomputation would hand out half of one. All three look
// exactly like an ordinary result at the call site, and all three would surface
// as a dynamics discrepancy many passes later.
#include <cstdio>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "orvd/multibody_runtime/multibody_cache_slot.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/multibody_runtime/multibody_state_layout.h"

namespace {

using orvd::multibody_runtime::JointActuatorParameters;
using orvd::multibody_runtime::MultibodyStateInstance;
using orvd::multibody_runtime::MultibodyStateLayout;
using orvd::multibody_runtime::MultibodyStateLayoutDescription;
using orvd::multibody_runtime::MultibodyStateVersionSource;
using orvd::multibody_runtime::RigidBodyInertiaParameters;
using orvd::multibody_runtime::VersionedCacheSlot;

int failure_count = 0;

void ExpectTrue(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL %s\n", what.c_str());
        ++failure_count;
    }
}

/// Runs `attempt` and reports whether it refused with the cold/stale error.
template <typename Attempt>
bool WasRefused(Attempt&& attempt) {
    try {
        attempt();
    } catch (const std::logic_error&) {
        return true;
    }
    return false;
}

void ExpectRefused(bool refused, const std::string& what) {
    ExpectTrue(refused, what + ": expected a refusal, but a value was returned");
}

/// Stands in for a rigid tree cache value. The base layer must not name the real
/// ones — the dependency runs the other way — so what is exercised here is the
/// slot mechanism, with a value type that only has to be constructible at a size
/// and writable in place.
struct SyntheticPass {
    explicit SyntheticPass(int entry_count)
        : entries(static_cast<std::size_t>(entry_count), 0.0) {}
    std::vector<double> entries;
};

// Two representative dependency subsets. The full current runtime-contract
// mapping is G24's subject; what matters here is that a slot depending on two
// sources and a slot depending on one behave independently.
using PositionPassSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kGeneralizedPositions,
                       MultibodyStateVersionSource::kFixedFramePoses>;
using ReflectedInertiaSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kJointActuatorParameters>;

// A slot carries a bound state, a pre-allocated value and a committed snapshot.
// Copying it would produce a second value claiming the first one's committed
// snapshot: two slots that say they were computed, one of which never was.
static_assert(!std::is_copy_constructible_v<PositionPassSlot>);
static_assert(!std::is_copy_assignable_v<PositionPassSlot>);
static_assert(!std::is_move_constructible_v<PositionPassSlot>);
static_assert(!std::is_move_assignable_v<PositionPassSlot>);

// Binding to a state that dies at the end of the full expression would leave the
// slot reading versions through a dangling pointer. `MultibodyStateInstance` has
// no move constructor, but guaranteed copy elision still lets a function return
// one as a prvalue, so the hazard is reachable and is refused by name.
static_assert(!std::is_constructible_v<PositionPassSlot,
                                       MultibodyStateInstance&&, std::in_place_t,
                                       int>);

// An untagged forwarding constructor would swallow calls meant for other
// overloads, so the in-place tag is required rather than optional.
static_assert(!std::is_constructible_v<PositionPassSlot,
                                       const MultibodyStateInstance&, int>);

MultibodyStateLayoutDescription SampleDescription() {
    MultibodyStateLayoutDescription description;
    description.generalized_position_count = 9;
    description.generalized_velocity_count = 8;
    description.rigid_body_count = 3;
    description.fixed_frame_count = 2;
    description.joint_velocity_counts = {6, 1, 1};
    description.joint_actuator_count = 2;
    description.revolute_spring_count = 1;
    description.linear_bushing_count = 1;
    return description;
}

/// Writes something into the slot's storage and commits, as a calculator would.
void RecomputeAndCommit(PositionPassSlot& slot, double marker) {
    SyntheticPass& storage = slot.mutable_value_for_recomputation();
    for (double& entry : storage.entries) {
        entry = marker;
    }
    slot.CommitRecomputation();
}

void CheckColdSlotIsColdEvenWhenEveryVersionIsZero() {
    const MultibodyStateLayout layout(SampleDescription());
    const MultibodyStateInstance state(layout);
    const PositionPassSlot slot(state, std::in_place, 4);

    // The trap this exists to close: a fresh state has every version at zero, so
    // a slot that inferred freshness by comparing versions alone would find a
    // zero snapshot matching a zero state and serve its default-constructed
    // buffer as a computed result. "Never computed" has to be a distinct
    // statement from "computed when everything was at zero".
    ExpectTrue(!slot.is_fresh(),
               "a slot that has never been computed is not fresh, even though "
               "every version it depends on is still at its initial value");
    ExpectRefused(WasRefused([&] { (void)slot.value(); }),
                  "reading a slot that has never been computed");
}

void CheckCommitMakesTheValueReadable() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);

    RecomputeAndCommit(slot, 7.5);
    ExpectTrue(slot.is_fresh(), "a slot is fresh once a recomputation is committed");
    ExpectTrue(slot.value().entries.size() == 4,
               "the slot's value keeps the size it was pre-allocated at");
    ExpectTrue(slot.value().entries[0] == 7.5 && slot.value().entries[3] == 7.5,
               "the slot hands back what the recomputation wrote");

    // Repeated reads do not disturb freshness: reading is not an event.
    (void)slot.value();
    (void)slot.value();
    ExpectTrue(slot.is_fresh(), "reading a fresh slot leaves it fresh");
}

void CheckADeclaredSourceGoingStaleWithdrawsTheValue() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);
    RecomputeAndCommit(slot, 1.0);

    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 0.5));
    ExpectTrue(!slot.is_fresh(),
               "changing a declared source makes the slot stale");
    ExpectRefused(WasRefused([&] { (void)slot.value(); }),
                  "reading a slot whose declared source has moved");

    RecomputeAndCommit(slot, 2.0);
    ExpectTrue(slot.is_fresh() && slot.value().entries[0] == 2.0,
               "recomputing against the new state makes it readable again");

    // The second declared source, independently.
    orvd::multibody_runtime::FixedFramePoseParameters pose;
    pose.p_PoFo_P = Eigen::Vector3d(1.0, 0.0, 0.0);
    state.set_fixed_frame_pose_parameters(0, pose);
    ExpectTrue(!slot.is_fresh(),
               "changing the slot's other declared source also makes it stale");
}

void CheckAnUndeclaredSourceLeavesTheSlotAlone() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);
    RecomputeAndCommit(slot, 3.0);

    // Velocities, inertias and actuator parameters are not in this slot's
    // dependency set. If any of them invalidated it, every velocity write would
    // drag the position pass along with it — correct answers, and a recompute
    // count that reports a dependency the physics does not have.
    state.set_generalized_velocities(Eigen::VectorXd::Constant(8, 4.0));
    ExpectTrue(slot.is_fresh(), "a velocity write does not disturb a slot that "
                                "does not declare velocities");
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 3.0;
    state.set_rigid_body_inertia_parameters(0, inertia);
    ExpectTrue(slot.is_fresh(), "an inertia write does not disturb it either");
    JointActuatorParameters actuator;
    actuator.rotor_inertia = 0.25;
    state.set_joint_actuator_parameters(0, actuator);
    ExpectTrue(slot.is_fresh(),
               "nor does an actuator write");
    ExpectTrue(slot.value().entries[0] == 3.0,
               "and the value it still holds is the one that was committed");
}

void CheckTheMutableEntryPointInvalidatesBeforeItHandsOverStorage() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);
    RecomputeAndCommit(slot, 5.0);
    ExpectTrue(slot.is_fresh(), "the slot starts this check fresh");

    SyntheticPass& storage = slot.mutable_value_for_recomputation();
    ExpectTrue(!slot.is_fresh(),
               "taking the storage for recomputation makes the slot stale at "
               "once, before anything has been written");
    ExpectRefused(WasRefused([&] { (void)slot.value(); }),
                  "reading a slot whose recomputation is in progress");

    // An abandoned recomputation: the caller writes part of the value and then
    // gives up. Nothing has changed in the state, so a slot that invalidated
    // only on failure — or that re-derived freshness from the versions — would
    // now serve a half-written buffer as a finished result.
    storage.entries[0] = 99.0;
    ExpectTrue(!slot.is_fresh(),
               "a recomputation that was abandoned leaves the slot stale even "
               "though no source has changed");
    ExpectRefused(WasRefused([&] { (void)slot.value(); }),
                  "reading a slot after an abandoned recomputation");

    RecomputeAndCommit(slot, 6.0);
    ExpectTrue(slot.is_fresh() && slot.value().entries[0] == 6.0,
               "a later completed recomputation clears the abandoned one");
}

void CheckSlotsWithDifferentDependenciesMoveIndependently() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot position_slot(state, std::in_place, 4);
    ReflectedInertiaSlot actuator_slot(state, std::in_place, 2);

    RecomputeAndCommit(position_slot, 1.0);
    actuator_slot.mutable_value_for_recomputation().entries[0] = 8.0;
    actuator_slot.CommitRecomputation();
    ExpectTrue(position_slot.is_fresh() && actuator_slot.is_fresh(),
               "both slots start fresh");

    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 0.25));
    ExpectTrue(!position_slot.is_fresh(),
               "the position write makes the slot that declares positions stale");
    ExpectTrue(actuator_slot.is_fresh(),
               "the same write leaves the slot that does not declare positions "
               "fresh");

    JointActuatorParameters actuator;
    actuator.gear_ratio = -3.0;
    state.set_joint_actuator_parameters(1, actuator);
    ExpectTrue(!actuator_slot.is_fresh(),
               "the actuator write makes the actuator slot stale");

    RecomputeAndCommit(position_slot, 2.0);
    ExpectTrue(!actuator_slot.is_fresh(),
               "committing one slot does not make another one fresh");
}

void CheckTwoSlotsOnTwoStatesDoNotShareFreshness() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance first(layout);
    MultibodyStateInstance second(layout);
    PositionPassSlot first_slot(first, std::in_place, 4);
    PositionPassSlot second_slot(second, std::in_place, 4);

    RecomputeAndCommit(first_slot, 1.0);
    RecomputeAndCommit(second_slot, 2.0);

    first.set_generalized_positions(Eigen::VectorXd::Constant(9, 9.0));
    ExpectTrue(!first_slot.is_fresh(),
               "writing to one state makes its own slot stale");
    ExpectTrue(second_slot.is_fresh(),
               "writing to one state does not make the other state's slot stale");
    ExpectTrue(second_slot.value().entries[0] == 2.0,
               "and the other state's slot still holds its own value");
}

void CheckTheValueObjectKeepsItsAddress() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);

    // The slot is pre-allocated: the value is built once, at the size the model
    // implies, and invalidating or committing must not rebuild it. A slot that
    // reconstructed the value would allocate on a path that runs every time the
    // state moves.
    const void* const first_address = &slot.mutable_value_for_recomputation();
    slot.CommitRecomputation();
    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 1.0));
    const void* const after_invalidation =
        &slot.mutable_value_for_recomputation();
    slot.CommitRecomputation();
    const void* const after_commit = &slot.mutable_value_for_recomputation();

    ExpectTrue(first_address == after_invalidation &&
                   first_address == after_commit,
               "the value object stays at one address across invalidation and "
               "commit");
    ExpectTrue(slot.mutable_value_for_recomputation().entries.size() == 4,
               "and keeps the size it was pre-allocated at");
}

}  // namespace

int main() {
    CheckColdSlotIsColdEvenWhenEveryVersionIsZero();
    CheckCommitMakesTheValueReadable();
    CheckADeclaredSourceGoingStaleWithdrawsTheValue();
    CheckAnUndeclaredSourceLeavesTheSlotAlone();
    CheckTheMutableEntryPointInvalidatesBeforeItHandsOverStorage();
    CheckSlotsWithDifferentDependenciesMoveIndependently();
    CheckTwoSlotsOnTwoStatesDoNotShareFreshness();
    CheckTheValueObjectKeepsItsAddress();

    if (failure_count > 0) {
        std::printf("%d cache slot check(s) failed\n", failure_count);
        return 1;
    }
    std::printf(
        "a cache slot hands out its value only when the sources it declares have"
        " not moved since it was computed, and refuses when it is cold, stale or"
        " mid-recomputation\n");
    return 0;
}
