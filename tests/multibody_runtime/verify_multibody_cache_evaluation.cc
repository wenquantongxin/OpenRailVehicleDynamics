// When the evaluator calculates, when it does not, and what it leaves behind
// when a calculation goes wrong.
//
// The interesting cases are the ones where something fails. A calculation that
// throws part-way must leave a slot that refuses to be read, not one holding
// half an answer; a calculator that writes to the state must be refused before
// anything is committed, because the value it produced belongs to a state that
// no longer exists. Both look like ordinary success from the outside if they are
// handled wrongly, which is why they are checked here rather than trusted.
#include <array>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "orvd/multibody_runtime/multibody_cache_evaluation.h"
#include "orvd/multibody_runtime/multibody_cache_slot.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/multibody_runtime/multibody_state_layout.h"

namespace {

using orvd::multibody_runtime::EvaluateVersionedCacheSlot;
using orvd::multibody_runtime::FixedFramePoseParameters;
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

struct SyntheticPass {
    explicit SyntheticPass(int entry_count)
        : entries(static_cast<std::size_t>(entry_count), 0.0) {}

    SyntheticPass(const SyntheticPass&) = delete;
    SyntheticPass& operator=(const SyntheticPass&) = delete;
    SyntheticPass(SyntheticPass&&) = delete;
    SyntheticPass& operator=(SyntheticPass&&) = delete;

    void FillFromMemberFunction() { entries[0] = 17.0; }

    std::vector<double> entries;
};

static_assert(!std::is_copy_constructible_v<SyntheticPass>);
static_assert(!std::is_copy_assignable_v<SyntheticPass>);
static_assert(!std::is_move_constructible_v<SyntheticPass>);
static_assert(!std::is_move_assignable_v<SyntheticPass>);

/// Declares two of the five sources. The other three are what makes the
/// during-calculation guard worth testing: a guard that only looked at the
/// declared sources would let a calculator write velocities unnoticed.
using PositionPassSlot =
    VersionedCacheSlot<SyntheticPass,
                       MultibodyStateVersionSource::kGeneralizedPositions,
                       MultibodyStateVersionSource::kFixedFramePoses>;

/// Distinguishable from the evaluator's own refusals, so a test cannot pass by
/// catching the wrong thing.
struct CalculatorGaveUp : std::runtime_error {
    CalculatorGaveUp() : std::runtime_error("the calculator gave up") {}
};

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

std::array<std::uint64_t, 5> AllRevisions(const MultibodyStateInstance& state) {
    return {state.generalized_positions_version().revision_number(),
            state.generalized_velocities_version().revision_number(),
            state.fixed_frame_poses_version().revision_number(),
            state.rigid_body_inertias_version().revision_number(),
            state.joint_actuator_parameters_version().revision_number()};
}

void CheckColdComputesOnceAndHotQueriesDoNot() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);
    int calls = 0;
    const auto fill = [&calls](SyntheticPass& storage) {
        ++calls;
        storage.entries[0] = static_cast<double>(calls);
    };

    const SyntheticPass& first = EvaluateVersionedCacheSlot(slot, fill);
    ExpectTrue(calls == 1, "a cold slot calls the calculator exactly once");
    ExpectTrue(first.entries[0] == 1.0, "and returns what the calculator wrote");

    const SyntheticPass& second = EvaluateVersionedCacheSlot(slot, fill);
    ExpectTrue(calls == 1, "a second query does not call the calculator");
    ExpectTrue(&first == &second,
               "and hands back the same object rather than a rebuilt one");
    EvaluateVersionedCacheSlot(slot, fill);
    EvaluateVersionedCacheSlot(slot, fill);
    ExpectTrue(calls == 1, "nor do any number of further queries");
}

void CheckTheInvocationContractMatchesTheAcceptedCalculator() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);

    const auto calculate = &SyntheticPass::FillFromMemberFunction;
    const SyntheticPass& result =
        EvaluateVersionedCacheSlot(slot, calculate);
    ExpectTrue(result.entries[0] == 17.0,
               "a member-function calculator accepted by the invocation "
               "constraint is invoked with the slot value as its object");
}

void CheckRecomputationHappensOnRequestAndNotOnWrite() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);
    int calls = 0;
    const auto fill = [&calls](SyntheticPass& storage) {
        ++calls;
        storage.entries[0] = static_cast<double>(calls);
    };
    EvaluateVersionedCacheSlot(slot, fill);

    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 1.0));
    ExpectTrue(calls == 1,
               "writing a declared source does not itself recompute — work "
               "nobody has asked for is work that may never be wanted");

    ExpectTrue(EvaluateVersionedCacheSlot(slot, fill).entries[0] == 2.0,
               "the query after the write recomputes");
    ExpectTrue(calls == 2, "and does so exactly once");
    EvaluateVersionedCacheSlot(slot, fill);
    ExpectTrue(calls == 2, "and is fresh again afterwards");
}

void CheckTheCommittedRecomputationCountTracksSuccessesOnly() {
    // The count is what over-invalidation is seen with: a wrongly widened
    // dependency wastes work and changes no answer, so nothing in the values
    // could show it. What it counts therefore has to be exactly the
    // recomputations that produced a readable value — an attempt that threw
    // left the buffer half written, and counting it would be saying a value
    // became readable when none did.
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);

    ExpectTrue(slot.committed_recomputation_count() == 0,
               "a slot that has never been computed has committed nothing");

    int calls = 0;
    const auto fill = [&calls](SyntheticPass& storage) {
        ++calls;
        storage.entries[0] = static_cast<double>(calls);
    };
    EvaluateVersionedCacheSlot(slot, fill);
    ExpectTrue(slot.committed_recomputation_count() == 1,
               "a cold evaluation commits one recomputation");

    EvaluateVersionedCacheSlot(slot, fill);
    EvaluateVersionedCacheSlot(slot, fill);
    ExpectTrue(slot.committed_recomputation_count() == 1,
               "reading a fresh slot commits nothing further");

    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 1.0));
    ExpectTrue(slot.committed_recomputation_count() == 1,
               "a write nobody has queried yet commits nothing: the work has "
               "not been done, only made necessary");

    EvaluateVersionedCacheSlot(slot, fill);
    ExpectTrue(slot.committed_recomputation_count() == 2,
               "and the query after the write commits exactly one more");

    // A calculator that throws leaves the slot stale and unreadable, so no
    // value became readable and the count must not move.
    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 2.0));
    bool threw = false;
    try {
        EvaluateVersionedCacheSlot(slot, [](SyntheticPass& storage) {
            storage.entries[0] = -1.0;
            throw std::runtime_error("the calculator gave up half way");
        });
    } catch (const std::runtime_error&) {
        threw = true;
    }
    ExpectTrue(threw, "the calculator's exception reached the caller");
    ExpectTrue(slot.committed_recomputation_count() == 2,
               "a calculator that threw committed nothing");
    ExpectTrue(!slot.is_fresh(), "and left the slot stale");

    // A calculator that writes a versioned source: the evaluator refuses to
    // commit the result, so again nothing became readable.
    bool guard_fired = false;
    try {
        EvaluateVersionedCacheSlot(slot, [&state](SyntheticPass& storage) {
            storage.entries[0] = -2.0;
            state.set_generalized_positions(Eigen::VectorXd::Constant(9, 3.0));
        });
    } catch (const std::logic_error&) {
        guard_fired = true;
    }
    ExpectTrue(guard_fired, "the state guard refused the calculation");
    ExpectTrue(slot.committed_recomputation_count() == 2,
               "a calculation the guard refused committed nothing");

    // Only getting all the way through moves it again.
    EvaluateVersionedCacheSlot(slot, fill);
    ExpectTrue(slot.committed_recomputation_count() == 3,
               "a successful retry after two failures commits exactly one");
}

void CheckAThrowingCalculatorLeavesNothingReadable() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);
    int calls = 0;
    const auto fill = [&calls](SyntheticPass& storage) {
        ++calls;
        storage.entries[0] = static_cast<double>(calls);
    };
    EvaluateVersionedCacheSlot(slot, fill);
    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 1.0));

    // The calculator writes part of the value and then fails. Nothing rolls the
    // buffer back — that would mean keeping a spare copy of every cache against
    // a possibility — so what has to hold is that the half-written buffer cannot
    // be read.
    bool propagated_unchanged = false;
    int failed_calculation_calls = 0;
    try {
        EvaluateVersionedCacheSlot(slot, [&](SyntheticPass& storage) {
            ++failed_calculation_calls;
            storage.entries[0] = 99.0;
            throw CalculatorGaveUp();
        });
    } catch (const CalculatorGaveUp&) {
        propagated_unchanged = true;
    } catch (...) {
        // Swallowed and re-thrown as something else, or caught and converted.
    }
    ExpectTrue(propagated_unchanged,
               "the calculator's own exception reaches the caller unchanged");
    ExpectTrue(failed_calculation_calls == 1,
               "one failed query makes exactly one calculation attempt");
    ExpectTrue(!slot.is_fresh(), "and leaves the slot stale");
    bool refused = false;
    try {
        (void)slot.value();
    } catch (const std::logic_error&) {
        refused = true;
    }
    ExpectTrue(refused, "so the half-written value cannot be read");

    // A retry after a failure is an ordinary evaluation: there is no failed
    // state to clear first.
    ExpectTrue(EvaluateVersionedCacheSlot(slot, fill).entries[0] == 2.0,
               "retrying after a failure succeeds and returns the new value");
    ExpectTrue(calls == 2, "the retry called the calculator once more");
}

void CheckACalculatorThatWritesTheStateIsRefused() {
    // Each of the five versioned sources in turn, from a slot that declares only
    // two of them. A guard that compared only the declared sources would accept
    // three of these five, and the value it committed would have been computed
    // from a state that no longer existed.
    struct Case {
        const char* what;
        void (*disturb)(MultibodyStateInstance&);
    };
    const Case cases[] = {
        {"generalized positions",
         [](MultibodyStateInstance& s) {
             s.set_generalized_positions(Eigen::VectorXd::Constant(9, 3.0));
         }},
        {"generalized velocities",
         [](MultibodyStateInstance& s) {
             s.set_generalized_velocities(Eigen::VectorXd::Constant(8, 3.0));
         }},
        {"fixed frame poses",
         [](MultibodyStateInstance& s) {
             FixedFramePoseParameters pose;
             pose.p_PoFo_P = Eigen::Vector3d::Constant(3.0);
             s.set_fixed_frame_pose_parameters(0, pose);
         }},
        {"rigid body inertias",
         [](MultibodyStateInstance& s) {
             RigidBodyInertiaParameters inertia;
             inertia.mass_kilograms = 3.0;
             s.set_rigid_body_inertia_parameters(0, inertia);
         }},
        {"joint actuator parameters",
         [](MultibodyStateInstance& s) {
             JointActuatorParameters actuator;
             actuator.rotor_inertia = 3.0;
             s.set_joint_actuator_parameters(0, actuator);
         }},
    };

    for (const Case& one : cases) {
        const MultibodyStateLayout layout(SampleDescription());
        MultibodyStateInstance state(layout);
        PositionPassSlot slot(state, std::in_place, 4);
        const std::string what =
            std::string("a calculator that writes ") + one.what;
        const std::array<std::uint64_t, 5> versions_before =
            AllRevisions(state);

        bool refused = false;
        bool disturbance_completed = false;
        try {
            EvaluateVersionedCacheSlot(slot, [&](SyntheticPass& storage) {
                storage.entries[0] = 42.0;
                one.disturb(state);
                disturbance_completed = true;
            });
        } catch (const std::logic_error&) {
            refused = true;
        }
        ExpectTrue(disturbance_completed,
                   what + ": the state write itself completed successfully");
        ExpectTrue(AllRevisions(state) != versions_before,
                   what + ": advanced a version before the evaluator refused");
        ExpectTrue(refused, what + ": is refused");
        ExpectTrue(!slot.is_fresh(), what + ": leaves the slot stale");

        bool value_refused = false;
        try {
            (void)slot.value();
        } catch (const std::logic_error&) {
            value_refused = true;
        }
        ExpectTrue(value_refused,
                   what + ": leaves the value it computed unreadable");

        int retry_calls = 0;
        const SyntheticPass& retried = EvaluateVersionedCacheSlot(
            slot, [&](SyntheticPass& storage) {
                ++retry_calls;
                storage.entries[0] = 84.0;
            });
        ExpectTrue(retry_calls == 1 && retried.entries[0] == 84.0,
                   what + ": can be followed by one successful clean retry");
    }
}

void CheckCalculatorExceptionTakesPrecedenceOverTheStateGuard() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);
    const std::array<std::uint64_t, 5> versions_before = AllRevisions(state);

    bool original_exception_observed = false;
    try {
        EvaluateVersionedCacheSlot(slot, [&](SyntheticPass& storage) {
            storage.entries[0] = 31.0;
            state.set_generalized_velocities(
                Eigen::VectorXd::Constant(8, 2.0));
            throw CalculatorGaveUp();
        });
    } catch (const CalculatorGaveUp&) {
        original_exception_observed = true;
    } catch (...) {
    }

    ExpectTrue(original_exception_observed,
               "a calculator that writes state and then throws keeps its own "
               "exception rather than being replaced by the state guard");
    ExpectTrue(AllRevisions(state) != versions_before,
               "the evaluator does not pretend to roll back the calculator's "
               "state side effect");
    ExpectTrue(!slot.is_fresh(),
               "but the exceptional path still leaves the slot stale");
    bool value_refused = false;
    try {
        (void)slot.value();
    } catch (const std::logic_error&) {
        value_refused = true;
    }
    ExpectTrue(value_refused,
               "and never exposes the partially computed value");
}

void CheckEvaluationDoesNotDisturbAnyVersion() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance state(layout);
    PositionPassSlot slot(state, std::in_place, 4);
    int calls = 0;
    const auto fill = [&calls](SyntheticPass& storage) {
        ++calls;
        storage.entries[0] = static_cast<double>(calls);
    };

    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 1.0));
    const std::array<std::uint64_t, 5> before = AllRevisions(state);

    EvaluateVersionedCacheSlot(slot, fill);  // cold
    EvaluateVersionedCacheSlot(slot, fill);  // hot
    EvaluateVersionedCacheSlot(slot, fill);  // hot again
    ExpectTrue(AllRevisions(state) == before,
               "a legitimate evaluation advances no state version — evaluating "
               "is a read of the state, whatever it costs");

    state.set_generalized_positions(Eigen::VectorXd::Constant(9, 2.0));
    const std::array<std::uint64_t, 5> after_write = AllRevisions(state);
    bool failure_observed = false;
    try {
        EvaluateVersionedCacheSlot(slot, [](SyntheticPass& storage) {
            storage.entries[0] = 5.0;
            throw CalculatorGaveUp();
        });
    } catch (const CalculatorGaveUp&) {
        failure_observed = true;
    }
    ExpectTrue(failure_observed,
               "the failed-evaluation version check observed the intended "
               "calculator failure");
    ExpectTrue(AllRevisions(state) == after_write,
               "and neither does a failed one");
}

void CheckTwoStatesEvaluateIndependently() {
    const MultibodyStateLayout layout(SampleDescription());
    MultibodyStateInstance first(layout);
    MultibodyStateInstance second(layout);
    PositionPassSlot first_slot(first, std::in_place, 4);
    PositionPassSlot second_slot(second, std::in_place, 4);
    int first_calls = 0;
    int second_calls = 0;
    const auto fill_first = [&first_calls](SyntheticPass& storage) {
        ++first_calls;
        storage.entries[0] = 100.0 + first_calls;
    };
    const auto fill_second = [&second_calls](SyntheticPass& storage) {
        ++second_calls;
        storage.entries[0] = 200.0 + second_calls;
    };

    EvaluateVersionedCacheSlot(first_slot, fill_first);
    EvaluateVersionedCacheSlot(second_slot, fill_second);
    const std::array<std::uint64_t, 5> second_versions = AllRevisions(second);

    first.set_generalized_positions(Eigen::VectorXd::Constant(9, 1.0));
    EvaluateVersionedCacheSlot(first_slot, fill_first);

    ExpectTrue(first_calls == 2 && second_calls == 1,
               "recomputing one state's slot does not recompute the other's");
    ExpectTrue(second_slot.is_fresh(),
               "nor does it disturb the other's freshness");
    ExpectTrue(second_slot.value().entries[0] == 201.0,
               "nor its cached value");
    ExpectTrue(AllRevisions(second) == second_versions,
               "nor any of the other state's versions");
    ExpectTrue(first_slot.value().entries[0] == 102.0,
               "and the recomputed one holds its own new value");
}

}  // namespace

int main() {
    CheckColdComputesOnceAndHotQueriesDoNot();
    CheckTheInvocationContractMatchesTheAcceptedCalculator();
    CheckRecomputationHappensOnRequestAndNotOnWrite();
    CheckTheCommittedRecomputationCountTracksSuccessesOnly();
    CheckAThrowingCalculatorLeavesNothingReadable();
    CheckACalculatorThatWritesTheStateIsRefused();
    CheckCalculatorExceptionTakesPrecedenceOverTheStateGuard();
    CheckEvaluationDoesNotDisturbAnyVersion();
    CheckTwoStatesEvaluateIndependently();

    if (failure_count > 0) {
        std::printf("%d cache evaluation check(s) failed\n", failure_count);
        return 1;
    }
    std::printf(
        "the evaluator calculates when the slot is cold or stale and at no other"
        " time, leaves a failed calculation unreadable rather than half"
        " committed, and refuses to commit a value computed from a state the"
        " calculator moved\n");
    return 0;
}
