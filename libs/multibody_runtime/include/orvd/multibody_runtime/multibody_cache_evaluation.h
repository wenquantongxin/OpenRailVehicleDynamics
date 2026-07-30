#pragma once

/// @file
/// Lazy, in-place evaluation of a versioned cache slot.
///
/// One function, and the sequence written once. Recomputing a cache is four
/// steps that have to happen in one order — check freshness, take the storage,
/// calculate, commit — and getting that order wrong is not the kind of mistake
/// that shows up where it was made. Written out at each of the adapter's binding
/// sites it would be eleven chances to get it wrong; written here it is one.
///
/// The function is short because the slot already carries the hard part. Taking
/// the storage drops the committed snapshot, so a calculation that throws leaves
/// the slot stale and its buffer unreadable with no `try`, no rollback, and no
/// copy of the cache value made against the possibility of failure. The
/// exception simply propagates: nothing here has anything to undo.
///
/// What this does add is a guard on the state. A calculator is supposed to read
/// the state and nothing else; if it writes to it, the value comes from the
/// state as it was and the commit stamps the state as it now is, and the result
/// is a stale value wearing a fresh label — the exact failure the versions exist
/// to prevent. So the versions are read before the calculation and again after,
/// and a difference refuses the commit. All five are checked, not just the ones
/// the slot declares: a slot depending on positions and frame poses whose
/// calculator quietly wrote a velocity would sail past a check that only looked
/// at what was declared.
///
/// The guard proves that no versioned source was written during the calculation.
/// It does not prove the declaration is complete, and it says nothing about the
/// unversioned parameters, the model constants or externally supplied forces —
/// those are held by const binding and by the cold-versus-hot comparison the
/// adapter runs when it binds real calculators.
///
/// Single-threaded. No lock and no state machine between the comparison and the
/// commit, because there is no second thread for one to protect against.

#include <array>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "orvd/multibody_runtime/multibody_cache_slot.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/multibody_runtime/multibody_state_version.h"

namespace orvd::multibody_runtime {

namespace internal {

struct CacheSlotEvaluationAccess {
    template <typename Slot>
    static const MultibodyStateInstance& BoundState(const Slot& slot) {
        return *slot.state_;
    }
};

/// Every versioned source, for the during-calculation guard.
using EveryStateVersion = std::array<MultibodyStateVersion, 5>;

inline EveryStateVersion EveryVersionOf(const MultibodyStateInstance& state) {
    return {state.generalized_positions_version(),
            state.generalized_velocities_version(),
            state.fixed_frame_poses_version(),
            state.rigid_body_inertias_version(),
            state.joint_actuator_parameters_version()};
}

}  // namespace internal

/// Returns the slot's value, calculating it first if it is cold or stale.
///
/// `calculate` is called with the slot's storage to fill in place, and only when
/// there is something to do: a fresh slot never reaches it. It must not write to
/// the state — see the file comment for why, and note that this is checked
/// rather than merely asked for.
///
/// @throws whatever `calculate` throws, unchanged, leaving the slot stale.
/// @throws std::logic_error if `calculate` changed any state version, before
/// anything is committed.
template <typename Value, MultibodyStateVersionSource... Sources,
          typename Calculate>
const Value& EvaluateVersionedCacheSlot(
    VersionedCacheSlot<Value, Sources...>& slot, Calculate&& calculate) {
    static_assert(std::is_invocable_v<Calculate&, Value&>,
                  "a cache calculator is called with the slot's storage to fill "
                  "in place");
    static_assert(
        std::is_same_v<std::invoke_result_t<Calculate&, Value&>, void>,
        "a cache calculator writes into the storage it is given; a returned "
        "value would be a second place for the answer to live");

    if (slot.is_fresh()) {
        return slot.value();
    }

    const MultibodyStateInstance& state =
        internal::CacheSlotEvaluationAccess::BoundState(slot);
    const internal::EveryStateVersion before = internal::EveryVersionOf(state);

    Value& storage = slot.mutable_value_for_recomputation();
    calculate(storage);

    if (internal::EveryVersionOf(state) != before) {
        throw std::logic_error(
            "multibody cache evaluation: the calculator changed the state while "
            "computing, so the value was computed from one state and would be "
            "recorded as belonging to another");
    }
    slot.CommitRecomputation();
    return slot.value();
}

}  // namespace orvd::multibody_runtime
