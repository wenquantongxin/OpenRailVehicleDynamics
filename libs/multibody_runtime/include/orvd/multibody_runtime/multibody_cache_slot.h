#pragma once

/// @file
/// A pre-allocated cache slot that knows which state sources it depends on.
///
/// A slot holds a value built once at the size the model implies, the versions
/// of the sources that value was computed from, and — implied by those —
/// whether it is still fresh. Beside those it keeps a count of how many
/// recomputations have been committed: a read-only diagnostic that takes no
/// part in freshness and that nothing consults to decide anything. It is there
/// because over-invalidation cannot be seen in any value (the answer stays
/// right and only the work is wasted), and [ADR-0002] names counting as how it
/// is seen instead.
///
/// A slot does not hold a calculator. What recomputes it is the caller's
/// business; what a slot guarantees is that a value can only be read when the
/// state it was computed from has not moved since.
///
/// The dependency set is part of the type. `VersionedCacheSlot<PositionPass,
/// kGeneralizedPositions, kFixedFramePoses>` says, in the declaration itself,
/// which declared sources make that value stale, and the slot always compares
/// exactly those sources. The slot does not own the calculator, so it cannot
/// prove that the declaration covers everything the calculation reads; the
/// rigid-tree adapter must verify that mapping with the real cold/hot-context
/// tests when it binds calculators to slots. There is no dependency graph and
/// no ticket: with the roots enumerated, comparing one to five versions answers
/// the freshness question directly.
///
/// Reads are checked. A cold or stale slot refuses to hand out its value rather
/// than returning whatever is currently in the buffer: a recomputation that
/// threw part-way leaves the buffer half written, and a half-written value looks
/// exactly like a finished one. The mutable entry point drops the committed
/// snapshot *before* it returns the reference, so an abandoned recomputation
/// leaves the slot stale — which is what stops the half-written value from ever
/// being read.
///
/// Freshness is not inferred from the versions alone. A brand new state has
/// every version at zero, and a brand new slot would have a zero snapshot to
/// match it, so a default-constructed value would read as a legitimately
/// computed one. The committed snapshot is therefore optional: absent means
/// nothing has ever been computed, which is a different statement from "computed
/// when everything was at zero".

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/multibody_runtime/multibody_state_version.h"

namespace orvd::multibody_runtime {

/// The state sources a cache slot can depend on.
///
/// These are exactly the versioned sources of `MultibodyStateInstance`. A cache
/// cannot depend on anything else in the state, because nothing else is
/// versioned — and nothing else is versioned because nothing retains a value
/// derived from it.
enum class MultibodyStateVersionSource {
    kGeneralizedPositions,
    kGeneralizedVelocities,
    kFixedFramePoses,
    kRigidBodyInertias,
    kJointActuatorParameters,
};

namespace internal {

/// The evaluator's narrow bridge to the versions of a slot's bound state.
///
/// It does not expose the state itself, a committed snapshot, or a way to
/// change freshness. Its definition lives with the evaluator that consumes the
/// observation.
struct CacheSlotEvaluationAccess;

inline MultibodyStateVersion CurrentVersionOf(
    const MultibodyStateInstance& state, MultibodyStateVersionSource source) {
    switch (source) {
        case MultibodyStateVersionSource::kGeneralizedPositions:
            return state.generalized_positions_version();
        case MultibodyStateVersionSource::kGeneralizedVelocities:
            return state.generalized_velocities_version();
        case MultibodyStateVersionSource::kFixedFramePoses:
            return state.fixed_frame_poses_version();
        case MultibodyStateVersionSource::kRigidBodyInertias:
            return state.rigid_body_inertias_version();
        case MultibodyStateVersionSource::kJointActuatorParameters:
            return state.joint_actuator_parameters_version();
    }
    throw std::logic_error(
        "multibody cache slot: unknown state version source");
}

template <MultibodyStateVersionSource... Sources>
constexpr bool SourcesAreDistinct() {
    const std::array<MultibodyStateVersionSource, sizeof...(Sources)> declared{
        Sources...};
    for (std::size_t i = 0; i < declared.size(); ++i) {
        for (std::size_t j = i + 1; j < declared.size(); ++j) {
            if (declared[i] == declared[j]) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace internal

/// A slot holding one cached `Value` computed from the declared `Sources`.
///
/// `Value` is any type this layer can hold. Deliberately any type: the base
/// layer must not name the rigid tree's cache value types, because the
/// dependency runs the other way. The adapter that owns those types composes
/// them into a workspace of named slot members, and that composition is what
/// freezes the slot layout — a type cannot have a slot appended to it, which is
/// a stronger statement than a registry that merely refuses to.
template <typename Value, MultibodyStateVersionSource... Sources>
class VersionedCacheSlot {
    static_assert(sizeof...(Sources) > 0,
                  "a cache slot with no dependencies would never go stale, so "
                  "its value could never be wrong and it would not need a slot");
    static_assert(internal::SourcesAreDistinct<Sources...>(),
                  "a repeated dependency source compares the same version twice "
                  "and states the dependency once too often");

   public:
    /// The versions this slot's value was computed from, in declaration order.
    using VersionSnapshot = std::array<MultibodyStateVersion, sizeof...(Sources)>;

    /// Builds the value in place, once, at whatever size the arguments imply.
    ///
    /// `std::in_place` is required rather than convenient: an untagged
    /// forwarding constructor would swallow calls meant for other overloads,
    /// including a single-argument one that looks like a copy.
    ///
    /// `state` must be a stable lvalue that outlives this slot. The slot reads
    /// its versions from that exact object; it never accepts them from a caller,
    /// because a caller that passed the wrong ones would be reporting a
    /// dependency the slot does not have, and nothing would notice.
    template <typename... Arguments>
    VersionedCacheSlot(const MultibodyStateInstance& state, std::in_place_t,
                       Arguments&&... arguments)
        : state_(&state), value_(std::forward<Arguments>(arguments)...) {}

    template <typename... Arguments>
    VersionedCacheSlot(const MultibodyStateInstance&&, std::in_place_t,
                       Arguments&&...) = delete;

    // A copy would produce a second value carrying the first one's committed
    // snapshot: two slots claiming to have been computed, only one of which
    // ever was. A move would leave its source bound to a state but holding a
    // value that had been taken away.
    VersionedCacheSlot(const VersionedCacheSlot&) = delete;
    VersionedCacheSlot& operator=(const VersionedCacheSlot&) = delete;
    VersionedCacheSlot(VersionedCacheSlot&&) = delete;
    VersionedCacheSlot& operator=(VersionedCacheSlot&&) = delete;

    /// Whether the value may be read: something has been committed, and none of
    /// the declared sources has moved since.
    [[nodiscard]] bool is_fresh() const {
        return committed_snapshot_.has_value() &&
               *committed_snapshot_ == CurrentSnapshot();
    }

    /// @throws std::logic_error if the slot is cold or stale. Returning the
    /// buffer instead would hand out either a value that was never computed or
    /// one computed from state that has since changed, and in both cases it
    /// looks like an ordinary result.
    [[nodiscard]] const Value& value() const {
        if (!committed_snapshot_.has_value()) {
            throw std::logic_error(
                "multibody cache slot: the value has never been computed");
        }
        if (*committed_snapshot_ != CurrentSnapshot()) {
            throw std::logic_error(
                "multibody cache slot: the value is stale — a source it depends "
                "on has changed since it was computed");
        }
        return value_;
    }

    /// The value's storage, for recomputing in place.
    ///
    /// Dropping the committed snapshot happens here, before the reference is
    /// handed over, not after the caller reports success. If it happened after,
    /// a recomputation that threw would leave a half-written buffer marked
    /// fresh; dropping it first means an abandoned recomputation leaves the slot
    /// stale and its buffer unreadable, which is the state that is safe to be in.
    [[nodiscard]] Value& mutable_value_for_recomputation() {
        committed_snapshot_.reset();
        return value_;
    }

    /// Records that the value now in storage was computed from the state as it
    /// stands.
    ///
    /// This is the success edge of recomputation, not a way to make an old
    /// buffer current. Call it only after the value has been completely
    /// recomputed through `mutable_value_for_recomputation()`. G25's evaluator
    /// owns that call sequence and tests the precondition; this storage
    /// primitive does not duplicate the evaluator with a second state machine.
    void CommitRecomputation() {
        // Counted before the snapshot, so that a count which cannot advance
        // leaves the slot stale rather than fresh-with-a-wrong-count. Stale is
        // the state that is safe to be in.
        if (committed_recomputation_count_ ==
            std::numeric_limits<std::uint64_t>::max()) {
            throw std::overflow_error(
                "multibody cache slot: the committed-recomputation count has "
                "reached its ceiling and cannot advance; it does not wrap, for "
                "the same reason a state version does not");
        }
        ++committed_recomputation_count_;
        committed_snapshot_ = CurrentSnapshot();
    }

    /// How many recomputations of this slot have been committed.
    ///
    /// Committed, not attempted: a calculator that threw, and a calculation the
    /// evaluator refused to commit because the state moved underneath it, both
    /// leave this where it was. What it counts is the number of times a value in
    /// this slot became readable.
    ///
    /// This is the observable [ADR-0002] names for the over-invalidation half of
    /// its criterion. Under-invalidation shows up as a wrong number, which a
    /// cold-versus-hot comparison finds. Over-invalidation does not: the answer
    /// stays right and only the work is wasted, so no value can reveal it. What
    /// reveals it is that a write touching nothing this slot declares still made
    /// the slot recompute.
    ///
    /// A count rather than a flag, because "did it recompute at all" cannot tell
    /// one recomputation from ten across a sequence of writes, and the sequence
    /// is where a dependency declared too widely shows itself.
    ///
    /// It is a long-term read-only diagnostic, not instrumentation added to
    /// prove a goal and then left behind. Nothing reads it to decide anything:
    /// it takes no part in freshness, is not a value hash, and exists to be
    /// compared between two moments.
    [[nodiscard]] std::uint64_t committed_recomputation_count() const {
        return committed_recomputation_count_;
    }

   private:
    VersionSnapshot CurrentSnapshot() const {
        return VersionSnapshot{internal::CurrentVersionOf(*state_, Sources)...};
    }

    friend struct internal::CacheSlotEvaluationAccess;

    const MultibodyStateInstance* state_;
    Value value_;
    /// Absent until a recomputation has been committed. Absent is not the same
    /// as a snapshot of zeros: see the file comment.
    std::optional<VersionSnapshot> committed_snapshot_;
    std::uint64_t committed_recomputation_count_{0};
};

}  // namespace orvd::multibody_runtime
