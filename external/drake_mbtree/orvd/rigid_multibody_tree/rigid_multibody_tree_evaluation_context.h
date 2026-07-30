#pragma once

/// @file
/// One evaluation's state and, from G27, its cache workspace.
///
/// This is the adapter layer's type, not the runtime base layer's. The base
/// layer owns state, versions and the slot mechanism and knows nothing about the
/// rigid tree; this context is where the tree's own cache workspace will be
/// composed on top of that state. Putting it here rather than in
/// `libs/multibody_runtime/` keeps the dependency running one way: the adapter
/// reaches down to the runtime, and the runtime never reaches back.
///
/// It is not a renamed `systems::Context`. That type was wide enough to be
/// accepted anywhere, which is why it ended up in 881 places in the landed tree,
/// most of which only ever read a generalized position. What each entry point
/// actually needs is stated at the entry point: a const state, a mutable state,
/// or — only where a lazily computed cache is evaluated — this. A signature that
/// asks for this context is a signature that says "I may compute something and
/// keep it".
///
/// It owns its state privately and cannot be copied, moved or rebound. That is
/// what makes two contexts on one model genuinely independent rather than
/// independent by convention: there is no way to assemble one out of somebody
/// else's state, and no way to move a workspace from one to another.
///
/// Not templated. The product is double-only, and a scalar parameter here would
/// be a slot reserved for a scalar that was decided against.
///
/// It holds no pointer back to the tree. The tree receives a context; a context
/// that also knew its tree would let a calculation reach the model through the
/// state, and then "what does this calculation read" would stop being answerable
/// from its signature.

#include <utility>

#include "drake/multibody/topology/forest.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/multibody_runtime/multibody_state_layout.h"
#include "orvd/rigid_multibody_tree/rigid_multibody_tree_cache_workspace.h"
#include "orvd/rigid_multibody_tree/rigid_multibody_tree_evaluation_context_fwd.h"

namespace drake::multibody::internal {
template <typename T>
class MultibodyTree;
}  // namespace drake::multibody::internal

namespace orvd::rigid_multibody_tree::internal {

class RigidMultibodyTreeEvaluationContext {
   public:
    // A context is one evaluation's private world. A copy would duplicate the
    // state while sharing nothing about how it came to hold what it holds; a
    // move would leave behind a context bound to a model with its state taken
    // away. Neither is a thing the design has a use for.
    RigidMultibodyTreeEvaluationContext(
        const RigidMultibodyTreeEvaluationContext&) = delete;
    RigidMultibodyTreeEvaluationContext& operator=(
        const RigidMultibodyTreeEvaluationContext&) = delete;
    RigidMultibodyTreeEvaluationContext(RigidMultibodyTreeEvaluationContext&&) =
        delete;
    RigidMultibodyTreeEvaluationContext& operator=(
        RigidMultibodyTreeEvaluationContext&&) = delete;

    const multibody_runtime::MultibodyStateInstance& state() const {
        return state_;
    }

    /// The eleven retained caches, for reading a value that is already fresh.
    const RigidMultibodyTreeCacheWorkspace& caches() const { return caches_; }

    /// The same caches, for an evaluation that may have to compute one.
    ///
    /// `mutable`, and reached through a const context. Evaluating a cache is a
    /// read as far as the model is concerned: it produces nothing the caller
    /// did not already imply by asking, and it leaves the state untouched. That
    /// is what logical constness means here, and it is expressed by the member
    /// being mutable rather than by a `const_cast` at each use — a cast would
    /// have to be written, and read, at every evaluation site, and each one
    /// would be a place where the reasoning could be wrong.
    RigidMultibodyTreeCacheWorkspace& mutable_caches() const { return caches_; }

   private:
    // Only the finalized tree may build one. A context that could be default
    // constructed would hold zero-filled positions and massless bodies, and it
    // would evaluate: the answers would be arithmetic on a model nobody built.
    // See MultibodyTree::CreateDefaultEvaluationContext().
    template <typename U>
    friend class drake::multibody::internal::MultibodyTree;

    RigidMultibodyTreeEvaluationContext(
        const multibody_runtime::MultibodyStateLayout& layout,
        const drake::multibody::internal::SpanningForest& forest,
        int num_links, int num_frames)
        : state_(layout),
          caches_(state_, forest, num_links, num_frames) {}

    RigidMultibodyTreeEvaluationContext(
        multibody_runtime::MultibodyStateLayout&&,
        const drake::multibody::internal::SpanningForest&, int, int) = delete;

    /// The state, writable only from within the tree's own entry points.
    ///
    /// Not public. A caller holding a mutable state could write positions
    /// without passing the model-aware gate that rejects a zero quaternion, and
    /// the first kinematics evaluation would then be computing with a rotation
    /// that is not one.
    multibody_runtime::MultibodyStateInstance& mutable_state() {
        return state_;
    }

   private:
    // Held by value, not by reference or pointer: a context that accepted a
    // state from outside could be handed the same state as another context, and
    // then "these two contexts are independent" would be something a caller had
    // to arrange rather than something the type guaranteed.
    //
    // G27 adds the cache workspace beside it, constructed together with it, so
    // that a workspace cannot outlive or be separated from the state its
    // freshness snapshots refer to.
    multibody_runtime::MultibodyStateInstance state_;

    // Constructed with the state and never separately: a workspace beside a
    // different state would hold freshness snapshots referring to versions that
    // are not the ones it would compare against, and those comparisons would
    // succeed often enough to be believed.
    mutable RigidMultibodyTreeCacheWorkspace caches_;
};

}  // namespace orvd::rigid_multibody_tree::internal
