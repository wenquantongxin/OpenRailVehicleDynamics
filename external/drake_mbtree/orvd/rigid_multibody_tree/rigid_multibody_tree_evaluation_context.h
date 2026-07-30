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

#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/multibody_runtime/multibody_state_layout.h"
#include "orvd/rigid_multibody_tree/rigid_multibody_tree_evaluation_context_fwd.h"

namespace orvd::rigid_multibody_tree::internal {

class RigidMultibodyTreeEvaluationContext {
   public:
    /// Builds the state this context owns against `layout`.
    ///
    /// `layout` belongs to the finalized model: it is created once when the
    /// model is finalized, outlives every context built against it, and is
    /// shared by all of them. The state is not shared — that is the whole point
    /// of a context.
    ///
    /// This constructor establishes storage only. The model-aware creation path
    /// that writes the finalized model's default positions and default physical
    /// parameters is G27's; a context built here and evaluated without it would
    /// be evaluating a model whose bodies are all massless and whose joints are
    /// all at zero.
    explicit RigidMultibodyTreeEvaluationContext(
        const multibody_runtime::MultibodyStateLayout& layout)
        : state_(layout) {}

    RigidMultibodyTreeEvaluationContext(
        multibody_runtime::MultibodyStateLayout&&) = delete;

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
};

}  // namespace orvd::rigid_multibody_tree::internal
