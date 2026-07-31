#pragma once

/// @file
/// What an evaluation context actually holds.
///
/// Private to this library, and deliberately so: it names the rigid tree's own
/// evaluation context, which is precisely the thing the public header exists to
/// keep out of a consumer's include graph.
///
/// Two translation units need it — the context's own members, and the model
/// that issues one — so it is a header rather than a definition sitting in
/// whichever file happened to need it first.

#include <memory>
#include <utility>

#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model_handles.h"
#include "orvd/rigid_multibody_tree/rigid_multibody_tree_evaluation_context.h"

namespace orvd::multibody_model {

class MultibodyEvaluationContext::Implementation {
   public:
    Implementation(internal::ModelIdentity issuer,
                   std::unique_ptr<rigid_multibody_tree::internal::
                                       RigidMultibodyTreeEvaluationContext>
                       tree_context)
        : issuer_(issuer), tree_context_(std::move(tree_context)) {}

    /// The model that issued this context.
    ///
    /// Carried so that a model handed somebody else's context refuses it by
    /// name. The rigid tree would refuse it too — the state is bound to a
    /// layout by object identity — but it would refuse it in terms of a layout
    /// the caller has never heard of, about a model they did not name.
    [[nodiscard]] internal::ModelIdentity issuer() const { return issuer_; }

    [[nodiscard]] const rigid_multibody_tree::internal::
        RigidMultibodyTreeEvaluationContext&
        tree_context() const {
        return *tree_context_;
    }

    [[nodiscard]] rigid_multibody_tree::internal::
        RigidMultibodyTreeEvaluationContext&
        mutable_tree_context() {
        return *tree_context_;
    }

   private:
    internal::ModelIdentity issuer_;

    // Owned by pointer because the tree's context cannot be moved: it is bound
    // by object identity to the layout it was created against, and a move would
    // be the one operation that could separate them.
    std::unique_ptr<
        rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext>
        tree_context_;
};

}  // namespace orvd::multibody_model
