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
#include "orvd/rigid_multibody_tree/rigid_multibody_tree_evaluation_context.h"

namespace orvd::multibody_model {

class MultibodyEvaluationContext::Implementation {
   public:
    explicit Implementation(
        std::unique_ptr<rigid_multibody_tree::internal::
                            RigidMultibodyTreeEvaluationContext>
            tree_context)
        : tree_context_(std::move(tree_context)) {}

    [[nodiscard]] const rigid_multibody_tree::internal::
        RigidMultibodyTreeEvaluationContext&
        tree_context() const {
        return *tree_context_;
    }

   private:
    // Owned by pointer because the tree's context cannot be moved: it is bound
    // by object identity to the layout it was created against, and a move would
    // be the one operation that could separate them.
    std::unique_ptr<
        rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext>
        tree_context_;
};

}  // namespace orvd::multibody_model
