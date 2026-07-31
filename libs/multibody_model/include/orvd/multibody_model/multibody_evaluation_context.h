#pragma once

/// @file
/// One evaluation's state, for a finalized model.
///
/// A context holds what varies between evaluations of the same model: the
/// generalized coordinates, the context-mutable physical parameters, and the
/// caches computed from those. The model holds what does not vary: the topology,
/// the gravity vector, the element names.
///
/// It comes from the model and from nowhere else. A context assembled any other
/// way would hold the state store's own zeros — zero positions, a zero
/// quaternion where a rotation belongs, massless bodies — and it would evaluate
/// perfectly happily, producing arithmetic on a model nobody described.
///
/// The model must outlive it. A context is sized and bound at creation from the
/// finalized model's layout, and it deliberately does not keep that model
/// alive: a context that owned its model would make "which model is this" a
/// question with two answers whenever a caller held both.
///
/// Nothing vendored appears here. The type behind the pointer is the rigid
/// tree's own evaluation context, and it stays behind the pointer.

#include <memory>

#include <Eigen/Dense>

namespace orvd::multibody_model {

class MultibodyModel;

class MultibodyEvaluationContext {
   public:
    ~MultibodyEvaluationContext();

    // Two contexts on one model are meant to be genuinely independent, which is
    // what makes it safe to evaluate the same model at two configurations. A
    // copy would duplicate the coordinates while sharing nothing about how they
    // came to hold what they hold; a move would leave behind a context bound to
    // a model with its state taken away.
    MultibodyEvaluationContext(const MultibodyEvaluationContext&) = delete;
    MultibodyEvaluationContext& operator=(const MultibodyEvaluationContext&) =
        delete;
    MultibodyEvaluationContext(MultibodyEvaluationContext&&) = delete;
    MultibodyEvaluationContext& operator=(MultibodyEvaluationContext&&) =
        delete;

    /// The model's generalized positions, in the order its coordinate ranges
    /// describe.
    ///
    /// Read-only. Which numbers are quaternion components and what a legitimate
    /// one is are model knowledge, so a write goes through the model rather than
    /// through here.
    [[nodiscard]] const Eigen::VectorXd& generalized_positions() const;

    /// The model's generalized velocities. Same ordering, same read-only rule.
    [[nodiscard]] const Eigen::VectorXd& generalized_velocities() const;

   private:
    // Only the finalized model may issue one. See
    // MultibodyModel::CreateDefaultContext().
    friend class MultibodyModel;

    class Implementation;
    explicit MultibodyEvaluationContext(
        std::unique_ptr<Implementation> implementation);

    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::multibody_model
