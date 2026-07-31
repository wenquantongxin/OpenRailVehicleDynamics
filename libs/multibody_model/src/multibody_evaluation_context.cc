#include "orvd/multibody_model/multibody_evaluation_context.h"

#include <utility>

#include "multibody_evaluation_context_implementation.h"

namespace orvd::multibody_model {

MultibodyEvaluationContext::MultibodyEvaluationContext(
    std::unique_ptr<Implementation> implementation)
    : implementation_(std::move(implementation)) {}

MultibodyEvaluationContext::~MultibodyEvaluationContext() = default;

const Eigen::VectorXd& MultibodyEvaluationContext::generalized_positions()
    const {
    return implementation_->tree_context().state().generalized_positions();
}

const Eigen::VectorXd& MultibodyEvaluationContext::generalized_velocities()
    const {
    return implementation_->tree_context().state().generalized_velocities();
}

}  // namespace orvd::multibody_model
