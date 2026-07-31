#include "orvd/multibody_model/forward_dynamics_workspace.h"

#include <utility>

#include "forward_dynamics_workspace_implementation.h"

namespace orvd::multibody_model {

ForwardDynamicsWorkspace::ForwardDynamicsWorkspace(
    std::unique_ptr<Implementation> implementation)
    : implementation_(std::move(implementation)) {}

ForwardDynamicsWorkspace::~ForwardDynamicsWorkspace() = default;

}  // namespace orvd::multibody_model
