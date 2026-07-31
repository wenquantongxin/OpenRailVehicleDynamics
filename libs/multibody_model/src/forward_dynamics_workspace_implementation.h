#pragma once

#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/articulated_body_force_cache.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "orvd/multibody_model/forward_dynamics_workspace.h"
#include "orvd/multibody_model/multibody_model_handles.h"

namespace orvd::multibody_model {

class ForwardDynamicsWorkspace::Implementation {
   public:
    Implementation(
        internal::ModelIdentity issuer,
        const drake::multibody::internal::MultibodyTree<double>& tree)
        : issuer_(issuer),
          forces_(tree),
          aba_force_(tree.forest()),
          acceleration_(tree.forest()),
          position_derivatives_(
              Eigen::VectorXd::Zero(tree.num_positions())) {}

    internal::ModelIdentity issuer_;
    drake::multibody::MultibodyForces<double> forces_;
    drake::multibody::internal::ArticulatedBodyForceCache<double> aba_force_;
    drake::multibody::internal::AccelerationKinematicsCache<double>
        acceleration_;
    Eigen::VectorXd position_derivatives_;
};

}  // namespace orvd::multibody_model
