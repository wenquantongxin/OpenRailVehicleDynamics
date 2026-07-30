#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/rigid_multibody_tree/rigid_multibody_tree_evaluation_context_fwd.h"

namespace drake {
namespace multibody {

/// This %ForceElement models a linear spring attached to a PrismaticJoint
/// and applies a force to that joint according to
/// <pre>
///   f = -k⋅(x - x₀)
/// </pre>
/// where x₀ is the nominal (zero spring force) position in meters,
/// x is the joint position in meters, f is the spring force in Newtons and
/// k is the spring constant in N/m.
/// Note that joint damping exists within the PrismaticJoint itself, and
/// so is not included here.
///
/// @note This is different from the LinearSpringDamper: this
/// %PrismaticSpring is associated with a joint, while the LinearSpringDamper
/// connects two bodies.
template <typename T>
class PrismaticSpring final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrismaticSpring);

  /// Constructor for a linear spring attached to the given prismatic joint.
  /// @param[in] nominal_position
  /// The nominal position of the spring x₀, in meters, at which the spring
  /// applies no force. This is measured the same way as the generalized
  /// position of the prismatic joint.
  /// @param[in] stiffness
  /// The stiffness k of the spring in N/m.
  /// @throws std::exception if `stiffness` is (strictly) negative.
  PrismaticSpring(const PrismaticJoint<T>& joint, double nominal_position,
                  double stiffness);

  ~PrismaticSpring() override;

  /// Returns the joint associated with this spring.
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  const PrismaticJoint<T>& joint() const;

  double nominal_position() const { return nominal_position_; }

  double stiffness() const { return stiffness_; }

  T DoCalcPotentialEnergy(
      const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
      const internal::PositionKinematicsCache<T>& pc) const override;

  T DoCalcConservativePower(
      const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

  T DoCalcNonConservativePower(
      const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const override;

 private:
  void DoCalcAndAddForceContribution(
      const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const override;

  std::unique_ptr<ForceElement<T>> DoShallowClone() const override;

  // Private constructor used by DoShallowClone(), which cannot use the public
  // constructor because it has no joint reference to hand it.
  PrismaticSpring(ModelInstanceIndex model_instance, JointIndex joint_index,
                  double nominal_position, double stiffness);

  const JointIndex joint_index_;
  double nominal_position_{0};
  double stiffness_{0};
};

}  // namespace multibody
}  // namespace drake

extern template class drake::multibody::PrismaticSpring<double>;
