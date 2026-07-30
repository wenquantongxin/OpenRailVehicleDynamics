#include "drake/multibody/tree/revolute_spring.h"

#include <utility>

#include "drake/multibody/tree/multibody_tree.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/rigid_multibody_tree/rigid_multibody_tree_evaluation_context.h"

namespace drake {
namespace multibody {

template <typename T>
RevoluteSpring<T>::RevoluteSpring(const RevoluteJoint<T>& joint,
                                  double nominal_angle, double stiffness)
    : RevoluteSpring(joint.model_instance(), joint.index(), nominal_angle,
                     stiffness) {}

template <typename T>
RevoluteSpring<T>::RevoluteSpring(ModelInstanceIndex model_instance,
                                  JointIndex joint_index, double nominal_angle,
                                  double stiffness)
    : ForceElement<T>(model_instance),
      joint_index_(joint_index),
      nominal_angle_(nominal_angle),
      stiffness_(stiffness) {
  DRAKE_THROW_UNLESS(stiffness >= 0);
}

template <typename T>
RevoluteSpring<T>::~RevoluteSpring() = default;

template <typename T>
const RevoluteJoint<T>& RevoluteSpring<T>::joint() const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  const RevoluteJoint<T>* joint = dynamic_cast<const RevoluteJoint<T>*>(
      &this->get_parent_tree().get_joint(joint_index_));
  DRAKE_DEMAND(joint != nullptr);
  return *joint;
}

template <typename T>
void RevoluteSpring<T>::DoCalcAndAddForceContribution(
    const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
    const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&,
    MultibodyForces<T>* forces) const {
  const T delta = this->GetNominalAngle(context) - joint().get_angle(context);
  const T torque = this->GetStiffness(context) * delta;
  joint().AddInTorque(context, torque, forces);
}

template <typename T>
T RevoluteSpring<T>::CalcPotentialEnergy(
    const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
    const internal::PositionKinematicsCache<T>&) const {
  const T delta = this->GetNominalAngle(context) - joint().get_angle(context);

  return 0.5 * this->GetStiffness(context) * delta * delta;
}

template <typename T>
T RevoluteSpring<T>::CalcConservativePower(
    const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context,
    const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&) const {
  // Since the potential energy is:
  //   V = 1/2⋅k⋅(θ₀-θ)²
  // The conservative power is defined as:
  //  Pc = -d(V)/dt = -[k⋅(θ₀-θ)⋅-dθ/dt] = k⋅(θ₀-θ)⋅dθ/dt
  // being positive when the potential energy decreases.
  const T delta = this->GetNominalAngle(context) - joint().get_angle(context);
  const T theta_dot = joint().get_angular_rate(context);
  return this->GetStiffness(context) * delta * theta_dot;
}

template <typename T>
T RevoluteSpring<T>::CalcNonConservativePower(
    const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext&, const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&) const {
  // Purely conservative spring
  return 0;
}

template <typename T>
std::unique_ptr<ForceElement<T>> RevoluteSpring<T>::DoShallowClone() const {
  // N.B. We use the private constructor since joint() requires a MbT pointer.
  return std::unique_ptr<ForceElement<T>>(
      new RevoluteSpring<T>(this->model_instance(), joint_index_,
                            default_nominal_angle(), default_stiffness()));
}

}  // namespace multibody
}  // namespace drake

template class drake::multibody::RevoluteSpring<double>;
