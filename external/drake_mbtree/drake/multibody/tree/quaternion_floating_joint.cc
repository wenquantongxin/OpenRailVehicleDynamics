#include "drake/multibody/tree/quaternion_floating_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
QuaternionFloatingJoint<T>::~QuaternionFloatingJoint() = default;

template <typename T>
std::unique_ptr<Joint<T>> QuaternionFloatingJoint<T>::DoShallowClone() const {
  return std::make_unique<QuaternionFloatingJoint<T>>(
      this->name(), this->frame_on_parent(), this->frame_on_child(),
      this->default_angular_damping(), this->default_translational_damping());
}

// N.B. Due to esoteric linking errors on Mac (see #9345) involving
// `MobilizerImpl`, we must place this implementation in the source file, not
// in the header file.
template <typename T>
std::unique_ptr<internal::Mobilizer<T>>
QuaternionFloatingJoint<T>::MakeMobilizerForJoint(
    const internal::SpanningForest::Mobod& mobod,
    internal::MultibodyTree<T>*) const {
  const auto [inboard_frame, outboard_frame] =
      this->tree_frames(mobod.is_reversed());
  // TODO(sherm1) The mobilizer needs to be reversed, not just the frames.
  auto quaternion_floating_mobilizer =
      std::make_unique<internal::QuaternionFloatingMobilizer<T>>(
          mobod, *inboard_frame, *outboard_frame);
  quaternion_floating_mobilizer->set_default_position(
      this->default_positions());
  return quaternion_floating_mobilizer;
}

template <typename T>
void QuaternionFloatingJoint<T>::DoAddInOneForce(const systems::Context<T>&,
                                                 int, const T&,
                                                 MultibodyForces<T>*) const {
  throw std::logic_error(
      "QuaternionFloating joints do not allow applying forces to individual "
      "degrees of freedom.");
}

template <typename T>
void QuaternionFloatingJoint<T>::DoAddInDamping(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  Eigen::Ref<VectorX<T>> t_BMo_F =
      get_mobilizer().get_mutable_generalized_forces_from_array(
          &forces->mutable_generalized_forces());
  const Vector3<T>& w_FM = get_angular_velocity(context);
  const Vector3<T>& v_FM = get_translational_velocity(context);
  const T& angular_damping = this->GetDampingVector(context)[0];
  const T& translational_damping = this->GetDampingVector(context)[3];
  t_BMo_F.template head<3>() -= angular_damping * w_FM;
  t_BMo_F.template tail<3>() -= translational_damping * v_FM;
}

}  // namespace multibody
}  // namespace drake

template class drake::multibody::QuaternionFloatingJoint<double>;
