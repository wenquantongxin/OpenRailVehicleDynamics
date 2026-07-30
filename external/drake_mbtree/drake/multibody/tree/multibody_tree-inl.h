#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/body_node.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/model_instance.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
template <template <typename Scalar> class FrameType>
const FrameType<T>& MultibodyTree<T>::AddFrame(
    std::unique_ptr<FrameType<T>> frame) {
  static_assert(std::is_convertible_v<FrameType<T>*, Frame<T>*>,
                "FrameType must be a sub-class of Frame<T>.");
  if (is_finalized()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. Therefore adding more frames "
        "is not allowed. See documentation for Finalize() for details.");
  }
  if (frame == nullptr) {
    throw std::logic_error("Input frame is a nullptr.");
  }
  if (HasFrameNamed(frame->name(), frame->model_instance())) {
    throw std::logic_error(fmt::format(
        "Model instance '{}' already contains a frame named '{}'. "
        "Frame names must be unique within a given model.",
        model_instances_.get_element(frame->model_instance()).name(),
        frame->name()));
  }
  DRAKE_DEMAND(frame->model_instance().is_valid());
  const FrameIndex frame_index(num_frames());
  frame->set_parent_tree(this, frame_index);
  FrameType<T>* result = frame.get();
  frames_.Add(std::move(frame));
  return *result;
}

template <typename T>
template <template <typename Scalar> class FrameType>
const FrameType<T>& MultibodyTree<T>::AddEphemeralFrame(
    std::unique_ptr<FrameType<T>> frame) {
  frame->set_is_ephemeral(true);
  return AddFrame(std::move(frame));
}

template <typename T>
template <template <typename Scalar> class FrameType, typename... Args>
const FrameType<T>& MultibodyTree<T>::AddFrame(Args&&... args) {
  static_assert(std::is_convertible_v<FrameType<T>*, Frame<T>*>,
                "FrameType must be a sub-class of Frame<T>.");
  return AddFrame(std::make_unique<FrameType<T>>(std::forward<Args>(args)...));
}

template <typename T>
template <template <typename Scalar> class MobilizerType>
const MobilizerType<T>& MultibodyTree<T>::AddMobilizer(
    std::unique_ptr<MobilizerType<T>> mobilizer) {
  static_assert(std::is_convertible_v<MobilizerType<T>*, Mobilizer<T>*>,
                "MobilizerType must be a sub-class of mobilizer<T>.");
  if (is_finalized()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. Therefore adding more "
        "mobilizers is not allowed. See documentation for Finalize() for "
        "details.");
  }
  if (mobilizer == nullptr) {
    throw std::logic_error("Input mobilizer is a nullptr.");
  }
  // Verifies that the inboard/outboard frames provided by the user do belong
  // to this tree. This is a pathological case, but in theory nothing
  // (but this test) stops a user from adding frames to a tree1 and attempting
  // later to define mobilizers between those frames in a second tree2.
  mobilizer->inboard_frame().HasThisParentTreeOrThrow(this);   // F frame
  mobilizer->outboard_frame().HasThisParentTreeOrThrow(this);  // M frame

  // Sanity check that we are processing in the expected order.
  DRAKE_DEMAND(mobilizer->mobod().index() == num_mobilizers());

  if (!mobilizer->model_instance().is_valid()) {
    mobilizer->set_model_instance(default_model_instance());
  }

  mobilizer->set_parent_tree(this, mobilizer->mobod().index());

  MobilizerType<T>* raw_mobilizer_ptr = mobilizer.get();
  mobilizers_.push_back(std::move(mobilizer));
  return *raw_mobilizer_ptr;
}

template <typename T>
template <template <typename Scalar> class ForceElementType>
const ForceElementType<T>& MultibodyTree<T>::AddForceElement(
    std::unique_ptr<ForceElementType<T>> force_element) {
  static_assert(std::is_convertible_v<ForceElementType<T>*, ForceElement<T>*>,
                "ForceElementType<T> must be a sub-class of ForceElement<T>.");
  if (is_finalized()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. Therefore adding more force "
        "elements is not allowed. See documentation for Finalize() for "
        "details.");
  }
  if (force_element == nullptr) {
    throw std::logic_error("Input force element is a nullptr.");
  }

  // This can throw in case it was already set. Keep this line ahead of any
  // changes to this class (i.e., the push_back).
  MaybeSetUniformGravityFieldElement(force_element.get());

  const ForceElementIndex force_element_index(num_force_elements());
  DRAKE_DEMAND(force_element->model_instance().is_valid());
  force_element->set_parent_tree(this, force_element_index);

  ForceElementType<T>* raw_force_element_ptr = force_element.get();
  force_elements_.push_back(std::move(force_element));
  return *raw_force_element_ptr;
}

template <typename T>
template <template <typename Scalar> class ForceElementType, typename... Args>
const ForceElementType<T>& MultibodyTree<T>::AddForceElement(Args&&... args) {
  static_assert(std::is_base_of_v<ForceElement<T>, ForceElementType<T>>,
                "ForceElementType<T> must be a sub-class of "
                "ForceElement<T>.");
  return AddForceElement(
      std::make_unique<ForceElementType<T>>(std::forward<Args>(args)...));
}

template <typename T>
template <template <typename Scalar> class JointType>
const JointType<T>& MultibodyTree<T>::AddJoint(
    std::unique_ptr<JointType<T>> joint, bool is_ephemeral_joint) {
  static_assert(std::is_convertible_v<JointType<T>*, Joint<T>*>,
                "JointType must be a sub-class of Joint<T>.");

  if (HasJointNamed(joint->name(), joint->model_instance())) {
    throw std::logic_error(fmt::format(
        "Model instance '{}' already contains a joint named '{}'. Joint names "
        "must be unique within a given model.",
        model_instances_.get_element(joint->model_instance()).name(),
        joint->name()));
  }

  if (is_finalized()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. Therefore adding more joints "
        "is not allowed. See documentation for Finalize() for details.");
  }
  if (joint == nullptr) {
    throw std::logic_error("Input joint is a nullptr.");
  }

  if (&joint->parent_body() == &joint->child_body()) {
    throw std::logic_error(
        fmt::format("AddJoint(): joint {} would connect body {} to itself.",
                    joint->name(), joint->parent_body().name()));
  }

  DRAKE_THROW_UNLESS(joint->parent_body().has_parent_tree());
  DRAKE_THROW_UNLESS(joint->child_body().has_parent_tree());

  if (&joint->parent_body().get_parent_tree() !=
      &joint->child_body().get_parent_tree()) {
    throw std::logic_error(
        fmt::format("AddJoint(): can't add joint {} because bodies {} "
                    "and {} are from different MultibodyPlants.",
                    joint->name(), joint->parent_body().name(),
                    joint->child_body().name()));
  }

  // User joints need to be registered with the LinkJointGraph but joints added
  // during modeling are already in the graph.
  if (!is_ephemeral_joint) RegisterJointAndMaybeJointTypeInGraph(*joint.get());

  joint->set_is_ephemeral(is_ephemeral_joint);
  joint->set_parent_tree(this, joints_.next_index());
  joint->set_ordinal(joints_.num_elements());
  JointType<T>* raw_joint_ptr = joint.get();
  joints_.Add(std::move(joint));
  return *raw_joint_ptr;
}

template <typename T>
template <template <typename> class JointType, typename... Args>
const JointType<T>& MultibodyTree<T>::AddJoint(
    const std::string& name, const RigidBody<T>& parent,
    const std::optional<math::RigidTransform<double>>& X_PJp,
    const RigidBody<T>& child,
    const std::optional<math::RigidTransform<double>>& X_CJc, Args&&... args) {
  static_assert(std::is_base_of_v<Joint<T>, JointType<T>>,
                "JointType<T> must be a sub-class of Joint<T>.");

  DRAKE_THROW_UNLESS(parent.has_parent_tree());
  DRAKE_THROW_UNLESS(child.has_parent_tree());

  if (&parent.get_parent_tree() != this || &child.get_parent_tree() != this) {
    throw std::logic_error(
        fmt::format("AddJoint(): can't add joint {} between {} and {} "
                    "because one or both belong to a different "
                    "MultibodyPlant.",
                    name, parent.name(), child.name()));
  }

  // The Joint constructor promises that the Joint's model instance will be the
  // same as the child body's model instance. We'll assume that for now, and
  // then cross-check it at the bottom of this function. We need to use the same
  // model instance when creating any offset frames needed for the joint.
  const ModelInstanceIndex joint_instance = child.model_instance();
  const Frame<T>& frame_on_parent =
      this->AddOrGetJointFrame(parent, X_PJp, joint_instance, name, "parent");
  const Frame<T>& frame_on_child =
      this->AddOrGetJointFrame(child, X_CJc, joint_instance, name, "child");
  const JointType<T>& result = AddJoint(std::make_unique<JointType<T>>(
      name, frame_on_parent, frame_on_child, std::forward<Args>(args)...));
  DRAKE_DEMAND(result.model_instance() == joint_instance);
  return result;
}

// This is internal use only so doesn't need the error checking provided
// by the AddJoint() signature.
template <typename T>
template <template <typename> class JointType, typename... Args>
const JointType<T>& MultibodyTree<T>::AddEphemeralJoint(
    const std::string& name, const RigidBody<T>& parent,
    const RigidBody<T>& child, Args&&... args) {
  const JointType<T>& result =
      AddJoint(std::make_unique<JointType<T>>(name, parent.body_frame(),
                                              child.body_frame(),
                                              std::forward<Args>(args)...),
               true /*ephemeral joint*/);
  return result;
}

template <typename T>
const JointActuator<T>& MultibodyTree<T>::AddJointActuator(
    const std::string& name, const Joint<T>& joint, double effort_limit) {
  if (HasJointActuatorNamed(name, joint.model_instance())) {
    throw std::logic_error(fmt::format(
        "Model instance '{}' already contains a joint actuator named '{}'. "
        "Joint actuator names must be unique within a given model.",
        model_instances_.get_element(joint.model_instance()).name(), name));
  }

  if (is_finalized()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. Therefore adding more "
        "actuators is not allowed. See documentation for Finalize() for "
        "details.");
  }

  // Create the JointActuator before making any changes to our member fields, so
  // if the JointActuator constructor throws our state will still be valid.
  auto actuator = std::make_unique<JointActuator<T>>(name, joint, effort_limit);
  const JointActuatorIndex actuator_index(actuators_.next_index());
  actuator->set_actuator_dof_start(num_actuated_dofs_);
  num_actuated_dofs_ += joint.num_velocities();
  actuator->set_parent_tree(this, actuator_index);
  return actuators_.Add(std::move(actuator));
}

template <typename T>
ModelInstanceIndex MultibodyTree<T>::AddModelInstance(const std::string& name) {
  if (HasModelInstanceNamed(name)) {
    throw std::logic_error(
        "This model already contains a model instance named '" + name +
        "'. Model instance names must be unique within a given model.");
  }

  if (is_finalized()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. Therefore adding more model "
        "instances is not allowed. See documentation for Finalize() for "
        "details.");
  }
  const ModelInstanceIndex index = model_instances_.next_index();
  auto element = std::make_unique<internal::ModelInstance<T>>(index, name);
  element->set_parent_tree(this, index);
  model_instances_.Add(std::move(element));
  return index;
}

template <typename T>
void MultibodyTree<T>::RenameModelInstance(ModelInstanceIndex model_instance,
                                           const std::string& name) {
  const auto old_name = this->GetModelInstanceName(model_instance);
  if (old_name == name) {
    return;
  }
  if (HasModelInstanceNamed(name)) {
    throw std::logic_error(
        "This model already contains a model instance named '" + name +
        "'. Model instance names must be unique within a given model.");
  }

  if (is_finalized()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. Therefore renaming model "
        "instances is not allowed. See documentation for Finalize() for "
        "details.");
  }

  model_instances_.Rename(model_instance, name);
}

// The state access definitions that lived here are gone with their
// declarations. They existed to locate q and v inside a framework state vector
// — discrete or continuous, joined as `[q; v]`, reached through a VectorBase
// that had to be dynamic_cast to a BasicVector. The typed store holds q and v
// as themselves, so there is nothing left to locate: MultibodyTree reads them
// from the state directly and writes them through the store's own setters.

}  // namespace internal
}  // namespace multibody
}  // namespace drake
