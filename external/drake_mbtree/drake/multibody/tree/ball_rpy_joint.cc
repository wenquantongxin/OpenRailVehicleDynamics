#include "drake/multibody/tree/ball_rpy_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
BallRpyJoint<T>::~BallRpyJoint() = default;

template <typename T>
const std::string& BallRpyJoint<T>::type_name() const {
  static const never_destroyed<std::string> name{kTypeName};
  return name.access();
}

template <typename T>
std::unique_ptr<Joint<T>> BallRpyJoint<T>::DoShallowClone() const {
  return std::make_unique<BallRpyJoint<T>>(
      this->name(), this->frame_on_parent(), this->frame_on_child(),
      this->default_damping());
}

// N.B. Due to esoteric linking errors on Mac (see #9345) involving
// `MobilizerImpl`, we must place this implementation in the source file, not
// in the header file.
template <typename T>
std::unique_ptr<internal::Mobilizer<T>> BallRpyJoint<T>::MakeMobilizerForJoint(
    const internal::SpanningForest::Mobod& mobod,
    internal::MultibodyTree<T>*) const {
  const auto [inboard_frame, outboard_frame] =
      this->tree_frames(mobod.is_reversed());
  // TODO(sherm1) The mobilizer needs to be reversed, not just the frames.
  auto ballrpy_mobilizer = std::make_unique<internal::RpyBallMobilizer<T>>(
      mobod, *inboard_frame, *outboard_frame);
  ballrpy_mobilizer->set_default_position(this->default_positions());
  return ballrpy_mobilizer;
}

}  // namespace multibody
}  // namespace drake

template class drake::multibody::BallRpyJoint<double>;
