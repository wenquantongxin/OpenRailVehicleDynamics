#include "drake/multibody/tree/rpy_floating_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
RpyFloatingJoint<T>::~RpyFloatingJoint() = default;

template <typename T>
const std::string& RpyFloatingJoint<T>::type_name() const {
  static const never_destroyed<std::string> name{kTypeName};
  return name.access();
}

template <typename T>
std::unique_ptr<Joint<T>> RpyFloatingJoint<T>::DoShallowClone() const {
  return std::make_unique<RpyFloatingJoint<T>>(
      this->name(), this->frame_on_parent(), this->frame_on_child(),
      this->default_angular_damping(), this->default_translational_damping());
}

// N.B. Due to esoteric linking errors on Mac (see #9345) involving
// `MobilizerImpl`, we must place this implementation in the source file, not
// in the header file.
template <typename T>
std::unique_ptr<internal::Mobilizer<T>>
RpyFloatingJoint<T>::MakeMobilizerForJoint(
    const internal::SpanningForest::Mobod& mobod,
    internal::MultibodyTree<T>*) const {
  const auto [inboard_frame, outboard_frame] =
      this->tree_frames(mobod.is_reversed());
  // TODO(sherm1) The mobilizer needs to be reversed, not just the frames.
  auto rpy_floating_mobilizer =
      std::make_unique<internal::RpyFloatingMobilizer<T>>(mobod, *inboard_frame,
                                                          *outboard_frame);
  rpy_floating_mobilizer->set_default_position(this->default_positions());
  return rpy_floating_mobilizer;
}

}  // namespace multibody
}  // namespace drake

template class drake::multibody::RpyFloatingJoint<double>;
