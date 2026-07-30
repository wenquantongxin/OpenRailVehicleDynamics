#include "drake/multibody/tree/curvilinear_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/multibody/tree/multibody_tree.h"

using drake::never_destroyed;

namespace drake {
namespace multibody {

template <typename T>
CurvilinearJoint<T>::CurvilinearJoint(
    const std::string& name, const Frame<T>& frame_on_parent,
    const Frame<T>& frame_on_child,
    const trajectories::PiecewiseConstantCurvatureTrajectory<double>
        curvilinear_path,
    double pos_lower_limit, double pos_upper_limit, double damping)
    : Joint<T>(
          name, frame_on_parent, frame_on_child,
          VectorX<double>::Constant(1, damping),
          VectorX<double>::Constant(1, pos_lower_limit),
          VectorX<double>::Constant(1, pos_upper_limit),
          VectorX<double>::Constant(1,
                                    -std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1, std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1,
                                    -std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1,
                                    std::numeric_limits<double>::infinity())),
      curvilinear_path_(curvilinear_path) {
  if (damping < 0) {
    throw std::logic_error(
        "CurvilinearJoint joint damping must be nonnegative.");
  }
}

template <typename T>
CurvilinearJoint<T>::~CurvilinearJoint() = default;

template <typename T>
const std::string& CurvilinearJoint<T>::type_name() const {
  static const never_destroyed<std::string> name{kTypeName};
  return name.access();
}

template <typename T>
std::unique_ptr<Joint<T>> CurvilinearJoint<T>::DoShallowClone() const {
  return std::make_unique<CurvilinearJoint<T>>(
      this->name(), this->frame_on_parent(), this->frame_on_child(),
      curvilinear_path_, this->position_lower_limit(),
      this->position_upper_limit(), this->default_damping());
}

template <typename T>
std::unique_ptr<internal::Mobilizer<T>>
CurvilinearJoint<T>::MakeMobilizerForJoint(
    const internal::SpanningForest::Mobod& mobod,
    internal::MultibodyTree<T>*) const {
  const auto [inboard_frame, outboard_frame] =
      this->tree_frames(mobod.is_reversed());
  // TODO(sherm1) The mobilizer needs to be reversed, not just the frames.
  auto curvilinear_mobilizer =
      std::make_unique<internal::CurvilinearMobilizer<T>>(
          mobod, *inboard_frame, *outboard_frame, curvilinear_path_);
  curvilinear_mobilizer->set_default_position(this->default_positions());
  return curvilinear_mobilizer;
}

}  // namespace multibody
}  // namespace drake

template class drake::multibody::CurvilinearJoint<double>;
