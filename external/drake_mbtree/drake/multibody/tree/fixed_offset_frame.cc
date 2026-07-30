#include "drake/multibody/tree/fixed_offset_frame.h"

#include <exception>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/rigid_body.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(
    const std::string& name, const Frame<T>& P,
    const math::RigidTransform<double>& X_PF,
    std::optional<ModelInstanceIndex> model_instance)
    : Frame<T>(name, P.body(), model_instance.value_or(P.model_instance())),
      parent_frame_(P),
      X_PF_(X_PF) {}

template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(const std::string& name,
                                      const RigidBody<T>& B,
                                      const math::RigidTransform<double>& X_BF)
    : Frame<T>(name, B), parent_frame_(B.body_frame()), X_PF_(X_BF) {}

template <typename T>
FixedOffsetFrame<T>::~FixedOffsetFrame() = default;

template <typename T>
void FixedOffsetFrame<T>::SetPoseInParentFrame(
    orvd::multibody_runtime::MultibodyStateInstance* state,
    const math::RigidTransform<T>& X_PF) const {
  orvd::multibody_runtime::FixedFramePoseParameters parameters;
  parameters.R_PF = X_PF.rotation().matrix();
  parameters.p_PoFo_P = X_PF.translation();
  state->set_fixed_frame_pose_parameters(pose_parameter_slot_, parameters);
}

template <typename T>
math::RigidTransform<T> FixedOffsetFrame<T>::GetPoseInParentFrame(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  const orvd::multibody_runtime::FixedFramePoseParameters& parameters =
      state.fixed_frame_pose_parameters(pose_parameter_slot_);
  return math::RigidTransform<T>(
      math::RotationMatrix<T>(parameters.R_PF), parameters.p_PoFo_P);
}

template <typename T>
std::unique_ptr<Frame<T>> FixedOffsetFrame<T>::DoShallowClone() const {
  auto new_frame = std::make_unique<FixedOffsetFrame<T>>(
      this->name(), parent_frame_, X_PF_, this->model_instance());
  new_frame->set_is_ephemeral(this->is_ephemeral());
  return new_frame;
}

template <typename T>
math::RigidTransform<T> FixedOffsetFrame<T>::DoCalcPoseInBodyFrame(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  // X_BF = X_BP * X_PF
  const orvd::multibody_runtime::FixedFramePoseParameters& stored =
      state.fixed_frame_pose_parameters(pose_parameter_slot_);
  const math::RigidTransform<T> X_PF(math::RotationMatrix<T>(stored.R_PF),
                                     stored.p_PoFo_P);
  return parent_frame_.CalcOffsetPoseInBody(state, X_PF);
}

template <typename T>
math::RotationMatrix<T> FixedOffsetFrame<T>::DoCalcRotationMatrixInBodyFrame(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  // R_BF = R_BP * R_PF
  const orvd::multibody_runtime::FixedFramePoseParameters& stored =
      state.fixed_frame_pose_parameters(pose_parameter_slot_);
  return parent_frame_.CalcOffsetRotationMatrixInBody(
      state, math::RotationMatrix<T>(stored.R_PF));
}

}  // namespace multibody
}  // namespace drake

template class drake::multibody::FixedOffsetFrame<double>;
