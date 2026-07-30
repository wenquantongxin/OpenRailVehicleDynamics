#include "drake/multibody/tree/frame.h"

#include "drake/common/nice_type_name.h"
#include "drake/multibody/tree/rigid_body.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {
namespace internal {

std::string DeprecateWhenEmptyName(std::string name, std::string_view type) {
  if (name.empty()) {
    throw std::runtime_error(fmt::format(
        "The name parameter to the {} constructor is required.", type));
  }
  return name;
}

}  // namespace internal

template <typename T>
Frame<T>::~Frame() = default;

template <typename T>
ScopedName Frame<T>::scoped_name() const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  return ScopedName(
      this->get_parent_tree().GetModelInstanceName(this->model_instance()),
      name_);
}

// TODO(Mitiguy) The calculation below assumes "this" frame is attached to a
//  rigid body (not a soft body). Modify if soft bodies are possible.
template <typename T>
Vector3<T> Frame<T>::CalcAngularVelocity(
    const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context, const Frame<T>& measured_in_frame,
    const Frame<T>& expressed_in_frame) const {
  const Frame<T>& frame_M = measured_in_frame;
  const Frame<T>& frame_E = expressed_in_frame;
  const Vector3<T>& w_WF_W = EvalAngularVelocityInWorld(context);
  const Vector3<T>& w_WM_W = frame_M.EvalAngularVelocityInWorld(context);
  const Vector3<T> w_MF_W = w_WF_W - w_WM_W;

  // If the expressed-in frame E is the world, no need to re-express results.
  if (frame_E.is_world_frame()) return w_MF_W;

  const math::RotationMatrix<T> R_WE =
      frame_E.CalcRotationMatrixInWorld(context);
  const math::RotationMatrix<T> R_EW = R_WE.inverse();
  const Vector3<T> w_MF_E = R_EW * w_MF_W;
  return w_MF_E;
}

// TODO(Mitiguy) The calculation below assumes "this" frame is attached to a
//  rigid body (not a soft body). Modify if soft bodies are possible.
template <typename T>
SpatialVelocity<T> Frame<T>::CalcSpatialVelocityInWorld(
    const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context) const {
  const math::RotationMatrix<T>& R_WB =
      body().EvalPoseInWorld(context).rotation();
  const Vector3<T> p_BF_B = GetFixedPoseInBodyFrame().translation();
  const Vector3<T> p_BF_W = R_WB * p_BF_B;
  const SpatialVelocity<T>& V_WB = body().EvalSpatialVelocityInWorld(context);
  const SpatialVelocity<T> V_WF = V_WB.Shift(p_BF_W);
  return V_WF;
}

template <typename T>
SpatialVelocity<T> Frame<T>::CalcSpatialVelocity(
    const orvd::rigid_multibody_tree::internal::RigidMultibodyTreeEvaluationContext& context, const Frame<T>& frame_M,
    const Frame<T>& frame_E) const {
  const math::RotationMatrix<T> R_WM =
      frame_M.CalcRotationMatrixInWorld(context);
  const Vector3<T> p_MF_M = this->CalcPose(context, frame_M).translation();
  const Vector3<T> p_MF_W = R_WM * p_MF_M;
  const SpatialVelocity<T> V_WM_W = frame_M.CalcSpatialVelocityInWorld(context);
  const SpatialVelocity<T> V_WF_W = this->CalcSpatialVelocityInWorld(context);
  // V_MF is calculated from a rearranged "composition" of spatial velocities.
  // The angular velocity addition theorem  ω_WF = ω_WM + ω_MF
  // rearranges to                          ω_MF = ω_WF - ω_WM
  // The translational velocity formula for a point Fo moving on frame M is
  // v_WFo = v_WMo + ω_WM x p_MoFo + v_MFo   which rearranges to
  // v_MFo = V_WFo - (v_WMo + ω_WM x p_MoFo).
  const SpatialVelocity<T> V_MF_W = V_WF_W - V_WM_W.Shift(p_MF_W);

  // If the expressed-in frame E is the world, no need to re-express results.
  if (frame_E.is_world_frame()) return V_MF_W;

  // Otherwise re-express results from world frame W to frame E.
  const math::RotationMatrix<T> R_WE =
      frame_E.CalcRotationMatrixInWorld(context);
  return R_WE.inverse() * V_MF_W;
}

template <typename T>
std::unique_ptr<Frame<T>> Frame<T>::ShallowClone() const {
  std::unique_ptr<Frame<T>> result = DoShallowClone();
  DRAKE_THROW_UNLESS(result != nullptr);
  return result;
}

template <typename T>
std::unique_ptr<Frame<T>> Frame<T>::DoShallowClone() const {
  throw std::logic_error(fmt::format("{} failed to override DoShallowClone()",
                                     NiceTypeName::Get(*this)));
}

}  // namespace multibody
}  // namespace drake

// The double class instantiation emits all out-of-line Frame members.
template class drake::multibody::Frame<double>;
