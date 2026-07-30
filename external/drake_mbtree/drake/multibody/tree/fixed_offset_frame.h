#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/frame.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {

// Forward declarations.
template <class T>
class RigidBodyFrame;
template <class T>
class RigidBody;

/// %FixedOffsetFrame represents a material frame F whose pose is fixed with
/// respect to a _parent_ material frame P. The pose offset is given by a
/// spatial transform `X_PF`, which is constant after construction. For
/// instance, we could rigidly attach a frame F to move with a rigid body B at a
/// fixed pose `X_BF`, where B is the RigidBodyFrame associated with body B.
/// Thus, the World frame pose `X_WF` of a %FixedOffsetFrame F depends only on
/// the World frame pose `X_WP` of its parent P, and the constant pose `X_PF`,
/// with `X_WF=X_WP*X_PF`.
///
/// For more information about spatial transforms, see
/// @ref multibody_spatial_pose. <!-- https://drake.mit.edu/doxygen_cxx/
///                                   group__multibody__spatial__pose.html -->
template <typename T>
class FixedOffsetFrame final : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedOffsetFrame);

  /// Creates a material Frame F whose pose is fixed with respect to its
  /// parent material Frame P. The pose is given by a spatial transform `X_PF`;
  /// see class documentation for more information.
  ///
  /// @param[in] name
  ///   The name of this frame. Cannot be empty.
  /// @param[in] P
  ///   The frame to which this frame is attached with a fixed pose.
  /// @param[in] X_PF
  ///   The _default_ transform giving the pose of F in P, therefore only the
  ///   value (as a RigidTransform<double>) is provided.
  /// @param[in] model_instance
  ///   The model instance to which this frame belongs to. If unspecified, will
  ///   use P.model_instance().
  FixedOffsetFrame(const std::string& name, const Frame<T>& P,
                   const math::RigidTransform<double>& X_PF,
                   std::optional<ModelInstanceIndex> model_instance = {});

  /// Creates a material Frame F whose pose is fixed with respect to the
  /// RigidBodyFrame B of the given RigidBody, which serves as F's parent frame.
  /// The pose is given by a RigidTransform `X_BF`; see class documentation
  /// for more information.
  ///
  /// @param[in] name  The name of this frame. Cannot be empty.
  /// @param[in] bodyB The body whose RigidBodyFrame B is to be F's parent
  ///                  frame.
  /// @param[in] X_BF  The transform giving the pose of F in B.
  FixedOffsetFrame(const std::string& name, const RigidBody<T>& bodyB,
                   const math::RigidTransform<double>& X_BF);

  ~FixedOffsetFrame() override;

  /// Sets the pose of `this` frame F in its parent frame P.
  /// @param[in,out] state of the finalized multibody tree associated with this
  /// frame.
  /// @param[in] X_PF Rigid transform that characterizes `this` frame F's pose
  ///   (orientation and position) in its parent frame P.
  void SetPoseInParentFrame(orvd::multibody_runtime::MultibodyStateInstance* state,
                            const math::RigidTransform<T>& X_PF) const;

  /// Returns the rigid transform X_PF that characterizes `this` frame F's pose
  /// in its parent frame P.
  /// @param[in] state of the finalized multibody tree associated with this
  /// frame.
  math::RigidTransform<T> GetPoseInParentFrame(
      const orvd::multibody_runtime::MultibodyStateInstance& state) const;

  /// @returns The default fixed pose in the body frame.
  math::RigidTransform<T> GetFixedPoseInBodyFrame() const override {
    // X_BF = X_BP * X_PF
    return parent_frame_.GetFixedOffsetPoseInBody(X_PF_);
  }

  /// @returns The default rotation matrix of this fixed pose in the body frame.
  math::RotationMatrix<T> GetFixedRotationMatrixInBodyFrame() const override {
    // R_BF = R_BP * R_PF
    const math::RotationMatrix<double>& R_PF = X_PF_.rotation();
    return parent_frame_.GetFixedRotationMatrixInBody(R_PF);
  }

  /// @returns The parent frame to which this frame is attached.
  const Frame<T>& parent_frame() const { return parent_frame_; }

 protected:
  std::unique_ptr<Frame<T>> DoShallowClone() const override;

  math::RigidTransform<T> DoCalcPoseInBodyFrame(
      const orvd::multibody_runtime::MultibodyStateInstance& state) const override;

  math::RotationMatrix<T> DoCalcRotationMatrixInBodyFrame(
      const orvd::multibody_runtime::MultibodyStateInstance& state) const override;

 private:
  // Implementation for Frame::DoAssignFrameParameterSlots().
  void DoAssignFrameParameterSlots(
      orvd::rigid_multibody_tree::internal::MultibodyParameterSlotAllocator*
          allocator) final {
    pose_parameter_slot_ = allocator->AllocateFixedFramePoseSlot();
  }

  // Implementation for Frame::DoWriteDefaultFrameParameters().
  //
  // The pose is stored as a rotation and a translation rather than as twelve
  // numbers in row-major order. The flattened form was a transport format: it
  // needed a comment to say which nine entries were the rotation, and the
  // rotation could not be validated without first knowing that.
  void DoWriteDefaultFrameParameters(
      orvd::multibody_runtime::MultibodyStateInstance* state) const final {
    orvd::multibody_runtime::FixedFramePoseParameters parameters;
    parameters.R_PF = X_PF_.rotation().matrix();
    parameters.p_PoFo_P = X_PF_.translation();
    state->set_fixed_frame_pose_parameters(pose_parameter_slot_, parameters);
  }

  // The frame to which this frame is attached.
  const Frame<T>& parent_frame_;

  // Spatial transform giving the fixed pose of this frame F measured in the
  // parent frame P.
  const math::RigidTransform<double> X_PF_;

  // Where this frame's pose in its parent lives in the state, assigned when
  // the model was finalized.
  int pose_parameter_slot_{
      orvd::rigid_multibody_tree::internal::kUnassignedParameterSlot};
};

}  // namespace multibody
}  // namespace drake

extern template class drake::multibody::FixedOffsetFrame<double>;
