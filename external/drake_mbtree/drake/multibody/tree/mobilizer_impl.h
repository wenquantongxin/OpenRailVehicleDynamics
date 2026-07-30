#pragma once

#include <memory>
#include <optional>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_element.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {
namespace internal {

/* Base class for specific Mobilizer implementations with the number of
generalized positions and velocities resolved at compile time as template
parameters. This allows specific mobilizer implementations to only work on
fixed-size Eigen expressions therefore allowing for optimized operations on
fixed-size matrices. In addition, this layer discourages the proliferation
of dynamic-sized Eigen matrices that would otherwise lead to run-time
dynamic memory allocations.

Every concrete Mobilizer derived from MobilizerImpl must implement the
following (ideally inline) methods (some have defaults; see below). Note that
these are not virtual methods so we have to document them here in a comment
rather than as declarations in the header file. The code won't compile if
any mobilizer fails to implement the non-defaulted methods. These are const
member functions (rather than static members) so are permitted to depend on
mobilizer-specific data members, though in many cases they don't require any
such data.

@note The coordinate pointers q and v are guaranteed to point to the kNq or kNv
state variables for the particular mobilizer. They are only 8-byte aligned so be
careful when interpreting them as Eigen vectors for computation purposes.

  // Computes every element of X_FM(q).
  RigidTransform<T> calc_X_FM(const T* q) const;

  // Given current q and an X_FM that was initialized to the identity transform
  // and then possibly updated by this same mobilizer (meaning it has the
  // right structure), update to X_FM(q) by filling in only the
  // potentially-changed elements. For example, a revolute mobilizer about
  // one of the frame axes will update only the four sine & cosine entries.
  // A prismatic mobilizer along the Z axis updates only the z shift element.
  void update_X_FM(const T* q, RigidTransform<T>* X_FM) const;

  // Compose X_AM = X_AF ⋅ X_FM, optimized for the known structure of X_FM.
  // For example, a revolute mobilizer has only 4 significant entries in X_FM
  // out of 12, and a prismatic along Z has only 1.
  RigidTransform<T> post_multiply_by_X_FM(const RigidTransform<T>& X_AF,
                                          const RigidTransform<T>& X_FM) const;

  // Compose X_FB = X_FM ⋅ X_MB, optimized for the known structure of X_FM.
  RigidTransform<T> pre_multiply_by_X_FM(const RigidTransform<T>& X_FM,
                                         const RigidTransform<T>& X_MB) const;

  // Returns v_F = R_FM ⋅ v_M (re-express vector).
  Vector3<T> apply_R_FM(const RotationMatrix<T>& R_FM,
                        const Vector3<T>& v_M) const;

  // Returns V_FM_F = H_FM_F(q)⋅v.
  SpatialVelocity<T> calc_V_FM(const T* q,
                               const T* v) const;

  // Returns A_FM_F = H_FM_F(q)⋅vdot + Hdot_FM_F(q,v)⋅v.
  SpatialAcceleration<T> calc_A_FM(const T* q,
                                   const T* v,
                                   const T* vdot) const;

  // Returns tau = H_FM_Fᵀ(q)⋅F_BMo_F.
  void calc_tau(const T* q, const SpatialForce<T>& F_BMo_F, T* tau) const;

  // TODO(sherm1) More to come (see #22253)

MobilizerImpl also provides a number of size specific methods to retrieve
multibody quantities of interest from caching structures. These are common
to all mobilizer implementations and therefore they live in this class.
Users should not need to interact with this class directly unless they need
to implement a custom Mobilizer class. */
template <typename T, int compile_time_num_positions,
          int compile_time_num_velocities>
class MobilizerImpl : public Mobilizer<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilizerImpl);

  using ScalarType = T;

  // Handy enum to grant specific implementations compile time sizes.
  // static constexpr int i = 42; discouraged.  See answer in:
  // http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {
    kNq = compile_time_num_positions,
    kNv = compile_time_num_velocities,
    kNx = compile_time_num_positions + compile_time_num_velocities
  };
  template <typename U>
  using QVector = Eigen::Matrix<U, kNq, 1>;
  template <typename U>
  using VVector = Eigen::Matrix<U, kNv, 1>;
  template <typename U>
  using HMatrix = Eigen::Matrix<U, 6, kNv>;

  // As with Mobilizer this the only constructor available for this base class.
  // The minimum amount of information that we need to define a mobilizer is
  // provided here. Subclasses of MobilizerImpl are therefore forced to
  // provide this information in their respective constructors.
  MobilizerImpl(const SpanningForest::Mobod& mobod,
                const Frame<T>& inboard_frame, const Frame<T>& outboard_frame)
      : Mobilizer<T>(mobod, inboard_frame, outboard_frame) {}

  ~MobilizerImpl() override;

  // Writes this mobilizer's pose coordinates from the supplied orientation and
  // translation, committing its complete q segment once.
  bool DoSetPosePair(
      Eigen::Quaternion<T> q_FM, const Vector3<T>& p_FM,
      orvd::multibody_runtime::MultibodyStateInstance* state) const final;

  bool DoSetSpatialVelocity(
      const SpatialVelocity<T>& V_FM,
      orvd::multibody_runtime::MultibodyStateInstance* state) const final;

  // Writes this mobilizer's default positions into a caller-owned q vector.
  // MultibodyTree commits the complete model q only after every mobilizer has
  // contributed.
  void WriteDefaultPositions(EigenPtr<VectorX<T>> q) const final;

  // Sets the default position used by subsequent model default-state writes.
  void set_default_position(const Eigen::Ref<const QVector<double>>& position) {
    default_position_.emplace(position);
  }

  // N.B. no default implementations possible for calc_X_FM() and update_X_FM()
  // here. However, a minimal implementation for update_X_FM() in a concrete
  // mobilizer is just *X_FM = calc_X_FM(q).

  // Returns the composition X_AM = X_AF ⋅ X_FM. The default implementation
  // treats X_FM as fully general and performs this in 63 flops. Mobilizers
  // that know more about the structure of their X_FM should override.
  math::RigidTransform<T> post_multiply_by_X_FM(
      const math::RigidTransform<T>& X_AF,
      const math::RigidTransform<T>& X_FM) const {
    const math::RigidTransform<T> X_AM = X_AF * X_FM;
    return X_AM;
  }

  // Returns the composition X_FB = X_FM ⋅ X_MB. The default implementation
  // treats X_FM as fully general and performs this in 63 flops. Mobilizers
  // that know more about the structure of their X_FM should override.
  math::RigidTransform<T> pre_multiply_by_X_FM(
      const math::RigidTransform<T>& X_FM,
      const math::RigidTransform<T>& X_MB) const {
    const math::RigidTransform<T> X_FB = X_FM * X_MB;
    return X_FB;
  }

  // N.B. no default implementations possible for calc_X_FM() and update_X_FM()
  // here. However, a minimal implementation for update_X_FM() in a concrete
  // mobilizer is just *X_FM = calc_X_FM(q).

  // Returns v_F = R_FM ⋅ v_M (re-express). The default implementation
  // treats R_FM as fully general and performs this in 15 flops. Mobilizers
  // that know more about the structure of their R_FM should override.
  Vector3<T> apply_R_FM(const math::RotationMatrix<T>& R_FM,
                        const Vector3<T>& v_M) const {
    return R_FM * v_M;
  }

 protected:
  // Returns the zero configuration for the mobilizer.
  virtual QVector<double> get_zero_position() const {
    return QVector<double>::Zero();
  }

  // A mobilizer is free to take its time finding a reasonable approximation
  // to this pose. 6-dof mobilizers are required to represent it as close to
  // bit-exactly as possible. In particular, QuaternionFloatingMobilizer must
  // represent this perfectly to guarantee consistent pose representation
  // pre- and post-finalize for floating base bodies.
  virtual std::optional<QVector<T>> DoPoseToPositions(
      const Eigen::Quaternion<T> orientation,
      const Vector3<T>& translation) const;

  // A mobilizer is free to take its time finding a reasonable approximation
  // to this spatial velocity. 6 dof mobilizers are required to represent it
  // as close to bit-exactly as possible.
  virtual std::optional<VVector<T>> DoSpatialVelocityToVelocities(
      const SpatialVelocity<T>& velocity) const;

  // Returns the default configuration for the mobilizer.  The default
  // configuration is the configuration used to populate the context in
  // MultibodyPlant::SetDefaultContext().
  QVector<double> get_default_position() const {
    return default_position_.value_or(get_zero_position());
  }

  // @name    Helper methods to reach this mobilizer's own q and v
  //@{
  // Returns this mobilizer's segment of the generalized positions.
  Eigen::VectorBlock<const VectorX<T>, kNq> get_positions(
      const orvd::multibody_runtime::MultibodyStateInstance& state) const {
    DRAKE_ASSERT(this->has_parent_tree());
    return this->get_parent_tree().template get_position_segment<kNq>(
        state, this->position_start_in_q());
  }

  // Returns this mobilizer's segment of the generalized velocities.
  //
  // The offset is `velocity_start_in_v()`, with nothing added to it. Upstream
  // reached v through a concatenated `[q; v]`, so its offset had to be written
  // `num_qs_in_state() + velocity_start_in_v()`; q and v are separate vectors
  // now and that addition has no meaning.
  Eigen::VectorBlock<const VectorX<T>, kNv> get_velocities(
      const orvd::multibody_runtime::MultibodyStateInstance& state) const {
    DRAKE_ASSERT(this->has_parent_tree());
    return this->get_parent_tree().template get_velocity_segment<kNv>(
        state, this->velocity_start_in_v());
  }

  // Writes this mobilizer's whole segment of q, as one transaction.
  //
  // Whole segment, not part of one. A caller changing several parts of this
  // mobilizer's configuration — a floating body's rotation and its translation,
  // say — must assemble the complete segment and call this once. Calling it
  // twice would advance the position version twice for one change and would
  // leave the state, in between, holding a configuration nobody asked for.
  void SetPositions(orvd::multibody_runtime::MultibodyStateInstance* state,
                    const Eigen::Ref<const QVector<T>>& q) const {
    DRAKE_ASSERT(this->has_parent_tree());
    this->get_parent_tree().SetPositionSegment(state,
                                               this->position_start_in_q(), q);
  }

  // Writes this mobilizer's whole segment of v, as one transaction.
  void SetVelocities(orvd::multibody_runtime::MultibodyStateInstance* state,
                     const Eigen::Ref<const VVector<T>>& v) const {
    DRAKE_ASSERT(this->has_parent_tree());
    this->get_parent_tree().SetVelocitySegment(state,
                                               this->velocity_start_in_v(), v);
  }
  //@}

 private:

  std::optional<QVector<double>> default_position_{};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
