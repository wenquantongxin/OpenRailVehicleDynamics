#include "drake/multibody/tree/mobilizer_impl.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "drake/common/unused.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T, int nq, int nv>
MobilizerImpl<T, nq, nv>::~MobilizerImpl() = default;

template <typename T, int nq, int nv>
bool MobilizerImpl<T, nq, nv>::SetPosePair(
    const Eigen::Quaternion<T> q_FM, const Vector3<T>& p_FM,
    orvd::multibody_runtime::MultibodyStateInstance* state) const {
  // The pose becomes this mobilizer's whole q, committed once. A mobilizer that
  // cannot represent the pose writes nothing at all rather than writing the part
  // it can.
  const std::optional<QVector<T>> q = DoPoseToPositions(q_FM, p_FM);
  if (q.has_value()) this->SetPositions(state, *q);
  return q.has_value();
}

template <typename T, int nq, int nv>
bool MobilizerImpl<T, nq, nv>::SetSpatialVelocity(
    const SpatialVelocity<T>& V_FM,
    orvd::multibody_runtime::MultibodyStateInstance* state) const {
  const std::optional<VVector<T>> v = DoSpatialVelocityToVelocities(V_FM);
  if (v.has_value()) this->SetVelocities(state, *v);
  return v.has_value();
}

template <typename T, int nq, int nv>
void MobilizerImpl<T, nq, nv>::WriteDefaultPositions(
    EigenPtr<VectorX<T>> q) const {
  // Written into the caller's vector, not committed. A model fills every
  // mobilizer's default segment and then commits q once; committing here would
  // advance the position version once per mobilizer and copy the whole of q as
  // many times as there are mobilizers.
  DRAKE_DEMAND(q != nullptr);
  q->template segment<nq>(this->position_start_in_q()) = get_default_position();
}

template <typename T, int nq, int nv>
auto MobilizerImpl<T, nq, nv>::DoPoseToPositions(
    const Eigen::Quaternion<T> orientation, const Vector3<T>& translation) const
    -> std::optional<QVector<T>> {
  unused(orientation, translation);
  return {};
}

template <typename T, int nq, int nv>
auto MobilizerImpl<T, nq, nv>::DoSpatialVelocityToVelocities(
    const SpatialVelocity<T>& velocity) const -> std::optional<VVector<T>> {
  unused(velocity);
  return {};
}

// Macro used to explicitly instantiate implementations on all sizes needed.
#define EXPLICITLY_INSTANTIATE_IMPLS(T)  \
  template class MobilizerImpl<T, 0, 0>; \
  template class MobilizerImpl<T, 1, 1>; \
  template class MobilizerImpl<T, 2, 2>; \
  template class MobilizerImpl<T, 3, 3>; \
  template class MobilizerImpl<T, 6, 6>; \
  template class MobilizerImpl<T, 7, 6>;

// double is the only scalar this tree instantiates.
EXPLICITLY_INSTANTIATE_IMPLS(double);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
