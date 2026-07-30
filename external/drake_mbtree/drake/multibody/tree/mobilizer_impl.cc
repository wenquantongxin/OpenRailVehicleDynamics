#include "drake/multibody/tree/mobilizer_impl.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T, int nq, int nv>
MobilizerImpl<T, nq, nv>::~MobilizerImpl() = default;

template <typename T, int nq, int nv>
void MobilizerImpl<T, nq, nv>::SetZeroState(const systems::Context<T>&,
                                            systems::State<T>* state) const {
  get_mutable_positions(state) = get_zero_position();
  get_mutable_velocities(state).setZero();
}

template <typename T, int nq, int nv>
bool MobilizerImpl<T, nq, nv>::SetPosePair(const systems::Context<T>&,
                                           const Eigen::Quaternion<T> q_FM,
                                           const Vector3<T>& p_FM,
                                           systems::State<T>* state) const {
  const std::optional<QVector<T>> q = DoPoseToPositions(q_FM, p_FM);
  if (q.has_value()) get_mutable_positions(&*state) = *q;
  return q.has_value();
}

template <typename T, int nq, int nv>
bool MobilizerImpl<T, nq, nv>::SetSpatialVelocity(
    const systems::Context<T>&, const SpatialVelocity<T>& V_FM,
    systems::State<T>* state) const {
  const std::optional<VVector<T>> v = DoSpatialVelocityToVelocities(V_FM);
  if (v.has_value()) get_mutable_velocities(&*state) = *v;
  return v.has_value();
}

template <typename T, int nq, int nv>
void MobilizerImpl<T, nq, nv>::set_default_state(
    const systems::Context<T>&, systems::State<T>* state) const {
  get_mutable_positions(&*state) = get_default_position();
  get_mutable_velocities(&*state).setZero();
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
