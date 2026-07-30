#include "drake/multibody/tree/planar_mobilizer.h"

#include <memory>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
PlanarMobilizer<T>::~PlanarMobilizer() = default;

template <typename T>
std::unique_ptr<BodyNode<T>> PlanarMobilizer<T>::CreateBodyNode(
    const BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<BodyNodeImpl<T, PlanarMobilizer>>(parent_node, body,
                                                            mobilizer);
}

template <typename T>
std::string PlanarMobilizer<T>::position_suffix(
    int position_index_in_mobilizer) const {
  switch (position_index_in_mobilizer) {
    case 0:
      return "x";
    case 1:
      return "y";
    case 2:
      return "qz";
  }
  throw std::runtime_error("PlanarMobilizer has only 3 positions.");
}

template <typename T>
std::string PlanarMobilizer<T>::velocity_suffix(
    int velocity_index_in_mobilizer) const {
  switch (velocity_index_in_mobilizer) {
    case 0:
      return "vx";
    case 1:
      return "vy";
    case 2:
      return "wz";
  }
  throw std::runtime_error("PlanarMobilizer has only 3 velocities.");
}

template <typename T>
Vector2<T> PlanarMobilizer<T>::get_translations(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  auto q = this->get_positions(state);
  DRAKE_ASSERT(q.size() == kNq);
  return q.head(2);
}

template <typename T>
const PlanarMobilizer<T>& PlanarMobilizer<T>::set_translations(
    orvd::multibody_runtime::MultibodyStateInstance* state,
    const Eigen::Ref<const Vector2<T>>& translations) const {
  DRAKE_DEMAND(state != nullptr);
  QVector<T> q = this->get_positions(*state);
  DRAKE_ASSERT(q.size() == kNq);
  q.head(2) = translations;
  this->SetPositions(state, q);
  return *this;
}

template <typename T>
const T& PlanarMobilizer<T>::get_angle(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  auto q = this->get_positions(state);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(2);
}

template <typename T>
const PlanarMobilizer<T>& PlanarMobilizer<T>::SetAngle(
    orvd::multibody_runtime::MultibodyStateInstance* state, const T& angle) const {
  DRAKE_DEMAND(state != nullptr);
  QVector<T> q = this->get_positions(*state);
  DRAKE_ASSERT(q.size() == kNq);
  q[2] = angle;
  this->SetPositions(state, q);
  return *this;
}

template <typename T>
Vector2<T> PlanarMobilizer<T>::get_translation_rates(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  const auto& v = this->get_velocities(state);
  DRAKE_ASSERT(v.size() == kNv);
  return v.head(2);
}

template <typename T>
const PlanarMobilizer<T>& PlanarMobilizer<T>::SetTranslationRates(
    orvd::multibody_runtime::MultibodyStateInstance* state,
    const Eigen::Ref<const Vector2<T>>& v_FM_F) const {
  DRAKE_DEMAND(state != nullptr);
  VVector<T> v = this->get_velocities(*state);
  DRAKE_ASSERT(v.size() == kNv);
  v.head(2) = v_FM_F;
  this->SetVelocities(state, v);
  return *this;
}

template <typename T>
const T& PlanarMobilizer<T>::get_angular_rate(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  const auto& v = this->get_velocities(state);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(2);
}

template <typename T>
const PlanarMobilizer<T>& PlanarMobilizer<T>::SetAngularRate(
    orvd::multibody_runtime::MultibodyStateInstance* state, const T& theta_dot) const {
  DRAKE_DEMAND(state != nullptr);
  VVector<T> v = this->get_velocities(*state);
  DRAKE_ASSERT(v.size() == kNv);
  v[2] = theta_dot;
  this->SetVelocities(state, v);
  return *this;
}

template <typename T>
math::RigidTransform<T> PlanarMobilizer<T>::DoCalcAcrossMobilizerTransform(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  const auto& q = this->get_positions(state);
  DRAKE_ASSERT(q.size() == kNq);
  return calc_X_FM(q.data());
}

template <typename T>
SpatialVelocity<T> PlanarMobilizer<T>::DoCalcAcrossMobilizerSpatialVelocity(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return calc_V_FM(nullptr, v.data());
}

template <typename T>
SpatialAcceleration<T>
PlanarMobilizer<T>::DoCalcAcrossMobilizerSpatialAcceleration(
    const orvd::multibody_runtime::MultibodyStateInstance&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return calc_A_FM(nullptr, nullptr, vdot.data());
}

template <typename T>
void PlanarMobilizer<T>::DoProjectSpatialForce(const orvd::multibody_runtime::MultibodyStateInstance&,
                                             const SpatialForce<T>& F_BMo_F,
                                             Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  calc_tau(nullptr, F_BMo_F, tau.data());
}

template <typename T>
void PlanarMobilizer<T>::DoCalcNMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                       EigenPtr<MatrixX<T>> N) const {
  *N = Matrix3<T>::Identity();
}

template <typename T>
void PlanarMobilizer<T>::DoCalcNplusMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                           EigenPtr<MatrixX<T>> Nplus) const {
  *Nplus = Matrix3<T>::Identity();
}

template <typename T>
void PlanarMobilizer<T>::DoCalcNDotMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                          EigenPtr<MatrixX<T>> Ndot) const {
  *Ndot = Matrix3<T>::Zero();
}

template <typename T>
void PlanarMobilizer<T>::DoCalcNplusDotMatrix(
    const orvd::multibody_runtime::MultibodyStateInstance&, EigenPtr<MatrixX<T>> NplusDot) const {
  *NplusDot = Matrix3<T>::Zero();
}

template <typename T>
void PlanarMobilizer<T>::DoMapVelocityToQDot(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  *qdot = v;
}

template <typename T>
void PlanarMobilizer<T>::DoMapQDotToVelocity(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  *v = qdot;
}

template <typename T>
void PlanarMobilizer<T>::DoMapAccelerationToQDDot(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  *qddot = vdot;
}

template <typename T>
void PlanarMobilizer<T>::DoMapQDDotToAcceleration(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& qddot,
    EigenPtr<VectorX<T>> vdot) const {
  *vdot = qddot;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

template class drake::multibody::internal::PlanarMobilizer<double>;
