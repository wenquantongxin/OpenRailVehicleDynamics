#include "drake/multibody/tree/screw_mobilizer.h"

#include <cmath>
#include <limits>

#include "drake/multibody/tree/body_node_impl.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
ScrewMobilizer<T>::~ScrewMobilizer() = default;

template <typename T>
std::unique_ptr<BodyNode<T>> ScrewMobilizer<T>::CreateBodyNode(
    const BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<BodyNodeImpl<T, ScrewMobilizer>>(parent_node, body,
                                                           mobilizer);
}

template <typename T>
std::string ScrewMobilizer<T>::position_suffix(
    int position_index_in_mobilizer) const {
  if (position_index_in_mobilizer == 0) {
    return "q";
  }
  throw std::runtime_error("ScrewMobilizer has only 1 position.");
}

template <typename T>
std::string ScrewMobilizer<T>::velocity_suffix(
    int velocity_index_in_mobilizer) const {
  if (velocity_index_in_mobilizer == 0) {
    return "w";
  }
  throw std::runtime_error("ScrewMobilizer has only 1 velocity.");
}

template <typename T>
double ScrewMobilizer<T>::screw_pitch() const {
  return screw_pitch_;
}

template <typename T>
T ScrewMobilizer<T>::get_translation(const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  auto q = this->get_positions(state);
  DRAKE_ASSERT(q.size() == kNq);
  return GetScrewTranslationFromRotation(q[0], screw_pitch_);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::SetTranslation(
    orvd::multibody_runtime::MultibodyStateInstance* state, const T& translation) const {
  DRAKE_DEMAND(state != nullptr);
  const double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
  using std::abs;
  DRAKE_THROW_UNLESS(abs(screw_pitch_) > kEpsilon ||
                     abs(translation) < kEpsilon);
  QVector<T> q = this->get_positions(*state);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = GetScrewRotationFromTranslation(translation, screw_pitch_);
  this->SetPositions(state, q);
  return *this;
}

template <typename T>
const T& ScrewMobilizer<T>::get_angle(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  auto q = this->get_positions(state);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(0);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::SetAngle(
    orvd::multibody_runtime::MultibodyStateInstance* state, const T& angle) const {
  DRAKE_DEMAND(state != nullptr);
  QVector<T> q = this->get_positions(*state);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = angle;
  this->SetPositions(state, q);
  return *this;
}

template <typename T>
T ScrewMobilizer<T>::get_translation_rate(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  auto v = this->get_velocities(state);
  DRAKE_ASSERT(v.size() == kNv);
  return GetScrewTranslationFromRotation(v[0], screw_pitch_);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::SetTranslationRate(
    orvd::multibody_runtime::MultibodyStateInstance* state, const T& vz) const {
  DRAKE_DEMAND(state != nullptr);
  const double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
  using std::abs;
  DRAKE_THROW_UNLESS(abs(screw_pitch_) > kEpsilon || abs(vz) < kEpsilon);

  VVector<T> v = this->get_velocities(*state);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = GetScrewRotationFromTranslation(vz, screw_pitch_);
  this->SetVelocities(state, v);
  return *this;
}

template <typename T>
const T& ScrewMobilizer<T>::get_angular_rate(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  auto v = this->get_velocities(state);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(0);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::SetAngularRate(
    orvd::multibody_runtime::MultibodyStateInstance* state, const T& theta_dot) const {
  DRAKE_DEMAND(state != nullptr);
  VVector<T> v = this->get_velocities(*state);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = theta_dot;
  this->SetVelocities(state, v);
  return *this;
}

template <typename T>
math::RigidTransform<T> ScrewMobilizer<T>::DoCalcAcrossMobilizerTransform(
    const orvd::multibody_runtime::MultibodyStateInstance& state) const {
  const auto& q = this->get_positions(state);
  DRAKE_ASSERT(q.size() == kNq);
  return calc_X_FM(q.data());
}

template <typename T>
SpatialVelocity<T> ScrewMobilizer<T>::DoCalcAcrossMobilizerSpatialVelocity(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return calc_V_FM(nullptr, v.data());
}

template <typename T>
SpatialAcceleration<T>
ScrewMobilizer<T>::DoCalcAcrossMobilizerSpatialAcceleration(
    const orvd::multibody_runtime::MultibodyStateInstance&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return calc_A_FM(nullptr, nullptr, vdot.data());
}

template <typename T>
void ScrewMobilizer<T>::DoProjectSpatialForce(const orvd::multibody_runtime::MultibodyStateInstance&,
                                            const SpatialForce<T>& F_BMo_F,
                                            Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  calc_tau(nullptr, F_BMo_F, tau.data());
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                      EigenPtr<MatrixX<T>> N) const {
  *N = Eigen::Matrix<T, 1, 1>::Identity();
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNplusMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                          EigenPtr<MatrixX<T>> Nplus) const {
  *Nplus = Eigen::Matrix<T, 1, 1>::Identity();
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNDotMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                         EigenPtr<MatrixX<T>> Ndot) const {
  (*Ndot)(0, 0) = 0.0;
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNplusDotMatrix(
    const orvd::multibody_runtime::MultibodyStateInstance&, EigenPtr<MatrixX<T>> NplusDot) const {
  (*NplusDot)(0, 0) = 0.0;
}

template <typename T>
void ScrewMobilizer<T>::DoMapVelocityToQDot(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  *qdot = v;
}

template <typename T>
void ScrewMobilizer<T>::DoMapQDotToVelocity(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  *v = qdot;
}

template <typename T>
void ScrewMobilizer<T>::DoMapAccelerationToQDDot(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  *qddot = vdot;
}

template <typename T>
void ScrewMobilizer<T>::DoMapQDDotToAcceleration(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& qddot,
    EigenPtr<VectorX<T>> vdot) const {
  *vdot = qddot;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

template class drake::multibody::internal::ScrewMobilizer<double>;
