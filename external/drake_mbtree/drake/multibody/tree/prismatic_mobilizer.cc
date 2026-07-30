#include "drake/multibody/tree/prismatic_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
PrismaticMobilizer<T>::PrismaticMobilizer(const SpanningForest::Mobod& mobod,
                                          const Frame<T>& inboard_frame_F,
                                          const Frame<T>& outboard_frame_M,
                                          int axis)
    : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M) {
  DRAKE_DEMAND(0 <= axis && axis <= 2);
  axis_ = Eigen::Vector3d::Unit(axis);
}

template <typename T>
PrismaticMobilizer<T>::~PrismaticMobilizer() = default;

template <typename T>
std::string PrismaticMobilizer<T>::position_suffix(
    int position_index_in_mobilizer) const {
  if (position_index_in_mobilizer == 0) {
    return "x";
  }
  throw std::runtime_error("PrismaticMobilizer has only 1 position.");
}

template <typename T>
std::string PrismaticMobilizer<T>::velocity_suffix(
    int velocity_index_in_mobilizer) const {
  if (velocity_index_in_mobilizer == 0) {
    return "v";
  }
  throw std::runtime_error("PrismaticMobilizer has only 1 velocity.");
}

template <typename T>
const T& PrismaticMobilizer<T>::get_translation(
    const orvd::multibody_runtime::MultibodyStateInstance& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(0);
}

template <typename T>
const PrismaticMobilizer<T>& PrismaticMobilizer<T>::SetTranslation(
    orvd::multibody_runtime::MultibodyStateInstance* context, const T& translation) const {
  QVector<T> q = this->get_positions(*context);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = translation;
  this->SetPositions(context, q);
  return *this;
}

template <typename T>
const T& PrismaticMobilizer<T>::get_translation_rate(
    const orvd::multibody_runtime::MultibodyStateInstance& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == this->kNv);
  return v.coeffRef(0);
}

template <typename T>
const PrismaticMobilizer<T>& PrismaticMobilizer<T>::SetTranslationRate(
    orvd::multibody_runtime::MultibodyStateInstance* context, const T& translation_dot) const {
  VVector<T> v = this->get_velocities(*context);
  DRAKE_ASSERT(v.size() == this->kNv);
  v[0] = translation_dot;
  this->SetVelocities(context, v);
  return *this;
}

template <typename T, int axis>
  requires(0 <= axis && axis <= 2)
PrismaticMobilizerAxial<T, axis>::~PrismaticMobilizerAxial() = default;

template <typename T, int axis>
  requires(0 <= axis && axis <= 2)
math::RigidTransform<T>
PrismaticMobilizerAxial<T, axis>::CalcAcrossMobilizerTransform(
    const orvd::multibody_runtime::MultibodyStateInstance& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == this->kNq);
  return calc_X_FM(q.data());
}

template <typename T, int axis>
  requires(0 <= axis && axis <= 2)
SpatialVelocity<T>
PrismaticMobilizerAxial<T, axis>::CalcAcrossMobilizerSpatialVelocity(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == this->kNv);
  return calc_V_FM(nullptr, v.data());
}

template <typename T, int axis>
  requires(0 <= axis && axis <= 2)
SpatialAcceleration<T>
PrismaticMobilizerAxial<T, axis>::CalcAcrossMobilizerSpatialAcceleration(
    const orvd::multibody_runtime::MultibodyStateInstance&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == this->kNv);
  return calc_A_FM(nullptr, nullptr, vdot.data());
}

template <typename T, int axis>
  requires(0 <= axis && axis <= 2)
void PrismaticMobilizerAxial<T, axis>::ProjectSpatialForce(
    const orvd::multibody_runtime::MultibodyStateInstance&, const SpatialForce<T>& F_BMo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == this->kNv);
  calc_tau(nullptr, F_BMo_F, tau.data());
}

template <typename T>
void PrismaticMobilizer<T>::DoCalcNMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                          EigenPtr<MatrixX<T>> N) const {
  (*N)(0, 0) = 1.0;
}

template <typename T>
void PrismaticMobilizer<T>::DoCalcNplusMatrix(
    const orvd::multibody_runtime::MultibodyStateInstance&, EigenPtr<MatrixX<T>> Nplus) const {
  (*Nplus)(0, 0) = 1.0;
}

template <typename T>
void PrismaticMobilizer<T>::DoCalcNDotMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                             EigenPtr<MatrixX<T>> Ndot) const {
  (*Ndot)(0, 0) = 0.0;
}

template <typename T>
void PrismaticMobilizer<T>::DoCalcNplusDotMatrix(
    const orvd::multibody_runtime::MultibodyStateInstance&, EigenPtr<MatrixX<T>> NplusDot) const {
  (*NplusDot)(0, 0) = 0.0;
}

template <typename T>
void PrismaticMobilizer<T>::DoMapVelocityToQDot(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  *qdot = v;
}

template <typename T>
void PrismaticMobilizer<T>::DoMapQDotToVelocity(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  *v = qdot;
}

template <typename T>
void PrismaticMobilizer<T>::DoMapAccelerationToQDDot(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  *qddot = vdot;
}

template <typename T>
void PrismaticMobilizer<T>::DoMapQDDotToAcceleration(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>& qddot,
    EigenPtr<VectorX<T>> vdot) const {
  *vdot = qddot;
}

template <typename T, int axis>
  requires(0 <= axis && axis <= 2)
std::unique_ptr<BodyNode<T>> PrismaticMobilizerAxial<T, axis>::CreateBodyNode(
    const BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<BodyNodeImpl<T, PrismaticMobilizerAxial>>(
      parent_node, body, mobilizer);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

template class drake::multibody::internal::PrismaticMobilizer<double>;

#define DRAKE_DEFINE_PRISMATIC_MOBILIZER(axis)                               \
  template class ::drake::multibody::internal::PrismaticMobilizerAxial<double, \
                                                                       axis>

DRAKE_DEFINE_PRISMATIC_MOBILIZER(0);
DRAKE_DEFINE_PRISMATIC_MOBILIZER(1);
DRAKE_DEFINE_PRISMATIC_MOBILIZER(2);

#undef DRAKE_DEFINE_PRISMATIC_MOBILIZER
