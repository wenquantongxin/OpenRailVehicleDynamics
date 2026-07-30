#include "drake/multibody/tree/weld_mobilizer.h"

#include <memory>

#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
WeldMobilizer<T>::~WeldMobilizer() = default;

template <typename T>
std::unique_ptr<BodyNode<T>> WeldMobilizer<T>::CreateBodyNode(
    const BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<BodyNodeImpl<T, WeldMobilizer>>(parent_node, body,
                                                          mobilizer);
}

template <typename T>
math::RigidTransform<T> WeldMobilizer<T>::CalcAcrossMobilizerTransform(
    const orvd::multibody_runtime::MultibodyStateInstance&) const {
  return math::RigidTransform<T>();  // Identity
}

template <typename T>
SpatialVelocity<T> WeldMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>&) const {
  return SpatialVelocity<T>::Zero();
}

template <typename T>
SpatialAcceleration<T> WeldMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>&) const {
  return SpatialAcceleration<T>::Zero();
}

template <typename T>
void WeldMobilizer<T>::ProjectSpatialForce(const orvd::multibody_runtime::MultibodyStateInstance&,
                                           const SpatialForce<T>&,
                                           Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
}

template <typename T>
void WeldMobilizer<T>::DoCalcNMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                     EigenPtr<MatrixX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoCalcNplusMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                         EigenPtr<MatrixX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoCalcNDotMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                        EigenPtr<MatrixX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoCalcNplusDotMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                            EigenPtr<MatrixX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoMapVelocityToQDot(const orvd::multibody_runtime::MultibodyStateInstance&,
                                           const Eigen::Ref<const VectorX<T>>&,
                                           EigenPtr<VectorX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoMapQDotToVelocity(const orvd::multibody_runtime::MultibodyStateInstance&,
                                           const Eigen::Ref<const VectorX<T>>&,
                                           EigenPtr<VectorX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoMapAccelerationToQDDot(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>&,
    EigenPtr<VectorX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoMapQDDotToAcceleration(
    const orvd::multibody_runtime::MultibodyStateInstance&, const Eigen::Ref<const VectorX<T>>&,
    EigenPtr<VectorX<T>>) const {}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

template class drake::multibody::internal::WeldMobilizer<double>;
