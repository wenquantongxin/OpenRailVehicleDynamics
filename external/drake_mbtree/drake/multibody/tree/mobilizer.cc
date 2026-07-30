#include "drake/multibody/tree/mobilizer.h"

#include <sstream>
#include <string>
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
Mobilizer<T>::~Mobilizer() = default;

template <typename T>
std::pair<Eigen::Quaternion<T>, Vector3<T>> Mobilizer<T>::GetPosePair(
    const orvd::multibody_runtime::MultibodyStateInstance& context) const {
  const math::RigidTransform<T> X_FM = CalcAcrossMobilizerTransform(context);
  return std::pair(X_FM.rotation().ToQuaternion(), X_FM.translation());
}

template <typename T>
void Mobilizer<T>::DoMapAccelerationToQDDot(const orvd::multibody_runtime::MultibodyStateInstance&,
                                            const Eigen::Ref<const VectorX<T>>&,
                                            EigenPtr<VectorX<T>>) const {
  // TODO(Mitiguy) remove this base class implementation when
  //  Mobilizer::DoMapAccelerationToQDDot() is changed to a pure virtual
  //  function that requires override.
  const std::string error_message = fmt::format(
      "The function {}() has not been implemented for this "
      "mobilizer.",
      __func__);
  throw std::logic_error(error_message);
}

template <typename T>
void Mobilizer<T>::DoMapQDDotToAcceleration(const orvd::multibody_runtime::MultibodyStateInstance&,
                                            const Eigen::Ref<const VectorX<T>>&,
                                            EigenPtr<VectorX<T>>) const {
  // TODO(Mitiguy) remove this base class implementation when
  //  Mobilizer::DoMapQDDotToAcceleration() is changed to a pure virtual
  //  function that requires override.
  const std::string error_message = fmt::format(
      "The function {}() has not been implemented for this "
      "mobilizer.",
      __func__);
  throw std::logic_error(error_message);
}

template <typename T>
void Mobilizer<T>::DoCalcNDotMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                    EigenPtr<MatrixX<T>>) const {
  // TODO(Mitiguy) remove this function when Mobilizer::CalcNDotMatrix()
  //  is changed to a pure virtual function that requires override.
  const std::string error_message = fmt::format(
      "The function {}() has not been implemented for this mobilizer.",
      __func__);
  throw std::logic_error(error_message);
}

template <typename T>
void Mobilizer<T>::DoCalcNplusDotMatrix(const orvd::multibody_runtime::MultibodyStateInstance&,
                                        EigenPtr<MatrixX<T>>) const {
  // TODO(Mitiguy) remove this function when Mobilizer::CalcNplusDotMatrix()
  //  is changed to a pure virtual function that requires override.
  const std::string error_message = fmt::format(
      "The function {}() has not been implemented for this mobilizer.",
      __func__);
  throw std::logic_error(error_message);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

template class drake::multibody::internal::Mobilizer<double>;
