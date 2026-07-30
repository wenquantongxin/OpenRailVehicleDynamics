#include "drake/multibody/tree/multibody_element.h"

#include <utility>
#include "orvd/multibody_runtime/multibody_state_instance.h"

namespace drake {
namespace multibody {

using internal::MultibodyTreeSystem;

template <typename T>
MultibodyElement<T>::~MultibodyElement() {
  // Clear the tree to help fail-fast in case of accidental use-after-free of
  // this MultibodyElement by end users who accidentally kept around a stale
  // point to us.
  parent_tree_ = nullptr;
}

template <typename T>
void MultibodyElement<T>::AssignParameterSlots(
    orvd::rigid_multibody_tree::internal::MultibodyParameterSlotAllocator*
        allocator) {
  DRAKE_DEMAND(allocator != nullptr);
  DoAssignParameterSlots(allocator);
}

template <typename T>
void MultibodyElement<T>::WriteDefaultParameters(
    orvd::multibody_runtime::MultibodyStateInstance* state) const {
  DRAKE_DEMAND(state != nullptr);
  DoWriteDefaultParameters(state);
}

template <typename T>
MultibodyElement<T>::MultibodyElement() {}

template <typename T>
MultibodyElement<T>::MultibodyElement(ModelInstanceIndex model_instance)
    : model_instance_(model_instance) {}

template <typename T>
MultibodyElement<T>::MultibodyElement(ModelInstanceIndex model_instance,
                                      int64_t index)
    : MultibodyElement(model_instance) {
  index_ = index;
}

template <typename T>
void MultibodyElement<T>::DoAssignParameterSlots(
    orvd::rigid_multibody_tree::internal::MultibodyParameterSlotAllocator*) {}

template <typename T>
void MultibodyElement<T>::DoWriteDefaultParameters(
    orvd::multibody_runtime::MultibodyStateInstance*) const {}

template <typename T>
void MultibodyElement<T>::ThrowNoParentTree() const {
  throw std::logic_error(
      "This multibody element was not added to a MultibodyTree.");
}

template <typename T>
void MultibodyElement<T>::HasThisParentTreeOrThrow(
    const internal::MultibodyTree<T>* tree) const {
  DRAKE_ASSERT(tree != nullptr);
  if (parent_tree_ != tree) {
    throw std::logic_error(
        "This multibody element does not belong to the "
        "supplied MultibodyTree.");
  }
}

}  // namespace multibody
}  // namespace drake

template class drake::multibody::MultibodyElement<double>;
