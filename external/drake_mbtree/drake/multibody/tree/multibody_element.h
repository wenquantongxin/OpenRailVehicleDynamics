#pragma once

#include <set>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "orvd/multibody_runtime/multibody_state_instance.h"
#include "orvd/rigid_multibody_tree/multibody_parameter_slot_allocator.h"

namespace drake {
namespace multibody {

/// A class representing an element (subcomponent) of a MultibodyPlant or
/// (internally) a MultibodyTree. Examples of multibody elements are bodies,
/// joints, force elements, and actuators. After a Finalize() call, multibody
/// elements get assigned a type-specific index that uniquely identifies them.
/// By convention, every subclass of MultibodyElement provides an `index()`
/// member function that returns the assigned index, e.g.,
///
/// @code
/// /** Returns this element's unique index. */
/// BodyIndex index() const { return this->template index_impl<BodyIndex>(); }
/// @endcode
///
/// Some multibody elements are added during Finalize() and are not part of
/// the user-specified model. These are called "ephemeral" elements and can
/// be identified using the `is_ephemeral()` function here. Examples include
///   - free joints added to connect lone bodies or free-floating trees
///     to World
///   - fixed offset frames added when joints are modeled by mobilizers
///   - all mobilizers.
template <typename T>
class MultibodyElement {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyElement);

  virtual ~MultibodyElement();

  /// Returns the ModelInstanceIndex of the model instance to which this
  /// element belongs.
  ModelInstanceIndex model_instance() const { return model_instance_; }

  /// Returns `true` if this %MultibodyElement was added during Finalize()
  /// rather than something a user added. (See class comments.)
  bool is_ephemeral() const { return is_ephemeral_; }

  /// (Internal use only) Sets the `is_ephemeral` flag to the indicated value.
  /// The default if this is never called is `false`. Any element that is added
  /// during Finalize() should set this flag to `true`.
  void set_is_ephemeral(bool is_ephemeral) { is_ephemeral_ = is_ephemeral; }

 protected:
  /// Default constructor made protected so that sub-classes can still declare
  /// their default constructors if they need to.
  MultibodyElement();

  /// Constructor which allows specifying a model instance.
  explicit MultibodyElement(ModelInstanceIndex model_instance);

  /// Both the model instance and element index are specified.
  explicit MultibodyElement(ModelInstanceIndex model_instance, int64_t index);

  /// Returns this element's unique index.
  template <typename ElementIndexType>
  ElementIndexType index_impl() const {
    DRAKE_ASSERT(index_ >= 0);
    return ElementIndexType{index_};
  }

  /// Returns this element's unique ordinal.
  /// @note The int64_t default is present for backwards compatibility but
  /// you should not use it. Instead, define a ThingOrdinal specialization of
  /// TypeSafeIndex for any element Thing that has a meaningful ordinal. Then
  /// use that type explicitly in Thing's public `ordinal()` method.
  template <typename ElementOrdinalType = int64_t>
  ElementOrdinalType ordinal_impl() const {
    DRAKE_ASSERT(ordinal_ >= 0);
    return ElementOrdinalType{ordinal_};
  }

  /// Returns a constant reference to the parent MultibodyTree that owns this
  /// element.
  /// @pre has_parent_tree is true.
  const internal::MultibodyTree<T>& get_parent_tree() const {
    if constexpr (kDrakeAssertIsArmed) {
      if (parent_tree_ == nullptr) {
        ThrowNoParentTree();
      }
    }
    return *parent_tree_;
  }

  /// Rejects a typed state that was not built from this element's finalized
  /// parent tree. Equal dimensions are not sufficient: two models can have the
  /// same-sized state while assigning different meanings to every slot.
  void ValidateStateInstance(
      const orvd::multibody_runtime::MultibodyStateInstance& state) const {
    if (parent_tree_ == nullptr) {
      ThrowNoParentTree();
    }
    parent_tree_->ValidateStateInstance(state);
  }

  /// Returns a constant reference to the parent MultibodyTreeSystem that
  /// owns the parent MultibodyTree that owns this element.
  /// @throws std::exception if has_parent_tree() is false.
  const internal::MultibodyTreeSystem<T>& GetParentTreeSystem() const {
    if (parent_tree_ == nullptr) {
      ThrowNoParentTree();
    }
    return get_parent_tree().tree_system();
  }

  /// (Internal use only) Gives MultibodyElement-derived objects the opportunity
  /// to set data members that depend on topology and coordinate assignments
  /// having been finalized. This is invoked at the end of
  /// MultibodyTree::Finalize(). NVI to pure virtual method DoSetTopology().
  void SetTopology() { DoSetTopology(); }

  /// Implementation of the NVI SetTopology(). For advanced use only for
  /// developers implementing new MultibodyTree components.
  virtual void DoSetTopology() = 0;

  /// Implementation of the NVI AssignParameterSlots(). MultibodyElement-derived
  /// objects may override to claim the slot their parameters live in.
  virtual void DoAssignParameterSlots(
      orvd::rigid_multibody_tree::internal::MultibodyParameterSlotAllocator*);

  /// Implementation of the NVI WriteDefaultParameters().
  /// MultibodyElement-derived objects may override to write their model default
  /// values into a new state.
  virtual void DoWriteDefaultParameters(
      orvd::multibody_runtime::MultibodyStateInstance*) const;

  // Discrete state and generic per-element cache entries were declared here.
  // Both chains were dead: no landed element overrode either hook, and no
  // landed call site invoked one. Cache entries are now named members of the
  // adapter's workspace, which is what lets the slot layout be frozen by a type
  // rather than by a registry that merely declines to grow.

  /// Returns true if this multibody element has a parent tree, otherwise false.
  bool has_parent_tree() const { return parent_tree_ != nullptr; }

 private:
  // MultibodyTree<T> is a natural friend of MultibodyElement objects and
  // therefore it can set the owning parent tree and unique index in that tree.
  friend class internal::MultibodyTree<T>;
  // Give unit tests access to the tree.
  friend class MultibodyElementTester;

  /// Claims this element's typed parameter slot during the tree's one
  /// finalize-time traversal. Kept private so no caller can reassign slots
  /// after the layout has been frozen.
  void AssignParameterSlots(
      orvd::rigid_multibody_tree::internal::MultibodyParameterSlotAllocator*
          allocator);

  /// Writes this element's model defaults during model-aware state creation.
  /// Kept private so an element cannot be used to write its defaults into a
  /// same-sized state that belongs to another tree.
  void WriteDefaultParameters(
      orvd::multibody_runtime::MultibodyStateInstance* state) const;

  // Index and ordinal are given the same numerical value here. The ordinal
  // can be overridden with set_ordinal().
  void set_parent_tree(const internal::MultibodyTree<T>* tree, int64_t index) {
    index_ = index;
    ordinal_ = index;
    parent_tree_ = tree;
  }

  void set_ordinal(int64_t ordinal) { ordinal_ = ordinal; }

  void set_model_instance(ModelInstanceIndex model_instance) {
    model_instance_ = model_instance;
  }

  // @throws std::exception that this element is not in a MultibodyTree.
  [[noreturn]] void ThrowNoParentTree() const;

  // Checks whether this MultibodyElement belongs to the provided
  // MultibodyTree `tree` and throws an exception if not.
  // @throws std::exception if `this` element is not in the given `tree`.
  void HasThisParentTreeOrThrow(const internal::MultibodyTree<T>* tree) const;

  const internal::MultibodyTree<T>* parent_tree_{nullptr};

  // The default index value is *invalid*. This must be set to a valid index
  // value before the element is released to the wild.
  int64_t index_{-1};

  // Keeps track of the index into contiguous containers that have an entry
  // for each of a concrete MultibodyElement type (Joint, RigidBody, etc.)
  // Ordinals should be updated upon element removal so that the ordinals always
  // form a contiguous sequence from 0 to N-1, where N is the number of a
  // particular element type. Default ordinal value is *invalid*. Concrete
  // MultibodyElements may choose to not expose this ordinal if not needed (e.g.
  // if MultibodyPlant does not expose any port that has an entry per concrete
  // MultibodyElement type.) This must be set to a valid ordinal value before
  // the element is released to the wild.
  int64_t ordinal_{-1};

  // The default model instance id is *invalid*. This must be set to a
  // valid index value before the element is released to the wild.
  ModelInstanceIndex model_instance_;

  bool is_ephemeral_{false};
};

}  // namespace multibody
}  // namespace drake

extern template class drake::multibody::MultibodyElement<double>;
