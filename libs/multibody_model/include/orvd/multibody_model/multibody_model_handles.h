#pragma once

/// @file
/// What the model hands back when you add something to it.
///
/// A handle names one element of one model. It carries the identity of the model
/// it came from, so a handle from a different model is refused at the entry point
/// that receives it rather than silently indexing into whatever happens to sit at
/// that position — which is what a bare integer would do, and it would be right
/// often enough to be trusted.
///
/// Deliberately not convertible to an integer, not arithmetic, not ordered. The
/// only things a caller does with a handle are hold it and give it back. Exposing
/// the ordinal would invite `handle + 1`, or a loop over a range the caller
/// assembled itself, and both would be reaching past the model into a layout that
/// belongs to it.
///
/// The upstream index types stay out of this header entirely. `MobodIndex`,
/// `LinkOrdinal`, `q_start` and their relatives describe how the rigid tree
/// arranges itself, which is exactly the thing a caller must not have to know.

#include <cstdint>

namespace orvd::multibody_model {
namespace internal {

struct HandleAccess;

/// The identity of one model, for catching handles that came from another.
///
/// A counter rather than an address: two models can occupy the same address one
/// after the other, and a handle held across that would pass an address check
/// while naming an element of a model that no longer exists.
using ModelIdentity = std::uint64_t;

/// The shared shape of every handle.
///
/// A template so that a `RigidBodyHandle` cannot be passed where a `FrameHandle`
/// is expected. They would otherwise be the same two integers, and the compiler
/// would have no reason to object — while the model, receiving a frame ordinal
/// as a body ordinal, would have no way to tell either.
template <typename Tag>
class ElementHandle {
   public:
    /// A handle that names nothing.
    ///
    /// Present so that a caller can declare one before it has something to put
    /// in it. Every entry point refuses it: it belongs to no model, so it cannot
    /// be the model's own element.
    constexpr ElementHandle() = default;

    [[nodiscard]] constexpr bool is_valid() const { return model_ != 0; }

    [[nodiscard]] constexpr bool operator==(const ElementHandle&) const =
        default;

   private:
    // The one way in. Friendship does not extend to a class's nested classes,
    // and the model's implementation is one, so the access point is a type of
    // its own rather than the model itself.
    friend struct HandleAccess;

    constexpr ElementHandle(ModelIdentity model, int ordinal)
        : model_(model), ordinal_(ordinal) {}

    ModelIdentity model_{0};
    int ordinal_{-1};
};

struct RigidBodyTag {};
struct FrameTag {};
struct JointTag {};

/// Reads and builds handles, for the model and nobody else.
///
/// In `internal` and undocumented outside this file's purpose: a caller who
/// found it could manufacture a handle naming any ordinal of any model, which is
/// precisely what the handle exists to prevent.
struct HandleAccess {
    template <typename Handle>
    static constexpr ModelIdentity model_of(const Handle& handle) {
        return handle.model_;
    }
    template <typename Handle>
    static constexpr int ordinal_of(const Handle& handle) {
        return handle.ordinal_;
    }
    template <typename Handle>
    static constexpr Handle Make(ModelIdentity model, int ordinal) {
        return Handle(model, ordinal);
    }
};

}  // namespace internal

/// One rigid body of one model.
using RigidBodyHandle = internal::ElementHandle<internal::RigidBodyTag>;

/// One frame of one model: the world frame, a body's own frame, or a fixed frame
/// somebody added to a body.
///
/// One type for all three rather than a variant of endpoint kinds. A joint
/// attaches to a frame; which of the three that frame happens to be is not a
/// distinction the joint makes, and making the caller state it would be asking
/// for information the model already has.
using FrameHandle = internal::ElementHandle<internal::FrameTag>;

/// One joint of one model, whether revolute, prismatic or weld.
using JointHandle = internal::ElementHandle<internal::JointTag>;

}  // namespace orvd::multibody_model
