#pragma once

/// @file
/// Where one element's coordinates sit in the finalized model's q and v.
///
/// A range is a start and a length, and that is all a caller needs to take its
/// element's coordinates out of a whole-model vector. What decided that start —
/// the order the forest was walked, which body ended up inboard, how many trees
/// the model split into — is the rigid tree's business, and a caller who had to
/// know it would be writing against the arrangement rather than the model.
///
/// Positions and velocities are separate types. They are not the same count: a
/// free body has seven positions and six velocities, because a quaternion needs
/// four numbers to carry three degrees of freedom. Two types that were one
/// would let a position range be passed where a velocity range was meant, and
/// the four-versus-three difference is exactly where that goes wrong.
///
/// Nothing here is arithmetic. `start()` and `size()` come out as integers
/// because that is what indexing a vector takes; the range itself does not add,
/// compare or order, because two elements' ranges have no relationship a caller
/// should be forming.

namespace orvd::multibody_model {

class MultibodyModel;

namespace internal {

/// The shared shape of both range types.
///
/// A template on a tag for the same reason the handles are: a position range
/// and a velocity range are otherwise the same two integers, and the compiler
/// would have no reason to object to one standing in for the other.
template <typename Tag>
class CoordinateRange {
   public:
    /// The index of this element's first coordinate in the whole-model vector.
    ///
    /// For an element with no coordinates this is where they would have begun.
    /// It is a valid place to take a zero-length block from, and it is not a
    /// coordinate anybody owns.
    [[nodiscard]] constexpr int start() const { return start_; }

    /// How many coordinates this element has. Zero for a weld.
    [[nodiscard]] constexpr int size() const { return size_; }

    [[nodiscard]] constexpr bool operator==(const CoordinateRange&) const =
        default;

   private:
    // Only the model states where an element's coordinates are. A range a
    // caller could build would name a span of the model's q or v that the model
    // never assigned to anything.
    friend class ::orvd::multibody_model::MultibodyModel;

    constexpr CoordinateRange(int start, int size)
        : start_(start), size_(size) {}

    int start_{0};
    int size_{0};
};

struct GeneralizedPositionTag {};
struct GeneralizedVelocityTag {};

}  // namespace internal

/// One element's generalized positions, as a span of the model's q.
using GeneralizedPositionRange =
    internal::CoordinateRange<internal::GeneralizedPositionTag>;

/// One element's generalized velocities, as a span of the model's v.
using GeneralizedVelocityRange =
    internal::CoordinateRange<internal::GeneralizedVelocityTag>;

}  // namespace orvd::multibody_model
