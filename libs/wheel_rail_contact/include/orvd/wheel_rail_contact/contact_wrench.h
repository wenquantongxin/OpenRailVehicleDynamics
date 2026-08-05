#pragma once

#include <Eigen/Core>

// The force and moment one contact patch exerts, and the two operations that
// move such a thing around.
//
// A wrench is only meaningful together with the point it is reduced about and
// the frame its components are written in. Both are stated here in the type's
// name and in every operation's signature, because the two mistakes this module
// exists to prevent are adding two wrenches reduced about different points and
// rotating one without saying which basis it started in.
//
// The rail side is a construction, not a measurement. The track is not a body
// in the vehicle's multibody tree — it is a kinematic parameterisation with no
// state and no inertia — so nothing computes a force on it. What the contact
// evaluation can guarantee is that the two halves of the interaction are built
// from the same numbers at the same point, and that is what the paired form
// below is for: it emits the wheel-side wrench and its exact negation about the
// same point in the same frame, from one set of doubles.
//
// That pairing makes an action-reaction check over the pair itself vacuous, and
// this header says so rather than letting a later gate believe otherwise. What
// the pair does buy is that any consumer which transports, rotates or projects
// the two sides differently is caught: the residual of a sum that should cancel
// term by term is then no longer at the level of arithmetic rounding. The check
// with real content is the power balance, and it needs a relative velocity this
// module does not have.

namespace orvd::wheel_rail_contact {

// A force and a moment, reduced about one point. The point itself is not stored
// here: an operation that needs it takes it explicitly, so that a wrench can
// never silently carry a stale reduction point.
struct SpatialWrench {
    Eigen::Vector3d moment_newton_meters{Eigen::Vector3d::Zero()};
    Eigen::Vector3d force_newtons{Eigen::Vector3d::Zero()};
};

// Moves the reduction point of a wrench without changing the frame.
//
// The moment picks up the cross product of the vector running from the new
// point to the old one with the force. Reversing that order is the single most
// common way to get this wrong, and it is invisible whenever the two points
// coincide, which they do in every degenerate fixture.
[[nodiscard]] SpatialWrench TransportWrench(
    const SpatialWrench& wrench, const Eigen::Vector3d& from_point_meters,
    const Eigen::Vector3d& to_point_meters);

// Rewrites a wrench's components in another basis, about the same point.
//
// Rotation and reduction-point translation do not commute. Every conversion in
// this module is therefore one or the other, never both at once.
[[nodiscard]] SpatialWrench RotateWrench(const SpatialWrench& wrench,
                                         const Eigen::Matrix3d& rotation);

// One patch's interaction, with both halves reduced about the contact point and
// written in the same frame.
//
// `rail_on_wheel` is what the contact exerts on the wheel; `wheel_on_rail` is
// its exact negation. Emitting the negation here rather than leaving each
// consumer to form its own is the whole point: two consumers that each negate
// separately will eventually negate about different points.
struct PairedContactWrench {
    Eigen::Vector3d contact_point_meters{Eigen::Vector3d::Zero()};
    SpatialWrench rail_on_wheel;
    SpatialWrench wheel_on_rail;
};

// Builds the pair from the wheel-side force and moment.
//
// The moment is the one that accompanies the force at the contact point — for a
// contact model that suppresses the direct spin torque it is zero, and passing
// zero is then stating that, not omitting it.
[[nodiscard]] PairedContactWrench MakePairedContactWrench(
    const Eigen::Vector3d& contact_point_meters,
    const Eigen::Vector3d& force_on_wheel_newtons,
    const Eigen::Vector3d& moment_on_wheel_newton_meters);

}  // namespace orvd::wheel_rail_contact
