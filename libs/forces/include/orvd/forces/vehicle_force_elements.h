#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "orvd/multibody_model/multibody_model_handles.h"

// The force elements a rail vehicle's suspension is made of, named by what they
// are constitutively rather than by the vendor or the vehicle they came from.
//
// Every element connects two named frames, but the constitutive frame and the
// wrench application points belong to the element family. The translational,
// series and clipped families below use a reference end and its balancing
// support moment. The half-angle bushing instead uses named A/C ends and applies
// both wrenches at one instantaneous common midpoint. Keeping those contracts
// on their own types prevents a shared endpoint vocabulary from silently
// turning into one shared — and wrong — force law.

namespace orvd::forces {

// Which axis of the reference frame an element acts along, for the families
// that act along exactly one. Stating it as a choice rather than as a vector of
// which two entries happen to be zero keeps the number of internal state
// components a property of the family instead of a property of the data.
enum class ForceElementAxis { kLongitudinal, kLateral, kVertical };

// One end of a force element.  The frame is its sole identity.  The multibody
// model resolves the frame origin's carrier body and body-fixed position from
// its current context when the wrench is emitted, so there is no second copy of
// that identity for a mutable vehicle record to contradict.
struct ForceElementEnd {
    multibody_model::FrameHandle frame;
};

// A spring and a viscous damper in parallel, acting between two frames, with a
// diagonal stiffness and damping in the reference frame.
//
// The nominal force is not here. It is a per-instance startup quantity that a
// resolved start-up state supplies to a context, not part of what the vehicle
// is, so the plan reads it from the context each evaluation.
struct TranslationalSpringDamper {
    std::string name;
    ForceElementEnd reference_end;
    ForceElementEnd opposite_end;
    Eigen::Vector3d stiffness_newtons_per_meter{Eigen::Vector3d::Zero()};
    Eigen::Vector3d damping_newton_seconds_per_meter{Eigen::Vector3d::Zero()};
};

// A linear spring and viscous damper acting as a pure couple about the
// reference frame's own first axis.
//
// The roll coordinate is the (2,1) entry of the rotation of the opposite frame
// in the reference frame — Eigen's zero-based indexing, so the third row and
// second column — which is the sine of the roll angle for a pure roll and
// agrees with the roll angle to first order. The rate is the first component of
// the relative angular velocity in the reference frame. The moment this
// produces is already a moment in physical space: it is not a generalized force
// conjugate to an Euler angle and does not pass through any attitude map.
struct RollSpringDamperCouple {
    std::string name;
    ForceElementEnd reference_end;
    ForceElementEnd opposite_end;
    double stiffness_newton_meters_per_radian{0.0};
    double damping_newton_meter_seconds_per_radian{0.0};
};

// A spring in series with a viscous damper, acting along one axis.
//
// The force is not an algebraic function of the relative motion: the spring and
// the damper share it while their deflections differ, which makes the force
// itself a state,
//
//     dF/dt = stiffness * relative velocity - (stiffness / damping) * F
//
// with time constant damping / stiffness. This is the only family in this
// header that carries continuous state, and it is the reason the system's state
// has a third block at all.
struct SeriesSpringViscousDamper {
    std::string name;
    ForceElementEnd reference_end;
    ForceElementEnd opposite_end;
    ForceElementAxis axis{ForceElementAxis::kLateral};
    double series_stiffness_newtons_per_meter{0.0};
    double series_damping_newton_seconds_per_meter{0.0};
};

// A damper whose force law is a piecewise linear odd function of the relative
// velocity along one axis, saturating outside the stated range.
//
// The curve is stated by its breakpoints for non-negative velocity, starting at
// the origin; the negative half is its odd reflection and is not written down,
// so a table cannot be asymmetric by transcription. Beyond the last breakpoint
// the force holds at that breakpoint's value.  For GZ18 the final declared
// segment is flat, so this continuation preserves the source curve rather than
// inventing a new slope outside it.
struct SaturatedPiecewiseLinearDamperPoint {
    double relative_velocity_meters_per_second{0.0};
    double force_newtons{0.0};
};

struct SaturatedPiecewiseLinearDamper {
    std::string name;
    ForceElementEnd reference_end;
    ForceElementEnd opposite_end;
    ForceElementAxis axis{ForceElementAxis::kLateral};
    // Strictly ascending in velocity, with finite non-negative force and the
    // first point at zero velocity and zero force.
    std::vector<SaturatedPiecewiseLinearDamperPoint> curve;
};

// A six-component linear bushing with the execution semantics frozen by the
// IRW reference path.
//
// Translation is stated in the half-angle frame between A and C. Rotation is
// represented by space-fixed XYZ roll-pitch-yaw coordinates of C in A; their
// rates are generalized speeds, not the components of angular velocity. The
// resulting force and physical torque are applied as equal-and-opposite
// wrenches at the instantaneous world-space midpoint between the two frame
// origins. In particular, this family does not carry the reference-end support
// moment used by the translational families above.
struct HalfAngleMidpointRollPitchYawBushing {
    std::string name;
    ForceElementEnd frame_a_end;
    ForceElementEnd frame_c_end;
    Eigen::Vector3d rotational_stiffness_newton_meters_per_radian{
        Eigen::Vector3d::Zero()};
    Eigen::Vector3d rotational_damping_newton_meter_seconds_per_radian{
        Eigen::Vector3d::Zero()};
    Eigen::Vector3d translational_stiffness_newtons_per_meter{
        Eigen::Vector3d::Zero()};
    Eigen::Vector3d translational_damping_newton_seconds_per_meter{
        Eigen::Vector3d::Zero()};
};

// The complete typed force-element input to one immutable vehicle force plan.
// A named aggregate keeps family identity at call sites and lets the collection
// grow without another position-dependent constructor argument. It is an
// ownership envelope, not a common constitutive base class: each vector keeps
// its own force law and output contract.
struct VehicleForceElementCollection {
    std::vector<TranslationalSpringDamper> translational_spring_dampers;
    std::vector<RollSpringDamperCouple> roll_spring_damper_couples;
    std::vector<SeriesSpringViscousDamper> series_spring_viscous_dampers;
    std::vector<SaturatedPiecewiseLinearDamper>
        saturated_piecewise_linear_dampers;
    std::vector<HalfAngleMidpointRollPitchYawBushing>
        half_angle_midpoint_roll_pitch_yaw_bushings;
};

}  // namespace orvd::forces
