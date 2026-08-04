#pragma once

#include <vector>

#include <Eigen/Core>

#include "orvd/multibody_model/multibody_applied_forces.h"
#include "orvd/multibody_model/multibody_model_handles.h"

// The force elements a rail vehicle's suspension is made of, named by what they
// are constitutively rather than by the vendor or the vehicle they came from.
//
// Every element connects two named frames. One of them is the reference end:
// the relative displacement and velocity the constitutive law reads are
// expressed in that frame, and so is the force the law produces. Which end is
// the reference end is part of the element, not a convention a reader has to
// remember — swapping the two ends leaves the force balance intact but changes
// which body carries the support moment, and no balance identity can see that.
//
// The wrench a translational element produces is a pair, and the pair is not
// simply equal and opposite forces at the two ends. The constitutive force is
// not in general parallel to the line joining them, so two forces alone would
// leave a residual couple and quietly create angular momentum. The reference
// end therefore also carries
//
//     support moment = (opposite point - reference point) x reference force
//
// which is exactly what cancels that residual. This is a property of the
// element, not a diagnostic that can be switched off, and it is emitted in one
// place so that there is one implementation to check.

namespace orvd::forces {

// Which axis of the reference frame an element acts along, for the families
// that act along exactly one. Stating it as a choice rather than as a vector of
// which two entries happen to be zero keeps the number of internal state
// components a property of the family instead of a property of the data.
enum class ForceElementAxis { kLongitudinal, kLateral, kVertical };

// One end of a force element.
//
// The frame answers the kinematic question — where the other end is and how
// fast it is moving, in this end's own axes. The body and the point answer the
// dynamic one — where the wrench lands. They are not redundant: the multibody
// facade takes a wrench at a body-fixed point, and this end's point is that
// frame's origin in that body. A builder resolves all three from the one record
// entry that declares the frame, so they cannot disagree with each other unless
// the builder is wrong, which is what the assembly gate is for.
struct ForceElementEnd {
    multibody_model::FrameHandle frame;
    multibody_model::RigidBodyHandle body;
    Eigen::Vector3d point_in_body_frame_meters{Eigen::Vector3d::Zero()};
};

// A spring and a viscous damper in parallel, acting between two frames, with a
// diagonal stiffness and damping in the reference frame.
//
// The nominal force is not here. It is a per-instance startup quantity that a
// resolved start-up state supplies to a context, not part of what the vehicle
// is, so the plan reads it from the context each evaluation.
struct TranslationalSpringDamper {
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
// the force holds at that breakpoint's value: a real damper's relief valve does
// not keep growing, and extrapolating a line would invent force nobody stated.
struct SaturatedPiecewiseLinearDamperPoint {
    double relative_velocity_meters_per_second{0.0};
    double force_newtons{0.0};
};

struct SaturatedPiecewiseLinearDamper {
    ForceElementEnd reference_end;
    ForceElementEnd opposite_end;
    ForceElementAxis axis{ForceElementAxis::kLateral};
    // Ascending in velocity, first point at zero velocity and zero force.
    std::vector<SaturatedPiecewiseLinearDamperPoint> curve;
};

// The wrench pair one translational or one roll element produces. Both entries
// are always written; a roll element writes zero force and equal and opposite
// moments, and its two application points are the two frame origins.
struct ForceElementWrenchPair {
    multibody_model::AppliedBodyWrench on_reference_body;
    multibody_model::AppliedBodyWrench on_opposite_body;
};

}  // namespace orvd::forces
