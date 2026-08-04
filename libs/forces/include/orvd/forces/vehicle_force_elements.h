#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

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

}  // namespace orvd::forces
