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
// fixed-axis series, clipped and bushing families use a reference end and its
// balancing support moment. Point-to-point elements act along the line joining
// their attachment points without an additional couple.

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
// with time constant damping / stiffness. The series families carry their force
// states in the system's third continuous-state block.
struct SeriesSpringViscousDamper {
    std::string name;
    ForceElementEnd reference_end;
    ForceElementEnd opposite_end;
    ForceElementAxis axis{ForceElementAxis::kLateral};
    double series_stiffness_newtons_per_meter{0.0};
    double series_damping_newton_seconds_per_meter{0.0};
};

// A linear spring in series with an odd bilinear damper, acting along the
// instantaneous line between its two attachment points. Positive force pulls
// the points together; neither endpoint carries an additional couple.
//
// The damping slopes below and above the positive knee force are C and C2.
// The inverse damping law is F/C inside the knee, and
// sign(F) * (Fb/C + (abs(F) - Fb)/C2) outside it. The tensile force F is the
// state, with dF/dt = K * (length_rate - inverse_damping(F)).
struct PointToPointSeriesSpringBilinearDamper {
    std::string name;
    ForceElementEnd reference_end;
    ForceElementEnd opposite_end;
    double series_stiffness_newtons_per_meter{0.0};
    double damping_below_knee_newton_seconds_per_meter{0.0};
    double knee_force_newtons{0.0};
    double damping_above_knee_newton_seconds_per_meter{0.0};
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

// A bushing with diagonal translational stiffness, translational damping and
// rotational damping expressed in the reference frame. Translation and its
// derivative are measured in that rotating frame; rotational damping uses the
// physical relative angular velocity, without an Euler-angle rate map.
//
// Forces act at the two attachment points. The reference end also carries
// (p_opposite - p_reference) x force_on_reference. The rotational damping
// moment acts as an equal-and-opposite couple on the two bodies.
struct ReferenceFrameLinearBushing {
    std::string name;
    ForceElementEnd reference_end;
    ForceElementEnd opposite_end;
    Eigen::Vector3d translational_stiffness_newtons_per_meter{
        Eigen::Vector3d::Zero()};
    Eigen::Vector3d translational_damping_newton_seconds_per_meter{
        Eigen::Vector3d::Zero()};
    Eigen::Vector3d rotational_damping_newton_meter_seconds_per_radian{
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
    std::vector<PointToPointSeriesSpringBilinearDamper>
        point_to_point_series_spring_bilinear_dampers;
    std::vector<SaturatedPiecewiseLinearDamper>
        saturated_piecewise_linear_dampers;
    std::vector<ReferenceFrameLinearBushing> reference_frame_linear_bushings;
};

}  // namespace orvd::forces
