#pragma once

#include <Eigen/Core>

// How fast the two surfaces slide past each other where they touch, expressed
// as the three numbers a tangential contact law reads.
//
// A wheel rolling on a rail does not roll perfectly. The contact patch is
// deformed, so material entering it and material leaving it travel slightly
// different distances, and the difference is a small relative sliding — the
// creep. Divided by the speed the contact travels at, it becomes a creepage: a
// dimensionless slip for each of the two in-plane directions, and a spin about
// the contact normal that is not dimensionless at all but a rate per unit
// length.
//
// ## The frame is built from the rail's slope, not from the common normal
//
// There are two candidate angles at a contact and they are not the same. The
// common normal characterises the two profile surfaces geometrically. The
// selected GZ18 formulation resolves the closing speed, the normal-force vector
// and the tangential force in a frame built from the rail's own surface slope
// plus its laying cant. Substituting the common normal for that rail-reference
// angle rotates every force by their difference.
//
// The frame is a pure roll about the track's longitudinal axis. Its x axis is
// therefore exactly the track tangent, which is why the longitudinal creepage's
// numerator is the raw longitudinal relative velocity with nothing done to it.
// There is no yaw and no wheelset roll in this frame.
//
// ## The reference speed, and what happens near standstill
//
// The creepages divide by a reference speed: the mean of how fast the contact
// advances along the line and how fast the wheel's rim turns under it. At a
// standstill both vanish and the ratio is meaningless, so the divisor is held
// away from zero.
//
// That floor is not continuous through zero. Approaching standstill from
// forward gives one sign; from reverse, the other; and the two differ by twice
// the floor. So all three creepages flip sign as the reference speed crosses
// zero even though their numerators do not. This is stated rather than smoothed
// because smoothing it would change the forces at every speed, not just at
// zero, and because a vehicle that reverses through standstill is outside what
// this module is for.

namespace orvd::wheel_rail_contact {

// The contact's own axes, as a rotation from them into the track's.
//
// The third column is the contact normal. The same frame resolves the
// creepages, the closing speed and the resulting force, and a consumer should
// build it once per patch rather than three times.
struct ContactFrame {
    Eigen::Matrix3d rotation_track_from_contact{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d normal_track{Eigen::Vector3d::UnitZ()};
};

// Builds the frame from the rail's surface slope angle at the contact,
// including the laying cant. A pure roll about the track's longitudinal axis.
[[nodiscard]] ContactFrame MakeContactFrame(double rail_contact_angle_radians);

// How the wheel's material at the contact point is moving relative to the
// rail's, in track axes, together with the two rates the reference speed is
// built from.
struct ContactRelativeMotion {
    // The velocity of the wheel's material point at the contact, less the
    // rail's — which is zero, the rail being treated as stationary.
    Eigen::Vector3d relative_velocity_track_meters_per_second{
        Eigen::Vector3d::Zero()};
    // The angular velocity of the wheel relative to the rail.
    Eigen::Vector3d relative_spin_track_radians_per_second{Eigen::Vector3d::Zero()};
    // How fast the contact advances along the line.
    double arc_rate_meters_per_second{0.0};
    // The wheel's rotation rate about its own axle. Negative running forward,
    // by the sign convention of a right-handed axle pointing to the right of
    // travel — so the rim speed below comes out positive.
    double wheel_pitch_rate_radians_per_second{0.0};
};

struct CreepageConfiguration {
    // The magnitude the reference speed is held away from zero at. See the
    // header note: this is a floor, not a smoothing, and it is discontinuous
    // through zero.
    double minimum_reference_speed_meters_per_second{0.01};
};

struct Creepages {
    // Dimensionless: relative sliding per unit travel.
    double longitudinal{0.0};
    double lateral{0.0};
    // Per metre: rotation about the contact normal per unit travel.
    double spin_per_meter{0.0};
    // The divisor, after flooring.
    double reference_speed_meters_per_second{0.0};
};

// The three creepages.
//
// `rolling_radius_meters` is the undeformed local radius at the contact — the
// radius before any elastic penetration. Subtracting half the penetration from
// it, which some formulations do, is a different convention and changes the
// reference speed by a part in ten thousand.
//
// Allocates nothing, throws nothing.
[[nodiscard]] Creepages ComputeCreepages(const ContactRelativeMotion& motion,
                                         const ContactFrame& frame,
                                         double rolling_radius_meters,
                                         const CreepageConfiguration& configuration);

// How fast the two bodies are closing along the contact normal. Positive while
// approaching, which is the sign the normal contact law's damping expects.
[[nodiscard]] double ComputeNormalApproachSpeed(const ContactRelativeMotion& motion,
                                                const ContactFrame& frame);

}  // namespace orvd::wheel_rail_contact
