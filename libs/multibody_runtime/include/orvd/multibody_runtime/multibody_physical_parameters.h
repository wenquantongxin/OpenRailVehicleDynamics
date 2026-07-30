#pragma once

/// @file
/// The context-mutable physical parameters of a multibody model.
///
/// Each record says what it is in its own field names. Upstream packs these into
/// anonymous numeric vectors — a spatial inertia as ten doubles, a fixed frame
/// pose as a flattened three-by-four — which is a transport format, not a
/// meaning. A caller reading `parameter_vector[7]` has to consult a table to
/// learn it is holding a product of inertia; a caller reading
/// `unit_inertia_products.x()` does not.
///
/// These are first-party records. They deliberately do not use the vendored
/// value types (`SpatialInertia`, `RigidTransform`): this layer owns state and
/// must not depend on the rigid tree, so the conversion happens once, at the
/// adapter boundary, rather than making the base layer reach upward.
///
/// Only parameters that vary per evaluation context appear here. Gravity and the
/// constants held directly by force elements belong to the finalized model,
/// which is immutable and shared.

#include <Eigen/Dense>

namespace orvd::multibody_runtime {

/// Mass properties of one rigid body, about the body origin and expressed in the
/// body frame.
///
/// A massless body is legitimate — a frame carrier with no inertia is a normal
/// modelling device — so zero is an accepted value throughout, not an error.
struct RigidBodyInertiaParameters {
    /// Mass in kilograms. Non-negative.
    double mass_kilograms{0.0};

    /// Position of the centre of mass from the body origin, expressed in the
    /// body frame, in metres.
    Eigen::Vector3d center_of_mass_in_body_frame{Eigen::Vector3d::Zero()};

    /// Diagonal of the unit inertia (inertia per unit mass) about the body
    /// origin, expressed in the body frame: Gxx, Gyy, Gzz, in square metres.
    /// Physical validity is checked after the complete symmetric inertia is
    /// shifted to the centre of mass and reduced to its principal moments.
    Eigen::Vector3d unit_inertia_moments{Eigen::Vector3d::Zero()};

    /// Off-diagonal of the same unit inertia: Gxy, Gxz, Gyz, in square metres.
    /// Sign is unconstrained; a product of inertia may be negative.
    Eigen::Vector3d unit_inertia_products{Eigen::Vector3d::Zero()};
};

/// The fixed pose of a frame F in its parent frame P.
///
/// `R_PF` and `p_PoFo_P` are the project's standard notation: the rotation of F
/// in P, and the position vector from P's origin to F's origin expressed in P.
struct FixedFramePoseParameters {
    /// Rotation matrix of F in P. Must be a rotation: orthonormal with unit
    /// determinant. It is validated, never repaired — silently orthonormalising
    /// a caller's matrix turns a modelling mistake into a plausible result.
    Eigen::Matrix3d R_PF{Eigen::Matrix3d::Identity()};

    /// Position of F's origin from P's origin, expressed in P, in metres.
    Eigen::Vector3d p_PoFo_P{Eigen::Vector3d::Zero()};
};

/// Reflected inertia contribution of one joint actuator.
struct JointActuatorParameters {
    /// Rotor moment of inertia about the rotor axis, in kg·m². Non-negative.
    double rotor_inertia{0.0};

    /// Gear ratio, dimensionless. Only required to be finite: the sign encodes
    /// the direction of the reduction, and constraining it here would forbid a
    /// legitimate arrangement for no reason. Reflected inertia goes as the
    /// square of this, so the sign does not reach the inertia either way.
    double gear_ratio{1.0};
};

/// A torsional spring acting about one revolute joint's axis.
struct RevoluteSpringParameters {
    /// Torsional stiffness in N·m/rad. Non-negative.
    double stiffness{0.0};

    /// The angle at which the spring exerts no torque, in radians.
    double nominal_angle_radians{0.0};
};

/// A six-degree-of-freedom bushing between two frames, with roll-pitch-yaw
/// torque terms and translational force terms.
struct LinearBushingRollPitchYawParameters {
    /// Torsional stiffness about the roll, pitch and yaw axes, in N·m/rad.
    /// Non-negative.
    Eigen::Vector3d torque_stiffness{Eigen::Vector3d::Zero()};

    /// Torsional damping about the same axes, in N·m·s/rad. Non-negative.
    Eigen::Vector3d torque_damping{Eigen::Vector3d::Zero()};

    /// Translational stiffness along the bushing axes, in N/m. Non-negative.
    Eigen::Vector3d force_stiffness{Eigen::Vector3d::Zero()};

    /// Translational damping along the same axes, in N·s/m. Non-negative.
    Eigen::Vector3d force_damping{Eigen::Vector3d::Zero()};
};

}  // namespace orvd::multibody_runtime
