#include "orvd/multibody_runtime/multibody_state_instance.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

#include <Eigen/Eigenvalues>

namespace orvd::multibody_runtime {
namespace {

// A rotation supplied by a caller has usually been composed from a few
// elementary rotations, so it carries a few units of round-off. This bound
// admits that and rejects anything that is not a rotation by any margin that
// matters — a transposed sign, a scaled axis, a shear. It is not a repair
// threshold: nothing is orthonormalised, the value is simply refused.
constexpr double kRotationOrthonormalityTolerance =
    128 * std::numeric_limits<double>::epsilon();

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("multibody state: " + detail);
}

void RequireIndexInRange(int index, int count, const char* what) {
    if (index < 0 || index >= count) {
        Reject(std::string(what) + " index " + std::to_string(index) +
               " is outside the " + std::to_string(count) + " that exist");
    }
}

void RequireFinite(double value, const std::string& what) {
    if (!std::isfinite(value)) {
        Reject(what + " is " + std::to_string(value) + ", which is not finite");
    }
}

void RequireFinite(const Eigen::Ref<const Eigen::VectorXd>& values,
                   std::string_view what) {
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i])) {
            Reject(std::string(what) + " entry " + std::to_string(i) + " is " +
                   std::to_string(values[i]) + ", which is not finite");
        }
    }
}

void RequireFinite(const Eigen::Vector3d& values, const std::string& what) {
    for (int i = 0; i < 3; ++i) {
        RequireFinite(values[i], what + " component " + std::to_string(i));
    }
}

void RequireNonNegative(double value, const std::string& what) {
    RequireFinite(value, what);
    if (value < 0.0) {
        Reject(what + " is " + std::to_string(value) +
               ", which cannot be negative");
    }
}

void RequireNonNegative(const Eigen::Vector3d& values, const std::string& what) {
    for (int i = 0; i < 3; ++i) {
        RequireNonNegative(values[i], what + " component " + std::to_string(i));
    }
}

void RequireSize(Eigen::Index actual, int expected, std::string_view what) {
    if (actual != expected) {
        Reject(std::string(what) + " has " + std::to_string(actual) +
               " entries, but the layout says " + std::to_string(expected));
    }
}

void RequireRotation(const Eigen::Matrix3d& rotation, const std::string& what) {
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            RequireFinite(rotation(row, column),
                          what + " element (" + std::to_string(row) + "," +
                              std::to_string(column) + ")");
        }
    }
    const Eigen::Matrix3d deviation =
        rotation * rotation.transpose() - Eigen::Matrix3d::Identity();
    const double orthonormality_error = deviation.cwiseAbs().maxCoeff();
    if (orthonormality_error > kRotationOrthonormalityTolerance) {
        Reject(what + " is not orthonormal: max|RRᵀ - I| is " +
               std::to_string(orthonormality_error) + ", above " +
               std::to_string(kRotationOrthonormalityTolerance));
    }
    if (rotation.determinant() <= 0.0) {
        Reject(what + " has determinant " +
               std::to_string(rotation.determinant()) +
               "; a rotation's determinant is +1, and a negative one means the "
               "basis is left-handed");
    }
}

// Checks the complete spatial-inertia condition at the boundary where the
// first-party value becomes live state. Checking only the three diagonal
// entries misses impossible products of inertia; checking about the body origin
// misses an inertia that becomes impossible after shifting to the centre of
// mass. The adapter must not be relied upon to reject a value after this store
// has already committed it.
void RequirePhysicallyValidSpatialInertia(
    const RigidBodyInertiaParameters& parameters, int rigid_body_index) {
    const std::string body =
        "rigid body " + std::to_string(rigid_body_index) + "'s ";
    RequireNonNegative(parameters.mass_kilograms, body + "mass");
    RequireFinite(parameters.center_of_mass_in_body_frame,
                  body + "centre of mass");
    RequireFinite(parameters.unit_inertia_moments,
                  body + "unit inertia moment");
    RequireFinite(parameters.unit_inertia_products, body + "unit inertia product");

    const Eigen::Vector3d& moments = parameters.unit_inertia_moments;
    const Eigen::Vector3d& products = parameters.unit_inertia_products;
    Eigen::Matrix3d unit_inertia_about_body_origin;
    unit_inertia_about_body_origin << moments[0], products[0], products[1],
        products[0], moments[1], products[2], products[1], products[2],
        moments[2];

    const Eigen::Vector3d& center_of_mass =
        parameters.center_of_mass_in_body_frame;
    const Eigen::Matrix3d point_mass_unit_inertia =
        center_of_mass.squaredNorm() * Eigen::Matrix3d::Identity() -
        center_of_mass * center_of_mass.transpose();
    const Eigen::Matrix3d central_rotational_inertia =
        parameters.mass_kilograms *
        (unit_inertia_about_body_origin - point_mass_unit_inertia);
    if (!central_rotational_inertia.allFinite()) {
        Reject(body + "central rotational inertia is not finite");
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> principal_moment_solver(
        central_rotational_inertia, Eigen::EigenvaluesOnly);
    if (principal_moment_solver.info() != Eigen::Success) {
        Reject(body + "central principal moments could not be computed");
    }
    const Eigen::Vector3d principal_moments =
        principal_moment_solver.eigenvalues();
    const double scale =
        std::max(1.0, 0.5 * std::abs(central_rotational_inertia.trace()));
    const double tolerance =
        16 * std::numeric_limits<double>::epsilon() * scale;
    if (principal_moments[0] < -tolerance ||
        principal_moments[0] + principal_moments[1] + tolerance <
            principal_moments[2]) {
        Reject(body + "central principal moments (" +
               std::to_string(principal_moments[0]) + ", " +
               std::to_string(principal_moments[1]) + ", " +
               std::to_string(principal_moments[2]) +
               ") are negative or violate the triangle inequality");
    }
}

}  // namespace

MultibodyStateInstance::MultibodyStateInstance(const MultibodyStateLayout& layout)
    : layout_(&layout),
      generalized_positions_(
          Eigen::VectorXd::Zero(layout.generalized_position_count())),
      generalized_velocities_(
          Eigen::VectorXd::Zero(layout.generalized_velocity_count())),
      rigid_body_inertia_parameters_(
          static_cast<std::size_t>(layout.rigid_body_count())),
      fixed_frame_pose_parameters_(
          static_cast<std::size_t>(layout.fixed_frame_count())),
      joint_damping_(
          Eigen::VectorXd::Zero(layout.total_joint_damping_count())),
      joint_actuator_parameters_(
          static_cast<std::size_t>(layout.joint_actuator_count())),
      revolute_spring_parameters_(
          static_cast<std::size_t>(layout.revolute_spring_count())),
      linear_bushing_parameters_(
          static_cast<std::size_t>(layout.linear_bushing_count())) {}

// Every versioned write runs in the same four steps, in this order:
//
//   1. validate the whole input;
//   2. compute the successor version, which fails if the counter is exhausted;
//   3. store the data;
//   4. commit the version.
//
// Step 2 sits before step 3 so that an exhausted counter cannot leave data
// written under a version that never advanced — the one state in which a stale
// cache reads as fresh. Step 4 sits after step 3 so that no reader can observe
// the new version while the old data is still in place.
void MultibodyStateInstance::set_generalized_positions(
    const Eigen::Ref<const Eigen::VectorXd>& positions) {
    RequireSize(positions.size(), layout_->generalized_position_count(),
                "the generalized position vector");
    RequireFinite(positions, "generalized position");
    const MultibodyStateVersion next = generalized_positions_version_.Next();
    generalized_positions_ = positions;
    generalized_positions_version_ = next;
}

void MultibodyStateInstance::set_generalized_velocities(
    const Eigen::Ref<const Eigen::VectorXd>& velocities) {
    RequireSize(velocities.size(), layout_->generalized_velocity_count(),
                "the generalized velocity vector");
    RequireFinite(velocities, "generalized velocity");
    const MultibodyStateVersion next = generalized_velocities_version_.Next();
    generalized_velocities_ = velocities;
    generalized_velocities_version_ = next;
}

const RigidBodyInertiaParameters&
MultibodyStateInstance::rigid_body_inertia_parameters(
    int rigid_body_index) const {
    RequireIndexInRange(rigid_body_index, layout_->rigid_body_count(),
                        "rigid body");
    return rigid_body_inertia_parameters_[static_cast<std::size_t>(
        rigid_body_index)];
}

void MultibodyStateInstance::set_rigid_body_inertia_parameters(
    int rigid_body_index, const RigidBodyInertiaParameters& parameters) {
    RequireIndexInRange(rigid_body_index, layout_->rigid_body_count(),
                        "rigid body");
    RequirePhysicallyValidSpatialInertia(parameters, rigid_body_index);
    const MultibodyStateVersion next = rigid_body_inertias_version_.Next();
    rigid_body_inertia_parameters_[static_cast<std::size_t>(rigid_body_index)] =
        parameters;
    rigid_body_inertias_version_ = next;
}

const FixedFramePoseParameters&
MultibodyStateInstance::fixed_frame_pose_parameters(
    int fixed_frame_index) const {
    RequireIndexInRange(fixed_frame_index, layout_->fixed_frame_count(),
                        "fixed frame");
    return fixed_frame_pose_parameters_[static_cast<std::size_t>(
        fixed_frame_index)];
}

void MultibodyStateInstance::set_fixed_frame_pose_parameters(
    int fixed_frame_index, const FixedFramePoseParameters& parameters) {
    RequireIndexInRange(fixed_frame_index, layout_->fixed_frame_count(),
                        "fixed frame");
    const std::string frame =
        "fixed frame " + std::to_string(fixed_frame_index) + "'s ";
    RequireRotation(parameters.R_PF, frame + "rotation R_PF");
    RequireFinite(parameters.p_PoFo_P, frame + "translation p_PoFo_P");
    const MultibodyStateVersion next = fixed_frame_poses_version_.Next();
    fixed_frame_pose_parameters_[static_cast<std::size_t>(fixed_frame_index)] =
        parameters;
    fixed_frame_poses_version_ = next;
}

Eigen::Ref<const Eigen::VectorXd> MultibodyStateInstance::joint_damping(
    int joint_index) const {
    const int count = layout_->joint_velocity_count(joint_index);
    return joint_damping_.segment(layout_->joint_damping_start(joint_index),
                                  count);
}

void MultibodyStateInstance::set_joint_damping(
    int joint_index, const Eigen::Ref<const Eigen::VectorXd>& damping) {
    const int count = layout_->joint_velocity_count(joint_index);
    const std::string joint = "joint " + std::to_string(joint_index) + "'s ";
    RequireSize(damping.size(), count, joint + "damping vector");
    for (Eigen::Index i = 0; i < damping.size(); ++i) {
        RequireNonNegative(damping[i],
                           joint + "damping coefficient " + std::to_string(i));
    }
    joint_damping_.segment(layout_->joint_damping_start(joint_index), count) =
        damping;
}

const JointActuatorParameters&
MultibodyStateInstance::joint_actuator_parameters(
    int joint_actuator_index) const {
    RequireIndexInRange(joint_actuator_index, layout_->joint_actuator_count(),
                        "joint actuator");
    return joint_actuator_parameters_[static_cast<std::size_t>(
        joint_actuator_index)];
}

void MultibodyStateInstance::set_joint_actuator_parameters(
    int joint_actuator_index, const JointActuatorParameters& parameters) {
    RequireIndexInRange(joint_actuator_index, layout_->joint_actuator_count(),
                        "joint actuator");
    const std::string actuator =
        "joint actuator " + std::to_string(joint_actuator_index) + "'s ";
    RequireNonNegative(parameters.rotor_inertia, actuator + "rotor inertia");
    RequireFinite(parameters.gear_ratio, actuator + "gear ratio");
    const MultibodyStateVersion next = joint_actuator_parameters_version_.Next();
    joint_actuator_parameters_[static_cast<std::size_t>(joint_actuator_index)] =
        parameters;
    joint_actuator_parameters_version_ = next;
}

const RevoluteSpringParameters&
MultibodyStateInstance::revolute_spring_parameters(
    int revolute_spring_index) const {
    RequireIndexInRange(revolute_spring_index, layout_->revolute_spring_count(),
                        "revolute spring");
    return revolute_spring_parameters_[static_cast<std::size_t>(
        revolute_spring_index)];
}

void MultibodyStateInstance::set_revolute_spring_parameters(
    int revolute_spring_index, const RevoluteSpringParameters& parameters) {
    RequireIndexInRange(revolute_spring_index, layout_->revolute_spring_count(),
                        "revolute spring");
    const std::string spring =
        "revolute spring " + std::to_string(revolute_spring_index) + "'s ";
    RequireNonNegative(parameters.stiffness, spring + "stiffness");
    RequireFinite(parameters.nominal_angle_radians, spring + "nominal angle");
    revolute_spring_parameters_[static_cast<std::size_t>(
        revolute_spring_index)] = parameters;
}

const LinearBushingRollPitchYawParameters&
MultibodyStateInstance::linear_bushing_parameters(
    int linear_bushing_index) const {
    RequireIndexInRange(linear_bushing_index, layout_->linear_bushing_count(),
                        "linear bushing");
    return linear_bushing_parameters_[static_cast<std::size_t>(
        linear_bushing_index)];
}

void MultibodyStateInstance::set_linear_bushing_parameters(
    int linear_bushing_index,
    const LinearBushingRollPitchYawParameters& parameters) {
    RequireIndexInRange(linear_bushing_index, layout_->linear_bushing_count(),
                        "linear bushing");
    const std::string bushing =
        "linear bushing " + std::to_string(linear_bushing_index) + "'s ";
    RequireNonNegative(parameters.torque_stiffness, bushing + "torque stiffness");
    RequireNonNegative(parameters.torque_damping, bushing + "torque damping");
    RequireNonNegative(parameters.force_stiffness, bushing + "force stiffness");
    RequireNonNegative(parameters.force_damping, bushing + "force damping");
    linear_bushing_parameters_[static_cast<std::size_t>(linear_bushing_index)] =
        parameters;
}

}  // namespace orvd::multibody_runtime
