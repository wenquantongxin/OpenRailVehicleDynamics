#include "orvd/multibody_runtime/multibody_state_instance.h"

#include "orvd/multibody_runtime/multibody_physical_parameter_validation.h"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace orvd::multibody_runtime {
namespace {

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

bool OverlapsStorage(const Eigen::Ref<const Eigen::VectorXd>& values,
                     const Eigen::VectorXd& storage) {
    if (values.size() == 0 || storage.size() == 0) {
        return false;
    }
    const double* const values_begin = values.data();
    const double* const values_end = values_begin + values.size();
    const double* const storage_begin = storage.data();
    const double* const storage_end = storage_begin + storage.size();
    const std::less<const double*> less;
    return less(values_begin, storage_end) &&
           less(storage_begin, values_end);
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
// the new version while the old data is still in place during single-threaded
// evaluation. This ordering is not a synchronization mechanism.
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

void MultibodyStateInstance::set_generalized_state(
    const Eigen::Ref<const Eigen::VectorXd>& positions,
    const Eigen::Ref<const Eigen::VectorXd>& velocities) {
    RequireSize(positions.size(), layout_->generalized_position_count(),
                "the generalized position vector");
    RequireSize(velocities.size(), layout_->generalized_velocity_count(),
                "the generalized velocity vector");
    RequireFinite(positions, "generalized position");
    RequireFinite(velocities, "generalized velocity");
    const MultibodyStateVersion next_positions =
        generalized_positions_version_.Next();
    const MultibodyStateVersion next_velocities =
        generalized_velocities_version_.Next();
    // q is stored first. Preserve v only when its input borrows q; the normal
    // ODE path supplies external contiguous storage and allocates nothing here.
    if (OverlapsStorage(velocities, generalized_positions_)) {
        const Eigen::VectorXd velocities_at_entry = velocities;
        generalized_positions_ = positions;
        generalized_velocities_ = velocities_at_entry;
    } else {
        generalized_positions_ = positions;
        generalized_velocities_ = velocities;
    }
    generalized_positions_version_ = next_positions;
    generalized_velocities_version_ = next_velocities;
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
    ThrowIfNotRealisableInertia(
        parameters, "rigid body " + std::to_string(rigid_body_index));
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
    ThrowIfNotAValidFixedFramePose(
        parameters, "fixed frame " + std::to_string(fixed_frame_index));
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
