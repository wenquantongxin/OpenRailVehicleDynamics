#include "contract/scenario_definition.h"

#include <cmath>

#include <stdexcept>
#include <string_view>

namespace orvd_contract {
namespace {

constexpr double kUpperLinkLengthMeters = 0.5;
constexpr double kLowerLinkLengthMeters = 0.3;
constexpr double kUpperLinkMassKilograms = 1.3;
constexpr double kLowerLinkMassKilograms = 0.7;
constexpr double kReverseBranchMassKilograms = 0.45;
constexpr double kFloatingLinkMassKilograms = 2.1;
constexpr double kGravityAccelerationMetersPerSecondSquared = 9.81;

// Layout produced by this topology, in Drake's depth-first coordinate order:
//   positions  [0]=shoulder angle, [1]=elbow angle,
//              [2]=reverse-branch angle,
//              [3..6]=floating quaternion, [7..9]=floating translation
//   velocities [0]=shoulder rate,  [1]=elbow rate,
//              [2]=reverse-branch rate,
//              [3..5]=floating angular rate, [6..8]=floating linear rate
constexpr int kVelocityCount = 9;
constexpr int kFirstFloatingLinearVelocityIndex = 6;

}  // namespace

ScenarioDefinition MakeRevoluteChainWithFloatingBodyScenario(
    std::string_view excitation) {
    ScenarioDefinition scenario;
    scenario.gravity_acceleration_meters_per_second_squared =
        kGravityAccelerationMetersPerSecondSquared;

    scenario.links = {
        {"upper_link", kUpperLinkMassKilograms,
         {0.0, -kUpperLinkLengthMeters / 2.0, 0.0},
         {0.05, kUpperLinkLengthMeters, 0.05}},
        {"lower_link", kLowerLinkMassKilograms,
         {0.0, -kLowerLinkLengthMeters / 2.0, 0.0},
         {0.04, kLowerLinkLengthMeters, 0.04}},
        {"reverse_branch_link", kReverseBranchMassKilograms,
         {0.04, -0.03, 0.02},
         {0.16, 0.12, 0.1}},
        {"floating_link", kFloatingLinkMassKilograms,
         {0.01, 0.02, -0.03},
         {0.2, 0.3, 0.4}},
    };
    // The elbow's mount is turned as well as displaced. Without a non-identity
    // rotation somewhere in the chain, the code that carries one is never
    // exercised by this comparison, and the axis below — stated in the mount's
    // frame — would point the same way whether the rotation survived or not.
    Eigen::Matrix3d elbow_mount_rotation;
    {
        const double angle = 0.37;
        elbow_mount_rotation << std::cos(angle), 0.0, std::sin(angle), 0.0, 1.0,
            0.0, -std::sin(angle), 0.0, std::cos(angle);
    }
    Eigen::Matrix3d reverse_branch_mount_rotation;
    {
        const double angle = -0.29;
        reverse_branch_mount_rotation << 1.0, 0.0, 0.0, 0.0,
            std::cos(angle), -std::sin(angle), 0.0, std::sin(angle),
            std::cos(angle);
    }
    scenario.revolute_joints = {
        {"shoulder_joint", "", "upper_link", Eigen::Matrix3d::Identity(),
         Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.14},
        {"elbow_joint", "upper_link", "lower_link", elbow_mount_rotation,
         {0.0, -kUpperLinkLengthMeters, 0.0}, Eigen::Vector3d::UnitX(), 0.23},
        // The branch link is named as parent while the already world-connected
        // upper link is named as child. The forest must traverse this relation
        // backwards to reach the branch, so the coordinate and velocity signs
        // are independently observable.
        {"reverse_branch_joint", "reverse_branch_link", "upper_link",
         reverse_branch_mount_rotation, {0.12, -0.08, 0.15},
         Eigen::Vector3d::UnitY(), 0.31},
    };
    // The floating link is free by statement, not because nothing joins it.
    scenario.free_body_names = {"floating_link"};
    scenario.generalized_position_observation_kinds = {
        ObservationKind::kAngleRadians,
        ObservationKind::kAngleRadians,
        ObservationKind::kAngleRadians,
        ObservationKind::kUnitQuaternionComponent,
        ObservationKind::kUnitQuaternionComponent,
        ObservationKind::kUnitQuaternionComponent,
        ObservationKind::kUnitQuaternionComponent,
        ObservationKind::kTranslationMeters,
        ObservationKind::kTranslationMeters,
        ObservationKind::kTranslationMeters,
    };
    scenario.generalized_velocity_observation_kinds = {
        ObservationKind::kAngularVelocityRadiansPerSecond,
        ObservationKind::kAngularVelocityRadiansPerSecond,
        ObservationKind::kAngularVelocityRadiansPerSecond,
        ObservationKind::kAngularVelocityRadiansPerSecond,
        ObservationKind::kAngularVelocityRadiansPerSecond,
        ObservationKind::kAngularVelocityRadiansPerSecond,
        ObservationKind::kTranslationalVelocityMetersPerSecond,
        ObservationKind::kTranslationalVelocityMetersPerSecond,
        ObservationKind::kTranslationalVelocityMetersPerSecond,
    };
    scenario.generalized_position_derivative_observation_kinds = {
        ObservationKind::kAngularVelocityRadiansPerSecond,
        ObservationKind::kAngularVelocityRadiansPerSecond,
        ObservationKind::kAngularVelocityRadiansPerSecond,
        ObservationKind::kQuaternionDerivativePerSecond,
        ObservationKind::kQuaternionDerivativePerSecond,
        ObservationKind::kQuaternionDerivativePerSecond,
        ObservationKind::kQuaternionDerivativePerSecond,
        ObservationKind::kTranslationalVelocityMetersPerSecond,
        ObservationKind::kTranslationalVelocityMetersPerSecond,
        ObservationKind::kTranslationalVelocityMetersPerSecond,
    };
    scenario.differential_kinematics_probe_generalized_velocities = {
        0.31, -0.27, 0.43, -0.52, 0.38, -0.29, 0.47, -0.36, 0.22};
    scenario.inverse_mapping_probe_generalized_position_derivatives = {
        0.26, -0.31, 0.44, -0.37, 0.18, 0.52, -0.29, 0.33, -0.41, 0.24};
    scenario.generalized_force_component_kinds.assign(
        kVelocityCount, GeneralizedForceComponentKind::kTorqueNewtonMetres);
    for (int velocity_index = kFirstFloatingLinearVelocityIndex;
         velocity_index < kVelocityCount; ++velocity_index) {
        scenario.generalized_force_component_kinds[velocity_index] =
            GeneralizedForceComponentKind::kForceNewtons;
    }
    scenario.relative_spatial_velocity_observations = {
        {"lower_relative_to_floating_expressed_in_upper", "lower_link",
         "floating_link", "upper_link"}};

    // A legal unit quaternion whose four components are all non-zero and
    // different, including one different sign. Equal components would make any
    // wxyz permutation invisible; that is exactly the ordering this scenario's
    // index-keyed state is meant to exercise. How an implementation handles a
    // non-unit quaternion is a separate normalization question.
    const double quaternion_scale = 1.0 / std::sqrt(30.0);
    const double quaternion_w = quaternion_scale;
    const double quaternion_x = 2.0 * quaternion_scale;
    const double quaternion_y = -3.0 * quaternion_scale;
    const double quaternion_z = 4.0 * quaternion_scale;

    if (excitation == "near_zero_cancellation") {
        scenario.generalized_positions = {
            0.0, 0.0, 0.0,
            quaternion_w, quaternion_x, quaternion_y, quaternion_z,
            0.11, -0.23, 0.31};
        scenario.generalized_velocities.assign(kVelocityCount, 0.0);
        scenario.generalized_accelerations.assign(kVelocityCount, 0.0);
    } else if (excitation == "dynamic_excitation") {
        scenario.generalized_positions = {
            0.37, -0.62, 0.28,
            quaternion_w, quaternion_x, quaternion_y, quaternion_z,
            0.11, -0.23, 0.31};
        scenario.generalized_velocities = {
            -1.9, 1.4, -0.6, 0.9, -1.1, 1.7, -0.8, 1.3, -1.6};
        scenario.generalized_accelerations = {
            7.0, -5.5, 2.6, 3.1, -4.2, 6.3, -2.7, 4.9, -3.4};
        scenario.applied_body_wrenches = {{
            "lower_link", {0.08, -0.04, 0.03}, "upper_link",
            {0.17, -0.22, 0.31}, {1.4, -0.9, 0.6}}};
        scenario.applied_revolute_joint_torques = {
            {"shoulder_joint", 0.7}, {"reverse_branch_joint", -0.35}};
    } else {
        throw std::invalid_argument(
            "unknown excitation: " + std::string(excitation));
    }

    const double reach_meters = kUpperLinkLengthMeters + kLowerLinkLengthMeters;
    const double angular_acceleration_radians_per_second_squared =
        kGravityAccelerationMetersPerSecondSquared / reach_meters;
    scenario.mass_matrix_column_generalized_accelerations.assign(
        kFirstFloatingLinearVelocityIndex,
        angular_acceleration_radians_per_second_squared);
    scenario.mass_matrix_column_generalized_accelerations.resize(
        kVelocityCount, kGravityAccelerationMetersPerSecondSquared);
    return scenario;
}

}  // namespace orvd_contract
