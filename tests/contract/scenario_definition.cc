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
constexpr double kFloatingLinkMassKilograms = 2.1;
constexpr double kGravityAccelerationMetersPerSecondSquared = 9.81;

// Layout produced by this topology, in Drake's depth-first coordinate order:
//   positions  [0]=shoulder angle, [1]=elbow angle,
//              [2..5]=floating quaternion, [6..8]=floating translation
//   velocities [0]=shoulder rate,  [1]=elbow rate,
//              [2..4]=floating angular rate, [5..7]=floating linear rate
constexpr int kVelocityCount = 8;
constexpr int kFirstFloatingLinearVelocityIndex = 5;

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
    scenario.revolute_joints = {
        {"shoulder_joint", "", "upper_link", Eigen::Matrix3d::Identity(),
         Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()},
        {"elbow_joint", "upper_link", "lower_link", elbow_mount_rotation,
         {0.0, -kUpperLinkLengthMeters, 0.0}, Eigen::Vector3d::UnitX()},
    };
    // The floating link is free by statement, not because nothing joins it.
    scenario.free_body_names = {"floating_link"};
    scenario.generalized_position_observation_kinds = {
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
    scenario.generalized_force_component_kinds.assign(
        kVelocityCount, GeneralizedForceComponentKind::kTorqueNewtonMetres);
    for (int velocity_index = kFirstFloatingLinearVelocityIndex;
         velocity_index < kVelocityCount; ++velocity_index) {
        scenario.generalized_force_component_kinds[velocity_index] =
            GeneralizedForceComponentKind::kForceNewtons;
    }

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
            0.0, 0.0,
            quaternion_w, quaternion_x, quaternion_y, quaternion_z,
            0.11, -0.23, 0.31};
        scenario.generalized_velocities.assign(kVelocityCount, 0.0);
        scenario.generalized_accelerations.assign(kVelocityCount, 0.0);
    } else if (excitation == "dynamic_excitation") {
        scenario.generalized_positions = {
            0.37, -0.62,
            quaternion_w, quaternion_x, quaternion_y, quaternion_z,
            0.11, -0.23, 0.31};
        scenario.generalized_velocities = {-1.9, 1.4, 0.9, -1.1, 1.7, -0.8, 1.3, -1.6};
        scenario.generalized_accelerations = {7.0, -5.5, 3.1, -4.2, 6.3, -2.7, 4.9, -3.4};
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
