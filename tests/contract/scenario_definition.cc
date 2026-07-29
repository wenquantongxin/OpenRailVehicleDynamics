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
constexpr double kGravityAcceleration = 9.81;

// Layout produced by this topology, in Drake's depth-first coordinate order:
//   positions  [0]=shoulder angle, [1]=elbow angle,
//              [2..5]=floating quaternion, [6..8]=floating translation
//   velocities [0]=shoulder rate,  [1]=elbow rate,
//              [2..4]=floating angular rate, [5..7]=floating linear rate
constexpr int kPositionCount = 9;
constexpr int kVelocityCount = 8;
constexpr int kFirstFloatingLinearVelocityIndex = 5;

}  // namespace

ScenarioDefinition MakeRevoluteChainWithFloatingBodyScenario(
    std::string_view excitation) {
    ScenarioDefinition scenario;
    scenario.name = std::string("revolute_chain_with_floating_body.") +
                    std::string(excitation);
    scenario.gravity_acceleration_meters_per_second_squared = kGravityAcceleration;

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
    scenario.revolute_joints = {
        {"shoulder_joint", "", "upper_link",
         Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()},
        {"elbow_joint", "upper_link", "lower_link",
         {0.0, -kUpperLinkLengthMeters, 0.0}, Eigen::Vector3d::UnitX()},
    };
    scenario.free_floating_link_name = "floating_link";

    // A legal unit quaternion. How an implementation handles a non-unit
    // quaternion is a separate question about normalization, and mixing it in
    // here would confound an equivalence result with that question.
    const double unit_quaternion_component = 0.5;

    if (excitation == "near_zero_cancellation") {
        scenario.generalized_positions = {
            0.0, 0.0,
            unit_quaternion_component, unit_quaternion_component,
            unit_quaternion_component, unit_quaternion_component,
            0.11, -0.23, 0.31};
        scenario.generalized_velocities.assign(kVelocityCount, 0.0);
        scenario.generalized_accelerations.assign(kVelocityCount, 0.0);
    } else if (excitation == "dynamic_excitation") {
        scenario.generalized_positions = {
            0.37, -0.62,
            unit_quaternion_component, unit_quaternion_component,
            unit_quaternion_component, unit_quaternion_component,
            0.11, -0.23, 0.31};
        scenario.generalized_velocities = {-1.9, 1.4, 0.9, -1.1, 1.7, -0.8, 1.3, -1.6};
        scenario.generalized_accelerations = {7.0, -5.5, 3.1, -4.2, 6.3, -2.7, 4.9, -3.4};
    } else {
        throw std::invalid_argument(
            "unknown excitation: " + std::string(excitation));
    }

    const double reach_meters = kUpperLinkLengthMeters + kLowerLinkLengthMeters;
    const double characteristic_angular_acceleration = kGravityAcceleration / reach_meters;

    scenario.mass_matrix_excitation_scales.assign(
        kFirstFloatingLinearVelocityIndex, characteristic_angular_acceleration);
    scenario.mass_matrix_excitation_scales.resize(kVelocityCount, kGravityAcceleration);

    ComparisonScales& scales = scenario.comparison_scales;
    scales.angle_radians = 1.0;
    scales.translation_meters = reach_meters;

    // Rotational velocity indices carry torque; translational ones carry force.
    // Each entry is the gravitational moment or weight the corresponding degree
    // of freedom actually works against, built from the masses and lengths above.
    const double revolute_torque_scale =
        (kUpperLinkMassKilograms + kLowerLinkMassKilograms) * kGravityAcceleration *
        reach_meters;
    const double floating_torque_scale =
        kFloatingLinkMassKilograms * kGravityAcceleration * reach_meters;
    const double floating_force_scale =
        kFloatingLinkMassKilograms * kGravityAcceleration;

    scales.generalized_torque_component_newton_metres.assign(kVelocityCount, 0.0);
    scales.generalized_force_component_newtons.assign(kVelocityCount, 0.0);
    for (int velocity_index = 0; velocity_index < kVelocityCount; ++velocity_index) {
        if (velocity_index < 2) {
            scales.generalized_torque_component_newton_metres[velocity_index] =
                revolute_torque_scale;
        } else if (velocity_index < kFirstFloatingLinearVelocityIndex) {
            scales.generalized_torque_component_newton_metres[velocity_index] =
                floating_torque_scale;
        } else {
            scales.generalized_force_component_newtons[velocity_index] =
                floating_force_scale;
        }
    }
    return scenario;
}

}  // namespace orvd_contract
