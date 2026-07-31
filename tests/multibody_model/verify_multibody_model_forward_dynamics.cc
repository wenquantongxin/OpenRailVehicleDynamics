#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::AppliedBodyWrench;
using orvd::multibody_model::AppliedPrismaticJointForce;
using orvd::multibody_model::AppliedRevoluteJointTorque;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

int failure_count = 0;

void Expect(bool condition, const char* description) {
    if (!condition) {
        std::printf("FAIL %s\n", description);
        ++failure_count;
    }
}

void ExpectNear(double actual, double expected, const char* description) {
    const double tolerance = 1024.0 * std::numeric_limits<double>::epsilon() *
                             std::max(1.0, std::abs(expected));
    Expect(std::abs(actual - expected) <= tolerance, description);
}

RigidBodyInertiaParameters MakeInertia(double mass, double com_x,
                                       double central_unit_izz) {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass;
    inertia.center_of_mass_in_body_frame = Eigen::Vector3d(com_x, 0.0, 0.0);
    inertia.unit_inertia_moments =
        Eigen::Vector3d(central_unit_izz + com_x * com_x,
                        central_unit_izz + com_x * com_x,
                        central_unit_izz + com_x * com_x);
    inertia.unit_inertia_products.setZero();
    return inertia;
}

void CheckOffsetWrenchAndStateDerivative() {
    constexpr double mass = 2.0;
    constexpr double com_x = 0.2;
    constexpr double central_unit_izz = 0.3;
    MultibodyModel model;
    const auto body =
        model.AddRigidBody("rotor", MakeInertia(mass, com_x,
                                                central_unit_izz));
    const auto joint = model.AddRevoluteJoint(
        "hinge", model.world_frame(), model.body_frame(body),
        Eigen::Vector3d::UnitZ(), 0.0);
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();
    auto context = model.CreateDefaultContext();
    Eigen::VectorXd velocity(1);
    velocity << 0.73;
    model.SetGeneralizedVelocities(context.get(), velocity);
    auto workspace = model.CreateForwardDynamicsWorkspace();

    const std::array body_wrenches{AppliedBodyWrench{
        body, Eigen::Vector3d(0.3, 0.0, 0.0), model.world_frame(),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 2.0, 0.0)}};
    const std::array torques{
        AppliedRevoluteJointTorque{joint, 0.4}};
    Eigen::VectorXd vdot(1);
    model.CalcGeneralizedVelocityDerivatives(
        *context, body_wrenches, torques,
        std::span<const AppliedPrismaticJointForce>{}, *workspace, vdot);
    const double inertia = mass * (central_unit_izz + com_x * com_x);
    ExpectNear(vdot[0], (1.0 + 0.3 * 2.0 + 0.4) / inertia,
               "an offset wrench is shifted to the body origin before ABA");
    Eigen::VectorXd required(1);
    model.CalcRequiredGeneralizedForces(*context, vdot, required);
    ExpectNear(required[0], 2.0,
               "ABA followed by RNEA recovers the applied generalized force");

    Eigen::VectorXd xdot(2);
    model.CalcStateTimeDerivatives(
        *context, body_wrenches, torques,
        std::span<const AppliedPrismaticJointForce>{}, *workspace, xdot);
    ExpectNear(xdot[0], velocity[0], "state derivative begins with N(q)v");
    ExpectNear(xdot[1], vdot[0], "state derivative ends with ABA vdot");
}

void CheckWrenchExpressionAndAccumulation() {
    constexpr double mass = 2.4;
    constexpr double unit_izz = 0.37;
    MultibodyModel model;
    const auto rotor =
        model.AddRigidBody("expression_rotor",
                           MakeInertia(mass, 0.0, unit_izz));
    const auto expression_carrier =
        model.AddRigidBody("expression_carrier",
                           MakeInertia(0.6, 0.0, 0.11));
    model.AddRevoluteJoint(
        "expression_hinge", model.world_frame(), model.body_frame(rotor),
        Eigen::Vector3d::UnitZ(), 0.0);
    model.AddWeldJoint("expression_carrier_weld", model.world_frame(),
                       model.body_frame(expression_carrier));

    FixedFramePoseParameters expression_pose;
    expression_pose.R_PF =
        Eigen::AngleAxisd(
            0.61, Eigen::Vector3d(0.23, -0.41, 0.88).normalized())
            .toRotationMatrix();
    expression_pose.p_PoFo_P = Eigen::Vector3d(-0.17, 0.29, 0.13);
    const auto expression_frame = model.AddFixedFrame(
        "rotated_expression_frame", expression_carrier, expression_pose);
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();

    auto context = model.CreateDefaultContext();
    Eigen::VectorXd positions(1);
    positions << 0.47;
    model.SetGeneralizedPositions(context.get(), positions);
    auto workspace = model.CreateForwardDynamicsWorkspace();

    const Eigen::Vector3d p_BoQ_B(0.31, -0.22, 0.17);
    const Eigen::Vector3d torque_Q_W(0.46, -0.73, 1.12);
    const Eigen::Vector3d force_W(-1.7, 2.3, 0.58);
    const Eigen::Matrix3d R_WE = expression_pose.R_PF;
    const Eigen::Vector3d torque_Q_E = R_WE.transpose() * torque_Q_W;
    const Eigen::Vector3d force_E = R_WE.transpose() * force_W;

    const std::array wrench_in_world{AppliedBodyWrench{
        rotor, p_BoQ_B, model.world_frame(), torque_Q_W, force_W}};
    const std::array wrench_in_rotated_frame{AppliedBodyWrench{
        rotor, p_BoQ_B, expression_frame, torque_Q_E, force_E}};
    const std::array split_wrench{
        AppliedBodyWrench{rotor, p_BoQ_B, expression_frame,
                          0.35 * torque_Q_E, 0.35 * force_E},
        AppliedBodyWrench{rotor, p_BoQ_B, expression_frame,
                          0.65 * torque_Q_E, 0.65 * force_E}};

    Eigen::VectorXd vdot_world(1);
    Eigen::VectorXd vdot_rotated(1);
    Eigen::VectorXd vdot_split(1);
    model.CalcGeneralizedVelocityDerivatives(
        *context, wrench_in_world, {}, {}, *workspace, vdot_world);
    model.CalcGeneralizedVelocityDerivatives(
        *context, wrench_in_rotated_frame, {}, {}, *workspace, vdot_rotated);
    model.CalcGeneralizedVelocityDerivatives(
        *context, split_wrench, {}, {}, *workspace, vdot_split);

    const Eigen::Matrix3d R_WB =
        Eigen::AngleAxisd(positions[0], Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    const Eigen::Vector3d p_BoQ_W = R_WB * p_BoQ_B;
    const double generalized_effort =
        Eigen::Vector3d::UnitZ().dot(torque_Q_W) +
        Eigen::Vector3d::UnitZ().cross(p_BoQ_W).dot(force_W);
    const double expected_vdot = generalized_effort / (mass * unit_izz);
    ExpectNear(vdot_world[0], expected_vdot,
               "a nontrivial body-point wrench satisfies virtual power");
    ExpectNear(vdot_rotated[0], expected_vdot,
               "the same wrench in a rotated frame has the same dynamics");
    ExpectNear(vdot_split[0], expected_vdot,
               "multiple wrenches on one body accumulate linearly");
}

void CheckWorkspaceReuseAndTypedPrismaticForce() {
    MultibodyModel model;
    const auto body = model.AddRigidBody("slider", MakeInertia(2.0, 0.0, 0.2));
    const auto joint = model.AddPrismaticJoint(
        "rail", model.world_frame(), model.body_frame(body),
        Eigen::Vector3d::UnitX(), 0.0);
    model.SetGravityVector(Eigen::Vector3d(0.0, -9.81, 0.0));
    model.Finalize();
    auto context = model.CreateDefaultContext();
    auto workspace = model.CreateForwardDynamicsWorkspace();
    Eigen::VectorXd vdot(1);

    const std::array first{AppliedPrismaticJointForce{joint, 4.0}};
    const std::array second{AppliedPrismaticJointForce{joint, -2.0}};
    const auto no_wrenches = std::span<const AppliedBodyWrench>{};
    const auto no_torques = std::span<const AppliedRevoluteJointTorque>{};
    model.CalcGeneralizedVelocityDerivatives(
        *context, no_wrenches, no_torques, first, *workspace, vdot);
    ExpectNear(vdot[0], 2.0, "a prismatic force follows F = m a");
    model.CalcGeneralizedVelocityDerivatives(
        *context, no_wrenches, no_torques, second, *workspace, vdot);
    ExpectNear(vdot[0], -1.0, "a reused workspace is fully overwritten");
    model.CalcGeneralizedVelocityDerivatives(
        *context, no_wrenches, no_torques, first, *workspace, vdot);
    ExpectNear(vdot[0], 2.0, "A-B-A reuse leaves no force residue");
}

void CheckMixedTreeAbaRneaRoundTrip() {
    MultibodyModel model;
    const auto welded =
        model.AddRigidBody("welded", MakeInertia(0.8, -0.11, 0.17));
    const auto slider =
        model.AddRigidBody("slider", MakeInertia(1.4, 0.09, 0.24));
    const auto root =
        model.AddRigidBody("root", MakeInertia(2.1, 0.13, 0.31));
    const auto revolute = model.AddRevoluteJoint(
        "root_joint", model.world_frame(), model.body_frame(root),
        Eigen::Vector3d(0.31, -0.47, 0.83), 0.0);

    FixedFramePoseParameters slider_mount_pose;
    slider_mount_pose.R_PF =
        Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitY()).toRotationMatrix();
    slider_mount_pose.p_PoFo_P = Eigen::Vector3d(0.34, -0.16, 0.22);
    const auto slider_mount =
        model.AddFixedFrame("slider_mount", root, slider_mount_pose);
    const auto prismatic = model.AddPrismaticJoint(
        "slider_joint", slider_mount, model.body_frame(slider),
        Eigen::Vector3d(0.91, 0.28, -0.19), 0.0);

    FixedFramePoseParameters weld_mount_pose;
    weld_mount_pose.R_PF =
        Eigen::AngleAxisd(-0.29, Eigen::Vector3d::UnitX()).toRotationMatrix();
    weld_mount_pose.p_PoFo_P = Eigen::Vector3d(-0.21, 0.18, 0.13);
    const auto weld_mount =
        model.AddFixedFrame("weld_mount", slider, weld_mount_pose);
    model.AddWeldJoint("weld_joint", weld_mount, model.body_frame(welded));
    model.SetGravityVector(Eigen::Vector3d::Zero());
    model.Finalize();

    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    positions[model.GetJointPositionRange(revolute).start()] = 0.43;
    positions[model.GetJointPositionRange(prismatic).start()] = -0.24;
    Eigen::VectorXd velocities =
        Eigen::VectorXd::Zero(model.num_generalized_velocities());
    velocities[model.GetJointVelocityRange(revolute).start()] = 0.76;
    velocities[model.GetJointVelocityRange(prismatic).start()] = -0.39;
    auto context = model.CreateDefaultContext();
    model.SetGeneralizedPositions(context.get(), positions);
    model.SetGeneralizedVelocities(context.get(), velocities);

    constexpr double applied_torque = 1.7;
    constexpr double applied_force = -2.4;
    const std::array torques{
        AppliedRevoluteJointTorque{revolute, applied_torque}};
    const std::array forces{
        AppliedPrismaticJointForce{prismatic, applied_force}};
    auto workspace = model.CreateForwardDynamicsWorkspace();
    Eigen::VectorXd vdot(model.num_generalized_velocities());
    model.CalcGeneralizedVelocityDerivatives(
        *context, {}, torques, forces, *workspace, vdot);

    Eigen::VectorXd required(model.num_generalized_velocities());
    model.CalcRequiredGeneralizedForces(*context, vdot, required);
    Eigen::VectorXd applied =
        Eigen::VectorXd::Zero(model.num_generalized_velocities());
    applied[model.GetJointVelocityRange(revolute).start()] = applied_torque;
    applied[model.GetJointVelocityRange(prismatic).start()] = applied_force;
    Expect((required - applied).norm() <=
               4096.0 * std::numeric_limits<double>::epsilon() *
                   std::max(1.0, applied.norm()),
           "ABA and RNEA agree on a rotated revolute-prismatic-weld tree");
}

void CheckWorkspaceAndEffortRefusals() {
    MultibodyModel first_model;
    const auto first_body = first_model.AddRigidBody(
        "first", MakeInertia(1.0, 0.0, 0.2));
    const auto revolute = first_model.AddRevoluteJoint(
        "joint", first_model.world_frame(), first_model.body_frame(first_body),
        Eigen::Vector3d::UnitZ(), 0.0);
    first_model.Finalize();
    auto first_context = first_model.CreateDefaultContext();

    MultibodyModel second_model;
    const auto second_body = second_model.AddRigidBody(
        "second", MakeInertia(1.0, 0.0, 0.2));
    second_model.AddPrismaticJoint(
        "joint", second_model.world_frame(), second_model.body_frame(second_body),
        Eigen::Vector3d::UnitX(), 0.0);
    second_model.Finalize();
    auto foreign_workspace = second_model.CreateForwardDynamicsWorkspace();
    Eigen::VectorXd output = Eigen::VectorXd::Constant(1, 19.0);
    const Eigen::VectorXd before = output;
    bool refused = false;
    try {
        first_model.CalcGeneralizedVelocityDerivatives(
            *first_context, {}, {}, {}, *foreign_workspace, output);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused && output == before,
           "a foreign workspace is refused before output is written");

    auto workspace = first_model.CreateForwardDynamicsWorkspace();
    const std::array wrong_kind{AppliedPrismaticJointForce{revolute, 2.0}};
    refused = false;
    try {
        first_model.CalcGeneralizedVelocityDerivatives(
            *first_context, {}, {}, wrong_kind, *workspace, output);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused && output == before,
           "a typed effort cannot be applied to the wrong joint kind");
}

}  // namespace

int main() {
    CheckOffsetWrenchAndStateDerivative();
    CheckWrenchExpressionAndAccumulation();
    CheckWorkspaceReuseAndTypedPrismaticForce();
    CheckMixedTreeAbaRneaRoundTrip();
    CheckWorkspaceAndEffortRefusals();
    if (failure_count != 0) return 1;
    std::printf("PASS multibody external forces and forward dynamics\n");
    return 0;
}
