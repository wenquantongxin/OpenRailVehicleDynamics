#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::AppliedBodyWrench;
using orvd::multibody_model::AppliedPrismaticJointForce;
using orvd::multibody_model::AppliedRevoluteJointTorque;
using orvd::multibody_model::MultibodyModel;
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
    CheckWorkspaceReuseAndTypedPrismaticForce();
    CheckWorkspaceAndEffortRefusals();
    if (failure_count != 0) return 1;
    std::printf("PASS multibody external forces and forward dynamics\n");
    return 0;
}
