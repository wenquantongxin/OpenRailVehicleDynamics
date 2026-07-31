#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_model.h"

namespace {

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
    const double tolerance = 512.0 * std::numeric_limits<double>::epsilon() *
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

void CheckSingleRevoluteDynamics() {
    constexpr double mass = 2.0;
    constexpr double com_x = 0.4;
    constexpr double central_unit_izz = 0.2;
    constexpr double damping = 0.7;
    constexpr double velocity = 1.5;
    constexpr double acceleration = 2.3;

    MultibodyModel model;
    const auto body =
        model.AddRigidBody("pendulum", MakeInertia(mass, com_x,
                                                   central_unit_izz));
    model.AddRevoluteJoint("hinge", model.world_frame(),
                           model.body_frame(body), Eigen::Vector3d::UnitZ(),
                           damping);
    model.SetGravityVector(Eigen::Vector3d(0.0, -9.81, 0.0));
    model.Finalize();
    auto context = model.CreateDefaultContext();
    model.SetGeneralizedPositions(context.get(), Eigen::VectorXd::Zero(1));
    Eigen::VectorXd velocities(1);
    velocities << velocity;
    model.SetGeneralizedVelocities(context.get(), velocities);

    Eigen::VectorXd bias(1), gravity(1), damping_force(1), required(1);
    const double* const required_storage = required.data();
    Eigen::VectorXd vdot(1);
    vdot << acceleration;
    model.CalcVelocityBiasGeneralizedForces(*context, bias);
    model.CalcGravityAppliedGeneralizedForces(*context, gravity);
    model.CalcJointDampingAppliedGeneralizedForces(*context, damping_force);
    model.CalcRequiredGeneralizedForces(*context, vdot, required);

    const double inertia_about_axis =
        mass * (central_unit_izz + com_x * com_x);
    const double expected_gravity = -mass * 9.81 * com_x;
    const double expected_damping = -damping * velocity;
    const double expected_required = inertia_about_axis * acceleration -
                                     expected_gravity - expected_damping;
    ExpectNear(bias[0], 0.0, "single-axis velocity bias is zero");
    ExpectNear(gravity[0], expected_gravity,
               "gravity has the applied-force sign");
    ExpectNear(damping_force[0], expected_damping,
               "joint damping opposes velocity");
    ExpectNear(required[0], expected_required,
               "inverse dynamics follows the documented force balance");
    Expect(required.data() == required_storage,
           "inverse dynamics retains caller-owned output storage");

    Eigen::MatrixXd mass_matrix(1, 1);
    model.CalcGeneralizedMassMatrix(*context, mass_matrix);
    ExpectNear(required[0], mass_matrix(0, 0) * acceleration + bias[0] -
                                gravity[0] - damping_force[0],
               "the four public dynamics quantities close their balance");
}

void CheckRefusalsAndEmptyModel() {
    MultibodyModel model;
    model.Finalize();
    auto context = model.CreateDefaultContext();
    Eigen::VectorXd empty_output;
    const Eigen::VectorXd empty_input;
    model.CalcVelocityBiasGeneralizedForces(*context, empty_output);
    model.CalcGravityAppliedGeneralizedForces(*context, empty_output);
    model.CalcJointDampingAppliedGeneralizedForces(*context, empty_output);
    model.CalcRequiredGeneralizedForces(*context, empty_input, empty_output);
    Expect(empty_output.size() == 0,
           "a zero-velocity model has empty force vectors");

    MultibodyModel one;
    const auto body = one.AddRigidBody("body", MakeInertia(1.0, 0.1, 0.2));
    one.AddRevoluteJoint("joint", one.world_frame(), one.body_frame(body),
                         Eigen::Vector3d::UnitZ(), 0.0);
    one.Finalize();
    auto one_context = one.CreateDefaultContext();
    Eigen::VectorXd wrong = Eigen::VectorXd::Constant(2, 17.0);
    const Eigen::VectorXd before = wrong;
    bool refused = false;
    try {
        one.CalcVelocityBiasGeneralizedForces(*one_context, wrong);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused && wrong == before,
           "wrong output size is refused before writing");

    Eigen::VectorXd nonfinite(1), output = Eigen::VectorXd::Constant(1, 23.0);
    nonfinite << std::numeric_limits<double>::quiet_NaN();
    const Eigen::VectorXd output_before = output;
    refused = false;
    try {
        one.CalcRequiredGeneralizedForces(*one_context, nonfinite, output);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    Expect(refused && output == output_before,
           "non-finite acceleration is refused before writing");
}

}  // namespace

int main() {
    CheckSingleRevoluteDynamics();
    CheckRefusalsAndEmptyModel();
    if (failure_count != 0) return 1;
    std::printf("PASS multibody inverse dynamics and force elements\n");
    return 0;
}
