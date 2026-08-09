// The public Ball-RPY contract, exercised in the topology that consumes it:
// a free axle bridge carrying a massive longitudinal bar. Expectations use
// elementary rigid-body formulas and virtual power, not a second Ball joint.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::AppliedBodyWrench;
using orvd::multibody_model::FrameHandle;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyEvaluationContext;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

void ExpectMatrixNear(const Eigen::MatrixXd& actual,
                      const Eigen::MatrixXd& expected, double tolerance,
                      const std::string& description) {
    if (actual.rows() != expected.rows() || actual.cols() != expected.cols()) {
        Expect(false, description + ": shape differs");
        return;
    }
    const double scale = std::max(1.0, expected.cwiseAbs().maxCoeff());
    const double error = (actual - expected).cwiseAbs().maxCoeff();
    Expect(error <= tolerance * scale,
           description + ": error " + std::to_string(error));
}

template <typename Attempt>
std::string RefusalMessage(Attempt&& attempt) {
    try {
        attempt();
    } catch (const std::exception& reason) {
        return reason.what();
    }
    return {};
}

Eigen::Matrix3d RotationFromRollPitchYaw(const Eigen::Vector3d& angles) {
    return Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ())
               .toRotationMatrix() *
           Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY())
               .toRotationMatrix() *
           Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX())
               .toRotationMatrix();
}

RigidBodyInertiaParameters MakeAxleBridgeInertia() {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 615.0;
    inertia.center_of_mass_in_body_frame = Eigen::Vector3d(0.02, -0.01, 0.04);
    inertia.unit_inertia_moments = Eigen::Vector3d(0.39, 0.76, 0.58);
    inertia.unit_inertia_products = Eigen::Vector3d(0.006, -0.004, 0.003);
    return inertia;
}

RigidBodyInertiaParameters MakeLongitudinalBarInertia() {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 23.958904;
    inertia.center_of_mass_in_body_frame =
        Eigen::Vector3d(-0.018, 0.006, -0.011);
    const Eigen::Matrix3d central_unit_inertia =
        (Eigen::Vector3d(1.38411, 0.0140195, 1.38411) /
         inertia.mass_kilograms)
            .asDiagonal();
    const Eigen::Vector3d& center = inertia.center_of_mass_in_body_frame;
    const Eigen::Matrix3d unit_inertia_about_origin =
        central_unit_inertia +
        center.squaredNorm() * Eigen::Matrix3d::Identity() -
        center * center.transpose();
    inertia.unit_inertia_moments = unit_inertia_about_origin.diagonal();
    inertia.unit_inertia_products =
        Eigen::Vector3d(unit_inertia_about_origin(0, 1),
                        unit_inertia_about_origin(0, 2),
                        unit_inertia_about_origin(1, 2));
    return inertia;
}

class BallRpyFixture {
   public:
    BallRpyFixture() {
        axle_bridge =
            model.AddRigidBody("axle_bridge", MakeAxleBridgeInertia());
        longitudinal_bar = model.AddRigidBody("longitudinal_bar",
                                              MakeLongitudinalBarInertia());

        parent_pose.R_PF = RotationFromRollPitchYaw(
            Eigen::Vector3d(0.05, -0.07, 0.13));
        parent_pose.p_PoFo_P = Eigen::Vector3d(0.31, -0.18, 0.22);
        parent_frame = model.AddFixedFrame("axle_bridge_ball_frame",
                                           axle_bridge, parent_pose);

        child_pose.R_PF = RotationFromRollPitchYaw(
            Eigen::Vector3d(-0.09, 0.04, -0.11));
        child_pose.p_PoFo_P = Eigen::Vector3d(-0.27, 0.14, -0.19);
        child_frame = model.AddFixedFrame("longitudinal_bar_ball_frame",
                                          longitudinal_bar, child_pose);

        model.DeclareFreeBody(axle_bridge);
        ball_joint = model.AddBallRpyJoint(
            "axle_bridge_to_longitudinal_bar", parent_frame, child_frame,
            default_angles);
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();
    }

    std::unique_ptr<MultibodyEvaluationContext> MakeContext(
        const Eigen::Vector3d& angles,
        const Eigen::Vector3d& angular_velocity) const {
        auto context = model.CreateDefaultContext();
        Eigen::VectorXd positions = context->generalized_positions();
        positions.segment<3>(model.GetJointPositionRange(ball_joint).start()) =
            angles;
        Eigen::VectorXd velocities = Eigen::VectorXd::Zero(
            model.num_generalized_velocities());
        velocities.segment<3>(
            model.GetJointVelocityRange(ball_joint).start()) =
            angular_velocity;
        model.SetGeneralizedState(context.get(), positions, velocities);
        return context;
    }

    MultibodyModel model;
    RigidBodyHandle axle_bridge;
    RigidBodyHandle longitudinal_bar;
    FrameHandle parent_frame;
    FrameHandle child_frame;
    JointHandle ball_joint;
    FixedFramePoseParameters parent_pose;
    FixedFramePoseParameters child_pose;
    const Eigen::Vector3d default_angles{-0.0010084428852911165,
                                         -2.7105054312137605e-20,
                                         -4.573471908445189};
};

void CheckTopologyDefaultsAndPose() {
    BallRpyFixture fixture;
    const auto q_range =
        fixture.model.GetJointPositionRange(fixture.ball_joint);
    const auto v_range =
        fixture.model.GetJointVelocityRange(fixture.ball_joint);
    Expect(q_range.size() == 3 && q_range.start() == 7,
           "Ball positions occupy a named three-entry range after free-body q");
    Expect(v_range.size() == 3 && v_range.start() == 6,
           "Ball velocities occupy a distinct three-entry range after free-body v");
    Expect(fixture.model.num_generalized_positions() == 10 &&
               fixture.model.num_generalized_velocities() == 9,
           "the IRW-shaped free-body plus Ball topology has 10q and 9v");
    Expect(fixture.model.GetJointName(fixture.ball_joint) ==
               "axle_bridge_to_longitudinal_bar",
           "the Ball joint retains its public name");

    const auto context = fixture.model.CreateDefaultContext();
    Expect((context->generalized_positions()
                .segment<3>(q_range.start())
                .array() == fixture.default_angles.array())
               .all(),
           "default RPY values, including the frozen yaw branch, are exact");

    const Eigen::Matrix3d R_FM =
        RotationFromRollPitchYaw(fixture.default_angles);
    const Eigen::Matrix3d R_WB = fixture.parent_pose.R_PF * R_FM *
                                 fixture.child_pose.R_PF.transpose();
    const Eigen::Vector3d p_WBo =
        fixture.parent_pose.p_PoFo_P -
        R_WB * fixture.child_pose.p_PoFo_P;
    const auto pose = fixture.model.CalcPoseInWorld(
        *context, fixture.longitudinal_bar);
    constexpr double tolerance =
        256.0 * std::numeric_limits<double>::epsilon();
    ExpectMatrixNear(pose.rotation(), R_WB, tolerance,
                     "R_FM follows Rz(yaw) Ry(pitch) Rx(roll)");
    ExpectMatrixNear(pose.translation(), p_WBo, tolerance,
                     "nonzero joint frames keep their origins coincident");
}

void CheckVelocityAndPositionDerivativeMaps() {
    BallRpyFixture fixture;
    const Eigen::Vector3d angles{0.012, -0.009, -4.571};
    const Eigen::Vector3d angular_velocity{0.31, -0.27, 0.19};
    auto context = fixture.MakeContext(angles, angular_velocity);

    const auto relative_velocity =
        fixture.model.CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
            *context, fixture.child_frame, fixture.parent_frame,
            fixture.parent_frame);
    constexpr double tolerance =
        512.0 * std::numeric_limits<double>::epsilon();
    ExpectMatrixNear(relative_velocity.angular_velocity_radians_per_second(),
                     angular_velocity, tolerance,
                     "Ball velocity is physical omega_FM expressed in F");
    ExpectMatrixNear(
        relative_velocity
            .translational_velocity_at_frame_origin_meters_per_second(),
        Eigen::Vector3d::Zero(), tolerance,
        "the two Ball frame origins have zero relative translation velocity");

    Eigen::VectorXd qdot(fixture.model.num_generalized_positions());
    fixture.model.MapGeneralizedVelocitiesToPositionDerivatives(
        *context, context->generalized_velocities(), &qdot);

    const double pitch = angles.y();
    const double yaw = angles.z();
    Eigen::Matrix3d angular_velocity_from_rpy_rates;
    angular_velocity_from_rpy_rates <<
        std::cos(yaw) * std::cos(pitch), -std::sin(yaw), 0.0,
        std::sin(yaw) * std::cos(pitch), std::cos(yaw), 0.0,
        -std::sin(pitch), 0.0, 1.0;
    const Eigen::Vector3d expected_rpy_rates =
        angular_velocity_from_rpy_rates.fullPivLu().solve(angular_velocity);
    const auto q_range =
        fixture.model.GetJointPositionRange(fixture.ball_joint);
    ExpectMatrixNear(qdot.segment<3>(q_range.start()), expected_rpy_rates,
                     2048.0 * std::numeric_limits<double>::epsilon(),
                     "qdot is the independently reconstructed N(q) times v");
    Expect((expected_rpy_rates - angular_velocity).norm() > 0.1,
           "the fixture distinguishes angular velocity from RPY rates");

    Eigen::VectorXd mapped_back(fixture.model.num_generalized_velocities());
    fixture.model.MapGeneralizedPositionDerivativesToVelocities(
        *context, qdot, &mapped_back);
    ExpectMatrixNear(mapped_back, context->generalized_velocities(),
                     2048.0 * std::numeric_limits<double>::epsilon(),
                     "the public qdot-to-v map recovers the supplied velocity");

    Eigen::VectorXd damping_force(fixture.model.num_generalized_velocities());
    fixture.model.CalcJointDampingAppliedGeneralizedForces(*context,
                                                            damping_force);
    Expect(damping_force == Eigen::VectorXd::Zero(damping_force.size()),
           "the frozen Ball joint contributes no internal damping force");
}

void CheckMassAndForwardInverseDynamics() {
    BallRpyFixture fixture;
    const Eigen::Vector3d angles{0.018, -0.014, -4.569};
    auto context = fixture.MakeContext(angles, Eigen::Vector3d::Zero());
    const int nv = fixture.model.num_generalized_velocities();

    Eigen::MatrixXd mass_matrix(nv, nv);
    fixture.model.CalcGeneralizedMassMatrix(*context, mass_matrix);
    ExpectMatrixNear(mass_matrix, mass_matrix.transpose(),
                     4096.0 * std::numeric_limits<double>::epsilon(),
                     "the Ball topology mass matrix is symmetric");
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(mass_matrix);
    Expect(eigensolver.info() == Eigen::Success &&
               eigensolver.eigenvalues().minCoeff() > 0.0,
           "massive axle bridge and bar give a positive mass matrix");

    const Eigen::Vector3d point_in_bar(0.12, -0.04, 0.08);
    const Eigen::Vector3d torque_world(37.0, -22.0, 16.0);
    const Eigen::Vector3d force_world(125.0, -81.0, 54.0);
    Eigen::MatrixXd angular_jacobian(3, nv);
    Eigen::MatrixXd point_jacobian(3, nv);
    fixture.model
        .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
            *context, fixture.longitudinal_bar, point_in_bar,
            &angular_jacobian, &point_jacobian);
    const Eigen::VectorXd expected_generalized_force =
        angular_jacobian.transpose() * torque_world +
        point_jacobian.transpose() * force_world;

    const std::array applied_wrenches{AppliedBodyWrench{
        fixture.longitudinal_bar, point_in_bar, fixture.model.world_frame(),
        torque_world, force_world}};
    auto workspace = fixture.model.CreateForwardDynamicsWorkspace();
    Eigen::VectorXd acceleration(nv);
    fixture.model.CalcGeneralizedVelocityDerivatives(
        *context, applied_wrenches, {}, {}, *workspace, acceleration);

    Eigen::VectorXd required(nv);
    fixture.model.CalcRequiredGeneralizedForces(*context, acceleration,
                                                required);
    constexpr double dynamics_tolerance = 2.0e-11;
    ExpectMatrixNear(required, expected_generalized_force,
                     dynamics_tolerance,
                     "forward and inverse dynamics recover virtual-power force");
    ExpectMatrixNear(mass_matrix * acceleration, expected_generalized_force,
                     dynamics_tolerance,
                     "the public mass matrix closes the zero-speed Ball dynamics");
}

void CheckRefusals() {
    MultibodyModel finite_model;
    const auto finite_body = finite_model.AddRigidBody(
        "finite_body", MakeLongitudinalBarInertia());
    const Eigen::Vector3d nonfinite(
        0.0, std::numeric_limits<double>::quiet_NaN(), 0.0);
    const std::string nonfinite_message = RefusalMessage([&] {
        finite_model.AddBallRpyJoint(
            "finite_ball", finite_model.world_frame(),
            finite_model.body_frame(finite_body), nonfinite);
    });
    Expect(nonfinite_message.find("finite_ball") != std::string::npos &&
               nonfinite_message.find("finite") != std::string::npos,
           "a non-finite default angle is refused by joint name");
    finite_model.AddBallRpyJoint(
        "finite_ball", finite_model.world_frame(),
        finite_model.body_frame(finite_body), Eigen::Vector3d::Zero());
    finite_model.Finalize();

    MultibodyModel reversed_model;
    const auto reversed_body = reversed_model.AddRigidBody(
        "reversed_body", MakeLongitudinalBarInertia());
    reversed_model.AddBallRpyJoint(
        "reverse_ball", reversed_model.body_frame(reversed_body),
        reversed_model.world_frame(), Eigen::Vector3d::Zero());
    const std::string reversed_message =
        RefusalMessage([&] { reversed_model.Finalize(); });
    Expect(reversed_message.find("reverse_ball") != std::string::npos &&
               reversed_message.find("reversed") != std::string::npos,
           "a Ball relation requiring reverse traversal fails loudly by name");
}

}  // namespace

int main() {
    CheckTopologyDefaultsAndPose();
    CheckVelocityAndPositionDerivativeMaps();
    CheckMassAndForwardInverseDynamics();
    CheckRefusals();
    if (failure_count != 0) return 1;
    std::printf("PASS public Ball-RPY modelling and dynamics contract\n");
    return 0;
}
