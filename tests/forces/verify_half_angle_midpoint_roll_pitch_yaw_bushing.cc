#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/multibody_runtime/multibody_physical_parameters.h"

namespace {

using orvd::forces::ForceElementEnd;
using orvd::forces::HalfAngleMidpointRollPitchYawBushing;
using orvd::forces::VehicleForceElementCollection;
using orvd::forces::VehicleForcePlan;
using orvd::multibody_model::AppliedBodyWrench;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;

constexpr double kMachine = std::numeric_limits<double>::epsilon();
int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

void ExpectNear(double actual, double expected, double tolerance,
                const std::string& description) {
    if (!std::isfinite(actual) || !std::isfinite(expected) ||
        !std::isfinite(tolerance) || tolerance < 0.0 ||
        std::abs(actual - expected) > tolerance) {
        std::printf("FAIL %s: actual %.17g expected %.17g tolerance %.3g\n",
                    description.c_str(), actual, expected, tolerance);
        ++failure_count;
    }
}

void ExpectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected, double tolerance,
                      const std::string& description) {
    const double error = (actual - expected).cwiseAbs().maxCoeff();
    if (!actual.allFinite() || !expected.allFinite() ||
        !std::isfinite(tolerance) || tolerance < 0.0 ||
        !std::isfinite(error) || error > tolerance) {
        std::printf(
            "FAIL %s: max error %.17g tolerance %.3g\n"
            "  actual   %.17g %.17g %.17g\n"
            "  expected %.17g %.17g %.17g\n",
            description.c_str(), error, tolerance, actual.x(), actual.y(),
            actual.z(), expected.x(), expected.y(), expected.z());
        ++failure_count;
    }
}

Eigen::Matrix3d SpaceXyzRotation(const Eigen::Vector3d& angles) {
    return (Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

// The fixture states both marker poses deliberately. This keeps every length,
// rate and angle on an ordinary rail-vehicle scale while ensuring that fixed
// marker rotations, body rotations and the half-angle rotation are all
// different matrices.
struct BushingFixture {
    std::unique_ptr<MultibodyModel> model{std::make_unique<MultibodyModel>()};
    RigidBodyHandle body_a;
    RigidBodyHandle body_c;
    ForceElementEnd frame_a_end;
    ForceElementEnd frame_c_end;
    std::unique_ptr<orvd::multibody_model::MultibodyEvaluationContext> context;

    Eigen::Matrix3d rotation_body_a_frame_a;
    Eigen::Matrix3d rotation_body_c_frame_c;
    Eigen::Vector3d position_body_a_frame_a;
    Eigen::Vector3d position_body_c_frame_c;

    BushingFixture() {
        orvd::multibody_runtime::RigidBodyInertiaParameters inertia;
        inertia.mass_kilograms = 185.0;
        inertia.unit_inertia_moments = Eigen::Vector3d(0.42, 0.36, 0.31);
        body_a = model->AddRigidBody("longitudinal_bar", inertia);
        body_c = model->AddRigidBody("bogie_frame", inertia);

        rotation_body_a_frame_a =
            (Eigen::AngleAxisd(0.13, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(-0.09, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitZ()))
                .toRotationMatrix();
        rotation_body_c_frame_c =
            (Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(0.11, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitZ()))
                .toRotationMatrix();
        position_body_a_frame_a = Eigen::Vector3d(0.12, -0.04, 0.07);
        position_body_c_frame_c = Eigen::Vector3d(-0.08, 0.05, -0.03);

        orvd::multibody_runtime::FixedFramePoseParameters frame_a_pose;
        frame_a_pose.R_PF = rotation_body_a_frame_a;
        frame_a_pose.p_PoFo_P = position_body_a_frame_a;
        const auto frame_a =
            model->AddFixedFrame("frame_a", body_a, frame_a_pose);
        orvd::multibody_runtime::FixedFramePoseParameters frame_c_pose;
        frame_c_pose.R_PF = rotation_body_c_frame_c;
        frame_c_pose.p_PoFo_P = position_body_c_frame_c;
        const auto frame_c =
            model->AddFixedFrame("frame_c", body_c, frame_c_pose);

        model->DeclareFreeBody(body_a);
        model->DeclareFreeBody(body_c);
        model->Finalize();
        frame_a_end = ForceElementEnd{frame_a};
        frame_c_end = ForceElementEnd{frame_c};
        context = model->CreateDefaultContext();
        SetState();
    }

    void SetState() const {
        const Eigen::Matrix3d rotation_world_body_a =
            Eigen::AngleAxisd(0.24,
                              Eigen::Vector3d(0.3, -0.5, 0.8).normalized())
                .toRotationMatrix();
        const Eigen::Vector3d position_world_body_a(0.31, -0.22, 0.48);
        const Eigen::Matrix3d rotation_world_frame_a =
            rotation_world_body_a * rotation_body_a_frame_a;
        const Eigen::Vector3d position_world_frame_a =
            position_world_body_a +
            rotation_world_body_a * position_body_a_frame_a;

        const Eigen::Vector3d relative_angles(0.020, -0.014, 0.018);
        const Eigen::Matrix3d rotation_frame_a_frame_c =
            SpaceXyzRotation(relative_angles);
        const Eigen::Vector3d position_frame_a_frame_c(0.002, -0.001, 0.003);
        const Eigen::Matrix3d rotation_world_frame_c =
            rotation_world_frame_a * rotation_frame_a_frame_c;
        const Eigen::Vector3d position_world_frame_c =
            position_world_frame_a +
            rotation_world_frame_a * position_frame_a_frame_c;
        const Eigen::Matrix3d rotation_world_body_c =
            rotation_world_frame_c * rotation_body_c_frame_c.transpose();
        const Eigen::Vector3d position_world_body_c =
            position_world_frame_c -
            rotation_world_body_c * position_body_c_frame_c;

        Eigen::VectorXd positions =
            Eigen::VectorXd::Zero(model->num_generalized_positions());
        const auto state_body = [&](RigidBodyHandle body,
                                    const Eigen::Matrix3d& rotation,
                                    const Eigen::Vector3d& position) {
            const auto range = model->GetFreeBodyPositionRange(body);
            const Eigen::Quaterniond quaternion(rotation);
            positions.segment<4>(range.start()) <<
                quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
            positions.segment<3>(range.start() + 4) = position;
        };
        state_body(body_a, rotation_world_body_a, position_world_body_a);
        state_body(body_c, rotation_world_body_c, position_world_body_c);

        Eigen::VectorXd velocities =
            Eigen::VectorXd::Zero(model->num_generalized_velocities());
        const auto velocity_body = [&](RigidBodyHandle body,
                                       const Eigen::Vector3d& angular,
                                       const Eigen::Vector3d& translational) {
            const auto range = model->GetFreeBodyVelocityRange(body);
            velocities.segment<3>(range.start()) = angular;
            velocities.segment<3>(range.start() + 3) = translational;
        };
        velocity_body(body_a, Eigen::Vector3d(0.07, -0.04, 0.05),
                      Eigen::Vector3d(0.12, -0.09, 0.06));
        velocity_body(body_c, Eigen::Vector3d(-0.02, 0.05, 0.11),
                      Eigen::Vector3d(-0.04, 0.08, -0.03));
        model->SetGeneralizedState(context.get(), positions, velocities);
    }
};

struct ReconstructedBushing {
    Eigen::Matrix3d rotation_world_frame_a;
    Eigen::Matrix3d rotation_frame_a_frame_b;
    Eigen::Matrix3d rotation_world_frame_b;
    Eigen::Vector3d frame_separation_in_a;
    Eigen::Vector3d frame_separation_in_b;
    Eigen::Vector3d relative_angular_velocity_in_a;
    Eigen::Vector3d relative_midpoint_velocity_in_b;
    Eigen::Vector3d rpy;
    Eigen::Vector3d rpy_rate;
    Eigen::Matrix3d rpy_rate_from_angular_velocity;
    Eigen::Vector3d force_on_c_in_b;
    Eigen::Vector3d generalized_torque_on_c;
    Eigen::Vector3d torque_on_c_in_a;
    Eigen::Vector3d midpoint_in_world;
    Eigen::Vector3d point_on_a_in_body_a;
    Eigen::Vector3d point_on_c_in_body_c;
};

Eigen::Matrix3d HalfAngleRotation(const Eigen::Matrix3d& rotation_a_c) {
    Eigen::Quaterniond q_a_c(rotation_a_c);
    if (q_a_c.w() < 0.0) {
        q_a_c.coeffs() *= -1.0;
    }
    const double half_scalar = std::sqrt(0.5 * (q_a_c.w() + 1.0));
    const Eigen::Quaterniond q_a_b(
        half_scalar, q_a_c.x() / (2.0 * half_scalar),
        q_a_c.y() / (2.0 * half_scalar),
        q_a_c.z() / (2.0 * half_scalar));
    return q_a_b.toRotationMatrix();
}

Eigen::Vector3d SpaceXyzAngles(const Eigen::Matrix3d& rotation_a_c) {
    const double roll = std::atan2(rotation_a_c(2, 1), rotation_a_c(2, 2));
    const double pitch =
        std::atan2(-rotation_a_c(2, 0),
                   std::hypot(rotation_a_c(0, 0), rotation_a_c(1, 0)));
    const double yaw = std::atan2(rotation_a_c(1, 0), rotation_a_c(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Matrix3d SpaceXyzRateMap(double pitch, double yaw) {
    const double cosine_pitch = std::cos(pitch);
    Eigen::Matrix3d map;
    map << std::cos(yaw) / cosine_pitch,
        std::sin(yaw) / cosine_pitch, 0.0, -std::sin(yaw),
        std::cos(yaw), 0.0, std::cos(yaw) * std::tan(pitch),
        std::sin(yaw) * std::tan(pitch), 1.0;
    return map;
}

ReconstructedBushing Reconstruct(
    const BushingFixture& fixture,
    const HalfAngleMidpointRollPitchYawBushing& bushing) {
    const auto pose_world_a = fixture.model->CalcPoseInWorld(
        *fixture.context, fixture.frame_a_end.frame);
    const auto pose_world_c = fixture.model->CalcPoseInWorld(
        *fixture.context, fixture.frame_c_end.frame);
    const auto velocity_world_a =
        fixture.model->CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
            *fixture.context, fixture.frame_a_end.frame);
    const auto velocity_world_c =
        fixture.model->CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
            *fixture.context, fixture.frame_c_end.frame);

    ReconstructedBushing result;
    result.rotation_world_frame_a = pose_world_a.rotation();
    const Eigen::Matrix3d rotation_a_c =
        pose_world_a.rotation().transpose() * pose_world_c.rotation();
    result.rotation_frame_a_frame_b = HalfAngleRotation(rotation_a_c);
    result.rotation_world_frame_b =
        pose_world_a.rotation() * result.rotation_frame_a_frame_b;
    result.frame_separation_in_a =
        pose_world_a.rotation().transpose() *
        (pose_world_c.translation() - pose_world_a.translation());
    result.frame_separation_in_b =
        result.rotation_frame_a_frame_b.transpose() *
        result.frame_separation_in_a;

    const Eigen::Vector3d angular_velocity_a_in_a =
        pose_world_a.rotation().transpose() *
        velocity_world_a.angular_velocity_radians_per_second();
    result.relative_angular_velocity_in_a =
        pose_world_a.rotation().transpose() *
        (velocity_world_c.angular_velocity_radians_per_second() -
         velocity_world_a.angular_velocity_radians_per_second());
    const Eigen::Vector3d frame_separation_rate_in_a =
        pose_world_a.rotation().transpose() *
            (velocity_world_c
                 .translational_velocity_at_frame_origin_meters_per_second() -
             velocity_world_a
                 .translational_velocity_at_frame_origin_meters_per_second()) -
        angular_velocity_a_in_a.cross(result.frame_separation_in_a);
    result.relative_midpoint_velocity_in_b =
        result.rotation_frame_a_frame_b.transpose() *
        (frame_separation_rate_in_a -
         0.5 * result.relative_angular_velocity_in_a.cross(
                   result.frame_separation_in_a));

    result.rpy = SpaceXyzAngles(rotation_a_c);
    result.rpy_rate_from_angular_velocity =
        SpaceXyzRateMap(result.rpy.y(), result.rpy.z());
    result.rpy_rate = result.rpy_rate_from_angular_velocity *
                      result.relative_angular_velocity_in_a;
    result.force_on_c_in_b =
        -(bushing.translational_stiffness_newtons_per_meter.cwiseProduct(
              result.frame_separation_in_b) +
          bushing.translational_damping_newton_seconds_per_meter.cwiseProduct(
              result.relative_midpoint_velocity_in_b));
    result.generalized_torque_on_c =
        -(bushing.rotational_stiffness_newton_meters_per_radian.cwiseProduct(
              result.rpy) +
          bushing.rotational_damping_newton_meter_seconds_per_radian
              .cwiseProduct(result.rpy_rate));
    result.torque_on_c_in_a =
        result.rpy_rate_from_angular_velocity.transpose() *
        result.generalized_torque_on_c;
    result.midpoint_in_world =
        0.5 * (pose_world_a.translation() + pose_world_c.translation());

    const auto pose_world_body_a =
        fixture.model->CalcPoseInWorld(*fixture.context, fixture.body_a);
    const auto pose_world_body_c =
        fixture.model->CalcPoseInWorld(*fixture.context, fixture.body_c);
    result.point_on_a_in_body_a =
        pose_world_body_a.rotation().transpose() *
        (result.midpoint_in_world - pose_world_body_a.translation());
    result.point_on_c_in_body_c =
        pose_world_body_c.rotation().transpose() *
        (result.midpoint_in_world - pose_world_body_c.translation());
    return result;
}

std::array<AppliedBodyWrench, 2> Evaluate(
    const BushingFixture& fixture,
    const HalfAngleMidpointRollPitchYawBushing& bushing) {
    VehicleForceElementCollection elements;
    elements.half_angle_midpoint_roll_pitch_yaw_bushings.push_back(bushing);
    const VehicleForcePlan plan(*fixture.model, std::move(elements));
    Expect(plan.half_angle_midpoint_roll_pitch_yaw_bushing_count() == 1 &&
               plan.body_wrench_count() == 2 &&
               plan.series_spring_damper_force_state_count() == 0 &&
               plan.nominal_force_component_count() == 0,
           "the bushing contributes one typed element, two wrenches and no "
           "runtime state or nominal-force slot");
    std::array<AppliedBodyWrench, 2> wrenches;
    Eigen::VectorXd no_series_state(0);
    Eigen::VectorXd no_nominal_force(0);
    Eigen::VectorXd no_series_derivative(0);
    plan.CalcAppliedForces(*fixture.context, no_series_state, no_nominal_force,
                           wrenches, no_series_derivative);
    return wrenches;
}

Eigen::Vector3d WorldPoint(const BushingFixture& fixture,
                           const AppliedBodyWrench& wrench) {
    const auto pose =
        fixture.model->CalcPoseInWorld(*fixture.context, wrench.body);
    return pose.translation() +
           pose.rotation() * wrench.point_position_in_body_frame_meters;
}

double AppliedPower(const BushingFixture& fixture,
                    const std::array<AppliedBodyWrench, 2>& wrenches) {
    double power = 0.0;
    for (const AppliedBodyWrench& wrench : wrenches) {
        const auto pose =
            fixture.model->CalcPoseInWorld(*fixture.context, wrench.body);
        const auto& model = *fixture.model;
        const auto velocity =
            model.CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *fixture.context, wrench.body);
        const Eigen::Vector3d offset_world =
            pose.rotation() * wrench.point_position_in_body_frame_meters;
        const Eigen::Vector3d point_velocity =
            velocity.translational_velocity_at_frame_origin_meters_per_second() +
            velocity.angular_velocity_radians_per_second().cross(offset_world);
        const Eigen::Matrix3d rotation_world_expressed =
            fixture.model
                ->CalcPoseInWorld(*fixture.context, wrench.expressed_in_frame)
                .rotation();
        const Eigen::Vector3d force_world =
            rotation_world_expressed * wrench.force_newtons;
        const Eigen::Vector3d torque_world =
            rotation_world_expressed * wrench.torque_about_point_newton_metres;
        power += force_world.dot(point_velocity) +
                 torque_world.dot(
                     velocity.angular_velocity_radians_per_second());
    }
    return power;
}

double GeneralizedVirtualPower(
    const BushingFixture& fixture,
    const std::array<AppliedBodyWrench, 2>& wrenches) {
    Eigen::VectorXd generalized_force =
        Eigen::VectorXd::Zero(fixture.model->num_generalized_velocities());
    for (const AppliedBodyWrench& wrench : wrenches) {
        Eigen::MatrixXd angular_jacobian(
            3, fixture.model->num_generalized_velocities());
        Eigen::MatrixXd point_jacobian(
            3, fixture.model->num_generalized_velocities());
        fixture.model
            ->CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                *fixture.context, wrench.body,
                wrench.point_position_in_body_frame_meters, &angular_jacobian,
                &point_jacobian);
        const Eigen::Matrix3d rotation_world_expressed =
            fixture.model
                ->CalcPoseInWorld(*fixture.context, wrench.expressed_in_frame)
                .rotation();
        generalized_force +=
            angular_jacobian.transpose() *
                (rotation_world_expressed *
                 wrench.torque_about_point_newton_metres) +
            point_jacobian.transpose() *
                (rotation_world_expressed * wrench.force_newtons);
    }
    return generalized_force.dot(fixture.context->generalized_velocities());
}

void CheckGeneralSixComponentLaw() {
    const BushingFixture fixture;
    const HalfAngleMidpointRollPitchYawBushing bushing{
        "general_six_component", fixture.frame_a_end, fixture.frame_c_end,
        Eigen::Vector3d(1200.0, 900.0, 700.0),
        Eigen::Vector3d(80.0, 110.0, 60.0),
        Eigen::Vector3d(8.0e7, 1.0e8, 1.2e8),
        Eigen::Vector3d(90.0, 120.0, 75.0)};
    const ReconstructedBushing expected = Reconstruct(fixture, bushing);
    const auto wrenches = Evaluate(fixture, bushing);
    const Eigen::Vector3d expected_force_world =
        expected.rotation_world_frame_b * expected.force_on_c_in_b;
    const Eigen::Vector3d expected_torque_world =
        expected.rotation_world_frame_a * expected.torque_on_c_in_a;
    const double force_tolerance =
        4096.0 * kMachine * std::max(1.0, expected_force_world.norm());
    const double torque_tolerance =
        4096.0 * kMachine * std::max(1.0, expected_torque_world.norm());
    ExpectVectorNear(wrenches[1].force_newtons, expected_force_world,
                     force_tolerance,
                     "all three independent translational coefficients reach "
                     "the C-side force");
    ExpectVectorNear(wrenches[1].torque_about_point_newton_metres,
                     expected_torque_world, torque_tolerance,
                     "non-zero rotational stiffness and damping reach the "
                     "mapped physical torque");
    const Eigen::Vector3d elastic_generalized_torque =
        bushing.rotational_stiffness_newton_meters_per_radian.cwiseProduct(
            expected.rpy);
    const Eigen::Vector3d viscous_generalized_torque =
        bushing.rotational_damping_newton_meter_seconds_per_radian
            .cwiseProduct(expected.rpy_rate);
    Expect(elastic_generalized_torque.norm() > 1.0 &&
               viscous_generalized_torque.norm() > 1.0,
           "both rotational stiffness and damping are measurable on the "
           "general six-component fixture");
    const double constitutive_power =
        expected.force_on_c_in_b.dot(
            expected.relative_midpoint_velocity_in_b) +
        expected.generalized_torque_on_c.dot(expected.rpy_rate);
    const double applied_power = AppliedPower(fixture, wrenches);
    const double power_tolerance =
        16384.0 * kMachine *
        std::max({1.0, std::abs(constitutive_power),
                  std::abs(applied_power)});
    ExpectNear(applied_power, constitutive_power, power_tolerance,
               "the general six-component law closes virtual power without "
               "claiming an anisotropic translational energy identity");
}

void CheckFrozenIrwBushingWrenchAndPower() {
    const BushingFixture fixture;
    const Eigen::Vector3d rotational_stiffness = Eigen::Vector3d::Zero();
    const Eigen::Vector3d rotational_damping =
        Eigen::Vector3d::Constant(100.0);
    const Eigen::Vector3d translational_stiffness =
        Eigen::Vector3d::Constant(1.0e8);
    const Eigen::Vector3d translational_damping =
        Eigen::Vector3d::Constant(100.0);
    const HalfAngleMidpointRollPitchYawBushing bushing{
        "irw_longitudinal_bar_fixed", fixture.frame_a_end,
        fixture.frame_c_end, rotational_stiffness, rotational_damping,
        translational_stiffness, translational_damping};
    const ReconstructedBushing expected = Reconstruct(fixture, bushing);
    const auto wrenches = Evaluate(fixture, bushing);

    Expect(expected.frame_separation_in_b.norm() > 1.0e-3 &&
               expected.relative_midpoint_velocity_in_b.norm() > 1.0e-2 &&
               expected.rpy.norm() > 1.0e-2 &&
               expected.rpy_rate.norm() > 1.0e-2,
           "the fixture exercises millimetre translation, ordinary velocity, "
           "non-zero RPY and non-zero RPY rate");
    const Eigen::Vector3d half_angle_transport =
        0.5 * expected.relative_angular_velocity_in_a.cross(
                  expected.frame_separation_in_a);
    Expect(half_angle_transport.norm() > 1.0e-5,
           "the -0.5 omega cross p half-angle transport term is measurable");
    Expect((expected.rpy_rate - expected.relative_angular_velocity_in_a).norm() >
               1.0e-3,
           "the fixture distinguishes space-XYZ rates from angular-velocity "
           "components");

    const Eigen::Vector3d force_on_c_world =
        expected.rotation_world_frame_b * expected.force_on_c_in_b;
    const Eigen::Vector3d torque_on_c_world =
        expected.rotation_world_frame_a * expected.torque_on_c_in_a;
    const double force_tolerance =
        4096.0 * kMachine * std::max(1.0, force_on_c_world.norm());
    const double torque_tolerance =
        4096.0 * kMachine * std::max(1.0, torque_on_c_world.norm());
    const double point_tolerance = 4096.0 * kMachine;

    Expect(wrenches[0].body == fixture.body_a &&
               wrenches[1].body == fixture.body_c &&
               wrenches[0].expressed_in_frame == fixture.model->world_frame() &&
               wrenches[1].expressed_in_frame == fixture.model->world_frame(),
           "the pair targets A then C and is expressed in the world frame");
    ExpectVectorNear(wrenches[0].force_newtons, -force_on_c_world,
                     force_tolerance, "frame A receives minus the B-frame law");
    ExpectVectorNear(wrenches[1].force_newtons, force_on_c_world,
                     force_tolerance, "frame C receives the B-frame law");
    ExpectVectorNear(wrenches[0].torque_about_point_newton_metres,
                     -torque_on_c_world, torque_tolerance,
                     "frame A receives minus the mapped physical torque");
    ExpectVectorNear(wrenches[1].torque_about_point_newton_metres,
                     torque_on_c_world, torque_tolerance,
                     "frame C receives the mapped physical torque");
    ExpectVectorNear(wrenches[0].point_position_in_body_frame_meters,
                     expected.point_on_a_in_body_a, point_tolerance,
                     "the A wrench lands at the instantaneous common midpoint");
    ExpectVectorNear(wrenches[1].point_position_in_body_frame_meters,
                     expected.point_on_c_in_body_c, point_tolerance,
                     "the C wrench lands at the instantaneous common midpoint");
    ExpectVectorNear(WorldPoint(fixture, wrenches[0]),
                     expected.midpoint_in_world, point_tolerance,
                     "A's material application point is the common midpoint");
    ExpectVectorNear(WorldPoint(fixture, wrenches[1]),
                     expected.midpoint_in_world, point_tolerance,
                     "C's material application point is the common midpoint");

    const Eigen::Vector3d arbitrary_origin(-0.37, 0.82, -0.19);
    const Eigen::Vector3d total_force =
        wrenches[0].force_newtons + wrenches[1].force_newtons;
    const Eigen::Vector3d total_moment =
        wrenches[0].torque_about_point_newton_metres +
        wrenches[1].torque_about_point_newton_metres +
        (WorldPoint(fixture, wrenches[0]) - arbitrary_origin)
            .cross(wrenches[0].force_newtons) +
        (WorldPoint(fixture, wrenches[1]) - arbitrary_origin)
            .cross(wrenches[1].force_newtons);
    ExpectVectorNear(total_force, Eigen::Vector3d::Zero(), force_tolerance,
                     "the bushing pair has zero resultant force");
    ExpectVectorNear(total_moment, Eigen::Vector3d::Zero(),
                     force_tolerance,
                     "the common-midpoint pair has zero resultant moment");

    const double constitutive_power =
        expected.force_on_c_in_b.dot(
            expected.relative_midpoint_velocity_in_b) +
        expected.generalized_torque_on_c.dot(expected.rpy_rate);
    const double applied_power = AppliedPower(fixture, wrenches);
    const double generalized_power =
        GeneralizedVirtualPower(fixture, wrenches);
    const double power_scale =
        std::max({1.0, std::abs(constitutive_power), std::abs(applied_power)});
    const double power_tolerance = 16384.0 * kMachine * power_scale;
    ExpectNear(applied_power, constitutive_power, power_tolerance,
               "common-midpoint wrench power equals the independently "
               "reconstructed constitutive power");
    ExpectNear(generalized_power, applied_power, power_tolerance,
               "Jacobian-projected generalized virtual power equals spatial "
               "wrench power");

    const double translational_potential_rate =
        translational_stiffness.cwiseProduct(
            expected.frame_separation_in_b)
            .dot(expected.relative_midpoint_velocity_in_b);
    const double dissipation =
        expected.relative_midpoint_velocity_in_b.dot(
            translational_damping.cwiseProduct(
                expected.relative_midpoint_velocity_in_b)) +
        expected.rpy_rate.dot(
            rotational_damping.cwiseProduct(expected.rpy_rate));
    Expect(dissipation > 1.0,
           "both frozen viscous laws contribute non-negative, measurable "
           "dissipation on the fixture");
    ExpectNear(-applied_power, translational_potential_rate + dissipation,
               power_tolerance,
               "the frozen isotropic IRW stiffness closes the elastic-energy "
               "rate plus viscous dissipation account");
}

}  // namespace

int main() {
    CheckGeneralSixComponentLaw();
    CheckFrozenIrwBushingWrenchAndPower();
    if (failure_count != 0) {
        std::printf("%d half-angle midpoint bushing checks failed\n",
                    failure_count);
        return 1;
    }
    std::puts("Half-angle midpoint roll-pitch-yaw bushing checks passed");
    return 0;
}
