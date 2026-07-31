// Differential kinematics through the public modelling boundary.
//
// Expectations are independent rigid-body identities: the quaternion map is
// written out, Jacobian columns come from central differences of the public
// pose query, and full acceleration comes from central differences of the
// public spatial-velocity query. A(vdot)=A(0)+J*vdot is a secondary identity.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::FrameHandle;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyEvaluationContext;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

int failure_count = 0;

void ExpectTrue(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

template <typename Attempt>
bool RefusalMentions(Attempt&& attempt, std::string_view fragment) {
    try {
        attempt();
    } catch (const std::exception& why) {
        return std::string_view(why.what()).find(fragment) !=
               std::string_view::npos;
    }
    return false;
}

RigidBodyInertiaParameters PhysicalInertia(double mass) {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass;
    inertia.center_of_mass_in_body_frame = Eigen::Vector3d(0.04, -0.03, 0.02);
    inertia.unit_inertia_moments = Eigen::Vector3d(0.22, 0.27, 0.31);
    inertia.unit_inertia_products = Eigen::Vector3d(0.01, -0.008, 0.006);
    return inertia;
}

Eigen::Matrix3d RotationAboutY(double angle) {
    Eigen::Matrix3d rotation;
    rotation << std::cos(angle), 0.0, std::sin(angle), 0.0, 1.0, 0.0,
        -std::sin(angle), 0.0, std::cos(angle);
    return rotation;
}

void ExpectMatrixNear(const Eigen::MatrixXd& actual,
                      const Eigen::MatrixXd& expected, double tolerance,
                      const std::string& description) {
    if (actual.rows() != expected.rows() || actual.cols() != expected.cols()) {
        ExpectTrue(false, description + ": shape differs");
        return;
    }
    const double scale =
        std::max(1.0, expected.cwiseAbs().maxCoeff());
    const double error = (actual - expected).cwiseAbs().maxCoeff();
    ExpectTrue(error <= tolerance * scale,
               description + ": error " + std::to_string(error));
}

void ExpectVectorNear(const Eigen::VectorXd& actual,
                      const Eigen::VectorXd& expected, double tolerance,
                      const std::string& description) {
    ExpectMatrixNear(actual, expected, tolerance, description);
}

void PutJointCoordinate(const MultibodyModel& model, JointHandle joint,
                        double value, Eigen::VectorXd* positions) {
    const auto range = model.GetJointPositionRange(joint);
    if (range.size() == 1) {
        (*positions)[range.start()] = value;
    }
}

void PutJointVelocity(const MultibodyModel& model, JointHandle joint,
                      double value, Eigen::VectorXd* velocities) {
    const auto range = model.GetJointVelocityRange(joint);
    if (range.size() == 1) {
        (*velocities)[range.start()] = value;
    }
}

class DifferentialKinematicsFixture {
   public:
    DifferentialKinematicsFixture() {
        base = model.AddRigidBody("base", PhysicalInertia(1.2));
        slider = model.AddRigidBody("slider", PhysicalInertia(0.9));
        welded = model.AddRigidBody("welded", PhysicalInertia(0.7));
        reversed = model.AddRigidBody("reversed", PhysicalInertia(1.1));
        free = model.AddRigidBody("free", PhysicalInertia(0.8));

        base_joint = model.AddRevoluteJoint(
            "base_joint", model.world_frame(), model.body_frame(base),
            Eigen::Vector3d::UnitZ(), 0.0);

        FixedFramePoseParameters slider_mount_pose;
        slider_mount_pose.R_PF = RotationAboutY(0.41);
        slider_mount_pose.p_PoFo_P = Eigen::Vector3d(0.31, -0.17, 0.23);
        slider_mount =
            model.AddFixedFrame("slider_mount", base, slider_mount_pose);
        slider_joint = model.AddPrismaticJoint(
            "slider_joint", slider_mount, model.body_frame(slider),
            Eigen::Vector3d::UnitX(), 0.0);

        FixedFramePoseParameters weld_parent_pose;
        weld_parent_pose.R_PF = RotationAboutY(-0.28);
        weld_parent_pose.p_PoFo_P = Eigen::Vector3d(-0.22, 0.19, 0.14);
        weld_parent =
            model.AddFixedFrame("weld_parent", slider, weld_parent_pose);
        model.AddWeldJoint("weld", weld_parent, model.body_frame(welded));

        FixedFramePoseParameters point_pose;
        point_pose.R_PF = RotationAboutY(0.17);
        point_pose.p_PoFo_P = point_in_welded;
        point_frame = model.AddFixedFrame("point", welded, point_pose);

        FixedFramePoseParameters reverse_mount_pose;
        reverse_mount_pose.R_PF = RotationAboutY(0.33);
        reverse_mount_pose.p_PoFo_P = Eigen::Vector3d(0.18, -0.12, 0.16);
        reverse_mount =
            model.AddFixedFrame("reverse_mount", reversed, reverse_mount_pose);
        reverse_joint = model.AddRevoluteJoint(
            "reverse_joint", reverse_mount, model.world_frame(),
            Eigen::Vector3d::UnitY(), 0.0);

        model.DeclareFreeBody(free);
        model.Finalize();

        positions = Eigen::VectorXd::Zero(model.num_generalized_positions());
        PutJointCoordinate(model, base_joint, 0.37, &positions);
        PutJointCoordinate(model, slider_joint, -0.16, &positions);
        PutJointCoordinate(model, reverse_joint, 0.29, &positions);
        const auto free_q = model.GetFreeBodyPositionRange(free);
        positions.segment<7>(free_q.start()) << 1.7, -0.4, 0.9, 0.6, 0.42,
            -0.31, 0.27;

        velocities =
            Eigen::VectorXd::Zero(model.num_generalized_velocities());
        PutJointVelocity(model, base_joint, 0.83, &velocities);
        PutJointVelocity(model, slider_joint, -0.47, &velocities);
        PutJointVelocity(model, reverse_joint, 0.61, &velocities);
        const auto free_v = model.GetFreeBodyVelocityRange(free);
        velocities.segment<6>(free_v.start()) << 0.52, -0.34, 0.27, -0.19,
            0.44, 0.31;
    }

    std::unique_ptr<MultibodyEvaluationContext> MakeContext(
        const Eigen::VectorXd& q, const Eigen::VectorXd& v) const {
        auto context = model.CreateDefaultContext();
        model.SetGeneralizedPositions(context.get(), q);
        model.SetGeneralizedVelocities(context.get(), v);
        return context;
    }

    MultibodyModel model;
    RigidBodyHandle base;
    RigidBodyHandle slider;
    RigidBodyHandle welded;
    RigidBodyHandle reversed;
    RigidBodyHandle free;
    JointHandle base_joint;
    JointHandle slider_joint;
    JointHandle reverse_joint;
    FrameHandle slider_mount;
    FrameHandle weld_parent;
    FrameHandle reverse_mount;
    FrameHandle point_frame;
    const Eigen::Vector3d point_in_welded{0.16, -0.11, 0.21};
    Eigen::VectorXd positions;
    Eigen::VectorXd velocities;
};

Eigen::Vector4d QuaternionDerivative(const Eigen::Vector4d& q,
                                     const Eigen::Vector3d& angular_velocity) {
    Eigen::Matrix<double, 4, 3> quaternion_map;
    // `q` is a plain four-vector in the public wxyz layout. Eigen's geometric
    // x()/y()/z()/w() accessors would instead interpret it as xyzw.
    const double qw = q[0];
    const double qx = q[1];
    const double qy = q[2];
    const double qz = q[3];
    quaternion_map << -qx, -qy, -qz, qw, qz, -qy, -qz, qw, qx, qy, -qx, qw;
    return 0.5 * quaternion_map * angular_velocity;
}

Eigen::Vector3d AngularVelocityFromQuaternionDerivative(
    const Eigen::Vector4d& q, const Eigen::Vector4d& qdot) {
    Eigen::Matrix<double, 4, 3> quaternion_map;
    const double qw = q[0];
    const double qx = q[1];
    const double qy = q[2];
    const double qz = q[3];
    quaternion_map << -qx, -qy, -qz, qw, qz, -qy, -qz, qw, qx, qy, -qx, qw;
    return (2.0 / q.squaredNorm()) * quaternion_map.transpose() * qdot;
}

void CheckMappings() {
    DifferentialKinematicsFixture fixture;
    auto context = fixture.MakeContext(fixture.positions, fixture.velocities);
    const Eigen::VectorXd mapping_probe = Eigen::VectorXd::LinSpaced(
        fixture.model.num_generalized_velocities(), 0.67, -0.41);
    ExpectTrue(mapping_probe != fixture.velocities,
               "the explicit mapping probe differs from context velocity");
    Eigen::VectorXd qdot(fixture.model.num_generalized_positions());
    fixture.model.MapGeneralizedVelocitiesToPositionDerivatives(
        *context, mapping_probe, &qdot);

    const auto free_q = fixture.model.GetFreeBodyPositionRange(fixture.free);
    const auto free_v = fixture.model.GetFreeBodyVelocityRange(fixture.free);
    const Eigen::Vector4d quaternion =
        fixture.positions.segment<4>(free_q.start());
    const Eigen::Vector3d angular_velocity =
        mapping_probe.segment<3>(free_v.start());
    ExpectVectorNear(
        qdot.segment<4>(free_q.start()),
        QuaternionDerivative(quaternion, angular_velocity),
        128.0 * std::numeric_limits<double>::epsilon(),
        "wxyz quaternion derivative follows the written Q(q) map");
    ExpectVectorNear(qdot.segment<3>(free_q.start() + 4),
                     mapping_probe.segment<3>(free_v.start() + 3),
                     0.0, "free-body translational derivative is v");

    Eigen::VectorXd mapped_back(fixture.model.num_generalized_velocities());
    fixture.model.MapGeneralizedPositionDerivativesToVelocities(
        *context, qdot, &mapped_back);
    ExpectVectorNear(mapped_back, mapping_probe,
                     512.0 * std::numeric_limits<double>::epsilon(),
                     "the explicit v to qdot to v mapping is an identity");

    const Eigen::VectorXd independent_qdot = Eigen::VectorXd::LinSpaced(
        fixture.model.num_generalized_positions(), -0.57, 0.49);
    ExpectTrue(
        std::abs(quaternion.dot(
            independent_qdot.segment<4>(free_q.start()))) > 1.0e-2,
        "the independent qdot contains a quaternion radial component");
    Eigen::VectorXd expected_independent_velocity = Eigen::VectorXd::Zero(
        fixture.model.num_generalized_velocities());
    for (const JointHandle joint : {fixture.base_joint, fixture.slider_joint,
                                    fixture.reverse_joint}) {
        const auto q_range = fixture.model.GetJointPositionRange(joint);
        const auto v_range = fixture.model.GetJointVelocityRange(joint);
        expected_independent_velocity[v_range.start()] =
            independent_qdot[q_range.start()];
    }
    expected_independent_velocity.segment<3>(free_v.start()) =
        AngularVelocityFromQuaternionDerivative(
            quaternion, independent_qdot.segment<4>(free_q.start()));
    expected_independent_velocity.segment<3>(free_v.start() + 3) =
        independent_qdot.segment<3>(free_q.start() + 4);
    fixture.model.MapGeneralizedPositionDerivativesToVelocities(
        *context, independent_qdot, &mapped_back);
    ExpectVectorNear(mapped_back, expected_independent_velocity,
                     512.0 * std::numeric_limits<double>::epsilon(),
                     "independent qdot follows the written inverse map");

    Eigen::VectorXd remapped_qdot(qdot.size());
    fixture.model.MapGeneralizedVelocitiesToPositionDerivatives(
        *context, mapped_back, &remapped_qdot);
    Eigen::VectorXd expected_projected_qdot = independent_qdot;
    expected_projected_qdot.segment<4>(free_q.start()) =
        QuaternionDerivative(
            quaternion,
            expected_independent_velocity.segment<3>(free_v.start()));
    ExpectVectorNear(remapped_qdot, expected_projected_qdot,
                     512.0 * std::numeric_limits<double>::epsilon(),
                     "qdot to v to qdot is the tangent projection");

    Eigen::VectorXd scaled_positions = fixture.positions;
    scaled_positions.segment<4>(free_q.start()) *= 2.4;
    auto scaled_context =
        fixture.MakeContext(scaled_positions, fixture.velocities);
    Eigen::VectorXd scaled_qdot(qdot.size());
    fixture.model.MapGeneralizedVelocitiesToPositionDerivatives(
        *scaled_context, mapping_probe, &scaled_qdot);
    ExpectVectorNear(scaled_qdot.segment<4>(free_q.start()),
                     2.4 * qdot.segment<4>(free_q.start()),
                     512.0 * std::numeric_limits<double>::epsilon(),
                     "quaternion scaling scales only its derivative block");
    ExpectVectorNear(scaled_qdot.segment<3>(free_q.start() + 4),
                     qdot.segment<3>(free_q.start() + 4), 0.0,
                     "quaternion scaling leaves translation derivative alone");
    fixture.model.MapGeneralizedPositionDerivativesToVelocities(
        *scaled_context, scaled_qdot, &mapped_back);
    ExpectVectorNear(mapped_back, mapping_probe,
                     1024.0 * std::numeric_limits<double>::epsilon(),
                     "the inverse map accounts for safe quaternion scaling");
    ExpectTrue(scaled_context->generalized_positions() == scaled_positions,
               "mapping a safe non-unit quaternion does not normalize or "
               "rewrite stored positions");

    Eigen::VectorXd sentinel = Eigen::VectorXd::Constant(qdot.size(), 73.0);
    const Eigen::VectorXd before = sentinel;
    const Eigen::VectorXd wrong = Eigen::VectorXd::Zero(
        fixture.model.num_generalized_velocities() + 1);
    ExpectTrue(
        RefusalMentions(
            [&] {
                fixture.model.MapGeneralizedVelocitiesToPositionDerivatives(
                    *context, wrong, &sentinel);
            },
            "generalized velocities"),
        "mapping refuses a wrong input size by physical name");
    ExpectTrue(sentinel == before,
               "a refused mapping leaves the caller's output unchanged");

    Eigen::VectorXd non_finite_qdot = qdot;
    non_finite_qdot[non_finite_qdot.size() - 1] =
        std::numeric_limits<double>::quiet_NaN();
    Eigen::VectorXd inverse_sentinel = Eigen::VectorXd::Constant(
        fixture.model.num_generalized_velocities(), -31.0);
    const Eigen::VectorXd inverse_before = inverse_sentinel;
    ExpectTrue(
        RefusalMentions(
            [&] {
                fixture.model.MapGeneralizedPositionDerivativesToVelocities(
                    *context, non_finite_qdot, &inverse_sentinel);
            },
            "non-finite"),
        "inverse mapping refuses a non-finite position derivative");
    ExpectTrue(inverse_sentinel == inverse_before,
               "a refused inverse mapping leaves its output unchanged");
}

Eigen::Vector3d AngularVelocityFromCentralDifference(
    const Eigen::Matrix3d& rotation_minus,
    const Eigen::Matrix3d& rotation_at_state,
    const Eigen::Matrix3d& rotation_plus, double step) {
    const Eigen::Matrix3d rotation_derivative =
        (rotation_plus - rotation_minus) / (2.0 * step);
    const Eigen::Matrix3d angular_cross =
        rotation_derivative * rotation_at_state.transpose();
    return Eigen::Vector3d(
        0.5 * (angular_cross(2, 1) - angular_cross(1, 2)),
        0.5 * (angular_cross(0, 2) - angular_cross(2, 0)),
        0.5 * (angular_cross(1, 0) - angular_cross(0, 1)));
}

void CheckJacobianColumns() {
    DifferentialKinematicsFixture fixture;
    auto context = fixture.MakeContext(fixture.positions, fixture.velocities);
    const double step = 1.0e-5;

    for (int body_index = 0; body_index < fixture.model.num_rigid_bodies();
         ++body_index) {
        const RigidBodyHandle body = fixture.model.GetRigidBody(body_index);
        const Eigen::Vector3d point =
            body_index == 2 ? fixture.point_in_welded
                            : Eigen::Vector3d(0.07, -0.04, 0.09);
        Eigen::MatrixXd angular(3,
                                fixture.model.num_generalized_velocities());
        Eigen::MatrixXd translational(
            3, fixture.model.num_generalized_velocities());
        fixture.model
            .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                *context, body, point, &angular, &translational);

        const auto pose_at_state = fixture.model.CalcPoseInWorld(*context, body);
        for (int column = 0;
             column < fixture.model.num_generalized_velocities(); ++column) {
            Eigen::VectorXd basis = Eigen::VectorXd::Zero(
                fixture.model.num_generalized_velocities());
            basis[column] = 1.0;
            Eigen::VectorXd qdot(fixture.model.num_generalized_positions());
            fixture.model.MapGeneralizedVelocitiesToPositionDerivatives(
                *context, basis, &qdot);
            auto minus = fixture.MakeContext(fixture.positions - step * qdot,
                                             fixture.velocities);
            auto plus = fixture.MakeContext(fixture.positions + step * qdot,
                                            fixture.velocities);
            const auto pose_minus = fixture.model.CalcPoseInWorld(*minus, body);
            const auto pose_plus = fixture.model.CalcPoseInWorld(*plus, body);

            const Eigen::Vector3d point_minus =
                pose_minus.translation() + pose_minus.rotation() * point;
            const Eigen::Vector3d point_plus =
                pose_plus.translation() + pose_plus.rotation() * point;
            const Eigen::Vector3d translational_difference =
                (point_plus - point_minus) / (2.0 * step);
            const Eigen::Vector3d angular_difference =
                AngularVelocityFromCentralDifference(
                    pose_minus.rotation(), pose_at_state.rotation(),
                    pose_plus.rotation(), step);
            ExpectVectorNear(
                angular.col(column), angular_difference, 2.0e-8,
                "Jacobian angular column " + std::to_string(column) +
                    " for body " + std::to_string(body_index));
            ExpectVectorNear(
                translational.col(column), translational_difference, 2.0e-8,
                "Jacobian point column " + std::to_string(column) +
                    " for body " + std::to_string(body_index));
        }
    }

    Eigen::MatrixXd angular(3, fixture.model.num_generalized_velocities());
    Eigen::MatrixXd translational(3,
                                  fixture.model.num_generalized_velocities());
    fixture.model
        .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
            *context, fixture.welded, fixture.point_in_welded, &angular,
            &translational);
    const auto point_velocity =
        fixture.model.CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
            *context, fixture.point_frame);
    ExpectVectorNear(
        angular * fixture.velocities,
        point_velocity.angular_velocity_radians_per_second(), 2.0e-13,
        "Jacobian times v gives the point frame's angular velocity");
    ExpectVectorNear(
        translational * fixture.velocities,
        point_velocity
            .translational_velocity_at_frame_origin_meters_per_second(),
        2.0e-13,
        "Jacobian times v gives the point frame's translational velocity");
}

void CalculateAccelerations(const DifferentialKinematicsFixture& fixture,
                            const MultibodyEvaluationContext& context,
                            const Eigen::VectorXd& vdot,
                            Eigen::MatrixXd* angular,
                            Eigen::MatrixXd* translational) {
    fixture.model
        .CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
            context, vdot, angular, translational);
}

void CheckSpatialAccelerationAgainstVelocityDerivativeAndBodyOrder() {
    MultibodyModel model;
    // Add the leaf first on purpose. Public body order is [leaf, root], while
    // every valid spanning forest must place the root's mobod before the leaf's.
    const RigidBodyHandle leaf =
        model.AddRigidBody("acceleration_leaf", PhysicalInertia(0.8));
    const RigidBodyHandle root =
        model.AddRigidBody("acceleration_root", PhysicalInertia(1.1));
    const JointHandle root_joint = model.AddRevoluteJoint(
        "acceleration_root_joint", model.world_frame(), model.body_frame(root),
        Eigen::Vector3d::UnitZ(), 0.0);
    FixedFramePoseParameters leaf_mount_pose;
    leaf_mount_pose.R_PF = RotationAboutY(0.39);
    leaf_mount_pose.p_PoFo_P = Eigen::Vector3d(0.32, -0.21, 0.17);
    const FrameHandle leaf_mount =
        model.AddFixedFrame("acceleration_leaf_mount", root, leaf_mount_pose);
    const JointHandle leaf_joint = model.AddRevoluteJoint(
        "acceleration_leaf_joint", leaf_mount, model.body_frame(leaf),
        Eigen::Vector3d::UnitX(), 0.0);
    model.Finalize();

    ExpectTrue(model.GetRigidBody(0) == leaf && model.GetRigidBody(1) == root,
               "the acceleration oracle makes public body order differ from "
               "topological mobod order");
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    PutJointCoordinate(model, root_joint, 0.31, &positions);
    PutJointCoordinate(model, leaf_joint, -0.27, &positions);
    Eigen::VectorXd velocities =
        Eigen::VectorXd::Zero(model.num_generalized_velocities());
    PutJointVelocity(model, root_joint, 0.83, &velocities);
    PutJointVelocity(model, leaf_joint, -0.59, &velocities);
    Eigen::VectorXd velocity_derivatives =
        Eigen::VectorXd::Zero(model.num_generalized_velocities());
    PutJointVelocity(model, root_joint, 0.47, &velocity_derivatives);
    PutJointVelocity(model, leaf_joint, -0.36, &velocity_derivatives);
    Eigen::VectorXd position_derivatives =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    position_derivatives[model.GetJointPositionRange(root_joint).start()] =
        velocities[model.GetJointVelocityRange(root_joint).start()];
    position_derivatives[model.GetJointPositionRange(leaf_joint).start()] =
        velocities[model.GetJointVelocityRange(leaf_joint).start()];

    auto make_context = [&](const Eigen::VectorXd& q,
                            const Eigen::VectorXd& v) {
        auto context = model.CreateDefaultContext();
        model.SetGeneralizedPositions(context.get(), q);
        model.SetGeneralizedVelocities(context.get(), v);
        return context;
    };
    auto context = make_context(positions, velocities);
    Eigen::MatrixXd angular(3, model.num_rigid_bodies());
    Eigen::MatrixXd translational(3, model.num_rigid_bodies());
    model.CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
        *context, velocity_derivatives, &angular, &translational);

    Eigen::MatrixXd bias_angular(3, model.num_rigid_bodies());
    Eigen::MatrixXd bias_translational(3, model.num_rigid_bodies());
    model.CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
        *context, Eigen::VectorXd::Zero(model.num_generalized_velocities()),
        &bias_angular, &bias_translational);
    ExpectTrue(bias_angular.col(0).norm() > 1.0e-4 &&
                   bias_translational.col(0).norm() > 1.0e-4,
               "the nonparallel two-revolute leaf excites angular and "
               "translational bias acceleration");

    const double step = 1.0e-5;
    auto minus = make_context(positions - step * position_derivatives,
                              velocities - step * velocity_derivatives);
    auto plus = make_context(positions + step * position_derivatives,
                             velocities + step * velocity_derivatives);
    for (int body_index = 0; body_index < model.num_rigid_bodies();
         ++body_index) {
        const RigidBodyHandle body = model.GetRigidBody(body_index);
        const auto velocity_minus =
            model.CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *minus, model.body_frame(body));
        const auto velocity_plus =
            model.CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *plus, model.body_frame(body));
        ExpectVectorNear(
            angular.col(body_index),
            (velocity_plus.angular_velocity_radians_per_second() -
             velocity_minus.angular_velocity_radians_per_second()) /
                (2.0 * step),
            2.0e-8,
            "full angular acceleration is the independent velocity derivative "
            "for public body " +
                std::to_string(body_index));
        ExpectVectorNear(
            translational.col(body_index),
            (velocity_plus
                 .translational_velocity_at_frame_origin_meters_per_second() -
             velocity_minus
                 .translational_velocity_at_frame_origin_meters_per_second()) /
                (2.0 * step),
            2.0e-8,
            "full translational acceleration is the independent velocity "
            "derivative for public body " +
                std::to_string(body_index));
    }
}

void CheckSpatialAccelerations() {
    DifferentialKinematicsFixture fixture;
    auto context = fixture.MakeContext(fixture.positions, fixture.velocities);
    const int body_count = fixture.model.num_rigid_bodies();
    const int velocity_count = fixture.model.num_generalized_velocities();
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(velocity_count);
    const Eigen::VectorXd first =
        Eigen::VectorXd::LinSpaced(velocity_count, -0.63, 0.71);
    const Eigen::VectorXd second =
        Eigen::VectorXd::LinSpaced(velocity_count, 0.48, -0.39);
    Eigen::MatrixXd angular_zero(3, body_count);
    Eigen::MatrixXd translational_zero(3, body_count);
    Eigen::MatrixXd angular_first(3, body_count);
    Eigen::MatrixXd translational_first(3, body_count);
    Eigen::MatrixXd angular_second(3, body_count);
    Eigen::MatrixXd translational_second(3, body_count);
    Eigen::MatrixXd angular_sum(3, body_count);
    Eigen::MatrixXd translational_sum(3, body_count);
    CalculateAccelerations(fixture, *context, zero, &angular_zero,
                           &translational_zero);
    CalculateAccelerations(fixture, *context, first, &angular_first,
                           &translational_first);
    CalculateAccelerations(fixture, *context, second, &angular_second,
                           &translational_second);
    CalculateAccelerations(fixture, *context, first + second, &angular_sum,
                           &translational_sum);

    ExpectTrue(angular_zero.cwiseAbs().maxCoeff() > 1.0e-4 ||
                   translational_zero.cwiseAbs().maxCoeff() > 1.0e-4,
               "nonparallel moving chain excites bias acceleration");
    ExpectMatrixNear(angular_sum - angular_zero,
                     (angular_first - angular_zero) +
                         (angular_second - angular_zero),
                     2.0e-13, "angular acceleration is affine in vdot");
    ExpectMatrixNear(translational_sum - translational_zero,
                     (translational_first - translational_zero) +
                         (translational_second - translational_zero),
                     2.0e-13, "translational acceleration is affine in vdot");

    for (int body_index = 0; body_index < body_count; ++body_index) {
        Eigen::MatrixXd angular_jacobian(3, velocity_count);
        Eigen::MatrixXd translational_jacobian(3, velocity_count);
        fixture.model
            .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                *context, fixture.model.GetRigidBody(body_index),
                Eigen::Vector3d::Zero(), &angular_jacobian,
                &translational_jacobian);
        ExpectVectorNear(
            angular_first.col(body_index) - angular_zero.col(body_index),
            angular_jacobian * first, 3.0e-13,
            "body angular acceleration equals bias plus J*vdot");
        ExpectVectorNear(
            translational_first.col(body_index) -
                translational_zero.col(body_index),
            translational_jacobian * first, 3.0e-13,
            "body translational acceleration equals bias plus J*vdot");
    }

    auto rest_context = fixture.MakeContext(
        fixture.positions, Eigen::VectorXd::Zero(velocity_count));
    Eigen::MatrixXd rest_angular(3, body_count);
    Eigen::MatrixXd rest_translational(3, body_count);
    CalculateAccelerations(fixture, *rest_context, zero, &rest_angular,
                           &rest_translational);
    ExpectTrue(rest_angular.isZero(0.0) && rest_translational.isZero(0.0),
               "v zero and vdot zero gives exact zero acceleration");

    Eigen::MatrixXd sentinel_angular =
        Eigen::MatrixXd::Constant(3, body_count, 19.0);
    Eigen::MatrixXd sentinel_translational =
        Eigen::MatrixXd::Constant(3, body_count, -23.0);
    const Eigen::MatrixXd before_angular = sentinel_angular;
    const Eigen::MatrixXd before_translational = sentinel_translational;
    Eigen::VectorXd invalid = first;
    invalid[invalid.size() - 1] = std::numeric_limits<double>::infinity();
    ExpectTrue(
        RefusalMentions(
            [&] {
                CalculateAccelerations(fixture, *context, invalid,
                                       &sentinel_angular,
                                       &sentinel_translational);
            },
            "non-finite"),
        "acceleration query refuses a non-finite vdot");
    ExpectTrue(sentinel_angular == before_angular &&
                   sentinel_translational == before_translational,
               "a refused acceleration leaves both outputs unchanged");

    Eigen::MatrixXd aliased_output =
        Eigen::MatrixXd::Constant(3, body_count, 41.0);
    const Eigen::MatrixXd aliased_before = aliased_output;
    ExpectTrue(
        RefusalMentions(
            [&] {
                CalculateAccelerations(fixture, *context, first,
                                       &aliased_output, &aliased_output);
            },
            "same object"),
        "acceleration refuses aliased angular and translational outputs");
    ExpectTrue(aliased_output == aliased_before,
               "refusing aliased acceleration outputs writes nothing");
}

void CheckFailureBoundaries() {
    DifferentialKinematicsFixture first;
    DifferentialKinematicsFixture second;
    auto first_context =
        first.MakeContext(first.positions, first.velocities);
    auto second_context =
        second.MakeContext(second.positions, second.velocities);

    MultibodyModel unfinalized;
    const RigidBodyHandle body =
        unfinalized.AddRigidBody("body", PhysicalInertia(1.0));
    Eigen::MatrixXd angular(3, 1);
    Eigen::MatrixXd translational(3, 1);
    ExpectTrue(
        RefusalMentions(
            [&] {
                unfinalized
                    .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                        *first_context, body, Eigen::Vector3d::Zero(), &angular,
                        &translational);
            },
            "before Finalize"),
        "Jacobian query rejects an unfinalized model before using context");

    Eigen::VectorXd mapped_qdot(first.model.num_generalized_positions());
    Eigen::VectorXd mapped_velocity(first.model.num_generalized_velocities());
    ExpectTrue(
        RefusalMentions(
            [&] {
                unfinalized.MapGeneralizedVelocitiesToPositionDerivatives(
                    *first_context, first.velocities, &mapped_qdot);
            },
            "before Finalize"),
        "forward mapping rejects an unfinalized model");
    ExpectTrue(
        RefusalMentions(
            [&] {
                unfinalized.MapGeneralizedPositionDerivativesToVelocities(
                    *first_context, first.positions, &mapped_velocity);
            },
            "before Finalize"),
        "inverse mapping rejects an unfinalized model");

    Eigen::MatrixXd acceleration_angular(3, first.model.num_rigid_bodies());
    Eigen::MatrixXd acceleration_translational(
        3, first.model.num_rigid_bodies());
    ExpectTrue(
        RefusalMentions(
            [&] {
                unfinalized
                    .CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
                        *first_context, first.velocities,
                        &acceleration_angular, &acceleration_translational);
            },
            "before Finalize"),
        "acceleration query rejects an unfinalized model");

    Eigen::MatrixXd output_a(3, first.model.num_generalized_velocities());
    Eigen::MatrixXd output_b(3, first.model.num_generalized_velocities());
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model
                    .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                        *second_context, first.base, Eigen::Vector3d::Zero(),
                        &output_a, &output_b);
            },
            "different model"),
        "Jacobian query rejects a foreign context at the facade");
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model.MapGeneralizedVelocitiesToPositionDerivatives(
                    *second_context, first.velocities, &mapped_qdot);
            },
            "different model"),
        "forward mapping rejects a foreign context at the facade");
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model.MapGeneralizedPositionDerivativesToVelocities(
                    *second_context, first.positions, &mapped_velocity);
            },
            "different model"),
        "inverse mapping rejects a foreign context at the facade");
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model
                    .CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
                        *second_context, first.velocities,
                        &acceleration_angular, &acceleration_translational);
            },
            "different model"),
        "acceleration query rejects a foreign context at the facade");
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model
                    .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                        *first_context, second.base, Eigen::Vector3d::Zero(),
                        &output_a, &output_b);
            },
            "different model"),
        "Jacobian query rejects a foreign body handle at the facade");

    Eigen::VectorXd wrong_size_mapping_output =
        Eigen::VectorXd::Constant(
            first.model.num_generalized_positions() + 1, 13.0);
    const Eigen::VectorXd wrong_size_mapping_output_before =
        wrong_size_mapping_output;
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model.MapGeneralizedVelocitiesToPositionDerivatives(
                    *first_context, first.velocities,
                    &wrong_size_mapping_output);
            },
            "generalized position derivatives"),
        "mapping rejects an output with the wrong coordinate count");
    ExpectTrue(wrong_size_mapping_output ==
                   wrong_size_mapping_output_before,
               "refusing a wrong-size mapping output writes nothing");

    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model.MapGeneralizedVelocitiesToPositionDerivatives(
                    *first_context, first.velocities, nullptr);
            },
            "output is null"),
        "mapping rejects a null vector output");

    Eigen::MatrixXd valid_acceleration_output =
        Eigen::MatrixXd::Constant(
            3, first.model.num_rigid_bodies(), 19.0);
    Eigen::MatrixXd wrong_size_acceleration_output =
        Eigen::MatrixXd::Constant(
            3, first.model.num_rigid_bodies() + 1, 23.0);
    const Eigen::MatrixXd valid_acceleration_output_before =
        valid_acceleration_output;
    const Eigen::MatrixXd wrong_size_acceleration_output_before =
        wrong_size_acceleration_output;
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model
                    .CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
                        *first_context, first.velocities,
                        &valid_acceleration_output,
                        &wrong_size_acceleration_output);
            },
            "body-origin translational acceleration output must be"),
        "acceleration rejects a wrong-size second physical output");
    ExpectTrue(valid_acceleration_output ==
                   valid_acceleration_output_before &&
                   wrong_size_acceleration_output ==
                       wrong_size_acceleration_output_before,
               "refusing one wrong-size acceleration output writes neither "
               "physical output");

    Eigen::MatrixXd null_acceleration_sentinel =
        Eigen::MatrixXd::Constant(
            3, first.model.num_rigid_bodies(), -43.0);
    const Eigen::MatrixXd null_acceleration_sentinel_before =
        null_acceleration_sentinel;
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model
                    .CalcRigidBodyFrameSpatialAccelerationsRelativeToWorldExpressedInWorld(
                        *first_context, first.velocities,
                        &null_acceleration_sentinel, nullptr);
            },
            "output is null"),
        "acceleration rejects a null second physical output");
    ExpectTrue(null_acceleration_sentinel ==
                   null_acceleration_sentinel_before,
               "refusing a null acceleration output leaves the other output "
               "unchanged");

    Eigen::MatrixXd nonfinite_point_angular =
        Eigen::MatrixXd::Constant(
            3, first.model.num_generalized_velocities(), 29.0);
    Eigen::MatrixXd nonfinite_point_translational =
        Eigen::MatrixXd::Constant(
            3, first.model.num_generalized_velocities(), 31.0);
    const Eigen::MatrixXd nonfinite_point_angular_before =
        nonfinite_point_angular;
    const Eigen::MatrixXd nonfinite_point_translational_before =
        nonfinite_point_translational;
    Eigen::Vector3d nonfinite_point = Eigen::Vector3d::Zero();
    nonfinite_point.x() = std::numeric_limits<double>::quiet_NaN();
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model
                    .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                        *first_context, first.base, nonfinite_point,
                        &nonfinite_point_angular,
                        &nonfinite_point_translational);
            },
            "non-finite"),
        "Jacobian query rejects a non-finite body-fixed point");
    ExpectTrue(nonfinite_point_angular ==
                       nonfinite_point_angular_before &&
                   nonfinite_point_translational ==
                       nonfinite_point_translational_before,
               "refusing a non-finite Jacobian point writes neither output");

    Eigen::MatrixXd aliased_jacobian(
        3, first.model.num_generalized_velocities());
    const Eigen::MatrixXd aliased_jacobian_before =
        Eigen::MatrixXd::Constant(
            3, first.model.num_generalized_velocities(), 17.0);
    aliased_jacobian = aliased_jacobian_before;
    ExpectTrue(
        RefusalMentions(
            [&] {
                first.model
                    .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                        *first_context, first.base, Eigen::Vector3d::Zero(),
                        &aliased_jacobian, &aliased_jacobian);
            },
            "same object"),
        "Jacobian query refuses aliased physical outputs");
    ExpectTrue(aliased_jacobian == aliased_jacobian_before,
               "refusing aliased Jacobian outputs writes nothing");

    MultibodyModel alias_model;
    const RigidBodyHandle alias_body =
        alias_model.AddRigidBody("alias_body", PhysicalInertia(1.0));
    alias_model.AddRevoluteJoint(
        "alias_joint", alias_model.world_frame(),
        alias_model.body_frame(alias_body), Eigen::Vector3d::UnitZ(), 0.0);
    alias_model.Finalize();
    auto alias_context = alias_model.CreateDefaultContext();
    Eigen::VectorXd same_vector = Eigen::VectorXd::Constant(1, 0.23);
    const Eigen::VectorXd same_vector_before = same_vector;
    ExpectTrue(
        RefusalMentions(
            [&] {
                alias_model.MapGeneralizedVelocitiesToPositionDerivatives(
                    *alias_context, same_vector, &same_vector);
            },
            "same object"),
        "mapping refuses exact in-place aliasing when nq equals nv");
    ExpectTrue(same_vector == same_vector_before,
               "refusing an in-place mapping leaves the vector unchanged");
    ExpectTrue(
        RefusalMentions(
            [&] {
                alias_model.MapGeneralizedPositionDerivativesToVelocities(
                    *alias_context, same_vector, &same_vector);
            },
            "same object"),
        "inverse mapping refuses exact in-place aliasing when nq equals nv");
    ExpectTrue(
        same_vector == same_vector_before,
        "refusing an in-place inverse mapping leaves the vector unchanged");
}

}  // namespace

int main() {
    CheckMappings();
    CheckJacobianColumns();
    CheckSpatialAccelerationAgainstVelocityDerivativeAndBodyOrder();
    CheckSpatialAccelerations();
    CheckFailureBoundaries();
    if (failure_count == 0) {
        std::printf("PASS multibody differential kinematics\n");
    }
    return failure_count == 0 ? 0 : 1;
}
