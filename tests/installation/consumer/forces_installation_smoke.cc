#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <utility>

#include <Eigen/Geometry>

#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/multibody_runtime/multibody_physical_parameters.h"

namespace {

using orvd::forces::ForceElementEnd;
using orvd::forces::HalfAngleMidpointRollPitchYawBushing;
using orvd::forces::VehicleForceElementCollection;
using orvd::forces::VehicleForcePlan;
using orvd::multibody_model::AppliedBodyWrench;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

bool Near(const Eigen::Vector3d& measured, const Eigen::Vector3d& expected,
          double tolerance) {
    return (measured - expected).cwiseAbs().maxCoeff() <= tolerance;
}

Eigen::Vector3d WrenchPointInWorld(
    const MultibodyModel& model,
    const orvd::multibody_model::MultibodyEvaluationContext& context,
    const AppliedBodyWrench& wrench) {
    const auto body_pose = model.CalcPoseInWorld(context, wrench.body);
    return body_pose.translation() +
           body_pose.rotation() * wrench.point_position_in_body_frame_meters;
}

}  // namespace

int main() {
    try {
        RigidBodyInertiaParameters inertia;
        inertia.mass_kilograms = 9.0;
        inertia.unit_inertia_moments = Eigen::Vector3d(0.7, 0.8, 0.9);

        MultibodyModel model;
        const auto body_a = model.AddRigidBody("body_a", inertia);
        const auto body_c = model.AddRigidBody("body_c", inertia);

        FixedFramePoseParameters pose_a;
        pose_a.R_PF =
            (Eigen::AngleAxisd(0.13, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        pose_a.p_PoFo_P = Eigen::Vector3d(0.31, -0.17, 0.23);
        const auto frame_a = model.AddFixedFrame("frame_a", body_a, pose_a);

        FixedFramePoseParameters pose_c;
        pose_c.R_PF =
            (Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(0.06, Eigen::Vector3d::UnitY()))
                .toRotationMatrix();
        pose_c.p_PoFo_P = Eigen::Vector3d(-0.24, 0.29, -0.11);
        const auto frame_c = model.AddFixedFrame("frame_c", body_c, pose_c);

        model.DeclareFreeBody(body_a);
        model.DeclareFreeBody(body_c);
        model.SetGravityVector(Eigen::Vector3d::Zero());
        model.Finalize();

        HalfAngleMidpointRollPitchYawBushing bushing;
        bushing.name = "installed_half_angle_midpoint_bushing";
        bushing.frame_a_end = ForceElementEnd{frame_a};
        bushing.frame_c_end = ForceElementEnd{frame_c};
        bushing.rotational_stiffness_newton_meters_per_radian =
            Eigen::Vector3d(310.0, 270.0, 190.0);
        bushing.rotational_damping_newton_meter_seconds_per_radian =
            Eigen::Vector3d(17.0, 19.0, 23.0);
        bushing.translational_stiffness_newtons_per_meter =
            Eigen::Vector3d(31000.0, 27000.0, 23000.0);
        bushing.translational_damping_newton_seconds_per_meter =
            Eigen::Vector3d(170.0, 190.0, 230.0);

        VehicleForceElementCollection elements;
        elements.half_angle_midpoint_roll_pitch_yaw_bushings.push_back(
            bushing);
        const VehicleForcePlan plan(model, std::move(elements));
        auto context = model.CreateDefaultContext();

        std::array<AppliedBodyWrench, 2> wrenches;
        Eigen::VectorXd no_series_state(0);
        Eigen::VectorXd no_nominal_force(0);
        Eigen::VectorXd no_series_derivative(0);
        plan.CalcAppliedForces(*context, no_series_state, no_nominal_force,
                               wrenches, no_series_derivative);

        const auto frame_a_pose = model.CalcPoseInWorld(*context, frame_a);
        const auto frame_c_pose = model.CalcPoseInWorld(*context, frame_c);
        const Eigen::Vector3d expected_midpoint =
            0.5 * (frame_a_pose.translation() + frame_c_pose.translation());
        const Eigen::Vector3d point_a =
            WrenchPointInWorld(model, *context, wrenches[0]);
        const Eigen::Vector3d point_c =
            WrenchPointInWorld(model, *context, wrenches[1]);

        constexpr double kTolerance = 1.0e-12;
        const bool finite =
            wrenches[0].point_position_in_body_frame_meters.allFinite() &&
            wrenches[1].point_position_in_body_frame_meters.allFinite() &&
            wrenches[0].force_newtons.allFinite() &&
            wrenches[1].force_newtons.allFinite() &&
            wrenches[0].torque_about_point_newton_metres.allFinite() &&
            wrenches[1].torque_about_point_newton_metres.allFinite();
        const bool paired =
            Near(wrenches[0].force_newtons + wrenches[1].force_newtons,
                 Eigen::Vector3d::Zero(), kTolerance) &&
            Near(wrenches[0].torque_about_point_newton_metres +
                     wrenches[1].torque_about_point_newton_metres,
                 Eigen::Vector3d::Zero(), kTolerance);
        const bool common_midpoint =
            Near(point_a, expected_midpoint, kTolerance) &&
            Near(point_c, expected_midpoint, kTolerance);
        const bool real_evaluation =
            wrenches[0].force_newtons.norm() > 1.0 &&
            wrenches[0].torque_about_point_newton_metres.norm() > 1.0e-3;

        if (plan.half_angle_midpoint_roll_pitch_yaw_bushing_count() != 1 ||
            plan.body_wrench_count() != 2 || wrenches[0].body != body_a ||
            wrenches[1].body != body_c ||
            wrenches[0].expressed_in_frame != model.world_frame() ||
            wrenches[1].expressed_in_frame != model.world_frame() || !finite ||
            !paired || !common_midpoint || !real_evaluation) {
            std::fprintf(
                stderr,
                "installed ORVD forces smoke did not produce one finite "
                "equal-and-opposite wrench pair at the common midpoint\n");
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed ORVD forces smoke failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
