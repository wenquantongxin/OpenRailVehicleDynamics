#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/multibody_model/multibody_frame_spatial_velocity.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/multibody_runtime/multibody_physical_parameters.h"

namespace {

using orvd::forces::IndependentWheelActiveTorqueCoupleDefinition;
using orvd::forces::IndependentWheelActiveTorquePlan;
using orvd::multibody_model::AppliedBodyWrench;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;

int failures = 0;

void Require(bool condition, std::string_view description) {
    if (!condition) {
        std::fprintf(stderr, "active torque plan: %.*s\n",
                     static_cast<int>(description.size()), description.data());
        ++failures;
    }
}

void RequireNear(const Eigen::Vector3d& actual,
                 const Eigen::Vector3d& expected, double tolerance,
                 std::string_view description) {
    Require(actual.allFinite() && expected.allFinite() &&
                (actual - expected).lpNorm<Eigen::Infinity>() <= tolerance,
            description);
}

void RequireRefusal(const std::function<void()>& action,
                    std::string_view description) {
    try {
        action();
        Require(false, description);
    } catch (const std::invalid_argument&) {
    } catch (...) {
        Require(false, description);
    }
}

bool SameWrench(const AppliedBodyWrench& left,
                const AppliedBodyWrench& right) {
    return left.body == right.body &&
           left.point_position_in_body_frame_meters ==
               right.point_position_in_body_frame_meters &&
           left.expressed_in_frame == right.expressed_in_frame &&
           left.torque_about_point_newton_metres ==
               right.torque_about_point_newton_metres &&
           left.force_newtons == right.force_newtons;
}

struct Fixture {
    MultibodyModel model;
    RigidBodyHandle provider;
    RigidBodyHandle wheel;
    RigidBodyHandle frame;
    std::unique_ptr<orvd::multibody_model::MultibodyEvaluationContext> context;

    Fixture() {
        orvd::multibody_runtime::RigidBodyInertiaParameters inertia;
        inertia.mass_kilograms = 320.0;
        inertia.unit_inertia_moments = Eigen::Vector3d(0.45, 0.37, 0.29);
        provider = model.AddRigidBody("axle_bridge", inertia);
        wheel = model.AddRigidBody("independent_wheel", inertia);
        frame = model.AddRigidBody("bogie_frame", inertia);
        model.DeclareFreeBody(provider);
        model.DeclareFreeBody(wheel);
        model.DeclareFreeBody(frame);
        model.Finalize();
        context = model.CreateDefaultContext();
        SetState();
    }

    void SetState() {
        const std::array<Eigen::Matrix3d, 3> rotations{
            (Eigen::AngleAxisd(0.31, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(-0.17, Eigen::Vector3d::UnitZ()))
                .toRotationMatrix(),
            (Eigen::AngleAxisd(-0.12, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitY()))
                .toRotationMatrix(),
            (Eigen::AngleAxisd(0.23, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitX()))
                .toRotationMatrix()};
        const std::array<RigidBodyHandle, 3> bodies{provider, wheel, frame};
        Eigen::VectorXd positions =
            Eigen::VectorXd::Zero(model.num_generalized_positions());
        for (std::size_t ordinal = 0; ordinal < bodies.size(); ++ordinal) {
            const auto range = model.GetFreeBodyPositionRange(bodies[ordinal]);
            const Eigen::Quaterniond q(rotations[ordinal]);
            positions.segment<4>(range.start()) << q.w(), q.x(), q.y(), q.z();
            positions.segment<3>(range.start() + 4) =
                Eigen::Vector3d(0.8 * ordinal, -0.2 * ordinal,
                                0.5 + 0.1 * ordinal);
        }

        Eigen::VectorXd velocities =
            Eigen::VectorXd::Zero(model.num_generalized_velocities());
        const std::array<Eigen::Vector3d, 3> angular_velocities{
            Eigen::Vector3d(0.4, -0.2, 0.1),
            Eigen::Vector3d(-0.3, 6.2, 0.7),
            Eigen::Vector3d(0.8, -0.5, -0.4)};
        for (std::size_t ordinal = 0; ordinal < bodies.size(); ++ordinal) {
            const auto range = model.GetFreeBodyVelocityRange(bodies[ordinal]);
            velocities.segment<3>(range.start()) = angular_velocities[ordinal];
            velocities.segment<3>(range.start() + 3) =
                Eigen::Vector3d(0.2 * ordinal, -0.1, 0.03);
        }
        model.SetGeneralizedState(context.get(), positions, velocities);
    }
};

}  // namespace

int main() {
    try {
        Fixture fixture;
        IndependentWheelActiveTorquePlan plan(
            fixture.model,
            {IndependentWheelActiveTorqueCoupleDefinition{
                .channel_name = "wheel_front_left",
                .axis_provider_body_name = "axle_bridge",
                .wheel_body_name = "independent_wheel",
                .reaction_frame_body_name = "bogie_frame"}});
        Require(plan.channel_count() == 1 && plan.body_wrench_count() == 2 &&
                    plan.channel_name(0) == "wheel_front_left" &&
                    plan.axis_provider_body_name(0) == "axle_bridge" &&
                    plan.wheel_body_name(0) == "independent_wheel" &&
                    plan.reaction_frame_body_name(0) == "bogie_frame",
                "the named three-body channel was not frozen");

        constexpr double kTorque = 1375.25;
        std::array<AppliedBodyWrench, 2> wrenches;
        plan.CalcAppliedForces(*fixture.context, std::array{kTorque},
                               wrenches);
        const Eigen::Vector3d axis_in_world =
            fixture.model.CalcPoseInWorld(*fixture.context, fixture.provider)
                .rotation() *
            Eigen::Vector3d::UnitY();
        const Eigen::Vector3d expected_moment = kTorque * axis_in_world;
        Require(wrenches[0].body == fixture.wheel &&
                    wrenches[1].body == fixture.frame &&
                    wrenches[0].expressed_in_frame ==
                        fixture.model.world_frame() &&
                    wrenches[1].expressed_in_frame ==
                        fixture.model.world_frame() &&
                    wrenches[0].point_position_in_body_frame_meters.isZero() &&
                    wrenches[1].point_position_in_body_frame_meters.isZero(),
                "the pure couple did not act on the wheel and bogie-frame "
                "origins");
        RequireNear(wrenches[0].torque_about_point_newton_metres,
                    expected_moment, 2.0e-12,
                    "the wheel did not receive positive torque about the "
                    "axle-bridge +Y axis");
        RequireNear(wrenches[1].torque_about_point_newton_metres,
                    -expected_moment, 2.0e-12,
                    "the bogie frame did not receive the reaction torque");
        Require(wrenches[0].force_newtons.isZero() &&
                    wrenches[1].force_newtons.isZero() &&
                    (wrenches[0].torque_about_point_newton_metres +
                     wrenches[1].torque_about_point_newton_metres)
                        .isZero(2.0e-12),
                "the active source is not a zero-resultant pure couple");

        const auto wheel_velocity =
            fixture.model
                .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                    *fixture.context, fixture.wheel);
        const auto frame_velocity =
            fixture.model
                .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                    *fixture.context, fixture.frame);
        const double wrench_power =
            wrenches[0].torque_about_point_newton_metres.dot(
                wheel_velocity.angular_velocity_radians_per_second()) +
            wrenches[1].torque_about_point_newton_metres.dot(
                frame_velocity.angular_velocity_radians_per_second());
        const double expected_power =
            kTorque * axis_in_world.dot(
                          wheel_velocity.angular_velocity_radians_per_second() -
                          frame_velocity.angular_velocity_radians_per_second());
        Require(std::abs(wrench_power - expected_power) <= 2.0e-12,
                "the couple virtual power is not torque times wheel-frame "
                "relative angular speed about the provider axis");

        const std::array<AppliedBodyWrench, 2> sentinels = wrenches;
        RequireRefusal(
            [&] {
                plan.CalcAppliedForces(*fixture.context, std::span<const double>{},
                                       wrenches);
            },
            "a wrong held-torque count was accepted");
        Require(SameWrench(wrenches[0], sentinels[0]) &&
                    SameWrench(wrenches[1], sentinels[1]),
                "a rejected torque count changed caller output");
        const std::array nonfinite{
            std::numeric_limits<double>::quiet_NaN()};
        RequireRefusal(
            [&] {
                plan.CalcAppliedForces(*fixture.context, nonfinite, wrenches);
            },
            "a non-finite held torque was accepted");
        Require(SameWrench(wrenches[0], sentinels[0]) &&
                    SameWrench(wrenches[1], sentinels[1]),
                "a rejected non-finite torque changed caller output");

        plan.CalcAppliedForces(*fixture.context, std::array{0.0}, wrenches);
        Require(wrenches[0].torque_about_point_newton_metres.isZero() &&
                    wrenches[1].torque_about_point_newton_metres.isZero() &&
                    wrenches[0].force_newtons.isZero() &&
                    wrenches[1].force_newtons.isZero(),
                "a zero held torque did not emit two valid zero wrenches");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "active torque plan threw: %s\n", error.what());
        return 1;
    }
    if (failures != 0) {
        return 1;
    }
    std::puts("independent-wheel active torque plan verified");
    return 0;
}
