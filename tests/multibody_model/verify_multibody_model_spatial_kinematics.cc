// Spatial velocity through the public modelling boundary, checked against the
// relations that were stated.
//
// The analytic checks below use independent elementary rigid-body formulas.
// The later linearity checks intentionally combine results from the same public
// queries; they test linearity in v, not frame, point or expression semantics.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_frame_spatial_velocity.h"
#include "orvd/multibody_model/multibody_model.h"

namespace {

using orvd::multibody_model::FrameHandle;
using orvd::multibody_model::FrameSpatialVelocity;
using orvd::multibody_model::JointHandle;
using orvd::multibody_model::MultibodyEvaluationContext;
using orvd::multibody_model::MultibodyModel;
using orvd::multibody_model::RigidBodyHandle;
using orvd::multibody_runtime::FixedFramePoseParameters;
using orvd::multibody_runtime::RigidBodyInertiaParameters;

static_assert(!std::is_default_constructible_v<FrameSpatialVelocity>);
static_assert(!std::is_constructible_v<FrameSpatialVelocity, Eigen::Vector3d,
                                       Eigen::Vector3d>);

// Name every ref-qualified accessor overload. A return-type check on a call
// cannot show that all three overloads exist: an rvalue can silently fall back
// to the const-rvalue overload and still return the same value type.
[[maybe_unused]] constexpr auto kAngularOnLvalue =
    static_cast<const Eigen::Vector3d& (FrameSpatialVelocity::*)() const&>(
        &FrameSpatialVelocity::angular_velocity_radians_per_second);
[[maybe_unused]] constexpr auto kAngularOnRvalue =
    static_cast<Eigen::Vector3d (FrameSpatialVelocity::*)() &&>(
        &FrameSpatialVelocity::angular_velocity_radians_per_second);
[[maybe_unused]] constexpr auto kAngularOnConstRvalue =
    static_cast<Eigen::Vector3d (FrameSpatialVelocity::*)() const&&>(
        &FrameSpatialVelocity::angular_velocity_radians_per_second);
[[maybe_unused]] constexpr auto kTranslationalOnLvalue =
    static_cast<const Eigen::Vector3d& (FrameSpatialVelocity::*)() const&>(
        &FrameSpatialVelocity::
            translational_velocity_at_frame_origin_meters_per_second);
[[maybe_unused]] constexpr auto kTranslationalOnRvalue =
    static_cast<Eigen::Vector3d (FrameSpatialVelocity::*)() &&>(
        &FrameSpatialVelocity::
            translational_velocity_at_frame_origin_meters_per_second);
[[maybe_unused]] constexpr auto kTranslationalOnConstRvalue =
    static_cast<Eigen::Vector3d (FrameSpatialVelocity::*)() const&&>(
        &FrameSpatialVelocity::
            translational_velocity_at_frame_origin_meters_per_second);

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

RigidBodyInertiaParameters SolidishBody(double mass) {
    RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = mass;
    inertia.center_of_mass_in_body_frame =
        Eigen::Vector3d(0.05, -0.03, 0.02);
    inertia.unit_inertia_moments = Eigen::Vector3d(0.20, 0.25, 0.30);
    return inertia;
}

Eigen::Matrix3d RotationAboutX(double angle) {
    Eigen::Matrix3d rotation;
    rotation << 1.0, 0.0, 0.0, 0.0, std::cos(angle), -std::sin(angle), 0.0,
        std::sin(angle), std::cos(angle);
    return rotation;
}

Eigen::Matrix3d RotationAboutY(double angle) {
    Eigen::Matrix3d rotation;
    rotation << std::cos(angle), 0.0, std::sin(angle), 0.0, 1.0, 0.0,
        -std::sin(angle), 0.0, std::cos(angle);
    return rotation;
}

Eigen::Matrix3d RotationAboutZ(double angle) {
    Eigen::Matrix3d rotation;
    rotation << std::cos(angle), -std::sin(angle), 0.0, std::sin(angle),
        std::cos(angle), 0.0, 0.0, 0.0, 1.0;
    return rotation;
}

struct VelocityComponents {
    Eigen::Vector3d angular;
    Eigen::Vector3d translational;
};

VelocityComponents Components(const FrameSpatialVelocity& velocity) {
    return {velocity.angular_velocity_radians_per_second(),
            velocity
                .translational_velocity_at_frame_origin_meters_per_second()};
}

constexpr double kArithmeticTolerance =
    256.0 * std::numeric_limits<double>::epsilon();

void ExpectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      const std::string& description) {
    const double scale =
        std::max(1.0, expected.cwiseAbs().maxCoeff());
    const double error = (actual - expected).cwiseAbs().maxCoeff();
    ExpectTrue(error <= kArithmeticTolerance * scale,
               description + ": error " + std::to_string(error));
}

void ExpectVelocityNear(const FrameSpatialVelocity& actual,
                        const VelocityComponents& expected,
                        const std::string& description) {
    ExpectVectorNear(actual.angular_velocity_radians_per_second(),
                     expected.angular, description + " angular velocity");
    ExpectVectorNear(
        actual.translational_velocity_at_frame_origin_meters_per_second(),
        expected.translational,
        description + " frame-origin translational velocity");
}

void PlaceJointPosition(const MultibodyModel& model, JointHandle joint,
                        double value, Eigen::VectorXd* positions) {
    (*positions)[model.GetJointPositionRange(joint).start()] = value;
}

void PlaceJointVelocity(const MultibodyModel& model, JointHandle joint,
                        double value, Eigen::VectorXd* velocities) {
    (*velocities)[model.GetJointVelocityRange(joint).start()] = value;
}

// A non-coplanar chain with a separate branch. It supplies enough geometry to
// distinguish a frame-origin velocity from a body-origin velocity, a relative
// velocity from a simple subtraction at different points, and world components
// from components expressed in a moving frame.
class BranchedSpatialKinematicsFixture {
   public:
    BranchedSpatialKinematicsFixture() {
        base = model.AddRigidBody("base", SolidishBody(1.0));
        serial = model.AddRigidBody("serial", SolidishBody(1.0));
        branch = model.AddRigidBody("branch", SolidishBody(1.0));

        root_joint = model.AddRevoluteJoint(
            "root", model.world_frame(), model.body_frame(base),
            Eigen::Vector3d::UnitZ());

        FixedFramePoseParameters serial_mount_pose;
        serial_mount_pose.R_PF = RotationAboutY(kSerialMountTilt);
        serial_mount_pose.p_PoFo_P = kSerialMountOffset;
        serial_mount =
            model.AddFixedFrame("serial_mount", base, serial_mount_pose);
        serial_joint = model.AddRevoluteJoint(
            "serial_joint", serial_mount, model.body_frame(serial),
            Eigen::Vector3d::UnitX());

        FixedFramePoseParameters branch_mount_pose;
        branch_mount_pose.R_PF = RotationAboutX(kBranchMountTilt);
        branch_mount_pose.p_PoFo_P = kBranchMountOffset;
        branch_mount =
            model.AddFixedFrame("branch_mount", base, branch_mount_pose);
        branch_joint = model.AddRevoluteJoint(
            "branch_joint", branch_mount, model.body_frame(branch),
            Eigen::Vector3d::UnitY());

        FixedFramePoseParameters moving_point_pose;
        moving_point_pose.R_PF = RotationAboutZ(-0.22);
        moving_point_pose.p_PoFo_P = kMovingPointOffsetInSerial;
        moving_point =
            model.AddFixedFrame("moving_point", serial, moving_point_pose);
        model.Finalize();
    }

    std::unique_ptr<MultibodyEvaluationContext> MakeContext(
        double root_angle, double serial_angle, double branch_angle,
        const Eigen::Vector3d& joint_rates) const {
        auto context = model.CreateDefaultContext();
        Eigen::VectorXd positions =
            Eigen::VectorXd::Zero(model.num_generalized_positions());
        PlaceJointPosition(model, root_joint, root_angle, &positions);
        PlaceJointPosition(model, serial_joint, serial_angle, &positions);
        PlaceJointPosition(model, branch_joint, branch_angle, &positions);
        model.SetGeneralizedPositions(context.get(), positions);
        SetRates(context.get(), joint_rates);
        return context;
    }

    void SetRates(MultibodyEvaluationContext* context,
                  const Eigen::Vector3d& joint_rates) const {
        Eigen::VectorXd velocities =
            Eigen::VectorXd::Zero(model.num_generalized_velocities());
        PlaceJointVelocity(model, root_joint, joint_rates.x(), &velocities);
        PlaceJointVelocity(model, serial_joint, joint_rates.y(), &velocities);
        PlaceJointVelocity(model, branch_joint, joint_rates.z(), &velocities);
        model.SetGeneralizedVelocities(context, velocities);
    }

    struct Expected {
        Eigen::Matrix3d R_WB;
        Eigen::Matrix3d R_WC;
        Eigen::Matrix3d R_WD;
        Eigen::Vector3d p_WCo;
        Eigen::Vector3d p_WDo;
        Eigen::Vector3d p_WFo;
        VelocityComponents base_velocity;
        VelocityComponents serial_velocity;
        VelocityComponents branch_velocity;
        VelocityComponents moving_point_velocity;
        VelocityComponents relative_velocity_expressed_in_serial_mount;
    };

    Expected CalculateExpected(double root_angle, double serial_angle,
                               double branch_angle,
                               const Eigen::Vector3d& joint_rates) const {
        Expected result;
        result.R_WB = RotationAboutZ(root_angle);
        const Eigen::Matrix3d R_W_serial_mount =
            result.R_WB * RotationAboutY(kSerialMountTilt);
        const Eigen::Matrix3d R_W_branch_mount =
            result.R_WB * RotationAboutX(kBranchMountTilt);
        result.R_WC = R_W_serial_mount * RotationAboutX(serial_angle);
        result.R_WD = R_W_branch_mount * RotationAboutY(branch_angle);

        result.p_WCo = result.R_WB * kSerialMountOffset;
        result.p_WDo = result.R_WB * kBranchMountOffset;
        const Eigen::Vector3d p_CoFo_W =
            result.R_WC * kMovingPointOffsetInSerial;
        result.p_WFo = result.p_WCo + p_CoFo_W;

        const Eigen::Vector3d root_axis_W = Eigen::Vector3d::UnitZ();
        const Eigen::Vector3d serial_axis_W =
            R_W_serial_mount * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d branch_axis_W =
            R_W_branch_mount * Eigen::Vector3d::UnitY();
        const Eigen::Vector3d w_WB =
            root_axis_W * joint_rates.x();
        const Eigen::Vector3d w_WC =
            w_WB + serial_axis_W * joint_rates.y();
        const Eigen::Vector3d w_WD =
            w_WB + branch_axis_W * joint_rates.z();
        const Eigen::Vector3d v_WCo = w_WB.cross(result.p_WCo);
        const Eigen::Vector3d v_WDo = w_WB.cross(result.p_WDo);
        const Eigen::Vector3d v_WFo =
            v_WCo + w_WC.cross(p_CoFo_W);

        result.base_velocity = {w_WB, Eigen::Vector3d::Zero()};
        result.serial_velocity = {w_WC, v_WCo};
        result.branch_velocity = {w_WD, v_WDo};
        result.moving_point_velocity = {w_WC, v_WFo};

        // F relative to D, expressed in the rotated fixed frame E at the serial
        // mount, with the translational component at Fo. The reference velocity
        // must be shifted from Do to Fo before it is subtracted; subtracting
        // v_WDo directly would be a different quantity.
        const Eigen::Vector3d p_DoFo_W = result.p_WFo - result.p_WDo;
        const Eigen::Vector3d w_DF_W = w_WC - w_WD;
        const Eigen::Vector3d v_DFo_W =
            v_WFo - (v_WDo + w_WD.cross(p_DoFo_W));
        const Eigen::Matrix3d R_EW = R_W_serial_mount.transpose();
        result.relative_velocity_expressed_in_serial_mount =
            {R_EW * w_DF_W, R_EW * v_DFo_W};
        return result;
    }

    MultibodyModel model;
    RigidBodyHandle base;
    RigidBodyHandle serial;
    RigidBodyHandle branch;
    FrameHandle serial_mount;
    FrameHandle branch_mount;
    FrameHandle moving_point;
    JointHandle root_joint;
    JointHandle serial_joint;
    JointHandle branch_joint;

    static constexpr double kSerialMountTilt = 0.43;
    static constexpr double kBranchMountTilt = -0.31;
    inline static const Eigen::Vector3d kSerialMountOffset{0.42, -0.17, 0.28};
    inline static const Eigen::Vector3d kBranchMountOffset{-0.23, 0.34, 0.19};
    inline static const Eigen::Vector3d kMovingPointOffsetInSerial{0.16, 0.27,
                                                                  -0.12};
};

void CheckNonCoplanarChainBranchPointAndExpressionSemantics() {
    constexpr double kRootAngle = 0.37;
    constexpr double kSerialAngle = -0.61;
    constexpr double kBranchAngle = 0.29;
    const Eigen::Vector3d rates(1.3, -0.8, 0.55);

    BranchedSpatialKinematicsFixture fixture;
    const auto context = fixture.MakeContext(
        kRootAngle, kSerialAngle, kBranchAngle, rates);
    const auto expected = fixture.CalculateExpected(
        kRootAngle, kSerialAngle, kBranchAngle, rates);

    ExpectVelocityNear(
        fixture.model
            .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, fixture.base),
        expected.base_velocity, "the root body");
    ExpectVelocityNear(
        fixture.model
            .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, fixture.serial),
        expected.serial_velocity,
        "the serial body accumulates the two non-coplanar joint rates");
    ExpectVelocityNear(
        fixture.model
            .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, fixture.branch),
        expected.branch_velocity,
        "the branch contains its own rate and not the serial branch's");
    ExpectVelocityNear(
        fixture.model.CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
            *context, fixture.moving_point),
        expected.moving_point_velocity,
        "a fixed frame shifts the translational velocity to its own origin");
    ExpectVelocityNear(
        fixture.model
            .CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
                *context, fixture.moving_point,
                fixture.model.body_frame(fixture.branch),
                fixture.serial_mount),
        expected.relative_velocity_expressed_in_serial_mount,
        "a relative velocity shifts the reference to the moving origin and "
        "expresses both components in the requested frame");

    ExpectVectorNear(
        fixture.model
            .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, fixture.serial)
            .angular_velocity_radians_per_second(),
        expected.serial_velocity.angular,
        "the rvalue angular accessor returns the angular member");
    ExpectVectorNear(
        fixture.model
            .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, fixture.serial)
            .translational_velocity_at_frame_origin_meters_per_second(),
        expected.serial_velocity.translational,
        "the rvalue translational accessor returns the translational member");
    const FrameSpatialVelocity const_angular_velocity =
        fixture.model
            .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, fixture.serial);
    const FrameSpatialVelocity const_translational_velocity =
        fixture.model
            .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, fixture.serial);
    ExpectVectorNear(
        std::move(const_angular_velocity)
            .angular_velocity_radians_per_second(),
        expected.serial_velocity.angular,
        "the const-rvalue angular accessor returns the angular member");
    ExpectVectorNear(
        std::move(const_translational_velocity)
            .translational_velocity_at_frame_origin_meters_per_second(),
        expected.serial_velocity.translational,
        "the const-rvalue translational accessor returns the translational "
        "member");

    const auto world_velocity =
        fixture.model.CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
            *context, fixture.model.world_frame());
    ExpectTrue(
        world_velocity.angular_velocity_radians_per_second().isZero(0.0) &&
            world_velocity
                .translational_velocity_at_frame_origin_meters_per_second()
                .isZero(0.0),
        "the world frame has exactly zero spatial velocity");
}

VelocityComponents LinearCombination(const VelocityComponents& first,
                                     const VelocityComponents& second,
                                     double first_scale,
                                     double second_scale) {
    return {first_scale * first.angular + second_scale * second.angular,
            first_scale * first.translational +
                second_scale * second.translational};
}

struct ThreeQueryVelocities {
    VelocityComponents body;
    VelocityComponents frame;
    VelocityComponents relative;
};

ThreeQueryVelocities ObserveThreeQueries(
    const BranchedSpatialKinematicsFixture& fixture,
    const MultibodyEvaluationContext& context) {
    return {
        Components(
            fixture.model
                .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                    context, fixture.serial)),
        Components(
            fixture.model
                .CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                    context, fixture.moving_point)),
        Components(
            fixture.model
                .CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
                    context, fixture.moving_point,
                    fixture.model.body_frame(fixture.branch),
                    fixture.serial_mount))};
}

void ExpectComponentsNear(const VelocityComponents& actual,
                          const VelocityComponents& expected,
                          const std::string& description) {
    ExpectVectorNear(actual.angular, expected.angular,
                     description + " angular component");
    ExpectVectorNear(actual.translational, expected.translational,
                     description + " translational component");
}

void CheckZeroAndLinearityInGeneralizedVelocity() {
    constexpr double kRootAngle = -0.28;
    constexpr double kSerialAngle = 0.52;
    constexpr double kBranchAngle = -0.47;
    constexpr double kFirstScale = -1.7;
    constexpr double kSecondScale = 0.65;
    const Eigen::Vector3d first_rates(0.7, -1.1, 0.3);
    const Eigen::Vector3d second_rates(-0.2, 0.45, 1.2);

    BranchedSpatialKinematicsFixture fixture;
    auto context = fixture.MakeContext(kRootAngle, kSerialAngle, kBranchAngle,
                                       Eigen::Vector3d::Zero());
    const ThreeQueryVelocities zero = ObserveThreeQueries(fixture, *context);
    for (const auto& entry :
         {std::pair{&zero.body, "body"}, std::pair{&zero.frame, "frame"},
          std::pair{&zero.relative, "relative"}}) {
        ExpectTrue(entry.first->angular.isZero(0.0) &&
                       entry.first->translational.isZero(0.0),
                   std::string(entry.second) +
                       " spatial velocity is exactly zero when v is zero");
    }

    fixture.SetRates(context.get(), first_rates);
    const ThreeQueryVelocities first = ObserveThreeQueries(fixture, *context);
    fixture.SetRates(context.get(), second_rates);
    const ThreeQueryVelocities second = ObserveThreeQueries(fixture, *context);
    fixture.SetRates(context.get(),
                     kFirstScale * first_rates + kSecondScale * second_rates);
    const ThreeQueryVelocities combined =
        ObserveThreeQueries(fixture, *context);

    ExpectComponentsNear(
        combined.body,
        LinearCombination(first.body, second.body, kFirstScale, kSecondScale),
        "body velocity is linear in v");
    ExpectComponentsNear(
        combined.frame,
        LinearCombination(first.frame, second.frame, kFirstScale, kSecondScale),
        "frame velocity is linear in v");
    ExpectComponentsNear(
        combined.relative,
        LinearCombination(first.relative, second.relative, kFirstScale,
                          kSecondScale),
        "relative velocity is linear in v");
}

void CheckFreeBodyVelocityLayoutAndPointShift() {
    constexpr double kAngle = 0.64;
    const Eigen::Vector3d angular(0.9, -1.2, 1.7);
    const Eigen::Vector3d translational(-0.8, 1.3, -1.6);
    const Eigen::Vector3d point_in_body(0.21, -0.14, 0.32);

    MultibodyModel model;
    const RigidBodyHandle body =
        model.AddRigidBody("floating", SolidishBody(1.0));
    FixedFramePoseParameters point_pose;
    point_pose.p_PoFo_P = point_in_body;
    const FrameHandle point =
        model.AddFixedFrame("floating_point", body, point_pose);
    model.DeclareFreeBody(body);
    model.Finalize();

    auto context = model.CreateDefaultContext();
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    const auto q_range = model.GetFreeBodyPositionRange(body);
    positions[q_range.start()] = std::cos(kAngle / 2.0);
    positions[q_range.start() + 3] = std::sin(kAngle / 2.0);
    positions.segment<3>(q_range.start() + 4) =
        Eigen::Vector3d(0.3, -0.5, 0.7);
    model.SetGeneralizedPositions(context.get(), positions);

    Eigen::VectorXd velocities =
        Eigen::VectorXd::Zero(model.num_generalized_velocities());
    const auto v_range = model.GetFreeBodyVelocityRange(body);
    velocities.segment<3>(v_range.start()) = angular;
    velocities.segment<3>(v_range.start() + 3) = translational;
    model.SetGeneralizedVelocities(context.get(), velocities);

    ExpectVelocityNear(
        model.CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
            *context, body),
        {angular, translational},
        "a free body's first three rates are world angular velocity and its "
        "last three are body-origin world translational velocity");

    const Eigen::Matrix3d R_WB = RotationAboutZ(kAngle);
    const Eigen::Vector3d point_offset_W = R_WB * point_in_body;
    const Eigen::Vector3d point_velocity =
        translational + angular.cross(point_offset_W);
    ExpectVelocityNear(
        model.CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(*context,
                                                                      point),
        {angular, point_velocity},
        "a point fixed to a free body receives the angular point shift");

    ExpectVelocityNear(
        model.CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
            *context, point, model.world_frame(), model.body_frame(body)),
        {R_WB.transpose() * angular, R_WB.transpose() * point_velocity},
        "the free-body point velocity can be expressed in the body frame "
        "without rotating the stored generalized velocity");
}

void CheckRelationsTowardsWorldReverseVelocitySigns() {
    constexpr double kAngle = 0.41;
    constexpr double kAngularRate = 0.73;
    {
        MultibodyModel model;
        const RigidBodyHandle body =
            model.AddRigidBody("reversed_revolute", SolidishBody(1.0));
        const Eigen::Vector3d kJointOffsetInBody(0.24, -0.13, 0.19);
        FixedFramePoseParameters joint_frame_pose;
        joint_frame_pose.p_PoFo_P = kJointOffsetInBody;
        const FrameHandle joint_frame = model.AddFixedFrame(
            "reversed_revolute_joint_frame", body, joint_frame_pose);
        const JointHandle joint = model.AddRevoluteJoint(
            "towards_world", joint_frame, model.world_frame(),
            Eigen::Vector3d::UnitZ());
        model.Finalize();
        auto context = model.CreateDefaultContext();
        Eigen::VectorXd q =
            Eigen::VectorXd::Zero(model.num_generalized_positions());
        PlaceJointPosition(model, joint, kAngle, &q);
        model.SetGeneralizedPositions(context.get(), q);
        Eigen::VectorXd v =
            Eigen::VectorXd::Zero(model.num_generalized_velocities());
        PlaceJointVelocity(model, joint, kAngularRate, &v);
        model.SetGeneralizedVelocities(context.get(), v);
        const Eigen::Vector3d angular_velocity =
            -kAngularRate * Eigen::Vector3d::UnitZ();
        const Eigen::Vector3d position_from_joint_to_body_in_world =
            -RotationAboutZ(-kAngle) * kJointOffsetInBody;
        ExpectVelocityNear(
            model.CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, body),
            {angular_velocity,
             angular_velocity.cross(position_from_joint_to_body_in_world)},
            "a revolute relation stated towards the world reverses the "
            "positive rate and shifts it from the offset joint frame to the "
            "body origin");
    }

    constexpr double kLinearRate = -0.62;
    {
        MultibodyModel model;
        const RigidBodyHandle body =
            model.AddRigidBody("reversed_prismatic", SolidishBody(1.0));
        const JointHandle joint = model.AddPrismaticJoint(
            "towards_world", model.body_frame(body), model.world_frame(),
            Eigen::Vector3d::UnitX());
        model.Finalize();
        auto context = model.CreateDefaultContext();
        Eigen::VectorXd q =
            Eigen::VectorXd::Zero(model.num_generalized_positions());
        PlaceJointPosition(model, joint, 0.27, &q);
        model.SetGeneralizedPositions(context.get(), q);
        Eigen::VectorXd v =
            Eigen::VectorXd::Zero(model.num_generalized_velocities());
        PlaceJointVelocity(model, joint, kLinearRate, &v);
        model.SetGeneralizedVelocities(context.get(), v);
        ExpectVelocityNear(
            model.CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                *context, body),
            {Eigen::Vector3d::Zero(),
             -kLinearRate * Eigen::Vector3d::UnitX()},
            "a prismatic relation stated towards the world reverses the "
            "positive axis velocity");
    }
}

void CheckVelocityQueriesRejectForeignInputs() {
    MultibodyModel first;
    const RigidBodyHandle first_body =
        first.AddRigidBody("body", SolidishBody(1.0));
    first.DeclareFreeBody(first_body);
    first.Finalize();
    const auto first_context = first.CreateDefaultContext();

    MultibodyModel second;
    const RigidBodyHandle second_body =
        second.AddRigidBody("body", SolidishBody(1.0));
    second.DeclareFreeBody(second_body);
    second.Finalize();
    const auto second_context = second.CreateDefaultContext();

    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)first
                    .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                        *second_context, first_body);
            },
            "issued by a different model"),
        "a spatial-velocity query rejects a context from another model");
    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)first
                    .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                        *first_context, second_body);
            },
            "different model"),
        "a body spatial-velocity query rejects a foreign body handle");
    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)first
                    .CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                        *second_context, first.body_frame(first_body));
            },
            "issued by a different model"),
        "a frame spatial-velocity query rejects a context from another model "
        "at the public boundary");
    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)first
                    .CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
                        *second_context, first.body_frame(first_body),
                        first.world_frame(), first.world_frame());
            },
            "issued by a different model"),
        "a relative spatial-velocity query rejects a context from another "
        "model at the public boundary");
    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)first
                    .CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                        *first_context, second.body_frame(second_body));
            },
            "different model"),
        "a frame spatial-velocity query rejects a foreign frame handle");
    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)first
                    .CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
                        *first_context, second.body_frame(second_body),
                        first.world_frame(), first.world_frame());
            },
            "different model"),
        "a relative query resolves and rejects a foreign moving frame");
    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)first
                    .CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
                        *first_context, first.body_frame(first_body),
                        second.body_frame(second_body), first.world_frame());
            },
            "different model"),
        "a relative query resolves and rejects a foreign reference frame");
    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)first
                    .CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
                        *first_context, first.body_frame(first_body),
                        first.world_frame(), second.body_frame(second_body));
            },
            "different model"),
        "a relative query resolves and rejects a foreign expression frame");

    // Reading both components from returned temporaries is the common call
    // shape. The rvalue-qualified accessors return independent values.
    const Eigen::Vector3d angular =
        first.CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                 *first_context, first_body)
            .angular_velocity_radians_per_second();
    const Eigen::Vector3d translational =
        first.CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                 *first_context, first_body)
            .translational_velocity_at_frame_origin_meters_per_second();
    ExpectTrue(angular.isZero(0.0) && translational.isZero(0.0),
               "components read from returned temporaries remain valid");
}

void CheckVelocityQueriesRequireFinalization() {
    MultibodyModel unfinalized;
    const RigidBodyHandle body =
        unfinalized.AddRigidBody("body", SolidishBody(1.0));

    MultibodyModel context_owner;
    const RigidBodyHandle context_body =
        context_owner.AddRigidBody("context_body", SolidishBody(1.0));
    context_owner.DeclareFreeBody(context_body);
    context_owner.Finalize();
    const auto context = context_owner.CreateDefaultContext();

    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)unfinalized
                    .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                        *context, body);
            },
            "before Finalize()"),
        "a body spatial-velocity query rejects an unfinalized model at the "
        "public boundary");
    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)unfinalized
                    .CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                        *context, unfinalized.body_frame(body));
            },
            "before Finalize()"),
        "a frame spatial-velocity query rejects an unfinalized model at the "
        "public boundary");
    ExpectTrue(
        RefusalMentions(
            [&] {
                (void)unfinalized
                    .CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
                        *context, unfinalized.body_frame(body),
                        unfinalized.world_frame(), unfinalized.world_frame());
            },
            "before Finalize()"),
        "a relative spatial-velocity query rejects an unfinalized model at "
        "the public boundary");
}

}  // namespace

int main() {
    CheckNonCoplanarChainBranchPointAndExpressionSemantics();
    CheckZeroAndLinearityInGeneralizedVelocity();
    CheckFreeBodyVelocityLayoutAndPointShift();
    CheckRelationsTowardsWorldReverseVelocitySigns();
    CheckVelocityQueriesRejectForeignInputs();
    CheckVelocityQueriesRequireFinalization();

    if (failure_count > 0) {
        std::printf("%d spatial-kinematics check(s) failed\n", failure_count);
        return 1;
    }
    std::printf(
        "public spatial velocity preserves frame, point, expression and "
        "reversal semantics and is linear in generalized velocity\n");
    return 0;
}
