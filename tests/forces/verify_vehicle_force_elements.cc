#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "orvd/forces/vehicle_force_plan.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/multibody_runtime/multibody_physical_parameters.h"

namespace {

using orvd::forces::ForceElementAxis;
using orvd::forces::ForceElementEnd;
using orvd::forces::HalfAngleMidpointRollPitchYawBushing;
using orvd::forces::RollSpringDamperCouple;
using orvd::forces::SaturatedPiecewiseLinearDamper;
using orvd::forces::SaturatedPiecewiseLinearDamperPoint;
using orvd::forces::SeriesSpringViscousDamper;
using orvd::forces::TranslationalSpringDamper;
using orvd::forces::VehicleForceElementCollection;
using orvd::forces::VehicleForcePlan;
using orvd::multibody_model::AppliedBodyWrench;
using orvd::multibody_model::MultibodyModel;

static_assert(!std::is_constructible_v<
              VehicleForcePlan, MultibodyModel&&,
              VehicleForceElementCollection>);

constexpr double kMachine = std::numeric_limits<double>::epsilon();

int failure_count = 0;

void Expect(bool condition, const std::string& description) {
    if (!condition) {
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    }
}

// A fixture whose two ends do not coincide, whose reference end is rotated and
// turning, and whose relative motion is non-zero along all three axes.
//
// The vehicle's default context cannot be used for this contract: every
// generalized velocity is zero, while the side-roll and clipped families also
// emit zero wrench there. That makes the virtual-power checks degenerate even
// though several attachment pairs are metres apart. A purpose-built fixture
// supplies non-zero relative motion and non-collinear attachment geometry.
struct TwoBodyFixture {
    std::unique_ptr<MultibodyModel> model;
    ForceElementEnd reference_end;
    ForceElementEnd same_body_end;
    ForceElementEnd opposite_end;
    std::unique_ptr<orvd::multibody_model::MultibodyEvaluationContext> context;

    TwoBodyFixture() : model(std::make_unique<MultibodyModel>()) {
        orvd::multibody_runtime::RigidBodyInertiaParameters inertia;
        inertia.mass_kilograms = 7.0;
        inertia.unit_inertia_moments = Eigen::Vector3d(1.3, 1.1, 0.9);
        const auto first = model->AddRigidBody("first", inertia);
        const auto second = model->AddRigidBody("second", inertia);

        orvd::multibody_runtime::FixedFramePoseParameters reference_pose;
        // A rotated reference frame, so that "expressed in the reference frame"
        // is a different thing from "expressed in the world" and a gate can
        // tell them apart.
        const double angle = 0.37;
        reference_pose.R_PF << std::cos(angle), -std::sin(angle), 0.0,
            std::sin(angle), std::cos(angle), 0.0, 0.0, 0.0, 1.0;
        reference_pose.p_PoFo_P = Eigen::Vector3d(0.31, -0.17, 0.23);
        const auto reference_frame =
            model->AddFixedFrame("reference_attachment", first, reference_pose);
        orvd::multibody_runtime::FixedFramePoseParameters same_body_pose;
        same_body_pose.p_PoFo_P = Eigen::Vector3d(-0.09, 0.16, 0.28);
        const auto same_body_frame =
            model->AddFixedFrame("same_body_attachment", first, same_body_pose);

        orvd::multibody_runtime::FixedFramePoseParameters opposite_pose;
        opposite_pose.p_PoFo_P = Eigen::Vector3d(-0.41, 0.29, -0.13);
        const auto opposite_frame =
            model->AddFixedFrame("opposite_attachment", second, opposite_pose);

        model->DeclareFreeBody(first);
        model->DeclareFreeBody(second);
        model->Finalize();

        reference_end = ForceElementEnd{reference_frame};
        same_body_end = ForceElementEnd{same_body_frame};
        opposite_end = ForceElementEnd{opposite_frame};
        context = model->CreateDefaultContext();
        SetState();
    }

    // Positions the two bodies apart, with the reference body rotated, and gives
    // both bodies angular and translational velocity so that the relative
    // motion has content along every axis and the two ends' angular velocities
    // differ.
    void SetState() const {
        Eigen::VectorXd positions(model->num_generalized_positions());
        Eigen::VectorXd velocities(model->num_generalized_velocities());
        const Eigen::Quaterniond first_orientation(
            Eigen::AngleAxisd(0.21, Eigen::Vector3d(0.3, 0.5, 0.81).normalized()));
        positions.segment<4>(0) << first_orientation.w(), first_orientation.x(),
            first_orientation.y(), first_orientation.z();
        positions.segment<3>(4) << 0.13, -0.07, 0.19;
        const Eigen::Quaterniond second_orientation(
            Eigen::AngleAxisd(-0.14, Eigen::Vector3d(0.7, -0.2, 0.6).normalized()));
        positions.segment<4>(7) << second_orientation.w(),
            second_orientation.x(), second_orientation.y(),
            second_orientation.z();
        positions.segment<3>(11) << 0.92, 0.44, -0.51;
        velocities.segment<3>(0) << 0.23, -0.41, 0.17;
        velocities.segment<3>(3) << 0.11, 0.29, -0.37;
        velocities.segment<3>(6) << -0.19, 0.33, 0.27;
        velocities.segment<3>(9) << 0.41, -0.23, 0.13;
        model->SetGeneralizedState(context.get(), positions, velocities);
    }
};

struct MotionSample {
    Eigen::Vector3d displacement_in_reference;
    Eigen::Vector3d velocity_in_reference;
    Eigen::Vector3d angular_velocity_in_reference;
    Eigen::Matrix3d rotation_of_opposite_in_reference;
    Eigen::Matrix3d rotation_of_reference_in_world;
    Eigen::Vector3d displacement_in_world;
    Eigen::Vector3d reference_point_in_world;
    Eigen::Vector3d opposite_point_in_world;
};

// The same relative motion the plan reads, recomputed here from the model's
// pose and velocity queries rather than from the plan's own arithmetic.
MotionSample SampleMotion(const TwoBodyFixture& fixture) {
    const auto& model = *fixture.model;
    const auto reference_pose =
        model.CalcPoseInWorld(*fixture.context, fixture.reference_end.frame);
    const auto opposite_pose =
        model.CalcPoseInWorld(*fixture.context, fixture.opposite_end.frame);
    const auto relative =
        model.CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame(
            *fixture.context, fixture.opposite_end.frame,
            fixture.reference_end.frame, fixture.reference_end.frame);
    MotionSample sample;
    sample.reference_point_in_world = reference_pose.translation();
    sample.opposite_point_in_world = opposite_pose.translation();
    sample.displacement_in_world =
        opposite_pose.translation() - reference_pose.translation();
    sample.rotation_of_reference_in_world = reference_pose.rotation();
    sample.displacement_in_reference =
        reference_pose.rotation().transpose() * sample.displacement_in_world;
    sample.rotation_of_opposite_in_reference =
        reference_pose.rotation().transpose() * opposite_pose.rotation();
    sample.velocity_in_reference =
        relative.translational_velocity_at_frame_origin_meters_per_second();
    sample.angular_velocity_in_reference =
        relative.angular_velocity_radians_per_second();
    return sample;
}

Eigen::Vector3d PointVelocityInWorld(const TwoBodyFixture& fixture,
                                     const ForceElementEnd& end) {
    const auto velocity =
        fixture.model->CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
            *fixture.context, end.frame);
    return velocity.translational_velocity_at_frame_origin_meters_per_second();
}

Eigen::Vector3d AngularVelocityInWorld(const TwoBodyFixture& fixture,
                                       const ForceElementEnd& end) {
    return fixture.model
        ->CalcFrameSpatialVelocityRelativeToWorldExpressedInWorld(
            *fixture.context, end.frame)
        .angular_velocity_radians_per_second();
}

// The power the two wrenches deliver to the two bodies, computed from the
// wrenches and the endpoint velocities and not from any constitutive law. This
// is what makes the energy statements below independent of the code they check.
double AppliedPower(const TwoBodyFixture& fixture,
                    const std::vector<AppliedBodyWrench>& wrenches) {
    const Eigen::Vector3d reference_velocity =
        PointVelocityInWorld(fixture, fixture.reference_end);
    const Eigen::Vector3d opposite_velocity =
        PointVelocityInWorld(fixture, fixture.opposite_end);
    const Eigen::Vector3d reference_angular =
        AngularVelocityInWorld(fixture, fixture.reference_end);
    const Eigen::Vector3d opposite_angular =
        AngularVelocityInWorld(fixture, fixture.opposite_end);
    return wrenches[0].force_newtons.dot(reference_velocity) +
           wrenches[0].torque_about_point_newton_metres.dot(reference_angular) +
           wrenches[1].force_newtons.dot(opposite_velocity) +
           wrenches[1].torque_about_point_newton_metres.dot(opposite_angular);
}

std::vector<AppliedBodyWrench> Evaluate(
    const TwoBodyFixture& fixture, const VehicleForcePlan& plan,
    const Eigen::VectorXd& series_forces, const Eigen::VectorXd& nominal_forces,
    Eigen::VectorXd& series_derivatives) {
    std::vector<AppliedBodyWrench> wrenches(
        static_cast<std::size_t>(plan.body_wrench_count()));
    plan.CalcAppliedForces(*fixture.context, series_forces, nominal_forces,
                           wrenches, series_derivatives);
    return wrenches;
}

VehicleForceElementCollection WithTranslationalElements(
    std::vector<TranslationalSpringDamper> elements) {
    VehicleForceElementCollection collection;
    collection.translational_spring_dampers = std::move(elements);
    return collection;
}

VehicleForceElementCollection WithRollElements(
    std::vector<RollSpringDamperCouple> elements) {
    VehicleForceElementCollection collection;
    collection.roll_spring_damper_couples = std::move(elements);
    return collection;
}

VehicleForceElementCollection WithSeriesElements(
    std::vector<SeriesSpringViscousDamper> elements) {
    VehicleForceElementCollection collection;
    collection.series_spring_viscous_dampers = std::move(elements);
    return collection;
}

VehicleForceElementCollection WithClippedElements(
    std::vector<SaturatedPiecewiseLinearDamper> elements) {
    VehicleForceElementCollection collection;
    collection.saturated_piecewise_linear_dampers = std::move(elements);
    return collection;
}

VehicleForceElementCollection WithBushingElements(
    std::vector<HalfAngleMidpointRollPitchYawBushing> elements) {
    VehicleForceElementCollection collection;
    collection.half_angle_midpoint_roll_pitch_yaw_bushings =
        std::move(elements);
    return collection;
}

// ---------------------------------------------------------------------------
// Gate 3 — the full wrench, and the endpoint power identity
// ---------------------------------------------------------------------------

void CheckFullWrenchAndPower() {
    const TwoBodyFixture fixture;
    const Eigen::Vector3d stiffness(31000.0, 17000.0, 23000.0);
    const Eigen::Vector3d damping(410.0, 230.0, 370.0);
    VehicleForcePlan plan(
        *fixture.model,
        WithTranslationalElements({TranslationalSpringDamper{
            "test_translation", fixture.reference_end, fixture.opposite_end,
            stiffness, damping}}));
    Eigen::VectorXd no_state(0);
    Eigen::VectorXd nominal(3);
    nominal << 1300.0, -900.0, 2100.0;
    Eigen::VectorXd derivatives(0);
    const auto wrenches =
        Evaluate(fixture, plan, no_state, nominal, derivatives);

    const MotionSample motion = SampleMotion(fixture);
    const Eigen::Vector3d force_in_reference =
        stiffness.cwiseProduct(motion.displacement_in_reference) +
        damping.cwiseProduct(motion.velocity_in_reference) + nominal;
    const Eigen::Vector3d force_in_world =
        motion.rotation_of_reference_in_world * force_in_reference;
    const Eigen::Vector3d support_moment =
        motion.displacement_in_world.cross(force_in_world);

    Expect(motion.displacement_in_world.norm() > 0.5,
           "the fixture's two ends are far apart, so the support moment is a "
           "real quantity rather than zero");
    Expect(std::abs(force_in_world.normalized().dot(
               motion.displacement_in_world.normalized())) < 0.99,
           "the constitutive force is not along the line joining the ends, "
           "which is what makes the support moment necessary");

    // The per-body wrench, componentwise against the analytic contract. This is
    // the assertion that survives every wrong emission below; the balance
    // identities on their own do not.
    Expect((wrenches[0].force_newtons - force_in_world).cwiseAbs().maxCoeff() <=
               1.0e-9 * force_in_world.cwiseAbs().maxCoeff(),
           "the reference end receives the constitutive force");
    Expect((wrenches[0].torque_about_point_newton_metres - support_moment)
                   .cwiseAbs()
                   .maxCoeff() <= 1.0e-9 * support_moment.cwiseAbs().maxCoeff(),
           "the reference end receives the support moment (opposite point minus "
           "reference point) crossed with that force");
    Expect((wrenches[1].force_newtons + force_in_world).cwiseAbs().maxCoeff() <=
               1.0e-9 * force_in_world.cwiseAbs().maxCoeff(),
           "the opposite end receives the equal and opposite force");
    Expect(wrenches[1].torque_about_point_newton_metres ==
               Eigen::Vector3d::Zero(),
           "the opposite end receives no free moment");
    const auto reference_point = fixture.model->CalcFrameOriginAsBodyFixedPoint(
        *fixture.context, fixture.reference_end.frame);
    const auto opposite_point = fixture.model->CalcFrameOriginAsBodyFixedPoint(
        *fixture.context, fixture.opposite_end.frame);
    Expect(wrenches[0].body == reference_point.body &&
               wrenches[1].body == opposite_point.body &&
               wrenches[0].point_position_in_body_frame_meters ==
                   reference_point.position_in_body_frame_meters &&
               wrenches[1].point_position_in_body_frame_meters ==
                   opposite_point.position_in_body_frame_meters,
           "each wrench lands on the body and at the point its end names");

    // Balance about an arbitrary origin.
    const Eigen::Vector3d moment_centre(0.83, -1.27, 0.44);
    const Eigen::Vector3d total_force =
        wrenches[0].force_newtons + wrenches[1].force_newtons;
    const Eigen::Vector3d total_moment =
        wrenches[0].torque_about_point_newton_metres +
        wrenches[1].torque_about_point_newton_metres +
        (motion.reference_point_in_world - moment_centre)
            .cross(wrenches[0].force_newtons) +
        (motion.opposite_point_in_world - moment_centre)
            .cross(wrenches[1].force_newtons);
    const double scale = force_in_world.norm() *
                         std::max(1.0, motion.displacement_in_world.norm());
    Expect(total_force.cwiseAbs().maxCoeff() <= 1.0e-9 * force_in_world.norm(),
           "the pair exerts no net force");
    Expect(total_moment.cwiseAbs().maxCoeff() <= 1.0e-9 * scale,
           "the pair exerts no net moment about an arbitrary point");

    // What each wrong way of emitting the pair would give. Stating them here
    // rather than only mutating the library keeps the reason each is wrong in
    // the same place as the contract, and shows what each identity can and
    // cannot see.
    const Eigen::Vector3d moment_without_support =
        total_moment - wrenches[0].torque_about_point_newton_metres;
    Expect(moment_without_support.cwiseAbs().maxCoeff() > 1.0e-3 * scale,
           "omitting the support moment leaves a net moment the balance above "
           "would catch");
    // Both forces moved to the midpoint: still balanced, for any moment centre,
    // because a pair of equal and opposite forces at one point always is. Only
    // the per-body assertion and the power identity see this one.
    // Moving both forces to their midpoint would remain balanced by algebra;
    // the product-independent balance above therefore cannot distinguish that
    // wrong implementation. The endpoint-power check below can.

    // The endpoint power identity, which does see the midpoint variant.
    const double applied_power = AppliedPower(fixture, wrenches);
    const double constitutive_power =
        -force_in_reference.dot(motion.velocity_in_reference);
    Expect(std::abs(applied_power - constitutive_power) <=
               1.0e-9 * std::abs(constitutive_power),
           "the power the two wrenches deliver equals the constitutive power, "
           "computed from the wrenches and the endpoint velocities rather than "
           "from the force law");
    const Eigen::Vector3d reference_angular =
        AngularVelocityInWorld(fixture, fixture.reference_end);
    const Eigen::Vector3d opposite_angular =
        AngularVelocityInWorld(fixture, fixture.opposite_end);
    const double midpoint_power_error =
        force_in_world.dot(
            0.5 * (PointVelocityInWorld(fixture, fixture.reference_end) -
                   PointVelocityInWorld(fixture, fixture.opposite_end))) -
        constitutive_power;
    Expect(std::abs(midpoint_power_error) > 1.0e-3 * std::abs(constitutive_power),
           "and it separates the midpoint variant, whose endpoint power differs "
           "measurably");
    Expect((reference_angular - opposite_angular).cwiseAbs().maxCoeff() > 0.1,
           "the two ends turn at different rates, which is the condition under "
           "which the power identity can see a misplaced application point");
}

// ---------------------------------------------------------------------------
// Gate 4 — storage and dissipation, family by family
// ---------------------------------------------------------------------------

void CheckTranslationalEnergyIdentity() {
    const TwoBodyFixture fixture;
    const Eigen::Vector3d stiffness(31000.0, 17000.0, 23000.0);
    const Eigen::Vector3d damping(410.0, 230.0, 370.0);
    VehicleForcePlan plan(
        *fixture.model,
        WithTranslationalElements({TranslationalSpringDamper{
            "test_translation", fixture.reference_end, fixture.opposite_end,
            stiffness, damping}}));
    Eigen::VectorXd no_state(0);
    Eigen::VectorXd nominal(3);
    nominal << 1300.0, -900.0, 2100.0;
    Eigen::VectorXd derivatives(0);
    const auto wrenches =
        Evaluate(fixture, plan, no_state, nominal, derivatives);
    const MotionSample motion = SampleMotion(fixture);

    // With a preload the potential is affine, not quadratic:
    //   U = 1/2 d' K d + F_nominal . d
    // and its rate is (K d + F_nominal) . d_dot. The damper dissipates
    //   D = d_dot' C d_dot >= 0.
    const double potential_rate =
        (stiffness.cwiseProduct(motion.displacement_in_reference) + nominal)
            .dot(motion.velocity_in_reference);
    const double dissipation = motion.velocity_in_reference.dot(
        damping.cwiseProduct(motion.velocity_in_reference));
    const double absorbed = -AppliedPower(fixture, wrenches);
    const double scale =
        std::max({1.0, std::abs(potential_rate), std::abs(dissipation)});
    Expect(std::abs(absorbed - (potential_rate + dissipation)) <=
               512.0 * kMachine * scale,
           "an algebraic translational spring-damper absorbs exactly the rate "
           "of its affine potential plus its dissipation");
    Expect(dissipation > 0.0, "and its dissipation is positive on this fixture");
    Expect(std::abs(nominal.dot(motion.velocity_in_reference)) >
               1.0e-3 * std::abs(potential_rate),
           "the preload contributes measurably to the potential rate, so the "
           "affine term is exercised rather than assumed");
}

void CheckSeriesEnergyIdentity() {
    const TwoBodyFixture fixture;
    constexpr double kStiffness = 4.0e5;
    constexpr double kDamping = 9.0e3;
    VehicleForcePlan plan(
        *fixture.model,
        WithSeriesElements({SeriesSpringViscousDamper{
            "test_series", fixture.reference_end, fixture.opposite_end,
            ForceElementAxis::kLateral, kStiffness, kDamping}}));
    Eigen::VectorXd series(1);
    series << 3700.0;
    Eigen::VectorXd nominal(0);
    Eigen::VectorXd derivatives(1);
    const auto wrenches = Evaluate(fixture, plan, series, nominal, derivatives);

    // The spring stores F^2 / (2 K) and the damper dissipates F^2 / C.
    const double force = series[0];
    const double stored_rate = force * derivatives[0] / kStiffness;
    const double dissipation = force * force / kDamping;
    const double absorbed = -AppliedPower(fixture, wrenches);
    const double scale =
        std::max({1.0, std::abs(stored_rate), std::abs(dissipation)});
    Expect(std::abs(absorbed - (stored_rate + dissipation)) <=
               512.0 * kMachine * scale,
           "a series spring-viscous damper absorbs the rate of its stored "
           "energy plus its dissipation");
    Expect(std::abs(stored_rate) > 1.0 && dissipation > 1.0,
           "both terms are far from zero on this fixture");
}

void CheckClippedDamperLawAndDissipation() {
    const TwoBodyFixture fixture;
    SaturatedPiecewiseLinearDamper damper{
        "test_clipped", fixture.reference_end, fixture.opposite_end,
        ForceElementAxis::kLongitudinal,
        {SaturatedPiecewiseLinearDamperPoint{0.0, 0.0},
         SaturatedPiecewiseLinearDamperPoint{0.02, 12000.0},
         SaturatedPiecewiseLinearDamperPoint{0.04, 12000.0}}};

    // Continuity at the origin, on the linear run, at the saturation corner and
    // inside both saturated regions; and odd symmetry everywhere.
    Expect(VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(damper, 0.0) ==
               0.0,
           "the clipped law passes through the origin");
    Expect(std::abs(VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(
                        damper, 0.01) -
                    6000.0) <= 1.0e-9,
           "it interpolates linearly inside the first segment");
    Expect(std::abs(VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(
                        damper, 0.02) -
                    12000.0) <= 1.0e-9,
           "it reaches the stated force at the saturation corner");
    for (const double speed : {0.041, 0.5, 12.0}) {
        Expect(VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(damper,
                                                                    speed) ==
                   12000.0,
               "beyond the last breakpoint it holds rather than extrapolating");
    }
    double worst_odd_residual = 0.0;
    double worst_continuity_jump = 0.0;
    double previous =
        VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(damper, -0.06);
    for (int step = -600; step <= 600; ++step) {
        const double speed = 1.0e-4 * static_cast<double>(step);
        const double force =
            VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(damper, speed);
        const double mirrored =
            VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(damper, -speed);
        worst_odd_residual = std::max(worst_odd_residual, std::abs(force + mirrored));
        worst_continuity_jump =
            std::max(worst_continuity_jump, std::abs(force - previous));
        previous = force;
        Expect(force * speed >= 0.0,
               "the clipped damper never returns power at any sampled speed");
    }
    Expect(worst_odd_residual <= 1.0e-9,
           "the law is odd, because only its non-negative half is written down");
    // One step of 1e-4 m/s on the steepest segment moves the force by
    // 600000 * 1e-4 = 60 N; anything much larger would be a jump.
    Expect(worst_continuity_jump <= 61.0,
           "the law is continuous: no sampled step jumps by more than the "
           "steepest segment's slope allows");

    VehicleForcePlan plan(
        *fixture.model,
        WithClippedElements({damper}));
    Eigen::VectorXd none(0);
    Eigen::VectorXd derivatives(0);
    const auto wrenches = Evaluate(fixture, plan, none, none, derivatives);
    const double absorbed = -AppliedPower(fixture, wrenches);
    Expect(absorbed >= 0.0,
           "a damper with no storage absorbs non-negative power");
    Expect(absorbed > 1.0,
           "and it absorbs measurably on this fixture, so the sign statement is "
           "not a comparison of two zeros");
}

void CheckRollCoupleAndItsFiniteAttitudeResidual() {
    const TwoBodyFixture fixture;
    constexpr double kStiffness = 1.5e6;
    constexpr double kDamping = 2.5e2;
    VehicleForcePlan plan(
        *fixture.model,
        WithRollElements({RollSpringDamperCouple{
            "test_roll", fixture.reference_end, fixture.opposite_end,
            kStiffness, kDamping}}));
    Eigen::VectorXd none(0);
    Eigen::VectorXd derivatives(0);
    const auto wrenches = Evaluate(fixture, plan, none, none, derivatives);
    const MotionSample motion = SampleMotion(fixture);

    // A pure couple: no force at all, and equal and opposite moments.
    Expect(wrenches[0].force_newtons == Eigen::Vector3d::Zero() &&
               wrenches[1].force_newtons == Eigen::Vector3d::Zero(),
           "a roll couple exerts no translational force");
    Expect((wrenches[0].torque_about_point_newton_metres +
            wrenches[1].torque_about_point_newton_metres)
                   .cwiseAbs()
                   .maxCoeff() <= 1.0e-9,
           "its two moments are equal and opposite");

    // The moment is about the reference frame's own first axis and nothing
    // else. An implementation that mapped it through an attitude Jacobian would
    // put components on the other two axes.
    const Eigen::Vector3d moment_in_reference =
        motion.rotation_of_reference_in_world.transpose() *
        wrenches[0].torque_about_point_newton_metres;
    Expect(std::abs(moment_in_reference.y()) <=
                   1.0e-9 * std::abs(moment_in_reference.x()) &&
               std::abs(moment_in_reference.z()) <=
                   1.0e-9 * std::abs(moment_in_reference.x()),
           "the moment lies on the reference frame's first axis alone, which a "
           "generalized-force map through Euler angles would not give");

    const double roll = motion.rotation_of_opposite_in_reference(2, 1);
    const double roll_rate = motion.angular_velocity_in_reference.x();
    Expect(std::abs(roll) > 1.0e-3,
           "the fixture is at a finite roll, so the linearisation residual "
           "below is a real quantity");

    // The geometric rate of the roll coordinate is the exact derivative of that
    // matrix entry, which is not the same as the angular velocity's first
    // component once the attitude is finite:
    //   d/dt R(2,1) = [ -omega x R ](2,1)  for R the rotation of the opposite
    // frame in the reference frame, with omega the relative angular velocity in
    // the reference frame.
    const Eigen::Matrix3d& rotation = motion.rotation_of_opposite_in_reference;
    const Eigen::Vector3d omega = motion.angular_velocity_in_reference;
    Eigen::Matrix3d skew;
    skew << 0.0, -omega.z(), omega.y(), omega.z(), 0.0, -omega.x(), -omega.y(),
        omega.x(), 0.0;
    const double geometric_roll_rate = (skew * rotation)(2, 1);

    const double potential_rate = kStiffness * roll * geometric_roll_rate;
    const double dissipation = kDamping * roll_rate * roll_rate;
    const double residual =
        kStiffness * roll * (roll_rate - geometric_roll_rate);
    const double absorbed = -AppliedPower(fixture, wrenches);
    const double scale = std::max({1.0, std::abs(potential_rate),
                                   std::abs(dissipation), std::abs(residual)});
    Expect(std::abs(absorbed - (potential_rate + dissipation + residual)) <=
               512.0 * kMachine * scale,
           "the roll couple absorbs the potential rate plus the dissipation "
           "plus the finite-attitude linearisation residual, rebuilt as its own "
           "term");
    Expect(std::abs(residual) > 1.0e-4 * std::abs(potential_rate),
           "that residual is measurable at this attitude rather than being "
           "absorbed into a tolerance; the fixture carries relative yaw and "
           "pitch rate, without which its leading term would vanish");
}

// ---------------------------------------------------------------------------
// Gate 1 — the analytic oracle for the series law
// ---------------------------------------------------------------------------

void CheckSeriesAnalyticDerivative() {
    // The full time-domain analytic response is checked through the real
    // VehicleForcePlan -> SystemRhsBridge -> CVODE chain in the integrator
    // gate.  This family-level gate keeps only the independent one-step law.
    constexpr double kStiffness = 2000.0;
    constexpr double kDamping = 500.0;
    const TwoBodyFixture fixture;
    VehicleForcePlan plan(
        *fixture.model,
        WithSeriesElements({SeriesSpringViscousDamper{
            "test_series", fixture.reference_end, fixture.opposite_end,
            ForceElementAxis::kLateral, kStiffness, kDamping}}));
    Eigen::VectorXd series(1);
    series << 640.0;
    Eigen::VectorXd nominal(0);
    Eigen::VectorXd derivatives(1);
    (void)Evaluate(fixture, plan, series, nominal, derivatives);
    const MotionSample motion = SampleMotion(fixture);
    const double expected_rate =
        kStiffness * motion.velocity_in_reference.y() -
        (kStiffness / kDamping) * series[0];
    Expect(std::abs(derivatives[0] - expected_rate) <=
               1.0e-9 * std::abs(expected_rate),
           "the plan produces the same state derivative the law states, along "
           "the axis the element names");
    Expect(std::abs(motion.velocity_in_reference.y()) > 1.0e-3,
           "the fixture's relative velocity along that axis is non-zero, so the "
           "velocity term is exercised");
}

void CheckPlanRefusals() {
    const TwoBodyFixture fixture;
    const auto refuses = [](auto&& build, const std::string& description) {
        try {
            build();
        } catch (const std::exception&) {
            return;
        }
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    };
    const auto refusal_mentions = [](auto&& build, std::string_view fragment,
                                     const std::string& description) {
        try {
            build();
        } catch (const std::exception& error) {
            if (std::string_view(error.what()).find(fragment) !=
                std::string_view::npos) {
                return;
            }
        }
        std::printf("FAIL %s\n", description.c_str());
        ++failure_count;
    };
    refuses(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithTranslationalElements({TranslationalSpringDamper{
                    "same_end", fixture.reference_end, fixture.reference_end,
                    Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero()}}));
        },
        "an element naming one frame as both ends is refused");
    refuses(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithTranslationalElements({TranslationalSpringDamper{
                    "same_body", fixture.reference_end, fixture.same_body_end,
                    Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero()}}));
        },
        "two distinct frames on the same body are refused as force ends");
    refuses(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithSeriesElements({SeriesSpringViscousDamper{
                    "zero_series", fixture.reference_end,
                    fixture.opposite_end, ForceElementAxis::kLateral, 0.0,
                    1000.0}}));
        },
        "a series element with zero stiffness is refused: it has no time "
        "constant and belongs to another family");
    refusal_mentions(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithSeriesElements({SeriesSpringViscousDamper{
                    "overflowing_series", fixture.reference_end,
                    fixture.opposite_end, ForceElementAxis::kLateral, 1.0e9,
                    1.0e-300}}));
        },
        "overflowing_series",
        "a series element whose stiffness-to-damping ratio overflows is "
        "refused by name before it can produce NaN");
    refuses(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithClippedElements({SaturatedPiecewiseLinearDamper{
                    "unordered_curve", fixture.reference_end,
                    fixture.opposite_end, ForceElementAxis::kLateral,
                    {SaturatedPiecewiseLinearDamperPoint{0.0, 0.0},
                     SaturatedPiecewiseLinearDamperPoint{0.02, 12000.0},
                     SaturatedPiecewiseLinearDamperPoint{0.01, 9000.0}}}}));
        },
        "a curve whose velocities do not ascend is refused");
    refuses(
        [&] {
            SaturatedPiecewiseLinearDamper empty;
            empty.name = "empty_curve";
            static_cast<void>(
                VehicleForcePlan::SaturatedPiecewiseLinearDamperForce(empty,
                                                                       0.0));
        },
        "the public constitutive sampler rejects an empty raw curve instead "
        "of dereferencing it");
    refuses(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithClippedElements({SaturatedPiecewiseLinearDamper{
                    "missing_origin", fixture.reference_end,
                    fixture.opposite_end, ForceElementAxis::kLateral,
                    {SaturatedPiecewiseLinearDamperPoint{0.01, 500.0},
                     SaturatedPiecewiseLinearDamperPoint{0.02, 12000.0}}}}));
        },
        "a curve that does not start at the origin is refused");
    refuses(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithTranslationalElements({TranslationalSpringDamper{
                    "", fixture.reference_end, fixture.opposite_end,
                    Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero()}}));
        },
        "an empty force-element name is refused");
    refuses(
        [&] {
            VehicleForceElementCollection elements;
            elements.translational_spring_dampers = {
                TranslationalSpringDamper{
                    "duplicate", fixture.reference_end, fixture.opposite_end,
                    Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero()}};
            elements.series_spring_viscous_dampers = {
                SeriesSpringViscousDamper{
                    "duplicate", fixture.reference_end, fixture.opposite_end,
                    ForceElementAxis::kLateral, 1000.0, 100.0}};
            VehicleForcePlan plan(
                *fixture.model, std::move(elements));
        },
        "force-element names are unique across constitutive families");
    refusal_mentions(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithTranslationalElements({TranslationalSpringDamper{
                    "active_spring", fixture.reference_end,
                    fixture.opposite_end, Eigen::Vector3d(-1.0, 2.0, 3.0),
                    Eigen::Vector3d::Zero()}}));
        },
        "active_spring",
        "a negative translational stiffness is refused with the element name");
    refuses(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithRollElements({RollSpringDamperCouple{
                    "active_roll", fixture.reference_end,
                    fixture.opposite_end, 1000.0, -1.0}}));
        },
        "a negative roll damping coefficient is refused");
    refuses(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithClippedElements({SaturatedPiecewiseLinearDamper{
                    "active_curve", fixture.reference_end,
                    fixture.opposite_end, ForceElementAxis::kLateral,
                    {SaturatedPiecewiseLinearDamperPoint{0.0, 0.0},
                     SaturatedPiecewiseLinearDamperPoint{0.02, -10.0}}}}));
        },
        "a negative force on the non-negative half of a damper curve is "
        "refused");
    refusal_mentions(
        [&] {
            VehicleForcePlan plan(
                *fixture.model,
                WithBushingElements({HalfAngleMidpointRollPitchYawBushing{
                    "active_bushing", fixture.reference_end,
                    fixture.opposite_end, Eigen::Vector3d::Zero(),
                    Eigen::Vector3d(100.0, -1.0, 100.0),
                    Eigen::Vector3d::Constant(1.0e8),
                    Eigen::Vector3d::Constant(100.0)}}));
        },
        "active_bushing",
        "a negative bushing coefficient is refused with the element name");
}

}  // namespace

int main() {
    try {
        CheckFullWrenchAndPower();
        CheckTranslationalEnergyIdentity();
        CheckSeriesEnergyIdentity();
        CheckClippedDamperLawAndDissipation();
        CheckRollCoupleAndItsFiniteAttitudeResidual();
        CheckSeriesAnalyticDerivative();
        CheckPlanRefusals();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle force elements failed: %s\n",
                     error.what());
        return 1;
    }
    if (failure_count != 0) {
        std::printf("%d vehicle force element checks failed\n", failure_count);
        return 1;
    }
    std::printf("vehicle force elements verified\n");
    return 0;
}
