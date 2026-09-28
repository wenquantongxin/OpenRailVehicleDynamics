#include <cmath>
#include <cstdio>
#include <span>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/scene_observation/body_state.h"
#include "orvd/scene_observation/scene_topology.h"
#include "orvd/scene_observation/track_sampling.h"
#include "orvd/scene_observation/wheel_spin_sampling.h"
#include "orvd/track_geometry/track_geometry.h"

// The public scene observation checked against stated values: a free body at
// a stated pose, wheel placement from a stated profile basis and datum, and a
// line with closed-form centerline and frame.

namespace {

using namespace orvd;

[[noreturn]] void Fail(const std::string& detail) {
    throw std::runtime_error(detail);
}

bool Near(double measured, double expected, double absolute_tolerance) {
    return std::abs(measured - expected) <= absolute_tolerance;
}

void VerifyFreeBodySampling() {
    multibody_model::MultibodyModel model;
    multibody_runtime::RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 1.0;
    inertia.unit_inertia_moments = Eigen::Vector3d(0.01, 0.02, 0.02);
    const auto body = model.AddRigidBody("test_body", inertia);
    model.DeclareFreeBody(body);
    model.Finalize();
    auto context = model.CreateDefaultContext();
    Eigen::VectorXd positions =
        Eigen::VectorXd::Zero(model.num_generalized_positions());
    positions[0] = std::cos(0.2);
    positions[3] = std::sin(0.2);
    positions.segment<3>(4) = Eigen::Vector3d(1.5, -2.25, 0.75);
    model.SetGeneralizedPositions(context.get(), positions);
    Eigen::VectorXd velocities =
        Eigen::VectorXd::Zero(model.num_generalized_velocities());
    const auto range = model.GetFreeBodyVelocityRange(body);
    velocities.segment<3>(range.start()) = Eigen::Vector3d(0.3, -0.7, 1.2);
    velocities.segment<3>(range.start() + 3) = Eigen::Vector3d(2.0, 3.0, -4.0);
    model.SetGeneralizedVelocities(context.get(), velocities);

    const scene_observation::BodyStateSampler sampler(model);
    const auto states = sampler.Sample(*context);
    if (sampler.names() != std::vector<std::string>{"test_body"} ||
        states.size() != 1 ||
        states[0].position_meters != std::array<double, 3>{1.5, -2.25, 0.75}) {
        Fail("free-body sampling did not return the stated position");
    }
    // The stored quaternion is a rotation of 0.4 rad about z; the sampled
    // quaternion may carry either sign.
    const double sign = states[0].orientation_wxyz[0] < 0.0 ? -1.0 : 1.0;
    if (!Near(sign * states[0].orientation_wxyz[0], std::cos(0.2), 1e-14) ||
        !Near(sign * states[0].orientation_wxyz[3], std::sin(0.2), 1e-14) ||
        !Near(states[0].orientation_wxyz[1], 0.0, 1e-14) ||
        !Near(states[0].orientation_wxyz[2], 0.0, 1e-14)) {
        Fail("free-body sampling did not return the stated orientation");
    }
    if (states[0].angular_velocity_radians_per_second !=
            std::array<double, 3>{0.3, -0.7, 1.2} ||
        states[0].linear_velocity_meters_per_second !=
            std::array<double, 3>{2.0, 3.0, -4.0}) {
        Fail("angular and origin-linear velocity slots were swapped or lost");
    }
    const auto topology = scene_observation::DescribeSceneTopology(model, nullptr);
    if (topology.bodies.size() != 1 || topology.bodies[0].name != "test_body" ||
        !topology.bodies[0].moves_freely_in_world ||
        !topology.wheel_placements.empty()) {
        Fail("topology without a contact plan did not list the free body alone");
    }
}

void VerifyWheelPlacement() {
    wheel_rail_contact::WheelRailPoseConstants right_constants;
    right_constants.wheel_lateral_datum_meters = 0.7465;
    right_constants.nominal_rolling_radius_meters = 0.43;
    wheel_rail_contact::WheelRailPoseConstants left_constants = right_constants;
    left_constants.wheel_lateral_datum_meters = -0.7465;

    // A carrier whose body basis is the source model basis, half a turn about
    // x away from the profile axes: body +y points left and body +z up. Its
    // logical carrier name differs from its body name on purpose: the
    // placement must carry the body.
    forces::WheelRailContactCarrierDefinition half_turn_carrier;
    half_turn_carrier.carrier_name = "axle_logical";
    half_turn_carrier.body_name = "axle_body";
    half_turn_carrier.rotation_body_from_nonspinning_wheel_profile =
        Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
    forces::WheelRailContactInterfaceDefinition right_interface;
    right_interface.interface_name = "wheel_r";
    right_interface.carrier_name = "axle_logical";
    right_interface.wheel_body_name = "wheel_r";
    right_interface.side = wheel_rail_contact::WheelSide::kRight;
    const auto right = scene_observation::DeriveWheelPlacement(
        right_interface, half_turn_carrier, right_constants);
    if (right.carrier_body_name != "axle_body") {
        Fail("the placement must carry the carrier's body name, not its "
             "logical carrier name");
    }
    if (right.wheel_body_name != "wheel_r" ||
        right.side != wheel_rail_contact::WheelSide::kRight ||
        !right.datum_in_wheel_body_frame_meters.isApprox(
            Eigen::Vector3d(0.0, -0.7465, 0.0), 1e-15) ||
        !right.spin_axis_in_wheel_body_frame.isApprox(
            Eigen::Vector3d(0.0, -1.0, 0.0), 1e-15) ||
        right.nominal_rolling_radius_meters != 0.43) {
        Fail("half-turn basis: the right wheel datum must sit at body -y");
    }
    forces::WheelRailContactInterfaceDefinition left_interface = right_interface;
    left_interface.interface_name = "wheel_l";
    left_interface.wheel_body_name = "wheel_l";
    left_interface.side = wheel_rail_contact::WheelSide::kLeft;
    const auto left = scene_observation::DeriveWheelPlacement(
        left_interface, half_turn_carrier, left_constants);
    if (!left.datum_in_wheel_body_frame_meters.isApprox(
            Eigen::Vector3d(0.0, 0.7465, 0.0), 1e-15)) {
        Fail("half-turn basis: the left wheel datum must sit at body +y");
    }

    // A rigid wheelset whose body axes are the profile axes: the wheel body
    // is the carrier body, and there is no independent spin joint.
    forces::WheelRailContactCarrierDefinition identity_carrier;
    identity_carrier.carrier_name = "wheelset_logical";
    identity_carrier.body_name = "wheelset";
    identity_carrier.rotation_body_from_nonspinning_wheel_profile =
        Eigen::Matrix3d::Identity();
    forces::WheelRailContactInterfaceDefinition wheelset_interface =
        right_interface;
    wheelset_interface.carrier_name = "wheelset_logical";
    wheelset_interface.wheel_body_name = "wheelset";
    const auto identity_right = scene_observation::DeriveWheelPlacement(
        wheelset_interface, identity_carrier, right_constants);
    if (identity_right.carrier_body_name != "wheelset" ||
        identity_right.wheel_body_name != identity_right.carrier_body_name ||
        identity_right.spin_joint_name.has_value()) {
        Fail("a rigid wheelset placement must name the wheelset as both wheel "
             "and carrier body, without a spin joint");
    }
    if (!identity_right.datum_in_wheel_body_frame_meters.isApprox(
            Eigen::Vector3d(0.0, 0.7465, 0.0), 1e-15) ||
        !identity_right.spin_axis_in_wheel_body_frame.isApprox(
            Eigen::Vector3d::UnitY(), 1e-15)) {
        Fail("identity basis: the right wheel datum must sit at body +y");
    }
}

// A free carrier with one wheel on a +y revolute joint. With the source
// body basis (half a turn about x from the profile axes) the wheel's spin axis
// is body -y, so a positive joint position is a negative spin angle; with the
// identity basis the two agree. A joint about +x is not a spin joint.
void VerifyWheelSpinSampling(const Eigen::Matrix3d& rotation_body_from_profile,
                             const Eigen::Vector3d& joint_axis,
                             double expected_sign) {
    multibody_model::MultibodyModel model;
    multibody_runtime::RigidBodyInertiaParameters inertia;
    inertia.mass_kilograms = 1.0;
    inertia.unit_inertia_moments = Eigen::Vector3d(0.01, 0.02, 0.02);
    const auto carrier = model.AddRigidBody("carrier", inertia);
    const auto wheel = model.AddRigidBody("wheel", inertia);
    model.DeclareFreeBody(carrier);
    model.AddRevoluteJoint("spin", model.body_frame(carrier),
                           model.body_frame(wheel), joint_axis, 0.0);
    model.Finalize();

    scene_observation::SceneTopology topology =
        scene_observation::DescribeSceneTopology(model, nullptr);
    scene_observation::SceneWheelPlacement placement;
    placement.interface_name = "wheel_r";
    placement.wheel_body_name = "wheel";
    placement.carrier_body_name = "carrier";
    placement.side = wheel_rail_contact::WheelSide::kRight;
    placement.datum_in_wheel_body_frame_meters =
        rotation_body_from_profile * Eigen::Vector3d(0.0, 0.7465, 0.0);
    placement.spin_axis_in_wheel_body_frame =
        rotation_body_from_profile * Eigen::Vector3d::UnitY();
    placement.nominal_rolling_radius_meters = 0.43;
    placement.spin_joint_name = "spin";
    topology.wheel_placements.push_back(placement);

    if (expected_sign == 0.0) {
        bool refused = false;
        try {
            scene_observation::WheelSpinAngleSampler rejected(model, topology);
        } catch (const std::invalid_argument&) {
            refused = true;
        }
        if (!refused) {
            Fail("a joint that is not about the spin axis was accepted");
        }
        return;
    }
    const scene_observation::WheelSpinAngleSampler sampler(model, topology);
    if (!sampler.available() || sampler.wheel_count() != 1) {
        Fail("the spin sampler did not bind the one jointed wheel");
    }
    auto context = model.CreateDefaultContext();
    Eigen::VectorXd positions = context->generalized_positions();
    const auto joint_range =
        model.GetJointPositionRange(model.GetJointByName("spin"));
    positions[joint_range.start()] = 4.0;  // more than a half turn
    model.SetGeneralizedPositions(context.get(), positions);
    double angle = 0.0;
    sampler.Sample(*context, std::span<double>(&angle, 1));
    if (!Near(angle, expected_sign * 4.0, 1e-15)) {
        Fail("the sampled spin angle is not the signed joint position");
    }
    // The wheel orientation must be the spin rotation about the placement's
    // axis by exactly that angle.
    const Eigen::Matrix3d relative =
        model.CalcPoseInWorld(*context, carrier).rotation().transpose() *
        model.CalcPoseInWorld(*context, wheel).rotation();
    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd(angle, placement.spin_axis_in_wheel_body_frame)
            .toRotationMatrix();
    if ((relative - expected).cwiseAbs().maxCoeff() > 1e-12) {
        Fail("the spin angle does not reproduce the wheel orientation about "
             "the spin axis");
    }
    // The topology without a spin joint makes the sampler unavailable.
    topology.wheel_placements[0].spin_joint_name.reset();
    const scene_observation::WheelSpinAngleSampler unavailable(model, topology);
    if (unavailable.available()) {
        Fail("a wheel without a spin joint was reported as sampled");
    }
}

track_geometry::TrackGeometry MakeLine(double curvature_radians_per_meter) {
    track_geometry::TrackScalarSegment curvature;
    curvature.length_meters = 200.0;
    curvature.shape = track_geometry::TrackScalarSegmentShape::kConstant;
    curvature.start_value = curvature_radians_per_meter;
    curvature.end_value = curvature_radians_per_meter;
    track_geometry::TrackScalarSegment level = curvature;
    level.start_value = 0.0;
    level.end_value = 0.0;
    return track_geometry::TrackGeometry(
        track_geometry::TrackScalarProfile(0.0, {curvature}, {}),
        track_geometry::TrackScalarProfile(0.0, {level}, {}),
        track_geometry::TrackVerticalProfile(
            0.0, {track_geometry::ConstantGradeSegment{200.0, 0.0}}, {}),
        1.5, 1.0);
}

void VerifyTrackSampling() {
    const scene_observation::RailDatumPlacement left{-0.75, 0.01};
    const scene_observation::RailDatumPlacement right{0.75, 0.01};
    scene_observation::TrackSampleTable table;
    const double stations[] = {0.0, 40.0, 100.0};

    scene_observation::SampleTrackGeometry(MakeLine(0.0), stations, left,
                                           right, table);
    if (table.stations_meters.size() != 3 ||
        table.centerline_in_inertial_meters[1] !=
            std::array<double, 3>{40.0, 0.0, 0.0} ||
        !Near(table.left_rail_datum_in_inertial_meters[1][1], -0.75, 1e-15) ||
        !Near(table.right_rail_datum_in_inertial_meters[1][1], 0.75, 1e-15) ||
        !Near(table.right_rail_datum_in_inertial_meters[1][2], 0.01, 1e-15) ||
        !Near(std::abs(table.rotation_inertial_from_track_wxyz[1][0]), 1.0,
              1e-15) ||
        table.curvature_radians_per_meter[1] != 0.0 ||
        table.superelevation_meters[1] != 0.0) {
        Fail("straight level line: centerline, rails or frame differ from "
             "the closed form");
    }

    // A constant right-hand curve: heading psi = kappa s, centerline
    // x = sin(psi)/kappa, y = (1 - cos(psi))/kappa, y positive to the right.
    constexpr double kCurvature = 1.0 / 300.0;
    scene_observation::SampleTrackGeometry(MakeLine(kCurvature), stations,
                                           left, right, table);
    const double heading = kCurvature * 100.0;
    const auto& point = table.centerline_in_inertial_meters[2];
    if (!Near(point[0], std::sin(heading) / kCurvature, 1e-8) ||
        !Near(point[1], (1.0 - std::cos(heading)) / kCurvature, 1e-8) ||
        !Near(point[2], 0.0, 1e-12) || table.curvature_radians_per_meter[2] != kCurvature) {
        Fail("curved line: the centerline differs from the closed form");
    }
    const auto& q = table.rotation_inertial_from_track_wxyz[2];
    const Eigen::Quaterniond orientation(q[0], q[1], q[2], q[3]);
    const Eigen::Vector3d tangent = orientation * Eigen::Vector3d::UnitX();
    if (!Near(tangent.x(), std::cos(heading), 1e-12) ||
        !Near(tangent.y(), std::sin(heading), 1e-12)) {
        Fail("curved line: the track frame x axis is not the heading");
    }
    const Eigen::Vector3d right_offset =
        Eigen::Vector3d(table.right_rail_datum_in_inertial_meters[2].data()) -
        Eigen::Vector3d(point.data());
    if (!Near(right_offset.x(), -0.75 * std::sin(heading), 1e-12) ||
        !Near(right_offset.y(), 0.75 * std::cos(heading), 1e-12)) {
        Fail("curved line: the right rail datum is not to the right of the "
             "heading");
    }

    bool empty_rejected = false;
    try {
        scene_observation::SampleTrackGeometry(MakeLine(0.0), {}, left, right,
                                               table);
    } catch (const std::invalid_argument&) {
        empty_rejected = true;
    }
    if (!empty_rejected) {
        Fail("an empty station list was accepted");
    }
}

}  // namespace

int main() {
    try {
        VerifyFreeBodySampling();
        VerifyWheelPlacement();
        VerifyWheelSpinSampling(Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal(),
                                Eigen::Vector3d::UnitY(), -1.0);
        VerifyWheelSpinSampling(Eigen::Matrix3d::Identity(),
                                Eigen::Vector3d::UnitY(), 1.0);
        VerifyWheelSpinSampling(Eigen::Matrix3d::Identity(),
                                Eigen::Vector3d::UnitX(), 0.0);
        VerifyTrackSampling();
        std::puts("scene observation sampling, wheel placement and track "
                  "sampling passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
