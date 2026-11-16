#include "vehicle_project_preview_computation.h"

#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "orvd/configuration/vehicle_contact_record.h"
#include "orvd/multibody_model/multibody_evaluation_context.h"
#include "orvd/multibody_model/multibody_rigid_pose.h"
#include "orvd/vehicle_project/free_body_placements.h"
#include "orvd/wheel_rail_contact/profile_points.h"

namespace orvd::vehicle_project_editing::internal {
namespace {

using vehicle_project::ElementKind;

// The body each frame name resolves to: a body's own name names the body's
// frame, and a fixed frame belongs to the body it is fixed to.
std::unordered_map<std::string, std::string> MapFrameNamesToBodies(
    const configuration::VehicleDefinition& vehicle) {
    std::unordered_map<std::string, std::string> body_of_frame;
    for (const configuration::VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        body_of_frame.emplace(body.name, body.name);
    }
    for (const configuration::VehicleFixedFrameDefinition& frame : vehicle.fixed_frames) {
        body_of_frame.emplace(frame.name, frame.body_name);
    }
    return body_of_frame;
}

// The free bodies without a placement and every body jointed to one of them,
// directly or through other bodies. No joint reaches the world, so every body
// hangs through joints from exactly one free body, whose pose fixes its own.
std::unordered_set<std::string> CollectUndrawableBodyNames(
    const vehicle_project::VehicleProject& project, const configuration::VehicleDefinition& vehicle,
    const std::unordered_map<std::string, std::string>& body_of_frame) {
    std::unordered_map<std::string, std::vector<std::string>> jointed_bodies;
    const auto relate = [&](const auto& joints) {
        for (const auto& joint : joints) {
            const std::string& parent_body = body_of_frame.at(joint.parent_frame_name);
            const std::string& child_body = body_of_frame.at(joint.child_frame_name);
            jointed_bodies[parent_body].push_back(child_body);
            jointed_bodies[child_body].push_back(parent_body);
        }
    };
    relate(vehicle.revolute_joints);
    relate(vehicle.ball_rpy_joints);
    relate(vehicle.weld_joints);

    std::vector<std::string> unvisited = vehicle_project::CollectUnplacedFreeBodyNames(project);
    std::unordered_set<std::string> undrawable(unvisited.begin(), unvisited.end());
    while (!unvisited.empty()) {
        const std::string body_name = std::move(unvisited.back());
        unvisited.pop_back();
        const auto neighbours = jointed_bodies.find(body_name);
        if (neighbours == jointed_bodies.end()) {
            continue;
        }
        for (const std::string& neighbour : neighbours->second) {
            if (undrawable.insert(neighbour).second) {
                unvisited.push_back(neighbour);
            }
        }
    }
    return undrawable;
}

}  // namespace

VehicleProjectPreview ComputeVehicleProjectPreview(const vehicle_project::VehicleProject& project,
                                                   const configuration::VehicleDefinition& vehicle,
                                                   const multibody_model::MultibodyModel& model,
                                                   std::uint64_t project_revision) {
    std::unordered_map<std::string, double> station_offset_of_body;
    for (const configuration::VehicleFreeBodyStationOffsetDefinition& offset :
         vehicle.mechanical_track_station_layout.free_body_station_offsets) {
        station_offset_of_body.emplace(offset.body_name, offset.station_offset_meters);
    }

    // The start-up assembly writes a free body's seven positions as the
    // quaternion of R_inertial_from_body = R_inertial_from_track *
    // R_local_track_from_body, in w, x, y, z order, followed by the origin
    // p_track + R_inertial_from_track * (0, lateral, vertical). On a straight,
    // level line whose layout reference station is the drawing frame's
    // origin, the local track frame at a body's station is the drawing frame
    // moved along +x by the body's mechanical station offset, so the two
    // reduce to the placement's own rotation and to (station offset, lateral,
    // vertical). Every other coordinate keeps the model's default, which puts
    // every revolute joint at zero and every Ball-RPY joint at its default
    // angles.
    const std::unique_ptr<multibody_model::MultibodyEvaluationContext> context =
        model.CreateDefaultContext();
    Eigen::VectorXd positions = context->generalized_positions();
    for (const vehicle_project::FreeBodyPlacement& placement : project.free_body_placements) {
        const int start =
            model.GetFreeBodyPositionRange(model.GetRigidBodyByName(placement.body_name)).start();
        const Eigen::Quaterniond& rotation_drawing_from_body =
            placement.rotation_local_track_from_body;
        positions[start + 0] = rotation_drawing_from_body.w();
        positions[start + 1] = rotation_drawing_from_body.x();
        positions[start + 2] = rotation_drawing_from_body.y();
        positions[start + 3] = rotation_drawing_from_body.z();
        positions.segment<3>(start + 4) =
            Eigen::Vector3d(station_offset_of_body.at(placement.body_name),
                            placement.lateral_offset_in_local_track_frame_meters,
                            placement.vertical_offset_in_local_track_frame_meters);
    }
    model.SetGeneralizedPositions(context.get(), positions);

    const std::unordered_map<std::string, std::string> body_of_frame =
        MapFrameNamesToBodies(vehicle);
    const std::unordered_set<std::string> undrawable_bodies =
        CollectUndrawableBodyNames(project, vehicle, body_of_frame);
    const auto drawable = [&](const std::string& frame_name) {
        return !undrawable_bodies.contains(body_of_frame.at(frame_name));
    };
    const auto pose_of_frame = [&](const std::string& frame_name) {
        return model.CalcPoseInWorld(*context, model.GetFrameByName(frame_name));
    };

    VehicleProjectPreview preview{.project_revision = project_revision};
    for (const configuration::VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        if (!drawable(body.name)) {
            preview.not_drawable_elements.push_back({ElementKind::kRigidBody, body.name});
            continue;
        }
        const multibody_model::RigidPose pose =
            model.CalcPoseInWorld(*context, model.GetRigidBodyByName(body.name));
        preview.rigid_bodies.push_back({body.name, pose.rotation(), pose.translation()});
    }
    for (const configuration::VehicleFixedFrameDefinition& frame : vehicle.fixed_frames) {
        if (!drawable(frame.name)) {
            preview.not_drawable_elements.push_back({ElementKind::kFixedFrame, frame.name});
            continue;
        }
        const multibody_model::RigidPose pose = pose_of_frame(frame.name);
        preview.fixed_frames.push_back({frame.name, pose.rotation(), pose.translation()});
    }

    // A joint keeps the origins of its two frames together, and both frames
    // hang from the same free body, so the parent frame decides both where the
    // joint is drawn and whether it can be.
    for (const configuration::VehicleRevoluteJointDefinition& joint : vehicle.revolute_joints) {
        if (!drawable(joint.parent_frame_name)) {
            preview.not_drawable_elements.push_back({ElementKind::kRevoluteJoint, joint.name});
            continue;
        }
        const multibody_model::RigidPose parent_frame = pose_of_frame(joint.parent_frame_name);
        preview.revolute_joints.push_back(
            {joint.name, parent_frame.translation(),
             parent_frame.rotation() * joint.axis_in_parent_frame.normalized()});
    }
    const auto draw_joint_locations = [&](ElementKind kind, const auto& joints,
                                          std::vector<JointLocationDrawing>& drawings) {
        for (const auto& joint : joints) {
            if (!drawable(joint.parent_frame_name)) {
                preview.not_drawable_elements.push_back({kind, joint.name});
                continue;
            }
            drawings.push_back({joint.name, pose_of_frame(joint.parent_frame_name).translation()});
        }
    };
    draw_joint_locations(ElementKind::kBallRpyJoint, vehicle.ball_rpy_joints,
                         preview.ball_rpy_joints);
    draw_joint_locations(ElementKind::kWeldJoint, vehicle.weld_joints, preview.weld_joints);

    const auto draw_force_elements = [&](ElementKind kind, const auto& elements) {
        for (const auto& element : elements) {
            if (!drawable(element.reference_frame_name) || !drawable(element.opposite_frame_name)) {
                preview.not_drawable_elements.push_back({kind, element.name});
                continue;
            }
            preview.force_elements.push_back(
                {{kind, element.name},
                 pose_of_frame(element.reference_frame_name).translation(),
                 pose_of_frame(element.opposite_frame_name).translation()});
        }
    };
    draw_force_elements(ElementKind::kTranslationalSpringDamper,
                        vehicle.translational_spring_dampers);
    draw_force_elements(ElementKind::kRollSpringDamperCouple, vehicle.roll_spring_damper_couples);
    draw_force_elements(ElementKind::kSeriesSpringViscousDamper,
                        vehicle.series_spring_viscous_dampers);
    draw_force_elements(ElementKind::kPointToPointSeriesSpringBilinearDamper,
                        vehicle.point_to_point_series_spring_bilinear_dampers);
    draw_force_elements(ElementKind::kSaturatedPiecewiseLinearDamper,
                        vehicle.saturated_piecewise_linear_dampers);
    draw_force_elements(ElementKind::kReferenceFrameLinearBushing,
                        vehicle.reference_frame_linear_bushings);
    draw_force_elements(ElementKind::kPointToPointLinearSpringDamper,
                        vehicle.point_to_point_linear_spring_dampers);
    draw_force_elements(ElementKind::kPointToPointHystereticSpring,
                        vehicle.point_to_point_hysteretic_springs);

    // One wheel per contact interface, in the declaration's interface order,
    // as vehicle_project_preview.h states. The carrier-frame components R_BW
    // gives are the wheel body's own for the reason scene_observation's
    // scene_topology.h gives for SceneWheelPlacement; the contact declaration
    // check that reason needs has passed, as this function requires.
    if (project.contact_record.has_value()) {
        const configuration::VehicleContactRecord& contact_record = *project.contact_record;
        std::unordered_map<std::string, Eigen::Matrix3d> rotation_body_from_profile_of_carrier;
        for (const configuration::VehicleContactCarrierDeclaration& carrier :
             contact_record.declaration.carriers) {
            rotation_body_from_profile_of_carrier.emplace(
                carrier.carrier_name, carrier.rotation_body_from_nonspinning_wheel_profile);
        }
        const double datum_magnitude_meters =
            contact_record.wheel.wheel_lateral_datum_magnitude_meters;
        for (const configuration::VehicleContactInterfaceDeclaration& interface :
             contact_record.declaration.interfaces) {
            if (undrawable_bodies.contains(interface.wheel_body_name)) {
                preview.not_drawable_elements.push_back(
                    {ElementKind::kContactInterface, interface.interface_name});
                continue;
            }
            // +magnitude for the right wheel, -magnitude for the left: the
            // sign MakePoseConstants gives the lateral datum in configuration's
            // wheel-rail contact assembly.
            const double signed_lateral_datum_meters =
                interface.side == wheel_rail_contact::WheelSide::kRight ? datum_magnitude_meters
                                                                        : -datum_magnitude_meters;
            const multibody_model::RigidPose wheel_body = model.CalcPoseInWorld(
                *context, model.GetRigidBodyByName(interface.wheel_body_name));
            const Eigen::Matrix3d rotation_drawing_from_profile =
                wheel_body.rotation() *
                rotation_body_from_profile_of_carrier.at(interface.carrier_name);
            preview.wheels.push_back(
                {interface.interface_name, interface.wheel_body_name,
                 wheel_body.translation() +
                     rotation_drawing_from_profile *
                         Eigen::Vector3d(0.0, signed_lateral_datum_meters, 0.0),
                 rotation_drawing_from_profile *
                     Eigen::Vector3d(0.0, std::copysign(1.0, signed_lateral_datum_meters), 0.0),
                 contact_record.wheel.nominal_rolling_radius_meters});
        }
    }
    return preview;
}

}  // namespace orvd::vehicle_project_editing::internal
