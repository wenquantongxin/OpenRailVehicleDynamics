#include "orvd/scene_observation/scene_topology.h"

#include <cstddef>
#include <stdexcept>
#include <string>

namespace orvd::scene_observation {

SceneWheelPlacement DeriveWheelPlacement(
    const forces::WheelRailContactInterfaceDefinition& interface,
    const forces::WheelRailContactCarrierDefinition& carrier,
    const wheel_rail_contact::WheelRailPoseConstants& constants) {
    // The profile datum sits on the axle at the signed lateral datum of this
    // side, in the non-spinning wheel-profile axes W. The carrier's fixed
    // basis maps W-frame components into body components.
    const Eigen::Matrix3d& rotation_body_from_profile =
        carrier.rotation_body_from_nonspinning_wheel_profile;
    const Eigen::Vector3d datum_in_profile(
        0.0, constants.wheel_lateral_datum_meters, 0.0);
    SceneWheelPlacement placement;
    placement.interface_name = interface.interface_name;
    placement.wheel_body_name = interface.wheel_body_name;
    placement.carrier_body_name = carrier.body_name;
    placement.side = interface.side;
    placement.datum_in_wheel_body_frame_meters =
        rotation_body_from_profile * datum_in_profile;
    placement.spin_axis_in_wheel_body_frame =
        rotation_body_from_profile * Eigen::Vector3d::UnitY();
    placement.nominal_rolling_radius_meters =
        constants.nominal_rolling_radius_meters;
    if (interface.independent_wheel_revolute_joint.has_value()) {
        placement.spin_joint_name =
            interface.independent_wheel_revolute_joint->joint_name;
    }
    return placement;
}

SceneTopology DescribeSceneTopology(
    const multibody_model::MultibodyModel& model,
    const forces::WheelRailContactForcePlan* contact_plan) {
    SceneTopology topology;
    topology.bodies.reserve(static_cast<std::size_t>(model.num_rigid_bodies()));
    for (int index = 0; index < model.num_rigid_bodies(); ++index) {
        const auto body = model.GetRigidBody(index);
        topology.bodies.push_back(SceneBody{
            std::string(model.GetRigidBodyName(body)), model.IsFreeBody(body)});
    }
    if (contact_plan == nullptr) {
        return topology;
    }
    topology.wheel_placements.reserve(
        static_cast<std::size_t>(contact_plan->interface_count()));
    for (int interface_index = 0;
         interface_index < contact_plan->interface_count();
         ++interface_index) {
        const auto& interface =
            contact_plan->interface_definition(interface_index);
        const forces::WheelRailContactCarrierDefinition* carrier = nullptr;
        for (int carrier_index = 0;
             carrier_index < contact_plan->carrier_count(); ++carrier_index) {
            const auto& candidate =
                contact_plan->carrier_definition(carrier_index);
            if (candidate.carrier_name == interface.carrier_name) {
                carrier = &candidate;
                break;
            }
        }
        if (carrier == nullptr) {
            throw std::logic_error(
                "scene topology: interface '" + interface.interface_name +
                "' names a carrier the contact plan does not define");
        }
        topology.wheel_placements.push_back(DeriveWheelPlacement(
            interface, *carrier, contact_plan->pose_constants(interface.side)));
    }
    return topology;
}

}  // namespace orvd::scene_observation
