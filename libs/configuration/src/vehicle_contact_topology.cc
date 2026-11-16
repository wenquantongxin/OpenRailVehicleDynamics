#include "orvd/configuration/vehicle_contact_topology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace orvd::configuration {
namespace {

using wheel_rail_contact::WheelSide;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("vehicle contact declaration: " + detail);
}

const VehicleRigidBodyDefinition& RequireBody(const VehicleDefinition& vehicle,
                                              const std::string& body_name,
                                              const std::string& subject) {
    const auto body =
        std::ranges::find(vehicle.rigid_bodies, body_name, &VehicleRigidBodyDefinition::name);
    if (body == vehicle.rigid_bodies.end()) {
        Reject(subject + " names undeclared rigid body '" + body_name + "'");
    }
    return *body;
}

void RequireProfileBasis(const VehicleContactCarrierDeclaration& carrier) {
    const Eigen::Matrix3d& rotation = carrier.rotation_body_from_nonspinning_wheel_profile;
    constexpr double kRotationTolerance = 1.0e-12;
    if (!rotation.allFinite() ||
        (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() >
            kRotationTolerance ||
        std::abs(rotation.determinant() - 1.0) > kRotationTolerance) {
        Reject("carrier '" + carrier.carrier_name +
               "' requires a finite right-handed orthonormal non-spinning profile basis");
    }
}

void RequireIndependentWheelJoint(const VehicleDefinition& vehicle,
                                  const VehicleContactCarrierDeclaration& carrier,
                                  const VehicleContactInterfaceDeclaration& interface) {
    const std::string subject = "interface '" + interface.interface_name + "'";
    const std::string& joint_name = *interface.independent_wheel_revolute_joint_name;
    const auto joint = std::ranges::find(vehicle.revolute_joints, joint_name,
                                         &VehicleRevoluteJointDefinition::name);
    if (joint == vehicle.revolute_joints.end()) {
        Reject(subject + " names '" + joint_name + "', which is not a declared revolute joint");
    }
    if (interface.wheel_body_name == carrier.body_name) {
        Reject(subject + " names an independent-wheel joint but acts on its carrier body '" +
               carrier.body_name + "'");
    }
    if (joint->parent_frame_name != carrier.body_name ||
        joint->child_frame_name != interface.wheel_body_name) {
        Reject(subject + " joint '" + joint_name + "' must connect carrier body frame '" +
               carrier.body_name + "' directly to wheel body frame '" + interface.wheel_body_name +
               "', without an offset or rotated mounting frame");
    }
    const Eigen::Vector3d& axis = joint->axis_in_parent_frame;
    const double axis_norm = axis.norm();
    const Eigen::Vector3d expected_axis =
        -carrier.rotation_body_from_nonspinning_wheel_profile.col(1);
    constexpr double kAxisTolerance = 1.0e-12;
    if (!axis.allFinite() || !std::isfinite(axis_norm) || !(axis_norm > 0.0) ||
        (axis / axis_norm - expected_axis).norm() > kAxisTolerance) {
        Reject(subject + " joint '" + joint_name +
               "' must have unit axis -R_BP * e_y in its carrier body frame");
    }
}

}  // namespace

VehicleContactTopology BuildVehicleContactTopology(const VehicleDefinition& vehicle,
                                                   const VehicleContactDeclaration& declaration) {
    if (declaration.carriers.empty()) {
        Reject("at least one carrier with left and right interfaces is required");
    }
    VehicleContactTopology topology;
    topology.carriers.reserve(declaration.carriers.size());
    topology.interfaces.reserve(declaration.interfaces.size());
    std::unordered_map<std::string, std::size_t> carrier_indices;
    std::unordered_map<std::string, std::string> carrier_of_body;
    for (const auto& carrier : declaration.carriers) {
        const std::string subject = "carrier '" + carrier.carrier_name + "'";
        if (carrier.carrier_name.empty()) {
            Reject("a carrier name is empty");
        }
        if (!carrier_indices.emplace(carrier.carrier_name, topology.carriers.size()).second) {
            Reject(subject + " is declared more than once");
        }
        const auto& body = RequireBody(vehicle, carrier.body_name, subject);
        if (!body.moves_freely_in_world) {
            Reject(subject + " body '" + body.name + "' is not a free body");
        }
        const auto [previous, inserted] =
            carrier_of_body.emplace(carrier.body_name, carrier.carrier_name);
        if (!inserted) {
            Reject(subject + " and carrier '" + previous->second + "' both refer to body '" +
                   carrier.body_name + "'");
        }
        RequireProfileBasis(carrier);
        topology.carriers.push_back({carrier.carrier_name, carrier.body_name, 0.0,
                                     carrier.rotation_body_from_nonspinning_wheel_profile});
    }

    std::unordered_set<std::string> interface_names;
    std::unordered_set<std::string> independent_joint_names;
    std::vector<std::array<std::string, 2>> interface_by_carrier_side(declaration.carriers.size());
    for (const auto& interface : declaration.interfaces) {
        const std::string subject = "interface '" + interface.interface_name + "'";
        if (interface.interface_name.empty()) {
            Reject("an interface name is empty");
        }
        if (!interface_names.insert(interface.interface_name).second) {
            Reject(subject + " is declared more than once");
        }
        const auto carrier_index = carrier_indices.find(interface.carrier_name);
        if (carrier_index == carrier_indices.end()) {
            Reject(subject + " names undeclared carrier '" + interface.carrier_name + "'");
        }
        const auto& carrier = declaration.carriers[carrier_index->second];
        RequireBody(vehicle, interface.wheel_body_name, subject);
        if (interface.side != WheelSide::kLeft && interface.side != WheelSide::kRight) {
            Reject(subject + " must state either the left or right wheel side");
        }
        const std::size_t side_index = interface.side == WheelSide::kLeft ? 0 : 1;
        auto& previous_interface = interface_by_carrier_side[carrier_index->second][side_index];
        if (!previous_interface.empty()) {
            Reject(subject + " repeats the carrier side already supplied by interface '" +
                   previous_interface + "'");
        }
        previous_interface = interface.interface_name;

        std::optional<forces::IndependentWheelRevoluteJointDefinition> joint;
        if (interface.independent_wheel_revolute_joint_name.has_value()) {
            RequireIndependentWheelJoint(vehicle, carrier, interface);
            const auto& joint_name = *interface.independent_wheel_revolute_joint_name;
            if (!independent_joint_names.insert(joint_name).second) {
                Reject(subject + " repeats independent-wheel joint '" + joint_name + "'");
            }
            joint = forces::IndependentWheelRevoluteJointDefinition{joint_name};
        } else if (interface.wheel_body_name != carrier.body_name) {
            Reject(subject +
                   " has no independent-wheel joint, so its wheel body must be carrier '" +
                   carrier.carrier_name + "' body '" + carrier.body_name + "'");
        }
        topology.interfaces.push_back({interface.interface_name, interface.carrier_name,
                                       interface.wheel_body_name, interface.side,
                                       std::move(joint)});
    }
    for (std::size_t carrier_index = 0; carrier_index < declaration.carriers.size();
         ++carrier_index) {
        const auto& sides = interface_by_carrier_side[carrier_index];
        if (sides[0].empty() || sides[1].empty()) {
            const std::string existing = sides[0].empty() ? sides[1] : sides[0];
            Reject("carrier '" + declaration.carriers[carrier_index].carrier_name +
                   "' requires one left and one right interface; missing " +
                   (sides[0].empty() ? "left" : "right") + " interface" +
                   (existing.empty() ? "" : " beside interface '" + existing + "'"));
        }
    }
    return topology;
}

}  // namespace orvd::configuration
