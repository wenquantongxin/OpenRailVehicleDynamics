#include "vehicle_definition_validation.h"

#include <Eigen/LU>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "configuration_identifier_validation.h"
#include "point_to_point_spring_definition_validation.h"
#include "vehicle_definition_inertia.h"
#include "vehicle_definition_invariants.h"

namespace orvd::configuration::internal {
namespace {

std::string ElementPath(const std::string& path, std::size_t index) {
    return path + "[" + std::to_string(index) + "]";
}

void RequireDeclared(const std::unordered_set<std::string>& declared, const std::string& name,
                     const std::string& path) {
    if (!declared.contains(name)) {
        throw std::invalid_argument(path + " names '" + name +
                                    "', which this description never declares");
    }
}

void RequireAxis(const std::string& axis, const std::string& path) {
    if (axis != "longitudinal" && axis != "lateral" && axis != "vertical") {
        throw std::invalid_argument(path + " must be 'longitudinal', 'lateral' or 'vertical'");
    }
}

void RequirePositive(double value, const std::string& path) {
    if (!(value > 0.0)) {
        throw std::invalid_argument(path + " must be a strictly positive number");
    }
}

void RequireNonNegative(const Eigen::Vector3d& values, const std::string& path) {
    if ((values.array() < 0.0).any()) {
        throw std::invalid_argument(path + " must be three finite non-negative coefficients");
    }
}

}  // namespace

void RequireVehicleDefinitionDeclarationInvariants(const VehicleDefinition& vehicle,
                                                   const std::string& path) {
    if (vehicle.vehicle_name.empty()) {
        throw std::invalid_argument(path + ".vehicle_name must be a non-empty string");
    }
    RequireConfigurationIdentifier(vehicle.mechanical_definition_identifier,
                                   path + ".mechanical_definition_identifier",
                                   "mechanical definition identifier");

    const auto& layout = vehicle.mechanical_track_station_layout;
    const std::string layout_path = path + ".mechanical_track_station_layout";
    if (layout.reference_body_name.empty()) {
        throw std::invalid_argument(layout_path +
                                    ".reference_body_name must be a non-empty rigid-body name");
    }
    for (std::size_t index = 0; index < layout.free_body_station_offsets.size(); ++index) {
        if (layout.free_body_station_offsets[index].body_name.empty()) {
            throw std::invalid_argument(
                ElementPath(layout_path + ".free_body_station_offsets", index) +
                ".body_name must be a non-empty rigid-body name");
        }
    }
    for (std::size_t index = 0; index < layout.wheel_pair_station_reference_body_names.size();
         ++index) {
        if (layout.wheel_pair_station_reference_body_names[index].empty()) {
            throw std::invalid_argument(
                ElementPath(layout_path + ".wheel_pair_station_reference_body_names", index) +
                " must be a non-empty rigid-body name");
        }
    }
    RequireVehicleMechanicalTrackStationLayoutInvariants(vehicle, path);

    // Each body also declares its body frame.
    std::unordered_set<std::string> body_names;
    std::unordered_set<std::string> frame_names;
    for (const auto& body : vehicle.rigid_bodies) {
        body_names.insert(body.name);
        frame_names.insert(body.name);
    }
    for (std::size_t index = 0; index < vehicle.fixed_frames.size(); ++index) {
        const auto& frame = vehicle.fixed_frames[index];
        RequireDeclared(body_names, frame.body_name,
                        ElementPath(path + ".fixed_frames", index) + ".body_name");
        frame_names.insert(frame.name);
    }
    const auto require_joint_ends = [&](const char* key, const auto& joints) {
        for (std::size_t index = 0; index < joints.size(); ++index) {
            const auto& joint = joints[index];
            const std::string joint_path = ElementPath(path + "." + key, index);
            RequireDeclared(frame_names, joint.parent_frame_name,
                            joint_path + ".parent_frame_name");
            RequireDeclared(frame_names, joint.child_frame_name, joint_path + ".child_frame_name");
        }
    };
    require_joint_ends("revolute_joints", vehicle.revolute_joints);
    require_joint_ends("ball_rpy_joints", vehicle.ball_rpy_joints);
    require_joint_ends("weld_joints", vehicle.weld_joints);

    std::unordered_map<std::string, std::string> force_name_paths;
    const auto require_force_declarations = [&](const char* key, const auto& elements) {
        for (std::size_t index = 0; index < elements.size(); ++index) {
            const auto& element = elements[index];
            const std::string element_path = ElementPath(path + "." + key, index);
            const std::string name_path = element_path + ".name";
            if (element.name.empty()) {
                throw std::invalid_argument(name_path + " must be a non-empty force-element name");
            }
            const auto [first, inserted] = force_name_paths.emplace(element.name, name_path);
            if (!inserted) {
                throw std::invalid_argument(name_path + " repeats force-element name '" +
                                            element.name + "', first declared at " + first->second);
            }
            RequireDeclared(frame_names, element.reference_frame_name,
                            element_path + ".reference_frame_name");
            RequireDeclared(frame_names, element.opposite_frame_name,
                            element_path + ".opposite_frame_name");
        }
    };
    require_force_declarations("translational_spring_dampers",
                               vehicle.translational_spring_dampers);
    require_force_declarations("roll_spring_damper_couples", vehicle.roll_spring_damper_couples);
    require_force_declarations("series_spring_viscous_dampers",
                               vehicle.series_spring_viscous_dampers);
    require_force_declarations("point_to_point_series_spring_bilinear_dampers",
                               vehicle.point_to_point_series_spring_bilinear_dampers);
    require_force_declarations("saturated_piecewise_linear_dampers",
                               vehicle.saturated_piecewise_linear_dampers);
    require_force_declarations("reference_frame_linear_bushings",
                               vehicle.reference_frame_linear_bushings);
    require_force_declarations("point_to_point_linear_spring_dampers",
                               vehicle.point_to_point_linear_spring_dampers);
    require_force_declarations("point_to_point_hysteretic_springs",
                               vehicle.point_to_point_hysteretic_springs);
}

void RequireVehicleDefinitionInvariants(const VehicleDefinition& vehicle, const std::string& path) {
    for (std::size_t index = 0; index < vehicle.rigid_bodies.size(); ++index) {
        const auto& body = vehicle.rigid_bodies[index];
        const std::string body_path = ElementPath(path + ".rigid_bodies", index);
        if (!(body.mass_kilograms > 0.0)) {
            throw std::invalid_argument(body_path +
                                        ".mass_kilograms must be a positive number of kilograms");
        }
        ThrowIfSingularFreeBodyCenterOfMassInertia(body, body_path);
    }
    for (std::size_t index = 0; index < vehicle.fixed_frames.size(); ++index) {
        const auto& rotation = vehicle.fixed_frames[index].rotation_in_body_frame;
        constexpr double kRotationTolerance = 1.0e-12;
        const Eigen::Matrix3d gram = rotation.transpose() * rotation;
        if ((gram - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > kRotationTolerance ||
            std::abs(rotation.determinant() - 1.0) > kRotationTolerance) {
            throw std::invalid_argument(ElementPath(path + ".fixed_frames", index) +
                                        ".rotation_in_body_frame must be a finite right-handed "
                                        "orthonormal rotation matrix");
        }
    }
    for (std::size_t index = 0; index < vehicle.series_spring_viscous_dampers.size(); ++index) {
        RequireAxis(vehicle.series_spring_viscous_dampers[index].axis,
                    ElementPath(path + ".series_spring_viscous_dampers", index) + ".axis");
    }
    for (std::size_t index = 0;
         index < vehicle.point_to_point_series_spring_bilinear_dampers.size(); ++index) {
        const auto& element = vehicle.point_to_point_series_spring_bilinear_dampers[index];
        const std::string element_path =
            ElementPath(path + ".point_to_point_series_spring_bilinear_dampers", index);
        RequirePositive(element.series_stiffness_newtons_per_meter,
                        element_path + ".series_stiffness_newtons_per_meter");
        RequirePositive(element.damping_below_knee_newton_seconds_per_meter,
                        element_path + ".damping_below_knee_newton_seconds_per_meter");
        RequirePositive(element.knee_force_newtons, element_path + ".knee_force_newtons");
        RequirePositive(element.damping_above_knee_newton_seconds_per_meter,
                        element_path + ".damping_above_knee_newton_seconds_per_meter");
    }
    for (std::size_t index = 0; index < vehicle.saturated_piecewise_linear_dampers.size();
         ++index) {
        RequireAxis(vehicle.saturated_piecewise_linear_dampers[index].axis,
                    ElementPath(path + ".saturated_piecewise_linear_dampers", index) + ".axis");
    }
    for (std::size_t index = 0; index < vehicle.reference_frame_linear_bushings.size(); ++index) {
        const auto& element = vehicle.reference_frame_linear_bushings[index];
        const std::string element_path =
            ElementPath(path + ".reference_frame_linear_bushings", index);
        RequireNonNegative(element.translational_stiffness_newtons_per_meter,
                           element_path + ".translational_stiffness_newtons_per_meter");
        RequireNonNegative(element.translational_damping_newton_seconds_per_meter,
                           element_path + ".translational_damping_newton_seconds_per_meter");
        RequireNonNegative(element.rotational_damping_newton_meter_seconds_per_radian,
                           element_path + ".rotational_damping_newton_meter_seconds_per_radian");
    }
    for (std::size_t index = 0; index < vehicle.point_to_point_linear_spring_dampers.size();
         ++index) {
        RequirePointToPointLinearSpringDamperInvariants(
            vehicle.point_to_point_linear_spring_dampers[index],
            ElementPath(path + ".point_to_point_linear_spring_dampers", index));
    }
    for (std::size_t index = 0; index < vehicle.point_to_point_hysteretic_springs.size(); ++index) {
        RequirePointToPointHystereticSpringInvariants(
            vehicle.point_to_point_hysteretic_springs[index],
            ElementPath(path + ".point_to_point_hysteretic_springs", index));
    }
    RequireVehicleDefinitionDeclarationInvariants(vehicle, path);
}

}  // namespace orvd::configuration::internal
