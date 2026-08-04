#include "orvd/configuration/load_vehicle_definition.h"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "strict_json.h"

namespace orvd::configuration {
namespace {

using strict_json::Json;
using strict_json::ParseStrictJson;
using strict_json::ReadWholeFile;
using strict_json::RequireArray;
using strict_json::RequireBool;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireObject;
using strict_json::RequireString;
using strict_json::ThrowExpected;

std::string ElementPath(const std::string& array_path, std::size_t index) {
    return array_path + "[" + std::to_string(index) + "]";
}

Eigen::Vector3d RequireVector(const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"x", "y", "z"});
    return Eigen::Vector3d(
        RequireFiniteNumber(value.at("x"), path + ".x"),
        RequireFiniteNumber(value.at("y"), path + ".y"),
        RequireFiniteNumber(value.at("z"), path + ".z"));
}

// The rotation is a tagged form rather than nine bare numbers. Every named frame
// of the first vehicle is aligned with its body, so writing out nine constants
// would be six hundred lines of noise in which a transposed pair would be
// invisible. A second form is added when a record needs one, not before.
Eigen::Matrix3d RequireRotation(const Json& value, const std::string& path) {
    RequireObject(value, path);
    if (!value.contains("form")) {
        throw std::invalid_argument(path + ".form is required");
    }
    const std::string form = RequireString(value.at("form"), path + ".form");
    if (form == "aligned_with_body") {
        RequireExactKeys(value, path, {"form"});
        return Eigen::Matrix3d::Identity();
    }
    ThrowExpected(path + ".form", "'aligned_with_body'");
}

VehicleRigidBodyDefinition ParseRigidBody(const Json& value,
                                          const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "moves_freely_in_world", "mass_kilograms",
         "center_of_mass_in_body_frame_meters",
         "inertia_moments_about_center_of_mass_kilogram_square_meters",
         "inertia_products_about_center_of_mass_kilogram_square_meters"});

    VehicleRigidBodyDefinition body;
    body.name = RequireString(value.at("name"), path + ".name");
    body.moves_freely_in_world = RequireBool(value.at("moves_freely_in_world"),
                                             path + ".moves_freely_in_world");
    body.mass_kilograms =
        RequireFiniteNumber(value.at("mass_kilograms"), path + ".mass_kilograms");
    if (!(body.mass_kilograms > 0.0)) {
        ThrowExpected(path + ".mass_kilograms", "a positive number of kilograms");
    }
    body.center_of_mass_in_body_frame_meters =
        RequireVector(value.at("center_of_mass_in_body_frame_meters"),
                      path + ".center_of_mass_in_body_frame_meters");
    body.inertia_moments_about_center_of_mass_kilogram_square_meters =
        RequireVector(
            value.at(
                "inertia_moments_about_center_of_mass_kilogram_square_meters"),
            path +
                ".inertia_moments_about_center_of_mass_kilogram_square_meters");
    body.inertia_products_about_center_of_mass_kilogram_square_meters =
        RequireVector(
            value.at(
                "inertia_products_about_center_of_mass_kilogram_square_meters"),
            path +
                ".inertia_products_about_center_of_mass_kilogram_square_meters");
    return body;
}

VehicleFixedFrameDefinition ParseFixedFrame(const Json& value,
                                            const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "body_name", "position_in_body_frame_meters",
                      "rotation_in_body_frame"});
    VehicleFixedFrameDefinition frame;
    frame.name = RequireString(value.at("name"), path + ".name");
    frame.body_name = RequireString(value.at("body_name"), path + ".body_name");
    frame.position_in_body_frame_meters =
        RequireVector(value.at("position_in_body_frame_meters"),
                      path + ".position_in_body_frame_meters");
    frame.rotation_in_body_frame = RequireRotation(
        value.at("rotation_in_body_frame"), path + ".rotation_in_body_frame");
    return frame;
}

VehicleRevoluteJointDefinition ParseRevoluteJoint(const Json& value,
                                                  const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "parent_frame_name", "child_frame_name",
                      "axis_in_parent_frame",
                      "damping_newton_metre_seconds_per_radian"});
    VehicleRevoluteJointDefinition joint;
    joint.name = RequireString(value.at("name"), path + ".name");
    joint.parent_frame_name =
        RequireString(value.at("parent_frame_name"), path + ".parent_frame_name");
    joint.child_frame_name =
        RequireString(value.at("child_frame_name"), path + ".child_frame_name");
    joint.axis_in_parent_frame = RequireVector(value.at("axis_in_parent_frame"),
                                               path + ".axis_in_parent_frame");
    joint.damping_newton_metre_seconds_per_radian = RequireFiniteNumber(
        value.at("damping_newton_metre_seconds_per_radian"),
        path + ".damping_newton_metre_seconds_per_radian");
    return joint;
}

VehicleWeldJointDefinition ParseWeldJoint(const Json& value,
                                          const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "parent_frame_name", "child_frame_name"});
    VehicleWeldJointDefinition joint;
    joint.name = RequireString(value.at("name"), path + ".name");
    joint.parent_frame_name =
        RequireString(value.at("parent_frame_name"), path + ".parent_frame_name");
    joint.child_frame_name =
        RequireString(value.at("child_frame_name"), path + ".child_frame_name");
    return joint;
}

template <typename Element, typename Parse>
std::vector<Element> ParseArray(const Json& root, const std::string& key,
                                Parse parse) {
    const std::string path = "$." + key;
    const Json& array = root.at(key);
    RequireArray(array, path);
    std::vector<Element> elements;
    elements.reserve(array.size());
    for (std::size_t index = 0; index < array.size(); ++index) {
        elements.push_back(parse(array[index], ElementPath(path, index)));
    }
    return elements;
}

// A name a joint or a frame refers to must have been declared. The assembler
// would also fail to resolve it, but only this layer can say which key of which
// array element is the one that names something absent.
void RequireDeclared(const std::unordered_set<std::string>& declared,
                     const std::string& name, const std::string& path) {
    if (!declared.contains(name)) {
        throw std::invalid_argument(path + " names '" + name +
                                    "', which this description never declares");
    }
}

}  // namespace

VehicleDefinition LoadVehicleDefinitionFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const std::string document =
        ReadWholeFile(configuration_path, "vehicle definition");
    const Json root = ParseStrictJson(document);
    RequireExactKeys(root, "$",
                     {"schema_version", "vehicle_name", "rigid_bodies",
                      "fixed_frames", "revolute_joints", "weld_joints"});

    if (!root.at("schema_version").is_number_integer() ||
        root.at("schema_version") != 1) {
        ThrowExpected("$.schema_version", "the integer 1");
    }

    VehicleDefinition vehicle;
    vehicle.vehicle_name =
        RequireString(root.at("vehicle_name"), "$.vehicle_name");
    if (vehicle.vehicle_name.empty()) {
        ThrowExpected("$.vehicle_name", "a non-empty string");
    }
    vehicle.rigid_bodies = ParseArray<VehicleRigidBodyDefinition>(
        root, "rigid_bodies", ParseRigidBody);
    vehicle.fixed_frames = ParseArray<VehicleFixedFrameDefinition>(
        root, "fixed_frames", ParseFixedFrame);
    vehicle.revolute_joints = ParseArray<VehicleRevoluteJointDefinition>(
        root, "revolute_joints", ParseRevoluteJoint);
    vehicle.weld_joints =
        ParseArray<VehicleWeldJointDefinition>(root, "weld_joints",
                                               ParseWeldJoint);

    // Bodies and frames share one namespace, so one set answers both "is this a
    // body?" and "is this a frame a joint may attach to?".
    std::unordered_set<std::string> body_names;
    std::unordered_set<std::string> frame_names;
    for (const VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        body_names.insert(body.name);
        frame_names.insert(body.name);
    }
    for (std::size_t index = 0; index < vehicle.fixed_frames.size(); ++index) {
        const VehicleFixedFrameDefinition& frame = vehicle.fixed_frames[index];
        RequireDeclared(body_names, frame.body_name,
                        ElementPath("$.fixed_frames", index) + ".body_name");
        frame_names.insert(frame.name);
    }
    for (std::size_t index = 0; index < vehicle.revolute_joints.size();
         ++index) {
        const VehicleRevoluteJointDefinition& joint =
            vehicle.revolute_joints[index];
        const std::string path = ElementPath("$.revolute_joints", index);
        RequireDeclared(frame_names, joint.parent_frame_name,
                        path + ".parent_frame_name");
        RequireDeclared(frame_names, joint.child_frame_name,
                        path + ".child_frame_name");
    }
    for (std::size_t index = 0; index < vehicle.weld_joints.size(); ++index) {
        const VehicleWeldJointDefinition& joint = vehicle.weld_joints[index];
        const std::string path = ElementPath("$.weld_joints", index);
        RequireDeclared(frame_names, joint.parent_frame_name,
                        path + ".parent_frame_name");
        RequireDeclared(frame_names, joint.child_frame_name,
                        path + ".child_frame_name");
    }
    return vehicle;
}

}  // namespace orvd::configuration
