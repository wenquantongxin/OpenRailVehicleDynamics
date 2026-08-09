#include "orvd/configuration/load_vehicle_definition.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "strict_json.h"
#include "vehicle_definition_invariants.h"
#include "vehicle_definition_inertia.h"

namespace orvd::configuration {
namespace {

using strict_json::ElementPath;
using strict_json::Json;
using strict_json::ParseStrictJson;
using strict_json::ReadWholeFile;
using strict_json::RequireArray;
using strict_json::RequireBool;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireFiniteVector3;
using strict_json::RequireIdentifier;
using strict_json::RequireObject;
using strict_json::RequireString;
using strict_json::ThrowExpected;

Eigen::Vector3d RequireVector(const Json& value, const std::string& path) {
    return RequireFiniteVector3(value, path);
}

// The common aligned form remains compact. The matrix form carries the source
// marker orientation row by row, making the direction (body <- frame) explicit
// without converting a qualified source matrix through Euler angles.
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
    if (form == "matrix_rows") {
        RequireExactKeys(value, path, {"form", "rows"});
        const std::string rows_path = path + ".rows";
        const Json& rows = value.at("rows");
        RequireArray(rows, rows_path);
        if (rows.size() != 3) {
            ThrowExpected(rows_path, "exactly three finite row vectors");
        }
        Eigen::Matrix3d rotation;
        for (std::size_t row = 0; row < 3; ++row) {
            rotation.row(static_cast<Eigen::Index>(row)) =
                RequireVector(rows[row], ElementPath(rows_path, row));
        }
        constexpr double kRotationTolerance = 1.0e-12;
        const Eigen::Matrix3d gram = rotation.transpose() * rotation;
        if ((gram - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() >
                kRotationTolerance ||
            std::abs(rotation.determinant() - 1.0) > kRotationTolerance) {
            ThrowExpected(path,
                          "a finite right-handed orthonormal rotation matrix");
        }
        return rotation;
    }
    ThrowExpected(path + ".form", "'aligned_with_body' or 'matrix_rows'");
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
    internal::ThrowIfSingularFreeBodyCenterOfMassInertia(body, path);
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

VehicleBallRpyJointDefinition ParseBallRpyJoint(const Json& value,
                                                const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "parent_frame_name", "child_frame_name",
                      "default_roll_pitch_yaw_angles_radians"});
    VehicleBallRpyJointDefinition joint;
    joint.name = RequireString(value.at("name"), path + ".name");
    joint.parent_frame_name =
        RequireString(value.at("parent_frame_name"), path + ".parent_frame_name");
    joint.child_frame_name =
        RequireString(value.at("child_frame_name"), path + ".child_frame_name");
    joint.default_roll_pitch_yaw_angles_radians = RequireVector(
        value.at("default_roll_pitch_yaw_angles_radians"),
        path + ".default_roll_pitch_yaw_angles_radians");
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

// Every force element states both ends and which one is the reference. The
// three fields are read together so a reader sees the whole endpoint contract
// at one place.
struct ForceElementEnds {
    std::string reference_frame_name;
    std::string opposite_frame_name;
};

ForceElementEnds ParseEnds(const Json& value, const std::string& path) {
    return ForceElementEnds{
        RequireString(value.at("reference_frame_name"),
                      path + ".reference_frame_name"),
        RequireString(value.at("opposite_frame_name"),
                      path + ".opposite_frame_name")};
}

std::string ParseAxis(const Json& value, const std::string& path) {
    const std::string axis = RequireString(value, path);
    if (axis != "longitudinal" && axis != "lateral" && axis != "vertical") {
        ThrowExpected(path, "'longitudinal', 'lateral' or 'vertical'");
    }
    return axis;
}

VehicleTranslationalSpringDamperDefinition ParseTranslationalSpringDamper(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "reference_frame_name", "opposite_frame_name",
                      "stiffness_newtons_per_meter",
                      "damping_newton_seconds_per_meter"});
    VehicleTranslationalSpringDamperDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    element.stiffness_newtons_per_meter =
        RequireVector(value.at("stiffness_newtons_per_meter"),
                      path + ".stiffness_newtons_per_meter");
    element.damping_newton_seconds_per_meter =
        RequireVector(value.at("damping_newton_seconds_per_meter"),
                      path + ".damping_newton_seconds_per_meter");
    return element;
}

VehicleRollSpringDamperCoupleDefinition ParseRollSpringDamperCouple(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "reference_frame_name", "opposite_frame_name",
                      "stiffness_newton_meters_per_radian",
                      "damping_newton_meter_seconds_per_radian"});
    VehicleRollSpringDamperCoupleDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    element.stiffness_newton_meters_per_radian =
        RequireFiniteNumber(value.at("stiffness_newton_meters_per_radian"),
                            path + ".stiffness_newton_meters_per_radian");
    element.damping_newton_meter_seconds_per_radian = RequireFiniteNumber(
        value.at("damping_newton_meter_seconds_per_radian"),
        path + ".damping_newton_meter_seconds_per_radian");
    return element;
}

VehicleSeriesSpringViscousDamperDefinition ParseSeriesSpringViscousDamper(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "reference_frame_name", "opposite_frame_name",
                      "axis", "series_stiffness_newtons_per_meter",
                      "series_damping_newton_seconds_per_meter"});
    VehicleSeriesSpringViscousDamperDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    element.axis = ParseAxis(value.at("axis"), path + ".axis");
    element.series_stiffness_newtons_per_meter =
        RequireFiniteNumber(value.at("series_stiffness_newtons_per_meter"),
                            path + ".series_stiffness_newtons_per_meter");
    element.series_damping_newton_seconds_per_meter = RequireFiniteNumber(
        value.at("series_damping_newton_seconds_per_meter"),
        path + ".series_damping_newton_seconds_per_meter");
    return element;
}

VehicleSaturatedPiecewiseLinearDamperDefinition
ParseSaturatedPiecewiseLinearDamper(const Json& value,
                                    const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "reference_frame_name", "opposite_frame_name",
                      "axis", "curve"});
    VehicleSaturatedPiecewiseLinearDamperDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    element.axis = ParseAxis(value.at("axis"), path + ".axis");
    const Json& curve = value.at("curve");
    const std::string curve_path = path + ".curve";
    RequireArray(curve, curve_path);
    element.curve.reserve(curve.size());
    for (std::size_t point = 0; point < curve.size(); ++point) {
        const std::string point_path = ElementPath(curve_path, point);
        RequireExactKeys(curve[point], point_path,
                         {"relative_velocity_meters_per_second",
                          "force_newtons"});
        element.curve.push_back(
            VehicleSaturatedPiecewiseLinearDamperPointDefinition{
                RequireFiniteNumber(
                    curve[point].at("relative_velocity_meters_per_second"),
                    point_path + ".relative_velocity_meters_per_second"),
                RequireFiniteNumber(curve[point].at("force_newtons"),
                                    point_path + ".force_newtons")});
    }
    return element;
}

VehicleHalfAngleMidpointRollPitchYawBushingDefinition
ParseHalfAngleMidpointRollPitchYawBushing(const Json& value,
                                          const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "frame_a_name", "frame_c_name",
         "rotational_stiffness_newton_meters_per_radian",
         "rotational_damping_newton_meter_seconds_per_radian",
         "translational_stiffness_newtons_per_meter",
         "translational_damping_newton_seconds_per_meter"});
    VehicleHalfAngleMidpointRollPitchYawBushingDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    element.frame_a_name =
        RequireString(value.at("frame_a_name"), path + ".frame_a_name");
    element.frame_c_name =
        RequireString(value.at("frame_c_name"), path + ".frame_c_name");
    element.rotational_stiffness_newton_meters_per_radian = RequireVector(
        value.at("rotational_stiffness_newton_meters_per_radian"),
        path + ".rotational_stiffness_newton_meters_per_radian");
    element.rotational_damping_newton_meter_seconds_per_radian = RequireVector(
        value.at("rotational_damping_newton_meter_seconds_per_radian"),
        path + ".rotational_damping_newton_meter_seconds_per_radian");
    element.translational_stiffness_newtons_per_meter = RequireVector(
        value.at("translational_stiffness_newtons_per_meter"),
        path + ".translational_stiffness_newtons_per_meter");
    element.translational_damping_newton_seconds_per_meter = RequireVector(
        value.at("translational_damping_newton_seconds_per_meter"),
        path + ".translational_damping_newton_seconds_per_meter");
    return element;
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

VehicleFreeBodyStationOffsetDefinition ParseFreeBodyStationOffset(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"body_name", "station_offset_meters"});
    VehicleFreeBodyStationOffsetDefinition offset;
    offset.body_name = RequireString(value.at("body_name"), path + ".body_name");
    if (offset.body_name.empty()) {
        ThrowExpected(path + ".body_name", "a non-empty rigid-body name");
    }
    offset.station_offset_meters = RequireFiniteNumber(
        value.at("station_offset_meters"), path + ".station_offset_meters");
    return offset;
}

VehicleMechanicalTrackStationLayoutDefinition ParseMechanicalTrackStationLayout(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"reference_body_name", "free_body_station_offsets",
                      "wheel_contact_carrier_body_names"});
    VehicleMechanicalTrackStationLayoutDefinition layout;
    layout.reference_body_name = RequireString(
        value.at("reference_body_name"), path + ".reference_body_name");
    if (layout.reference_body_name.empty()) {
        ThrowExpected(path + ".reference_body_name",
                      "a non-empty rigid-body name");
    }

    const std::string offsets_path = path + ".free_body_station_offsets";
    const Json& offsets = value.at("free_body_station_offsets");
    RequireArray(offsets, offsets_path);
    layout.free_body_station_offsets.reserve(offsets.size());
    for (std::size_t index = 0; index < offsets.size(); ++index) {
        layout.free_body_station_offsets.push_back(ParseFreeBodyStationOffset(
            offsets[index], ElementPath(offsets_path, index)));
    }

    const std::string carriers_path =
        path + ".wheel_contact_carrier_body_names";
    const Json& carriers = value.at("wheel_contact_carrier_body_names");
    RequireArray(carriers, carriers_path);
    layout.wheel_contact_carrier_body_names.reserve(carriers.size());
    for (std::size_t index = 0; index < carriers.size(); ++index) {
        const std::string element_path = ElementPath(carriers_path, index);
        std::string name = RequireString(carriers[index], element_path);
        if (name.empty()) {
            ThrowExpected(element_path, "a non-empty rigid-body name");
        }
        layout.wheel_contact_carrier_body_names.push_back(std::move(name));
    }
    return layout;
}

}  // namespace

VehicleDefinition LoadVehicleDefinitionFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const std::string document =
        ReadWholeFile(configuration_path, "vehicle definition");
    const Json root = ParseStrictJson(document);
    RequireExactKeys(root, "$",
                     {"vehicle_name",
                      "mechanical_definition_identifier",
                      "mechanical_track_station_layout", "rigid_bodies",
                      "fixed_frames", "revolute_joints", "ball_rpy_joints",
                      "weld_joints",
                      "translational_spring_dampers",
                      "roll_spring_damper_couples",
                      "series_spring_viscous_dampers",
                      "saturated_piecewise_linear_dampers",
                      "half_angle_midpoint_roll_pitch_yaw_bushings"});

    VehicleDefinition vehicle;
    vehicle.vehicle_name =
        RequireString(root.at("vehicle_name"), "$.vehicle_name");
    if (vehicle.vehicle_name.empty()) {
        ThrowExpected("$.vehicle_name", "a non-empty string");
    }
    vehicle.mechanical_definition_identifier =
        RequireIdentifier(root.at("mechanical_definition_identifier"),
                          "$.mechanical_definition_identifier",
                          "mechanical definition identifier");
    vehicle.mechanical_track_station_layout =
        ParseMechanicalTrackStationLayout(
            root.at("mechanical_track_station_layout"),
            "$.mechanical_track_station_layout");
    vehicle.rigid_bodies = ParseArray<VehicleRigidBodyDefinition>(
        root, "rigid_bodies", ParseRigidBody);
    vehicle.fixed_frames = ParseArray<VehicleFixedFrameDefinition>(
        root, "fixed_frames", ParseFixedFrame);
    vehicle.revolute_joints = ParseArray<VehicleRevoluteJointDefinition>(
        root, "revolute_joints", ParseRevoluteJoint);
    vehicle.ball_rpy_joints = ParseArray<VehicleBallRpyJointDefinition>(
        root, "ball_rpy_joints", ParseBallRpyJoint);
    vehicle.weld_joints =
        ParseArray<VehicleWeldJointDefinition>(root, "weld_joints",
                                               ParseWeldJoint);
    vehicle.translational_spring_dampers =
        ParseArray<VehicleTranslationalSpringDamperDefinition>(
            root, "translational_spring_dampers",
            ParseTranslationalSpringDamper);
    vehicle.roll_spring_damper_couples =
        ParseArray<VehicleRollSpringDamperCoupleDefinition>(
            root, "roll_spring_damper_couples", ParseRollSpringDamperCouple);
    vehicle.series_spring_viscous_dampers =
        ParseArray<VehicleSeriesSpringViscousDamperDefinition>(
            root, "series_spring_viscous_dampers",
            ParseSeriesSpringViscousDamper);
    vehicle.saturated_piecewise_linear_dampers =
        ParseArray<VehicleSaturatedPiecewiseLinearDamperDefinition>(
            root, "saturated_piecewise_linear_dampers",
            ParseSaturatedPiecewiseLinearDamper);
    vehicle.half_angle_midpoint_roll_pitch_yaw_bushings =
        ParseArray<VehicleHalfAngleMidpointRollPitchYawBushingDefinition>(
            root, "half_angle_midpoint_roll_pitch_yaw_bushings",
            ParseHalfAngleMidpointRollPitchYawBushing);

    // Bodies and frames share one namespace, so one set answers both "is this a
    // body?" and "is this a frame a joint may attach to?".
    std::unordered_set<std::string> body_names;
    std::unordered_set<std::string> frame_names;
    for (const VehicleRigidBodyDefinition& body : vehicle.rigid_bodies) {
        body_names.insert(body.name);
        frame_names.insert(body.name);
    }

    internal::RequireVehicleMechanicalTrackStationLayoutInvariants(vehicle,
                                                                    "$");
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
    for (std::size_t index = 0; index < vehicle.ball_rpy_joints.size();
         ++index) {
        const VehicleBallRpyJointDefinition& joint =
            vehicle.ball_rpy_joints[index];
        const std::string path = ElementPath("$.ball_rpy_joints", index);
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

    const auto require_force_ends = [&frame_names](
                                        const std::string& array_key,
                                        std::size_t index,
                                        const std::string& reference,
                                        const std::string& opposite) {
        const std::string path = ElementPath("$." + array_key, index);
        RequireDeclared(frame_names, reference, path + ".reference_frame_name");
        RequireDeclared(frame_names, opposite, path + ".opposite_frame_name");
    };
    std::unordered_map<std::string, std::string> force_name_paths;
    const auto require_force_name = [&force_name_paths](
                                        const std::string& array_key,
                                        std::size_t index,
                                        const std::string& name) {
        const std::string path =
            ElementPath("$." + array_key, index) + ".name";
        if (name.empty()) {
            ThrowExpected(path, "a non-empty force-element name");
        }
        const auto [first, inserted] = force_name_paths.emplace(name, path);
        if (!inserted) {
            throw std::invalid_argument(
                path + " repeats force-element name '" + name +
                "', first declared at " + first->second);
        }
    };
    for (std::size_t index = 0;
         index < vehicle.translational_spring_dampers.size(); ++index) {
        const auto& element = vehicle.translational_spring_dampers[index];
        require_force_name("translational_spring_dampers", index,
                           element.name);
        require_force_ends("translational_spring_dampers", index,
                           element.reference_frame_name,
                           element.opposite_frame_name);
    }
    for (std::size_t index = 0;
         index < vehicle.roll_spring_damper_couples.size(); ++index) {
        const auto& element = vehicle.roll_spring_damper_couples[index];
        require_force_name("roll_spring_damper_couples", index,
                           element.name);
        require_force_ends("roll_spring_damper_couples", index,
                           element.reference_frame_name,
                           element.opposite_frame_name);
    }
    for (std::size_t index = 0;
         index < vehicle.series_spring_viscous_dampers.size(); ++index) {
        const auto& element = vehicle.series_spring_viscous_dampers[index];
        require_force_name("series_spring_viscous_dampers", index,
                           element.name);
        require_force_ends("series_spring_viscous_dampers", index,
                           element.reference_frame_name,
                           element.opposite_frame_name);
    }
    for (std::size_t index = 0;
         index < vehicle.saturated_piecewise_linear_dampers.size(); ++index) {
        const auto& element = vehicle.saturated_piecewise_linear_dampers[index];
        require_force_name("saturated_piecewise_linear_dampers", index,
                           element.name);
        require_force_ends("saturated_piecewise_linear_dampers", index,
                           element.reference_frame_name,
                           element.opposite_frame_name);
    }
    for (std::size_t index = 0;
         index < vehicle.half_angle_midpoint_roll_pitch_yaw_bushings.size();
         ++index) {
        const auto& element =
            vehicle.half_angle_midpoint_roll_pitch_yaw_bushings[index];
        require_force_name("half_angle_midpoint_roll_pitch_yaw_bushings",
                           index, element.name);
        const std::string path = ElementPath(
            "$.half_angle_midpoint_roll_pitch_yaw_bushings", index);
        RequireDeclared(frame_names, element.frame_a_name,
                        path + ".frame_a_name");
        RequireDeclared(frame_names, element.frame_c_name,
                        path + ".frame_c_name");
    }
    return vehicle;
}

}  // namespace orvd::configuration
