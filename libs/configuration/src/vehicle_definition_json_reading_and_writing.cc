#include "vehicle_definition_json_reading_and_writing.h"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "point_to_point_spring_definitions_json.h"
#include "strict_json.h"

namespace orvd::configuration::internal {
namespace {

using strict_json::ElementPath;
using strict_json::Json;
using strict_json::RequireArray;
using strict_json::RequireBool;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireFiniteVector3;
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
        return rotation;
    }
    ThrowExpected(path + ".form", "'aligned_with_body' or 'matrix_rows'");
}

VehicleRigidBodyDefinition ParseRigidBody(const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "moves_freely_in_world", "mass_kilograms", "center_of_mass_in_body_frame_meters",
         "inertia_moments_about_center_of_mass_kilogram_square_meters",
         "inertia_products_about_center_of_mass_kilogram_square_meters"});

    VehicleRigidBodyDefinition body;
    body.name = RequireString(value.at("name"), path + ".name");
    body.moves_freely_in_world =
        RequireBool(value.at("moves_freely_in_world"), path + ".moves_freely_in_world");
    body.mass_kilograms = RequireFiniteNumber(value.at("mass_kilograms"), path + ".mass_kilograms");
    body.center_of_mass_in_body_frame_meters =
        RequireVector(value.at("center_of_mass_in_body_frame_meters"),
                      path + ".center_of_mass_in_body_frame_meters");
    body.inertia_moments_about_center_of_mass_kilogram_square_meters =
        RequireVector(value.at("inertia_moments_about_center_of_mass_kilogram_square_meters"),
                      path + ".inertia_moments_about_center_of_mass_kilogram_square_meters");
    body.inertia_products_about_center_of_mass_kilogram_square_meters =
        RequireVector(value.at("inertia_products_about_center_of_mass_kilogram_square_meters"),
                      path + ".inertia_products_about_center_of_mass_kilogram_square_meters");
    return body;
}

VehicleFixedFrameDefinition ParseFixedFrame(const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "body_name", "position_in_body_frame_meters", "rotation_in_body_frame"});
    VehicleFixedFrameDefinition frame;
    frame.name = RequireString(value.at("name"), path + ".name");
    frame.body_name = RequireString(value.at("body_name"), path + ".body_name");
    frame.position_in_body_frame_meters = RequireVector(value.at("position_in_body_frame_meters"),
                                                        path + ".position_in_body_frame_meters");
    frame.rotation_in_body_frame =
        RequireRotation(value.at("rotation_in_body_frame"), path + ".rotation_in_body_frame");
    return frame;
}

VehicleRevoluteJointDefinition ParseRevoluteJoint(const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "parent_frame_name", "child_frame_name", "axis_in_parent_frame",
                      "damping_newton_metre_seconds_per_radian"});
    VehicleRevoluteJointDefinition joint;
    joint.name = RequireString(value.at("name"), path + ".name");
    joint.parent_frame_name =
        RequireString(value.at("parent_frame_name"), path + ".parent_frame_name");
    joint.child_frame_name =
        RequireString(value.at("child_frame_name"), path + ".child_frame_name");
    joint.axis_in_parent_frame =
        RequireVector(value.at("axis_in_parent_frame"), path + ".axis_in_parent_frame");
    joint.damping_newton_metre_seconds_per_radian =
        RequireFiniteNumber(value.at("damping_newton_metre_seconds_per_radian"),
                            path + ".damping_newton_metre_seconds_per_radian");
    return joint;
}

VehicleBallRpyJointDefinition ParseBallRpyJoint(const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "parent_frame_name", "child_frame_name", "default_roll_pitch_yaw_angles_radians"});
    VehicleBallRpyJointDefinition joint;
    joint.name = RequireString(value.at("name"), path + ".name");
    joint.parent_frame_name =
        RequireString(value.at("parent_frame_name"), path + ".parent_frame_name");
    joint.child_frame_name =
        RequireString(value.at("child_frame_name"), path + ".child_frame_name");
    joint.default_roll_pitch_yaw_angles_radians =
        RequireVector(value.at("default_roll_pitch_yaw_angles_radians"),
                      path + ".default_roll_pitch_yaw_angles_radians");
    return joint;
}

VehicleWeldJointDefinition ParseWeldJoint(const Json& value, const std::string& path) {
    RequireExactKeys(value, path, {"name", "parent_frame_name", "child_frame_name"});
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
        RequireString(value.at("reference_frame_name"), path + ".reference_frame_name"),
        RequireString(value.at("opposite_frame_name"), path + ".opposite_frame_name")};
}

VehicleTranslationalSpringDamperDefinition ParseTranslationalSpringDamper(const Json& value,
                                                                          const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "reference_frame_name", "opposite_frame_name",
                      "stiffness_newtons_per_meter", "damping_newton_seconds_per_meter"});
    VehicleTranslationalSpringDamperDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    element.stiffness_newtons_per_meter = RequireVector(value.at("stiffness_newtons_per_meter"),
                                                        path + ".stiffness_newtons_per_meter");
    element.damping_newton_seconds_per_meter = RequireVector(
        value.at("damping_newton_seconds_per_meter"), path + ".damping_newton_seconds_per_meter");
    return element;
}

VehicleRollSpringDamperCoupleDefinition ParseRollSpringDamperCouple(const Json& value,
                                                                    const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "reference_frame_name", "opposite_frame_name",
         "stiffness_newton_meters_per_radian", "damping_newton_meter_seconds_per_radian"});
    VehicleRollSpringDamperCoupleDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    element.stiffness_newton_meters_per_radian =
        RequireFiniteNumber(value.at("stiffness_newton_meters_per_radian"),
                            path + ".stiffness_newton_meters_per_radian");
    element.damping_newton_meter_seconds_per_radian =
        RequireFiniteNumber(value.at("damping_newton_meter_seconds_per_radian"),
                            path + ".damping_newton_meter_seconds_per_radian");
    return element;
}

VehicleSeriesSpringViscousDamperDefinition ParseSeriesSpringViscousDamper(const Json& value,
                                                                          const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "reference_frame_name", "opposite_frame_name", "axis",
         "series_stiffness_newtons_per_meter", "series_damping_newton_seconds_per_meter"});
    VehicleSeriesSpringViscousDamperDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    element.axis = RequireString(value.at("axis"), path + ".axis");
    element.series_stiffness_newtons_per_meter =
        RequireFiniteNumber(value.at("series_stiffness_newtons_per_meter"),
                            path + ".series_stiffness_newtons_per_meter");
    element.series_damping_newton_seconds_per_meter =
        RequireFiniteNumber(value.at("series_damping_newton_seconds_per_meter"),
                            path + ".series_damping_newton_seconds_per_meter");
    return element;
}

VehiclePointToPointSeriesSpringBilinearDamperDefinition ParsePointToPointSeriesSpringBilinearDamper(
    const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "reference_frame_name", "opposite_frame_name",
         "series_stiffness_newtons_per_meter", "damping_below_knee_newton_seconds_per_meter",
         "knee_force_newtons", "damping_above_knee_newton_seconds_per_meter"});
    VehiclePointToPointSeriesSpringBilinearDamperDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    const auto finite_constant = [&value, &path](const char* key) {
        return RequireFiniteNumber(value.at(key), path + "." + key);
    };
    element.series_stiffness_newtons_per_meter =
        finite_constant("series_stiffness_newtons_per_meter");
    element.damping_below_knee_newton_seconds_per_meter =
        finite_constant("damping_below_knee_newton_seconds_per_meter");
    element.knee_force_newtons = finite_constant("knee_force_newtons");
    element.damping_above_knee_newton_seconds_per_meter =
        finite_constant("damping_above_knee_newton_seconds_per_meter");
    return element;
}

VehicleSaturatedPiecewiseLinearDamperDefinition ParseSaturatedPiecewiseLinearDamper(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "reference_frame_name", "opposite_frame_name", "axis", "curve"});
    VehicleSaturatedPiecewiseLinearDamperDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    element.axis = RequireString(value.at("axis"), path + ".axis");
    const Json& curve = value.at("curve");
    const std::string curve_path = path + ".curve";
    RequireArray(curve, curve_path);
    element.curve.reserve(curve.size());
    for (std::size_t point = 0; point < curve.size(); ++point) {
        const std::string point_path = ElementPath(curve_path, point);
        RequireExactKeys(curve[point], point_path,
                         {"relative_velocity_meters_per_second", "force_newtons"});
        element.curve.push_back(VehicleSaturatedPiecewiseLinearDamperPointDefinition{
            RequireFiniteNumber(curve[point].at("relative_velocity_meters_per_second"),
                                point_path + ".relative_velocity_meters_per_second"),
            RequireFiniteNumber(curve[point].at("force_newtons"), point_path + ".force_newtons")});
    }
    return element;
}

VehicleReferenceFrameLinearBushingDefinition ParseReferenceFrameLinearBushing(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "reference_frame_name", "opposite_frame_name",
                      "translational_stiffness_newtons_per_meter",
                      "translational_damping_newton_seconds_per_meter",
                      "rotational_damping_newton_meter_seconds_per_radian"});
    VehicleReferenceFrameLinearBushingDefinition element;
    element.name = RequireString(value.at("name"), path + ".name");
    const ForceElementEnds ends = ParseEnds(value, path);
    element.reference_frame_name = ends.reference_frame_name;
    element.opposite_frame_name = ends.opposite_frame_name;
    const auto coefficient = [&](const char* key) {
        return RequireVector(value.at(key), path + "." + key);
    };
    element.translational_stiffness_newtons_per_meter =
        coefficient("translational_stiffness_newtons_per_meter");
    element.translational_damping_newton_seconds_per_meter =
        coefficient("translational_damping_newton_seconds_per_meter");
    element.rotational_damping_newton_meter_seconds_per_radian =
        coefficient("rotational_damping_newton_meter_seconds_per_radian");
    return element;
}

template <typename Element, typename Parse>
std::vector<Element> ParseArray(const Json& root, const std::string& root_path,
                                const std::string& key, Parse parse) {
    const std::string path = root_path + "." + key;
    const Json& array = root.at(key);
    RequireArray(array, path);
    std::vector<Element> elements;
    elements.reserve(array.size());
    for (std::size_t index = 0; index < array.size(); ++index) {
        elements.push_back(parse(array[index], ElementPath(path, index)));
    }
    return elements;
}

VehicleFreeBodyStationOffsetDefinition ParseFreeBodyStationOffset(const Json& value,
                                                                  const std::string& path) {
    RequireExactKeys(value, path, {"body_name", "station_offset_meters"});
    VehicleFreeBodyStationOffsetDefinition offset;
    offset.body_name = RequireString(value.at("body_name"), path + ".body_name");
    offset.station_offset_meters =
        RequireFiniteNumber(value.at("station_offset_meters"), path + ".station_offset_meters");
    return offset;
}

VehicleMechanicalTrackStationLayoutDefinition ParseMechanicalTrackStationLayout(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"reference_body_name", "free_body_station_offsets",
                      "wheel_pair_station_reference_body_names"});
    VehicleMechanicalTrackStationLayoutDefinition layout;
    layout.reference_body_name =
        RequireString(value.at("reference_body_name"), path + ".reference_body_name");

    const std::string offsets_path = path + ".free_body_station_offsets";
    const Json& offsets = value.at("free_body_station_offsets");
    RequireArray(offsets, offsets_path);
    layout.free_body_station_offsets.reserve(offsets.size());
    for (std::size_t index = 0; index < offsets.size(); ++index) {
        layout.free_body_station_offsets.push_back(
            ParseFreeBodyStationOffset(offsets[index], ElementPath(offsets_path, index)));
    }

    const std::string station_references_path = path + ".wheel_pair_station_reference_body_names";
    const Json& station_references = value.at("wheel_pair_station_reference_body_names");
    RequireArray(station_references, station_references_path);
    layout.wheel_pair_station_reference_body_names.reserve(station_references.size());
    for (std::size_t index = 0; index < station_references.size(); ++index) {
        const std::string element_path = ElementPath(station_references_path, index);
        std::string name = RequireString(station_references[index], element_path);
        layout.wheel_pair_station_reference_body_names.push_back(std::move(name));
    }
    return layout;
}

Json EncodeVector(const Eigen::Vector3d& vector) {
    return {{"x", vector.x()}, {"y", vector.y()}, {"z", vector.z()}};
}

Json EncodeRotation(const Eigen::Matrix3d& rotation) {
    if ((rotation.array() == Eigen::Matrix3d::Identity().array()).all()) {
        return {{"form", "aligned_with_body"}};
    }
    // Keep every supplied entry, including finite unfinished orientations.
    Json rows = Json::array();
    for (Eigen::Index row = 0; row < 3; ++row) {
        rows.push_back(EncodeVector(rotation.row(row).transpose()));
    }
    return {{"form", "matrix_rows"}, {"rows", std::move(rows)}};
}

template <typename Element, typename Encode>
Json EncodeArray(const std::vector<Element>& elements, Encode encode) {
    Json array = Json::array();
    for (const auto& element : elements) {
        array.push_back(encode(element));
    }
    return array;
}

template <typename Element>
Json EncodeForceEnds(const Element& element) {
    return {{"name", element.name},
            {"reference_frame_name", element.reference_frame_name},
            {"opposite_frame_name", element.opposite_frame_name}};
}

}  // namespace

VehicleDefinition DecodeVehicleDefinitionFromJson(const Json& root, const std::string& path) {
    RequireExactKeys(
        root, path,
        {"vehicle_name", "mechanical_definition_identifier", "mechanical_track_station_layout",
         "rigid_bodies", "fixed_frames", "revolute_joints", "ball_rpy_joints", "weld_joints",
         "translational_spring_dampers", "roll_spring_damper_couples",
         "series_spring_viscous_dampers", "point_to_point_series_spring_bilinear_dampers",
         "saturated_piecewise_linear_dampers", "reference_frame_linear_bushings",
         "point_to_point_linear_spring_dampers", "point_to_point_hysteretic_springs"});

    VehicleDefinition vehicle;
    vehicle.vehicle_name = RequireString(root.at("vehicle_name"), path + ".vehicle_name");
    vehicle.mechanical_definition_identifier = RequireString(
        root.at("mechanical_definition_identifier"), path + ".mechanical_definition_identifier");
    vehicle.mechanical_track_station_layout = ParseMechanicalTrackStationLayout(
        root.at("mechanical_track_station_layout"), path + ".mechanical_track_station_layout");
    vehicle.rigid_bodies =
        ParseArray<VehicleRigidBodyDefinition>(root, path, "rigid_bodies", ParseRigidBody);
    vehicle.fixed_frames =
        ParseArray<VehicleFixedFrameDefinition>(root, path, "fixed_frames", ParseFixedFrame);
    vehicle.revolute_joints = ParseArray<VehicleRevoluteJointDefinition>(
        root, path, "revolute_joints", ParseRevoluteJoint);
    vehicle.ball_rpy_joints =
        ParseArray<VehicleBallRpyJointDefinition>(root, path, "ball_rpy_joints", ParseBallRpyJoint);
    vehicle.weld_joints =
        ParseArray<VehicleWeldJointDefinition>(root, path, "weld_joints", ParseWeldJoint);
    vehicle.translational_spring_dampers = ParseArray<VehicleTranslationalSpringDamperDefinition>(
        root, path, "translational_spring_dampers", ParseTranslationalSpringDamper);
    vehicle.roll_spring_damper_couples = ParseArray<VehicleRollSpringDamperCoupleDefinition>(
        root, path, "roll_spring_damper_couples", ParseRollSpringDamperCouple);
    vehicle.series_spring_viscous_dampers = ParseArray<VehicleSeriesSpringViscousDamperDefinition>(
        root, path, "series_spring_viscous_dampers", ParseSeriesSpringViscousDamper);
    vehicle.point_to_point_series_spring_bilinear_dampers =
        ParseArray<VehiclePointToPointSeriesSpringBilinearDamperDefinition>(
            root, path, "point_to_point_series_spring_bilinear_dampers",
            ParsePointToPointSeriesSpringBilinearDamper);
    vehicle.saturated_piecewise_linear_dampers =
        ParseArray<VehicleSaturatedPiecewiseLinearDamperDefinition>(
            root, path, "saturated_piecewise_linear_dampers", ParseSaturatedPiecewiseLinearDamper);
    vehicle.reference_frame_linear_bushings =
        ParseArray<VehicleReferenceFrameLinearBushingDefinition>(
            root, path, "reference_frame_linear_bushings", ParseReferenceFrameLinearBushing);

    vehicle.point_to_point_linear_spring_dampers =
        ParseArray<PointToPointLinearSpringDamperDefinition>(
            root, path, "point_to_point_linear_spring_dampers",
            internal::ParsePointToPointLinearSpringDamper);
    vehicle.point_to_point_hysteretic_springs = ParseArray<PointToPointHystereticSpringDefinition>(
        root, path, "point_to_point_hysteretic_springs",
        internal::ParsePointToPointHystereticSpring);

    return vehicle;
}

Json EncodeVehicleDefinitionToJson(const VehicleDefinition& vehicle, const std::string& path) {
    Json result = Json::object();
    result["vehicle_name"] = vehicle.vehicle_name;
    result["mechanical_definition_identifier"] = vehicle.mechanical_definition_identifier;
    const auto& layout = vehicle.mechanical_track_station_layout;
    result["mechanical_track_station_layout"] = {
        {"reference_body_name", layout.reference_body_name},
        {"free_body_station_offsets", EncodeArray(layout.free_body_station_offsets,
                                                  [](const auto& offset) -> Json {
                                                      return {{"body_name", offset.body_name},
                                                              {"station_offset_meters",
                                                               offset.station_offset_meters}};
                                                  })},
        {"wheel_pair_station_reference_body_names",
         layout.wheel_pair_station_reference_body_names}};
    result["rigid_bodies"] = EncodeArray(vehicle.rigid_bodies, [](const auto& body) -> Json {
        return {{"name", body.name},
                {"moves_freely_in_world", body.moves_freely_in_world},
                {"mass_kilograms", body.mass_kilograms},
                {"center_of_mass_in_body_frame_meters",
                 EncodeVector(body.center_of_mass_in_body_frame_meters)},
                {"inertia_moments_about_center_of_mass_kilogram_square_meters",
                 EncodeVector(body.inertia_moments_about_center_of_mass_kilogram_square_meters)},
                {"inertia_products_about_center_of_mass_kilogram_square_meters",
                 EncodeVector(body.inertia_products_about_center_of_mass_kilogram_square_meters)}};
    });
    result["fixed_frames"] = EncodeArray(vehicle.fixed_frames, [](const auto& frame) -> Json {
        return {
            {"name", frame.name},
            {"body_name", frame.body_name},
            {"position_in_body_frame_meters", EncodeVector(frame.position_in_body_frame_meters)},
            {"rotation_in_body_frame", EncodeRotation(frame.rotation_in_body_frame)}};
    });
    result["revolute_joints"] = EncodeArray(vehicle.revolute_joints, [](const auto& joint) -> Json {
        return {{"name", joint.name},
                {"parent_frame_name", joint.parent_frame_name},
                {"child_frame_name", joint.child_frame_name},
                {"axis_in_parent_frame", EncodeVector(joint.axis_in_parent_frame)},
                {"damping_newton_metre_seconds_per_radian",
                 joint.damping_newton_metre_seconds_per_radian}};
    });
    result["ball_rpy_joints"] = EncodeArray(vehicle.ball_rpy_joints, [](const auto& joint) -> Json {
        return {{"name", joint.name},
                {"parent_frame_name", joint.parent_frame_name},
                {"child_frame_name", joint.child_frame_name},
                {"default_roll_pitch_yaw_angles_radians",
                 EncodeVector(joint.default_roll_pitch_yaw_angles_radians)}};
    });
    result["weld_joints"] = EncodeArray(vehicle.weld_joints, [](const auto& joint) -> Json {
        return {{"name", joint.name},
                {"parent_frame_name", joint.parent_frame_name},
                {"child_frame_name", joint.child_frame_name}};
    });
    result["translational_spring_dampers"] =
        EncodeArray(vehicle.translational_spring_dampers, [](const auto& element) {
            Json encoded = EncodeForceEnds(element);
            encoded["stiffness_newtons_per_meter"] =
                EncodeVector(element.stiffness_newtons_per_meter);
            encoded["damping_newton_seconds_per_meter"] =
                EncodeVector(element.damping_newton_seconds_per_meter);
            return encoded;
        });
    result["roll_spring_damper_couples"] =
        EncodeArray(vehicle.roll_spring_damper_couples, [](const auto& element) {
            Json encoded = EncodeForceEnds(element);
            encoded["stiffness_newton_meters_per_radian"] =
                element.stiffness_newton_meters_per_radian;
            encoded["damping_newton_meter_seconds_per_radian"] =
                element.damping_newton_meter_seconds_per_radian;
            return encoded;
        });
    result["series_spring_viscous_dampers"] =
        EncodeArray(vehicle.series_spring_viscous_dampers, [](const auto& element) {
            Json encoded = EncodeForceEnds(element);
            encoded["axis"] = element.axis;
            encoded["series_stiffness_newtons_per_meter"] =
                element.series_stiffness_newtons_per_meter;
            encoded["series_damping_newton_seconds_per_meter"] =
                element.series_damping_newton_seconds_per_meter;
            return encoded;
        });
    result["point_to_point_series_spring_bilinear_dampers"] =
        EncodeArray(vehicle.point_to_point_series_spring_bilinear_dampers, [](const auto& element) {
            Json encoded = EncodeForceEnds(element);
            encoded["series_stiffness_newtons_per_meter"] =
                element.series_stiffness_newtons_per_meter;
            encoded["damping_below_knee_newton_seconds_per_meter"] =
                element.damping_below_knee_newton_seconds_per_meter;
            encoded["knee_force_newtons"] = element.knee_force_newtons;
            encoded["damping_above_knee_newton_seconds_per_meter"] =
                element.damping_above_knee_newton_seconds_per_meter;
            return encoded;
        });
    result["saturated_piecewise_linear_dampers"] =
        EncodeArray(vehicle.saturated_piecewise_linear_dampers, [](const auto& element) {
            Json encoded = EncodeForceEnds(element);
            encoded["axis"] = element.axis;
            encoded["curve"] = EncodeArray(element.curve, [](const auto& point) -> Json {
                return {{"relative_velocity_meters_per_second",
                         point.relative_velocity_meters_per_second},
                        {"force_newtons", point.force_newtons}};
            });
            return encoded;
        });
    result["reference_frame_linear_bushings"] =
        EncodeArray(vehicle.reference_frame_linear_bushings, [](const auto& element) {
            Json encoded = EncodeForceEnds(element);
            encoded["translational_stiffness_newtons_per_meter"] =
                EncodeVector(element.translational_stiffness_newtons_per_meter);
            encoded["translational_damping_newton_seconds_per_meter"] =
                EncodeVector(element.translational_damping_newton_seconds_per_meter);
            encoded["rotational_damping_newton_meter_seconds_per_radian"] =
                EncodeVector(element.rotational_damping_newton_meter_seconds_per_radian);
            return encoded;
        });
    result["point_to_point_linear_spring_dampers"] =
        EncodeArray(vehicle.point_to_point_linear_spring_dampers, [](const auto& element) {
            Json encoded = EncodeForceEnds(element);
            encoded["nominal_length_meters"] = element.nominal_length_meters;
            encoded["stiffness_newtons_per_meter"] = element.stiffness_newtons_per_meter;
            encoded["damping_newton_seconds_per_meter"] = element.damping_newton_seconds_per_meter;
            return encoded;
        });
    result["point_to_point_hysteretic_springs"] =
        EncodeArray(vehicle.point_to_point_hysteretic_springs, [](const auto& element) {
            Json encoded = EncodeForceEnds(element);
            encoded["nominal_length_meters"] = element.nominal_length_meters;
            encoded["stiffness_newtons_per_meter"] = element.stiffness_newtons_per_meter;
            encoded["hysteresis_ratio"] = element.hysteresis_ratio;
            encoded["hysteresis_transition_speed_meters_per_second"] =
                element.hysteresis_transition_speed_meters_per_second;
            return encoded;
        });
    strict_json::RequireFiniteJsonNumbers(result, path);
    return result;
}

}  // namespace orvd::configuration::internal
