#include "orvd/configuration/load_resolved_startup_state.h"

#include <cstddef>
#include <string>
#include <vector>

#include "resolved_startup_state_invariants.h"
#include "strict_json.h"

namespace orvd::configuration {
namespace {

using strict_json::ElementPath;
using strict_json::Json;
using strict_json::ParseStrictJson;
using strict_json::ReadWholeFile;
using strict_json::RequireArray;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireFiniteVector3;
using strict_json::RequireIdentifier;
using strict_json::RequireObject;
using strict_json::RequireString;
using strict_json::ThrowExpected;

// A quaternion is written as four named components so a reader never has to
// know which order a library stores them in. It is not normalised here: the
// unit-norm requirement belongs to the record's invariants, which both this
// loader and the start-up assembly check.
Eigen::Quaterniond RequireQuaternion(const Json& value,
                                     const std::string& path) {
    RequireExactKeys(value, path, {"w", "x", "y", "z"});
    return Eigen::Quaterniond(RequireFiniteNumber(value.at("w"), path + ".w"),
                              RequireFiniteNumber(value.at("x"), path + ".x"),
                              RequireFiniteNumber(value.at("y"), path + ".y"),
                              RequireFiniteNumber(value.at("z"), path + ".z"));
}

StartupRunningDirection RequireRunningDirection(const Json& value,
                                                const std::string& path) {
    const std::string direction = RequireString(value, path);
    if (direction == "increasing_track_station") {
        return StartupRunningDirection::kIncreasingTrackStation;
    }
    throw std::invalid_argument(
        path + " is '" + direction +
        "'; only 'increasing_track_station' is admitted. Reverse and "
        "through-zero start-up await qualification of the longitudinal "
        "creepage and creep force near and across zero speed");
}

StartupVehicleBinding ParseVehicleBinding(const Json& value,
                                          const std::string& path) {
    RequireExactKeys(value, path,
                     {"vehicle_name", "mechanical_definition_identifier"});
    StartupVehicleBinding binding;
    binding.vehicle_name =
        RequireString(value.at("vehicle_name"), path + ".vehicle_name");
    if (binding.vehicle_name.empty()) {
        ThrowExpected(path + ".vehicle_name", "a non-empty vehicle name");
    }
    binding.mechanical_definition_identifier =
        RequireIdentifier(value.at("mechanical_definition_identifier"),
                          path + ".mechanical_definition_identifier",
                          "mechanical definition identifier");
    return binding;
}

StartupWheelRailBinding ParseWheelRailBinding(const Json& value,
                                              const std::string& path) {
    RequireExactKeys(value, path,
                     {"wheel_profile_identifier", "rail_profile_identifier",
                      "wheel_rail_contact_strategy_identifier"});
    StartupWheelRailBinding binding;
    binding.wheel_profile_identifier =
        RequireIdentifier(value.at("wheel_profile_identifier"),
                          path + ".wheel_profile_identifier",
                          "wheel profile identifier");
    binding.rail_profile_identifier =
        RequireIdentifier(value.at("rail_profile_identifier"),
                          path + ".rail_profile_identifier",
                          "rail profile identifier");
    binding.wheel_rail_contact_strategy_identifier = RequireIdentifier(
        value.at("wheel_rail_contact_strategy_identifier"),
        path + ".wheel_rail_contact_strategy_identifier",
        "wheel-rail contact strategy identifier");
    return binding;
}

WheelsetTargetSupportForces ParseTargetSupportForces(const Json& value,
                                                     const std::string& path) {
    RequireExactKeys(value, path,
                     {"wheelset_body_name", "left_support_force_newtons",
                      "right_support_force_newtons"});
    WheelsetTargetSupportForces forces;
    forces.wheelset_body_name = RequireString(value.at("wheelset_body_name"),
                                              path + ".wheelset_body_name");
    if (forces.wheelset_body_name.empty()) {
        ThrowExpected(path + ".wheelset_body_name",
                      "a non-empty rigid-body name");
    }
    forces.left_support_force_newtons =
        RequireFiniteNumber(value.at("left_support_force_newtons"),
                            path + ".left_support_force_newtons");
    forces.right_support_force_newtons =
        RequireFiniteNumber(value.at("right_support_force_newtons"),
                            path + ".right_support_force_newtons");
    return forces;
}

WheelsetStartupKinematics ParseWheelsetStartupKinematics(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"wheelset_body_name",
                      "forward_spin_axis_in_body_frame"});
    WheelsetStartupKinematics kinematics;
    kinematics.wheelset_body_name = RequireString(
        value.at("wheelset_body_name"), path + ".wheelset_body_name");
    if (kinematics.wheelset_body_name.empty()) {
        ThrowExpected(path + ".wheelset_body_name",
                      "a non-empty rigid-body name");
    }
    kinematics.forward_spin_axis_in_body_frame = RequireFiniteVector3(
        value.at("forward_spin_axis_in_body_frame"),
        path + ".forward_spin_axis_in_body_frame");
    return kinematics;
}

FreeBodyStartupState ParseFreeBodyStartupState(const Json& value,
                                               const std::string& path) {
    RequireExactKeys(
        value, path,
        {"body_name", "rotation_local_track_from_body",
         "resolved_track_station_offset_from_mechanical_layout_meters",
         "lateral_offset_in_local_track_frame_meters",
         "vertical_offset_in_local_track_frame_meters",
         "additional_body_angular_velocity_in_inertial_expressed_in_body_"
         "frame_radians_per_second",
         "body_origin_lateral_velocity_in_inertial_expressed_in_local_track_"
         "frame_meters_per_second",
         "body_origin_vertical_velocity_in_inertial_expressed_in_local_track_"
         "frame_meters_per_second"});
    FreeBodyStartupState body;
    body.body_name = RequireString(value.at("body_name"), path + ".body_name");
    if (body.body_name.empty()) {
        ThrowExpected(path + ".body_name", "a non-empty rigid-body name");
    }
    body.rotation_local_track_from_body =
        RequireQuaternion(value.at("rotation_local_track_from_body"),
                          path + ".rotation_local_track_from_body");
    body.resolved_track_station_offset_from_mechanical_layout_meters =
        RequireFiniteNumber(
            value.at(
                "resolved_track_station_offset_from_mechanical_layout_meters"),
            path +
                ".resolved_track_station_offset_from_mechanical_layout_meters");
    body.lateral_offset_in_local_track_frame_meters = RequireFiniteNumber(
        value.at("lateral_offset_in_local_track_frame_meters"),
        path + ".lateral_offset_in_local_track_frame_meters");
    body.vertical_offset_in_local_track_frame_meters = RequireFiniteNumber(
        value.at("vertical_offset_in_local_track_frame_meters"),
        path + ".vertical_offset_in_local_track_frame_meters");
    body
        .additional_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second =
        RequireFiniteVector3(
            value.at(
                "additional_body_angular_velocity_in_inertial_expressed_in_"
                "body_frame_radians_per_second"),
            path +
                ".additional_body_angular_velocity_in_inertial_expressed_in_"
                "body_frame_radians_per_second");
    body
        .body_origin_lateral_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second =
        RequireFiniteNumber(
            value.at(
                "body_origin_lateral_velocity_in_inertial_expressed_in_local_"
                "track_frame_meters_per_second"),
            path +
                ".body_origin_lateral_velocity_in_inertial_expressed_in_"
                "local_track_frame_meters_per_second");
    body
        .body_origin_vertical_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second =
        RequireFiniteNumber(
            value.at(
                "body_origin_vertical_velocity_in_inertial_expressed_in_local_"
                "track_frame_meters_per_second"),
            path +
                ".body_origin_vertical_velocity_in_inertial_expressed_in_"
                "local_track_frame_meters_per_second");
    return body;
}

RevoluteJointStartupState ParseRevoluteJointStartupState(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"joint_name", "position_radians",
                      "rate_per_common_wheel_spin_magnitude"});
    RevoluteJointStartupState joint;
    joint.joint_name =
        RequireString(value.at("joint_name"), path + ".joint_name");
    if (joint.joint_name.empty()) {
        ThrowExpected(path + ".joint_name", "a non-empty joint name");
    }
    joint.position_radians = RequireFiniteNumber(value.at("position_radians"),
                                                 path + ".position_radians");
    joint.rate_per_common_wheel_spin_magnitude = RequireFiniteNumber(
        value.at("rate_per_common_wheel_spin_magnitude"),
        path + ".rate_per_common_wheel_spin_magnitude");
    return joint;
}

SeriesSpringViscousDamperForceState ParseSeriesForceState(
    const Json& value, const std::string& path) {
    RequireExactKeys(value, path,
                     {"element_name", "reference_end_axial_force_newtons"});
    SeriesSpringViscousDamperForceState element;
    element.element_name =
        RequireString(value.at("element_name"), path + ".element_name");
    if (element.element_name.empty()) {
        ThrowExpected(path + ".element_name",
                      "a non-empty force-element name");
    }
    element.reference_end_axial_force_newtons =
        RequireFiniteNumber(value.at("reference_end_axial_force_newtons"),
                            path + ".reference_end_axial_force_newtons");
    return element;
}

TranslationalSpringDamperNominalForce ParseNominalForce(
    const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"element_name", "force_on_reference_end_in_reference_frame_newtons"});
    TranslationalSpringDamperNominalForce element;
    element.element_name =
        RequireString(value.at("element_name"), path + ".element_name");
    if (element.element_name.empty()) {
        ThrowExpected(path + ".element_name",
                      "a non-empty force-element name");
    }
    element.force_on_reference_end_in_reference_frame_newtons =
        RequireFiniteVector3(
            value.at("force_on_reference_end_in_reference_frame_newtons"),
            path + ".force_on_reference_end_in_reference_frame_newtons");
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

}  // namespace

ResolvedStartupState LoadResolvedStartupStateFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const std::string document =
        ReadWholeFile(configuration_path, "resolved start-up state");
    const Json root = ParseStrictJson(document);
    RequireExactKeys(
        root, "$",
        {"vehicle_binding", "wheel_rail_binding",
         "load_condition_identifier",
         "gravitational_acceleration_meters_per_second_squared",
         "running_direction", "initial_longitudinal_speed_meters_per_second",
         "common_startup_effective_rolling_radius_meters",
         "rail_profile_reference_vertical_offset_meters",
         "target_wheel_support_forces", "wheelset_startup_kinematics",
         "free_body_startup_states",
         "revolute_joint_startup_states",
         "series_spring_viscous_damper_force_states",
         "translational_spring_damper_nominal_forces"});

    ResolvedStartupState state;
    state.vehicle_binding =
        ParseVehicleBinding(root.at("vehicle_binding"), "$.vehicle_binding");
    RequireObject(root.at("wheel_rail_binding"), "$.wheel_rail_binding");
    state.wheel_rail_binding = ParseWheelRailBinding(
        root.at("wheel_rail_binding"), "$.wheel_rail_binding");
    state.load_condition_identifier =
        RequireIdentifier(root.at("load_condition_identifier"),
                          "$.load_condition_identifier",
                          "load condition identifier");
    state.gravitational_acceleration_meters_per_second_squared =
        RequireFiniteNumber(
            root.at("gravitational_acceleration_meters_per_second_squared"),
            "$.gravitational_acceleration_meters_per_second_squared");
    state.running_direction = RequireRunningDirection(
        root.at("running_direction"), "$.running_direction");
    state.initial_longitudinal_speed_meters_per_second = RequireFiniteNumber(
        root.at("initial_longitudinal_speed_meters_per_second"),
        "$.initial_longitudinal_speed_meters_per_second");
    state.common_startup_effective_rolling_radius_meters =
        RequireFiniteNumber(
            root.at("common_startup_effective_rolling_radius_meters"),
            "$.common_startup_effective_rolling_radius_meters");
    state.rail_profile_reference_vertical_offset_meters = RequireFiniteNumber(
        root.at("rail_profile_reference_vertical_offset_meters"),
        "$.rail_profile_reference_vertical_offset_meters");

    state.target_wheel_support_forces =
        ParseArray<WheelsetTargetSupportForces>(
            root, "target_wheel_support_forces", ParseTargetSupportForces);
    state.wheelset_startup_kinematics =
        ParseArray<WheelsetStartupKinematics>(
            root, "wheelset_startup_kinematics",
            ParseWheelsetStartupKinematics);
    state.free_body_startup_states = ParseArray<FreeBodyStartupState>(
        root, "free_body_startup_states", ParseFreeBodyStartupState);
    state.revolute_joint_startup_states =
        ParseArray<RevoluteJointStartupState>(
            root, "revolute_joint_startup_states",
            ParseRevoluteJointStartupState);
    state.series_spring_viscous_damper_force_states =
        ParseArray<SeriesSpringViscousDamperForceState>(
            root, "series_spring_viscous_damper_force_states",
            ParseSeriesForceState);
    state.translational_spring_damper_nominal_forces =
        ParseArray<TranslationalSpringDamperNominalForce>(
            root, "translational_spring_damper_nominal_forces",
            ParseNominalForce);

    internal::RequireResolvedStartupStateInvariants(state, "$");
    return state;
}

}  // namespace orvd::configuration
