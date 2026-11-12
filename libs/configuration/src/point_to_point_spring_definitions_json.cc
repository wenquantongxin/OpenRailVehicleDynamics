#include "point_to_point_spring_definitions_json.h"

namespace orvd::configuration::internal {
namespace {

using strict_json::Json;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireString;
using strict_json::ThrowExpected;

std::string ReadName(const Json& value, const std::string& path, const char* key) {
    auto name = RequireString(value.at(key), path + "." + key);
    if (name.empty()) {
        ThrowExpected(path + "." + key, "a non-empty name");
    }
    return name;
}

}  // namespace

PointToPointLinearSpringDamperDefinition ParsePointToPointLinearSpringDamper(
    const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "reference_frame_name", "opposite_frame_name", "nominal_length_meters",
         "stiffness_newtons_per_meter", "damping_newton_seconds_per_meter"});
    PointToPointLinearSpringDamperDefinition element{
        ReadName(value, path, "name"),
        ReadName(value, path, "reference_frame_name"),
        ReadName(value, path, "opposite_frame_name"),
        RequireFiniteNumber(value.at("nominal_length_meters"), path + ".nominal_length_meters"),
        RequireFiniteNumber(value.at("stiffness_newtons_per_meter"),
                            path + ".stiffness_newtons_per_meter"),
        RequireFiniteNumber(value.at("damping_newton_seconds_per_meter"),
                            path + ".damping_newton_seconds_per_meter")};
    if (!(element.nominal_length_meters > 0.0) || element.stiffness_newtons_per_meter < 0.0 ||
        element.damping_newton_seconds_per_meter < 0.0) {
        ThrowExpected(path, "positive nominal length and non-negative stiffness and damping");
    }
    return element;
}

PointToPointHystereticSpringDefinition ParsePointToPointHystereticSpring(const Json& value,
                                                                         const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "reference_frame_name", "opposite_frame_name",
                      "nominal_length_meters", "stiffness_newtons_per_meter", "hysteresis_ratio",
                      "hysteresis_transition_speed_meters_per_second"});
    PointToPointHystereticSpringDefinition element{
        ReadName(value, path, "name"),
        ReadName(value, path, "reference_frame_name"),
        ReadName(value, path, "opposite_frame_name"),
        RequireFiniteNumber(value.at("nominal_length_meters"), path + ".nominal_length_meters"),
        RequireFiniteNumber(value.at("stiffness_newtons_per_meter"),
                            path + ".stiffness_newtons_per_meter"),
        RequireFiniteNumber(value.at("hysteresis_ratio"), path + ".hysteresis_ratio"),
        RequireFiniteNumber(value.at("hysteresis_transition_speed_meters_per_second"),
                            path + ".hysteresis_transition_speed_meters_per_second")};
    if (!(element.nominal_length_meters > 0.0) || element.stiffness_newtons_per_meter < 0.0 ||
        element.hysteresis_ratio < 0.0 || element.hysteresis_ratio > 1.0 ||
        !(element.hysteresis_transition_speed_meters_per_second > 0.0)) {
        ThrowExpected(path,
                      "positive nominal length and transition speed, non-negative stiffness "
                      "and hysteresis ratio in [0, 1]");
    }
    return element;
}

}  // namespace orvd::configuration::internal
