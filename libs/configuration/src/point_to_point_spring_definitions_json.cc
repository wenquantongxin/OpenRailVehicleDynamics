#include "point_to_point_spring_definitions_json.h"

namespace orvd::configuration::internal {
namespace {

using strict_json::Json;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireString;

}  // namespace

PointToPointLinearSpringDamperDefinition ParsePointToPointLinearSpringDamper(
    const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"name", "reference_frame_name", "opposite_frame_name", "nominal_length_meters",
         "stiffness_newtons_per_meter", "damping_newton_seconds_per_meter"});
    PointToPointLinearSpringDamperDefinition element{
        RequireString(value.at("name"), path + ".name"),
        RequireString(value.at("reference_frame_name"), path + ".reference_frame_name"),
        RequireString(value.at("opposite_frame_name"), path + ".opposite_frame_name"),
        RequireFiniteNumber(value.at("nominal_length_meters"), path + ".nominal_length_meters"),
        RequireFiniteNumber(value.at("stiffness_newtons_per_meter"),
                            path + ".stiffness_newtons_per_meter"),
        RequireFiniteNumber(value.at("damping_newton_seconds_per_meter"),
                            path + ".damping_newton_seconds_per_meter")};
    return element;
}

PointToPointHystereticSpringDefinition ParsePointToPointHystereticSpring(const Json& value,
                                                                         const std::string& path) {
    RequireExactKeys(value, path,
                     {"name", "reference_frame_name", "opposite_frame_name",
                      "nominal_length_meters", "stiffness_newtons_per_meter", "hysteresis_ratio",
                      "hysteresis_transition_speed_meters_per_second"});
    PointToPointHystereticSpringDefinition element{
        RequireString(value.at("name"), path + ".name"),
        RequireString(value.at("reference_frame_name"), path + ".reference_frame_name"),
        RequireString(value.at("opposite_frame_name"), path + ".opposite_frame_name"),
        RequireFiniteNumber(value.at("nominal_length_meters"), path + ".nominal_length_meters"),
        RequireFiniteNumber(value.at("stiffness_newtons_per_meter"),
                            path + ".stiffness_newtons_per_meter"),
        RequireFiniteNumber(value.at("hysteresis_ratio"), path + ".hysteresis_ratio"),
        RequireFiniteNumber(value.at("hysteresis_transition_speed_meters_per_second"),
                            path + ".hysteresis_transition_speed_meters_per_second")};
    return element;
}

}  // namespace orvd::configuration::internal
