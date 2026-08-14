#include "orvd/configuration/load_irw_longitudinal_cruise_controller.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

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
using strict_json::RequireIdentifier;

actuation::WheelDriveTorqueChannelValues ParseForwardSigns(
    const Json& value, const std::string& path) {
    RequireArray(value, path);
    if (value.size() != actuation::kWheelDriveTorqueChannelCount) {
        throw std::invalid_argument(
            path + " must contain exactly " +
            std::to_string(actuation::kWheelDriveTorqueChannelCount) +
            " values, but contains " + std::to_string(value.size()));
    }
    actuation::WheelDriveTorqueChannelValues signs{};
    for (std::size_t index = 0; index < signs.size(); ++index) {
        signs[index] =
            RequireFiniteNumber(value[index], ElementPath(path, index));
        if (signs[index] != -1.0 && signs[index] != 1.0) {
            throw std::invalid_argument(
                ElementPath(path, index) + " must be -1 or +1");
        }
    }
    return signs;
}

}  // namespace

IrwLongitudinalCruiseControllerAsset
LoadIrwLongitudinalCruiseControllerFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const Json root = ParseStrictJson(ReadWholeFile(
        configuration_path, "IRW longitudinal cruise controller"));
    RequireExactKeys(
        root, "$",
        {"irw_longitudinal_cruise_controller_identifier",
         "sample_period_seconds", "target_speed_meters_per_second",
         "nominal_rolling_radius_meters", "forward_joint_rate_signs",
         "speed_pi"});
    const Json& speed_pi = root.at("speed_pi");
    RequireExactKeys(
        speed_pi, "$.speed_pi",
        {"proportional_gain", "integral_time_seconds",
         "output_filter_time_constant_seconds",
         "integral_absolute_limit_meters",
         "raw_output_absolute_limit_newton_metres"});

    control::SampledLongitudinalCruiseControllerConfig controller_config;
    controller_config.identifier = RequireIdentifier(
        root.at("irw_longitudinal_cruise_controller_identifier"),
        "$.irw_longitudinal_cruise_controller_identifier",
        "IRW longitudinal cruise controller identifier");
    controller_config.sample_period_seconds = RequireFiniteNumber(
        root.at("sample_period_seconds"), "$.sample_period_seconds");
    controller_config.target_speed_meters_per_second = RequireFiniteNumber(
        root.at("target_speed_meters_per_second"),
        "$.target_speed_meters_per_second");
    controller_config.speed_pi.proportional_gain = RequireFiniteNumber(
        speed_pi.at("proportional_gain"),
        "$.speed_pi.proportional_gain");
    controller_config.speed_pi.integral_time_seconds = RequireFiniteNumber(
        speed_pi.at("integral_time_seconds"),
        "$.speed_pi.integral_time_seconds");
    controller_config.speed_pi.output_filter_time_constant_seconds =
        RequireFiniteNumber(
            speed_pi.at("output_filter_time_constant_seconds"),
            "$.speed_pi.output_filter_time_constant_seconds");
    controller_config.speed_pi.integral_absolute_limit =
        RequireFiniteNumber(speed_pi.at("integral_absolute_limit_meters"),
                            "$.speed_pi.integral_absolute_limit_meters");
    controller_config.speed_pi.raw_output_absolute_limit =
        RequireFiniteNumber(
            speed_pi.at("raw_output_absolute_limit_newton_metres"),
            "$.speed_pi.raw_output_absolute_limit_newton_metres");

    const double nominal_rolling_radius_meters = RequireFiniteNumber(
        root.at("nominal_rolling_radius_meters"),
        "$.nominal_rolling_radius_meters");
    if (!(nominal_rolling_radius_meters > 0.0)) {
        throw std::invalid_argument(
            "$.nominal_rolling_radius_meters must be positive");
    }
    auto forward_joint_rate_signs = ParseForwardSigns(
        root.at("forward_joint_rate_signs"),
        "$.forward_joint_rate_signs");

    return IrwLongitudinalCruiseControllerAsset{
        control::SampledLongitudinalCruiseController(
            std::move(controller_config)),
        nominal_rolling_radius_meters,
        std::move(forward_joint_rate_signs)};
}

}  // namespace orvd::configuration
