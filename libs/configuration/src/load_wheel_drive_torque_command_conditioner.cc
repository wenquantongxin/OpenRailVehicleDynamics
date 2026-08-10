#include "orvd/configuration/load_wheel_drive_torque_command_conditioner.h"

#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "strict_json.h"

namespace orvd::configuration {
namespace {

using actuation::WheelDriveTorqueCommandConditionerConfig;
using actuation::WheelDriveTorqueDirectionTable;
using strict_json::ElementPath;
using strict_json::Json;
using strict_json::ParseStrictJson;
using strict_json::ReadWholeFile;
using strict_json::RequireArray;
using strict_json::RequireBool;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireIdentifier;

constexpr std::size_t kSpeedNodeCount =
    actuation::kWheelDriveTorqueSpeedNodeCount;

void RequireSpeedNodeCount(const Json& array, const std::string& path) {
    RequireArray(array, path);
    if (array.size() != kSpeedNodeCount) {
        throw std::invalid_argument(
            path + " must contain exactly " +
            std::to_string(kSpeedNodeCount) + " values, but contains " +
            std::to_string(array.size()));
    }
}

std::array<double, kSpeedNodeCount> ParseFiniteTable(
    const Json& value, const std::string& path) {
    RequireSpeedNodeCount(value, path);
    std::array<double, kSpeedNodeCount> result{};
    for (std::size_t index = 0; index < result.size(); ++index) {
        result[index] =
            RequireFiniteNumber(value[index], ElementPath(path, index));
    }
    return result;
}

std::array<std::optional<double>, kSpeedNodeCount> ParseOptionalFiniteTable(
    const Json& value, const std::string& path) {
    RequireSpeedNodeCount(value, path);
    std::array<std::optional<double>, kSpeedNodeCount> result{};
    for (std::size_t index = 0; index < result.size(); ++index) {
        if (!value[index].is_null()) {
            result[index] =
                RequireFiniteNumber(value[index], ElementPath(path, index));
        }
    }
    return result;
}

std::array<bool, kSpeedNodeCount> ParseBoolTable(
    const Json& value, const std::string& path) {
    RequireSpeedNodeCount(value, path);
    std::array<bool, kSpeedNodeCount> result{};
    for (std::size_t index = 0; index < result.size(); ++index) {
        result[index] = RequireBool(value[index], ElementPath(path, index));
    }
    return result;
}

WheelDriveTorqueDirectionTable ParseDirectionTable(
    const Json& value, const std::string& path) {
    RequireExactKeys(
        value, path,
        {"dynamic_limit_drive_side_newton_metres",
         "magnitude_increase_rate_drive_side_newton_metres_per_second",
         "magnitude_decrease_rate_drive_side_newton_metres_per_second",
         "dynamic_advisory"});
    return WheelDriveTorqueDirectionTable{
        .dynamic_limit_drive_side_newton_metres = ParseOptionalFiniteTable(
            value.at("dynamic_limit_drive_side_newton_metres"),
            path + ".dynamic_limit_drive_side_newton_metres"),
        .magnitude_increase_rate_drive_side_newton_metres_per_second =
            ParseFiniteTable(
                value.at(
                    "magnitude_increase_rate_drive_side_newton_metres_per_"
                    "second"),
                path +
                    ".magnitude_increase_rate_drive_side_newton_metres_per_"
                    "second"),
        .magnitude_decrease_rate_drive_side_newton_metres_per_second =
            ParseFiniteTable(
                value.at(
                    "magnitude_decrease_rate_drive_side_newton_metres_per_"
                    "second"),
                path +
                    ".magnitude_decrease_rate_drive_side_newton_metres_per_"
                    "second"),
        .dynamic_advisory = ParseBoolTable(
            value.at("dynamic_advisory"), path + ".dynamic_advisory"),
    };
}

}  // namespace

actuation::WheelDriveTorqueCommandConditioner
LoadWheelDriveTorqueCommandConditionerFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const Json root = ParseStrictJson(ReadWholeFile(
        configuration_path, "wheel-drive torque-command conditioner"));
    RequireExactKeys(
        root, "$",
        {"wheel_drive_torque_command_conditioner_identifier",
         "sample_period_seconds", "drive_ratio", "drive_efficiency",
         "request_deadband_newton_metres",
         "wheel_angular_speed_direction_threshold_radians_per_second",
         "forward_wheel_angular_speed_sign",
         "drive_speed_nodes_revolutions_per_minute", "traction",
         "regeneration"});

    WheelDriveTorqueCommandConditionerConfig config;
    config.identifier = RequireIdentifier(
        root.at("wheel_drive_torque_command_conditioner_identifier"),
        "$.wheel_drive_torque_command_conditioner_identifier",
        "wheel-drive torque-command conditioner identifier");
    config.sample_period_seconds = RequireFiniteNumber(
        root.at("sample_period_seconds"), "$.sample_period_seconds");
    config.drive_ratio =
        RequireFiniteNumber(root.at("drive_ratio"), "$.drive_ratio");
    config.drive_efficiency = RequireFiniteNumber(
        root.at("drive_efficiency"), "$.drive_efficiency");
    config.request_deadband_newton_metres = RequireFiniteNumber(
        root.at("request_deadband_newton_metres"),
        "$.request_deadband_newton_metres");
    config.wheel_angular_speed_direction_threshold_radians_per_second =
        RequireFiniteNumber(
            root.at(
                "wheel_angular_speed_direction_threshold_radians_per_second"),
            "$.wheel_angular_speed_direction_threshold_radians_per_second");
    config.forward_wheel_angular_speed_sign = RequireFiniteNumber(
        root.at("forward_wheel_angular_speed_sign"),
        "$.forward_wheel_angular_speed_sign");
    config.drive_speed_nodes_revolutions_per_minute = ParseFiniteTable(
        root.at("drive_speed_nodes_revolutions_per_minute"),
        "$.drive_speed_nodes_revolutions_per_minute");
    config.traction =
        ParseDirectionTable(root.at("traction"), "$.traction");
    config.regeneration =
        ParseDirectionTable(root.at("regeneration"), "$.regeneration");
    return actuation::WheelDriveTorqueCommandConditioner(std::move(config));
}

}  // namespace orvd::configuration
