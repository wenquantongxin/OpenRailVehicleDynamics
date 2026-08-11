#include "orvd/configuration/load_irw_full_state_wheel_speed_guidance_controller.h"

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

#include "strict_json.h"

namespace orvd::configuration {
namespace {

using control::IrwFullStateWheelSpeedGuidanceControllerConfig;
using strict_json::ElementPath;
using strict_json::Json;
using strict_json::ParseStrictJson;
using strict_json::ReadWholeFile;
using strict_json::RequireArray;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireIdentifier;

template <std::size_t Size>
std::array<double, Size> ParseFiniteArray(const Json& value,
                                          const std::string& path) {
    RequireArray(value, path);
    if (value.size() != Size) {
        throw std::invalid_argument(
            path + " must contain exactly " + std::to_string(Size) +
            " values, but contains " + std::to_string(value.size()));
    }
    std::array<double, Size> result{};
    for (std::size_t index = 0; index < Size; ++index) {
        result[index] =
            RequireFiniteNumber(value[index], ElementPath(path, index));
    }
    return result;
}

}  // namespace

control::IrwFullStateWheelSpeedGuidanceController
LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile(
    const std::filesystem::path& configuration_path) {
    const Json root = ParseStrictJson(ReadWholeFile(
        configuration_path,
        "IRW full-state wheel-speed guidance controller"));
    RequireExactKeys(
        root, "$",
        {"irw_full_state_wheel_speed_guidance_controller_identifier",
         "sample_period_seconds", "rolling_radius_meters",
         "wheel_speed_reference", "guidance", "wheel_speed_pi"});

    const Json& speed_reference = root.at("wheel_speed_reference");
    RequireExactKeys(
        speed_reference, "$.wheel_speed_reference",
        {"base_speed_meters_per_second", "curve_radius_meters",
         "controller_calibration_lateral_half_span_meters",
         "wheel_side_signs", "ramp_start_track_station_meters",
         "ramp_end_track_station_meters"});

    const Json& guidance = root.at("guidance");
    RequireExactKeys(
        guidance, "$.guidance",
        {"axle_signs", "wheel_signs", "feedforward_gains",
         "yaw_rate_feedback_gains", "yaw_feedback_gains",
         "lateral_velocity_feedback_gains",
         "lateral_displacement_feedback_gains",
         "lateral_integral_feedback_gains",
         "wheel_speed_difference_feedback_gains",
         "wheel_speed_difference_reference_absolute_limit_radians_per_second",
         "yaw_rate_filter_time_constant_seconds",
         "lateral_velocity_filter_time_constant_seconds",
         "wheel_speed_difference_outer_filter_time_constant_seconds",
         "equilibrium_yaw_angles_radians",
         "equilibrium_lateral_displacements_meters",
         "start_track_station_meters", "end_track_station_meters",
         "lateral_integral_absolute_limit_meter_seconds"});

    const Json& wheel_speed_pi = root.at("wheel_speed_pi");
    RequireExactKeys(
        wheel_speed_pi, "$.wheel_speed_pi",
        {"proportional_gain", "integral_time_seconds",
         "output_filter_time_constant_seconds",
         "integral_absolute_limit_meters",
         "raw_output_absolute_limit_newton_metres", "wheel_signs"});

    IrwFullStateWheelSpeedGuidanceControllerConfig config;
    config.identifier = RequireIdentifier(
        root.at(
            "irw_full_state_wheel_speed_guidance_controller_identifier"),
        "$.irw_full_state_wheel_speed_guidance_controller_identifier",
        "IRW full-state wheel-speed guidance controller identifier");
    config.sample_period_seconds = RequireFiniteNumber(
        root.at("sample_period_seconds"), "$.sample_period_seconds");
    config.rolling_radius_meters = RequireFiniteNumber(
        root.at("rolling_radius_meters"), "$.rolling_radius_meters");

    config.base_speed_reference_meters_per_second = RequireFiniteNumber(
        speed_reference.at("base_speed_meters_per_second"),
        "$.wheel_speed_reference.base_speed_meters_per_second");
    config.curve_radius_meters = RequireFiniteNumber(
        speed_reference.at("curve_radius_meters"),
        "$.wheel_speed_reference.curve_radius_meters");
    config.controller_calibration_lateral_half_span_meters =
        RequireFiniteNumber(
            speed_reference.at(
                "controller_calibration_lateral_half_span_meters"),
            "$.wheel_speed_reference."
            "controller_calibration_lateral_half_span_meters");
    config.speed_reference_wheel_side_signs =
        ParseFiniteArray<control::kIrwGuidanceWheelCount>(
            speed_reference.at("wheel_side_signs"),
            "$.wheel_speed_reference.wheel_side_signs");
    config.speed_reference_ramp_start_track_station_meters =
        RequireFiniteNumber(
            speed_reference.at("ramp_start_track_station_meters"),
            "$.wheel_speed_reference.ramp_start_track_station_meters");
    config.speed_reference_ramp_end_track_station_meters =
        RequireFiniteNumber(
            speed_reference.at("ramp_end_track_station_meters"),
            "$.wheel_speed_reference.ramp_end_track_station_meters");

    config.guidance_axle_signs =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("axle_signs"), "$.guidance.axle_signs");
    config.guidance_wheel_signs =
        ParseFiniteArray<control::kIrwGuidanceWheelCount>(
            guidance.at("wheel_signs"), "$.guidance.wheel_signs");
    config.feedforward_gains =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("feedforward_gains"),
            "$.guidance.feedforward_gains");
    config.yaw_rate_feedback_gains =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("yaw_rate_feedback_gains"),
            "$.guidance.yaw_rate_feedback_gains");
    config.yaw_feedback_gains =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("yaw_feedback_gains"),
            "$.guidance.yaw_feedback_gains");
    config.lateral_velocity_feedback_gains =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("lateral_velocity_feedback_gains"),
            "$.guidance.lateral_velocity_feedback_gains");
    config.lateral_displacement_feedback_gains =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("lateral_displacement_feedback_gains"),
            "$.guidance.lateral_displacement_feedback_gains");
    config.lateral_integral_feedback_gains =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("lateral_integral_feedback_gains"),
            "$.guidance.lateral_integral_feedback_gains");
    config.wheel_speed_difference_feedback_gains =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("wheel_speed_difference_feedback_gains"),
            "$.guidance.wheel_speed_difference_feedback_gains");
    config
        .wheel_speed_difference_reference_absolute_limit_radians_per_second =
        RequireFiniteNumber(
            guidance.at(
                "wheel_speed_difference_reference_absolute_limit_radians_"
                "per_second"),
            "$.guidance."
            "wheel_speed_difference_reference_absolute_limit_radians_per_"
            "second");
    config.yaw_rate_filter_time_constant_seconds = RequireFiniteNumber(
        guidance.at("yaw_rate_filter_time_constant_seconds"),
        "$.guidance.yaw_rate_filter_time_constant_seconds");
    config.lateral_velocity_filter_time_constant_seconds =
        RequireFiniteNumber(
            guidance.at("lateral_velocity_filter_time_constant_seconds"),
            "$.guidance.lateral_velocity_filter_time_constant_seconds");
    config.wheel_speed_difference_outer_filter_time_constant_seconds =
        RequireFiniteNumber(
            guidance.at(
                "wheel_speed_difference_outer_filter_time_constant_seconds"),
            "$.guidance."
            "wheel_speed_difference_outer_filter_time_constant_seconds");
    config.equilibrium_yaw_angles_radians =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("equilibrium_yaw_angles_radians"),
            "$.guidance.equilibrium_yaw_angles_radians");
    config.equilibrium_lateral_displacements_meters =
        ParseFiniteArray<control::kIrwGuidanceAxleCount>(
            guidance.at("equilibrium_lateral_displacements_meters"),
            "$.guidance.equilibrium_lateral_displacements_meters");
    config.guidance_start_track_station_meters = RequireFiniteNumber(
        guidance.at("start_track_station_meters"),
        "$.guidance.start_track_station_meters");
    config.guidance_end_track_station_meters = RequireFiniteNumber(
        guidance.at("end_track_station_meters"),
        "$.guidance.end_track_station_meters");
    config.lateral_integral_absolute_limit_meter_seconds =
        RequireFiniteNumber(
            guidance.at("lateral_integral_absolute_limit_meter_seconds"),
            "$.guidance.lateral_integral_absolute_limit_meter_seconds");

    config.wheel_speed_pi.proportional_gain = RequireFiniteNumber(
        wheel_speed_pi.at("proportional_gain"),
        "$.wheel_speed_pi.proportional_gain");
    config.wheel_speed_pi.integral_time_seconds = RequireFiniteNumber(
        wheel_speed_pi.at("integral_time_seconds"),
        "$.wheel_speed_pi.integral_time_seconds");
    config.wheel_speed_pi.output_filter_time_constant_seconds =
        RequireFiniteNumber(
            wheel_speed_pi.at("output_filter_time_constant_seconds"),
            "$.wheel_speed_pi.output_filter_time_constant_seconds");
    config.wheel_speed_pi.integral_absolute_limit = RequireFiniteNumber(
        wheel_speed_pi.at("integral_absolute_limit_meters"),
        "$.wheel_speed_pi.integral_absolute_limit_meters");
    config.wheel_speed_pi.raw_output_absolute_limit = RequireFiniteNumber(
        wheel_speed_pi.at("raw_output_absolute_limit_newton_metres"),
        "$.wheel_speed_pi.raw_output_absolute_limit_newton_metres");
    config.wheel_speed_pi_wheel_signs =
        ParseFiniteArray<control::kIrwGuidanceWheelCount>(
            wheel_speed_pi.at("wheel_signs"),
            "$.wheel_speed_pi.wheel_signs");

    return control::IrwFullStateWheelSpeedGuidanceController(
        std::move(config));
}

}  // namespace orvd::configuration
