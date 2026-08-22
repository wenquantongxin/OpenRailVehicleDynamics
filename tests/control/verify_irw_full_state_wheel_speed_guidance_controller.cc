#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "orvd/configuration/load_irw_full_state_wheel_speed_guidance_controller.h"
#include "orvd/control/irw_full_state_wheel_speed_guidance_controller.h"
#include "orvd/control/sampled_filtered_pi.h"

namespace {

using orvd::configuration::
    LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile;
using orvd::control::IrwFullStateWheelSpeedGuidanceController;
using orvd::control::IrwFullStateWheelSpeedGuidanceControllerConfig;
using orvd::control::IrwFullStateWheelSpeedGuidanceControllerInput;
using orvd::control::IrwFullStateWheelSpeedGuidanceControllerState;
using orvd::control::IrwFullStateWheelSpeedGuidanceMechanicalInput;
using orvd::control::IrwFullStateWheelSpeedGuidanceOperatingPoint;
using orvd::control::IrwFullStateWheelSpeedGuidanceRecurrence;
using orvd::control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig;
using orvd::control::SampledFilteredPi;
using orvd::control::SampledFilteredPiConfig;
using orvd::control::SampledFilteredPiState;

int failures = 0;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        std::fprintf(stderr, "IRW full-state controller: %.*s\n",
                     static_cast<int>(message.size()), message.data());
        ++failures;
    }
}

void RequireNear(double actual, double expected, double tolerance,
                 std::string_view message) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::fprintf(stderr,
                     "IRW full-state controller: %.*s: got %.17g, "
                     "expected %.17g\n",
                     static_cast<int>(message.size()), message.data(), actual,
                     expected);
        ++failures;
    }
}

IrwFullStateWheelSpeedGuidanceControllerConfig MakeAnalyticConfig() {
    IrwFullStateWheelSpeedGuidanceControllerConfig config;
    config.identifier = "analytic_irw_controller";
    config.sample_period_seconds = 0.1;
    config.base_speed_reference_meters_per_second = 10.0;
    config.curve_radius_meters = 100.0;
    config.controller_calibration_lateral_half_span_meters = 2.0;
    config.speed_reference_wheel_side_signs =
        {1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0};
    config.speed_reference_ramp_start_track_station_meters = 10.0;
    config.speed_reference_ramp_end_track_station_meters = 20.0;
    config.rolling_radius_meters = 0.5;
    config.guidance_axle_signs = {1.0, -1.0, 1.0, -1.0};
    config.guidance_wheel_signs.fill(-1.0);
    config.feedforward_gains = {0.0, 10.0, 1.0, 1.0};
    config.yaw_rate_feedback_gains = {1.0, 0.0, 0.0, 0.0};
    config.yaw_feedback_gains = {2.0, 0.0, 0.0, 0.0};
    config.lateral_velocity_feedback_gains = {3.0, 0.0, 0.0, 0.0};
    config.lateral_displacement_feedback_gains = {4.0, 0.0, 0.0, 0.0};
    config.lateral_integral_feedback_gains = {5.0, 0.0, 0.0, 0.0};
    config.wheel_speed_difference_feedback_gains = {0.0, 1.0, 1.0, 1.0};
    config
        .wheel_speed_difference_reference_absolute_limit_radians_per_second =
        0.25;
    config.yaw_rate_filter_time_constant_seconds = 0.1;
    config.lateral_velocity_filter_time_constant_seconds = 0.1;
    config.wheel_speed_difference_outer_filter_time_constant_seconds = 0.1;
    config.equilibrium_yaw_angles_radians.fill(0.0);
    config.equilibrium_lateral_displacements_meters.fill(0.0);
    config.guidance_start_track_station_meters = 10.0;
    config.guidance_end_track_station_meters = 30.0;
    config.lateral_integral_absolute_limit_meter_seconds = 1.0;
    config.wheel_speed_pi = SampledFilteredPiConfig{
        .proportional_gain = 2.0,
        .integral_time_seconds = 4.0,
        .output_filter_time_constant_seconds = 0.1,
        .integral_absolute_limit = 0.5,
        .raw_output_absolute_limit = 3.0,
    };
    config.wheel_speed_pi_wheel_signs.fill(-1.0);
    return config;
}

void CheckSampledFilteredPiOrder() {
    const SampledFilteredPi pi(SampledFilteredPiConfig{
        .proportional_gain = 4.0,
        .integral_time_seconds = 2.0,
        .output_filter_time_constant_seconds = 0.1,
        .integral_absolute_limit = 0.2,
        .raw_output_absolute_limit = 10.0,
    });
    const SampledFilteredPiState previous{.integral = 0.19,
                                          .filtered_output = 2.0};
    const auto unclamped = pi.Step(2.0, 0.1, previous);
    RequireNear(unclamped.next_state.integral, 0.2, 0.0,
                "PI integral was not updated and limited first");
    RequireNear(unclamped.output, 5.2, 1.0e-15,
                "PI output did not use the updated integral before filtering");
    Require(previous.integral == 0.19 && previous.filtered_output == 2.0,
            "PI Step modified its caller-owned previous state");

    const auto raw_limited = pi.Step(5.0, 0.1, previous);
    RequireNear(raw_limited.output, 6.0, 0.0,
                "PI raw limit was not applied before the output filter");
    RequireNear(raw_limited.next_state.filtered_output, 6.0, 0.0,
                "PI output and committed filter memory diverged");
}

void CheckFirstAndSubsequentDifferenceFiltering() {
    const IrwFullStateWheelSpeedGuidanceController controller(
        MakeAnalyticConfig());
    IrwFullStateWheelSpeedGuidanceControllerInput first_input;
    first_input.axle_lateral_displacements_meters = {0.1, 0.2, 0.3, 0.4};
    first_input.axle_yaw_angles_radians = {0.01, 0.02, 0.03, 0.04};
    first_input.axle_track_stations_meters.fill(15.0);
    const IrwFullStateWheelSpeedGuidanceControllerState cold_state;
    const auto first = controller.Step(first_input, cold_state);
    for (std::size_t axle = 0; axle < 4; ++axle) {
        Require(first.observations
                        .filtered_lateral_velocities_meters_per_second[axle] ==
                    0.0 &&
                    first.observations
                            .filtered_yaw_rates_radians_per_second[axle] ==
                        0.0,
                "first sample did not suppress an undefined difference");
    }
    Require(!cold_state.initialized,
            "controller Step modified the caller-owned cold state");

    auto second_input = first_input;
    for (std::size_t axle = 0; axle < 4; ++axle) {
        second_input.axle_lateral_displacements_meters[axle] += 0.2;
        second_input.axle_yaw_angles_radians[axle] += 0.1;
    }
    const auto second = controller.Step(second_input, first.next_state);
    for (std::size_t axle = 0; axle < 4; ++axle) {
        RequireNear(
            second.observations
                .filtered_lateral_velocities_meters_per_second[axle],
            1.0, 2.0e-15,
            "subsequent lateral difference did not pass through its filter");
        RequireNear(
            second.observations.filtered_yaw_rates_radians_per_second[axle],
            0.5, 1.0e-15,
            "subsequent yaw difference did not pass through its filter");
    }
}

IrwFullStateWheelSpeedGuidanceRecurrenceConfig MakeRecurrenceConfig(
    const IrwFullStateWheelSpeedGuidanceControllerConfig& config) {
    return IrwFullStateWheelSpeedGuidanceRecurrenceConfig{
        .sample_period_seconds = config.sample_period_seconds,
        .rolling_radius_meters = config.rolling_radius_meters,
        .guidance_axle_signs = config.guidance_axle_signs,
        .guidance_wheel_signs = config.guidance_wheel_signs,
        .lateral_integral_absolute_limit_meter_seconds =
            config.lateral_integral_absolute_limit_meter_seconds,
        .wheel_speed_pi = config.wheel_speed_pi,
        .wheel_speed_pi_wheel_signs = config.wheel_speed_pi_wheel_signs,
    };
}

IrwFullStateWheelSpeedGuidanceOperatingPoint MakeOperatingPoint(
    const IrwFullStateWheelSpeedGuidanceControllerConfig& config,
    const IrwFullStateWheelSpeedGuidanceControllerInput& input) {
    IrwFullStateWheelSpeedGuidanceOperatingPoint operating_point;
    for (std::size_t axle = 0; axle < 4; ++axle) {
        const double ramp_fraction = std::clamp(
            (input.axle_track_stations_meters[axle] -
             config.speed_reference_ramp_start_track_station_meters) /
                (config.speed_reference_ramp_end_track_station_meters -
                 config.speed_reference_ramp_start_track_station_meters),
            0.0, 1.0);
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t wheel = 2 * axle + side;
            const double straight =
                config.base_speed_reference_meters_per_second;
            const double curved =
                config.base_speed_reference_meters_per_second *
                (config.curve_radius_meters +
                 config.speed_reference_wheel_side_signs[wheel] *
                     config.controller_calibration_lateral_half_span_meters) /
                config.curve_radius_meters;
            operating_point
                .base_wheel_speed_references_meters_per_second[wheel] =
                (1.0 - ramp_fraction) * straight + ramp_fraction * curved;
        }
        operating_point.guidance_active[axle] =
            std::isfinite(input.axle_track_stations_meters[axle]) &&
            input.axle_track_stations_meters[axle] >=
                config.guidance_start_track_station_meters &&
            input.axle_track_stations_meters[axle] <=
                config.guidance_end_track_station_meters;
    }
    operating_point.feedforward_gains = config.feedforward_gains;
    operating_point.yaw_rate_feedback_gains =
        config.yaw_rate_feedback_gains;
    operating_point.yaw_feedback_gains = config.yaw_feedback_gains;
    operating_point.lateral_velocity_feedback_gains =
        config.lateral_velocity_feedback_gains;
    operating_point.lateral_displacement_feedback_gains =
        config.lateral_displacement_feedback_gains;
    operating_point.lateral_integral_feedback_gains =
        config.lateral_integral_feedback_gains;
    operating_point.wheel_speed_difference_feedback_gains =
        config.wheel_speed_difference_feedback_gains;
    operating_point.yaw_rate_filter_time_constants_seconds.fill(
        config.yaw_rate_filter_time_constant_seconds);
    operating_point.lateral_velocity_filter_time_constants_seconds.fill(
        config.lateral_velocity_filter_time_constant_seconds);
    operating_point
        .wheel_speed_difference_outer_filter_time_constants_seconds.fill(
            config
                .wheel_speed_difference_outer_filter_time_constant_seconds);
    operating_point
        .wheel_speed_difference_reference_absolute_limits_radians_per_second
        .fill(
            config
                .wheel_speed_difference_reference_absolute_limit_radians_per_second);
    operating_point.equilibrium_yaw_angles_radians =
        config.equilibrium_yaw_angles_radians;
    operating_point.equilibrium_lateral_displacements_meters =
        config.equilibrium_lateral_displacements_meters;
    return operating_point;
}

void RequireSameResult(
    const orvd::control::IrwFullStateWheelSpeedGuidanceControllerResult& first,
    const orvd::control::IrwFullStateWheelSpeedGuidanceControllerResult& second) {
    const auto& first_observations = first.observations;
    const auto& second_observations = second.observations;
    const auto& first_state = first.next_state;
    const auto& second_state = second.next_state;
    Require(
        first.requested_wheel_torques_newton_metres ==
                second.requested_wheel_torques_newton_metres &&
            first_observations
                    .base_wheel_speed_references_meters_per_second ==
                second_observations
                    .base_wheel_speed_references_meters_per_second &&
            first_observations.wheel_speed_references_meters_per_second ==
                second_observations.wheel_speed_references_meters_per_second &&
            first_observations
                    .wheel_speed_difference_references_radians_per_second ==
                second_observations
                    .wheel_speed_difference_references_radians_per_second &&
            first_observations
                    .measured_wheel_speed_differences_radians_per_second ==
                second_observations
                    .measured_wheel_speed_differences_radians_per_second &&
            first_observations
                    .equilibrium_wheel_speed_differences_radians_per_second ==
                second_observations
                    .equilibrium_wheel_speed_differences_radians_per_second &&
            first_observations
                    .filtered_lateral_velocities_meters_per_second ==
                second_observations
                    .filtered_lateral_velocities_meters_per_second &&
            first_observations.filtered_yaw_rates_radians_per_second ==
                second_observations.filtered_yaw_rates_radians_per_second &&
            first_observations.guidance_active ==
                second_observations.guidance_active &&
            first_state.initialized == second_state.initialized &&
            first_state.previous_lateral_displacements_meters ==
                second_state.previous_lateral_displacements_meters &&
            first_state.previous_yaw_angles_radians ==
                second_state.previous_yaw_angles_radians &&
            first_state.filtered_lateral_velocities_meters_per_second ==
                second_state
                    .filtered_lateral_velocities_meters_per_second &&
            first_state.filtered_yaw_rates_radians_per_second ==
                second_state.filtered_yaw_rates_radians_per_second &&
            first_state.lateral_error_integrals_meter_seconds ==
                second_state.lateral_error_integrals_meter_seconds &&
            first_state
                    .filtered_wheel_speed_difference_commands_radians_per_second ==
                second_state
                    .filtered_wheel_speed_difference_commands_radians_per_second &&
            first_state.wheel_speed_pi_integrals_meters ==
                second_state.wheel_speed_pi_integrals_meters &&
            first_state.wheel_speed_pi_filtered_torques_newton_metres ==
                second_state.wheel_speed_pi_filtered_torques_newton_metres,
        "the fixed controller and explicit operating-point recurrence differ");
}

void CheckGuidanceBoundariesFeedbackAndPiChannels() {
    const auto config = MakeAnalyticConfig();
    const IrwFullStateWheelSpeedGuidanceController controller(config);
    IrwFullStateWheelSpeedGuidanceControllerState previous;
    previous.initialized = true;
    previous.filtered_lateral_velocities_meters_per_second.fill(0.4);
    previous.filtered_yaw_rates_radians_per_second.fill(0.2);
    previous.lateral_error_integrals_meter_seconds = {0.1, 0.2, 0.3, 0.4};
    previous.filtered_wheel_speed_difference_commands_radians_per_second =
        {0.2, 0.2, 0.2, 0.4};
    previous.wheel_speed_pi_integrals_meters.fill(0.4);
    for (std::size_t wheel = 0; wheel < 8; ++wheel) {
        previous.wheel_speed_pi_filtered_torques_newton_metres[wheel] =
            0.1 * static_cast<double>(wheel);
    }

    IrwFullStateWheelSpeedGuidanceControllerInput input;
    input.axle_lateral_displacements_meters = {0.2, 0.1, -0.1, 0.3};
    input.axle_yaw_angles_radians = {0.1, -0.2, 0.3, -0.4};
    input.axle_track_stations_meters = {10.0, 20.0, 30.0, 30.001};
    input
        .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second
        .fill(0.0);
    const auto result = controller.Step(input, previous);
    const IrwFullStateWheelSpeedGuidanceRecurrence recurrence(
        MakeRecurrenceConfig(config));
    const auto recurrence_result = recurrence.Step(
        IrwFullStateWheelSpeedGuidanceMechanicalInput{
            .axle_lateral_displacements_meters =
                input.axle_lateral_displacements_meters,
            .axle_yaw_angles_radians = input.axle_yaw_angles_radians,
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second =
                input
                    .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        },
        MakeOperatingPoint(config, input), previous);
    RequireSameResult(result, recurrence_result);

    Require(result.observations.guidance_active ==
                std::array<bool, 4>{true, true, true, false},
            "guidance interval endpoints were not inclusive");
    RequireNear(
        result.observations.base_wheel_speed_references_meters_per_second[0],
        10.0, 0.0, "speed-reference ramp start was not the straight value");
    RequireNear(
        result.observations.base_wheel_speed_references_meters_per_second[2],
        10.2, 2.0e-15,
        "speed-reference ramp end did not reach the curve value");
    RequireNear(
        result.observations.base_wheel_speed_references_meters_per_second[3],
        9.8, 2.0e-15,
        "right-wheel curve reference used the wrong side sign");

    // Axle zero carries all five nonzero full-state feedback terms. Its raw
    // target is 5.8 rad/s and the outer filter stores 3.0 rad/s, while the
    // published reference is limited to 0.25 rad/s. This distinguishes the
    // frozen pre-limit memory from a tempting post-limit rewrite.
    RequireNear(
        result.observations.filtered_lateral_velocities_meters_per_second[0],
        1.2, 3.0e-16, "nonzero lateral-velocity feedback was lost");
    RequireNear(
        result.observations.filtered_yaw_rates_radians_per_second[0], 0.6,
        3.0e-16, "nonzero yaw-rate feedback was lost");
    RequireNear(
        result.next_state
            .filtered_wheel_speed_difference_commands_radians_per_second[0],
        3.0, 2.0e-15,
        "outer-filter memory did not retain its pre-limit value");
    RequireNear(
        result.observations
            .wheel_speed_difference_references_radians_per_second[0],
        0.25, 0.0, "outer command was not limited after filtering");

    Require(result.next_state.lateral_error_integrals_meter_seconds[3] ==
                    0.0 &&
                result.next_state
                        .filtered_wheel_speed_difference_commands_radians_per_second
                            [3] == 0.0 &&
                result.observations
                        .wheel_speed_difference_references_radians_per_second
                            [3] == 0.0,
            "guidance shutdown did not independently clear both memories");

    for (std::size_t wheel = 0; wheel < 8; ++wheel) {
        RequireNear(result.next_state.wheel_speed_pi_integrals_meters[wheel],
                    0.5, 0.0,
                    "one of the eight PI integrals did not reach its limit");
        const double expected_filtered =
            1.5 + 0.05 * static_cast<double>(wheel);
        RequireNear(result.requested_wheel_torques_newton_metres[wheel],
                    expected_filtered, 3.0e-16,
                    "one of the eight PI channels did not apply raw limiting "
                    "before filtering");
        RequireNear(
            result.next_state
                .wheel_speed_pi_filtered_torques_newton_metres[wheel],
            expected_filtered, 3.0e-16,
            "PI request and next filter memory differ");
    }
}

std::string Read(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open the controller test asset");
    }
    const std::string source{std::istreambuf_iterator<char>(input),
                             std::istreambuf_iterator<char>()};
    std::string normalized;
    normalized.reserve(source.size());
    for (std::size_t index = 0; index < source.size(); ++index) {
        if (source[index] == '\r') {
            if (index + 1 < source.size() && source[index + 1] == '\n') {
                ++index;
            }
            normalized.push_back('\n');
        } else {
            normalized.push_back(source[index]);
        }
    }
    return normalized;
}

void Write(const std::filesystem::path& path, std::string_view contents) {
    std::ofstream output(path, std::ios::binary);
    if (!output || !(output << contents)) {
        throw std::runtime_error("could not write a controller test fixture");
    }
}

std::string ReplaceOnce(std::string source, std::string_view needle,
                        std::string_view replacement) {
    const std::size_t position = source.find(needle);
    if (position == std::string::npos ||
        source.find(needle, position + needle.size()) != std::string::npos) {
        throw std::runtime_error(
            "controller test mutation did not identify one substring");
    }
    source.replace(position, needle.size(), replacement);
    return source;
}

class ScratchFile final {
   public:
    ScratchFile()
        : path_(std::filesystem::temp_directory_path() /
                ("orvd-g75-controller-" +
                 std::to_string(reinterpret_cast<std::uintptr_t>(this)) +
                 ".json")) {}
    ~ScratchFile() { std::filesystem::remove(path_); }
    [[nodiscard]] const std::filesystem::path& path() const noexcept {
        return path_;
    }

   private:
    std::filesystem::path path_;
};

void ExpectInvalid(const std::filesystem::path& scratch_path,
                   const std::string& valid_document,
                   const std::function<std::string(std::string)>& mutate,
                   std::string_view diagnostic_fragment) {
    Write(scratch_path, mutate(valid_document));
    try {
        static_cast<void>(
            LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile(
                scratch_path));
    } catch (const std::invalid_argument& error) {
        if (std::string_view(error.what()).find(diagnostic_fragment) ==
            std::string_view::npos) {
            std::fprintf(stderr,
                         "IRW full-state controller: expected diagnostic "
                         "containing '%.*s', got '%s'\n",
                         static_cast<int>(diagnostic_fragment.size()),
                         diagnostic_fragment.data(), error.what());
            ++failures;
        }
        return;
    }
    throw std::runtime_error("an invalid controller asset was accepted");
}

void CheckStrictAssetAndLoader(const std::filesystem::path& asset_path) {
    const auto controller =
        LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile(asset_path);
    const auto& config = controller.config();
    Require(config.identifier ==
                    "irw_r300_v60_full_state_wheel_speed_guidance_controller" &&
                config.sample_period_seconds == 0.01 &&
                config.rolling_radius_meters == 0.43 &&
                config.base_speed_reference_meters_per_second ==
                    16.666666666666668 &&
                config.curve_radius_meters == 300.0 &&
                config.controller_calibration_lateral_half_span_meters ==
                    0.75 &&
                config.speed_reference_ramp_start_track_station_meters ==
                    50.0 &&
                config.speed_reference_ramp_end_track_station_meters ==
                    100.0 &&
                config.guidance_start_track_station_meters == 50.0 &&
                config.guidance_end_track_station_meters == 600.0,
            "real controller identity scalars drifted");
    Require(config.speed_reference_wheel_side_signs ==
                    orvd::control::IrwGuidanceWheelValues{
                        1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0} &&
                config.guidance_axle_signs ==
                    orvd::control::IrwGuidanceAxleValues{1.0, 1.0, 1.0,
                                                        1.0} &&
                config.guidance_wheel_signs ==
                    orvd::control::IrwGuidanceWheelValues{
                        -1.0, -1.0, -1.0, -1.0,
                        -1.0, -1.0, -1.0, -1.0} &&
                config.wheel_speed_pi_wheel_signs ==
                    orvd::control::IrwGuidanceWheelValues{
                        -1.0, -1.0, -1.0, -1.0,
                        -1.0, -1.0, -1.0, -1.0},
            "one of the four independent frozen sign arrays drifted");
    Require(config.feedforward_gains ==
                    orvd::control::IrwGuidanceAxleValues{3.8, 0.1, -2.4,
                                                        1.45} &&
                config.wheel_speed_difference_feedback_gains ==
                    orvd::control::IrwGuidanceAxleValues{0.3, 0.3, 0.3,
                                                        0.81} &&
                config.wheel_speed_pi.proportional_gain == 20000.0 &&
                config.wheel_speed_pi.integral_time_seconds == 5000.0 &&
                config.wheel_speed_pi.output_filter_time_constant_seconds ==
                    0.0001,
            "real controller gains drifted");

    const std::string valid_document = Read(asset_path);
    ScratchFile scratch;
    ExpectInvalid(
        scratch.path(), valid_document,
        [](std::string source) {
            return ReplaceOnce(
                std::move(source), "\"sample_period_seconds\": 0.01,",
                "\"sample_period_seconds\": 0.01,\n  "
                "\"sample_period_seconds\": 0.01,");
        },
        "duplicate JSON object key at $.sample_period_seconds");
    ExpectInvalid(
        scratch.path(), valid_document,
        [](std::string source) {
            return ReplaceOnce(
                std::move(source), "\"sample_period_seconds\":",
                "\"legacy_mode\": false,\n  \"sample_period_seconds\":");
        },
        "$.legacy_mode");
    ExpectInvalid(
        scratch.path(), valid_document,
        [](std::string source) {
            return ReplaceOnce(std::move(source),
                               "  \"rolling_radius_meters\": 0.43,\n", "");
        },
        "$.rolling_radius_meters");
    ExpectInvalid(
        scratch.path(), valid_document,
        [](std::string source) {
            return ReplaceOnce(std::move(source),
                               "\"sample_period_seconds\": 0.01",
                               "\"sample_period_seconds\": \"0.01\"");
        },
        "$.sample_period_seconds");
    ExpectInvalid(
        scratch.path(), valid_document,
        [](std::string source) {
            return ReplaceOnce(
                std::move(source),
                "\"wheel_side_signs\": [1.0, -1.0, 1.0, -1.0, 1.0, "
                "-1.0, 1.0, -1.0]",
                "\"wheel_side_signs\": [1.0, -1.0, 1.0, -1.0, 1.0, "
                "-1.0, 1.0]");
        },
        "must contain exactly 8 values");
    ExpectInvalid(
        scratch.path(), valid_document,
        [](std::string source) {
            return ReplaceOnce(std::move(source),
                               "\"axle_signs\": [1.0, 1.0, 1.0, 1.0]",
                               "\"axle_signs\": [0.0, 1.0, 1.0, 1.0]");
        },
        "guidance_axle_signs entries must be -1 or +1");
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 2) {
            throw std::invalid_argument(
                "expected the real IRW controller asset path");
        }
        CheckSampledFilteredPiOrder();
        CheckFirstAndSubsequentDifferenceFiltering();
        CheckGuidanceBoundariesFeedbackAndPiChannels();
        CheckStrictAssetAndLoader(argv[1]);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW full-state controller: %s\n", error.what());
        return 1;
    }
    return failures == 0 ? 0 : 1;
}
