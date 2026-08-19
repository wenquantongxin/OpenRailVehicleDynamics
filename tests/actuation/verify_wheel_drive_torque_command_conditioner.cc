#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "orvd/actuation/wheel_drive_torque_command_conditioner.h"
#include "orvd/configuration/load_wheel_drive_torque_command_conditioner.h"
#include "wheel_rail_contact/allocation_probe.h"

namespace {

using orvd::actuation::HasWheelDriveTorqueLimitFlag;
using orvd::actuation::WheelDriveTorqueChannelValues;
using orvd::actuation::WheelDriveTorqueCommandConditioner;
using orvd::actuation::WheelDriveTorqueCommandConditionerConfig;
using orvd::actuation::WheelDriveTorqueDirectionTable;
using orvd::actuation::WheelDriveTorqueLimitFlag;
using orvd::configuration::
    LoadWheelDriveTorqueCommandConditionerFromJsonFile;

int failures = 0;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        std::fprintf(stderr, "wheel-drive torque conditioner: %.*s\n",
                     static_cast<int>(message.size()), message.data());
        ++failures;
    }
}

void RequireNear(double actual, double expected, double tolerance,
                 std::string_view message) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::fprintf(stderr,
                     "wheel-drive torque conditioner: %.*s: got %.17g, "
                     "expected %.17g\n",
                     static_cast<int>(message.size()), message.data(), actual,
                     expected);
        ++failures;
    }
}

WheelDriveTorqueDirectionTable MakeDirectionTable(double limit_offset) {
    WheelDriveTorqueDirectionTable table;
    for (std::size_t index = 0;
         index < orvd::actuation::kWheelDriveTorqueSpeedNodeCount; ++index) {
        table.dynamic_limit_drive_side_newton_metres[index] =
            limit_offset + 10.0 * static_cast<double>(index);
        table
            .magnitude_increase_rate_drive_side_newton_metres_per_second
                [index] = 1000.0;
        table
            .magnitude_decrease_rate_drive_side_newton_metres_per_second
                [index] = 2000.0;
        table.dynamic_advisory[index] = false;
    }
    return table;
}

WheelDriveTorqueCommandConditionerConfig MakeConfig() {
    WheelDriveTorqueCommandConditionerConfig config;
    config.identifier = "analytic_conditioner";
    config.sample_period_seconds = 0.01;
    config.drive_ratio = 2.0;
    config.drive_efficiency = 0.5;
    config.request_deadband_newton_metres = 0.5;
    config.wheel_angular_speed_direction_threshold_radians_per_second = 0.1;
    config.forward_wheel_angular_speed_sign = -1.0;
    for (std::size_t index = 0;
         index < orvd::actuation::kWheelDriveTorqueSpeedNodeCount; ++index) {
        config.drive_speed_nodes_revolutions_per_minute[index] =
            100.0 * static_cast<double>(index + 1);
    }
    config.traction = MakeDirectionTable(100.0);
    config.regeneration = MakeDirectionTable(200.0);
    config.traction.dynamic_limit_drive_side_newton_metres.back() =
        std::nullopt;
    config.traction.dynamic_limit_drive_side_newton_metres[8] =
        std::nullopt;
    config.traction.dynamic_advisory.back() = true;
    config.traction.dynamic_advisory[8] = true;
    return config;
}

double WheelSpeedForDriveRpm(double drive_rpm) {
    return drive_rpm * 2.0 * std::numbers::pi / (2.0 * 60.0);
}

bool Has(WheelDriveTorqueLimitFlag value, WheelDriveTorqueLimitFlag flag) {
    return HasWheelDriveTorqueLimitFlag(value, flag);
}

void CheckEightDistinctBranches() {
    const WheelDriveTorqueCommandConditioner conditioner(MakeConfig());
    WheelDriveTorqueChannelValues requested{50.0, -50.0, -40.0, -40.0,
                                             0.5, 500.0, 80.0, -80.0};
    WheelDriveTorqueChannelValues speeds{
        WheelSpeedForDriveRpm(250.0), WheelSpeedForDriveRpm(250.0),
        0.0, WheelSpeedForDriveRpm(1001.0),
        WheelSpeedForDriveRpm(250.0), WheelSpeedForDriveRpm(250.0),
        WheelSpeedForDriveRpm(250.0), WheelSpeedForDriveRpm(250.0)};
    WheelDriveTorqueChannelValues previous{50.0, -50.0, 40.0, 40.0,
                                            100.0, 115.0, 0.0, 5.0};
    const auto requested_before = requested;
    const auto speeds_before = speeds;
    const auto previous_before = previous;
    const auto result = conditioner.Step(requested, speeds, previous);

    Require(requested == requested_before && speeds == speeds_before &&
                previous == previous_before,
            "Step modified one of its three input arrays");

    RequireNear(result.wheel_dynamic_torque_limits_newton_metres[0], 115.0,
                2.0e-14,
                "traction interpolation did not use its source table");
    RequireNear(result.actual_wheel_torques_newton_metres[0], 50.0, 0.0,
                "traction interpolation changed an unconstrained request");
    Require(result.limit_flags[0] == WheelDriveTorqueLimitFlag::kNone,
            "unconstrained traction reported a limit");

    RequireNear(result.wheel_dynamic_torque_limits_newton_metres[1], 215.0,
                0.0,
                "regeneration interpolation did not use its source table");
    RequireNear(result.actual_wheel_torques_newton_metres[1], -50.0, 0.0,
                "regeneration changed an unconstrained request");

    RequireNear(result.actual_wheel_torques_newton_metres[2], -40.0, 0.0,
                "low-speed source direction did not use the forward sign");
    RequireNear(result.wheel_dynamic_torque_limits_newton_metres[2], 100.0,
                0.0, "low speed did not query the first table node");
    Require(Has(result.limit_flags[2],
                WheelDriveTorqueLimitFlag::kLowSpeedLookupClamped),
            "low-speed lookup clamp was not reported");

    Require(result.actual_wheel_torques_newton_metres[3] == 0.0 &&
                result.wheel_dynamic_torque_limits_newton_metres[3] == 0.0 &&
                result.next_drive_side_torque_memory_newton_metres[3] == 0.0,
            "unsupported high speed did not close output and memory");
    Require(Has(result.limit_flags[3],
                WheelDriveTorqueLimitFlag::kUnsupportedHighSpeed) &&
                Has(result.limit_flags[3],
                    WheelDriveTorqueLimitFlag::kDynamicValueAdvisory),
            "unsupported high speed did not report both active flags");

    Require(result.actual_wheel_torques_newton_metres[4] == 0.0 &&
                result.wheel_dynamic_torque_limits_newton_metres[4] == 0.0 &&
                result.next_drive_side_torque_memory_newton_metres[4] == 0.0 &&
                result.limit_flags[4] == WheelDriveTorqueLimitFlag::kNone,
            "the inclusive request deadband did not clear output and memory");

    RequireNear(result.actual_wheel_torques_newton_metres[5], 115.0, 2.0e-14,
                "magnitude limit did not cap the wheel-side output");
    Require(Has(result.limit_flags[5],
                WheelDriveTorqueLimitFlag::kMagnitudeLimited),
            "magnitude limiting was not reported");

    RequireNear(result.actual_wheel_torques_newton_metres[6], 10.0, 0.0,
                "increase-rate limit did not use one sample period");
    RequireNear(result.next_drive_side_torque_memory_newton_metres[6], 10.0,
                0.0, "increase-rate result did not become the next memory");
    Require(Has(result.limit_flags[6],
                WheelDriveTorqueLimitFlag::kSlewLimited),
            "increase-rate limiting was not reported");

    RequireNear(result.actual_wheel_torques_newton_metres[7], -5.0, 0.0,
                "cross-zero rate limiting did not retain signed memory");
    RequireNear(result.next_drive_side_torque_memory_newton_metres[7], -5.0,
                0.0, "cross-zero next memory has the wrong source sign");
    Require(Has(result.limit_flags[7],
                WheelDriveTorqueLimitFlag::kSlewLimited),
            "cross-zero rate limiting was not reported");
}

void CheckDecreaseRateAndUnavailableEndpoint() {
    const WheelDriveTorqueCommandConditioner conditioner(MakeConfig());
    WheelDriveTorqueChannelValues requested{};
    WheelDriveTorqueChannelValues speeds{};
    WheelDriveTorqueChannelValues previous{};

    requested[0] = 50.0;
    speeds[0] = WheelSpeedForDriveRpm(250.0);
    previous[0] = 80.0;
    requested[1] = 50.0;
    speeds[1] = WheelSpeedForDriveRpm(950.0);
    previous[1] = 10.0;
    const auto result = conditioner.Step(requested, speeds, previous);

    RequireNear(result.actual_wheel_torques_newton_metres[0], 60.0, 0.0,
                "decrease-rate branch did not use its own rate table");
    Require(Has(result.limit_flags[0],
                WheelDriveTorqueLimitFlag::kSlewLimited),
            "decrease-rate limiting was not reported");

    Require(result.actual_wheel_torques_newton_metres[1] == 0.0 &&
                result.wheel_dynamic_torque_limits_newton_metres[1] == 0.0 &&
                result.next_drive_side_torque_memory_newton_metres[1] == 0.0,
            "an unavailable dynamic endpoint did not close immediately");
    Require(Has(result.limit_flags[1],
                WheelDriveTorqueLimitFlag::kDynamicValueAdvisory) &&
                Has(result.limit_flags[1],
                    WheelDriveTorqueLimitFlag::kMagnitudeLimited) &&
                Has(result.limit_flags[1],
                    WheelDriveTorqueLimitFlag::kSlewLimited),
            "an unavailable endpoint did not preserve its active diagnostics");
}

std::string Read(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open conditioner test asset");
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
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output || !(output << contents)) {
        throw std::runtime_error("could not write conditioner test asset");
    }
}

std::string ReplaceOnce(std::string source, std::string_view needle,
                        std::string_view replacement) {
    const std::size_t position = source.find(needle);
    if (position == std::string::npos ||
        source.find(needle, position + needle.size()) != std::string::npos) {
        throw std::runtime_error(
            "conditioner test mutation did not identify one substring");
    }
    source.replace(position, needle.size(), replacement);
    return source;
}

void ExpectInvalid(const std::filesystem::path& scratch_path,
                   const std::string& valid_document,
                   const std::function<std::string(std::string)>& mutate,
                   std::string_view diagnostic_fragment) {
    Write(scratch_path, mutate(valid_document));
    try {
        static_cast<void>(
            LoadWheelDriveTorqueCommandConditionerFromJsonFile(scratch_path));
    } catch (const std::invalid_argument& error) {
        if (std::string_view(error.what()).find(diagnostic_fragment) ==
            std::string_view::npos) {
            std::fprintf(stderr,
                         "wheel-drive torque conditioner: expected strict "
                         "diagnostic containing '%.*s', got '%s'\n",
                         static_cast<int>(diagnostic_fragment.size()),
                         diagnostic_fragment.data(), error.what());
            ++failures;
        }
        return;
    }
    throw std::runtime_error("an invalid conditioner asset was accepted");
}

void CheckStrictAssetAndLoader(const std::filesystem::path& asset_path,
                               const std::filesystem::path& scratch_path) {
    const auto conditioner =
        LoadWheelDriveTorqueCommandConditionerFromJsonFile(asset_path);
    const auto& config = conditioner.config();
    Require(config.identifier ==
                "irw_reference_wheel_drive_torque_conditioner" &&
                config.sample_period_seconds == 0.01 &&
                config.drive_ratio == 53.0 / 19.0 &&
                config.drive_efficiency == 1.0 &&
                config.request_deadband_newton_metres == 0.5 &&
                config
                        .wheel_angular_speed_direction_threshold_radians_per_second ==
                    0.1 &&
                config.forward_wheel_angular_speed_sign == -1.0,
            "the installed IRW conditioner identity scalars drifted");
    Require(config.drive_speed_nodes_revolutions_per_minute.front() == 500.0 &&
                config.drive_speed_nodes_revolutions_per_minute.back() ==
                    5000.0 &&
                config.traction
                    .dynamic_limit_drive_side_newton_metres.front() ==
                    std::optional<double>(1500.0) &&
                !config.traction
                     .dynamic_limit_drive_side_newton_metres.back()
                     .has_value() &&
                config.regeneration
                        .dynamic_limit_drive_side_newton_metres.back() ==
                    std::optional<double>(100.0),
            "the active IRW conditioner tables drifted");

    const std::string valid_document = Read(asset_path);
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(
                std::move(source),
                "\"sample_period_seconds\": 0.01,",
                "\"sample_period_seconds\": 0.01,\n  "
                "\"sample_period_seconds\": 0.01,");
        },
        "duplicate JSON object key at $.sample_period_seconds");
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(std::move(source),
                               "\"drive_ratio\":",
                               "\"unused_switch\": false,\n  "
                               "\"drive_ratio\":");
        },
        "$.unused_switch");
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(std::move(source),
                               "\"sample_period_seconds\": 0.01,\n", "");
        },
        "$.sample_period_seconds");
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(std::move(source),
                               "\"drive_efficiency\": 1.0",
                               "\"drive_efficiency\": \"one\"");
        },
        "$.drive_efficiency");
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(
                std::move(source),
                "500.0,\n    1000.0,",
                "500.0,");
        },
        "must contain exactly 10 values");
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(
                std::move(source),
                "32638.773294118677",
                "null");
        },
        "must be a finite JSON number");
}

void CheckDirectConfigValidation() {
    auto config = MakeConfig();
    config.drive_speed_nodes_revolutions_per_minute[4] =
        config.drive_speed_nodes_revolutions_per_minute[3];
    try {
        static_cast<void>(WheelDriveTorqueCommandConditioner(config));
    } catch (const std::invalid_argument& error) {
        Require(std::string_view(error.what()).find("strictly increasing") !=
                    std::string_view::npos,
                "direct C++ validation reported the wrong invariant");
        return;
    }
    throw std::runtime_error("non-increasing direct C++ speed nodes accepted");
}

void CheckWarmStepDoesNotUseOrdinaryCppAllocation() {
    const WheelDriveTorqueCommandConditioner conditioner(MakeConfig());
    const WheelDriveTorqueChannelValues requested{
        20.0, -30.0, 40.0, -50.0, 60.0, -70.0, 80.0, -90.0};
    WheelDriveTorqueChannelValues speeds{};
    const WheelDriveTorqueChannelValues previous{};
    speeds.fill(WheelSpeedForDriveRpm(250.0));
    static_cast<void>(conditioner.Step(requested, speeds, previous));

    orvd::test::AllocationScope allocations;
    const auto result = conditioner.Step(requested, speeds, previous);
    Require(allocations.allocations() == 0,
            "a warmed valid Step used ordinary C++ operator new/new[]");
    Require(result.actual_wheel_torques_newton_metres[0] == 10.0,
            "the allocation probe did not consume the real result");
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 3) {
            throw std::invalid_argument(
                "expected the real conditioner asset and scratch paths");
        }
        CheckEightDistinctBranches();
        CheckDecreaseRateAndUnavailableEndpoint();
        CheckStrictAssetAndLoader(argv[1], argv[2]);
        CheckDirectConfigValidation();
        CheckWarmStepDoesNotUseOrdinaryCppAllocation();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "wheel-drive torque conditioner: %s\n",
                     error.what());
        return 1;
    }
    return failures == 0 ? 0 : 1;
}
