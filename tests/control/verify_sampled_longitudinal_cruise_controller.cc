#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "orvd/configuration/load_irw_longitudinal_cruise_controller.h"
#include "orvd/control/sampled_longitudinal_cruise_controller.h"

namespace {

using orvd::configuration::
    LoadIrwLongitudinalCruiseControllerFromJsonFile;
using orvd::control::SampledLongitudinalCruiseController;
using orvd::control::SampledLongitudinalCruiseControllerConfig;
using orvd::control::SampledLongitudinalCruiseControllerState;

int failures = 0;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        std::fprintf(stderr, "longitudinal cruise controller: %.*s\n",
                     static_cast<int>(message.size()), message.data());
        ++failures;
    }
}

void RequireNear(double measured, double expected, double tolerance,
                 std::string_view message) {
    Require(std::abs(measured - expected) <= tolerance, message);
}

template <class Function>
bool Throws(Function&& function) {
    try {
        function();
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

SampledLongitudinalCruiseControllerConfig MakeConfig() {
    return SampledLongitudinalCruiseControllerConfig{
        .identifier = "test_longitudinal_cruise",
        .sample_period_seconds = 0.1,
        .target_speed_meters_per_second = 10.0,
        .speed_pi =
            {
                .proportional_gain = 100.0,
                .integral_time_seconds = 2.0,
                .output_filter_time_constant_seconds = 0.0,
                .integral_absolute_limit = 1.0,
                .raw_output_absolute_limit = 1000.0,
            },
    };
}

void CheckScalarCruiseCalculation() {
    const SampledLongitudinalCruiseController controller(MakeConfig());
    const SampledLongitudinalCruiseControllerState previous;
    const auto first = controller.Step(9.0, previous);
    RequireNear(first.speed_error_meters_per_second, 1.0, 0.0,
                "the target-minus-measurement speed error changed");
    RequireNear(first.next_state.speed_pi.integral, 0.1, 0.0,
                "the sole cruise integral was not updated once");
    RequireNear(first.requested_common_wheel_torque_newton_metres, 105.0,
                1.0e-14,
                "the common torque did not use the updated PI integral");
    Require(previous.speed_pi.integral == 0.0 &&
                previous.speed_pi.filtered_output == 0.0,
            "Step modified caller-owned state");

    const auto second = controller.Step(10.0, first.next_state);
    RequireNear(second.speed_error_meters_per_second, 0.0, 0.0,
                "zero speed error was not preserved");
    RequireNear(second.next_state.speed_pi.integral, 0.1, 0.0,
                "zero error changed the accepted integral");
    RequireNear(second.requested_common_wheel_torque_newton_metres, 5.0,
                1.0e-14,
                "the committed integral did not retain its torque request");

    Require(Throws([&] {
                (void)controller.Step(
                    std::numeric_limits<double>::quiet_NaN(), previous);
            }),
            "a non-finite measured speed was accepted");
    auto invalid = MakeConfig();
    invalid.target_speed_meters_per_second = 0.0;
    Require(Throws([&] {
                (void)SampledLongitudinalCruiseController(invalid);
            }),
            "a nonpositive target speed was accepted");
}

std::string Read(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open the cruise controller asset");
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

void Write(const std::filesystem::path& path, std::string_view contents) {
    std::ofstream output(path, std::ios::binary);
    if (!output || !(output << contents)) {
        throw std::runtime_error("could not write a cruise test fixture");
    }
}

std::string ReplaceOnce(std::string source, std::string_view needle,
                        std::string_view replacement) {
    const std::size_t position = source.find(needle);
    if (position == std::string::npos ||
        source.find(needle, position + needle.size()) != std::string::npos) {
        throw std::runtime_error(
            "cruise test mutation did not identify one substring");
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
        (void)LoadIrwLongitudinalCruiseControllerFromJsonFile(scratch_path);
    } catch (const std::invalid_argument& error) {
        if (std::string_view(error.what()).find(diagnostic_fragment) ==
            std::string_view::npos) {
            std::fprintf(stderr,
                         "longitudinal cruise controller: expected diagnostic "
                         "containing '%.*s', got '%s'\n",
                         static_cast<int>(diagnostic_fragment.size()),
                         diagnostic_fragment.data(), error.what());
            ++failures;
        }
        return;
    }
    throw std::runtime_error("an invalid cruise controller asset was accepted");
}

void CheckStrictAsset(const std::filesystem::path& asset_path,
                      const std::filesystem::path& scratch_path) {
    const auto asset =
        LoadIrwLongitudinalCruiseControllerFromJsonFile(asset_path);
    const auto& config = asset.controller.config();
    Require(config.identifier ==
                    "irw_v60_common_wheel_speed_cruise_controller" &&
                config.sample_period_seconds == 0.01 &&
                config.target_speed_meters_per_second ==
                    16.666666666666668 &&
                asset.nominal_rolling_radius_meters == 0.43,
            "the installed cruise identity or physical scalars drifted");
    Require(config.speed_pi.proportional_gain == 20000.0 &&
                config.speed_pi.integral_time_seconds == 10.0 &&
                config.speed_pi.output_filter_time_constant_seconds == 0.0 &&
                config.speed_pi.integral_absolute_limit == 2.0 &&
                config.speed_pi.raw_output_absolute_limit == 5000.0,
            "the installed common PI parameters drifted");
    for (const double sign : asset.forward_joint_rate_signs) {
        Require(sign == 1.0,
                "an installed forward joint-rate sign is not positive");
    }

    const std::string valid_document = Read(asset_path);
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(
                std::move(source), "\"sample_period_seconds\": 0.01,",
                "\"sample_period_seconds\": 0.01,\n  "
                "\"sample_period_seconds\": 0.01,");
        },
        "duplicate JSON object key at $.sample_period_seconds");
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(
                std::move(source), "\"speed_pi\": {",
                "\"legacy_mode\": false,\n  \"speed_pi\": {");
        },
        "$.legacy_mode");
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(std::move(source),
                               "\"nominal_rolling_radius_meters\": 0.43",
                               "\"nominal_rolling_radius_meters\": 0.0");
        },
        "$.nominal_rolling_radius_meters must be positive");
    ExpectInvalid(
        scratch_path, valid_document,
        [](std::string source) {
            return ReplaceOnce(
                std::move(source),
                "\"forward_joint_rate_signs\": [1.0, 1.0, 1.0, 1.0, "
                "1.0, 1.0, 1.0, 1.0]",
                "\"forward_joint_rate_signs\": [1.0, 1.0, 1.0, 1.0, "
                "1.0, 1.0, 0.0, 1.0]");
        },
        "$.forward_joint_rate_signs[6] must be -1 or +1");
    std::filesystem::remove(scratch_path);
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 3) {
            throw std::invalid_argument(
                "expected the real cruise controller asset and scratch path");
        }
        CheckScalarCruiseCalculation();
        CheckStrictAsset(argv[1], argv[2]);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "longitudinal cruise controller: %s\n",
                     error.what());
        return 1;
    }
    return failures == 0 ? 0 : 1;
}
