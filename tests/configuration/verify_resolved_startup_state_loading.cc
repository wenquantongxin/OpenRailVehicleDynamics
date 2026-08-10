// Strict loading of one small, non-degenerate resolved start-up state.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>

#include <Eigen/Core>

#include "orvd/configuration/load_resolved_startup_state.h"

namespace {

using orvd::configuration::LoadResolvedStartupStateFromJsonFile;
using orvd::configuration::ExplicitRevoluteJointRate;
using orvd::configuration::RevoluteJointRatePerCommonWheelSpin;
using orvd::configuration::StartupRunningDirection;

constexpr std::string_view kRecord = R"json({
  "vehicle_binding": {
    "vehicle_name": "startup_fixture",
    "mechanical_definition_identifier": "fixture_mechanics"
  },
  "wheel_rail_binding": {
    "wheel_profile_identifier": "wheel.prw",
    "rail_profile_identifier": "rail.prr",
    "wheel_rail_contact_strategy_identifier": "fixture_contact"
  },
  "load_condition_identifier": "fixture_load",
  "gravitational_acceleration_meters_per_second_squared": 9.7,
  "running_direction": "increasing_track_station",
  "initial_longitudinal_speed_meters_per_second": 10.0,
  "common_wheel_spin_generation": {
    "effective_rolling_radius_meters": 0.4
  },
  "rail_profile_reference_vertical_offset_meters": -0.0003,
  "wheel_pair_target_support_forces": [
    {
      "station_reference_body_name": "wheelset",
      "left_support_force_newtons": 123.0,
      "right_support_force_newtons": 124.0
    }
  ],
  "free_body_startup_states": [
    {
      "body_name": "wheelset",
      "rotation_local_track_from_body":
        {"w": 0.9689124217106447, "x": 0.24740395925452294, "y": 0.0, "z": 0.0},
      "resolved_track_station_offset_from_mechanical_layout_meters": 0.125,
      "lateral_offset_in_local_track_frame_meters": -0.004,
      "vertical_offset_in_local_track_frame_meters": -0.42,
      "explicit_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second":
        {"x": 0.01, "y": -0.02, "z": 0.03},
      "common_wheel_spin_coefficient_in_body_frame":
        {"x": 0.36, "y": -0.48, "z": 0.8},
      "body_origin_lateral_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second": 0.04,
      "body_origin_vertical_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second": -0.05
    }
  ],
  "revolute_joint_startup_states": [
    {
      "joint_name": "pivot",
      "position_radians": 0.007,
      "rate": {
        "kind": "per_common_wheel_spin_magnitude",
        "multiplier": 1.25
      }
    },
    {
      "joint_name": "wheel",
      "position_radians": -0.009,
      "rate": {
        "kind": "explicit_angular_rate",
        "angular_rate_radians_per_second": 31.25
      }
    }
  ],
  "ball_rpy_joint_startup_states": [
    {
      "joint_name": "ball",
      "roll_pitch_yaw_angles_radians": {"x": 0.11, "y": -0.22, "z": 0.33},
      "angular_velocity_of_child_in_parent_expressed_in_parent_frame_radians_per_second":
        {"x": -0.44, "y": 0.55, "z": -0.66}
    }
  ],
  "series_spring_viscous_damper_force_states": [
    {"element_name": "series", "reference_end_axial_force_newtons": -37.5}
  ],
  "translational_spring_damper_nominal_forces": [
    {
      "element_name": "spring",
      "force_on_reference_end_in_reference_frame_newtons":
        {"x": 11.0, "y": -22.0, "z": 33.0}
    }
  ]
})json";

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "resolved start-up state loading: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

std::string ReplaceOnce(std::string document, std::string_view needle,
                        std::string_view replacement) {
    const std::size_t position = document.find(needle);
    if (position == std::string::npos ||
        document.find(needle, position + needle.size()) != std::string::npos) {
        Require(false, "a test mutation is not unique");
        return document;
    }
    document.replace(position, needle.size(), replacement);
    return document;
}

void Write(const std::filesystem::path& path, const std::string& document) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output << document;
    Require(static_cast<bool>(output), "could not write the test record");
}

void ExpectRefusal(const std::filesystem::path& path,
                   const std::string& document, std::string_view fragment,
                   std::string_view what) {
    Write(path, document);
    try {
        (void)LoadResolvedStartupStateFromJsonFile(path);
    } catch (const std::exception& error) {
        if (std::string(error.what()).find(fragment) == std::string::npos) {
            std::fprintf(stderr,
                         "resolved start-up state loading: %.*s was refused "
                         "for another reason: %s\n",
                         static_cast<int>(what.size()), what.data(),
                         error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr,
                     "usage: verify_resolved_startup_state_loading "
                     "<scratch-dir>\n");
        return 2;
    }
    const std::filesystem::path scratch = argv[1];
    std::error_code ignored;
    std::filesystem::create_directories(scratch, ignored);
    const std::filesystem::path path = scratch / "resolved_startup_state.json";
    const std::string valid(kRecord);

    try {
        Write(path, valid);
        const auto state = LoadResolvedStartupStateFromJsonFile(path);
        Require(state.running_direction ==
                    StartupRunningDirection::kIncreasingTrackStation,
                "the running direction was not mapped");
        Require(state.initial_longitudinal_speed_meters_per_second == 10.0 &&
                    state.common_wheel_spin_generation.has_value() &&
                    state.common_wheel_spin_generation
                            ->effective_rolling_radius_meters == 0.4,
                "the two common start-up authorities were not mapped");
        Require(state.vehicle_binding.vehicle_name == "startup_fixture" &&
                    state.vehicle_binding.mechanical_definition_identifier ==
                        "fixture_mechanics" &&
                    state.wheel_rail_binding.wheel_profile_identifier ==
                        "wheel.prw" &&
                    state.wheel_rail_binding.rail_profile_identifier ==
                        "rail.prr" &&
                    state.wheel_rail_binding
                            .wheel_rail_contact_strategy_identifier ==
                        "fixture_contact" &&
                    state.load_condition_identifier == "fixture_load",
                "the start-up identity fields were not mapped");
        Require(state.gravitational_acceleration_meters_per_second_squared ==
                        9.7 &&
                    state.rail_profile_reference_vertical_offset_meters ==
                        -0.0003,
                "gravity or the rail-profile reference offset was not mapped");
        const auto& body = state.free_body_startup_states.front();
        Require(body.rotation_local_track_from_body.w() ==
                        0.9689124217106447 &&
                    body.rotation_local_track_from_body.x() ==
                        0.24740395925452294 &&
                    body.resolved_track_station_offset_from_mechanical_layout_meters ==
                        0.125 &&
                    body.lateral_offset_in_local_track_frame_meters == -0.004 &&
                    body.vertical_offset_in_local_track_frame_meters == -0.42,
                "the body pose and three placement offsets were not mapped");
        Require(body
                        .explicit_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second ==
                    Eigen::Vector3d(0.01, -0.02, 0.03) &&
                    body.common_wheel_spin_coefficient_in_body_frame ==
                        Eigen::Vector3d(0.36, -0.48, 0.8) &&
                    body.body_origin_lateral_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second ==
                        0.04 &&
                    body.body_origin_vertical_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second ==
                        -0.05,
                "the non-shared velocity components were not mapped");
        const auto& generated_joint =
            state.revolute_joint_startup_states.front();
        const auto& explicit_joint = state.revolute_joint_startup_states.back();
        Require(generated_joint.position_radians == 0.007 &&
                    std::holds_alternative<
                        RevoluteJointRatePerCommonWheelSpin>(
                        generated_joint.rate) &&
                    std::get<RevoluteJointRatePerCommonWheelSpin>(
                        generated_joint.rate)
                            .multiplier == 1.25 &&
                    explicit_joint.position_radians == -0.009 &&
                    std::holds_alternative<ExplicitRevoluteJointRate>(
                        explicit_joint.rate) &&
                    std::get<ExplicitRevoluteJointRate>(explicit_joint.rate)
                            .angular_rate_radians_per_second == 31.25,
                "the two typed joint-rate definitions were not mapped");
        const auto& ball = state.ball_rpy_joint_startup_states.front();
        Require(ball.roll_pitch_yaw_angles_radians ==
                        Eigen::Vector3d(0.11, -0.22, 0.33) &&
                    ball.angular_velocity_of_child_in_parent_expressed_in_parent_frame_radians_per_second ==
                        Eigen::Vector3d(-0.44, 0.55, -0.66),
                "the Ball-RPY position or physical angular velocity was not "
                "mapped");
        Require(state.series_spring_viscous_damper_force_states.front()
                            .reference_end_axial_force_newtons == -37.5 &&
                    state.translational_spring_damper_nominal_forces.front()
                            .force_on_reference_end_in_reference_frame_newtons ==
                        Eigen::Vector3d(11.0, -22.0, 33.0),
                "a named force state or nominal force was not mapped");
        Require(state.wheel_pair_target_support_forces.front()
                            .station_reference_body_name == "wheelset" &&
                    state.wheel_pair_target_support_forces.front()
                            .left_support_force_newtons == 123.0 &&
                    state.wheel_pair_target_support_forces.front()
                            .right_support_force_newtons == 124.0,
                "the wheel-pair station reference or support forces were not "
                "mapped");
    } catch (const std::exception& error) {
        std::fprintf(stderr,
                     "resolved start-up state loading failed on the valid "
                     "record: %s\n",
                     error.what());
        return 1;
    }

    ExpectRefusal(path,
                  ReplaceOnce(valid,
                              "  \"load_condition_identifier\": "
                              "\"fixture_load\",",
                              "  \"load_condition_identifier\": "
                              "\"fixture_load\",\n"
                              "  \"load_condition_identifier\": "
                              "\"fixture_load\","),
                  "duplicate JSON object key at $.load_condition_identifier",
                  "a duplicate JSON key was accepted");
    ExpectRefusal(
        path,
        ReplaceOnce(valid, "      \"position_radians\": 0.007,\n", ""),
        "position_radians is required", "a missing required key was accepted");
    ExpectRefusal(
        path,
        ReplaceOnce(valid,
                    "\"vertical_offset_in_local_track_frame_meters\": -0.42",
                    "\"vertical_offset_in_local_track_frame_meters\": \"low\""),
        "finite JSON number", "a value with the wrong JSON type was accepted");
    ExpectRefusal(path,
                  ReplaceOnce(valid, "\"increasing_track_station\"",
                              "\"decreasing_track_station\""),
                  "creep force near and across zero",
                  "reverse running was accepted");
    ExpectRefusal(path,
                  ReplaceOnce(valid,
                              "\"effective_rolling_radius_meters\": 0.4",
                              "\"effective_rolling_radius_meters\": 0.0"),
                  "strictly positive", "a zero effective radius was accepted");
    ExpectRefusal(path,
                  ReplaceOnce(
                      valid,
                      "\"common_wheel_spin_generation\": {\n"
                      "    \"effective_rolling_radius_meters\": 0.4\n"
                      "  }",
                      "\"common_wheel_spin_generation\": null"),
                  "no common wheel-spin generation",
                  "a generated body spin without its authority was accepted");
    ExpectRefusal(path,
                  ReplaceOnce(valid,
                              "\"per_common_wheel_spin_magnitude\"",
                              "\"wheel_rpm\""),
                  "expected 'explicit_angular_rate'",
                  "an unknown revolute-joint rate definition was accepted");
    ExpectRefusal(path,
                  ReplaceOnce(valid, "\"w\": 0.9689124217106447",
                              "\"w\": 0.5"),
                  "refused rather than normalised",
                  "a non-unit body quaternion was accepted");
    ExpectRefusal(
        path,
        ReplaceOnce(
            valid,
            "{\"element_name\": \"series\", \"reference_end_axial_force_newtons\": -37.5}",
            "{\"element_name\": \"series\", \"reference_end_axial_force_newtons\": -37.5},\n    "
            "{\"element_name\": \"series\", \"reference_end_axial_force_newtons\": 1.0}"),
        "more than once", "a repeated named series state was accepted");

    std::filesystem::remove(path, ignored);
    if (failures != 0) {
        return 1;
    }
    std::puts("resolved start-up state loading verified");
    return 0;
}
