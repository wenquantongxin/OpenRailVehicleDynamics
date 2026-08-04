// What the resolved start-up state loader refuses, and what it must not.
//
// argv[1] is a scratch directory. The record below is synthetic and small: one
// free body, one joint, one series element, one translational element and one
// wheelset. Nothing in it is degenerate — every value a mutation changes is one
// this record actually distinguishes.
//
// The mutations are the ones only this layer can name against a JSON path. How
// many free bodies a vehicle has, whether an identifier matches a vehicle
// record, and where a line begins to curve are all checked where both sides are
// in hand; testing them here would test a layer that cannot see them.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/configuration/load_resolved_startup_state.h"

namespace {

using orvd::configuration::LoadResolvedStartupStateFromJsonFile;
using orvd::configuration::StartupRunningDirection;

constexpr std::string_view kRecord = R"json({
  "schema_version": 1,
  "vehicle_binding": {
    "vehicle_name": "startup_fixture",
    "mechanical_definition_identifier": "startup_fixture_reference_mechanics"
  },
  "wheel_rail_binding": {
    "wheel_profile_identifier": "LM.prw",
    "rail_profile_identifier": "UIC60.prr",
    "wheel_rail_contact_strategy_identifier": "fixture_reference_contact"
  },
  "load_condition_identifier": "fixture_reference_load_condition",
  "gravitational_acceleration_meters_per_second_squared": 9.81,
  "running_direction": "increasing_track_station",
  "initial_longitudinal_speed_meters_per_second": 16.666666666666668,
  "rail_profile_reference_vertical_offset_meters": -0.0003055705255365684,
  "target_wheel_support_forces": [
    {
      "wheelset_body_name": "leading_wheelset",
      "left_support_force_newtons": 68295.99375,
      "right_support_force_newtons": 68290.5
    }
  ],
  "free_body_startup_states": [
    {
      "body_name": "leading_wheelset",
      "rotation_local_track_from_body":
        {"w": 0.9689124217106447, "x": 0.24740395925452294, "y": 0.0, "z": 0.0},
      "resolved_track_station_offset_from_mechanical_layout_meters": 0.125,
      "lateral_offset_in_local_track_frame_meters": -0.004,
      "vertical_offset_in_local_track_frame_meters": -0.42,
      "body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second":
        {"x": 0.01, "y": -39.659715290819015, "z": -0.02},
      "body_origin_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second":
        {"x": 16.666666666666668, "y": 0.03, "z": -0.05}
    }
  ],
  "revolute_joint_startup_states": [
    {
      "joint_name": "leading_left_axlebox_carrier_pivot",
      "position_radians": 0.007,
      "rate_radians_per_second": 39.659715290819015
    }
  ],
  "series_spring_viscous_damper_force_states": [
    {"element_name": "lateral_damper", "reference_end_axial_force_newtons": -37.5}
  ],
  "translational_spring_damper_nominal_forces": [
    {
      "element_name": "left_air_spring",
      "force_on_reference_end_in_reference_frame_newtons":
        {"x": 11.0, "y": -22.0, "z": 74926.3275}
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
    if (position == std::string::npos) {
        std::fprintf(stderr,
                     "resolved start-up state loading: mutation needle absent: "
                     "%.*s\n",
                     static_cast<int>(needle.size()), needle.data());
        ++failures;
        return document;
    }
    if (document.find(needle, position + needle.size()) != std::string::npos) {
        std::fprintf(stderr,
                     "resolved start-up state loading: mutation needle is not "
                     "unique: %.*s\n",
                     static_cast<int>(needle.size()), needle.data());
        ++failures;
        return document;
    }
    document.replace(position, needle.size(), replacement);
    return document;
}

void Write(const std::filesystem::path& path, const std::string& document) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output << document;
    if (!output) {
        std::fprintf(stderr,
                     "resolved start-up state loading: could not write the "
                     "fixture\n");
        ++failures;
    }
}

// A refusal must be the refusal that was asked for. Two fragments: where the
// diagnostic points, and what it says about the value.
void ExpectRefusal(const std::filesystem::path& path,
                   const std::string& document, std::string_view where,
                   std::string_view what) {
    Write(path, document);
    try {
        (void)LoadResolvedStartupStateFromJsonFile(path);
    } catch (const std::exception& error) {
        const std::string message = error.what();
        if (message.find(where) == std::string::npos ||
            message.find(what) == std::string::npos) {
            std::fprintf(stderr,
                         "resolved start-up state loading: refused for another "
                         "reason than '%.*s ... %.*s': %s\n",
                         static_cast<int>(where.size()), where.data(),
                         static_cast<int>(what.size()), what.data(),
                         message.c_str());
            ++failures;
        }
        return;
    }
    std::fprintf(stderr,
                 "resolved start-up state loading: accepted a record that "
                 "should have been refused at %.*s\n",
                 static_cast<int>(where.size()), where.data());
    ++failures;
}

void ExpectAccepted(const std::filesystem::path& path,
                    const std::string& document, std::string_view what) {
    Write(path, document);
    try {
        (void)LoadResolvedStartupStateFromJsonFile(path);
    } catch (const std::exception& error) {
        std::fprintf(stderr,
                     "resolved start-up state loading: refused %.*s: %s\n",
                     static_cast<int>(what.size()), what.data(), error.what());
        ++failures;
    }
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

        // Every field reached its own place. Each value below is distinct, so a
        // transposed or reordered read fails.
        Require(state.vehicle_binding.vehicle_name == "startup_fixture" &&
                    state.vehicle_binding.mechanical_definition_identifier ==
                        "startup_fixture_reference_mechanics",
                "the vehicle binding was not mapped");
        Require(state.wheel_rail_binding.wheel_profile_identifier == "LM.prw" &&
                    state.wheel_rail_binding.rail_profile_identifier ==
                        "UIC60.prr" &&
                    state.wheel_rail_binding
                            .wheel_rail_contact_strategy_identifier ==
                        "fixture_reference_contact",
                "the wheel-rail binding was not mapped, or two of its three "
                "identifiers were exchanged");
        Require(state.gravitational_acceleration_meters_per_second_squared ==
                    9.81,
                "the resolved gravity was not mapped");
        Require(state.running_direction ==
                    StartupRunningDirection::kIncreasingTrackStation,
                "the running direction was not mapped");
        Require(state.initial_longitudinal_speed_meters_per_second ==
                    16.666666666666668,
                "the initial speed was not mapped");
        Require(state.rail_profile_reference_vertical_offset_meters ==
                    -0.0003055705255365684,
                "the rail profile reference vertical offset was not mapped, or "
                "its sign was repaired");

        const auto& body = state.free_body_startup_states.front();
        Require(body.rotation_local_track_from_body.w() == 0.9689124217106447 &&
                    body.rotation_local_track_from_body.x() ==
                        0.24740395925452294,
                "the quaternion was not read in w, x, y, z order");
        Require(
            body.resolved_track_station_offset_from_mechanical_layout_meters ==
                    0.125 &&
                body.lateral_offset_in_local_track_frame_meters == -0.004 &&
                body.vertical_offset_in_local_track_frame_meters == -0.42,
            "the three place offsets were not mapped in order");
        Require(
            body
                    .body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second ==
                Eigen::Vector3d(0.01, -39.659715290819015, -0.02),
            "the angular velocity was not mapped, or its sign was repaired");
        Require(
            body
                    .body_origin_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second ==
                Eigen::Vector3d(16.666666666666668, 0.03, -0.05),
            "the origin velocity was not mapped");

        const auto& joint = state.revolute_joint_startup_states.front();
        Require(joint.position_radians == 0.007 &&
                    joint.rate_radians_per_second == 39.659715290819015,
                "a joint's position and rate were not mapped, or exchanged");
        Require(state.series_spring_viscous_damper_force_states.front()
                        .reference_end_axial_force_newtons == -37.5,
                "a series force state was not mapped, or its sign was "
                "repaired");
        Require(state.translational_spring_damper_nominal_forces.front()
                        .force_on_reference_end_in_reference_frame_newtons ==
                    Eigen::Vector3d(11.0, -22.0, 74926.3275),
                "a nominal force was not mapped in order");
        const auto& target = state.target_wheel_support_forces.front();
        Require(target.left_support_force_newtons == 68295.99375 &&
                    target.right_support_force_newtons == 68290.5,
                "the two target wheel loads were exchanged");
    } catch (const std::exception& error) {
        std::fprintf(stderr,
                     "resolved start-up state loading failed on the valid "
                     "record: %s\n",
                     error.what());
        return 1;
    }

    // --- The closed vocabularies and the version --------------------------
    ExpectRefusal(path, ReplaceOnce(valid, "\"schema_version\": 1", "\"schema_version\": 2"),
                  "$.schema_version", "the integer 1");
    ExpectRefusal(path,
                  ReplaceOnce(valid, "\"increasing_track_station\"",
                              "\"decreasing_track_station\""),
                  "$.running_direction", "creep force near and across zero");

    // --- Identifiers are names, not paths or sentences ---------------------
    ExpectRefusal(path,
                  ReplaceOnce(valid, "\"fixture_reference_contact\"",
                              "\"../fixture_reference_contact\""),
                  "wheel_rail_contact_strategy_identifier", "not a path");
    ExpectRefusal(path,
                  ReplaceOnce(valid, "\"fixture_reference_load_condition\"",
                              "\"fixture reference load condition\""),
                  "load_condition_identifier", "[A-Za-z0-9._-]");
    ExpectRefusal(path,
                  ReplaceOnce(valid, "\"startup_fixture_reference_mechanics\"",
                              "\"\""),
                  "mechanical_definition_identifier", "non-empty");

    // --- The record's own numeric invariants --------------------------------
    ExpectRefusal(
        path,
        ReplaceOnce(valid,
                    "\"gravitational_acceleration_meters_per_second_squared\": "
                    "9.81",
                    "\"gravitational_acceleration_meters_per_second_squared\": "
                    "0.0"),
        "gravitational_acceleration", "strictly positive");
    ExpectRefusal(
        path,
        ReplaceOnce(valid,
                    "\"initial_longitudinal_speed_meters_per_second\": "
                    "16.666666666666668,",
                    "\"initial_longitudinal_speed_meters_per_second\": "
                    "-16.666666666666668,"),
        "initial_longitudinal_speed", "stated by running_direction");
    ExpectRefusal(path,
                  ReplaceOnce(valid, "\"left_support_force_newtons\": 68295.99375",
                              "\"left_support_force_newtons\": -68295.99375"),
                  "target_wheel_support_forces", "compressive support");
    ExpectRefusal(
        path,
        ReplaceOnce(valid, "\"w\": 0.9689124217106447", "\"w\": 0.5"),
        "rotation_local_track_from_body", "refused rather than normalised");

    // --- Shape: keys, types, and what a strict record refuses ----------------
    ExpectRefusal(
        path,
        ReplaceOnce(valid, "\"rail_profile_reference_vertical_offset_meters\"",
                    "\"rail_profile_reference_vertical_offset_m\""),
        "rail_profile_reference_vertical_offset_m", "unknown key");
    ExpectRefusal(path, ReplaceOnce(valid, "\"position_radians\": 0.007,", ""),
                  "position_radians", "required");
    ExpectRefusal(path,
                  ReplaceOnce(valid, "\"vertical_offset_in_local_track_frame_meters\": -0.42",
                              "\"vertical_offset_in_local_track_frame_meters\": \"-0.42\""),
                  "vertical_offset_in_local_track_frame_meters",
                  "finite JSON number");
    ExpectRefusal(path,
                  ReplaceOnce(valid, "\"reference_end_axial_force_newtons\": -37.5",
                              "\"reference_end_axial_force_newtons\": 1e400"),
                  "reference_end_axial_force_newtons", "binary64");

    // --- A record may not name one entity twice ------------------------------
    ExpectRefusal(
        path,
        ReplaceOnce(
            valid,
            "{\"element_name\": \"lateral_damper\", \"reference_end_axial_force_newtons\": -37.5}",
            "{\"element_name\": \"lateral_damper\", \"reference_end_axial_force_newtons\": -37.5},\n    "
            "{\"element_name\": \"lateral_damper\", \"reference_end_axial_force_newtons\": 1.0}"),
        "series_spring_viscous_damper_force_states", "more than once");

    // --- What must not be refused -------------------------------------------
    //
    // A resolved spin that happens to equal the speed over some nominal radius
    // is a legal record. Nothing here recomputes it, so nothing here may
    // object to it agreeing.
    ExpectAccepted(path,
                   ReplaceOnce(valid, "\"y\": -39.659715290819015",
                               "\"y\": -39.68253968253968"),
                   "a spin that equals the speed over a nominal radius");
    // A zero preload is a resolved value like any other: the axlebox bushings
    // of a real vehicle carry none.
    ExpectAccepted(path,
                   ReplaceOnce(valid, "\"z\": 74926.3275", "\"z\": 0.0"),
                   "a zero nominal force");
    // The offset is signed, and negative is the physical case.
    ExpectAccepted(
        path,
        ReplaceOnce(valid, "\"rail_profile_reference_vertical_offset_meters\": -0.0003055705255365684",
                    "\"rail_profile_reference_vertical_offset_meters\": 0.0003055705255365684"),
        "a positive rail profile reference vertical offset");

    std::filesystem::remove(path, ignored);
    if (failures != 0) {
        return 1;
    }
    std::puts("resolved start-up state loading verified");
    return 0;
}
