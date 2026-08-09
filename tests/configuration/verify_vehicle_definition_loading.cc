#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/configuration/load_vehicle_definition.h"

namespace {

using orvd::configuration::LoadVehicleDefinitionFromJsonFile;

// Three bodies, each exercising a different path: a free body with a non-zero
// centre of mass and a frame on it, a welded placeholder, and a body reached
// through a revolute joint. Nothing here is degenerate: every value a mutation
// below changes is one this record actually distinguishes.
constexpr std::string_view kRecord = R"json({
  "vehicle_name": "assembly_fixture",
  "mechanical_definition_identifier": "assembly_fixture_reference_mechanics",
  "mechanical_track_station_layout": {
    "reference_body_name": "carbody",
    "free_body_station_offsets": [
      {"station_offset_meters": 0.0, "body_name": "carbody"},
      {"station_offset_meters": 8.75,
       "body_name": "front_bogie_leading_wheelset"}
    ],
    "wheelset_body_names": ["front_bogie_leading_wheelset"]
  },
  "rigid_bodies": [
    {
      "name": "carbody",
      "moves_freely_in_world": true,
      "mass_kilograms": 30000.0,
      "center_of_mass_in_body_frame_meters": {"x": 0.0, "y": 0.0, "z": -1.25},
      "inertia_moments_about_center_of_mass_kilogram_square_meters":
        {"x": 60000.0, "y": 1300000.0, "z": 1330000.0},
      "inertia_products_about_center_of_mass_kilogram_square_meters":
        {"x": 110.0, "y": -220.0, "z": 330.0}
    },
    {
      "name": "front_carbody_interface_body",
      "moves_freely_in_world": false,
      "mass_kilograms": 1.0,
      "center_of_mass_in_body_frame_meters": {"x": 0.0, "y": 0.0, "z": 0.0},
      "inertia_moments_about_center_of_mass_kilogram_square_meters":
        {"x": 1.0, "y": 1.0, "z": 1.0},
      "inertia_products_about_center_of_mass_kilogram_square_meters":
        {"x": 0.0, "y": 0.0, "z": 0.0}
    },
    {
      "name": "front_bogie_leading_left_axlebox_carrier",
      "moves_freely_in_world": false,
      "mass_kilograms": 1.0,
      "center_of_mass_in_body_frame_meters": {"x": 0.0, "y": 0.0, "z": 0.0},
      "inertia_moments_about_center_of_mass_kilogram_square_meters":
        {"x": 1.0, "y": 1.0, "z": 1.0},
      "inertia_products_about_center_of_mass_kilogram_square_meters":
        {"x": 0.0, "y": 0.0, "z": 0.0}
    },
    {
      "name": "front_bogie_leading_wheelset",
      "moves_freely_in_world": true,
      "mass_kilograms": 1800.0,
      "center_of_mass_in_body_frame_meters": {"x": 0.0, "y": 0.0, "z": 0.0},
      "inertia_moments_about_center_of_mass_kilogram_square_meters":
        {"x": 1050.0, "y": 1640.0, "z": 1070.0},
      "inertia_products_about_center_of_mass_kilogram_square_meters":
        {"x": 0.0, "y": 0.0, "z": 0.0}
    }
  ],
  "fixed_frames": [
    {
      "name": "front_carbody_interface_mount",
      "body_name": "carbody",
      "position_in_body_frame_meters": {"x": 7.85, "y": 0.0, "z": -1.0},
      "rotation_in_body_frame": {"form": "aligned_with_body"}
    },
    {
      "name": "front_bogie_leading_left_axlebox_pivot",
      "body_name": "front_bogie_leading_wheelset",
      "position_in_body_frame_meters": {"x": 0.0, "y": -1.0, "z": 0.0},
      "rotation_in_body_frame": {"form": "aligned_with_body"}
    }
  ],
  "revolute_joints": [
    {
      "name": "front_bogie_leading_left_axlebox_pivot_joint",
      "parent_frame_name": "front_bogie_leading_left_axlebox_pivot",
      "child_frame_name": "front_bogie_leading_left_axlebox_carrier",
      "axis_in_parent_frame": {"x": 0.0, "y": 1.0, "z": 0.0},
      "damping_newton_metre_seconds_per_radian": 0.0
    }
  ],
  "weld_joints": [
    {
      "name": "front_carbody_interface_body_weld",
      "parent_frame_name": "front_carbody_interface_mount",
      "child_frame_name": "front_carbody_interface_body"
    }
  ],
  "translational_spring_dampers": [
    {
      "name": "front_left_air_spring",
      "reference_frame_name": "front_bogie_leading_left_axlebox_pivot",
      "opposite_frame_name": "carbody",
      "stiffness_newtons_per_meter": {"x": 185000.0, "y": 190000.0, "z": 230000.0},
      "damping_newton_seconds_per_meter": {"x": 0.0, "y": 0.0, "z": 20000.0}
    }
  ],
  "roll_spring_damper_couples": [
    {
      "name": "front_anti_roll_bar",
      "reference_frame_name": "front_bogie_leading_left_axlebox_pivot",
      "opposite_frame_name": "carbody",
      "stiffness_newton_meters_per_radian": 1500000.0,
      "damping_newton_meter_seconds_per_radian": 250.0
    }
  ],
  "series_spring_viscous_dampers": [
    {
      "name": "front_lateral_damper",
      "reference_frame_name": "front_bogie_leading_left_axlebox_pivot",
      "opposite_frame_name": "carbody",
      "axis": "lateral",
      "series_stiffness_newtons_per_meter": 1000000000.0,
      "series_damping_newton_seconds_per_meter": 60000.0
    }
  ],
  "saturated_piecewise_linear_dampers": [
    {
      "name": "front_left_yaw_damper",
      "reference_frame_name": "front_bogie_leading_left_axlebox_pivot",
      "opposite_frame_name": "carbody",
      "axis": "longitudinal",
      "curve": [
        {"relative_velocity_meters_per_second": 0.0, "force_newtons": 0.0},
        {"relative_velocity_meters_per_second": 0.02, "force_newtons": 12000.0},
        {"relative_velocity_meters_per_second": 0.04, "force_newtons": 12000.0}
      ]
    }
  ]
})json";

void Require(bool condition, std::string_view description) {
    if (!condition) {
        throw std::runtime_error(std::string(description));
    }
}

void Write(const std::filesystem::path& path, std::string_view contents) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output.write(contents.data(),
                 static_cast<std::streamsize>(contents.size()));
    output.close();
    Require(static_cast<bool>(output), "test configuration was not written");
}

std::string ReplaceOnce(std::string source, std::string_view needle,
                        std::string_view replacement) {
    const std::size_t position = source.find(needle);
    if (position == std::string::npos ||
        source.find(needle, position + 1) != std::string::npos) {
        throw std::runtime_error("test mutation did not identify one substring: " +
                                 std::string(needle));
    }
    source.replace(position, needle.size(), replacement);
    return source;
}

void ExpectInvalid(const std::filesystem::path& path, std::string contents,
                   const std::vector<std::string>& diagnostic_fragments) {
    Write(path, contents);
    try {
        static_cast<void>(LoadVehicleDefinitionFromJsonFile(path));
    } catch (const std::invalid_argument& error) {
        for (const std::string& fragment : diagnostic_fragments) {
            Require(std::string_view(error.what()).find(fragment) !=
                        std::string_view::npos,
                    "vehicle description was rejected at the wrong diagnostic "
                    "layer: " +
                        std::string(error.what()));
        }
        return;
    }
    throw std::runtime_error("invalid vehicle description was accepted");
}

void CheckRecordIsMappedAndOwned(const std::filesystem::path& path) {
    Write(path, kRecord);
    const auto vehicle = LoadVehicleDefinitionFromJsonFile(path);
    Require(std::filesystem::remove(path),
            "test configuration was not deleted after loading");

    Require(vehicle.vehicle_name == "assembly_fixture", "vehicle name");
    Require(vehicle.rigid_bodies.size() == 4 &&
                vehicle.fixed_frames.size() == 2 &&
                vehicle.revolute_joints.size() == 1 &&
                vehicle.weld_joints.size() == 1 &&
                vehicle.translational_spring_dampers.size() == 1 &&
                vehicle.roll_spring_damper_couples.size() == 1 &&
                vehicle.series_spring_viscous_dampers.size() == 1 &&
                vehicle.saturated_piecewise_linear_dampers.size() == 1,
            "record does not carry the entities it declares");

    // Each constitutive family maps its own fields, in its own order. The three
    // stiffness components differ so a transposed read fails.
    const auto& translational = vehicle.translational_spring_dampers.front();
    Require(translational.reference_frame_name ==
                    "front_bogie_leading_left_axlebox_pivot" &&
                translational.opposite_frame_name == "carbody",
            "a force element's two ends were exchanged");
    Require(translational.stiffness_newtons_per_meter ==
                    Eigen::Vector3d(185000.0, 190000.0, 230000.0) &&
                translational.damping_newton_seconds_per_meter ==
                    Eigen::Vector3d(0.0, 0.0, 20000.0),
            "translational constants were not mapped in order");
    Require(vehicle.roll_spring_damper_couples.front()
                        .stiffness_newton_meters_per_radian == 1500000.0 &&
                vehicle.roll_spring_damper_couples.front()
                        .damping_newton_meter_seconds_per_radian == 250.0,
            "roll couple constants were not mapped, or were exchanged");
    const auto& series = vehicle.series_spring_viscous_dampers.front();
    Require(series.axis == "lateral" &&
                series.series_stiffness_newtons_per_meter == 1.0e9 &&
                series.series_damping_newton_seconds_per_meter == 60000.0,
            "series constants were not mapped, or stiffness and damping were "
            "exchanged");
    const auto& clipped = vehicle.saturated_piecewise_linear_dampers.front();
    Require(clipped.axis == "longitudinal" && clipped.curve.size() == 3 &&
                clipped.curve[1].relative_velocity_meters_per_second == 0.02 &&
                clipped.curve[1].force_newtons == 12000.0,
            "the clipped damper's curve was not mapped in order");

    const auto& carbody = vehicle.rigid_bodies.front();
    Require(carbody.name == "carbody" && carbody.moves_freely_in_world,
            "free-body flag was not mapped");
    Require(carbody.mass_kilograms == 30000.0, "mass was not mapped");
    // Each component distinct, so a transposed or reordered read fails.
    Require(carbody.center_of_mass_in_body_frame_meters ==
                Eigen::Vector3d(0.0, 0.0, -1.25),
            "centre of mass was not mapped, or its sign was flipped");
    Require(carbody.inertia_moments_about_center_of_mass_kilogram_square_meters ==
                Eigen::Vector3d(60000.0, 1300000.0, 1330000.0),
            "inertia moments were not mapped in order");
    Require(
        carbody.inertia_products_about_center_of_mass_kilogram_square_meters ==
            Eigen::Vector3d(110.0, -220.0, 330.0),
        "inertia matrix off-diagonal entries were reordered or lost their signs");
    Require(!vehicle.rigid_bodies[1].moves_freely_in_world,
            "a welded body was read as free");

    Require(vehicle.fixed_frames.front().body_name == "carbody" &&
                vehicle.fixed_frames.front().position_in_body_frame_meters ==
                    Eigen::Vector3d(7.85, 0.0, -1.0),
            "weld mounting frame was not mapped");
    Require(vehicle.fixed_frames.front().rotation_in_body_frame ==
                Eigen::Matrix3d::Identity(),
            "frame rotation was not mapped");
    Require(vehicle.fixed_frames[1].position_in_body_frame_meters.y() == -1.0,
            "a lateral offset lost its sign");

    const auto& revolute = vehicle.revolute_joints.front();
    Require(revolute.parent_frame_name ==
                    "front_bogie_leading_left_axlebox_pivot" &&
                revolute.child_frame_name ==
                    "front_bogie_leading_left_axlebox_carrier",
            "revolute endpoints were not mapped, or were exchanged");
    Require(revolute.axis_in_parent_frame == Eigen::Vector3d(0.0, 1.0, 0.0),
            "revolute axis was not mapped");
    Require(revolute.damping_newton_metre_seconds_per_radian == 0.0,
            "revolute damping was not mapped");
    Require(vehicle.weld_joints.front().parent_frame_name ==
                    "front_carbody_interface_mount" &&
                vehicle.weld_joints.front().child_frame_name ==
                    "front_carbody_interface_body",
            "weld endpoints were not mapped, or were exchanged");
}

void CheckRejections(const std::filesystem::path& path) {
    const std::string valid(kRecord);

    struct RejectionCase {
        std::string contents;
        std::vector<std::string> fragments;
    };
    const std::vector<RejectionCase> cases{
        // One nested duplicate proves this loader uses the shared strict-JSON
        // boundary. Its generic syntax, number and NUL cases are already owned
        // by that boundary's track-configuration gate; repeating them here
        // would test the parser twice rather than the vehicle schema once.
        {ReplaceOnce(valid, "\"name\": \"carbody\",",
                     "\"name\": \"carbody\", \"name\": \"carbody\","),
         {"duplicate JSON object key at $.rigid_bodies[0].name"}},
        {ReplaceOnce(valid, "\"mass_kilograms\": 30000.0,",
                     "\"mass_kilograms\": 30000.0, \"colour\": \"red\","),
         {"$.rigid_bodies[0].colour", "unknown key"}},
        {ReplaceOnce(valid,
                     ",\n      \"damping_newton_metre_seconds_per_radian\": 0.0",
                     ""),
         {"$.revolute_joints[0].damping_newton_metre_seconds_per_radian",
          "required"}},
        {ReplaceOnce(valid, "\"mass_kilograms\": 30000.0",
                     "\"mass_kilograms\": \"heavy\""),
         {"$.rigid_bodies[0].mass_kilograms", "finite JSON number"}},
        {ReplaceOnce(valid, "\"vehicle_name\": \"assembly_fixture\"",
                     "\"vehicle_name\": \"\""),
         {"$.vehicle_name", "non-empty string"}},
        {ReplaceOnce(valid, "\"name\": \"front_left_air_spring\"",
                     "\"name\": \"\""),
         {"$.translational_spring_dampers[0].name",
          "non-empty force-element name"}},
        {ReplaceOnce(valid, "\"name\": \"front_anti_roll_bar\"",
                     "\"name\": \"front_left_air_spring\""),
         {"$.roll_spring_damper_couples[0].name",
          "front_left_air_spring", "first declared at",
          "$.translational_spring_dampers[0].name"}},
        // A massless body is a legitimate frame carrier to the multibody layer,
        // but a record states inertia about the centre of mass, and dividing it
        // by zero would surface as a complaint about the unit inertia rather
        // than about the mass the author wrote.
        {ReplaceOnce(valid, "\"mass_kilograms\": 30000.0",
                     "\"mass_kilograms\": 0.0"),
         {"$.rigid_bodies[0].mass_kilograms", "positive number of kilograms"}},
        {ReplaceOnce(valid,
                     "{\"x\": 60000.0, \"y\": 1300000.0, \"z\": 1330000.0}",
                     "{\"x\": 0.0, \"y\": 0.0, \"z\": 0.0}"),
         {"$.rigid_bodies[0]", "positive-definite",
          "centre-of-mass inertia"}},  // This fixture body moves freely.
        // Reference integrity, at the JSON path that names the absent entity.
        {ReplaceOnce(valid,
                     "\"parent_frame_name\": "
                     "\"front_bogie_leading_left_axlebox_pivot\",",
                     "\"parent_frame_name\": \"no_such_frame\","),
         {"$.revolute_joints[0].parent_frame_name", "no_such_frame",
          "never declares"}},
        {ReplaceOnce(valid,
                     "\"child_frame_name\": "
                     "\"front_carbody_interface_body\"",
                     "\"child_frame_name\": \"no_such_body\""),
         {"$.weld_joints[0].child_frame_name", "no_such_body"}},
        {ReplaceOnce(valid, "\"body_name\": \"carbody\",",
                     "\"body_name\": \"no_such_body\","),
         {"$.fixed_frames[0].body_name", "no_such_body"}},
        // The axis is a closed vocabulary, and a force element's ends must be
        // declared like any other reference.
        {ReplaceOnce(valid, "\"axis\": \"lateral\"", "\"axis\": \"sideways\""),
         {"$.series_spring_viscous_dampers[0].axis", "lateral"}},
        {ReplaceOnce(valid,
                     "\"reference_frame_name\": "
                     "\"front_bogie_leading_left_axlebox_pivot\",\n      "
                     "\"opposite_frame_name\": \"carbody\",\n      "
                     "\"stiffness_newtons_per_meter\"",
                     "\"reference_frame_name\": \"no_such_frame\",\n      "
                     "\"opposite_frame_name\": \"carbody\",\n      "
                     "\"stiffness_newtons_per_meter\""),
         {"$.translational_spring_dampers[0].reference_frame_name",
          "no_such_frame"}},
        {ReplaceOnce(valid, "\"force_newtons\": 12000.0}\n      ]",
                     "\"force_newtons\": 12000.0, \"slope\": 1.0}\n      ]"),
         {"$.saturated_piecewise_linear_dampers[0].curve[2].slope",
          "unknown key"}},
        // The rotation form is a closed vocabulary, not a free string.
        {ReplaceOnce(valid,
                     "\"rotation_in_body_frame\": {\"form\": "
                     "\"aligned_with_body\"}\n    },\n    {\n      \"name\": "
                     "\"front_bogie_leading_left_axlebox_pivot\"",
                     "\"rotation_in_body_frame\": {\"form\": \"identity\"}\n"
                     "    },\n    {\n      \"name\": "
                     "\"front_bogie_leading_left_axlebox_pivot\""),
         {"$.fixed_frames[0].rotation_in_body_frame.form",
          "aligned_with_body"}},
    };
    for (const RejectionCase& one : cases) {
        ExpectInvalid(path, one.contents, one.fragments);
    }

}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 2) {
            throw std::invalid_argument("expected a scratch directory");
        }
        const std::filesystem::path scratch = argv[1];
        std::filesystem::remove_all(scratch);
        std::filesystem::create_directories(scratch);
        const std::filesystem::path configuration =
            scratch / "vehicle_definition.json";

        CheckRecordIsMappedAndOwned(configuration);
        CheckRejections(configuration);
        std::filesystem::remove_all(scratch);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle definition loading failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
