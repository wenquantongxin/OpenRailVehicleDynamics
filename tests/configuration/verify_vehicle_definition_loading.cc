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
  "schema_version": 1,
  "vehicle_name": "assembly_fixture",
  "rigid_bodies": [
    {
      "name": "carbody",
      "moves_freely_in_world": true,
      "mass_kilograms": 30000.0,
      "center_of_mass_in_body_frame_meters": {"x": 0.0, "y": 0.0, "z": -1.25},
      "inertia_moments_about_center_of_mass_kilogram_square_meters":
        {"x": 60000.0, "y": 1300000.0, "z": 1330000.0},
      "inertia_products_about_center_of_mass_kilogram_square_meters":
        {"x": 0.0, "y": 0.0, "z": 0.0}
    },
    {
      "name": "carbody_secondary_suspension_seat_front",
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
      "name": "carbody_secondary_suspension_seat_front_weld_seat",
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
      "name": "carbody_secondary_suspension_seat_front_weld",
      "parent_frame_name": "carbody_secondary_suspension_seat_front_weld_seat",
      "child_frame_name": "carbody_secondary_suspension_seat_front"
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
                vehicle.weld_joints.size() == 1,
            "record does not carry the entities it declares");

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
                    "carbody_secondary_suspension_seat_front_weld_seat" &&
                vehicle.weld_joints.front().child_frame_name ==
                    "carbody_secondary_suspension_seat_front",
            "weld endpoints were not mapped, or were exchanged");
}

void CheckRejections(const std::filesystem::path& path) {
    const std::string valid(kRecord);

    struct RejectionCase {
        std::string contents;
        std::vector<std::string> fragments;
    };
    const std::vector<RejectionCase> cases{
        // The strict JSON rules, checked at the vehicle record's own paths
        // rather than assumed to carry over from the track record's gate.
        {ReplaceOnce(valid, "\"vehicle_name\": \"assembly_fixture\",",
                     "\"vehicle_name\": \"assembly_fixture\", "
                     "\"vehicle_name\": \"other\","),
         {"duplicate JSON object key at $.vehicle_name"}},
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
        {ReplaceOnce(valid,
                     "\"name\": \"carbody\",\n      "
                     "\"moves_freely_in_world\": true",
                     "\"name\": \"carbody\",\n      "
                     "\"moves_freely_in_world\": 1"),
         {"$.rigid_bodies[0].moves_freely_in_world", "JSON boolean"}},
        {ReplaceOnce(valid,
                     "\"center_of_mass_in_body_frame_meters\": "
                     "{\"x\": 0.0, \"y\": 0.0, \"z\": -1.25}",
                     "\"center_of_mass_in_body_frame_meters\": "
                     "[0.0, 0.0, -1.25]"),
         {"$.rigid_bodies[0].center_of_mass_in_body_frame_meters",
          "a JSON object"}},
        {ReplaceOnce(valid, "\"weld_joints\": [\n    {",
                     "\"weld_joints\": {\"0\": {"),
         {"invalid JSON syntax"}},
        {ReplaceOnce(valid, "\"schema_version\": 1,", "\"schema_version\": 2,"),
         {"$.schema_version", "the integer 1"}},
        {ReplaceOnce(valid, "\"schema_version\": 1,", "\"schema_version\": 1.0,"),
         {"$.schema_version", "the integer 1"}},
        {ReplaceOnce(valid, "\"mass_kilograms\": 30000.0",
                     "\"mass_kilograms\": 1e-400"),
         {"$.rigid_bodies[0].mass_kilograms", "underflows binary64"}},
        {ReplaceOnce(valid, "\"mass_kilograms\": 30000.0",
                     "\"mass_kilograms\": 9007199254740993"),
         {"$.rigid_bodies[0].mass_kilograms", "exactly representable"}},
        // A massless body is a legitimate frame carrier to the multibody layer,
        // but a record states inertia about the centre of mass, and dividing it
        // by zero would surface as a complaint about the unit inertia rather
        // than about the mass the author wrote.
        {ReplaceOnce(valid, "\"mass_kilograms\": 30000.0",
                     "\"mass_kilograms\": 0.0"),
         {"$.rigid_bodies[0].mass_kilograms", "positive number of kilograms"}},
        {ReplaceOnce(valid, "\"mass_kilograms\": 30000.0",
                     "\"mass_kilograms\": -30000.0"),
         {"$.rigid_bodies[0].mass_kilograms", "positive number of kilograms"}},
        // Reference integrity, at the JSON path that names the absent entity.
        {ReplaceOnce(valid,
                     "\"parent_frame_name\": "
                     "\"front_bogie_leading_left_axlebox_pivot\",",
                     "\"parent_frame_name\": \"no_such_frame\","),
         {"$.revolute_joints[0].parent_frame_name", "no_such_frame",
          "never declares"}},
        {ReplaceOnce(valid,
                     "\"child_frame_name\": "
                     "\"carbody_secondary_suspension_seat_front\"",
                     "\"child_frame_name\": \"no_such_body\""),
         {"$.weld_joints[0].child_frame_name", "no_such_body"}},
        {ReplaceOnce(valid, "\"body_name\": \"carbody\",",
                     "\"body_name\": \"no_such_body\","),
         {"$.fixed_frames[0].body_name", "no_such_body"}},
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

    std::string nul_terminated_prefix(valid);
    nul_terminated_prefix.push_back('\0');
    nul_terminated_prefix += valid;
    ExpectInvalid(path, std::move(nul_terminated_prefix),
                  {"NUL byte", "before the end of the file"});
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
