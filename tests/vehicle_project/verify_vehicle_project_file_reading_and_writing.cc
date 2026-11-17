#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <ios>
#include <iterator>
#include <limits>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_vehicle_contact_record.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/vehicle_project/make_vehicle_project_from_vehicle_definition.h"
#include "orvd/vehicle_project/read_visual_definition_document_from_file.h"
#include "orvd/vehicle_project/vehicle_project.h"
#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "orvd/vehicle_project/vehicle_project_file_reading_and_writing.h"
#include "orvd/vehicle_project/vehicle_project_files_in_directory.h"
#include "orvd/vehicle_project/vehicle_project_free_body_placements.h"
#include "vehicle_project_comparison.h"

// Vehicle projects saved as files of a directory of projects. Both bundled
// vehicles, opened as the modelling service opens them, and unfinished
// projects are written and read back as the same project, the visual
// definition equal as a JSON value, and writing what was read gives the same
// bytes. Reading refuses what is wrong with the document by naming its place,
// the key a station layout no longer has included; a project name that is not
// one is refused; a write that fails leaves the directory as it was; and only
// project files are listed.

namespace {

using namespace orvd;
using Json = nlohmann::json;
using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;
using vehicle_project::VehicleProjectFileLocation;
using vehicle_project_comparison::SameProject;

[[noreturn]] void Fail(const std::string& detail) {
    throw std::runtime_error(detail);
}

void Require(bool condition, const std::string& detail) {
    if (!condition) {
        Fail(detail);
    }
}

// `action` must throw `Refusal` with a message containing each of `places`.
template <typename Refusal, typename Action>
void RequireRefusal(Action action, std::initializer_list<std::string_view> places,
                    const std::string& what) {
    try {
        action();
    } catch (const Refusal& error) {
        for (const std::string_view place : places) {
            Require(
                std::string_view(error.what()).find(place) != std::string_view::npos,
                what + " was refused without naming " + std::string(place) + ": " + error.what());
        }
        return;
    }
    Fail(what + " was accepted");
}

std::string ReadBytes(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    Require(stream.is_open(), "'" + path.string() + "' cannot be opened");
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void WriteBytes(const std::filesystem::path& path, const std::string& bytes) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    stream << bytes;
    stream.close();
    Require(!stream.fail(), "'" + path.string() + "' cannot be written");
}

// The names of everything in `directory`, sorted.
std::vector<std::string> EntriesOf(const std::filesystem::path& directory) {
    std::vector<std::string> entries;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(directory)) {
        entries.push_back(entry.path().filename().string());
    }
    std::ranges::sort(entries);
    return entries;
}

// An empty directory of projects under the work directory, made afresh.
std::filesystem::path FreshDirectory(const std::filesystem::path& work_directory,
                                     const std::string& name) {
    const std::filesystem::path directory = work_directory / name;
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    return directory;
}

struct ReferenceVehicleFiles {
    std::filesystem::path vehicle_definition;
    std::filesystem::path startup_state;
    std::filesystem::path contact_record;
    std::filesystem::path visual_definition;
};

// The project the modelling service opens from a vehicle's own files: the
// definition, every free body placed at its start-up pose, the start-up
// state's gravity, the contact record and the visual definition.
VehicleProject OpenAsTheModelingServiceDoes(const ReferenceVehicleFiles& files) {
    VehicleProject project = vehicle_project::MakeVehicleProject(
        configuration::LoadVehicleDefinitionFromJsonFile(files.vehicle_definition));
    const configuration::ResolvedStartupState startup_state =
        configuration::LoadResolvedStartupStateFromJsonFile(files.startup_state);
    vehicle_project::PlaceFreeBodiesAtStartupPoses(project, startup_state);
    project.gravitational_acceleration_meters_per_second_squared =
        startup_state.gravitational_acceleration_meters_per_second_squared;
    project.contact_record =
        configuration::LoadVehicleContactRecordFromJsonFile(files.contact_record);
    project.visual_definition_document =
        vehicle_project::ReadVisualDefinitionDocumentFromFile(files.visual_definition);
    return project;
}

bool SameVisualDefinition(const std::string& expected, const std::string& actual) {
    if (expected.empty() || actual.empty()) {
        return expected.empty() && actual.empty();
    }
    return Json::parse(expected) == Json::parse(actual);
}

// Writes `project` at `location` and reads it back: the same project, its
// visual definition equal as a JSON value. Writing what was read gives the
// same bytes again. Returns what was read.
VehicleProject RequireReadBackAsWritten(const VehicleProject& project,
                                        const VehicleProjectFileLocation& location,
                                        const std::string& what) {
    vehicle_project::WriteVehicleProjectFile(project, location);
    const std::filesystem::path path = vehicle_project::VehicleProjectFilePath(location);
    const std::string written = ReadBytes(path);
    const VehicleProject read = vehicle_project::ReadVehicleProjectFile(location);
    Require(
        SameVisualDefinition(project.visual_definition_document, read.visual_definition_document),
        what + ": the visual definition read back is not the one written");
    VehicleProject read_with_written_visual_definition_text = read;
    read_with_written_visual_definition_text.visual_definition_document =
        project.visual_definition_document;
    Require(SameProject(project, read_with_written_visual_definition_text),
            what + ": the project read back is not the one written");
    vehicle_project::WriteVehicleProjectFile(read, location);
    Require(ReadBytes(path) == written,
            what + ": writing the project read back did not give the same bytes again");
    return read;
}

// The file holds one JSON document with exactly the project file's keys, in
// two-space indentation and followed by one newline, and writes each placement
// under the keys a start-up state writes a free body's pose with.
void RequireProjectFileLayout(const std::filesystem::path& project_path,
                              const std::filesystem::path& startup_state_path,
                              const std::string& what) {
    const std::string text = ReadBytes(project_path);
    Require(text.ends_with("}\n") && !text.ends_with("\n\n"),
            what + ": the file does not end in exactly one newline after its document");
    const Json document = Json::parse(text);
    std::vector<std::string> keys;
    for (const auto& [key, value] : document.items()) {
        keys.push_back(key);
    }
    std::ranges::sort(keys);
    const std::vector<std::string> expected_keys = {
        "contact_record",
        "element_groups",
        "free_body_placements",
        "gravitational_acceleration_meters_per_second_squared",
        "vehicle_definition",
        "visual_definition",
        "visual_definition_body_names"};
    Require(keys == expected_keys, what + ": the document does not hold exactly the project keys");
    for (const std::string& key : expected_keys) {
        Require(text.find("\n  \"" + key + "\": ") != std::string::npos,
                what + ": '" + key + "' is not written at two-space indentation");
    }

    const Json startup_state = Json::parse(ReadBytes(startup_state_path));
    const Json& startup_poses = startup_state.at("free_body_startup_states");
    const std::vector<std::string> placement_keys = {
        "body_name", "lateral_offset_in_local_track_frame_meters", "rotation_local_track_from_body",
        "vertical_offset_in_local_track_frame_meters"};
    Require(!document.at("free_body_placements").empty(), what + ": no placement was written");
    for (const Json& placement : document.at("free_body_placements")) {
        const auto startup_pose = std::find_if(
            startup_poses.begin(), startup_poses.end(),
            [&](const Json& pose) { return pose.at("body_name") == placement.at("body_name"); });
        Require(startup_pose != startup_poses.end(),
                what + ": a placement names a body the start-up state does not place");
        std::vector<std::string> written_keys;
        for (const auto& [key, value] : placement.items()) {
            written_keys.push_back(key);
        }
        Require(written_keys == placement_keys &&
                    std::ranges::all_of(
                        placement_keys,
                        [&](const std::string& key) { return startup_pose->contains(key); }),
                what + ": a placement is not written under the keys of a start-up pose");
        const Json& rotation = placement.at("rotation_local_track_from_body");
        const Json& startup_rotation = startup_pose->at("rotation_local_track_from_body");
        Require(rotation.size() == startup_rotation.size() &&
                    std::ranges::all_of(rotation.items(),
                                        [&](const auto& component) {
                                            return startup_rotation.contains(component.key());
                                        }),
                what + ": a placement's rotation is not written as a start-up pose's");
    }
}

void VerifyReferenceVehicleReadBackAsWritten(const VehicleProject& opened,
                                             const ReferenceVehicleFiles& files,
                                             const std::filesystem::path& directory) {
    const std::string& vehicle_name = opened.vehicle_definition.vehicle_name;
    Require(opened.gravitational_acceleration_meters_per_second_squared.has_value() &&
                opened.contact_record.has_value() && !opened.free_body_placements.empty(),
            vehicle_name + " did not open with its gravity, contact record and placements");
    // The visual definition a project takes in is the file's JSON object.
    Require(Json::parse(opened.visual_definition_document) ==
                Json::parse(ReadBytes(files.visual_definition)),
            vehicle_name + ": the visual definition opened is not the object its file holds");

    const VehicleProjectFileLocation location{directory, "reference_" + vehicle_name};
    const VehicleProject read =
        RequireReadBackAsWritten(opened, location, vehicle_name + " opened from its files");
    // The visual definition a project file gives back is the text reading
    // its file gives.
    Require(read.visual_definition_document == opened.visual_definition_document,
            vehicle_name +
                ": the visual definition text differs between opening its file and "
                "reading the project file");
    RequireProjectFileLayout(vehicle_project::VehicleProjectFilePath(location), files.startup_state,
                             vehicle_name);
}

// Every kind word, in nested groups that also hold an empty one.
std::vector<vehicle_project::ElementGroup> NestedGroupsOfEveryKind() {
    std::vector<ElementReference> members;
    for (const ElementKind kind : vehicle_project::kElementKinds) {
        members.push_back({kind, std::string(vehicle_project::ElementKindWord(kind)) + "_member"});
    }
    const auto members_from = [&members](std::size_t first, std::size_t end) {
        return std::vector<ElementReference>(members.begin() + first, members.begin() + end);
    };
    const std::size_t third = members.size() / 3;
    return {{.name = "running_gear",
             .members = members_from(0, third),
             .subgroups = {{.name = "suspension",
                            .members = members_from(third, 2 * third),
                            .subgroups = {{.name = "contact",
                                           .members = members_from(2 * third, members.size())}}},
                           {.name = "still_empty"}}},
            {.name = "body_shell"}};
}

// Unfinished projects keep what they hold: GZ18 with a body of zero mass, a
// damper without an axis, a damper with an empty curve, a spring of zero
// nominal length, a free body without a placement, nested groups, no contact
// record and no visual definition; IRW with one interface's wheel joint
// reference null and another's an empty string.
void VerifyUnfinishedProjectsReadBackAsWritten(const VehicleProject& gz18,
                                               const VehicleProject& irw,
                                               const std::filesystem::path& directory) {
    VehicleProject unfinished_gz18 = gz18;
    configuration::VehicleDefinition& vehicle = unfinished_gz18.vehicle_definition;
    vehicle.rigid_bodies.push_back({.name = "body_of_zero_mass"});
    configuration::VehicleSeriesSpringViscousDamperDefinition damper_without_axis;
    damper_without_axis.name = "damper_without_axis";
    vehicle.series_spring_viscous_dampers.push_back(damper_without_axis);
    configuration::VehicleSaturatedPiecewiseLinearDamperDefinition damper_with_empty_curve;
    damper_with_empty_curve.name = "damper_with_empty_curve";
    damper_with_empty_curve.axis = "lateral";
    vehicle.saturated_piecewise_linear_dampers.push_back(damper_with_empty_curve);
    vehicle.point_to_point_linear_spring_dampers.push_back(
        {.name = "spring_of_zero_nominal_length",
         .reference_frame_name = vehicle.rigid_bodies.front().name,
         .opposite_frame_name = "body_of_zero_mass",
         .stiffness_newtons_per_meter = 1.0e5});
    const std::string unplaced_body_name = unfinished_gz18.free_body_placements.front().body_name;
    unfinished_gz18.free_body_placements.erase(unfinished_gz18.free_body_placements.begin());
    unfinished_gz18.element_groups = NestedGroupsOfEveryKind();
    unfinished_gz18.contact_record.reset();
    unfinished_gz18.visual_definition_document.clear();
    const VehicleProject read_gz18 = RequireReadBackAsWritten(
        unfinished_gz18, {directory, "unfinished_gz18"}, "the unfinished GZ18 project");
    Require(vehicle_project::CollectUnplacedFreeBodyNames(read_gz18) ==
                std::vector<std::string>{unplaced_body_name},
            "the unfinished GZ18 project read back does not miss exactly the placement left out");

    VehicleProject unfinished_irw = irw;
    auto& interfaces = unfinished_irw.contact_record->declaration.interfaces;
    Require(interfaces.size() >= 2, "IRW has fewer than two contact interfaces");
    interfaces[0].independent_wheel_revolute_joint_name.reset();
    interfaces[1].independent_wheel_revolute_joint_name = std::string();
    const VehicleProject read_irw = RequireReadBackAsWritten(
        unfinished_irw, {directory, "unfinished_irw"}, "the unfinished IRW project");
    const auto& read_interfaces = read_irw.contact_record->declaration.interfaces;
    Require(!read_interfaces[0].independent_wheel_revolute_joint_name.has_value() &&
                read_interfaces[1].independent_wheel_revolute_joint_name == std::string(),
            "a null wheel joint reference and an empty one did not read back as themselves");
}

// The document `project` is written as, for a test to spoil.
Json WrittenDocument(const VehicleProject& project, const std::filesystem::path& directory) {
    const VehicleProjectFileLocation location{directory, "document_to_spoil"};
    vehicle_project::WriteVehicleProjectFile(project, location);
    return Json::parse(ReadBytes(vehicle_project::VehicleProjectFilePath(location)));
}

// Reading the project file holding `text` is refused, naming each of
// `places`.
void RequireReadingRefused(const std::string& text, const std::filesystem::path& directory,
                           std::initializer_list<std::string_view> places,
                           const std::string& what) {
    const VehicleProjectFileLocation location{directory, "spoiled_document"};
    WriteBytes(vehicle_project::VehicleProjectFilePath(location), text);
    RequireRefusal<std::invalid_argument>(
        [&] { (void)vehicle_project::ReadVehicleProjectFile(location); }, places,
        "reading a project file with " + what);
}

void VerifyReadingRefusesFaultyDocuments(const VehicleProject& project,
                                         const std::filesystem::path& directory) {
    const Json document = WrittenDocument(project, directory);

    Json unknown_key = document;
    unknown_key["vehicle_colour"] = "red";
    RequireReadingRefused(unknown_key.dump(2), directory, {"$.vehicle_colour"}, "an unknown key");

    // A station layout naming a reference body, as a project file written
    // before the layout lost that name holds it.
    Json layout_reference_body = document;
    layout_reference_body["vehicle_definition"]["mechanical_track_station_layout"]
                         ["reference_body_name"] = "carbody";
    RequireReadingRefused(
        layout_reference_body.dump(2), directory,
        {"$.vehicle_definition.mechanical_track_station_layout.reference_body_name"},
        "a station layout naming a reference body");

    Json missing_key = document;
    missing_key.erase("element_groups");
    RequireReadingRefused(missing_key.dump(2), directory, {"$.element_groups"}, "a missing key");

    // A repeated key is gone from any parsed document, so it goes into the
    // text: the first placement's body name, stated twice.
    const std::string text = document.dump(2);
    const std::string body_name_line =
        "\"body_name\": " + document.at("free_body_placements").at(0).at("body_name").dump() + ",";
    const std::size_t first_placement_body_name =
        text.find(body_name_line, text.find("\"free_body_placements\""));
    Require(first_placement_body_name != std::string::npos,
            "the first placement's body name is not in the written text");
    std::string repeated_key = text;
    repeated_key.insert(first_placement_body_name, body_name_line + " ");
    RequireReadingRefused(repeated_key, directory, {"$.free_body_placements[0]", "body_name"},
                          "a key repeated inside a placement");

    Json wrong_member_type = document;
    wrong_member_type["element_groups"] =
        Json::parse(R"([{"name": "bogie", "members": [{"kind": "rigid_body", "name": 7}],
                         "subgroups": []}])");
    RequireReadingRefused(wrong_member_type.dump(2), directory,
                          {"$.element_groups[0].members[0].name"},
                          "a group member whose name is a number");

    Json unknown_kind_word = document;
    unknown_kind_word["element_groups"] =
        Json::parse(R"([{"name": "bogie", "members": [], "subgroups": [{"name": "dampers",
                         "members": [{"kind": "damper", "name": "yaw_damper"}],
                         "subgroups": []}]}])");
    RequireReadingRefused(unknown_kind_word.dump(2), directory,
                          {"$.element_groups[0].subgroups[0].members[0].kind"},
                          "a group member whose kind word names no kind");

    for (const Json& not_an_object : {Json("drawn elsewhere"), Json::array(), Json(1.0)}) {
        Json visual_definition = document;
        visual_definition["visual_definition"] = not_an_object;
        RequireReadingRefused(visual_definition.dump(2), directory, {"$.visual_definition"},
                              "a visual definition that is " + not_an_object.dump());
    }

    const std::string gravity_key = "\"gravitational_acceleration_meters_per_second_squared\": ";
    std::string huge_gravity = text;
    const std::size_t gravity_value = huge_gravity.find(gravity_key) + gravity_key.size();
    huge_gravity.replace(gravity_value, huge_gravity.find(',', gravity_value) - gravity_value,
                         "1e999");
    RequireReadingRefused(huge_gravity, directory,
                          {"$.gravitational_acceleration_meters_per_second_squared"},
                          "a gravity binary64 cannot hold");

    RequireReadingRefused(text.substr(0, text.size() / 2), directory, {"syntax"},
                          "half of its document");
}

void VerifyProjectNamesAreIdentifiers(const VehicleProject& project,
                                      const std::filesystem::path& directory) {
    const std::vector<std::string> entries = EntriesOf(directory);
    for (const std::string& name : {std::string("my vehicle"), std::string("vehicles/gz18")}) {
        RequireRefusal<std::invalid_argument>(
            [&] { (void)vehicle_project::VehicleProjectFilePath({directory, name}); },
            {"'" + name + "'"}, "the project name '" + name + "'");
        RequireRefusal<std::invalid_argument>(
            [&] { vehicle_project::WriteVehicleProjectFile(project, {directory, name}); },
            {"'" + name + "'"}, "writing the project named '" + name + "'");
        RequireRefusal<std::invalid_argument>(
            [&] { (void)vehicle_project::ReadVehicleProjectFile({directory, name}); },
            {"'" + name + "'"}, "reading the project named '" + name + "'");
    }
    RequireRefusal<std::invalid_argument>(
        [&] { vehicle_project::WriteVehicleProjectFile(project, {directory, ""}); },
        {"vehicle project name"}, "writing a project without a name");
    Require(EntriesOf(directory) == entries, "a refused project name left something written");
}

// A write that cannot complete its document leaves the file it would replace
// byte for byte as it was, and no pending file.
template <typename Spoil>
void RequireSpoiledWriteRefused(const VehicleProject& project,
                                const VehicleProjectFileLocation& location, Spoil spoil,
                                std::string_view place, const std::string& what) {
    const std::filesystem::path path = vehicle_project::VehicleProjectFilePath(location);
    const std::string kept = ReadBytes(path);
    const std::vector<std::string> entries = EntriesOf(location.directory);
    VehicleProject spoiled = project;
    spoil(spoiled);
    RequireRefusal<std::invalid_argument>(
        [&] { vehicle_project::WriteVehicleProjectFile(spoiled, location); }, {place},
        "writing a project with " + what);
    Require(ReadBytes(path) == kept, "writing a project with " + what + " changed the file");
    Require(EntriesOf(location.directory) == entries,
            "writing a project with " + what + " left something in the directory");
}

void VerifyFailedWritesLeaveTheDirectoryAsItWas(const VehicleProject& project,
                                                const std::filesystem::path& directory) {
    const VehicleProjectFileLocation location{directory, "kept_project"};
    vehicle_project::WriteVehicleProjectFile(project, location);
    constexpr double kNotANumber = std::numeric_limits<double>::quiet_NaN();
    RequireSpoiledWriteRefused(
        project, location,
        [](VehicleProject& spoiled) {
            spoiled.gravitational_acceleration_meters_per_second_squared =
                std::numeric_limits<double>::infinity();
        },
        "$.gravitational_acceleration_meters_per_second_squared", "an infinite gravity");
    RequireSpoiledWriteRefused(
        project, location,
        [](VehicleProject& spoiled) {
            spoiled.free_body_placements.at(1).lateral_offset_in_local_track_frame_meters =
                kNotANumber;
        },
        "$.free_body_placements[1]", "a placement offset that is not a number");
    RequireSpoiledWriteRefused(
        project, location,
        [](VehicleProject& spoiled) {
            spoiled.vehicle_definition.rigid_bodies.at(2).mass_kilograms = kNotANumber;
        },
        "$.vehicle_definition.rigid_bodies[2]", "a mass that is not a number");
    RequireSpoiledWriteRefused(
        project, location,
        [](VehicleProject& spoiled) {
            spoiled.gravitational_acceleration_meters_per_second_squared.reset();
        },
        "$.gravitational_acceleration_meters_per_second_squared", "no gravity");
    RequireSpoiledWriteRefused(
        project, location,
        [](VehicleProject& spoiled) { spoiled.visual_definition_document = "[\"wheel\"]"; },
        "$.visual_definition", "a visual definition that is not an object's text");

    const std::filesystem::path missing_directory = directory / "missing_directory";
    const std::vector<std::string> entries = EntriesOf(directory);
    RequireRefusal<std::runtime_error>(
        [&] { vehicle_project::WriteVehicleProjectFile(project, {missing_directory, "lost"}); },
        {missing_directory.string()}, "writing into a directory that does not exist");
    Require(!std::filesystem::exists(missing_directory) && EntriesOf(directory) == entries,
            "writing into a directory that does not exist created something");
}

void VerifyOnlyProjectFilesAreListed(const VehicleProject& project,
                                     const std::filesystem::path& directory) {
    vehicle_project::WriteVehicleProjectFile(project, {directory, "second_project"});
    vehicle_project::WriteVehicleProjectFile(project, {directory, "First-project.1"});
    WriteBytes(directory / "pending_project.vehicle_project.json.pending", "{}\n");
    WriteBytes(directory / "other_document.json", "{}\n");
    WriteBytes(directory / "not a name.vehicle_project.json", "{}\n");
    std::filesystem::create_directory(directory / "folder.vehicle_project.json");
    Require(vehicle_project::ListVehicleProjectNames(directory) ==
                std::vector<std::string>{"First-project.1", "second_project"},
            "the projects listed are not exactly the project files, sorted");
    const std::filesystem::path missing_directory = directory / "missing_directory";
    RequireRefusal<std::filesystem::filesystem_error>(
        [&] { (void)vehicle_project::ListVehicleProjectNames(missing_directory); },
        {missing_directory.string()}, "listing a directory that does not exist");
}

// A visual definition file is taken in only when it holds one JSON object.
void VerifyVisualDefinitionFilesAreReadStrictly(const std::filesystem::path& directory) {
    const auto require_refused = [&directory](const std::string& text, std::string_view place,
                                              const std::string& what) {
        const std::filesystem::path path = directory / "visual_definition.json";
        WriteBytes(path, text);
        RequireRefusal<std::invalid_argument>(
            [&] { (void)vehicle_project::ReadVisualDefinitionDocumentFromFile(path); }, {place},
            "a visual definition file holding " + what);
    };
    require_refused("[{\"name\": \"carbody\"}]", "$", "an array");
    require_refused("{\"parts\": [], \"parts\": []}", "$.parts", "a repeated key");
    require_refused("{\"parts\": [1e999]}", "$.parts[0]", "a number binary64 cannot hold");
    const std::filesystem::path missing_file = directory / "missing_visual_definition.json";
    RequireRefusal<std::runtime_error>(
        [&] { (void)vehicle_project::ReadVisualDefinitionDocumentFromFile(missing_file); },
        {missing_file.string()}, "a visual definition file that does not exist");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 10) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_project_file_reading_and_writing GZ18_VEHICLE "
                     "GZ18_STARTUP_STATE GZ18_CONTACT GZ18_VISUAL_DEFINITION IRW_VEHICLE "
                     "IRW_STARTUP_STATE IRW_CONTACT IRW_VISUAL_DEFINITION WORK_DIRECTORY\n");
        return 2;
    }
    try {
        const ReferenceVehicleFiles gz18_files{argv[1], argv[2], argv[3], argv[4]};
        const ReferenceVehicleFiles irw_files{argv[5], argv[6], argv[7], argv[8]};
        const std::filesystem::path work_directory = argv[9];
        const VehicleProject gz18 = OpenAsTheModelingServiceDoes(gz18_files);
        const VehicleProject irw = OpenAsTheModelingServiceDoes(irw_files);

        VerifyReferenceVehicleReadBackAsWritten(gz18, gz18_files,
                                                FreshDirectory(work_directory, "gz18"));
        VerifyReferenceVehicleReadBackAsWritten(irw, irw_files,
                                                FreshDirectory(work_directory, "irw"));
        VerifyUnfinishedProjectsReadBackAsWritten(gz18, irw,
                                                  FreshDirectory(work_directory, "unfinished"));
        VerifyReadingRefusesFaultyDocuments(gz18,
                                            FreshDirectory(work_directory, "faulty_documents"));
        VerifyProjectNamesAreIdentifiers(gz18, FreshDirectory(work_directory, "project_names"));
        VerifyFailedWritesLeaveTheDirectoryAsItWas(gz18,
                                                   FreshDirectory(work_directory, "failed_writes"));
        VerifyOnlyProjectFilesAreListed(gz18, FreshDirectory(work_directory, "listing"));
        VerifyVisualDefinitionFilesAreReadStrictly(
            FreshDirectory(work_directory, "visual_definitions"));
        std::puts(
            "vehicle project files: GZ18, IRW and unfinished projects read back as written, "
            "faulty documents, names and writes refused, project files listed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle project files: %s\n", error.what());
        return 1;
    }
}
