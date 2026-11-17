#include <algorithm>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include "../vehicle_project/configuration_record_comparison.h"
#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_contact_record.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/vehicle_contact_topology.h"
#include "strict_json.h"
#include "vehicle_contact_record_json_reading_and_writing.h"
#include "vehicle_contact_record_validation.h"
#include "vehicle_definition_json_reading_and_writing.h"

namespace {

using namespace orvd::configuration;
using orvd::wheel_rail_contact::WheelSide;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

template <typename Operation>
void RequireRefusal(Operation&& operation, std::string_view subject, std::string_view reason) {
    try {
        operation();
    } catch (const std::invalid_argument& refusal) {
        const std::string_view message(refusal.what());
        Require(message.find(subject) != message.npos && message.find(reason) != message.npos,
                "contact refusal did not identify the subject and violated condition");
        return;
    }
    throw std::runtime_error("an invalid contact declaration was accepted");
}

struct DeclaredVehicle {
    VehicleDefinition vehicle;
    VehicleContactDeclaration contact;
};

VehicleRigidBodyDefinition FreeBody(const std::string& name) {
    return {.name = name,
            .moves_freely_in_world = true,
            .mass_kilograms = 1000.0,
            .inertia_moments_about_center_of_mass_kilogram_square_meters =
                Eigen::Vector3d::Constant(100.0)};
}

DeclaredVehicle MakeThreeAxleBogie() {
    DeclaredVehicle result;
    result.vehicle.vehicle_name = "three_axle_bogie";
    result.vehicle.mechanical_definition_identifier = "three_axle_mechanics";
    result.vehicle.rigid_bodies.push_back(FreeBody("bogie"));
    auto& layout = result.vehicle.mechanical_track_station_layout;
    layout.free_body_station_offsets.push_back({"bogie", 0.0});
    for (int axle_index = 0; axle_index < 3; ++axle_index) {
        const std::string wheelset = "wheelset_" + std::to_string(axle_index);
        const std::string carrier = "station_" + std::to_string(axle_index);
        result.vehicle.rigid_bodies.push_back(FreeBody(wheelset));
        layout.free_body_station_offsets.push_back({wheelset, 2.0 - 2.0 * axle_index});
        layout.wheel_pair_station_reference_body_names.push_back(wheelset);
        result.vehicle.translational_spring_dampers.push_back(
            {.name = "suspension_" + std::to_string(axle_index),
             .reference_frame_name = "bogie",
             .opposite_frame_name = wheelset,
             .stiffness_newtons_per_meter = Eigen::Vector3d::Constant(1.0e6),
             .damping_newton_seconds_per_meter = Eigen::Vector3d::Constant(1.0e4)});
        result.contact.carriers.push_back({carrier, wheelset, Eigen::Matrix3d::Identity()});
        result.contact.interfaces.push_back(
            {wheelset + ".right", carrier, wheelset, WheelSide::kRight, std::nullopt});
        result.contact.interfaces.push_back(
            {wheelset + ".left", carrier, wheelset, WheelSide::kLeft, std::nullopt});
    }
    return result;
}

std::unique_ptr<AssembledVehicleSystem> RequireDeclarationAndAssembly(
    const DeclaredVehicle& declared) {
    auto system = AssembleVehicleSystem(declared.vehicle, 9.81, &declared.contact);
    const auto* checked_topology = system->checked_contact_topology();
    Require(checked_topology != nullptr, "assembly did not retain its checked contact declaration");
    const auto& topology = *checked_topology;
    Require(topology.carriers.size() == declared.contact.carriers.size() &&
                topology.interfaces.size() == declared.contact.interfaces.size(),
            "declaration generation changed the contact dimensions");
    Require(topology.active_torque_couples.empty(), "contact declaration created active torques");
    for (std::size_t carrier_index = 0; carrier_index < topology.carriers.size(); ++carrier_index) {
        const auto& input = declared.contact.carriers[carrier_index];
        const auto& output = topology.carriers[carrier_index];
        Require(output.carrier_name == input.carrier_name && output.body_name == input.body_name &&
                    output.rotation_body_from_nonspinning_wheel_profile ==
                        input.rotation_body_from_nonspinning_wheel_profile,
                "carrier declaration order or fields changed");
    }
    for (std::size_t interface_index = 0; interface_index < topology.interfaces.size();
         ++interface_index) {
        const auto& input = declared.contact.interfaces[interface_index];
        const auto& output = topology.interfaces[interface_index];
        Require(output.interface_name == input.interface_name &&
                    output.carrier_name == input.carrier_name &&
                    output.wheel_body_name == input.wheel_body_name && output.side == input.side,
                "interface declaration order or fields changed");
        Require(output.independent_wheel_revolute_joint.has_value() ==
                    input.independent_wheel_revolute_joint_name.has_value(),
                "independent-wheel joint presence changed");
        if (input.independent_wheel_revolute_joint_name) {
            Require(output.independent_wheel_revolute_joint->joint_name ==
                        *input.independent_wheel_revolute_joint_name,
                    "independent-wheel joint name changed");
        }
    }
    Require(system->model().num_rigid_bodies() ==
                static_cast<int>(declared.vehicle.rigid_bodies.size()),
            "mechanical assembly did not retain the stated bodies");
    Require(system->contact_force_plan() == nullptr && system->active_torque_plan() == nullptr,
            "declaration-only assembly created a runtime contact or actuation plan");
    return system;
}

DeclaredVehicle RenameMechanicalObjects(DeclaredVehicle declared) {
    std::unordered_map<std::string, std::string> frame_names;
    for (const auto& body : declared.vehicle.rigid_bodies) {
        frame_names.emplace(body.name, "body." + body.name);
    }
    for (const auto& frame : declared.vehicle.fixed_frames) {
        frame_names.emplace(frame.name, "frame." + frame.name);
    }
    for (auto& body : declared.vehicle.rigid_bodies) {
        body.name = frame_names.at(body.name);
    }
    for (auto& frame : declared.vehicle.fixed_frames) {
        frame.name = frame_names.at(frame.name);
        frame.body_name = frame_names.at(frame.body_name);
    }
    const auto rename_joints = [&](auto& joints) {
        for (auto& joint : joints) {
            joint.name = "joint." + joint.name;
            joint.parent_frame_name = frame_names.at(joint.parent_frame_name);
            joint.child_frame_name = frame_names.at(joint.child_frame_name);
        }
    };
    rename_joints(declared.vehicle.revolute_joints);
    rename_joints(declared.vehicle.ball_rpy_joints);
    rename_joints(declared.vehicle.weld_joints);
    const auto rename_force_ends = [&](auto& elements) {
        for (auto& element : elements) {
            element.reference_frame_name = frame_names.at(element.reference_frame_name);
            element.opposite_frame_name = frame_names.at(element.opposite_frame_name);
        }
    };
    rename_force_ends(declared.vehicle.translational_spring_dampers);
    rename_force_ends(declared.vehicle.roll_spring_damper_couples);
    rename_force_ends(declared.vehicle.series_spring_viscous_dampers);
    rename_force_ends(declared.vehicle.point_to_point_series_spring_bilinear_dampers);
    rename_force_ends(declared.vehicle.saturated_piecewise_linear_dampers);
    rename_force_ends(declared.vehicle.reference_frame_linear_bushings);
    rename_force_ends(declared.vehicle.point_to_point_linear_spring_dampers);
    rename_force_ends(declared.vehicle.point_to_point_hysteretic_springs);
    auto& layout = declared.vehicle.mechanical_track_station_layout;
    for (auto& offset : layout.free_body_station_offsets) {
        offset.body_name = frame_names.at(offset.body_name);
    }
    for (auto& body_name : layout.wheel_pair_station_reference_body_names) {
        body_name = frame_names.at(body_name);
    }
    for (auto& carrier : declared.contact.carriers) {
        carrier.carrier_name = "station." + carrier.carrier_name;
        carrier.body_name = frame_names.at(carrier.body_name);
    }
    for (auto& interface : declared.contact.interfaces) {
        interface.interface_name = "contact." + interface.interface_name;
        interface.carrier_name = "station." + interface.carrier_name;
        interface.wheel_body_name = frame_names.at(interface.wheel_body_name);
        if (interface.independent_wheel_revolute_joint_name) {
            *interface.independent_wheel_revolute_joint_name =
                "joint." + *interface.independent_wheel_revolute_joint_name;
        }
    }
    return declared;
}

void VerifyIndependentWheelRefusals(const DeclaredVehicle& reference) {
    const auto& first_interface = reference.contact.interfaces.front();
    const std::string joint_name = *first_interface.independent_wheel_revolute_joint_name;
    DeclaredVehicle changed = reference;
    const auto check = [&] {
        (void)AssembleVehicleSystem(changed.vehicle, 9.81, &changed.contact);
    };
    const auto joint_index =
        static_cast<std::size_t>(std::ranges::find(changed.vehicle.revolute_joints, joint_name,
                                                   &VehicleRevoluteJointDefinition::name) -
                                 changed.vehicle.revolute_joints.begin());

    const auto removed = changed.vehicle.revolute_joints[joint_index];
    changed.vehicle.revolute_joints.erase(changed.vehicle.revolute_joints.begin() + joint_index);
    changed.vehicle.ball_rpy_joints.push_back(
        {removed.name, removed.parent_frame_name, removed.child_frame_name});
    RequireRefusal(check, first_interface.interface_name, "not a declared revolute joint");

    changed = reference;
    changed.vehicle.revolute_joints[joint_index].child_frame_name =
        reference.contact.interfaces.back().wheel_body_name;
    RequireRefusal(check, first_interface.interface_name, "directly to wheel body frame");

    changed = reference;
    changed.vehicle.revolute_joints[joint_index].axis_in_parent_frame *= -1.0;
    RequireRefusal(check, first_interface.interface_name, "unit axis -R_BP");

    changed = reference;
    changed.vehicle.fixed_frames.push_back(
        {.name = "offset_wheel_mount",
         .body_name = removed.parent_frame_name,
         .position_in_body_frame_meters = Eigen::Vector3d(0.001, 0.0, 0.0)});
    changed.vehicle.revolute_joints[joint_index].parent_frame_name = "offset_wheel_mount";
    RequireRefusal(check, first_interface.interface_name, "without an offset");

    changed = reference;
    changed.contact.carriers[1].body_name = changed.contact.carriers.front().body_name;
    RequireRefusal(check, changed.contact.carriers[1].carrier_name, "both refer to body");

    changed = reference;
    changed.contact.interfaces.erase(changed.contact.interfaces.begin());
    RequireRefusal(check, first_interface.carrier_name,
                   first_interface.side == WheelSide::kLeft ? "missing left" : "missing right");

    changed = reference;
    auto carrier_body =
        std::ranges::find(changed.vehicle.rigid_bodies, changed.contact.carriers.front().body_name,
                          &VehicleRigidBodyDefinition::name);
    carrier_body->moves_freely_in_world = false;
    RequireRefusal(check, first_interface.carrier_name, "not a free body");
}

void VerifyProfileBasisControlsJointAxis(DeclaredVehicle declared) {
    for (auto& carrier : declared.contact.carriers) {
        carrier.rotation_body_from_nonspinning_wheel_profile.setIdentity();
    }
    for (const auto& interface : declared.contact.interfaces) {
        auto joint = std::ranges::find(declared.vehicle.revolute_joints,
                                       *interface.independent_wheel_revolute_joint_name,
                                       &VehicleRevoluteJointDefinition::name);
        joint->axis_in_parent_frame = -2.0 * Eigen::Vector3d::UnitY();
    }
    RequireDeclarationAndAssembly(declared);
    const auto& first = declared.contact.interfaces.front();
    auto joint = std::ranges::find(declared.vehicle.revolute_joints,
                                   *first.independent_wheel_revolute_joint_name,
                                   &VehicleRevoluteJointDefinition::name);
    joint->axis_in_parent_frame = Eigen::Vector3d::UnitY();
    RequireRefusal([&] { (void)AssembleVehicleSystem(declared.vehicle, 9.81, &declared.contact); },
                   first.interface_name, "unit axis -R_BP");
}

void VerifyJointAndForceLoops() {
    DeclaredVehicle branched = MakeThreeAxleBogie();
    for (const std::string branch : {"left_link", "right_link"}) {
        auto body = FreeBody(branch);
        body.moves_freely_in_world = false;
        branched.vehicle.rigid_bodies.push_back(std::move(body));
        branched.vehicle.revolute_joints.push_back(
            {.name = "pivot_" + branch,
             .parent_frame_name = "bogie",
             .child_frame_name = branch,
             .axis_in_parent_frame = Eigen::Vector3d::UnitZ()});
    }
    branched.vehicle.translational_spring_dampers.push_back(
        {.name = "cross_branch_spring",
         .reference_frame_name = "left_link",
         .opposite_frame_name = "right_link",
         .stiffness_newtons_per_meter = Eigen::Vector3d::Constant(1.0e6),
         .damping_newton_seconds_per_meter = Eigen::Vector3d::Constant(1.0e4)});
    const auto system = RequireDeclarationAndAssembly(branched);
    Require(system->force_plan().translational_spring_damper_count() == 4,
            "cross-branch force was not retained in the force plan");

    branched.vehicle.translational_spring_dampers.pop_back();
    branched.vehicle.revolute_joints.push_back({.name = "cross_branch_joint",
                                                .parent_frame_name = "left_link",
                                                .child_frame_name = "right_link",
                                                .axis_in_parent_frame = Eigen::Vector3d::UnitZ()});
    RequireRefusal([&] { (void)AssembleVehicleSystem(branched.vehicle, 9.81, &branched.contact); },
                   "cross_branch_joint", "closes a loop");
}

void VerifyReferenceAssemblyWithoutStartup(const std::filesystem::path& root) {
    for (const std::string reference : {"gz18", "irw"}) {
        const auto directory = root / "vehicle_library" / reference;
        const DeclaredVehicle declared{
            LoadVehicleDefinitionFromJsonFile(directory / "vehicle_definition.json"),
            LoadVehicleContactRecordFromJsonFile(directory / "contact_record.json").declaration};
        RequireDeclarationAndAssembly(declared);
    }
    const auto mechanical_only = AssembleVehicleSystem(MakeThreeAxleBogie().vehicle, 9.81);
    Require(mechanical_only->checked_contact_topology() == nullptr,
            "mechanical-only assembly reported a contact declaration");
}

void VerifyRecordLoading(const std::filesystem::path& record_path,
                         const std::filesystem::path& fixture_root) {
    std::ifstream source(record_path);
    const std::string document{std::istreambuf_iterator<char>(source), {}};
    Require(!document.empty(), "the reference contact record could not be read");
    std::filesystem::create_directories(fixture_root);
    const auto fixture = fixture_root / "contact_record.json";
    const auto reject_document = [&](const std::string& contents, std::string_view field) {
        {
            std::ofstream output(fixture);
            output << contents;
        }
        RequireRefusal([&] { (void)LoadVehicleContactRecordFromJsonFile(fixture); }, field, "");
    };
    std::string changed = document;
    changed.insert(changed.find('{') + 1, "\"unexpected_field\": 1,");
    reject_document(changed, "unexpected_field");
    changed = document;
    changed.insert(changed.find('{') + 1, "\"declaration\": {},");
    reject_document(changed, "declaration");
    changed = document;
    const auto side = changed.find("\"right\"");
    Require(side != std::string::npos, "the reference has no right-side interface");
    changed.replace(side, 7, "\"up\"");
    reject_document(changed, "side");

    const auto original = strict_json::ParseStrictJson(document);
    for (const std::string_view field : {"wheel", "rail", "computation"}) {
        for (const std::string invalid_identifier : {"", "invalid identifier"}) {
            auto invalid = original;
            if (field == "computation") {
                invalid[field]["contact_strategy_identifier"] = invalid_identifier;
                reject_document(invalid.dump(),
                                "vehicle contact record.computation.contact_strategy_identifier");
            } else {
                invalid[field]["profile"]["profile_identifier"] = invalid_identifier;
                reject_document(invalid.dump(), "vehicle contact record." + std::string(field) +
                                                    ".profile.profile_identifier");
            }
        }
    }
    std::filesystem::remove(fixture);
}

void VerifyConfigurationRoundtrip(const std::filesystem::path& root) {
    using namespace orvd::configuration::internal;
    const std::string mechanical_path = "$.vehicle_definition";
    const std::string contact_path = "$.contact_record";
    const auto require_contact_roundtrip = [&](const VehicleContactRecord& original) {
        const auto encoded = EncodeVehicleContactRecordToJson(original, contact_path);
        const auto decoded = DecodeVehicleContactRecordFromJson(
            strict_json::ParseStrictJson(encoded.dump()), contact_path);
        Require(orvd::configuration_record_comparison::Same(original, decoded),
                "contact JSON roundtrip changed a field or interface order");
    };
    for (const std::string reference : {"gz18", "irw"}) {
        const auto directory = root / "vehicle_library" / reference;
        const auto vehicle =
            LoadVehicleDefinitionFromJsonFile(directory / "vehicle_definition.json");
        const auto encoded = EncodeVehicleDefinitionToJson(vehicle, mechanical_path);
        const auto decoded = DecodeVehicleDefinitionFromJson(
            strict_json::ParseStrictJson(encoded.dump()), mechanical_path);
        Require(orvd::configuration_record_comparison::Same(vehicle, decoded),
                "reference mechanical record did not survive JSON roundtrip");
        require_contact_roundtrip(
            LoadVehicleContactRecordFromJsonFile(directory / "contact_record.json"));
    }
    require_contact_roundtrip(VehicleContactRecord{});

    const auto original =
        LoadVehicleContactRecordFromJsonFile(root / "vehicle_library/gz18/contact_record.json");
    auto unfinished = original;
    unfinished.wheel.profile.profile_identifier.clear();
    unfinished.wheel.profile.profile_path = "../profiles/待选型面.json";
    unfinished.rail.profile.profile_identifier = "unfinished identifier";
    unfinished.computation.contact_strategy_identifier.clear();
    unfinished.wheel.nominal_rolling_radius_meters = 0.0;
    unfinished.rail.track_gauge_meters = -1.0;
    unfinished.declaration.carriers.front().rotation_body_from_nonspinning_wheel_profile(0, 1) =
        0.25;
    unfinished.declaration.interfaces.front().independent_wheel_revolute_joint_name = "";
    unfinished.declaration.interfaces.back().independent_wheel_revolute_joint_name = std::nullopt;
    unfinished.rail.profile_origin_mode =
        orvd::wheel_rail_contact::RailProfileOriginMode::kTrackStation;
    unfinished.computation.creep_coefficients_outside_table =
        orvd::wheel_rail_contact::OutsideTableRule::kRefuse;
    require_contact_roundtrip(unfinished);
    RequireRefusal([&] { RequireVehicleContactRecordInvariants(unfinished, contact_path); },
                   "$.contact_record.wheel.profile.profile_identifier", "");

    const auto encoded_original = EncodeVehicleContactRecordToJson(original, contact_path);
    auto missing_key = encoded_original;
    missing_key["declaration"]["interfaces"][0].erase("wheel_body_name");
    RequireRefusal([&] { (void)DecodeVehicleContactRecordFromJson(missing_key, contact_path); },
                   "$.contact_record.declaration.interfaces[0].wheel_body_name", "");
    auto wrong_type = encoded_original;
    wrong_type["wheel"]["nominal_rolling_radius_meters"] = "unfinished";
    RequireRefusal([&] { (void)DecodeVehicleContactRecordFromJson(wrong_type, contact_path); },
                   "$.contact_record.wheel.nominal_rolling_radius_meters", "");
    auto wrong_enum = encoded_original;
    wrong_enum["declaration"]["interfaces"][0]["side"] = "up";
    RequireRefusal([&] { (void)DecodeVehicleContactRecordFromJson(wrong_enum, contact_path); },
                   "$.contact_record.declaration.interfaces[0].side", "");
    RequireRefusal(
        [&] {
            (void)strict_json::ParseStrictJson(
                R"({"contact_record":{"wheel":{"profile":{"profile_identifier":"a",)"
                R"("profile_identifier":"b"}}}})");
        },
        "$.contact_record.wheel.profile.profile_identifier", "");

    auto nonfinite = original;
    nonfinite.wheel.nominal_rolling_radius_meters = std::numeric_limits<double>::quiet_NaN();
    RequireRefusal([&] { (void)EncodeVehicleContactRecordToJson(nonfinite, contact_path); },
                   "$.contact_record.wheel.nominal_rolling_radius_meters", "");
    auto unrepresentable = original;
    unrepresentable.declaration.interfaces.front().side = static_cast<WheelSide>(-1);
    RequireRefusal([&] { (void)EncodeVehicleContactRecordToJson(unrepresentable, contact_path); },
                   "$.contact_record.declaration.interfaces[0].side", "");
}

void VerifyAliasedRuntimeCarriers(const std::filesystem::path& root) {
    const auto vehicle =
        LoadVehicleDefinitionFromJsonFile(root / "vehicle_library/gz18/vehicle_definition.json");
    const auto startup = LoadResolvedStartupStateFromJsonFile(
        root / "vehicle_library/gz18/startup_states/moving_startup_60kmh.json");
    auto record =
        LoadVehicleContactRecordFromJsonFile(root / "vehicle_library/gz18/contact_record.json");
    for (auto& carrier : record.declaration.carriers)
        carrier.carrier_name = "projection." + carrier.carrier_name;
    for (auto& interface : record.declaration.interfaces)
        interface.carrier_name = "projection." + interface.carrier_name;
    const auto line =
        LoadTrackGeometryFromJsonFile(root / "track_library/geometries/straight_level_1100m.json");
    const auto scenario =
        AssembleVehicleContactScenario(vehicle, startup, record, line, root, 15.0);
    const auto& plan = *scenario->vehicle_system().contact_force_plan();
    for (int carrier_index = 0; carrier_index < plan.carrier_count(); ++carrier_index) {
        const auto& carrier = plan.carrier_definition(carrier_index);
        const auto& placements = scenario->initial_context().wheel_pair_placements();
        const auto placement =
            std::ranges::find(placements, carrier.body_name,
                              &ResolvedWheelPairPlacement::station_reference_body_name);
        Require(placement != placements.end() && carrier.carrier_name != carrier.body_name &&
                    plan.initial_projection_station_meters(carrier_index) ==
                        placement->track_station_meters,
                "an aliased carrier did not bind its body's resolved station");
    }
    record.wheel.wheel_lateral_datum_magnitude_meters = 0.75;
    const auto prepared = PrepareVehicleContact(vehicle, startup, record, root);
    Require(prepared.personality->pose_constants(WheelSide::kRight).wheel_lateral_datum_meters ==
                    0.75 &&
                prepared.personality->pose_constants(WheelSide::kLeft).wheel_lateral_datum_meters ==
                    -0.75,
            "runtime contact ignored the supplied record's wheel datum");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::fprintf(stderr,
                     "usage: verify_vehicle_contact_declaration IRW_VEHICLE IRW_CONTACT DATA_ROOT "
                     "FIXTURE_ROOT\n");
        return 2;
    }
    try {
        const DeclaredVehicle three_axle = MakeThreeAxleBogie();
        RequireDeclarationAndAssembly(three_axle);
        const DeclaredVehicle irw{LoadVehicleDefinitionFromJsonFile(argv[1]),
                                  LoadVehicleContactRecordFromJsonFile(argv[2]).declaration};
        const DeclaredVehicle renamed = RenameMechanicalObjects(irw);
        RequireDeclarationAndAssembly(renamed);
        VerifyIndependentWheelRefusals(renamed);
        VerifyProfileBasisControlsJointAxis(renamed);
        VerifyJointAndForceLoops();
        VerifyReferenceAssemblyWithoutStartup(argv[3]);
        VerifyRecordLoading(argv[2], argv[4]);
        VerifyConfigurationRoundtrip(argv[3]);
        VerifyAliasedRuntimeCarriers(argv[3]);
        std::puts(
            "contact assembly without startup, declarations, force loops and carrier aliases "
            "passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "vehicle contact declaration: %s\n", error.what());
        return 1;
    }
}
