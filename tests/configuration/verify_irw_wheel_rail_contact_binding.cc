// The IRW contact personality at its real S1002/UIC60 asset boundary.
//
// G68 stops before the eight independent wheel bodies are connected to the
// vehicle RHS. This gate therefore checks the closed vehicle personality, the
// source-lateral wheel preparation, the applied roll transport and a real
// static contact without inventing G69's axle-bridge kinematics.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/configuration/irw_wheel_rail_contact.h"
#include "orvd/configuration/load_profile_points.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/wheel_rail_contact/wheel_rail_pose.h"
#include "wheel_rail_contact/allocation_probe.h"

namespace {

namespace fs = std::filesystem;

using orvd::configuration::AssembleIrwWheelRailContact;
using orvd::configuration::IrwWheelRailContact;
using orvd::configuration::LoadResolvedStartupStateFromJsonFile;
using orvd::configuration::ResolvedStartupState;
using orvd::configuration::StartupWheelRailBinding;
using orvd::test::AllocationScope;
using orvd::wheel_rail_contact::BuildContactPoseScalars;
using orvd::wheel_rail_contact::OutsideTableRule;
using orvd::wheel_rail_contact::ProfileTrackRollTransport;
using orvd::wheel_rail_contact::ProfileTrackRollTransportPolicy;
using orvd::wheel_rail_contact::WheelRailContactInput;
using orvd::wheel_rail_contact::WheelRailContactResult;
using orvd::wheel_rail_contact::WheelRailContactWorkspace;
using orvd::wheel_rail_contact::WheelRailPoseInput;
using orvd::wheel_rail_contact::WheelSide;
using orvd::wheel_rail_contact::WheelsetPlacement;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "IRW contact binding: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireClose(double actual, double expected, double tolerance,
                  std::string_view what) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::fprintf(stderr,
                     "IRW contact binding: %.*s (expected %.17g, got %.17g)\n",
                     static_cast<int>(what.size()), what.data(), expected,
                     actual);
        ++failures;
    }
}

void RequireRefusal(const std::function<void()>& action,
                    std::string_view diagnostic, std::string_view what) {
    try {
        action();
        Require(false, what);
    } catch (const std::invalid_argument& error) {
        Require(std::string_view(error.what()).find(diagnostic) !=
                    std::string_view::npos,
                what);
    } catch (...) {
        Require(false, what);
    }
}

std::string ReadText(const fs::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("cannot read fixture '" + path.string() + "'");
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

void WriteText(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output || !(output << text)) {
        throw std::runtime_error("cannot write fixture '" + path.string() + "'");
    }
}

void CopyAsset(const fs::path& source_root, const fs::path& destination_root,
               const fs::path& relative, std::string_view from = {},
               std::string_view to = {}) {
    std::string text = ReadText(source_root / relative);
    if (!from.empty()) {
        const std::size_t position = text.find(from);
        if (position == std::string::npos) {
            throw std::runtime_error("fixture mutation anchor was not found");
        }
        text.replace(position, from.size(), to);
    }
    WriteText(destination_root / relative, text);
}

WheelRailContactInput MakeStaticInput(
    const IrwWheelRailContact& contact, WheelSide side,
    const ProfileTrackRollTransport& transport = {}) {
    const auto& constants = contact.pose_constants(side);
    WheelRailPoseInput pose_input;
    pose_input.placement = WheelsetPlacement{
        .lateral_meters = 0.0,
        .vertical_meters = -constants.nominal_rolling_radius_meters,
        .roll_radians = 0.0,
        .yaw_radians = 0.0,
    };

    WheelRailContactInput input;
    input.pose = BuildContactPoseScalars(constants, pose_input, transport);
    input.rail_frame.origin_track_meters = Eigen::Vector3d(
        0.0, constants.rail_lateral_datum_meters,
        constants.rail_vertical_datum_meters);
    input.rail_frame.rotation_track_from_profile =
        Eigen::AngleAxisd(constants.rail_roll_radians,
                          Eigen::Vector3d::UnitX())
            .toRotationMatrix();
    input.wheel.origin_track_meters = Eigen::Vector3d(
        0.0, constants.wheel_lateral_datum_meters,
        -constants.nominal_rolling_radius_meters);
    input.wheel.rotation_track_from_profile = Eigen::Matrix3d::Identity();
    input.roll_transport = transport;
    return input;
}

void CheckPersonality(const IrwWheelRailContact& contact,
                      const ResolvedStartupState& startup,
                      const fs::path& data_root) {
    const auto& binding = contact.binding();
    Require(binding.wheel_profile_identifier ==
                    orvd::configuration::kIrwWheelProfileIdentifier &&
                binding.rail_profile_identifier ==
                    orvd::configuration::kIrwRailProfileIdentifier &&
                binding.wheel_rail_contact_strategy_identifier ==
                    orvd::configuration::
                        kIrwWheelRailContactStrategyIdentifier,
            "the three IRW logical identities were not retained");

    const auto& configuration = contact.contact_configuration();
    Require(configuration.geometry.nominal_rolling_radius_meters == 0.43 &&
                configuration.geometry.envelope_bin_width_meters == 2.0e-5 &&
                configuration.geometry.outline_sample_count == 1000 &&
                configuration.geometry.contact_gap_epsilon_meters == 0.0 &&
                configuration.geometry.island_merge_gap_tolerance_meters ==
                    1.0e-6 &&
                configuration.geometry.island_quadrature_stations == 120 &&
                configuration.geometry.rail_cant_radians == 0.024994792,
            "the IRW contact geometry personality is incomplete");
    Require(configuration.material.youngs_modulus_pascals == 210.0e9 &&
                configuration.material.poisson_ratio == 0.28 &&
                configuration.normal.penetration_equivalence_factor ==
                    1.0 / 0.55 &&
                configuration.normal.reference_damping_newton_seconds_per_meter ==
                    100000.0 &&
                configuration.normal.reference_stiffness_newtons_per_meter ==
                    500000000.0 &&
                configuration.normal.damping_activation_penetration_meters ==
                    1.0e-12,
            "the IRW material and normal-force personality is incomplete");
    Require(configuration.creepage.minimum_reference_speed_meters_per_second ==
                    0.01 &&
                configuration.friction.static_coefficient == 0.3 &&
                configuration.friction.limiting_fraction == 0.4 &&
                configuration.friction.decay_per_meter_per_second == 0.55 &&
                configuration.friction.minimum_reference_speed_meters_per_second ==
                    0.01 &&
                configuration.tangential.longitudinal_cells == 21 &&
                configuration.tangential.lateral_strips == 21 &&
                configuration.tangential.refinement_width == 0.01 &&
                configuration.creep_coefficients_outside_table ==
                    OutsideTableRule::kAsymptotic,
            "the IRW creepage, friction or FASTSIM personality is incomplete");

    const auto& preparation =
        contact.wheel_profile_preprocessing_configuration();
    Require(preparation.equal_arc_length_rescan_step_meters == 0.0 &&
                preparation.source_lateral_rediscretisation_step_meters ==
                    0.0001,
            "IRW did not select source-lateral wheel rediscretisation");
    Require(contact.profile_track_roll_transport_strategy().policy() ==
                ProfileTrackRollTransportPolicy::kApplied,
            "IRW did not bind the applied profile/track roll transport");
    Require(contact.track_gauge_meters() == 1.435 &&
                contact.gauge_measuring_depth_meters() == 0.016 &&
                contact.pose_rail_cant_radians() == 0.02499479361892016,
            "the IRW gauge construction inputs were not retained");

    const auto wheel = orvd::configuration::LoadProfilePointsFromJsonFile(
        data_root /
        "vehicle_library/irw/wheel_profiles/irw_reference_wheel_profile.json");
    Require(wheel.size() == 1000 &&
                wheel.identifier() ==
                    orvd::configuration::kIrwWheelProfileIdentifier,
            "the installed S1002 asset did not retain its 1000-point identity");

    const auto right_nodes = contact.model(WheelSide::kRight)
                                 .geometry()
                                 .wheel_node_lateral_meters();
    const auto left_nodes = contact.model(WheelSide::kLeft)
                                .geometry()
                                .wheel_node_lateral_meters();
    Require(right_nodes.size() == 1300 && left_nodes.size() == 1300,
            "the S1002 source-lateral march did not lay 1300 control nodes");
    if (right_nodes.size() == 1300 && left_nodes.size() == 1300) {
        Require(right_nodes.front() == -0.069870003 &&
                    right_nodes.back() == 0.059999999 &&
                    left_nodes.front() == -0.059999999 &&
                    left_nodes.back() == 0.069870003,
                "the source-lateral march lost the authored endpoints");
        RequireClose(right_nodes.back() - right_nodes[1298], 0.000070002,
                     1.0e-15,
                     "the right wheel lost the authored-end remainder");
        RequireClose(left_nodes[1] - left_nodes.front(), 0.000070002,
                     1.0e-15,
                     "the left wheel did not mirror the remainder end");
        for (std::size_t index = 0; index < right_nodes.size(); ++index) {
            Require(left_nodes[index] ==
                        -right_nodes[right_nodes.size() - 1 - index],
                    "the prepared left and right S1002 nodes are not mirrors");
        }
    }

    for (const WheelSide side : {WheelSide::kRight, WheelSide::kLeft}) {
        const auto& constants = contact.pose_constants(side);
        Require(constants.rail_vertical_datum_meters ==
                    startup.rail_profile_reference_vertical_offset_meters &&
                    constants.nominal_rolling_radius_meters == 0.43 &&
                    constants.apply_pitch_correction,
                "an IRW side lost its H3 rail offset or nominal radius");
    }
}

WheelRailContactResult EvaluatePrepared(const IrwWheelRailContact& contact,
                                        WheelSide side,
                                        const WheelRailContactInput& input,
                                        std::size_t* allocations) {
    WheelRailContactWorkspace workspace;
    contact.model(side).PrepareWorkspace(workspace);
    WheelRailContactResult result;
    {
        const AllocationScope scope;
        result = contact.model(side).Evaluate(input, workspace);
        *allocations += scope.allocations();
    }
    return result;
}

void CheckRealEvaluationAndTransport(const IrwWheelRailContact& contact) {
    std::size_t allocations = 0;
    const auto right = EvaluatePrepared(
        contact, WheelSide::kRight,
        MakeStaticInput(contact, WheelSide::kRight), &allocations);
    const auto left = EvaluatePrepared(
        contact, WheelSide::kLeft,
        MakeStaticInput(contact, WheelSide::kLeft), &allocations);
    Require(allocations == 0,
            "a prepared real-profile IRW evaluation called ordinary C++ new");
    Require(right.count == 1 && left.count == 1,
            "the H3 static profile placement did not produce one patch per side");
    if (right.count == 1 && left.count == 1) {
        const auto& right_patch = right.patches[0];
        const auto& left_patch = left.patches[0];
        Require(right_patch.normal.in_contact && left_patch.normal.in_contact &&
                    std::isfinite(right_patch.normal.normal_force_newtons) &&
                    std::isfinite(left_patch.normal.normal_force_newtons) &&
                    right_patch.normal.normal_force_newtons > 0.0 &&
                    left_patch.normal.normal_force_newtons > 0.0,
                "the real S1002/UIC60 pair did not carry finite compression");
        RequireClose(right_patch.normal.normal_force_newtons,
                     left_patch.normal.normal_force_newtons, 1.0e-8,
                     "the static left/right normal forces are not mirrors");
    }

    const Eigen::Vector3d shared_origin(1000.0, 20.0, 3.0);
    const Eigen::Matrix3d shared_rotation = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d body_in_shared(0.03, 0.001, -0.43);
    const Eigen::Vector3d profile_origin =
        shared_origin + Eigen::Vector3d(0.03, 0.0, 0.0);
    const Eigen::Matrix3d profile_rotation =
        Eigen::AngleAxisd(2.0e-5, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
    const ProfileTrackRollTransport correction =
        contact.profile_track_roll_transport_strategy().Compute(
            shared_origin, shared_rotation, body_in_shared, profile_origin,
            profile_rotation);
    Require(correction.roll_offset_radians != 0.0 &&
                correction.lateral_offset_meters != 0.0 &&
                correction.vertical_offset_meters != 0.0,
            "the bound applied transport produced a degenerate correction");

    allocations = 0;
    const auto transported = EvaluatePrepared(
        contact, WheelSide::kRight,
        MakeStaticInput(contact, WheelSide::kRight, correction), &allocations);
    Require(allocations == 0,
            "the applied-transport contact path called ordinary C++ new");
    Require(transported.count == 1 && right.count == 1,
            "the small physical roll transport lost the real contact");
    if (transported.count == 1 && right.count == 1) {
        Require(transported.patches[0].contact_frame_angle_radians !=
                    right.patches[0].contact_frame_angle_radians,
                "the IRW roll transport did not reach the contact frame");
        Require(transported.patches[0]
                    .wrench.rail_on_wheel.force_newtons !=
                    right.patches[0].wrench.rail_on_wheel.force_newtons,
                "the IRW roll transport did not reach the real contact result");
    }
}

void CheckIdentityRefusals(const fs::path& data_root,
                           const ResolvedStartupState& startup,
                           const fs::path& fixture_root) {
    for (const int field : {0, 1, 2}) {
        StartupWheelRailBinding wrong = startup.wheel_rail_binding;
        std::string* identifier =
            field == 0
                ? &wrong.wheel_profile_identifier
                : field == 1
                      ? &wrong.rail_profile_identifier
                      : &wrong.wheel_rail_contact_strategy_identifier;
        *identifier = "another_identity";
        RequireRefusal(
            [&] {
                (void)AssembleIrwWheelRailContact(
                    data_root, wrong,
                    startup.rail_profile_reference_vertical_offset_meters);
            },
            "requires", "an incorrect IRW logical identity was accepted");
    }

    StartupWheelRailBinding path_like = startup.wheel_rail_binding;
    path_like.wheel_profile_identifier = "../wheel_profile";
    RequireRefusal(
        [&] {
            (void)AssembleIrwWheelRailContact(
                data_root, path_like,
                startup.rail_profile_reference_vertical_offset_meters);
        },
        "logical identity", "a path-like IRW logical identity was accepted");

    RequireRefusal(
        [&] {
            (void)AssembleIrwWheelRailContact(
                data_root, startup.wheel_rail_binding,
                std::numeric_limits<double>::quiet_NaN());
        },
        "not finite", "a non-finite IRW rail reference offset was accepted");

    const fs::path wheel_relative =
        "vehicle_library/irw/wheel_profiles/irw_reference_wheel_profile.json";
    const fs::path rail_relative =
        "track_library/rail_profiles/uic60_rail_profile.json";
    fs::remove_all(fixture_root);
    CopyAsset(data_root, fixture_root, wheel_relative,
              "\"profile_identifier\": \"irw_reference_wheel_profile\"",
              "\"profile_identifier\": \"wrong_internal_identity\"");
    CopyAsset(data_root, fixture_root, rail_relative);
    RequireRefusal(
        [&] {
            (void)AssembleIrwWheelRailContact(
                fixture_root, startup.wheel_rail_binding,
                startup.rail_profile_reference_vertical_offset_meters);
        },
        "asset declares profile_identifier",
        "an IRW asset with the wrong internal identity was accepted");

    CopyAsset(data_root, fixture_root, wheel_relative);
    CopyAsset(data_root, fixture_root, rail_relative,
              "\"profile_role\": \"rail\"",
              "\"profile_role\": \"wheel\"");
    RequireRefusal(
        [&] {
            (void)AssembleIrwWheelRailContact(
                fixture_root, startup.wheel_rail_binding,
                startup.rail_profile_reference_vertical_offset_meters);
        },
        "other profile role",
        "an IRW asset with the wrong profile role was accepted");
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::fprintf(stderr,
                     "usage: verify_irw_wheel_rail_contact_binding "
                     "<data-root> <startup.json> <fixture-root>\n");
        return 2;
    }
    try {
        const fs::path data_root = argv[1];
        const ResolvedStartupState startup =
            LoadResolvedStartupStateFromJsonFile(argv[2]);
        const auto contact = AssembleIrwWheelRailContact(
            data_root, startup.wheel_rail_binding,
            startup.rail_profile_reference_vertical_offset_meters);
        CheckPersonality(*contact, startup, data_root);
        CheckRealEvaluationAndTransport(*contact);
        CheckIdentityRefusals(data_root, startup, argv[3]);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW contact binding threw: %s\n", error.what());
        return 1;
    }
    if (failures != 0) {
        return 1;
    }
    std::puts("IRW wheel-rail contact binding verified");
    return 0;
}
