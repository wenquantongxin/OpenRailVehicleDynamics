// The GZ18 contact personality at its real asset boundary.
//
// The numerical contact laws are tested in wheel_rail_contact. This test checks
// what only configuration can get wrong: logical identities resolving to the
// installed assets, the vehicle's complete typed personality, the gauge datum
// derived from the rail rather than the track reference span, and a real
// profile evaluation that stays allocation-free after preparation.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/configuration/gz18_wheel_rail_contact.h"
#include "orvd/configuration/load_profile_points.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/wheel_rail_contact/wheel_rail_pose.h"
#include "wheel_rail_contact/allocation_probe.h"

namespace {

namespace fs = std::filesystem;

using orvd::configuration::AssembleGz18WheelRailContact;
using orvd::configuration::Gz18WheelRailContact;
using orvd::configuration::LoadResolvedStartupStateFromJsonFile;
using orvd::configuration::ResolvedStartupState;
using orvd::configuration::StartupWheelRailBinding;
using orvd::test::AllocationScope;
using orvd::wheel_rail_contact::BuildContactPoseScalars;
using orvd::wheel_rail_contact::OutsideTableRule;
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
        std::fprintf(stderr, "GZ18 contact binding: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireClose(double actual, double expected, double tolerance,
                  std::string_view what) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::fprintf(stderr,
                     "GZ18 contact binding: %.*s (expected %.17g, got %.17g)\n",
                     static_cast<int>(what.size()), what.data(), expected,
                     actual);
        ++failures;
    }
}

void RequireRefusal(const std::function<void()>& action,
                    std::string_view diagnostic,
                    std::string_view what) {
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

WheelRailContactInput MakeInput(const Gz18WheelRailContact& contact,
                                const ResolvedStartupState& startup,
                                WheelSide side) {
    const auto& constants = contact.pose_constants(side);
    WheelRailPoseInput pose_input;
    pose_input.placement = WheelsetPlacement{
        .lateral_meters = 0.0,
        .vertical_meters = -constants.nominal_rolling_radius_meters,
        .roll_radians = 0.0,
        .yaw_radians = 0.0,
    };
    pose_input.arc_rate_meters_per_second =
        startup.initial_longitudinal_speed_meters_per_second;

    WheelRailContactInput input;
    input.pose = BuildContactPoseScalars(constants, pose_input, {});
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
    input.wheel.origin_velocity_track_meters_per_second = Eigen::Vector3d(
        startup.initial_longitudinal_speed_meters_per_second, 0.0, 0.0);
    const double spin =
        -startup.initial_longitudinal_speed_meters_per_second /
        startup.common_startup_effective_rolling_radius_meters;
    input.wheel.angular_velocity_track_radians_per_second =
        Eigen::Vector3d(0.0, spin, 0.0);
    input.wheel.arc_rate_meters_per_second =
        startup.initial_longitudinal_speed_meters_per_second;
    input.wheel.wheel_pitch_rate_radians_per_second = spin;
    return input;
}

void CheckPersonality(const Gz18WheelRailContact& contact,
                      const ResolvedStartupState& startup,
                      const fs::path& data_root) {
    const auto& binding = contact.binding();
    Require(binding.wheel_profile_identifier ==
                    orvd::configuration::kGz18WheelProfileIdentifier &&
                binding.rail_profile_identifier ==
                    orvd::configuration::kGz18RailProfileIdentifier &&
                binding.wheel_rail_contact_strategy_identifier ==
                    orvd::configuration::kGz18WheelRailContactStrategyIdentifier,
            "the three logical identities were not retained");

    const auto& configuration = contact.contact_configuration();
    Require(configuration.geometry.nominal_rolling_radius_meters == 0.42 &&
                configuration.geometry.envelope_bin_width_meters == 2.0e-5 &&
                configuration.geometry.outline_sample_count == 1000 &&
                configuration.geometry.contact_gap_epsilon_meters == 0.0 &&
                configuration.geometry.island_merge_gap_tolerance_meters ==
                    1.0e-6 &&
                configuration.geometry.island_quadrature_stations == 120 &&
                configuration.geometry.rail_cant_radians == 0.024994792,
            "the GZ18 contact geometry personality is incomplete");
    Require(configuration.material.youngs_modulus_pascals == 210.0e9 &&
                configuration.material.poisson_ratio == 0.28 &&
                configuration.normal.penetration_equivalence_factor ==
                    1.8181818181818181 &&
                configuration.normal.reference_damping_newton_seconds_per_meter ==
                    100000.0 &&
                configuration.normal.reference_stiffness_newtons_per_meter ==
                    500000000.0 &&
                configuration.normal.damping_activation_penetration_meters ==
                    1.0e-12,
            "the GZ18 material and normal-force personality is incomplete");
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
            "the GZ18 creepage, friction or FASTSIM personality is incomplete");
    const auto& preparation =
        contact.wheel_profile_preprocessing_configuration();
    Require(preparation.equal_arc_length_rescan_step_meters == 0.0005 &&
                preparation.source_lateral_rediscretisation_step_meters == 0.0,
            "the GZ18 wheel preparation is not the equal-arc rescan");
    Require(contact.profile_track_roll_transport_strategy().policy() ==
                ProfileTrackRollTransportPolicy::kSuppressed,
            "GZ18 unexpectedly applies the profile/track roll transport");

    Require(contact.track_gauge_meters() == 1.435 &&
                contact.gauge_measuring_depth_meters() == 0.016 &&
                contact.pose_rail_cant_radians() ==
                    0.02499479361892016,
            "the gauge construction inputs were not retained");
    const auto& right_datum = contact.rail_gauge_datum(WheelSide::kRight);
    const auto& left_datum = contact.rail_gauge_datum(WheelSide::kLeft);
    Require(right_datum.lateral_datum_meters > 0.0 &&
                left_datum.lateral_datum_meters < 0.0 &&
                right_datum.roll_radians < 0.0 &&
                left_datum.roll_radians > 0.0 &&
                right_datum.lateral_datum_meters ==
                    -left_datum.lateral_datum_meters &&
                right_datum.gauge_face_offset_meters ==
                    left_datum.gauge_face_offset_meters &&
                right_datum.roll_radians == -left_datum.roll_radians,
            "the two rail gauge datums lost their signed side identity");
    Require(std::abs(right_datum.lateral_datum_meters - 0.75) > 1.0e-3,
            "the gauge datum was replaced by half the track reference span");
    const auto rail = orvd::configuration::LoadProfilePointsFromJsonFile(
        data_root / "track_library/rail_profiles/uic60_rail_profile.json");
    const auto expected_right_datum =
        orvd::wheel_rail_contact::ComputeRailGaugeDatum(
            rail, 1.435, 0.016, 0.02499479361892016,
            WheelSide::kRight);
    const auto expected_left_datum =
        orvd::wheel_rail_contact::ComputeRailGaugeDatum(
            rail, 1.435, 0.016, 0.02499479361892016, WheelSide::kLeft);
    Require(right_datum.gauge_face_offset_meters ==
                    expected_right_datum.gauge_face_offset_meters &&
                right_datum.lateral_datum_meters ==
                    expected_right_datum.lateral_datum_meters &&
                right_datum.roll_radians ==
                    expected_right_datum.roll_radians &&
                left_datum.gauge_face_offset_meters ==
                    expected_left_datum.gauge_face_offset_meters &&
                left_datum.lateral_datum_meters ==
                    expected_left_datum.lateral_datum_meters &&
                left_datum.roll_radians == expected_left_datum.roll_radians,
            "the bundle datum was not derived from the declared gauge inputs");
    const auto deeper_datum = orvd::wheel_rail_contact::ComputeRailGaugeDatum(
        rail, contact.track_gauge_meters(),
        contact.gauge_measuring_depth_meters() + 0.002,
        contact.pose_rail_cant_radians(), WheelSide::kRight);
    Require(std::abs(deeper_datum.lateral_datum_meters -
                     right_datum.lateral_datum_meters) > 1.0e-6,
            "the real rail gauge datum does not respond to measuring depth");

    for (const WheelSide side : {WheelSide::kRight, WheelSide::kLeft}) {
        const auto& constants = contact.pose_constants(side);
        Require(constants.rail_vertical_datum_meters ==
                    startup.rail_profile_reference_vertical_offset_meters &&
                    constants.nominal_rolling_radius_meters == 0.42 &&
                    constants.apply_pitch_correction,
                "a side's pose constants lost the resolved offset or radius");
        Require(contact.model(side).geometry().configuration()
                        .nominal_rolling_radius_meters == 0.42 &&
                    contact.model(side).creep_coefficients().outside_rule() ==
                        OutsideTableRule::kAsymptotic,
                "a side model does not carry the inspected personality");
    }
    Require(contact.pose_constants(WheelSide::kRight)
                    .wheel_lateral_datum_meters > 0.0 &&
                contact.pose_constants(WheelSide::kLeft)
                    .wheel_lateral_datum_meters < 0.0,
            "the wheel pose constants lost their signed side identity");

    const auto right_nodes = contact.model(WheelSide::kRight)
                                 .geometry()
                                 .wheel_node_lateral_meters();
    const auto left_nodes = contact.model(WheelSide::kLeft)
                                .geometry()
                                .wheel_node_lateral_meters();
    Require(right_nodes.size() == left_nodes.size() &&
                right_nodes.size() > 100 && right_nodes.size() < 1000,
            "the real wheel did not pass through equal-arc preprocessing");
    if (right_nodes.size() == left_nodes.size()) {
        for (std::size_t index = 0; index < right_nodes.size(); ++index) {
            Require(left_nodes[index] ==
                        -right_nodes[right_nodes.size() - 1 - index],
                    "the prepared left and right wheel nodes are not mirrors");
        }
    }
}

void CheckRealEvaluation(const Gz18WheelRailContact& contact,
                         const ResolvedStartupState& startup) {
    WheelRailContactWorkspace right_workspace;
    WheelRailContactWorkspace left_workspace;
    contact.model(WheelSide::kRight).PrepareWorkspace(right_workspace);
    contact.model(WheelSide::kLeft).PrepareWorkspace(left_workspace);
    const WheelRailContactInput right_input =
        MakeInput(contact, startup, WheelSide::kRight);
    const WheelRailContactInput left_input =
        MakeInput(contact, startup, WheelSide::kLeft);

    WheelRailContactResult right;
    WheelRailContactResult left;
    std::size_t allocations = 0;
    {
        const AllocationScope scope;
        right = contact.model(WheelSide::kRight).Evaluate(right_input,
                                                          right_workspace);
        left = contact.model(WheelSide::kLeft).Evaluate(left_input,
                                                        left_workspace);
        allocations = scope.allocations();
    }
    Require(allocations == 0,
            "a prepared real-profile contact evaluation allocated storage");
    Require(right.count == 1 && left.count == 1,
            "the resolved GZ18 start produced other than one patch per wheel");
    Require(right.geometric_patch_count == 1 &&
                left.geometric_patch_count == 1 &&
                right.analytic_longitudinal_length_fallback_count == 0 &&
                left.analytic_longitudinal_length_fallback_count == 0,
            "the resolved GZ18 start did not resolve both three-dimensional "
            "contact lengths");
    if (right.count != 1 || left.count != 1) {
        return;
    }

    const auto& right_patch = right.patches[0];
    const auto& left_patch = left.patches[0];
    Require(right_patch.normal.in_contact && left_patch.normal.in_contact &&
                std::isfinite(right_patch.normal.normal_force_newtons) &&
                right_patch.normal.normal_force_newtons > 0.0 &&
                left_patch.normal.normal_force_newtons > 0.0,
            "a real GZ18 patch did not carry a finite compressive force");
    RequireClose(right_patch.geometry.centroid_lateral_meters,
                 -left_patch.geometry.centroid_lateral_meters, 1.0e-12,
                 "the two contact centroids are not mirrors");
    RequireClose(right_patch.normal.normal_force_newtons,
                 left_patch.normal.normal_force_newtons, 1.0e-8,
                 "the two normal forces are not mirrors");
    RequireClose(right_patch.wrench.rail_on_wheel.force_newtons.x(),
                 left_patch.wrench.rail_on_wheel.force_newtons.x(), 1.0e-8,
                 "the mirrored longitudinal forces differ");
    RequireClose(right_patch.wrench.rail_on_wheel.force_newtons.y(),
                 -left_patch.wrench.rail_on_wheel.force_newtons.y(), 1.0e-8,
                 "the mirrored lateral forces do not change sign");
    RequireClose(right_patch.wrench.rail_on_wheel.force_newtons.z(),
                 left_patch.wrench.rail_on_wheel.force_newtons.z(), 1.0e-8,
                 "the mirrored vertical forces differ");

    const double right_support =
        -right_patch.wrench.rail_on_wheel.force_newtons.z();
    const double left_support =
        -left_patch.wrench.rail_on_wheel.force_newtons.z();
    const auto& target = startup.target_wheel_support_forces.front();
    RequireClose(right_support, target.right_support_force_newtons, 0.1,
                 "the resolved right support is not reproduced");
    RequireClose(left_support, target.left_support_force_newtons, 0.1,
                 "the resolved left support is not reproduced");
}

void CheckIdentityRefusals(const fs::path& data_root,
                           const ResolvedStartupState& startup,
                           const fs::path& fixture_root) {
    for (const int field : {0, 1, 2}) {
        StartupWheelRailBinding wrong = startup.wheel_rail_binding;
        std::string* identifier = field == 0
                                      ? &wrong.wheel_profile_identifier
                                      : field == 1
                                            ? &wrong.rail_profile_identifier
                                            : &wrong.wheel_rail_contact_strategy_identifier;
        *identifier = "another_identity";
        RequireRefusal(
            [&] {
                (void)AssembleGz18WheelRailContact(
                    data_root, wrong,
                    startup.rail_profile_reference_vertical_offset_meters);
            },
            "requires", "an incorrect logical identity was accepted");
    }

    StartupWheelRailBinding path_like = startup.wheel_rail_binding;
    path_like.wheel_profile_identifier = "../wheel_profile";
    RequireRefusal(
        [&] {
            (void)AssembleGz18WheelRailContact(
                data_root, path_like,
                startup.rail_profile_reference_vertical_offset_meters);
        },
        "logical identity", "a path-like logical identity was accepted");

    const fs::path wheel_relative =
        "vehicle_library/gz18/wheel_profiles/gz18_reference_wheel_profile.json";
    const fs::path rail_relative =
        "track_library/rail_profiles/uic60_rail_profile.json";
    fs::remove_all(fixture_root);
    CopyAsset(data_root, fixture_root, wheel_relative,
              "\"profile_identifier\": \"gz18_reference_wheel_profile\"",
              "\"profile_identifier\": \"wrong_internal_identity\"");
    CopyAsset(data_root, fixture_root, rail_relative);
    RequireRefusal(
        [&] {
            (void)AssembleGz18WheelRailContact(
                fixture_root, startup.wheel_rail_binding,
                startup.rail_profile_reference_vertical_offset_meters);
        },
        "asset declares profile_identifier",
        "an asset whose internal identity disagrees with its binding was accepted");

    CopyAsset(data_root, fixture_root, wheel_relative);
    CopyAsset(data_root, fixture_root, rail_relative,
              "\"profile_role\": \"rail\"",
              "\"profile_role\": \"wheel\"");
    RequireRefusal(
        [&] {
            (void)AssembleGz18WheelRailContact(
                fixture_root, startup.wheel_rail_binding,
                startup.rail_profile_reference_vertical_offset_meters);
        },
        "other profile role",
        "an asset whose role disagrees with its binding was accepted");
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::fprintf(stderr,
                     "usage: verify_gz18_wheel_rail_contact_binding "
                     "<data-root> <startup.json> <fixture-root>\n");
        return 2;
    }
    try {
        const fs::path data_root = argv[1];
        const ResolvedStartupState startup =
            LoadResolvedStartupStateFromJsonFile(argv[2]);
        const auto contact = AssembleGz18WheelRailContact(
            data_root, startup.wheel_rail_binding,
            startup.rail_profile_reference_vertical_offset_meters);
        CheckPersonality(*contact, startup, data_root);
        CheckRealEvaluation(*contact, startup);
        CheckIdentityRefusals(data_root, startup, argv[3]);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "GZ18 contact binding threw: %s\n", error.what());
        return 1;
    }
    if (failures != 0) {
        return 1;
    }
    std::puts("GZ18 wheel-rail contact binding verified");
    return 0;
}
