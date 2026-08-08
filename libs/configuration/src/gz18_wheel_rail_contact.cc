#include "orvd/configuration/gz18_wheel_rail_contact.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "orvd/configuration/load_profile_points.h"

namespace orvd::configuration {
namespace {

using wheel_rail_contact::ContactGeometryConfiguration;
using wheel_rail_contact::ContactMaterial;
using wheel_rail_contact::CreepageConfiguration;
using wheel_rail_contact::FrictionLaw;
using wheel_rail_contact::NormalContactConfiguration;
using wheel_rail_contact::OutsideTableRule;
using wheel_rail_contact::ProfilePoints;
using wheel_rail_contact::ProfileRole;
using wheel_rail_contact::ProfileTrackRollTransportPolicy;
using wheel_rail_contact::ProfileTrackRollTransportStrategy;
using wheel_rail_contact::RailGaugeDatum;
using wheel_rail_contact::TangentialContactConfiguration;
using wheel_rail_contact::WheelProfilePreprocessingConfiguration;
using wheel_rail_contact::WheelRailContactConfiguration;
using wheel_rail_contact::WheelRailContactModel;
using wheel_rail_contact::WheelRailPoseConstants;
using wheel_rail_contact::WheelSide;

constexpr double kNominalRollingRadiusMeters = 0.42;
constexpr double kWheelLateralDatumMagnitudeMeters = 0.7465;
constexpr double kTrackGaugeMeters = 1.435;
constexpr double kGaugeMeasuringDepthMeters = 0.016;
// The pose path inherited the exact 1:40 angle. The geometry/force path was
// qualified with the rounded literal below in MakeGz18ContactConfiguration.
// They are two active source values and are deliberately not normalised here.
constexpr double kPoseRailCantRadians = 0.02499479361892016;

constexpr char kWheelProfileRelativePath[] =
    "vehicle_library/gz18/wheel_profiles/gz18_reference_wheel_profile.json";
constexpr char kRailProfileRelativePath[] =
    "track_library/rail_profiles/uic60_rail_profile.json";

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("GZ18 wheel-rail contact: " + detail);
}

void RequireClosedLogicalIdentifier(std::string_view actual,
                                    std::string_view expected,
                                    std::string_view what) {
    const bool path_like = actual.empty() || actual.find('/') != actual.npos ||
                           actual.find('\\') != actual.npos ||
                           actual.find("..") != actual.npos ||
                           actual.find("${") != actual.npos ||
                           std::filesystem::path(std::string(actual)).is_absolute();
    if (path_like) {
        Reject(std::string(what) + " is '" + std::string(actual) +
               "', but this field is a logical identity and may not be an "
               "absolute path, contain a path separator, '..' or a variable "
               "expression");
    }
    if (actual != expected) {
        Reject(std::string(what) + " is '" + std::string(actual) +
               "', but the GZ18 personality requires '" +
               std::string(expected) + "'");
    }
}

void RequireLoadedIdentityAndRole(const ProfilePoints& profile,
                                  std::string_view expected_identifier,
                                  ProfileRole expected_role,
                                  std::string_view what) {
    if (profile.identifier() != expected_identifier) {
        Reject(std::string(what) + " asset declares profile_identifier '" +
               profile.identifier() + "', but its binding names '" +
               std::string(expected_identifier) + "'");
    }
    if (profile.role() != expected_role) {
        Reject(std::string(what) + " asset declares the other profile role");
    }
}

WheelRailContactConfiguration MakeGz18ContactConfiguration() {
    const ContactMaterial material{
        .youngs_modulus_pascals = 210000000000.0,
        .poisson_ratio = 0.28,
    };
    const ContactGeometryConfiguration geometry{
        .nominal_rolling_radius_meters = kNominalRollingRadiusMeters,
        .envelope_bin_width_meters = 2.0e-5,
        .outline_sample_count = 1000,
        .contact_gap_epsilon_meters = 0.0,
        .island_merge_gap_tolerance_meters = 1.0e-6,
        .island_quadrature_stations = 120,
        .rail_cant_radians = 0.024994792,
    };
    const NormalContactConfiguration normal{
        .material = material,
        .penetration_equivalence_factor = 1.8181818181818181,
        .reference_damping_newton_seconds_per_meter = 100000.0,
        .reference_stiffness_newtons_per_meter = 500000000.0,
        .damping_activation_penetration_meters = 1.0e-12,
    };
    const CreepageConfiguration creepage{
        .minimum_reference_speed_meters_per_second = 0.01,
    };
    const FrictionLaw friction{
        .static_coefficient = 0.3,
        .limiting_fraction = 0.4,
        .decay_per_meter_per_second = 0.55,
        .minimum_reference_speed_meters_per_second = 0.01,
    };
    const TangentialContactConfiguration tangential{
        .longitudinal_cells = 21,
        .lateral_strips = 21,
        .refinement_width = 0.01,
        .material = material,
    };
    return WheelRailContactConfiguration{
        .geometry = geometry,
        .normal = normal,
        .creepage = creepage,
        .friction = friction,
        .tangential = tangential,
        .material = material,
        .creep_coefficients_outside_table = OutsideTableRule::kAsymptotic,
    };
}

WheelProfilePreprocessingConfiguration MakeGz18WheelPreparation() {
    return WheelProfilePreprocessingConfiguration{
        .equal_arc_length_rescan_step_meters = 0.0005,
        .source_lateral_rediscretisation_step_meters = 0.0,
    };
}

WheelRailPoseConstants MakePoseConstants(
    const RailGaugeDatum& rail_gauge_datum, WheelSide side,
    double rail_profile_reference_vertical_offset_meters) {
    return WheelRailPoseConstants{
        .rail_lateral_datum_meters = rail_gauge_datum.lateral_datum_meters,
        .rail_vertical_datum_meters =
            rail_profile_reference_vertical_offset_meters,
        .rail_roll_radians = rail_gauge_datum.roll_radians,
        .wheel_lateral_datum_meters =
            side == WheelSide::kRight ? kWheelLateralDatumMagnitudeMeters
                                      : -kWheelLateralDatumMagnitudeMeters,
        .nominal_rolling_radius_meters = kNominalRollingRadiusMeters,
        .apply_pitch_correction = true,
    };
}

}  // namespace

Gz18WheelRailContact::Gz18WheelRailContact(
    StartupWheelRailBinding binding,
    WheelRailContactConfiguration contact_configuration,
    WheelProfilePreprocessingConfiguration
        wheel_profile_preprocessing_configuration,
    double track_gauge_meters, double gauge_measuring_depth_meters,
    double pose_rail_cant_radians, RailGaugeDatum right_rail_gauge_datum,
    RailGaugeDatum left_rail_gauge_datum,
    WheelRailPoseConstants right_pose_constants,
    WheelRailPoseConstants left_pose_constants,
    ProfileTrackRollTransportStrategy profile_track_roll_transport_strategy,
    std::unique_ptr<WheelRailContactModel> right_model,
    std::unique_ptr<WheelRailContactModel> left_model)
    : binding_(std::move(binding)),
      contact_configuration_(std::move(contact_configuration)),
      wheel_profile_preprocessing_configuration_(
          wheel_profile_preprocessing_configuration),
      track_gauge_meters_(track_gauge_meters),
      gauge_measuring_depth_meters_(gauge_measuring_depth_meters),
      pose_rail_cant_radians_(pose_rail_cant_radians),
      right_rail_gauge_datum_(right_rail_gauge_datum),
      left_rail_gauge_datum_(left_rail_gauge_datum),
      right_pose_constants_(right_pose_constants),
      left_pose_constants_(left_pose_constants),
      profile_track_roll_transport_strategy_(
          profile_track_roll_transport_strategy),
      right_model_(std::move(right_model)),
      left_model_(std::move(left_model)) {}

Gz18WheelRailContact::~Gz18WheelRailContact() = default;

std::unique_ptr<Gz18WheelRailContact> AssembleGz18WheelRailContact(
    const std::filesystem::path& orvd_data_root,
    const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters) {
    if (orvd_data_root.empty()) {
        Reject("the explicitly supplied ORVD data root is empty");
    }
    RequireClosedLogicalIdentifier(startup_binding.wheel_profile_identifier,
                                   kGz18WheelProfileIdentifier,
                                   "wheel profile identifier");
    RequireClosedLogicalIdentifier(startup_binding.rail_profile_identifier,
                                   kGz18RailProfileIdentifier,
                                   "rail profile identifier");
    RequireClosedLogicalIdentifier(
        startup_binding.wheel_rail_contact_strategy_identifier,
        kGz18WheelRailContactStrategyIdentifier,
        "wheel-rail contact strategy identifier");
    if (!std::isfinite(rail_profile_reference_vertical_offset_meters)) {
        Reject("the rail profile reference vertical offset is not finite");
    }

    ProfilePoints wheel_profile = LoadProfilePointsFromJsonFile(
        orvd_data_root / kWheelProfileRelativePath);
    ProfilePoints rail_profile = LoadProfilePointsFromJsonFile(
        orvd_data_root / kRailProfileRelativePath);
    RequireLoadedIdentityAndRole(wheel_profile, kGz18WheelProfileIdentifier,
                                 ProfileRole::kWheel, "wheel profile");
    RequireLoadedIdentityAndRole(rail_profile, kGz18RailProfileIdentifier,
                                 ProfileRole::kRail, "rail profile");

    const RailGaugeDatum right_rail_gauge_datum =
        wheel_rail_contact::ComputeRailGaugeDatum(
            rail_profile, kTrackGaugeMeters, kGaugeMeasuringDepthMeters,
            kPoseRailCantRadians, WheelSide::kRight);
    const RailGaugeDatum left_rail_gauge_datum =
        wheel_rail_contact::ComputeRailGaugeDatum(
            rail_profile, kTrackGaugeMeters, kGaugeMeasuringDepthMeters,
            kPoseRailCantRadians, WheelSide::kLeft);
    const WheelRailPoseConstants right_pose_constants = MakePoseConstants(
        right_rail_gauge_datum, WheelSide::kRight,
        rail_profile_reference_vertical_offset_meters);
    const WheelRailPoseConstants left_pose_constants = MakePoseConstants(
        left_rail_gauge_datum, WheelSide::kLeft,
        rail_profile_reference_vertical_offset_meters);

    const WheelRailContactConfiguration contact_configuration =
        MakeGz18ContactConfiguration();
    const WheelProfilePreprocessingConfiguration wheel_preparation =
        MakeGz18WheelPreparation();
    auto right_model = std::make_unique<WheelRailContactModel>(
        wheel_profile, rail_profile, WheelSide::kRight, wheel_preparation,
        contact_configuration);
    auto left_model = std::make_unique<WheelRailContactModel>(
        wheel_profile, rail_profile, WheelSide::kLeft, wheel_preparation,
        contact_configuration);

    return std::unique_ptr<Gz18WheelRailContact>(new Gz18WheelRailContact(
        startup_binding, contact_configuration, wheel_preparation,
        kTrackGaugeMeters, kGaugeMeasuringDepthMeters, kPoseRailCantRadians,
        right_rail_gauge_datum, left_rail_gauge_datum, right_pose_constants,
        left_pose_constants,
        ProfileTrackRollTransportStrategy{
            ProfileTrackRollTransportPolicy::kSuppressed},
        std::move(right_model), std::move(left_model)));
}

}  // namespace orvd::configuration
