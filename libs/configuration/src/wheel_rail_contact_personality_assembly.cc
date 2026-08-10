#include "wheel_rail_contact_personality_assembly.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "orvd/configuration/load_profile_points.h"

namespace orvd::configuration::internal {
namespace {

using wheel_rail_contact::ProfilePoints;
using wheel_rail_contact::ProfileRole;
using wheel_rail_contact::ProfileTrackRollTransportStrategy;
using wheel_rail_contact::RailGaugeDatum;
using wheel_rail_contact::WheelRailContactModel;
using wheel_rail_contact::WheelRailPoseConstants;
using wheel_rail_contact::WheelSide;

[[noreturn]] void Reject(std::string_view vehicle, const std::string& detail) {
    throw std::invalid_argument(std::string(vehicle) +
                                " wheel-rail contact: " + detail);
}

void RequireClosedLogicalIdentifier(std::string_view actual,
                                    std::string_view expected,
                                    std::string_view what,
                                    std::string_view vehicle) {
    const bool path_like =
        actual.empty() || actual.find('/') != actual.npos ||
        actual.find('\\') != actual.npos || actual.find("..") != actual.npos ||
        actual.find("${") != actual.npos ||
        std::filesystem::path(std::string(actual)).is_absolute();
    if (path_like) {
        Reject(vehicle, std::string(what) + " is '" + std::string(actual) +
                            "', but this field is a logical identity and may "
                            "not be an absolute path, contain a path "
                            "separator, '..' or a variable expression");
    }
    if (actual != expected) {
        Reject(vehicle, std::string(what) + " is '" + std::string(actual) +
                            "', but the closed personality requires '" +
                            std::string(expected) + "'");
    }
}

void RequireLoadedIdentityAndRole(const ProfilePoints& profile,
                                  std::string_view expected_identifier,
                                  ProfileRole expected_role,
                                  std::string_view what,
                                  std::string_view vehicle) {
    if (profile.identifier() != expected_identifier) {
        Reject(vehicle,
               std::string(what) + " asset declares profile_identifier '" +
                   profile.identifier() + "', but its binding names '" +
                   std::string(expected_identifier) + "'");
    }
    if (profile.role() != expected_role) {
        Reject(vehicle,
               std::string(what) + " asset declares the other profile role");
    }
}

WheelRailPoseConstants MakePoseConstants(
    const RailGaugeDatum& rail_gauge_datum, WheelSide side,
    double rail_profile_reference_vertical_offset_meters,
    double wheel_lateral_datum_magnitude_meters,
    double nominal_rolling_radius_meters) {
    return WheelRailPoseConstants{
        .rail_lateral_datum_meters = rail_gauge_datum.lateral_datum_meters,
        .rail_vertical_datum_meters =
            rail_profile_reference_vertical_offset_meters,
        .rail_roll_radians = rail_gauge_datum.roll_radians,
        .wheel_lateral_datum_meters =
            side == WheelSide::kRight
                ? wheel_lateral_datum_magnitude_meters
                : -wheel_lateral_datum_magnitude_meters,
        .nominal_rolling_radius_meters = nominal_rolling_radius_meters,
        .apply_pitch_correction = true,
    };
}

}  // namespace

std::unique_ptr<BoundWheelRailContact> AssembleBoundWheelRailContact(
    const std::filesystem::path& orvd_data_root,
    const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters,
    WheelRailContactPersonalitySpecification specification) {
    const std::string_view vehicle = specification.diagnostic_vehicle_name;
    if (vehicle.empty()) {
        throw std::logic_error(
            "wheel-rail contact personality has no diagnostic vehicle name");
    }
    if (orvd_data_root.empty()) {
        Reject(vehicle, "the explicitly supplied ORVD data root is empty");
    }
    RequireClosedLogicalIdentifier(
        startup_binding.wheel_profile_identifier,
        specification.wheel_profile_identifier, "wheel profile identifier",
        vehicle);
    RequireClosedLogicalIdentifier(
        startup_binding.rail_profile_identifier,
        specification.rail_profile_identifier, "rail profile identifier",
        vehicle);
    RequireClosedLogicalIdentifier(
        startup_binding.wheel_rail_contact_strategy_identifier,
        specification.contact_strategy_identifier,
        "wheel-rail contact strategy identifier", vehicle);
    if (!std::isfinite(rail_profile_reference_vertical_offset_meters)) {
        Reject(vehicle,
               "the rail profile reference vertical offset is not finite");
    }

    ProfilePoints wheel_profile = LoadProfilePointsFromJsonFile(
        orvd_data_root / specification.wheel_profile_relative_path);
    ProfilePoints rail_profile = LoadProfilePointsFromJsonFile(
        orvd_data_root / specification.rail_profile_relative_path);
    RequireLoadedIdentityAndRole(
        wheel_profile, specification.wheel_profile_identifier,
        ProfileRole::kWheel, "wheel profile", vehicle);
    RequireLoadedIdentityAndRole(
        rail_profile, specification.rail_profile_identifier,
        ProfileRole::kRail, "rail profile", vehicle);

    const RailGaugeDatum right_rail_gauge_datum =
        wheel_rail_contact::ComputeRailGaugeDatum(
            rail_profile, specification.track_gauge_meters,
            specification.gauge_measuring_depth_meters,
            specification.pose_rail_cant_radians, WheelSide::kRight);
    const RailGaugeDatum left_rail_gauge_datum =
        wheel_rail_contact::ComputeRailGaugeDatum(
            rail_profile, specification.track_gauge_meters,
            specification.gauge_measuring_depth_meters,
            specification.pose_rail_cant_radians, WheelSide::kLeft);
    const double nominal_radius =
        specification.contact_configuration.geometry
            .nominal_rolling_radius_meters;
    const WheelRailPoseConstants right_pose_constants = MakePoseConstants(
        right_rail_gauge_datum, WheelSide::kRight,
        rail_profile_reference_vertical_offset_meters,
        specification.wheel_lateral_datum_magnitude_meters, nominal_radius);
    const WheelRailPoseConstants left_pose_constants = MakePoseConstants(
        left_rail_gauge_datum, WheelSide::kLeft,
        rail_profile_reference_vertical_offset_meters,
        specification.wheel_lateral_datum_magnitude_meters, nominal_radius);

    auto right_model = std::make_unique<WheelRailContactModel>(
        wheel_profile, rail_profile, WheelSide::kRight,
        specification.wheel_profile_preprocessing_configuration,
        specification.contact_configuration);
    auto left_model = std::make_unique<WheelRailContactModel>(
        wheel_profile, rail_profile, WheelSide::kLeft,
        specification.wheel_profile_preprocessing_configuration,
        specification.contact_configuration);
    auto runtime_personality = std::make_unique<
        wheel_rail_contact::WheelRailContactRuntimePersonality>(
        std::move(right_model), std::move(left_model), right_pose_constants,
        left_pose_constants,
        ProfileTrackRollTransportStrategy{specification.roll_transport_policy},
        specification.rail_profile_origin_mode);

    return std::make_unique<BoundWheelRailContact>(BoundWheelRailContact{
        .binding = startup_binding,
        .contact_configuration =
            std::move(specification.contact_configuration),
        .wheel_profile_preprocessing_configuration =
            specification.wheel_profile_preprocessing_configuration,
        .track_gauge_meters = specification.track_gauge_meters,
        .gauge_measuring_depth_meters =
            specification.gauge_measuring_depth_meters,
        .pose_rail_cant_radians = specification.pose_rail_cant_radians,
        .right_rail_gauge_datum = right_rail_gauge_datum,
        .left_rail_gauge_datum = left_rail_gauge_datum,
        .runtime_personality = std::move(runtime_personality),
    });
}

}  // namespace orvd::configuration::internal
