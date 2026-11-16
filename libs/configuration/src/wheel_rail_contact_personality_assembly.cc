#include "wheel_rail_contact_personality_assembly.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "orvd/configuration/load_profile_points.h"
#include "orvd/configuration/vehicle_contact_record.h"

namespace orvd::configuration::internal {
namespace {

using wheel_rail_contact::ProfilePoints;
using wheel_rail_contact::ProfileRole;
using wheel_rail_contact::RailGaugeDatum;
using wheel_rail_contact::WheelRailContactModel;
using wheel_rail_contact::WheelRailPoseConstants;
using wheel_rail_contact::WheelSide;

[[noreturn]] void Reject(std::string_view vehicle, const std::string& detail) {
    throw std::invalid_argument(std::string(vehicle) +
                                " wheel-rail contact: " + detail);
}

void RequireMatchingLogicalIdentifier(std::string_view actual, std::string_view expected,
                                      std::string_view what, std::string_view vehicle) {
    const bool path_like = actual.empty() || actual.find('/') != actual.npos ||
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
                            "', but the contact record requires '" + std::string(expected) + "'");
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
    const std::filesystem::path& orvd_data_root, const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters, const VehicleContactRecord& record) {
    constexpr std::string_view vehicle = "vehicle";
    const auto& settings = record.computation;
    const wheel_rail_contact::WheelRailContactConfiguration contact_configuration{
        .geometry =
            {
                .nominal_rolling_radius_meters = record.wheel.nominal_rolling_radius_meters,
                .envelope_bin_width_meters = settings.geometry.envelope_bin_width_meters,
                .outline_sample_count = settings.geometry.outline_sample_count,
                .contact_gap_epsilon_meters = settings.geometry.contact_gap_epsilon_meters,
                .island_merge_gap_tolerance_meters =
                    settings.geometry.island_merge_gap_tolerance_meters,
                .island_quadrature_stations = settings.geometry.island_quadrature_stations,
                .rail_cant_radians = record.rail.geometry_rail_cant_radians,
            },
        .normal =
            {
                .material = settings.material,
                .penetration_equivalence_factor = settings.normal.penetration_equivalence_factor,
                .reference_damping_newton_seconds_per_meter =
                    settings.normal.reference_damping_newton_seconds_per_meter,
                .reference_stiffness_newtons_per_meter =
                    settings.normal.reference_stiffness_newtons_per_meter,
                .damping_activation_penetration_meters =
                    settings.normal.damping_activation_penetration_meters,
            },
        .creepage = settings.creepage,
        .friction = settings.friction,
        .tangential =
            {
                .longitudinal_cells = settings.tangential.longitudinal_cells,
                .lateral_strips = settings.tangential.lateral_strips,
                .refinement_width = settings.tangential.refinement_width,
                .material = settings.material,
            },
        .material = settings.material,
        .creep_coefficients_outside_table = settings.creep_coefficients_outside_table,
    };
    if (orvd_data_root.empty()) {
        Reject(vehicle, "the explicitly supplied ORVD data root is empty");
    }
    RequireMatchingLogicalIdentifier(startup_binding.wheel_profile_identifier,
                                     record.wheel.profile.profile_identifier,
                                     "wheel profile identifier", vehicle);
    RequireMatchingLogicalIdentifier(startup_binding.rail_profile_identifier,
                                     record.rail.profile.profile_identifier,
                                     "rail profile identifier", vehicle);
    RequireMatchingLogicalIdentifier(startup_binding.wheel_rail_contact_strategy_identifier,
                                     record.computation.contact_strategy_identifier,
                                     "wheel-rail contact strategy identifier", vehicle);
    if (!std::isfinite(rail_profile_reference_vertical_offset_meters)) {
        Reject(vehicle, "the rail profile reference vertical offset is not finite");
    }

    ProfilePoints wheel_profile =
        LoadProfilePointsFromJsonFile(orvd_data_root / record.wheel.profile.profile_path);
    ProfilePoints rail_profile =
        LoadProfilePointsFromJsonFile(orvd_data_root / record.rail.profile.profile_path);
    RequireLoadedIdentityAndRole(wheel_profile, record.wheel.profile.profile_identifier,
                                 ProfileRole::kWheel, "wheel profile", vehicle);
    RequireLoadedIdentityAndRole(rail_profile, record.rail.profile.profile_identifier,
                                 ProfileRole::kRail, "rail profile", vehicle);

    const RailGaugeDatum right_rail_gauge_datum = wheel_rail_contact::ComputeRailGaugeDatum(
        rail_profile, record.rail.track_gauge_meters, record.rail.gauge_measuring_depth_meters,
        record.rail.pose_rail_cant_radians, WheelSide::kRight);
    const RailGaugeDatum left_rail_gauge_datum = wheel_rail_contact::ComputeRailGaugeDatum(
        rail_profile, record.rail.track_gauge_meters, record.rail.gauge_measuring_depth_meters,
        record.rail.pose_rail_cant_radians, WheelSide::kLeft);
    const double nominal_radius = contact_configuration.geometry.nominal_rolling_radius_meters;
    const WheelRailPoseConstants right_pose_constants = MakePoseConstants(
        right_rail_gauge_datum, WheelSide::kRight, rail_profile_reference_vertical_offset_meters,
        record.wheel.wheel_lateral_datum_magnitude_meters, nominal_radius);
    const WheelRailPoseConstants left_pose_constants = MakePoseConstants(
        left_rail_gauge_datum, WheelSide::kLeft, rail_profile_reference_vertical_offset_meters,
        record.wheel.wheel_lateral_datum_magnitude_meters, nominal_radius);

    auto right_model = std::make_unique<WheelRailContactModel>(
        wheel_profile, rail_profile, WheelSide::kRight, settings.wheel_profile_preprocessing,
        contact_configuration);
    auto left_model = std::make_unique<WheelRailContactModel>(
        wheel_profile, rail_profile, WheelSide::kLeft, settings.wheel_profile_preprocessing,
        contact_configuration);
    auto runtime_personality =
        std::make_unique<wheel_rail_contact::WheelRailContactRuntimePersonality>(
            std::move(right_model), std::move(left_model), right_pose_constants,
            left_pose_constants, record.rail.profile_origin_mode);

    return std::make_unique<BoundWheelRailContact>(BoundWheelRailContact{
        .binding = startup_binding,
        .contact_configuration = contact_configuration,
        .wheel_profile_preprocessing_configuration = settings.wheel_profile_preprocessing,
        .track_gauge_meters = record.rail.track_gauge_meters,
        .gauge_measuring_depth_meters = record.rail.gauge_measuring_depth_meters,
        .pose_rail_cant_radians = record.rail.pose_rail_cant_radians,
        .right_rail_gauge_datum = right_rail_gauge_datum,
        .left_rail_gauge_datum = left_rail_gauge_datum,
        .runtime_personality = std::move(runtime_personality),
    });
}

}  // namespace orvd::configuration::internal
