#pragma once

#include <filesystem>
#include <memory>
#include <string_view>

#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/wheel_rail_contact/rail_gauge_datum.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_model.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_runtime_personality.h"

// Private assembly shared by the closed GZ18 and IRW contact factories.
//
// This file deliberately lives below configuration/src rather than in the
// public wheel_rail_contact module. Resolving product identities and installed
// paths is configuration work; the numerical contact core must not learn how
// files are named or where an installation puts them.

namespace orvd::configuration::internal {

// Both qualified vehicles use the same EEC penetration relation. Keep the
// physical formula as the sole authority rather than preserving insignificant
// last-bit differences between source decimal spellings.
inline constexpr double kEecPenetrationEquivalenceFactor = 1.0 / 0.55;

struct WheelRailContactPersonalitySpecification {
    std::string_view diagnostic_vehicle_name;
    std::string_view wheel_profile_identifier;
    std::string_view rail_profile_identifier;
    std::string_view contact_strategy_identifier;
    std::string_view wheel_profile_relative_path;
    std::string_view rail_profile_relative_path;

    wheel_rail_contact::WheelRailContactConfiguration contact_configuration;
    double track_gauge_meters;
    double gauge_measuring_depth_meters;
    double pose_rail_cant_radians;
    double wheel_lateral_datum_magnitude_meters;
    wheel_rail_contact::RailProfileOriginMode rail_profile_origin_mode;
};

// Everything a source-facing vehicle factory retains after resolving its
// logical identities. The point-list objects are intentionally absent: each
// side model already owns the prepared surface it built from them.
struct BoundWheelRailContact {
    StartupWheelRailBinding binding;
    wheel_rail_contact::WheelRailContactConfiguration contact_configuration;
    wheel_rail_contact::WheelProfilePreprocessingConfiguration
        wheel_profile_preprocessing_configuration;
    double track_gauge_meters;
    double gauge_measuring_depth_meters;
    double pose_rail_cant_radians;
    wheel_rail_contact::RailGaugeDatum right_rail_gauge_datum;
    wheel_rail_contact::RailGaugeDatum left_rail_gauge_datum;
    std::unique_ptr<wheel_rail_contact::WheelRailContactRuntimePersonality>
        runtime_personality;
};

[[nodiscard]] std::unique_ptr<BoundWheelRailContact>
AssembleBoundWheelRailContact(
    const std::filesystem::path& orvd_data_root,
    const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters,
    WheelRailContactPersonalitySpecification specification);

}  // namespace orvd::configuration::internal
