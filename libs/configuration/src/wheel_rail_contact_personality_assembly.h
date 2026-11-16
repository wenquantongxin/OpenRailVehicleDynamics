#pragma once

#include <filesystem>
#include <memory>
#include <string_view>

#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/wheel_rail_contact/rail_gauge_datum.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_model.h"
#include "orvd/wheel_rail_contact/wheel_rail_contact_runtime_personality.h"

// Private assembly of explicit contact records into immutable side models.
//
// This file deliberately lives below configuration/src rather than in the
// public wheel_rail_contact module. Resolving product identities and installed
// paths is configuration work; the numerical contact core must not learn how
// files are named or where an installation puts them.

namespace orvd::configuration {
struct VehicleContactRecord;
}  // namespace orvd::configuration

namespace orvd::configuration::internal {

inline constexpr std::string_view kGz18ContactRecordRelativePath =
    "vehicle_library/gz18/contact_record.json";
inline constexpr std::string_view kIrwContactRecordRelativePath =
    "vehicle_library/irw/contact_record.json";

// The reference contact records use the same wheel-profile rescan grid.
// Each record states it explicitly; the supplied setting is used for both
// sides so independently constructed side models cannot acquire different grids.

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
    std::unique_ptr<wheel_rail_contact::WheelRailContactRuntimePersonality> runtime_personality;
};

[[nodiscard]] std::unique_ptr<BoundWheelRailContact> AssembleBoundWheelRailContact(
    const std::filesystem::path& orvd_data_root, const StartupWheelRailBinding& startup_binding,
    double rail_profile_reference_vertical_offset_meters, const VehicleContactRecord& record);

}  // namespace orvd::configuration::internal
