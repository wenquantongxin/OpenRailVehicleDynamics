#pragma once

#include <filesystem>
#include <string_view>

#include "orvd/wheel_rail_contact/track_irregularity_field.h"

namespace orvd::configuration {

// Loads one immutable track-irregularity field from the explicit data root and
// logical field identifier supplied by the caller.
//
// The field manifest is read from
//
//   <data_root>/track_library/irregularities/<field_identifier>.json
//
// and names one lateral and one vertical series, with optional gauge and roll
// series. Omitted gauge/roll channels are zero. Each series is read from
//
//   <data_root>/track_library/irregularities/series/<series_identifier>.json
//
// Paths are resolved exclusively under data_root. Each series has an independent
// finite, strictly increasing station grid; manifest keys assign channel roles.
//
// Lateral, vertical and gauge use displacement_meters; positive gauge narrows
// the track. Roll uses angle_radians; positive roll lowers the right rail.
// Documents use the common strict JSON rules. The returned value owns its splines.
//
// Throws std::runtime_error when a required file cannot be read. Throws
// std::invalid_argument for every other refusal, including an invalid or
// mismatched identifier, an unsupported coordinate frame, malformed parallel
// arrays, and the numeric-domain errors reported by TrackIrregularityField.
[[nodiscard]] wheel_rail_contact::TrackIrregularityField
LoadTrackIrregularityFieldFromDataRoot(
    const std::filesystem::path& data_root,
    std::string_view track_irregularity_identifier);

}  // namespace orvd::configuration
