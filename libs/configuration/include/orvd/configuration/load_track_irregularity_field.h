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
// and names one lateral and one vertical series. Each series is read from
//
//   <data_root>/track_library/irregularities/series/<series_identifier>.json
//
// There is no current-directory, environment-variable or source-tree fallback.
// The two series own independent station grids; each grid must be finite and
// strictly increasing, but the grids need not have the same size, knots or
// domain. Their roles come from the manifest keys. The loader does not infer a
// role from the values or attempt to correct a caller that binds the two series
// in the wrong order.
//
// All three documents use the common strict JSON rules. The returned value owns
// both splines and retains neither a DOM nor a reference to any input file.
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
