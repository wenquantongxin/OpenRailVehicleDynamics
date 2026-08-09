#pragma once

#include <filesystem>

#include "orvd/track_geometry/track_geometry.h"

namespace orvd::configuration {

// Loads one human-authored strict JSON document from the exact path supplied by
// the caller and immediately constructs an immutable TrackGeometry. The parser
// rejects duplicate, unknown and missing keys, wrong types, non-finite or
// underflowing floating-point tokens, and integer tokens that cannot be
// converted to binary64 without loss. Ordinary decimal
// floating-point tokens use binary64 rounding. The loader performs no path
// search, environment substitution or default insertion, and retains no JSON
// DOM after constructing the returned geometry.
//
// Throws std::runtime_error when the file cannot be read. Throws
// std::invalid_argument for invalid JSON or configuration, including the domain
// errors reported directly by TrackScalarProfile and TrackGeometry.
[[nodiscard]] track_geometry::TrackGeometry LoadTrackGeometryFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
