#pragma once

#include <filesystem>

#include "orvd/track_geometry/track_geometry.h"

namespace orvd::configuration {

// Loads one human-authored strict JSON document from the exact path supplied by
// the caller and immediately constructs an immutable TrackGeometry. The parser
// rejects duplicate, unknown and missing keys, wrong types, unsupported schema
// versions and non-representable numbers. It performs no path search,
// environment substitution, default insertion or retained DOM storage.
//
// Throws std::runtime_error when the file cannot be read. Throws
// std::invalid_argument for invalid JSON or configuration, including the domain
// errors reported directly by TrackScalarProfile and TrackGeometry.
[[nodiscard]] track_geometry::TrackGeometry LoadTrackGeometryFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
