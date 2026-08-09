#pragma once

#include <filesystem>

#include "orvd/wheel_rail_contact/profile_points.h"

namespace orvd::configuration {

// Loads one wheel or rail profile from the exact path the caller supplies and
// immediately constructs the typed value object.
//
// The document states the identifier the asset is known by, which of the two
// surfaces it describes, the coordinate convention its numbers are in, and the
// length unit they are expressed in. Naming the frame and the unit in the file
// is the point: a profile is a bare list of numbers, and a list of numbers that
// does not say what it means is one silent sign convention away from a wheel
// that climbs its own flange.
//
// The two coordinate columns are parallel arrays rather than a list of records.
// A point is genuinely two columns of one measurement and the columns cannot
// carry different meanings for different points, so the usual objection to
// parallel arrays — that two lists of separate things fall out of step — does
// not apply; what does apply is that a thousand points as a thousand objects is
// unreadable, and this file is meant to be read.
//
// The loader rejects duplicate, unknown and missing keys, wrong types,
// non-finite or underflowing floating-point tokens, and integer tokens that
// cannot be converted to binary64 without loss.
// It performs no path search, no environment substitution and no default
// insertion, and retains no JSON document after constructing the result.
//
// Throws std::runtime_error when the file cannot be read. Throws
// std::invalid_argument for invalid JSON or configuration, including the
// conditions ProfilePoints itself refuses.
[[nodiscard]] wheel_rail_contact::ProfilePoints LoadProfilePointsFromJsonFile(
    const std::filesystem::path& configuration_path);

}  // namespace orvd::configuration
