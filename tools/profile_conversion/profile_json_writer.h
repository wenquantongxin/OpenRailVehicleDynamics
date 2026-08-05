#pragma once

#include <filesystem>

#include "orvd/wheel_rail_contact/profile_points.h"

// Writing the first-party strict JSON profile record.
//
// The product only ever reads this format; nothing on a simulation path writes
// one. The writer therefore lives here, beside the reference-format reader,
// because the one job it has is to be the other end of a conversion a
// researcher runs by hand.
//
// The output is what the product's loader accepts, and the pairing is the
// contract: a file this writes must load, and what loads must mean the same
// thing. That is checked by round trip rather than by comparing text, because
// the text is allowed to change and the meaning is not.

namespace orvd::profile_conversion {

// Writes the profile as a strict JSON document.
//
// Coordinates are written with enough digits to name the double they came from
// exactly, so a round trip through the text loses nothing.
//
// Throws std::runtime_error when the file cannot be written.
void WriteProfilePointsJson(const std::filesystem::path& json_path,
                            const wheel_rail_contact::ProfilePoints& profile);

}  // namespace orvd::profile_conversion
