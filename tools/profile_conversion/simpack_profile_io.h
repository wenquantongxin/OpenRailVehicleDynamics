#pragma once

#include <filesystem>
#include <string>

#include "orvd/wheel_rail_contact/profile_points.h"

// Reading and writing the profile format the reference multibody tool uses, for
// local research work only.
//
// This is a development-time facility. It is not part of the product library,
// it is not installed, and nothing on the contact evaluation path may reach it.
// The product reads its own strict JSON and nothing else; this exists so that a
// researcher can carry a profile between the two worlds without retyping a
// thousand numbers, and so that a profile prepared here can be handed back to
// the reference tool for a cross-check.
//
// The reader implements exactly the subset the qualified assets use and refuses
// everything else out loud. That is the whole design. The format carries nine
// preprocessing steps — smoothing, decimation, translation, rotation, clipping,
// mirroring, order inversion, unit scaling — and the qualified assets leave all
// but one of them at identity. A reader that silently ignored the rest would be
// correct on exactly those files and would quietly mis-read the first asset
// that used one, which is the failure this project is least able to notice: the
// file parses, the point list is merely wrong.
//
// The one step that is not at identity is the order inversion, which the wheel
// assets do declare. It reverses the row order and is harmless on those files
// only because their rows already ascend in the lateral coordinate. It is
// applied here rather than assumed away, and a file whose rows are not
// monotone after applying it is refused rather than sorted into something
// plausible.

namespace orvd::profile_conversion {

// What the reader keeps beside the point list. These carry no meaning for the
// contact geometry and exist so that a round trip back to the reference tool
// does not lose the file's own measurement metadata.
struct SimpackProfileMetadata {
    // Present only for a wheel profile: the depths below the taping line at
    // which flange width and flange slope are dimensioned.
    double flange_width_measurement_depth_meters{0.0};
    double flange_slope_measurement_depth_meters{0.0};
    std::string comment;
};

struct SimpackProfile {
    wheel_rail_contact::ProfilePoints points;
    SimpackProfileMetadata metadata;
};

// Reads one profile file.
//
// `identifier` is the name the resulting value object will carry; the file's
// own name is not used for it, because a path is not an identity.
//
// Throws std::runtime_error when the file cannot be read. Throws
// std::invalid_argument for any structural error, any unknown key, any
// preprocessing step this reader does not implement, a declared role that
// disagrees with the file extension, a non-monotone point list, or a point row
// that is not two finite numbers.
[[nodiscard]] SimpackProfile ReadSimpackProfile(
    const std::filesystem::path& profile_path, std::string identifier);

// Writes one profile in the canonical minimal form: the full key set at its
// identity values, the declared role, and the point rows ascending.
//
// Byte-level fidelity to any particular source file is not attempted and not
// wanted. What is preserved is the meaning: role, frame, unit, order and the
// points themselves.
//
// Throws std::invalid_argument when the role and file extension disagree or
// the extension is not `.prw`/`.prr`. Throws std::runtime_error when the file
// cannot be written.
void WriteSimpackProfile(const std::filesystem::path& profile_path,
                         const SimpackProfile& profile);

}  // namespace orvd::profile_conversion
