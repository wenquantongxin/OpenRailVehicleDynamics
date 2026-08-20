#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <variant>

#include "orvd/track_irregularity/aar_track_irregularity_generator.h"
#include "orvd/wheel_rail_contact/track_irregularity_field.h"

namespace orvd::configuration {

/// Selects one of the three existing, immutable AAR5/AAR6/ERRI field manifests
/// below the explicit ORVD data root. Any other identifier is rejected. The
/// selected field already contains its historical station placement and must
/// not receive another envelope.
struct FrozenTrackIrregularityFieldSource {
    std::string track_irregularity_identifier;
};

/// Selects one newly generated, finite AAR PSD realization.
struct GeneratedAarTrackIrregularityFieldSource {
    track_irregularity::AarTrackIrregularityGenerationSpec specification;
};

/// Closed source choice authorized by DEC-043. It deliberately does not admit
/// an arbitrary measured-series import or a mutable frozen-field
/// transformation.
using TrackIrregularityFieldSource =
    std::variant<FrozenTrackIrregularityFieldSource,
                 GeneratedAarTrackIrregularityFieldSource>;

/// One field ready for either vehicle assembly, plus generation identity when
/// the selected source was a PSD realization. Frozen fields have no generated
/// metadata because their manifest and point series remain their identity.
struct ResolvedTrackIrregularityField {
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField> field;
    std::optional<track_irregularity::TrackIrregularityGenerationMetadata>
        generated_metadata;
};

/// Resolves exactly one source before vehicle assembly.
///
/// Frozen sources use `data_root` and retain the existing strict manifest
/// loader. Generated sources do no file I/O and use only their explicit
/// specification and seeds. A generated station series is cropped to its two
/// placement boundary knots before spline construction, so the field wrapper
/// provides exact zero displacement and slope outside the active domain.
/// Both paths return the same immutable field type consumed by the IRW and
/// GZ18 scenario factories.
[[nodiscard]] ResolvedTrackIrregularityField ResolveTrackIrregularityField(
    const std::filesystem::path& data_root,
    const TrackIrregularityFieldSource& source);

}  // namespace orvd::configuration
