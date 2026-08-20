#include "orvd/configuration/resolve_track_irregularity_field.h"

#include <filesystem>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::configuration::ResolveTrackIrregularityField(
                  std::declval<const std::filesystem::path&>(),
                  std::declval<const orvd::configuration::
                                   TrackIrregularityFieldSource&>())),
              orvd::configuration::ResolvedTrackIrregularityField>);
