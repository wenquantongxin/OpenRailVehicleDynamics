#include "orvd/track_irregularity/erri_b176_track_irregularity_generator.h"

#include <cstdint>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::track_irregularity::
                           DeriveErriB176TrackIrregularityChannelSeeds(
                               std::uint64_t{0})),
              orvd::track_irregularity::TrackIrregularityChannelSeeds>);
static_assert(std::is_same_v<
              decltype(orvd::track_irregularity::
                           GenerateErriB176TrackIrregularity(
                               std::declval<const orvd::track_irregularity::
                                                ErriB176TrackIrregularityGenerationSpec&>())),
              orvd::track_irregularity::
                  GeneratedErriB176TrackIrregularity>);
