#include "orvd/track_irregularity/aar_track_irregularity_generator.h"

#include <cstdint>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::track_irregularity::
                           DeriveAarTrackIrregularityChannelSeeds(
                               std::uint64_t{0})),
              orvd::track_irregularity::TrackIrregularityChannelSeeds>);
static_assert(std::is_same_v<
              decltype(orvd::track_irregularity::GenerateAarTrackIrregularity(
                  std::declval<const orvd::track_irregularity::
                                   AarTrackIrregularityGenerationSpec&>())),
              orvd::track_irregularity::GeneratedTrackIrregularity>);
