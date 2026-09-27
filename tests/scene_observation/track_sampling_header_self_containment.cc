#include "orvd/scene_observation/track_sampling.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::scene_observation::SampleTrackGeometry(
                  std::declval<const orvd::track_geometry::TrackGeometry&>(),
                  std::declval<std::span<const double>>(),
                  orvd::scene_observation::RailDatumPlacement{},
                  orvd::scene_observation::RailDatumPlacement{},
                  std::declval<orvd::scene_observation::TrackSampleTable&>())),
              void>);
