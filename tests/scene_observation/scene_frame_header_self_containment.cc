#include "orvd/scene_observation/scene_frame.h"

#include <type_traits>

static_assert(static_cast<int>(
                  orvd::scene_observation::SamplePhase::kInitialAcceptedState) == 0);
static_assert(static_cast<int>(
                  orvd::scene_observation::SamplePhase::kAcceptedEndpoint) == 2);
static_assert(std::is_same_v<
              decltype(orvd::scene_observation::SceneFrameIdentity{}.sample_index),
              std::int64_t>);
