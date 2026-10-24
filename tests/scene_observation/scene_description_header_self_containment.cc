#include "orvd/scene_observation/scene_description.h"

#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::scene_observation::EncodeSceneDescription(
                  std::declval<const orvd::scene_observation::SceneDescription&>())),
              std::string>);
static_assert(std::is_same_v<
              decltype(orvd::scene_observation::DecodeSceneDescription(
                  std::declval<std::string_view>())),
              orvd::scene_observation::SceneDescription>);
