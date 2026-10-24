#include "orvd/scene_record/scene_assets.h"

#include <filesystem>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::scene_record::ReadSceneBundle(
                  std::declval<const std::filesystem::path&>())),
              orvd::scene_record::SceneAssets>);
static_assert(std::is_same_v<
              decltype(orvd::scene_record::ReadSceneAssets(
                  std::declval<const std::filesystem::path&>())),
              orvd::scene_record::SceneAssets>);
