#include "orvd/scene_record/scene_record_reader.h"

#include <filesystem>
#include <type_traits>
#include <utility>

static_assert(std::is_same_v<
              decltype(orvd::scene_record::ReadSceneRecord(
                  std::declval<const std::filesystem::path&>())),
              orvd::scene_record::SceneRecord>);
