#pragma once

/// Static scene assets shared by live observations and recorded playback.

#include <filesystem>
#include <optional>
#include <string>

#include "orvd/scene_observation/scene_description.h"
#include "orvd/scene_observation/track_sampling.h"

namespace orvd::scene_record {

struct SceneAssets {
    std::optional<scene_observation::TrackSampleTable> track{};
    /// One JSON object, preserved verbatim; empty means absent.
    std::string visual_definition_json;
    std::string world_frame{scene_observation::kSceneWorldFrame};
};

/// Creates a new directory. scene.json is published after all other assets.
void WriteSceneBundle(const std::filesystem::path& directory,
                      const SceneAssets& assets);

[[nodiscard]] SceneAssets ReadSceneBundle(
    const std::filesystem::path& directory);

/// Reads static assets from a record or bundle scene.json, without opening
/// either frame table. A record's description and assets must share a frame.
[[nodiscard]] SceneAssets ReadSceneAssets(
    const std::filesystem::path& scene_json_path);

/// Present selected assets replace base assets; absent ones retain the base.
/// Both sets must name the same world frame.
[[nodiscard]] SceneAssets ComposeSceneAssets(const SceneAssets& base,
                                            const SceneAssets& selected);

}  // namespace orvd::scene_record
