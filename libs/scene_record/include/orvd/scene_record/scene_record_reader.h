#pragma once

/// @file
/// Reads a complete scene record directory back into scene values. The record
/// is first-party output and is read as trusted input: the fields the reader
/// uses must be present and consistent, and anything else is ignored.

#include <filesystem>
#include <vector>

#include "orvd/scene_observation/scene_description.h"
#include "orvd/scene_observation/scene_frame.h"
#include "orvd/scene_record/scene_assets.h"

namespace orvd::scene_record {

struct SceneRecord {
    scene_observation::SceneDescription description;
    SceneAssets assets;
    std::vector<scene_observation::SceneFrame> frames;
};

/// @throws std::runtime_error when the record is incomplete or its tables
/// disagree with the layout stated in scene.json.
[[nodiscard]] SceneRecord ReadSceneRecord(
    const std::filesystem::path& directory);

}  // namespace orvd::scene_record
