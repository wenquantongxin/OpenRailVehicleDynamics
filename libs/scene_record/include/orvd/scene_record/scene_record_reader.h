#pragma once

/// @file
/// Reads a complete scene record directory back into scene values. The record
/// is first-party output and is read as trusted input: the fields the reader
/// uses must be present and consistent, and anything else is ignored.

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "orvd/scene_observation/scalar_definition.h"
#include "orvd/scene_observation/scene_frame.h"
#include "orvd/scene_observation/scene_topology.h"
#include "orvd/scene_observation/track_sampling.h"

namespace orvd::scene_record {

struct SceneRecord {
    scene_observation::SceneTopology topology;
    std::vector<scene_observation::ScalarDefinition> scalar_definitions;
    /// The visual definition text as copied by the writer; empty when the
    /// record carries none.
    std::string visual_definition_json;
    std::optional<scene_observation::TrackSampleTable> track;
    std::vector<scene_observation::SceneFrame> frames;
};

/// @throws std::runtime_error when the record is incomplete or its tables
/// disagree with the layout stated in scene.json.
[[nodiscard]] SceneRecord ReadSceneRecord(
    const std::filesystem::path& directory);

}  // namespace orvd::scene_record
