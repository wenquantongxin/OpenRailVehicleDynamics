#pragma once

#include <filesystem>
#include <string>

#include <nlohmann/json.hpp>

#include "orvd/scene_record/scene_assets.h"

namespace orvd::scene_record::internal {

void ValidateTrackSampleTable(const scene_observation::TrackSampleTable& table);
void ValidateSceneAssets(const SceneAssets& assets);
[[nodiscard]] nlohmann::json EncodeSceneAssets(const SceneAssets& assets);
[[nodiscard]] SceneAssets DecodeSceneAssets(
    const nlohmann::json& assets, const std::filesystem::path& directory);
[[nodiscard]] std::string ReadWholeFile(const std::filesystem::path& path);
[[nodiscard]] nlohmann::json ReadSceneDocument(
    const std::filesystem::path& path);
void CreateSceneDirectory(const std::filesystem::path& directory);
void WriteVisualDefinition(const std::filesystem::path& directory,
                           const SceneAssets& assets);
void WriteSceneDocumentLast(const std::filesystem::path& directory,
                            const nlohmann::json& document);

}  // namespace orvd::scene_record::internal
