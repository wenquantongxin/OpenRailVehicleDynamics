#include "orvd/scene_record/scene_assets.h"

#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>

#include "scene_assets_json.h"
#include "scene_record_layout.h"

namespace orvd::scene_record {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::runtime_error("scene assets: " + detail);
}

void RequireFinite(double value, const char* what) {
    if (!std::isfinite(value)) {
        Reject(std::string(what) + " is not finite");
    }
}

template <typename Rows>
void RequireFiniteRows(const Rows& rows, const char* what) {
    for (const auto& row : rows) {
        for (double value : row) {
            RequireFinite(value, what);
        }
    }
}

void WriteWholeFile(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::out | std::ios::binary);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output.write(text.data(), static_cast<std::streamsize>(text.size()));
    output.flush();
    if (!output) {
        Reject("could not write '" + path.string() + "'");
    }
    output.close();
    if (!output) {
        Reject("could not close '" + path.string() + "'");
    }
}

nlohmann::json RailDatumJson(scene_observation::RailDatumPlacement datum) {
    return {{"lateral_meters", datum.lateral_meters},
            {"vertical_meters", datum.vertical_meters}};
}

scene_observation::RailDatumPlacement ReadRailDatum(const nlohmann::json& value) {
    return {value.at("lateral_meters").get<double>(),
            value.at("vertical_meters").get<double>()};
}

template <std::size_t N>
std::vector<std::array<double, N>> ReadRows(const nlohmann::json& value) {
    if (!value.is_array()) {
        Reject("track rows must be an array");
    }
    std::vector<std::array<double, N>> rows;
    rows.reserve(value.size());
    for (const auto& row : value) {
        if (!row.is_array() || row.size() != N) {
            Reject("a track array row does not have the expected length");
        }
        rows.push_back(row.get<std::array<double, N>>());
    }
    return rows;
}

SceneAssets ReadAssetsFromDocument(const nlohmann::json& document,
                                  const std::filesystem::path& directory) {
    const auto schema = document.at("schema").get<std::string>();
    if (schema != internal::kRecordIdentifier &&
        schema != internal::kBundleIdentifier) {
        Reject("scene.json is neither an ORVD scene record nor a scene bundle");
    }
    auto assets = internal::DecodeSceneAssets(document.at("assets"), directory);
    if (schema == internal::kRecordIdentifier &&
        document.at("description").at("world_frame").get<std::string>() !=
            assets.world_frame) {
        Reject("description and assets use different world frames");
    }
    return assets;
}

}  // namespace

namespace internal {

void ValidateTrackSampleTable(const scene_observation::TrackSampleTable& table) {
    const std::size_t count = table.stations_meters.size();
    if (count == 0 || table.centerline_in_inertial_meters.size() != count ||
        table.rotation_inertial_from_track_wxyz.size() != count ||
        table.curvature_radians_per_meter.size() != count ||
        table.superelevation_meters.size() != count ||
        table.left_rail_datum_in_inertial_meters.size() != count ||
        table.right_rail_datum_in_inertial_meters.size() != count) {
        Reject("the track table columns disagree in length or are empty");
    }
    for (std::size_t index = 0; index < count; ++index) {
        const double station = table.stations_meters[index];
        RequireFinite(station, "track station");
        if (index != 0 && station <= table.stations_meters[index - 1]) {
            Reject("track stations must be strictly increasing");
        }
    }
    for (double curvature : table.curvature_radians_per_meter) {
        RequireFinite(curvature, "track curvature");
    }
    for (double elevation : table.superelevation_meters) {
        RequireFinite(elevation, "track superelevation");
    }
    RequireFiniteRows(table.centerline_in_inertial_meters, "track centerline");
    RequireFiniteRows(table.rotation_inertial_from_track_wxyz,
                      "track orientation");
    RequireFiniteRows(table.left_rail_datum_in_inertial_meters, "left rail datum");
    RequireFiniteRows(table.right_rail_datum_in_inertial_meters, "right rail datum");
    for (const auto& q : table.rotation_inertial_from_track_wxyz) {
        const double norm_squared =
            q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if (std::abs(norm_squared - 1.0) > 1e-8) {
            Reject("track orientation must be a unit quaternion");
        }
    }
    for (const auto datum : {table.left_rail_datum_in_track,
                            table.right_rail_datum_in_track}) {
        RequireFinite(datum.lateral_meters, "rail datum lateral placement");
        RequireFinite(datum.vertical_meters, "rail datum vertical placement");
    }
}

void ValidateSceneAssets(const SceneAssets& assets) {
    if (assets.world_frame.empty()) {
        Reject("world_frame must not be empty");
    }
    if (assets.track) {
        ValidateTrackSampleTable(*assets.track);
    }
    if (!assets.visual_definition_json.empty()) {
        const auto visual = nlohmann::json::parse(
            assets.visual_definition_json, nullptr, false);
        if (!visual.is_object()) {
            Reject("the visual definition is not one JSON object");
        }
    }
}

nlohmann::json EncodeSceneAssets(const SceneAssets& assets) {
    ValidateSceneAssets(assets);
    nlohmann::json track = nullptr;
    if (assets.track) {
        const auto& table = *assets.track;
        track = {{"stations_meters", table.stations_meters},
                 {"centerline_in_inertial_meters",
                  table.centerline_in_inertial_meters},
                 {"rotation_inertial_from_track_wxyz",
                  table.rotation_inertial_from_track_wxyz},
                 {"curvature_radians_per_meter", table.curvature_radians_per_meter},
                 {"superelevation_meters", table.superelevation_meters},
                 {"left_rail_datum_in_inertial_meters",
                  table.left_rail_datum_in_inertial_meters},
                 {"right_rail_datum_in_inertial_meters",
                  table.right_rail_datum_in_inertial_meters},
                 {"rail_datums",
                  {{"left", RailDatumJson(table.left_rail_datum_in_track)},
                   {"right", RailDatumJson(table.right_rail_datum_in_track)}}}};
    }
    return {{"world_frame", assets.world_frame},
            {"length_unit", scene_observation::kSceneLengthUnit},
            {"angle_unit", scene_observation::kSceneAngleUnit},
            {"track", std::move(track)},
            {"visual_definition_file",
             assets.visual_definition_json.empty()
                 ? nlohmann::json(nullptr)
                 : nlohmann::json(std::string(kVisualDefinitionFileName))}};
}

SceneAssets DecodeSceneAssets(const nlohmann::json& value,
                             const std::filesystem::path& directory) {
    SceneAssets assets;
    assets.world_frame = value.at("world_frame").get<std::string>();
    if (value.at("length_unit") != scene_observation::kSceneLengthUnit ||
        value.at("angle_unit") != scene_observation::kSceneAngleUnit) {
        Reject("unsupported scene asset units");
    }
    const auto& track = value.at("track");
    if (!track.is_null()) {
        scene_observation::TrackSampleTable table;
        table.stations_meters =
            track.at("stations_meters").get<std::vector<double>>();
        table.centerline_in_inertial_meters =
            ReadRows<3>(track.at("centerline_in_inertial_meters"));
        table.rotation_inertial_from_track_wxyz =
            ReadRows<4>(track.at("rotation_inertial_from_track_wxyz"));
        table.curvature_radians_per_meter =
            track.at("curvature_radians_per_meter").get<std::vector<double>>();
        table.superelevation_meters =
            track.at("superelevation_meters").get<std::vector<double>>();
        table.left_rail_datum_in_inertial_meters =
            ReadRows<3>(track.at("left_rail_datum_in_inertial_meters"));
        table.right_rail_datum_in_inertial_meters =
            ReadRows<3>(track.at("right_rail_datum_in_inertial_meters"));
        table.left_rail_datum_in_track =
            ReadRailDatum(track.at("rail_datums").at("left"));
        table.right_rail_datum_in_track =
            ReadRailDatum(track.at("rail_datums").at("right"));
        assets.track = std::move(table);
    }
    const auto& visual = value.at("visual_definition_file");
    if (!visual.is_null()) {
        if (visual.get<std::string>() != kVisualDefinitionFileName) {
            Reject("unsupported visual definition filename");
        }
        assets.visual_definition_json =
            ReadWholeFile(directory / kVisualDefinitionFileName);
        if (assets.visual_definition_json.empty()) {
            Reject("visual definition file is empty");
        }
    }
    ValidateSceneAssets(assets);
    return assets;
}

std::string ReadWholeFile(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input) {
        Reject("could not open '" + path.string() + "'");
    }
    const std::string text(std::istreambuf_iterator<char>(input), {});
    if (input.bad()) {
        Reject("could not read '" + path.string() + "'");
    }
    return text;
}

nlohmann::json ReadSceneDocument(const std::filesystem::path& path) {
    try {
        return nlohmann::json::parse(ReadWholeFile(path));
    } catch (const nlohmann::json::exception& error) {
        Reject("invalid scene document '" + path.string() + "': " + error.what());
    }
}

void CreateSceneDirectory(const std::filesystem::path& directory) {
    std::error_code error;
    if (std::filesystem::exists(directory, error) || error) {
        Reject("'" + directory.string() + "' already exists or cannot be inspected");
    }
    if (!std::filesystem::create_directories(directory, error) || error) {
        Reject("could not create '" + directory.string() + "'");
    }
}

void WriteVisualDefinition(const std::filesystem::path& directory,
                           const SceneAssets& assets) {
    if (!assets.visual_definition_json.empty()) {
        WriteWholeFile(directory / kVisualDefinitionFileName,
                       assets.visual_definition_json);
    }
}

void WriteSceneDocumentLast(const std::filesystem::path& directory,
                            const nlohmann::json& document) {
    const auto temporary = directory / "scene.json.pending";
    WriteWholeFile(temporary, document.dump(2) + '\n');
    std::filesystem::rename(temporary, directory / kSceneFileName);
}

}  // namespace internal

void WriteSceneBundle(const std::filesystem::path& directory,
                      const SceneAssets& assets) {
    const auto encoded_assets = internal::EncodeSceneAssets(assets);
    internal::CreateSceneDirectory(directory);
    internal::WriteVisualDefinition(directory, assets);
    internal::WriteSceneDocumentLast(
        directory, {{"schema", std::string(internal::kBundleIdentifier)},
                    {"assets", encoded_assets}});
}

SceneAssets ReadSceneBundle(const std::filesystem::path& directory) {
    try {
        const auto document =
            internal::ReadSceneDocument(directory / internal::kSceneFileName);
        if (document.at("schema").get<std::string>() != internal::kBundleIdentifier) {
            Reject("scene.json is not an ORVD scene bundle");
        }
        return ReadAssetsFromDocument(document, directory);
    } catch (const nlohmann::json::exception& error) {
        Reject(std::string("invalid scene bundle: ") + error.what());
    }
}

SceneAssets ReadSceneAssets(const std::filesystem::path& scene_json_path) {
    try {
        return ReadAssetsFromDocument(internal::ReadSceneDocument(scene_json_path),
                                      scene_json_path.parent_path());
    } catch (const nlohmann::json::exception& error) {
        Reject(std::string("invalid scene assets: ") + error.what());
    }
}

SceneAssets ComposeSceneAssets(const SceneAssets& base,
                               const SceneAssets& selected) {
    internal::ValidateSceneAssets(base);
    internal::ValidateSceneAssets(selected);
    if (base.world_frame != selected.world_frame) {
        Reject("cannot compose assets from different world frames");
    }
    SceneAssets result = base;
    if (selected.track) {
        result.track = selected.track;
    }
    if (!selected.visual_definition_json.empty()) {
        result.visual_definition_json = selected.visual_definition_json;
    }
    return result;
}

}  // namespace orvd::scene_record
