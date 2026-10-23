#include "orvd/configuration/load_track_irregularity_field.h"

#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "strict_json.h"

namespace orvd::configuration {
namespace {

using strict_json::ElementPath;
using strict_json::Json;
using strict_json::ParseStrictJson;
using strict_json::ReadWholeFile;
using strict_json::RequireArray;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireIdentifier;
using strict_json::RequireObject;
using strict_json::RequireString;
using strict_json::ThrowExpected;

constexpr std::string_view kCoordinateFrame =
    "track_lateral_right_vertical_down";

struct SeriesRecord {
    std::vector<double> track_station_meters;
    std::vector<double> values;
};

std::string RequireCallerIdentifier(std::string_view identifier) {
    return RequireIdentifier(Json(std::string(identifier)),
                             "track_irregularity_identifier",
                             "track irregularity identifier");
}

std::vector<double> RequireFiniteColumn(const Json& root,
                                        std::string_view key) {
    const std::string key_string(key);
    const std::string path = "$." + key_string;
    const Json& array = root.at(key_string);
    RequireArray(array, path);
    std::vector<double> result;
    result.reserve(array.size());
    for (std::size_t index = 0; index < array.size(); ++index) {
        result.push_back(
            RequireFiniteNumber(array[index], ElementPath(path, index)));
    }
    return result;
}

SeriesRecord LoadSeries(const std::filesystem::path& series_directory,
                        const std::string& expected_identifier,
                        std::string_view value_key = "displacement_meters") {
    const std::filesystem::path path =
        series_directory / (expected_identifier + ".json");
    const Json root =
        ParseStrictJson(ReadWholeFile(path, "track irregularity series"));
    RequireExactKeys(root, "$",
                     {"series_identifier", "track_station_meters",
                      value_key});

    const std::string identifier = RequireIdentifier(
        root.at("series_identifier"), "$.series_identifier",
        "track irregularity series identifier");
    if (identifier != expected_identifier) {
        throw std::invalid_argument(
            "$.series_identifier is '" + identifier +
            "', but the field manifest references '" + expected_identifier +
            "'");
    }

    SeriesRecord result;
    result.track_station_meters =
        RequireFiniteColumn(root, "track_station_meters");
    result.values = RequireFiniteColumn(root, value_key);
    if (result.track_station_meters.size() !=
        result.values.size()) {
        throw std::invalid_argument(
            "$.track_station_meters and $." + std::string(value_key) +
            " must have the same length, but there are " +
            std::to_string(result.track_station_meters.size()) + " and " +
            std::to_string(result.values.size()));
    }
    if (result.track_station_meters.size() < 2) {
        throw std::invalid_argument(
            "$.track_station_meters and $." + std::string(value_key) +
            " must each contain at least two values");
    }
    for (std::size_t index = 1; index < result.track_station_meters.size();
         ++index) {
        if (!(result.track_station_meters[index] >
              result.track_station_meters[index - 1])) {
            throw std::invalid_argument(
                ElementPath("$.track_station_meters", index) +
                " must be strictly greater than its predecessor");
        }
    }
    return result;
}

}  // namespace

wheel_rail_contact::TrackIrregularityField
LoadTrackIrregularityFieldFromDataRoot(
    const std::filesystem::path& data_root,
    std::string_view track_irregularity_identifier) {
    if (data_root.empty()) {
        throw std::invalid_argument(
            "track irregularity data root must be supplied explicitly");
    }
    const std::string requested_identifier =
        RequireCallerIdentifier(track_irregularity_identifier);
    const std::filesystem::path field_directory =
        data_root / "track_library" / "irregularities";
    const std::filesystem::path manifest_path =
        field_directory / (requested_identifier + ".json");
    const Json root = ParseStrictJson(
        ReadWholeFile(manifest_path, "track irregularity field manifest"));
    RequireObject(root, "$");
    // Gauge and roll are optional; reject unknown manifest keys.
    Json required_fields = root;
    required_fields.erase("gauge_series_identifier");
    required_fields.erase("roll_series_identifier");
    RequireExactKeys(required_fields, "$",
                     {"track_irregularity_identifier",
                      "coordinate_frame", "lateral_series_identifier",
                      "vertical_series_identifier"});

    const std::string manifest_identifier = RequireIdentifier(
        root.at("track_irregularity_identifier"),
        "$.track_irregularity_identifier", "track irregularity identifier");
    if (manifest_identifier != requested_identifier) {
        throw std::invalid_argument(
            "$.track_irregularity_identifier is '" + manifest_identifier +
            "', but the caller requested '" + requested_identifier + "'");
    }

    const std::string coordinate_frame =
        RequireString(root.at("coordinate_frame"), "$.coordinate_frame");
    if (coordinate_frame != kCoordinateFrame) {
        ThrowExpected("$.coordinate_frame",
                      "'track_lateral_right_vertical_down', the only "
                      "convention this loader implements");
    }

    const std::string lateral_identifier = RequireIdentifier(
        root.at("lateral_series_identifier"),
        "$.lateral_series_identifier",
        "lateral track irregularity series identifier");
    const std::string vertical_identifier = RequireIdentifier(
        root.at("vertical_series_identifier"),
        "$.vertical_series_identifier",
        "vertical track irregularity series identifier");

    const std::filesystem::path series_directory = field_directory / "series";
    SeriesRecord lateral = LoadSeries(series_directory, lateral_identifier);
    SeriesRecord vertical = LoadSeries(series_directory, vertical_identifier);
    SeriesRecord gauge;
    SeriesRecord roll;
    if (root.contains("gauge_series_identifier")) {
        gauge = LoadSeries(
            series_directory,
            RequireIdentifier(root.at("gauge_series_identifier"),
                              "$.gauge_series_identifier",
                              "gauge track irregularity series identifier"));
    }
    if (root.contains("roll_series_identifier")) {
        roll = LoadSeries(
            series_directory,
            RequireIdentifier(root.at("roll_series_identifier"),
                              "$.roll_series_identifier",
                              "roll track irregularity series identifier"),
            "angle_radians");
    }

    return wheel_rail_contact::TrackIrregularityField(
        lateral.track_station_meters, lateral.values,
        vertical.track_station_meters, vertical.values,
        gauge.track_station_meters, gauge.values,
        roll.track_station_meters, roll.values);
}

}  // namespace orvd::configuration
