#include "orvd/configuration/load_track_geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace orvd::configuration {
namespace {

using Json = nlohmann::json;
using track_geometry::TrackScalarProfile;
using track_geometry::TrackScalarSegment;
using track_geometry::TrackScalarSegmentShape;
using track_geometry::TrackSeamTransition;

class DuplicateObjectKey final : public std::invalid_argument {
   public:
    explicit DuplicateObjectKey(std::string key)
        : std::invalid_argument("duplicate JSON object key '" + key + "'") {}
};

struct ParseFrame {
    enum class Kind { kObject, kArray };

    Kind kind{};
    std::string path;
    std::unordered_set<std::string> object_keys;
    std::string pending_object_key;
    std::size_t next_array_index{};
};

std::string PendingValuePath(const ParseFrame& frame) {
    if (frame.kind == ParseFrame::Kind::kObject) {
        return frame.path + "." + frame.pending_object_key;
    }
    return frame.path + "[" + std::to_string(frame.next_array_index) + "]";
}

void CompleteValue(ParseFrame& frame) {
    if (frame.kind == ParseFrame::Kind::kObject) {
        frame.pending_object_key.clear();
    } else {
        ++frame.next_array_index;
    }
}

[[noreturn]] void ThrowExpected(const std::string& path,
                                std::string_view expected) {
    throw std::invalid_argument(path + " must be " + std::string(expected));
}

Json ParseStrictJson(const std::string& document) {
    std::vector<ParseFrame> frames;
    const auto reject_duplicate_keys =
        [&frames](int, Json::parse_event_t event, Json& parsed) {
            switch (event) {
                case Json::parse_event_t::object_start:
                case Json::parse_event_t::array_start: {
                    const std::string path =
                        frames.empty() ? "$" : PendingValuePath(frames.back());
                    ParseFrame frame;
                    frame.kind = event == Json::parse_event_t::object_start
                                     ? ParseFrame::Kind::kObject
                                     : ParseFrame::Kind::kArray;
                    frame.path = path;
                    frames.push_back(std::move(frame));
                    break;
                }
                case Json::parse_event_t::key: {
                    const std::string key = parsed.get<std::string>();
                    if (frames.empty() ||
                        frames.back().kind != ParseFrame::Kind::kObject ||
                        !frames.back().object_keys.insert(key).second) {
                        throw DuplicateObjectKey(key);
                    }
                    frames.back().pending_object_key = key;
                    break;
                }
                case Json::parse_event_t::object_end:
                case Json::parse_event_t::array_end:
                    frames.pop_back();
                    if (!frames.empty()) {
                        CompleteValue(frames.back());
                    }
                    break;
                case Json::parse_event_t::value:
                    if (!frames.empty()) {
                        CompleteValue(frames.back());
                    }
                    break;
                default:
                    break;
            }
            return true;
        };

    try {
        return Json::parse(document, reject_duplicate_keys, true, false);
    } catch (const DuplicateObjectKey&) {
        throw;
    } catch (const Json::parse_error& error) {
        throw std::invalid_argument("invalid JSON syntax at byte " +
                                    std::to_string(error.byte) + ": " +
                                    error.what());
    } catch (const Json::out_of_range& error) {
        const std::string path =
            frames.empty() ? "$" : PendingValuePath(frames.back());
        throw std::invalid_argument(
            path + " contains a JSON number that is not representable as a "
                   "finite binary64 value: " +
            error.what());
    }
}

void RequireObject(const Json& value, const std::string& path) {
    if (!value.is_object()) {
        ThrowExpected(path, "a JSON object");
    }
}

void RequireArray(const Json& value, const std::string& path) {
    if (!value.is_array()) {
        ThrowExpected(path, "a JSON array");
    }
}

void RequireExactKeys(const Json& object, const std::string& path,
                      std::initializer_list<std::string_view> expected_keys) {
    RequireObject(object, path);
    for (const auto& [key, unused] : object.items()) {
        (void)unused;
        if (std::find(expected_keys.begin(), expected_keys.end(), key) ==
            expected_keys.end()) {
            throw std::invalid_argument(path + "." + key +
                                        " is an unknown key");
        }
    }
    for (const std::string_view key : expected_keys) {
        if (!object.contains(std::string(key))) {
            throw std::invalid_argument(path + "." + std::string(key) +
                                        " is required");
        }
    }
}

double RequireFiniteNumber(const Json& value, const std::string& path) {
    if (!value.is_number()) {
        ThrowExpected(path, "a finite JSON number");
    }
    double result{};
    try {
        result = value.get<double>();
    } catch (const Json::exception&) {
        ThrowExpected(path, "a finite binary64 number");
    }
    if (!std::isfinite(result)) {
        ThrowExpected(path, "a finite binary64 number");
    }
    return result;
}

std::string RequireString(const Json& value, const std::string& path) {
    if (!value.is_string()) {
        ThrowExpected(path, "a JSON string");
    }
    return value.get<std::string>();
}

std::size_t RequireIndex(const Json& value, const std::string& path) {
    if (!value.is_number_unsigned()) {
        ThrowExpected(path, "a non-negative JSON integer");
    }
    const std::uint64_t result = value.get<std::uint64_t>();
    if (result > std::numeric_limits<std::size_t>::max()) {
        ThrowExpected(path, "an integer representable as size_t");
    }
    return static_cast<std::size_t>(result);
}

struct ProfileFieldNames {
    std::string_view constant_value;
    std::string_view start_value;
    std::string_view end_value;
};

TrackScalarSegment ParseSegment(const Json& value, const std::string& path,
                                const ProfileFieldNames& fields) {
    RequireObject(value, path);
    if (!value.contains("shape")) {
        throw std::invalid_argument(path + ".shape is required");
    }
    const std::string shape = RequireString(value.at("shape"), path + ".shape");

    TrackScalarSegment result;
    if (shape == "constant") {
        RequireExactKeys(value, path,
                         {"shape", "length_meters", fields.constant_value});
        result.shape = TrackScalarSegmentShape::kConstant;
        result.length_meters =
            RequireFiniteNumber(value.at("length_meters"),
                                path + ".length_meters");
        result.start_value =
            RequireFiniteNumber(value.at(std::string(fields.constant_value)),
                                path + "." +
                                    std::string(fields.constant_value));
        result.end_value = result.start_value;
        return result;
    }
    if (shape == "hermite_cubic_blend") {
        RequireExactKeys(value, path,
                         {"shape", "length_meters", fields.start_value,
                          fields.end_value});
        result.shape = TrackScalarSegmentShape::kHermiteCubicBlend;
        result.length_meters =
            RequireFiniteNumber(value.at("length_meters"),
                                path + ".length_meters");
        result.start_value =
            RequireFiniteNumber(value.at(std::string(fields.start_value)),
                                path + "." + std::string(fields.start_value));
        result.end_value =
            RequireFiniteNumber(value.at(std::string(fields.end_value)),
                                path + "." + std::string(fields.end_value));
        return result;
    }
    ThrowExpected(path + ".shape",
                  "'constant' or 'hermite_cubic_blend'");
}

TrackScalarProfile ParseProfile(const Json& value, const std::string& path,
                                double start_track_station_meters,
                                const ProfileFieldNames& fields) {
    RequireExactKeys(value, path, {"segments", "seam_transitions"});
    const Json& segments_json = value.at("segments");
    const Json& seams_json = value.at("seam_transitions");
    RequireArray(segments_json, path + ".segments");
    RequireArray(seams_json, path + ".seam_transitions");

    std::vector<TrackScalarSegment> segments;
    segments.reserve(segments_json.size());
    for (std::size_t index = 0; index < segments_json.size(); ++index) {
        segments.push_back(ParseSegment(
            segments_json[index],
            path + ".segments[" + std::to_string(index) + "]", fields));
    }

    std::vector<TrackSeamTransition> seams;
    seams.reserve(seams_json.size());
    for (std::size_t index = 0; index < seams_json.size(); ++index) {
        const std::string seam_path =
            path + ".seam_transitions[" + std::to_string(index) + "]";
        const Json& seam_json = seams_json[index];
        RequireExactKeys(seam_json, seam_path,
                         {"preceding_segment_index", "window_length_meters"});
        seams.push_back(TrackSeamTransition{
            RequireIndex(seam_json.at("preceding_segment_index"),
                         seam_path + ".preceding_segment_index"),
            RequireFiniteNumber(seam_json.at("window_length_meters"),
                                seam_path + ".window_length_meters")});
    }
    return TrackScalarProfile(start_track_station_meters, std::move(segments),
                              std::move(seams));
}

}  // namespace

track_geometry::TrackGeometry LoadTrackGeometryFromJsonFile(
    const std::filesystem::path& configuration_path) {
    std::ifstream input(configuration_path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open track geometry configuration '" +
                                 configuration_path.string() + "'");
    }
    const std::string document{std::istreambuf_iterator<char>(input),
                               std::istreambuf_iterator<char>()};
    if (input.bad()) {
        throw std::runtime_error("could not read track geometry configuration '" +
                                 configuration_path.string() + "'");
    }

    const Json root = ParseStrictJson(document);
    RequireExactKeys(
        root, "$",
        {"schema_version", "start_track_station_meters",
         "station_node_spacing_meters",
         "rail_reference_lateral_span_meters", "curvature_profile",
         "superelevation_profile", "centerline_upward_grade_profile"});

    if (!root.at("schema_version").is_number_integer() ||
        root.at("schema_version") != 1) {
        ThrowExpected("$.schema_version", "the integer 1");
    }
    const double start_track_station_meters = RequireFiniteNumber(
        root.at("start_track_station_meters"), "$.start_track_station_meters");
    const double station_node_spacing_meters = RequireFiniteNumber(
        root.at("station_node_spacing_meters"),
        "$.station_node_spacing_meters");
    const double rail_reference_lateral_span_meters = RequireFiniteNumber(
        root.at("rail_reference_lateral_span_meters"),
        "$.rail_reference_lateral_span_meters");

    TrackScalarProfile curvature = ParseProfile(
        root.at("curvature_profile"), "$.curvature_profile",
        start_track_station_meters,
        {"curvature_radians_per_meter",
         "start_curvature_radians_per_meter",
         "end_curvature_radians_per_meter"});
    TrackScalarProfile superelevation = ParseProfile(
        root.at("superelevation_profile"), "$.superelevation_profile",
        start_track_station_meters,
        {"superelevation_meters", "start_superelevation_meters",
         "end_superelevation_meters"});
    TrackScalarProfile grade = ParseProfile(
        root.at("centerline_upward_grade_profile"),
        "$.centerline_upward_grade_profile", start_track_station_meters,
        {"centerline_upward_grade", "start_centerline_upward_grade",
         "end_centerline_upward_grade"});

    return track_geometry::TrackGeometry(
        std::move(curvature), std::move(superelevation), std::move(grade),
        rail_reference_lateral_span_meters, station_node_spacing_meters);
}

}  // namespace orvd::configuration
