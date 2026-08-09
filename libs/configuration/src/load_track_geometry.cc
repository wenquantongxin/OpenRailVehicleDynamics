#include "orvd/configuration/load_track_geometry.h"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "strict_json.h"

namespace orvd::configuration {
namespace {

using strict_json::Json;
using strict_json::ParseStrictJson;
using strict_json::RequireArray;
using strict_json::RequireExactKeys;
using strict_json::RequireFiniteNumber;
using strict_json::RequireIndex;
using strict_json::RequireObject;
using strict_json::RequireString;
using strict_json::ThrowExpected;
using track_geometry::TrackScalarProfile;
using track_geometry::TrackScalarSegment;
using track_geometry::TrackScalarSegmentShape;
using track_geometry::TrackSeamTransition;

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
    const std::string document = strict_json::ReadWholeFile(
        configuration_path, "track geometry configuration");

    const Json root = ParseStrictJson(document);
    RequireExactKeys(
        root, "$",
        {"start_track_station_meters",
         "station_node_spacing_meters",
         "superelevation_reference_baselength_meters", "curvature_profile",
         "superelevation_profile", "centerline_upward_grade_profile"});
    const double start_track_station_meters = RequireFiniteNumber(
        root.at("start_track_station_meters"), "$.start_track_station_meters");
    const double station_node_spacing_meters = RequireFiniteNumber(
        root.at("station_node_spacing_meters"),
        "$.station_node_spacing_meters");
    const double superelevation_reference_baselength_meters =
        RequireFiniteNumber(
            root.at("superelevation_reference_baselength_meters"),
            "$.superelevation_reference_baselength_meters");

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
        superelevation_reference_baselength_meters,
        station_node_spacing_meters);
}

}  // namespace orvd::configuration
