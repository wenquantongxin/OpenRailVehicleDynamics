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
using track_geometry::CircularVerticalCurveSegment;
using track_geometry::ConstantGradeSegment;
using track_geometry::ParabolicVerticalCurveSegment;
using track_geometry::TrackScalarProfile;
using track_geometry::TrackScalarSegment;
using track_geometry::TrackScalarSegmentShape;
using track_geometry::TrackSeamTransition;
using track_geometry::TrackVerticalProfile;
using track_geometry::TrackVerticalSegment;

struct ProfileFieldNames {
    std::string_view constant_value;
    std::string_view start_value;
    std::string_view end_value;
};

TrackScalarSegment ParseScalarSegment(const Json& value,
                                      const std::string& path,
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

std::vector<TrackSeamTransition> ParseSeamTransitions(
    const Json& value, const std::string& path) {
    RequireArray(value, path);
    std::vector<TrackSeamTransition> seams;
    seams.reserve(value.size());
    for (std::size_t index = 0; index < value.size(); ++index) {
        const std::string seam_path =
            path + "[" + std::to_string(index) + "]";
        const Json& seam_json = value[index];
        RequireExactKeys(seam_json, seam_path,
                         {"preceding_segment_index", "window_length_meters"});
        seams.push_back(TrackSeamTransition{
            RequireIndex(seam_json.at("preceding_segment_index"),
                         seam_path + ".preceding_segment_index"),
            RequireFiniteNumber(seam_json.at("window_length_meters"),
                                seam_path + ".window_length_meters")});
    }
    return seams;
}

TrackScalarProfile ParseScalarProfile(
    const Json& value, const std::string& path,
    double start_track_station_meters, const ProfileFieldNames& fields) {
    RequireExactKeys(value, path, {"segments", "seam_transitions"});
    const Json& segments_json = value.at("segments");
    const Json& seams_json = value.at("seam_transitions");
    RequireArray(segments_json, path + ".segments");

    std::vector<TrackScalarSegment> segments;
    segments.reserve(segments_json.size());
    for (std::size_t index = 0; index < segments_json.size(); ++index) {
        segments.push_back(ParseScalarSegment(
            segments_json[index],
            path + ".segments[" + std::to_string(index) + "]", fields));
    }

    return TrackScalarProfile(start_track_station_meters, std::move(segments),
                              ParseSeamTransitions(
                                  seams_json, path + ".seam_transitions"));
}

TrackVerticalSegment ParseVerticalSegment(const Json& value,
                                          const std::string& path) {
    RequireObject(value, path);
    if (!value.contains("shape")) {
        throw std::invalid_argument(path + ".shape is required");
    }
    const std::string shape = RequireString(value.at("shape"), path + ".shape");
    const auto finite = [&value, &path](std::string_view field) {
        return RequireFiniteNumber(value.at(std::string(field)),
                                   path + "." + std::string(field));
    };
    if (shape == "constant_grade") {
        RequireExactKeys(value, path,
                         {"shape", "length_meters",
                          "centerline_upward_grade"});
        return ConstantGradeSegment{finite("length_meters"),
                                    finite("centerline_upward_grade")};
    }
    if (shape == "parabolic_vertical_curve" ||
        shape == "circular_vertical_curve") {
        RequireExactKeys(value, path,
                         {"shape", "length_meters",
                          "start_centerline_upward_grade",
                          "end_centerline_upward_grade"});
        const double length = finite("length_meters");
        const double start = finite("start_centerline_upward_grade");
        const double end = finite("end_centerline_upward_grade");
        if (shape == "parabolic_vertical_curve") {
            return ParabolicVerticalCurveSegment{length, start, end};
        }
        return CircularVerticalCurveSegment{length, start, end};
    }
    ThrowExpected(path + ".shape",
                  "'constant_grade', 'parabolic_vertical_curve' or "
                  "'circular_vertical_curve'");
}

TrackVerticalProfile ParseVerticalProfile(
    const Json& value, const std::string& path,
    double start_track_station_meters) {
    RequireExactKeys(value, path, {"segments", "seam_transitions"});
    const Json& segments_json = value.at("segments");
    const Json& seams_json = value.at("seam_transitions");
    RequireArray(segments_json, path + ".segments");

    std::vector<TrackVerticalSegment> segments;
    segments.reserve(segments_json.size());
    for (std::size_t index = 0; index < segments_json.size(); ++index) {
        segments.push_back(ParseVerticalSegment(
            segments_json[index],
            path + ".segments[" + std::to_string(index) + "]"));
    }
    return TrackVerticalProfile(
        start_track_station_meters, std::move(segments),
        ParseSeamTransitions(seams_json, path + ".seam_transitions"));
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
         "superelevation_profile", "vertical_profile"});
    const double start_track_station_meters = RequireFiniteNumber(
        root.at("start_track_station_meters"), "$.start_track_station_meters");
    const double station_node_spacing_meters = RequireFiniteNumber(
        root.at("station_node_spacing_meters"),
        "$.station_node_spacing_meters");
    const double superelevation_reference_baselength_meters =
        RequireFiniteNumber(
            root.at("superelevation_reference_baselength_meters"),
            "$.superelevation_reference_baselength_meters");

    TrackScalarProfile curvature = ParseScalarProfile(
        root.at("curvature_profile"), "$.curvature_profile",
        start_track_station_meters,
        {"curvature_radians_per_meter",
         "start_curvature_radians_per_meter",
         "end_curvature_radians_per_meter"});
    TrackScalarProfile superelevation = ParseScalarProfile(
        root.at("superelevation_profile"), "$.superelevation_profile",
        start_track_station_meters,
        {"superelevation_meters", "start_superelevation_meters",
         "end_superelevation_meters"});
    TrackVerticalProfile vertical = ParseVerticalProfile(
        root.at("vertical_profile"), "$.vertical_profile",
        start_track_station_meters);

    return track_geometry::TrackGeometry(
        std::move(curvature), std::move(superelevation), std::move(vertical),
        superelevation_reference_baselength_meters,
        station_node_spacing_meters);
}

}  // namespace orvd::configuration
