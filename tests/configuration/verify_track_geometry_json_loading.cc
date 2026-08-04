#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/configuration/load_track_geometry.h"

namespace {

using orvd::configuration::LoadTrackGeometryFromJsonFile;

constexpr std::string_view kNondegenerateRecord = R"json({
  "schema_version": 1,
  "start_track_station_meters": 7.0,
  "station_node_spacing_meters": 0.7,
  "rail_reference_lateral_span_meters": 1.6,
  "curvature_profile": {
    "segments": [
      {
        "shape": "constant",
        "length_meters": 10.0,
        "curvature_radians_per_meter": 0.0
      },
      {
        "shape": "hermite_cubic_blend",
        "length_meters": 20.0,
        "start_curvature_radians_per_meter": 0.0,
        "end_curvature_radians_per_meter": 0.01
      },
      {
        "shape": "constant",
        "length_meters": 10.0,
        "curvature_radians_per_meter": 0.01
      }
    ],
    "seam_transitions": [
      {"preceding_segment_index": 0, "window_length_meters": 2.0}
    ]
  },
  "superelevation_profile": {
    "segments": [
      {
        "shape": "hermite_cubic_blend",
        "length_meters": 40.0,
        "start_superelevation_meters": 0.0,
        "end_superelevation_meters": 0.08
      }
    ],
    "seam_transitions": []
  },
  "centerline_upward_grade_profile": {
    "segments": [
      {
        "shape": "constant",
        "length_meters": 10.0,
        "centerline_upward_grade": 0.02
      },
      {
        "shape": "hermite_cubic_blend",
        "length_meters": 20.0,
        "start_centerline_upward_grade": 0.02,
        "end_centerline_upward_grade": 0.04
      },
      {
        "shape": "constant",
        "length_meters": 10.0,
        "centerline_upward_grade": 0.04
      }
    ],
    "seam_transitions": []
  }
})json";

void Require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

bool Near(double measured, double expected, double tolerance = 1.0e-12) {
    return std::abs(measured - expected) <= tolerance;
}

void Write(const std::filesystem::path& path, std::string_view contents) {
    std::ofstream output(path, std::ios::binary);
    if (!output || !(output << contents)) {
        throw std::runtime_error("could not write test configuration");
    }
}

std::string ReplaceOnce(std::string source, std::string_view needle,
                        std::string_view replacement) {
    const std::size_t position = source.find(needle);
    if (position == std::string::npos ||
        source.find(needle, position + needle.size()) != std::string::npos) {
        throw std::runtime_error("test mutation did not identify one substring: " +
                                 std::string(needle));
    }
    source.replace(position, needle.size(), replacement);
    return source;
}

void ExpectInvalid(const std::filesystem::path& path, std::string contents,
                   const std::vector<std::string>& diagnostic_fragments) {
    Write(path, contents);
    try {
        static_cast<void>(LoadTrackGeometryFromJsonFile(path));
    } catch (const std::invalid_argument& error) {
        for (const std::string& fragment : diagnostic_fragments) {
            Require(std::string_view(error.what()).find(fragment) !=
                        std::string_view::npos,
                    "configuration was rejected at the wrong diagnostic layer");
        }
        return;
    }
    throw std::runtime_error("invalid configuration was accepted");
}

void CheckRealAsset(const std::filesystem::path& asset_path) {
    const auto geometry = LoadTrackGeometryFromJsonFile(asset_path);
    Require(geometry.start_track_station_meters() == 0.0,
            "real asset start station changed");
    Require(geometry.end_track_station_meters() == 2000.0,
            "real asset end station changed");
    Require(geometry.rail_reference_lateral_span_meters() == 1.5,
            "real asset reference span changed");
    constexpr double kStation = 123.5;
    const auto point =
        geometry.CenterlinePositionInInertialMeters(kStation);
    Require(point.x() == kStation && point.y() == 0.0 && point.z() == 0.0,
            "real asset is not straight and level");
    Require(geometry.CurvatureRadiansPerMeter(kStation) == 0.0 &&
                geometry.SuperelevationMeters(kStation) == 0.0 &&
                geometry.CenterlineUpwardGrade(kStation) == 0.0,
            "real asset contains an unexpected active profile");
}

void CheckNondegenerateRecordAndLifetime(
    const std::filesystem::path& configuration_path) {
    Write(configuration_path, kNondegenerateRecord);
    auto geometry = LoadTrackGeometryFromJsonFile(configuration_path);
    Require(std::filesystem::remove(configuration_path),
            "test configuration was not deleted after loading");

    Require(geometry.start_track_station_meters() == 7.0 &&
                geometry.end_track_station_meters() == 47.0,
            "record domain was not mapped");
    Require(geometry.rail_reference_lateral_span_meters() == 1.6,
            "record reference span was not mapped");
    // A Hermite blend has the same value after swapping its ends at the exact
    // midpoint. Non-midpoint probes make the three field mappings directional.
    Require(Near(geometry.CurvatureRadiansPerMeter(22.0), 0.0015625),
            "Hermite curvature segment direction was not mapped");
    Require(Near(geometry.SuperelevationMeters(17.0), 0.0125),
            "Hermite superelevation segment direction was not mapped");
    Require(Near(geometry.CenterlineUpwardGrade(22.0), 0.023125),
            "Hermite grade segment direction was not mapped");
    Require(std::abs(geometry.CurvatureRadiansPerMeter(16.5)) > 1.0e-15,
            "declared seam transition did not enter the profile");

    const auto point = geometry.CenterlinePositionInInertialMeters(12.0);
    Require(Near(point.x(), 5.0) && point.y() == 0.0 && Near(point.z(), -0.1),
            "loaded geometry depends on the deleted input file");
}

void CheckStrictRejections(const std::filesystem::path& path) {
    const std::string valid(kNondegenerateRecord);
    const std::string invalid_syntax = ReplaceOnce(
        valid, "\"schema_version\": 1,", "\"schema_version\": 1,,");
    const std::size_t invalid_byte = invalid_syntax.find(",,") + 2;
    ExpectInvalid(path, invalid_syntax,
                  {"invalid JSON syntax at byte " +
                       std::to_string(invalid_byte),
                   "parse_error"});

    struct RejectionCase {
        std::string contents;
        std::vector<std::string> fragments;
    };
    const std::vector<RejectionCase> cases{
        {ReplaceOnce(valid, "\"schema_version\": 1,",
                     "\"schema_version\": 1, \"schema_version\": 1,"),
         {"duplicate JSON object key at $.schema_version"}},
        {ReplaceOnce(valid,
                     "\"curvature_radians_per_meter\": 0.0\n      }",
                     "\"curvature_radians_per_meter\": 0.0, "
                     "\"curvature_radians_per_meter\": 0.0\n      }"),
         {"duplicate JSON object key at "
          "$.curvature_profile.segments[0].curvature_radians_per_meter"}},
        {ReplaceOnce(valid,
                     "\"curvature_radians_per_meter\": 0.0\n      }",
                     "\"curvature_radians_per_meter\": 0.0,\n"
                     "        \"mystery\": 3\n      }"),
         {"$.curvature_profile.segments[0].mystery"}},
        {ReplaceOnce(valid,
                     ",\n        \"end_superelevation_meters\": 0.08",
                     ""),
         {"$.superelevation_profile.segments[0].end_superelevation_meters"}},
        {ReplaceOnce(valid, "\"station_node_spacing_meters\": 0.7",
                     "\"station_node_spacing_meters\": \"fine\""),
         {"$.station_node_spacing_meters"}},
        {ReplaceOnce(valid,
                     "\"rail_reference_lateral_span_meters\": 1.6",
                     "\"rail_reference_lateral_span_meters\": 1e400"),
         {"$.rail_reference_lateral_span_meters", "not representable"}},
        {ReplaceOnce(valid, "\"start_track_station_meters\": 7.0",
                     "\"start_track_station_meters\": 1e-400"),
         {"$.start_track_station_meters", "underflows binary64"}},
        {ReplaceOnce(valid, "\"start_track_station_meters\": 7.0",
                     "\"start_track_station_meters\": 9007199254740993"),
         {"$.start_track_station_meters", "exactly representable"}},
        {ReplaceOnce(valid, "\"schema_version\": 1",
                     "\"schema_version\": 2"),
         {"$.schema_version"}},
        {ReplaceOnce(valid,
                     "\"rail_reference_lateral_span_meters\": 1.6",
                     "\"rail_reference_lateral_span_meters\": 0.0"),
         {"TrackGeometry:"}},
    };
    for (const auto& rejection : cases) {
        ExpectInvalid(path, rejection.contents, rejection.fragments);
    }

    std::string nul_terminated_prefix = valid;
    nul_terminated_prefix.push_back('\0');
    nul_terminated_prefix += valid;
    ExpectInvalid(path, std::move(nul_terminated_prefix),
                  {"NUL byte", "before the end of the file"});
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 3) {
            throw std::invalid_argument(
                "expected the real asset path and a scratch directory");
        }
        const std::filesystem::path scratch = argv[2];
        std::filesystem::remove_all(scratch);
        std::filesystem::create_directories(scratch);
        const std::filesystem::path temporary_configuration =
            scratch / "track_geometry.json";

        CheckRealAsset(argv[1]);
        CheckNondegenerateRecordAndLifetime(temporary_configuration);
        CheckStrictRejections(temporary_configuration);
        std::filesystem::remove_all(scratch);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "track geometry JSON loading failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
