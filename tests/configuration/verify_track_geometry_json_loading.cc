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
  "start_track_station_meters": 7.0,
  "station_node_spacing_meters": 0.7,
  "superelevation_reference_baselength_meters": 1.6,
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
  "vertical_profile": {
    "segments": [
      {
        "shape": "constant_grade",
        "length_meters": 10.0,
        "centerline_upward_grade": 0.02
      },
      {
        "shape": "parabolic_vertical_curve",
        "length_meters": 10.0,
        "start_centerline_upward_grade": 0.02,
        "end_centerline_upward_grade": 0.03
      },
      {
        "shape": "circular_vertical_curve",
        "length_meters": 10.0,
        "start_centerline_upward_grade": 0.03,
        "end_centerline_upward_grade": 0.04
      },
      {
        "shape": "constant_grade",
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

double RawCurveTransitionFraction(double track_station_meters) {
    const auto hermite = [](double fraction) {
        return 3.0 * fraction * fraction -
               2.0 * fraction * fraction * fraction;
    };
    if (track_station_meters < 50.0) {
        return 0.0;
    }
    if (track_station_meters < 100.0) {
        return hermite((track_station_meters - 50.0) / 50.0);
    }
    if (track_station_meters < 600.0) {
        return 1.0;
    }
    if (track_station_meters < 650.0) {
        return 1.0 - hermite((track_station_meters - 600.0) / 50.0);
    }
    return 0.0;
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
    Require(geometry.superelevation_reference_baselength_meters() == 1.5,
            "real asset superelevation reference baselength changed");
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

void CheckR300QualificationAsset(const std::filesystem::path& asset_path) {
    const auto geometry = LoadTrackGeometryFromJsonFile(asset_path);
    Require(geometry.start_track_station_meters() == 0.0 &&
                geometry.end_track_station_meters() == 1100.0,
            "R300 asset has the wrong station interval");
    Require(geometry.superelevation_reference_baselength_meters() == 1.5,
            "R300 asset has the wrong superelevation reference baselength");
    Require(geometry.first_curved_track_station_meters() == 48.5 &&
                geometry.first_superelevated_track_station_meters() == 48.5 &&
                !geometry.first_graded_track_station_meters().has_value(),
            "R300 asset does not begin its first three-metre C2 seam at the "
            "declared station");

    constexpr double kCircularCurvature = 1.0 / 300.0;
    Require(geometry.CurvatureRadiansPerMeter(25.0) == 0.0 &&
                geometry.SuperelevationMeters(25.0) == 0.0,
            "R300 asset is not straight before the transition");
    Require(Near(geometry.CurvatureRadiansPerMeter(75.0),
                 0.5 * kCircularCurvature) &&
                Near(geometry.SuperelevationMeters(75.0), 0.06),
            "R300 entry transition does not follow its Hermite definition");
    Require(geometry.CurvatureRadiansPerMeter(150.0) ==
                    kCircularCurvature &&
                geometry.SuperelevationMeters(150.0) == 0.12 &&
                Near(geometry.TrackRollRadians(150.0),
                     std::asin(0.12 / 1.5)),
            "R300 circular segment does not carry the qualified curvature and "
            "centerline-based superelevation");
    Require(Near(geometry.CurvatureRadiansPerMeter(625.0),
                 0.5 * kCircularCurvature) &&
                Near(geometry.SuperelevationMeters(625.0), 0.06),
            "R300 exit transition does not follow its Hermite definition");
    Require(geometry.CurvatureRadiansPerMeter(900.0) == 0.0 &&
                geometry.SuperelevationMeters(900.0) == 0.0,
            "R300 asset is not straight after the transition");

    // Each segment boundary carries the declared three-metre quintic overlay.
    // A point inside every window must therefore differ from the raw
    // constant/Hermite segment value. This checks all four declarations rather
    // than inferring them from the first support station.
    for (const double station : {49.0, 99.0, 599.0, 649.0}) {
        const double raw_fraction = RawCurveTransitionFraction(station);
        Require(std::abs(geometry.CurvatureRadiansPerMeter(station) -
                         raw_fraction * kCircularCurvature) > 1.0e-10 &&
                    std::abs(geometry.SuperelevationMeters(station) -
                             raw_fraction * 0.12) > 1.0e-9,
                "R300 asset lost one of its four three-metre C2 seams");
    }

    for (const double station : {25.0, 75.0, 150.0, 625.0, 900.0}) {
        const auto point =
            geometry.CenterlinePositionInInertialMeters(station);
        Require(point.allFinite() && point.z() == 0.0,
                "R300 centerline is not finite and level");
        const auto projection = geometry.ProjectPointNearSeed(
            point, station + 0.2, 1.0);
        Require(Near(projection.track_station_meters(), station, 2.0e-12),
                "R300 centerline point did not project to its own station");
    }
}

void CheckImportedCurveAsset(const std::filesystem::path& asset_path,
                             double expected_end_station_meters,
                             double radius_meters,
                             double superelevation_meters,
                             bool ends_in_circular_segment) {
    const auto geometry = LoadTrackGeometryFromJsonFile(asset_path);
    Require(geometry.start_track_station_meters() == 0.0 &&
                geometry.end_track_station_meters() ==
                    expected_end_station_meters,
            "imported curve asset has the wrong station interval");
    Require(geometry.superelevation_reference_baselength_meters() == 1.5,
            "imported curve asset has the wrong superelevation reference "
            "baselength");
    Require(geometry.first_curved_track_station_meters() == 48.5 &&
                geometry.first_superelevated_track_station_meters() == 48.5 &&
                !geometry.first_graded_track_station_meters().has_value(),
            "imported curve asset has the wrong first seam support");

    const double circular_curvature = 1.0 / radius_meters;
    Require(geometry.CurvatureRadiansPerMeter(25.0) == 0.0 &&
                geometry.SuperelevationMeters(25.0) == 0.0 &&
                Near(geometry.CurvatureRadiansPerMeter(75.0),
                     0.5 * circular_curvature) &&
                Near(geometry.SuperelevationMeters(75.0),
                     0.5 * superelevation_meters) &&
                geometry.CurvatureRadiansPerMeter(150.0) ==
                    circular_curvature &&
                geometry.SuperelevationMeters(150.0) ==
                    superelevation_meters,
            "imported curve asset changed its entry or circular segment");
    if (ends_in_circular_segment) {
        Require(geometry.CurvatureRadiansPerMeter(
                    expected_end_station_meters - 25.0) ==
                        circular_curvature &&
                    geometry.SuperelevationMeters(
                        expected_end_station_meters - 25.0) ==
                        superelevation_meters &&
                    geometry.CenterlineUpwardGrade(
                        expected_end_station_meters - 25.0) == 0.0,
                "truncated curve asset does not end in its circular segment");
    } else {
        Require(Near(geometry.CurvatureRadiansPerMeter(625.0),
                     0.5 * circular_curvature) &&
                    Near(geometry.SuperelevationMeters(625.0),
                         0.5 * superelevation_meters) &&
                    geometry.CurvatureRadiansPerMeter(
                        expected_end_station_meters - 100.0) == 0.0 &&
                    geometry.SuperelevationMeters(
                        expected_end_station_meters - 100.0) == 0.0 &&
                    geometry.CenterlineUpwardGrade(
                        expected_end_station_meters - 100.0) == 0.0,
                "qualification curve asset does not end straight and level");
    }

    const auto check_seam = [&](double station) {
        const double raw_fraction = RawCurveTransitionFraction(station);
        Require(std::abs(geometry.CurvatureRadiansPerMeter(station) -
                         raw_fraction * circular_curvature) > 1.0e-10 &&
                    std::abs(geometry.SuperelevationMeters(station) -
                             raw_fraction * superelevation_meters) > 1.0e-9,
                "imported curve asset lost a three-metre seam");
    };
    for (const double station : {49.0, 99.0}) {
        check_seam(station);
    }
    if (!ends_in_circular_segment) {
        for (const double station : {599.0, 649.0}) {
            check_seam(station);
        }
    }

    const auto centerline =
        geometry.CenterlinePositionInInertialMeters(150.0);
    Require(centerline.allFinite() && centerline.z() == 0.0,
            "imported curve centerline is not finite and level");
    const auto projection =
        geometry.ProjectPointNearSeed(centerline, 150.2, 1.0);
    Require(Near(projection.track_station_meters(), 150.0, 2.0e-12),
            "imported curve centerline did not project to its own station");
}

void CheckQualificationStraightAsset(const std::filesystem::path& asset_path) {
    const auto geometry = LoadTrackGeometryFromJsonFile(asset_path);
    Require(geometry.start_track_station_meters() == 0.0 &&
                geometry.end_track_station_meters() == 1100.0 &&
                geometry.superelevation_reference_baselength_meters() == 1.5,
            "qualification straight asset has the wrong domain or baselength");
    constexpr double kStation = 1000.0;
    const auto point =
        geometry.CenterlinePositionInInertialMeters(kStation);
    Require(point.x() == kStation && point.y() == 0.0 && point.z() == 0.0 &&
                geometry.CurvatureRadiansPerMeter(kStation) == 0.0 &&
                geometry.SuperelevationMeters(kStation) == 0.0 &&
                geometry.CenterlineUpwardGrade(kStation) == 0.0,
            "qualification straight asset is not straight and level");
}

void CheckChunshenStationAsset(const std::filesystem::path& asset_path) {
    const auto geometry = LoadTrackGeometryFromJsonFile(asset_path);
    Require(Near(geometry.start_track_station_meters(), 0.0) &&
                Near(geometry.end_track_station_meters(), 18676.364),
            "Chunshen Station asset has the wrong common station interval");
    Require(geometry.superelevation_reference_baselength_meters() == 1.5,
            "Chunshen Station asset has the wrong superelevation reference "
            "baselength");

    const auto first_curved = geometry.first_curved_track_station_meters();
    const auto first_superelevated =
        geometry.first_superelevated_track_station_meters();
    const auto first_graded = geometry.first_graded_track_station_meters();
    Require(first_curved.has_value() && first_superelevated.has_value() &&
                first_graded.has_value() && Near(*first_curved, 298.5) &&
                Near(*first_superelevated, 298.5) &&
                Near(*first_graded, 4089.864),
            "Chunshen Station asset does not expose the declared first seam "
            "supports");

    struct PlanAndCantSample {
        double station_meters;
        double curvature_radians_per_meter;
        double superelevation_meters;
    };
    for (const PlanAndCantSample sample : {
             PlanAndCantSample{350.0, -1.0 / 1500.0, -0.054},
             PlanAndCantSample{800.0, 1.0 / 650.0, 0.12},
             PlanAndCantSample{15000.0, 1.0 / 1900.0, 0.14},
         }) {
        Require(Near(geometry.CurvatureRadiansPerMeter(sample.station_meters),
                     sample.curvature_radians_per_meter) &&
                    Near(geometry.SuperelevationMeters(sample.station_meters),
                         sample.superelevation_meters),
                "Chunshen Station plan or superelevation sign changed");
    }

    struct GradeSample {
        double station_meters;
        double centerline_upward_grade;
    };
    for (const GradeSample sample : {
             GradeSample{4089.0, 0.0},
             GradeSample{4101.364, 0.025},
             GradeSample{4200.0, 0.05},
             GradeSample{4271.364, 0.025},
             GradeSample{4283.0, 0.0},
             GradeSample{6741.364, 0.015},
             GradeSample{7000.0, 0.03},
             GradeSample{7441.364, 0.015},
             GradeSample{7468.0, 0.0},
         }) {
        Require(Near(geometry.CenterlineUpwardGrade(sample.station_meters),
                     sample.centerline_upward_grade),
                "Chunshen Station vertical profile changed at a qualified "
                "station");
    }

    for (const double station : {4200.0, 7000.0}) {
        Require(geometry.CurvatureRadiansPerMeter(station) == 0.0 &&
                    geometry.SuperelevationMeters(station) == 0.0,
                "a Chunshen Station vertical curve left its declared planar "
                "straight and zero-superelevation region");
    }

    const auto centerline =
        geometry.CenterlinePositionInInertialMeters(7000.0);
    Require(centerline.allFinite() && centerline.z() < 0.0,
            "Chunshen Station centerline is not finite and uphill");
    const auto projection =
        geometry.ProjectPointNearSeed(centerline, 7000.2, 1.0);
    Require(Near(projection.track_station_meters(), 7000.0, 2.0e-10),
            "a Chunshen Station centerline point did not project to its own "
            "station");
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
    Require(geometry.superelevation_reference_baselength_meters() == 1.6,
            "record superelevation reference baselength was not mapped");
    // Non-midpoint probes make the scalar Hermite and vertical PL2 field
    // mappings directional.
    Require(Near(geometry.CurvatureRadiansPerMeter(22.0), 0.0015625),
            "Hermite curvature segment direction was not mapped");
    Require(Near(geometry.SuperelevationMeters(17.0), 0.0125),
            "Hermite superelevation segment direction was not mapped");
    Require(Near(geometry.CenterlineUpwardGrade(22.0), 0.025),
            "parabolic vertical-curve direction was not mapped");
    const double circular_start_angle = std::atan(0.03);
    const double circular_end_angle = std::atan(0.04);
    const double circular_radius =
        10.0 / (std::sin(circular_end_angle) -
                std::sin(circular_start_angle));
    const double circular_midpoint_grade = std::tan(std::asin(
        std::sin(circular_start_angle) + 5.0 / circular_radius));
    Require(Near(geometry.CenterlineUpwardGrade(32.0),
                 circular_midpoint_grade),
            "circular vertical-curve direction was not mapped");
    Require(std::abs(geometry.CurvatureRadiansPerMeter(16.5)) > 1.0e-15,
            "declared seam transition did not enter the profile");

    const auto point = geometry.CenterlinePositionInInertialMeters(12.0);
    Require(Near(point.x(), 5.0) && point.y() == 0.0 && Near(point.z(), -0.1),
            "loaded geometry depends on the deleted input file");
}

void CheckStrictRejections(const std::filesystem::path& path) {
    const std::string valid(kNondegenerateRecord);
    const std::string invalid_syntax = ReplaceOnce(
        valid, "\"start_track_station_meters\": 7.0,",
        "\"start_track_station_meters\": 7.0,,");
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
        {ReplaceOnce(valid, "\"start_track_station_meters\": 7.0,",
                     "\"start_track_station_meters\": 7.0, "
                     "\"start_track_station_meters\": 7.0,"),
         {"duplicate JSON object key at $.start_track_station_meters"}},
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
        {ReplaceOnce(valid, "\"vertical_profile\": {",
                     "\"centerline_upward_grade_profile\": {"),
         {"$.centerline_upward_grade_profile", "unknown key"}},
        {ReplaceOnce(valid, "\"shape\": \"circular_vertical_curve\"",
                     "\"shape\": \"vertical_spline\""),
         {"$.vertical_profile.segments[2].shape",
          "'circular_vertical_curve'"}},
        {ReplaceOnce(valid,
                     "\"end_centerline_upward_grade\": 0.04",
                     "\"end_centerline_upward_grade\": 0.03"),
         {"TrackVerticalProfile: segment 2 has equal endpoint grades"}},
        {ReplaceOnce(valid,
                     "\"start_centerline_upward_grade\": 0.03",
                     "\"start_centerline_upward_grade\": 0.031"),
         {"TrackVerticalProfile: segment 1 ends at grade", "no seam"}},
        {ReplaceOnce(valid, "\"station_node_spacing_meters\": 0.7",
                     "\"station_node_spacing_meters\": \"fine\""),
         {"$.station_node_spacing_meters"}},
        {ReplaceOnce(valid,
                     "\"superelevation_reference_baselength_meters\": 1.6",
                     "\"superelevation_reference_baselength_meters\": 1e400"),
         {"$.superelevation_reference_baselength_meters",
          "not representable"}},
        {ReplaceOnce(valid, "\"start_track_station_meters\": 7.0",
                     "\"start_track_station_meters\": 1e-400"),
         {"$.start_track_station_meters", "underflows binary64"}},
        {ReplaceOnce(valid, "\"start_track_station_meters\": 7.0",
                     "\"start_track_station_meters\": 9007199254740993"),
         {"$.start_track_station_meters", "exactly representable"}},
        {ReplaceOnce(valid,
                     "\"superelevation_reference_baselength_meters\": 1.6",
                     "\"superelevation_reference_baselength_meters\": 0.0"),
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
        if (argc != 9) {
            throw std::invalid_argument(
                "expected the straight, R300, combined-line, R600, R800, "
                "R1000 and qualification-straight assets, and a scratch "
                "directory");
        }
        const std::filesystem::path scratch = argv[8];
        std::filesystem::remove_all(scratch);
        std::filesystem::create_directories(scratch);
        const std::filesystem::path temporary_configuration =
            scratch / "track_geometry.json";

        CheckRealAsset(argv[1]);
        CheckR300QualificationAsset(argv[2]);
        CheckChunshenStationAsset(argv[3]);
        CheckImportedCurveAsset(argv[4], 1100.0, 600.0, 0.1, false);
        CheckImportedCurveAsset(argv[5], 1100.0, 800.0, 0.11, false);
        CheckImportedCurveAsset(argv[6], 300.0, 1000.0, 0.12, true);
        CheckQualificationStraightAsset(argv[7]);
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
