#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "orvd/configuration/load_track_irregularity_field.h"

namespace {

using orvd::configuration::LoadTrackIrregularityFieldFromDataRoot;

constexpr std::string_view kManifest = R"json({
  "track_irregularity_identifier": "synthetic_nonuniform",
  "coordinate_frame": "track_lateral_right_vertical_down",
  "lateral_series_identifier": "synthetic_lateral",
  "vertical_series_identifier": "synthetic_vertical"
})json";

constexpr std::string_view kLateralSeries = R"json({
  "series_identifier": "synthetic_lateral",
  "track_station_meters": [0.0, 0.7, 2.0],
  "displacement_meters": [1.0, 2.4, 5.0]
})json";

constexpr std::string_view kVerticalSeries = R"json({
  "series_identifier": "synthetic_vertical",
  "track_station_meters": [-1.0, 0.5, 1.75, 3.0],
  "displacement_meters": [7.0, 2.5, -1.25, -5.0]
})json";

int failures = 0;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        std::fprintf(stderr, "track irregularity loading: %.*s\n",
                     static_cast<int>(message.size()), message.data());
        ++failures;
    }
}

void Write(const std::filesystem::path& path, std::string_view contents) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output || !(output << contents)) {
        throw std::runtime_error("could not write test track irregularity asset");
    }
}

std::string ReplaceOnce(std::string source, std::string_view needle,
                        std::string_view replacement) {
    const std::size_t position = source.find(needle);
    if (position == std::string::npos ||
        source.find(needle, position + needle.size()) != std::string::npos) {
        throw std::runtime_error("test mutation did not identify one substring");
    }
    source.replace(position, needle.size(), replacement);
    return source;
}

struct AssetPaths {
    std::filesystem::path manifest;
    std::filesystem::path lateral;
    std::filesystem::path vertical;
};

AssetPaths WriteValidAssets(const std::filesystem::path& data_root) {
    const std::filesystem::path directory =
        data_root / "track_library" / "irregularities";
    const std::filesystem::path series = directory / "series";
    AssetPaths paths{directory / "synthetic_nonuniform.json",
                     series / "synthetic_lateral.json",
                     series / "synthetic_vertical.json"};
    Write(paths.manifest, kManifest);
    Write(paths.lateral, kLateralSeries);
    Write(paths.vertical, kVerticalSeries);
    return paths;
}

void ExpectInvalid(const std::filesystem::path& data_root,
                   const std::function<void(const AssetPaths&)>& mutate,
                   std::string_view diagnostic_fragment) {
    std::filesystem::remove_all(data_root);
    const AssetPaths paths = WriteValidAssets(data_root);
    mutate(paths);
    try {
        static_cast<void>(LoadTrackIrregularityFieldFromDataRoot(
            data_root, "synthetic_nonuniform"));
    } catch (const std::invalid_argument& error) {
        if (std::string_view(error.what()).find(diagnostic_fragment) ==
            std::string_view::npos) {
            std::fprintf(stderr,
                         "track irregularity loading: expected diagnostic "
                         "containing '%.*s', got '%s'\n",
                         static_cast<int>(diagnostic_fragment.size()),
                         diagnostic_fragment.data(), error.what());
            ++failures;
        }
        return;
    }
    throw std::runtime_error("invalid track irregularity asset was accepted");
}

void CheckIndependentNonuniformSeriesAndLifetime(
    const std::filesystem::path& data_root) {
    std::filesystem::remove_all(data_root);
    WriteValidAssets(data_root);
    auto field = LoadTrackIrregularityFieldFromDataRoot(
        data_root, "synthetic_nonuniform");
    std::filesystem::remove_all(data_root);

    constexpr double kStation = 1.25;
    Require(std::abs(field.LateralDisplacementMeters(kStation) - 3.5) < 1.0e-14,
            "lateral values did not use the lateral series");
    Require(std::abs(field.LateralSlopeMetersPerMeter(kStation) - 2.0) < 1.0e-14,
            "lateral slopes did not use the lateral series");
    Require(std::abs(field.VerticalDisplacementMeters(kStation) - 0.25) < 1.0e-14,
            "vertical values did not use the independent vertical grid");
    Require(std::abs(field.VerticalSlopeMetersPerMeter(kStation) + 3.0) < 1.0e-14,
            "vertical slopes did not use the independent vertical grid");

    Require(field.LateralDisplacementMeters(-4.0) == 0.0 &&
                field.LateralSlopeMetersPerMeter(-4.0) == 0.0 &&
                field.VerticalDisplacementMeters(9.0) == 0.0 &&
                field.VerticalSlopeMetersPerMeter(9.0) == 0.0,
            "an excitation remains active outside its declared station "
            "interval");
}

void CheckStrictRejections(const std::filesystem::path& data_root) {
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.manifest,
                  ReplaceOnce(std::string(kManifest),
                              "\"track_irregularity_identifier\": "
                              "\"synthetic_nonuniform\",",
                              "\"track_irregularity_identifier\": "
                              "\"synthetic_nonuniform\", "
                              "\"track_irregularity_identifier\": "
                              "\"synthetic_nonuniform\","));
        },
        "duplicate JSON object key at $.track_irregularity_identifier");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.manifest,
                  ReplaceOnce(std::string(kManifest),
                              "\"coordinate_frame\":",
                              "\"unclaimed_key\": 4,\n  \"coordinate_frame\":"));
        },
        "$.unclaimed_key");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.manifest,
                  ReplaceOnce(std::string(kManifest),
                              ",\n  \"vertical_series_identifier\": "
                              "\"synthetic_vertical\"\n",
                              ""));
        },
        "$.vertical_series_identifier");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.manifest,
                  ReplaceOnce(std::string(kManifest),
                              "\"track_lateral_right_vertical_down\"",
                              "8"));
        },
        "$.coordinate_frame");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.manifest,
                  ReplaceOnce(std::string(kManifest),
                              "\"synthetic_lateral\"",
                              "\"synthetic lateral\""));
        },
        "outside [A-Za-z0-9._-]");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.manifest,
                  ReplaceOnce(std::string(kManifest),
                              "\"synthetic_nonuniform\"",
                              "\"different_field\""));
        },
        "but the caller requested");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.lateral,
                  ReplaceOnce(std::string(kLateralSeries),
                              "\"synthetic_lateral\"",
                              "\"different_series\""));
        },
        "but the field manifest references");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.lateral,
                  ReplaceOnce(std::string(kLateralSeries),
                              "[1.0, 2.4, 5.0]", "[1.0, 2.4]"));
        },
        "must have the same length");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.lateral,
                  ReplaceOnce(
                      ReplaceOnce(std::string(kLateralSeries),
                                  "[0.0, 0.7, 2.0]", "[0.0]"),
                      "[1.0, 2.4, 5.0]", "[1.0]"));
        },
        "at least two values");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.lateral,
                  ReplaceOnce(std::string(kLateralSeries), "0.7", "1e400"));
        },
        "not representable");
    ExpectInvalid(
        data_root,
        [](const AssetPaths& paths) {
            Write(paths.lateral,
                  ReplaceOnce(std::string(kLateralSeries),
                              "[0.0, 0.7, 2.0]", "[0.0, 2.0, 0.7]"));
        },
        "must be strictly greater than its predecessor");
}

void CheckMissingSeriesFailsLoudly(const std::filesystem::path& data_root) {
    std::filesystem::remove_all(data_root);
    const AssetPaths paths = WriteValidAssets(data_root);
    std::filesystem::remove(paths.vertical);
    try {
        static_cast<void>(LoadTrackIrregularityFieldFromDataRoot(
            data_root, "synthetic_nonuniform"));
    } catch (const std::runtime_error& error) {
        Require(std::string_view(error.what()).find("synthetic_vertical.json") !=
                    std::string_view::npos,
                "missing series diagnostic does not identify its path");
        return;
    }
    throw std::runtime_error("a missing referenced series was accepted");
}

void CheckGz18R300Aar5Asset(const std::filesystem::path& source_data_root) {
    const auto field = LoadTrackIrregularityFieldFromDataRoot(
        source_data_root, "gz18_r300_aar5_reference_irregularity");
    Require(field.LateralDisplacementMeters(59.0) == 0.0 &&
                field.VerticalDisplacementMeters(59.0) == 0.0 &&
                field.LateralDisplacementMeters(1001.0) == 0.0 &&
                field.VerticalDisplacementMeters(1001.0) == 0.0,
            "GZ18 R300 AAR5 asset remains active outside [60, 1000] m");
    Require(field.LateralDisplacementMeters(60.0) == 0.0 &&
                field.VerticalDisplacementMeters(60.0) == 0.0 &&
                field.LateralDisplacementMeters(1000.0) == 0.0 &&
                field.VerticalDisplacementMeters(1000.0) == 0.0,
            "GZ18 R300 AAR5 asset endpoints are not zero");
    Require(field.LateralDisplacementMeters(100.0) ==
                    -0.0010418007586758813 &&
                field.VerticalDisplacementMeters(100.0) ==
                    -0.0017274249872550945 &&
                field.LateralDisplacementMeters(960.0) ==
                    0.0032490507631524118 &&
                field.VerticalDisplacementMeters(960.0) ==
                    0.005387298316295291,
            "GZ18 R300 AAR5 exact fade boundaries drifted");
    Require(std::isfinite(field.LateralSlopeMetersPerMeter(100.0)) &&
                std::isfinite(field.VerticalSlopeMetersPerMeter(100.0)) &&
                std::isfinite(field.LateralSlopeMetersPerMeter(960.0)) &&
                std::isfinite(field.VerticalSlopeMetersPerMeter(960.0)),
            "GZ18 R300 AAR5 fade-boundary slopes are not finite");
}

void CheckIrwR300Aar5Asset(const std::filesystem::path& source_data_root) {
    const auto field = LoadTrackIrregularityFieldFromDataRoot(
        source_data_root, "irw_r300_aar5_reference_irregularity");
    Require(field.LateralDisplacementMeters(-1.0) == 0.0 &&
                field.VerticalDisplacementMeters(-1.0) == 0.0 &&
                field.LateralDisplacementMeters(1250.0) == 0.0 &&
                field.VerticalDisplacementMeters(1250.0) == 0.0,
            "IRW R300 AAR5 asset remains active outside its frozen point-list domain");
    Require(field.LateralDisplacementMeters(0.0) == 0.0 &&
                field.VerticalDisplacementMeters(0.0) == 0.0 &&
                field.LateralDisplacementMeters(1249.83) == 0.0 &&
                field.VerticalDisplacementMeters(1249.83) == 0.0,
            "IRW R300 AAR5 point-list endpoints are not zero");
    Require(field.LateralDisplacementMeters(160.1405) ==
                    -2.2541299999999999e-09 &&
                field.VerticalDisplacementMeters(160.1405) ==
                    -3.7376099999999999e-09 &&
                field.LateralDisplacementMeters(450.01400000000001) ==
                    -0.0022578400000000001 &&
                field.VerticalDisplacementMeters(450.01400000000001) ==
                    -0.0037437500000000001,
            "IRW R300 AAR5 frozen point values drifted");
    Require(std::isfinite(field.LateralSlopeMetersPerMeter(160.1405)) &&
                std::isfinite(field.VerticalSlopeMetersPerMeter(160.1405)) &&
                std::isfinite(
                    field.LateralSlopeMetersPerMeter(450.01400000000001)) &&
                std::isfinite(
                    field.VerticalSlopeMetersPerMeter(450.01400000000001)),
            "IRW R300 AAR5 spline slopes are not finite");
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc != 3) {
            throw std::invalid_argument(
                "expected the source and scratch data-root paths");
        }
        const std::filesystem::path source_data_root = argv[1];
        const std::filesystem::path data_root = argv[2];
        CheckGz18R300Aar5Asset(source_data_root);
        CheckIrwR300Aar5Asset(source_data_root);
        CheckIndependentNonuniformSeriesAndLifetime(data_root);
        CheckStrictRejections(data_root);
        CheckMissingSeriesFailsLoudly(data_root);
        std::filesystem::remove_all(data_root);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "track irregularity field loading failed: %s\n",
                     error.what());
        return 1;
    }
    return failures == 0 ? 0 : 1;
}
