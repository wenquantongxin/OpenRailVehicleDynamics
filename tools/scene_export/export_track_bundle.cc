#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Geometry>

#include "orvd/configuration/load_track_geometry.h"
#include "orvd/scene_record/scene_assets.h"

namespace {
namespace scene = orvd::scene_observation;
using Options = std::map<std::string, std::string, std::less<>>;

constexpr std::string_view kUsage =
    "orvd_export_track_bundle --track PATH --start METERS --end METERS "
    "--spacing METERS --output DIRECTORY\n"
    "  (--left-lateral METERS --left-vertical METERS --right-lateral METERS "
    "--right-vertical METERS | --rail-datums-from SCENE_JSON)\n"
    "  [--visual-definition PATH]\n";

Options Parse(int argc, char** argv) {
    const Options allowed{{"--track", {}}, {"--start", {}}, {"--end", {}},
        {"--spacing", {}}, {"--output", {}}, {"--left-lateral", {}},
        {"--left-vertical", {}}, {"--right-lateral", {}},
        {"--right-vertical", {}}, {"--rail-datums-from", {}},
        {"--visual-definition", {}}};
    Options result;
    for (int i = 1; i < argc; i += 2) {
        if (!allowed.contains(argv[i]) || i + 1 == argc) {
            throw std::invalid_argument("unknown option or missing value: " +
                                        std::string(argv[i]));
        }
        if (!result.emplace(argv[i], argv[i + 1]).second) {
            throw std::invalid_argument("duplicate option: " + std::string(argv[i]));
        }
    }
    return result;
}

const std::string& Required(const Options& options, std::string_view key) {
    const auto it = options.find(key);
    if (it == options.end() || it->second.empty()) {
        throw std::invalid_argument("missing " + std::string(key));
    }
    return it->second;
}

double Number(const Options& options, std::string_view key) {
    const auto& text = Required(options, key);
    double value{};
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size() || !std::isfinite(value)) {
        throw std::invalid_argument(std::string(key) + " must be finite");
    }
    return value;
}

std::vector<double> Stations(double start, double end, double spacing) {
    if (!(spacing > 0.0) || end < start) {
        throw std::invalid_argument("--spacing must be positive and --end >= --start");
    }
    const double intervals = std::floor((end - start) / spacing);
    std::vector<double> stations;
    if (!std::isfinite(intervals) || intervals >= static_cast<double>(stations.max_size() - 2)) {
        throw std::invalid_argument("sampling range is too large");
    }
    const auto count = static_cast<std::size_t>(intervals);
    stations.reserve(count + 2);
    stations.push_back(start);
    for (std::size_t i = 1; i <= count; ++i) {
        const double station = std::fma(static_cast<double>(i), spacing, start);
        if (station >= end) break;
        if (!(station > stations.back())) {
            throw std::invalid_argument("--spacing is below station resolution");
        }
        stations.push_back(station);
    }
    if (end > start) stations.push_back(end);
    return stations;
}

scene::RailDatumPlacement RecoverDatum(const scene::TrackSampleTable& table,
                                       const std::array<double, 3>& point) {
    const auto& center = table.centerline_in_inertial_meters.front();
    const auto& q = table.rotation_inertial_from_track_wxyz.front();
    const Eigen::Vector3d offset(point[0] - center[0], point[1] - center[1], point[2] - center[2]);
    const Eigen::Vector3d local = Eigen::Quaterniond(q[0], q[1], q[2], q[3])
        .toRotationMatrix().transpose() * offset;
    return {local.y(), local.z()};
}

void Export(const Options& options) {
    const double start = Number(options, "--start");
    const double end = Number(options, "--end");
    const auto stations = Stations(start, end, Number(options, "--spacing"));
    const auto& output = Required(options, "--output");
    const auto& track_path = Required(options, "--track");
    scene::RailDatumPlacement left;
    scene::RailDatumPlacement right;
    if (options.contains("--rail-datums-from")) {
        for (const auto* key : {"--left-lateral", "--left-vertical", "--right-lateral", "--right-vertical"}) {
            if (options.contains(key)) throw std::invalid_argument("--rail-datums-from conflicts with " + std::string(key));
        }
        const auto& source = Required(options, "--rail-datums-from");
        const auto assets = orvd::scene_record::ReadSceneAssets(source);
        if (!assets.track) throw std::invalid_argument("--rail-datums-from requires a track table");
        left = RecoverDatum(*assets.track, assets.track->left_rail_datum_in_inertial_meters.front());
        right = RecoverDatum(*assets.track, assets.track->right_rail_datum_in_inertial_meters.front());
        std::cout << "Rail datums from " << source << '\n';
    } else {
        left = {Number(options, "--left-lateral"), Number(options, "--left-vertical")};
        right = {Number(options, "--right-lateral"), Number(options, "--right-vertical")};
    }
    const auto line = orvd::configuration::LoadTrackGeometryFromJsonFile(track_path);
    orvd::scene_record::SceneAssets assets;
    assets.track.emplace();
    scene::SampleTrackGeometry(line, stations, left, right, *assets.track);
    if (options.contains("--visual-definition")) {
        const auto& path = Required(options, "--visual-definition");
        std::ifstream input(path);
        if (!input) throw std::runtime_error("cannot read visual definition: " + path);
        assets.visual_definition_json.assign(std::istreambuf_iterator<char>(input), {});
        if (input.bad()) throw std::runtime_error("cannot read visual definition: " + path);
        if (assets.visual_definition_json.empty()) {
            throw std::invalid_argument("--visual-definition must contain a JSON object");
        }
    }
    orvd::scene_record::WriteSceneBundle(output, assets);
    std::cout << "Wrote " << stations.size() << " stations to " << output << '\n';
}
}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string_view(argv[1]) == "--help") {
        std::cout << kUsage;
        return 0;
    }
    try {
        Export(Parse(argc, argv));
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n' << kUsage;
        return 1;
    }
}
