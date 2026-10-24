#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "orvd/scene_record/scene_record_reader.h"
#include "orvd/scene_stream/scene_stream.h"
#include "orvd/state_stream/state_stream.h"

namespace {

using Clock = std::chrono::steady_clock;
using orvd::scene_observation::ScalarStatus;

struct Arguments {
    std::filesystem::path record_directory;
    std::vector<std::string> destinations;
    std::optional<std::string> resource_bundle;
};

constexpr const char* kUsage =
    "Usage: orvd_replay_scene_record RECORD_DIRECTORY --udp-target IP:PORT "
    "[--udp-target IP:PORT ...] [--resource-bundle NAME]\n";

Arguments ParseArguments(int argc, char** argv) {
    if (argc < 2 || std::string(argv[1]).starts_with("--")) {
        throw std::invalid_argument(kUsage);
    }
    Arguments result;
    result.record_directory = argv[1];
    for (int index = 2; index < argc; ++index) {
        const std::string option = argv[index];
        if (index + 1 == argc) {
            throw std::invalid_argument("missing value for " + option);
        }
        const std::string value = argv[++index];
        if (value.empty()) {
            throw std::invalid_argument("empty value for " + option);
        }
        if (option == "--udp-target") {
            result.destinations.push_back(value);
        } else if (option == "--resource-bundle" && !result.resource_bundle.has_value()) {
            result.resource_bundle = value;
        } else {
            throw std::invalid_argument("unknown or repeated option: " + option);
        }
    }
    if (result.destinations.empty()) {
        throw std::invalid_argument(kUsage);
    }
    return result;
}

bool PrepareDescription(orvd::scene_record::SceneRecord& record) {
    if (record.frames.empty()) {
        throw std::invalid_argument("record has no samples");
    }
    const auto wheel_count = record.description.topology.wheel_placements.size();
    const bool has_wheel_angles = !record.frames.front().wheel_spin_angles_radians.empty();
    double previous_time = record.frames.front().identity.time_seconds;
    for (const auto& frame : record.frames) {
        const double time = frame.identity.time_seconds;
        if (!std::isfinite(time) || time < previous_time) {
            throw std::invalid_argument("record times must be finite and nondecreasing");
        }
        previous_time = time;
        if (frame.wheel_spin_angles_radians.size() != (has_wheel_angles ? wheel_count : 0)) {
            throw std::invalid_argument("record wheel-angle columns are inconsistent");
        }
    }
    const double span = previous_time - record.frames.front().identity.time_seconds;
    const double remaining_clock_seconds =
        std::chrono::duration<double>(Clock::time_point::max() - Clock::now()).count();
    if (!std::isfinite(span) || span >= remaining_clock_seconds) {
        throw std::invalid_argument("record duration exceeds the replay clock range");
    }
    if (!has_wheel_angles) return false;

    std::unordered_set<std::string> names;
    for (const auto& scalar : record.description.scalar_definitions) names.insert(scalar.name);
    for (auto& wheel : record.description.topology.wheel_placements) {
        const std::string base = wheel.interface_name + ".replay_spin_angle_radians";
        std::string name = base;
        for (std::size_t suffix = 1; names.contains(name); ++suffix) {
            name = base + "." + std::to_string(suffix);
        }
        names.insert(name);
        record.description.scalar_definitions.push_back({
            name, "rad", wheel.wheel_body_name, "unwrapped wheel spin angle",
            "recorded wheel-angle column", "recorded sample"});
        wheel.spin_angle_scalar = orvd::scene_observation::WheelSpinScalarBinding{name, 1.0};
    }
    return true;
}

int Replay(const Arguments& arguments) {
    auto record = orvd::scene_record::ReadSceneRecord(arguments.record_directory);
    const bool append_wheel_angles = PrepareDescription(record);
    const std::string description = orvd::scene_stream::EncodeDescription(
        {record.description, arguments.resource_bundle});
    orvd::state_stream::UdpStateStream sender(arguments.destinations);
    const double first_time = record.frames.front().identity.time_seconds;
    double published_time = first_time;
    std::uint64_t description_sequence = 0;
    std::uint64_t state_sequence = 0;
    sender.PublishDescription(description_sequence++, published_time, description);
    const auto started = Clock::now();
    auto next_description = started + std::chrono::seconds(1);
    auto publish_description_if_due = [&] {
        const auto now = Clock::now();
        if (now >= next_description) {
            sender.PublishDescription(description_sequence++, published_time, description);
            next_description = now + std::chrono::seconds(1);
        }
    };

    for (const auto& frame : record.frames) {
        const auto deadline = started + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(frame.identity.time_seconds - first_time));
        while (Clock::now() < deadline) {
            publish_description_if_due();
            std::this_thread::sleep_until(std::min(deadline, next_description));
        }
        publish_description_if_due();
        auto scalars = frame.scalars;
        if (append_wheel_angles) {
            scalars.values.insert(scalars.values.end(), frame.wheel_spin_angles_radians.begin(),
                                  frame.wheel_spin_angles_radians.end());
            scalars.statuses.insert(scalars.statuses.end(), frame.wheel_spin_angles_radians.size(),
                                   static_cast<std::uint8_t>(ScalarStatus::kValid));
        }
        sender.PublishState(state_sequence++, frame.identity.time_seconds, frame.bodies, scalars);
        published_time = frame.identity.time_seconds;
    }

    std::uint64_t failures = 0;
    for (const auto& destination : sender.statistics().destinations) {
        failures += destination.failed_datagrams;
    }
    std::cout << "Replayed " << state_sequence << " samples at 1x; run_id="
              << sender.statistics().run_id << "; failed_datagrams=" << failures << '\n';
    return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << kUsage;
        return 0;
    }
    try {
        return Replay(ParseArguments(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "scene replay: " << error.what() << '\n';
        return 1;
    }
}
