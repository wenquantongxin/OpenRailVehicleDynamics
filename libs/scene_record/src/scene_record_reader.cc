#include "orvd/scene_record/scene_record_reader.h"

#include <bit>
#include <cstdint>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "scene_assets_json.h"
#include "scene_record_layout.h"

namespace orvd::scene_record {
namespace {

using scene_observation::SamplePhase;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::runtime_error("scene record reader: " + detail);
}

double LittleEndianDouble(const unsigned char* bytes) {
    std::uint64_t bits = 0;
    for (unsigned byte = 0; byte < 8; ++byte) {
        bits |= static_cast<std::uint64_t>(bytes[byte]) << (byte * 8);
    }
    return std::bit_cast<double>(bits);
}

SamplePhase ParsePhase(double code) {
    if (code == static_cast<double>(SamplePhase::kInitialAcceptedState)) {
        return SamplePhase::kInitialAcceptedState;
    }
    if (code == static_cast<double>(SamplePhase::kDenseIntermediateSample)) {
        return SamplePhase::kDenseIntermediateSample;
    }
    if (code == static_cast<double>(SamplePhase::kAcceptedEndpoint)) {
        return SamplePhase::kAcceptedEndpoint;
    }
    Reject("unknown sample phase code");
}

}  // namespace

SceneRecord ReadSceneRecord(const std::filesystem::path& directory) try {
    const std::filesystem::path scene_path =
        directory / internal::kSceneFileName;
    if (!std::filesystem::exists(scene_path)) {
        Reject("'" + directory.string() +
               "' has no scene.json; the record is incomplete or absent");
    }
    const nlohmann::json scene = internal::ReadSceneDocument(scene_path);
    if (scene.at("schema").get<std::string>() != internal::kRecordIdentifier) {
        Reject("'" + scene_path.string() + "' is not an ORVD scene record");
    }

    SceneRecord record;
    record.description = scene_observation::DecodeSceneDescription(
        scene.at("description").dump());
    record.assets = internal::DecodeSceneAssets(scene.at("assets"), directory);
    if (record.description.world_frame != record.assets.world_frame) {
        Reject("description and assets use different world frames");
    }

    const auto& frame_table = scene.at("frame_table");
    const std::size_t frame_count = frame_table.at("frame_count").get<std::size_t>();
    const std::size_t row_value_count =
        frame_table.at("row_value_count").get<std::size_t>();
    const auto& columns = frame_table.at("columns");
    const std::size_t body_offset = columns.at("body_states").at("offset").get<std::size_t>();
    const std::size_t body_count = columns.at("body_states").at("body_count").get<std::size_t>();
    const std::size_t values_per_body =
        columns.at("body_states").at("values_per_body").get<std::size_t>();
    const std::size_t scalar_offset = columns.at("scalar_values").at("offset").get<std::size_t>();
    const std::size_t scalar_count = columns.at("scalar_values").at("count").get<std::size_t>();
    const std::size_t spin_offset = columns.at("wheel_spin_angles").at("offset").get<std::size_t>();
    const std::size_t spin_count = columns.at("wheel_spin_angles").at("count").get<std::size_t>();
    if (body_count != record.description.topology.bodies.size() ||
        values_per_body != internal::kValuesPerBody ||
        body_offset != internal::kIdentityValueCount ||
        spin_offset != body_offset + values_per_body * body_count ||
        (spin_count != 0 &&
         spin_count != record.description.topology.wheel_placements.size()) ||
        scalar_offset != spin_offset + spin_count ||
        scalar_count != record.description.scalar_definitions.size() ||
        row_value_count != scalar_offset + scalar_count) {
        Reject("the frame table layout disagrees with the topology or the "
               "scalar definitions");
    }

    const std::string frame_bytes = internal::ReadWholeFile(
        directory / frame_table.at("file").get<std::string>());
    if (frame_bytes.size() != frame_count * row_value_count * 8) {
        Reject("frames file size disagrees with the stated frame count");
    }
    const auto& status_table = scene.at("scalar_status_table");
    const std::string status_bytes = internal::ReadWholeFile(
        directory / status_table.at("file").get<std::string>());
    if (status_bytes.size() != frame_count * scalar_count) {
        Reject("scalar status file size disagrees with the stated counts");
    }

    record.frames.reserve(frame_count);
    const auto* bytes = reinterpret_cast<const unsigned char*>(frame_bytes.data());
    for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
        const unsigned char* row = bytes + frame_index * row_value_count * 8;
        const auto value_at = [row](std::size_t column) {
            return LittleEndianDouble(row + column * 8);
        };
        scene_observation::SceneFrame frame;
        frame.identity.time_seconds = value_at(internal::kTimeSecondsColumn);
        frame.identity.time_nanoseconds = static_cast<std::int64_t>(
            value_at(internal::kTimeNanosecondsColumn));
        frame.identity.sample_index =
            static_cast<std::int64_t>(value_at(internal::kSampleIndexColumn));
        frame.identity.phase = ParsePhase(value_at(internal::kPhaseColumn));
        frame.bodies.resize(body_count);
        for (std::size_t body = 0; body < body_count; ++body) {
            const std::size_t base = body_offset + body * values_per_body;
            auto& state = frame.bodies[body];
            for (std::size_t k = 0; k < 3; ++k) {
                state.position_meters[k] = value_at(base + k);
            }
            for (std::size_t k = 0; k < 4; ++k) {
                state.orientation_wxyz[k] = value_at(base + 3 + k);
            }
            for (std::size_t k = 0; k < 3; ++k) {
                state.linear_velocity_meters_per_second[k] =
                    value_at(base + 7 + k);
            }
            for (std::size_t k = 0; k < 3; ++k) {
                state.angular_velocity_radians_per_second[k] =
                    value_at(base + 10 + k);
            }
        }
        frame.wheel_spin_angles_radians.resize(spin_count);
        for (std::size_t wheel = 0; wheel < spin_count; ++wheel) {
            frame.wheel_spin_angles_radians[wheel] = value_at(spin_offset + wheel);
        }
        frame.scalars.values.resize(scalar_count);
        frame.scalars.statuses.resize(scalar_count);
        for (std::size_t scalar = 0; scalar < scalar_count; ++scalar) {
            frame.scalars.values[scalar] = value_at(scalar_offset + scalar);
            frame.scalars.statuses[scalar] = static_cast<std::uint8_t>(
                status_bytes[frame_index * scalar_count + scalar]);
        }
        record.frames.push_back(std::move(frame));
    }
    return record;
} catch (const nlohmann::json::exception& error) {
    Reject(std::string("invalid scene record: ") + error.what());
} catch (const std::invalid_argument& error) {
    Reject(error.what());
}

}  // namespace orvd::scene_record
