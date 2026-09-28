#include "orvd/scene_record/scene_record_reader.h"

#include <bit>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "scene_record_layout.h"

namespace orvd::scene_record {
namespace {

using scene_observation::SamplePhase;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::runtime_error("scene record reader: " + detail);
}

std::string ReadWholeFile(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input) {
        Reject("could not open '" + path.string() + "'");
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

double LittleEndianDouble(const unsigned char* bytes) {
    std::uint64_t bits = 0;
    for (unsigned byte = 0; byte < 8; ++byte) {
        bits |= static_cast<std::uint64_t>(bytes[byte]) << (byte * 8);
    }
    return std::bit_cast<double>(bits);
}

wheel_rail_contact::WheelSide ParseSide(const std::string& name) {
    if (name == "right") {
        return wheel_rail_contact::WheelSide::kRight;
    }
    if (name == "left") {
        return wheel_rail_contact::WheelSide::kLeft;
    }
    Reject("unknown wheel side '" + name + "'");
}

Eigen::Vector3d Vector3FromJson(const nlohmann::json& value) {
    if (!value.is_array() || value.size() != 3) {
        Reject("a three-vector does not have three entries");
    }
    return Eigen::Vector3d(value[0].get<double>(), value[1].get<double>(),
                           value[2].get<double>());
}

template <std::size_t N>
std::array<double, N> ArrayFromJson(const nlohmann::json& value) {
    if (!value.is_array() || value.size() != N) {
        Reject("an array row does not have the expected length");
    }
    std::array<double, N> out{};
    for (std::size_t index = 0; index < N; ++index) {
        out[index] = value[index].get<double>();
    }
    return out;
}

template <std::size_t N>
std::vector<std::array<double, N>> ArrayRowsFromJson(
    const nlohmann::json& value) {
    std::vector<std::array<double, N>> rows;
    rows.reserve(value.size());
    for (const auto& row : value) {
        rows.push_back(ArrayFromJson<N>(row));
    }
    return rows;
}

SamplePhase ParsePhase(double code) {
    if (code == 0.0) {
        return SamplePhase::kInitialAcceptedState;
    }
    if (code == 1.0) {
        return SamplePhase::kDenseIntermediateSample;
    }
    if (code == 2.0) {
        return SamplePhase::kAcceptedEndpoint;
    }
    Reject("unknown sample phase code");
}

}  // namespace

SceneRecord ReadSceneRecord(const std::filesystem::path& directory) {
    const std::filesystem::path scene_path =
        directory / internal::kSceneFileName;
    if (!std::filesystem::exists(scene_path)) {
        Reject("'" + directory.string() +
               "' has no scene.json; the record is incomplete or absent");
    }
    const nlohmann::json scene = nlohmann::json::parse(ReadWholeFile(scene_path));
    if (scene.at("record").get<std::string>() != internal::kRecordIdentifier) {
        Reject("'" + scene_path.string() + "' is not an ORVD scene record");
    }

    SceneRecord record;
    for (const auto& body : scene.at("bodies")) {
        record.topology.bodies.push_back(scene_observation::SceneBody{
            body.at("name").get<std::string>(),
            body.at("moves_freely_in_world").get<bool>()});
    }
    if (record.topology.bodies.empty()) {
        Reject("the record names no rigid body");
    }
    for (const auto& wheel : scene.at("wheel_placements")) {
        scene_observation::SceneWheelPlacement placement;
        placement.interface_name = wheel.at("interface_name").get<std::string>();
        placement.wheel_body_name =
            wheel.at("wheel_body_name").get<std::string>();
        placement.carrier_body_name =
            wheel.at("carrier_body_name").get<std::string>();
        placement.side = ParseSide(wheel.at("side").get<std::string>());
        placement.datum_in_wheel_body_frame_meters =
            Vector3FromJson(wheel.at("datum_in_wheel_body_frame_meters"));
        placement.spin_axis_in_wheel_body_frame =
            Vector3FromJson(wheel.at("spin_axis_in_wheel_body_frame"));
        placement.nominal_rolling_radius_meters =
            wheel.at("nominal_rolling_radius_meters").get<double>();
        record.topology.wheel_placements.push_back(std::move(placement));
    }
    for (const auto& definition : scene.at("scalars")) {
        record.scalar_definitions.push_back(scene_observation::ScalarDefinition{
            definition.at("name").get<std::string>(),
            definition.at("unit").get<std::string>(),
            definition.at("reference_frame").get<std::string>(),
            definition.at("quantity").get<std::string>(),
            definition.at("method").get<std::string>(),
            definition.at("sample_semantics").get<std::string>()});
    }
    const auto& visual_file = scene.at("visual_definition_file");
    if (!visual_file.is_null()) {
        record.visual_definition_json =
            ReadWholeFile(directory / visual_file.get<std::string>());
    }
    const auto& track = scene.at("track");
    if (!track.is_null()) {
        scene_observation::TrackSampleTable table;
        table.stations_meters =
            track.at("stations_meters").get<std::vector<double>>();
        table.centerline_in_inertial_meters =
            ArrayRowsFromJson<3>(track.at("centerline_in_inertial_meters"));
        table.rotation_inertial_from_track_wxyz =
            ArrayRowsFromJson<4>(track.at("rotation_inertial_from_track_wxyz"));
        table.curvature_radians_per_meter =
            track.at("curvature_radians_per_meter").get<std::vector<double>>();
        table.superelevation_meters =
            track.at("superelevation_meters").get<std::vector<double>>();
        table.left_rail_datum_in_inertial_meters =
            ArrayRowsFromJson<3>(track.at("left_rail_datum_in_inertial_meters"));
        table.right_rail_datum_in_inertial_meters = ArrayRowsFromJson<3>(
            track.at("right_rail_datum_in_inertial_meters"));
        const std::size_t count = table.stations_meters.size();
        if (count == 0 || table.centerline_in_inertial_meters.size() != count ||
            table.rotation_inertial_from_track_wxyz.size() != count ||
            table.curvature_radians_per_meter.size() != count ||
            table.superelevation_meters.size() != count ||
            table.left_rail_datum_in_inertial_meters.size() != count ||
            table.right_rail_datum_in_inertial_meters.size() != count) {
            Reject("the track table columns disagree in length");
        }
        record.track = std::move(table);
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
    if (body_count != record.topology.bodies.size() ||
        values_per_body != internal::kValuesPerBody ||
        body_offset != internal::kIdentityValueCount ||
        spin_offset != body_offset + values_per_body * body_count ||
        (spin_count != 0 &&
         spin_count != record.topology.wheel_placements.size()) ||
        scalar_offset != spin_offset + spin_count ||
        scalar_count != record.scalar_definitions.size() ||
        row_value_count != scalar_offset + scalar_count) {
        Reject("the frame table layout disagrees with the topology or the "
               "scalar definitions");
    }

    const std::string frame_bytes = ReadWholeFile(
        directory / frame_table.at("file").get<std::string>());
    if (frame_bytes.size() != frame_count * row_value_count * 8) {
        Reject("frames file size disagrees with the stated frame count");
    }
    const auto& status_table = scene.at("scalar_status_table");
    const std::string status_bytes = ReadWholeFile(
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
}

}  // namespace orvd::scene_record
