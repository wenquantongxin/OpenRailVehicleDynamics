#include "orvd/scene_record/scene_record_writer.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "scene_record_layout.h"

namespace orvd::scene_record {
namespace {

using scene_observation::SamplePhase;
using scene_observation::ScalarStatus;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::runtime_error("scene record writer: " + detail);
}

void AppendLittleEndian(std::vector<unsigned char>& out, double value) {
    static_assert(sizeof(double) == 8 &&
                  std::numeric_limits<double>::is_iec559);
    const auto bits = std::bit_cast<std::uint64_t>(value);
    for (unsigned byte = 0; byte < 8; ++byte) {
        out.push_back(static_cast<unsigned char>(bits >> (byte * 8)));
    }
}

void AppendFinite(std::vector<unsigned char>& out, double value,
                  const char* what) {
    if (!std::isfinite(value)) {
        Reject(std::string(what) + " is not finite");
    }
    AppendLittleEndian(out, value);
}

void AppendIntegerIdentity(std::vector<unsigned char>& out,
                           std::int64_t value, const char* what) {
    const double as_double = static_cast<double>(value);
    if (std::abs(as_double) > internal::kIntegerIdentityExactRange ||
        static_cast<std::int64_t>(as_double) != value) {
        Reject(std::string(what) +
               " exceeds the exactly representable integer range of the "
               "frame table");
    }
    AppendLittleEndian(out, as_double);
}

std::string SideName(wheel_rail_contact::WheelSide side) {
    return side == wheel_rail_contact::WheelSide::kRight ? "right" : "left";
}

nlohmann::json Vector3Json(const Eigen::Vector3d& value) {
    return nlohmann::json::array({value.x(), value.y(), value.z()});
}

template <std::size_t N>
nlohmann::json ArrayJson(const std::array<double, N>& value) {
    nlohmann::json out = nlohmann::json::array();
    for (double entry : value) {
        out.push_back(entry);
    }
    return out;
}

template <std::size_t N>
nlohmann::json ArrayRowsJson(const std::vector<std::array<double, N>>& rows) {
    nlohmann::json out = nlohmann::json::array();
    for (const auto& row : rows) {
        out.push_back(ArrayJson(row));
    }
    return out;
}

void CloseChecked(std::ofstream& stream, const std::filesystem::path& path) {
    stream.flush();
    if (!stream) {
        Reject("could not flush '" + path.string() + "'");
    }
    stream.close();
    if (!stream) {
        Reject("could not close '" + path.string() + "'");
    }
}

}  // namespace

SceneRecordWriter::SceneRecordWriter(
    const std::filesystem::path& directory,
    scene_observation::SceneTopology topology,
    std::vector<scene_observation::ScalarDefinition> scalar_definitions,
    std::string visual_definition_json, bool with_wheel_spin_angles)
    : directory_(directory),
      topology_(std::move(topology)),
      scalar_definitions_(std::move(scalar_definitions)),
      wheel_spin_angle_count_(
          with_wheel_spin_angles ? topology_.wheel_placements.size() : 0U) {
    if (topology_.bodies.empty()) {
        Reject("the scene topology names no rigid body");
    }
    const auto lists_body = [this](const std::string& name) {
        for (const auto& body : topology_.bodies) {
            if (body.name == name) {
                return true;
            }
        }
        return false;
    };
    for (const auto& wheel : topology_.wheel_placements) {
        if (!lists_body(wheel.wheel_body_name)) {
            Reject("wheel placement '" + wheel.interface_name +
                   "' names wheel body '" + wheel.wheel_body_name +
                   "', which the topology does not list");
        }
        if (!lists_body(wheel.carrier_body_name)) {
            Reject("wheel placement '" + wheel.interface_name +
                   "' names carrier body '" + wheel.carrier_body_name +
                   "', which the topology does not list");
        }
    }
    std::error_code error;
    if (std::filesystem::exists(directory_, error) || error) {
        Reject("'" + directory_.string() +
               "' already exists or cannot be inspected");
    }
    if (!std::filesystem::create_directories(directory_, error) || error) {
        Reject("could not create '" + directory_.string() + "'");
    }
    if (!visual_definition_json.empty()) {
        const nlohmann::json parsed =
            nlohmann::json::parse(visual_definition_json, nullptr, false);
        if (!parsed.is_object()) {
            Reject("the visual definition is not one JSON object");
        }
        const std::filesystem::path path =
            directory_ / internal::kVisualDefinitionFileName;
        std::ofstream visual(path, std::ios::out | std::ios::binary);
        if (!visual) {
            Reject("could not open '" + path.string() + "'");
        }
        visual.write(visual_definition_json.data(),
                     static_cast<std::streamsize>(
                         visual_definition_json.size()));
        CloseChecked(visual, path);
        has_visual_definition_ = true;
    }
    const std::filesystem::path frames_path =
        directory_ / internal::kFramesFileName;
    frames_.open(frames_path, std::ios::out | std::ios::binary);
    if (!frames_) {
        Reject("could not open '" + frames_path.string() + "'");
    }
    const std::filesystem::path statuses_path =
        directory_ / internal::kStatusesFileName;
    statuses_.open(statuses_path, std::ios::out | std::ios::binary);
    if (!statuses_) {
        Reject("could not open '" + statuses_path.string() + "'");
    }
    row_bytes_.reserve(8 * (internal::kIdentityValueCount +
                            internal::kValuesPerBody * topology_.bodies.size() +
                            wheel_spin_angle_count_ +
                            scalar_definitions_.size()));
}

SceneRecordWriter::~SceneRecordWriter() = default;

void SceneRecordWriter::WriteFrame(const scene_observation::SceneFrame& frame) {
    if (closed_) {
        Reject("the record is already closed");
    }
    if (frame.bodies.size() != topology_.bodies.size()) {
        Reject("the frame holds " + std::to_string(frame.bodies.size()) +
               " bodies, but the topology names " +
               std::to_string(topology_.bodies.size()));
    }
    if (frame.scalars.values.size() != scalar_definitions_.size() ||
        frame.scalars.statuses.size() != scalar_definitions_.size()) {
        Reject("the frame's scalar count differs from the definitions");
    }
    if (frame.wheel_spin_angles_radians.size() != wheel_spin_angle_count_) {
        Reject("the frame carries " +
               std::to_string(frame.wheel_spin_angles_radians.size()) +
               " wheel spin angles, but the record was opened for " +
               std::to_string(wheel_spin_angle_count_));
    }
    row_bytes_.clear();
    AppendFinite(row_bytes_, frame.identity.time_seconds, "time_seconds");
    AppendIntegerIdentity(row_bytes_, frame.identity.time_nanoseconds,
                          "time_nanoseconds");
    AppendIntegerIdentity(row_bytes_, frame.identity.sample_index,
                          "sample_index");
    AppendLittleEndian(row_bytes_,
                       static_cast<double>(frame.identity.phase));
    for (const auto& body : frame.bodies) {
        for (double value : body.position_meters) {
            AppendFinite(row_bytes_, value, "body position");
        }
        for (double value : body.orientation_wxyz) {
            AppendFinite(row_bytes_, value, "body orientation");
        }
        for (double value : body.linear_velocity_meters_per_second) {
            AppendFinite(row_bytes_, value, "body linear velocity");
        }
        for (double value : body.angular_velocity_radians_per_second) {
            AppendFinite(row_bytes_, value, "body angular velocity");
        }
    }
    for (double angle : frame.wheel_spin_angles_radians) {
        AppendFinite(row_bytes_, angle, "wheel spin angle");
    }
    for (std::size_t index = 0; index < scalar_definitions_.size(); ++index) {
        const std::uint8_t status = frame.scalars.statuses[index];
        const double value = frame.scalars.values[index];
        if (status > static_cast<std::uint8_t>(ScalarStatus::kPlaceholder)) {
            Reject("scalar '" + scalar_definitions_[index].name +
                   "' has an unknown status");
        }
        if (status != static_cast<std::uint8_t>(ScalarStatus::kValid) &&
            value != 0.0) {
            Reject("scalar '" + scalar_definitions_[index].name +
                   "' is not valid but carries a nonzero value");
        }
        AppendFinite(row_bytes_, value, "scalar value");
    }
    frames_.write(reinterpret_cast<const char*>(row_bytes_.data()),
                  static_cast<std::streamsize>(row_bytes_.size()));
    if (!frames_) {
        Reject("could not write a frame row");
    }
    if (!scalar_definitions_.empty()) {
        statuses_.write(
            reinterpret_cast<const char*>(frame.scalars.statuses.data()),
            static_cast<std::streamsize>(frame.scalars.statuses.size()));
        if (!statuses_) {
            Reject("could not write a scalar status row");
        }
    }
    ++frame_count_;
}

void SceneRecordWriter::WriteTrackSampleTable(
    const scene_observation::TrackSampleTable& table) {
    if (closed_) {
        Reject("the record is already closed");
    }
    if (track_.has_value()) {
        Reject("the track table was already written");
    }
    const std::size_t count = table.stations_meters.size();
    if (count == 0 || table.centerline_in_inertial_meters.size() != count ||
        table.rotation_inertial_from_track_wxyz.size() != count ||
        table.curvature_radians_per_meter.size() != count ||
        table.superelevation_meters.size() != count ||
        table.left_rail_datum_in_inertial_meters.size() != count ||
        table.right_rail_datum_in_inertial_meters.size() != count) {
        Reject("the track table columns disagree in length or are empty");
    }
    track_ = table;
}

void SceneRecordWriter::Close() {
    if (closed_) {
        Reject("the record is already closed");
    }
    CloseChecked(frames_, directory_ / internal::kFramesFileName);
    CloseChecked(statuses_, directory_ / internal::kStatusesFileName);

    nlohmann::json bodies = nlohmann::json::array();
    for (const auto& body : topology_.bodies) {
        bodies.push_back({{"name", body.name},
                          {"moves_freely_in_world", body.moves_freely_in_world}});
    }
    nlohmann::json wheel_placements = nlohmann::json::array();
    for (const auto& wheel : topology_.wheel_placements) {
        wheel_placements.push_back(
            {{"interface_name", wheel.interface_name},
             {"wheel_body_name", wheel.wheel_body_name},
             {"carrier_body_name", wheel.carrier_body_name},
             {"side", SideName(wheel.side)},
             {"datum_in_wheel_body_frame_meters",
              Vector3Json(wheel.datum_in_wheel_body_frame_meters)},
             {"spin_axis_in_wheel_body_frame",
              Vector3Json(wheel.spin_axis_in_wheel_body_frame)},
             {"nominal_rolling_radius_meters",
              wheel.nominal_rolling_radius_meters}});
    }
    nlohmann::json scalars = nlohmann::json::array();
    for (const auto& definition : scalar_definitions_) {
        scalars.push_back({{"name", definition.name},
                           {"unit", definition.unit},
                           {"reference_frame", definition.reference_frame},
                           {"quantity", definition.quantity},
                           {"method", definition.method},
                           {"sample_semantics", definition.sample_semantics}});
    }
    nlohmann::json body_value_layout = nlohmann::json::array();
    for (const std::string_view name : internal::kBodyValueLayout) {
        body_value_layout.push_back(std::string(name));
    }
    const std::size_t body_count = topology_.bodies.size();
    const std::size_t wheel_spin_offset =
        internal::kIdentityValueCount + internal::kValuesPerBody * body_count;
    const std::size_t scalar_offset = wheel_spin_offset + wheel_spin_angle_count_;
    const std::size_t row_value_count = scalar_offset + scalar_definitions_.size();
    nlohmann::json track = nullptr;
    if (track_.has_value()) {
        track = {
            {"stations_meters", track_->stations_meters},
            {"centerline_in_inertial_meters",
             ArrayRowsJson(track_->centerline_in_inertial_meters)},
            {"rotation_inertial_from_track_wxyz",
             ArrayRowsJson(track_->rotation_inertial_from_track_wxyz)},
            {"curvature_radians_per_meter",
             track_->curvature_radians_per_meter},
            {"superelevation_meters", track_->superelevation_meters},
            {"left_rail_datum_in_inertial_meters",
             ArrayRowsJson(track_->left_rail_datum_in_inertial_meters)},
            {"right_rail_datum_in_inertial_meters",
             ArrayRowsJson(track_->right_rail_datum_in_inertial_meters)}};
    }
    const nlohmann::json scene = {
        {"record", std::string(internal::kRecordIdentifier)},
        {"world_frame",
         "ORVD model world frame; a track scenario's world frame is the track "
         "inertial frame I: +x along increasing station at the line origin, "
         "+y to the right, +z downward"},
        {"length_unit", "meter"},
        {"time_unit", "second"},
        {"angle_unit", "radian"},
        {"orientation",
         "unit quaternion [w, x, y, z] mapping body coordinates to world "
         "coordinates; a wheel body's orientation includes its spin"},
        {"bodies", bodies},
        {"wheel_placements", wheel_placements},
        {"visual_definition_file",
         has_visual_definition_
             ? nlohmann::json(std::string(internal::kVisualDefinitionFileName))
             : nlohmann::json(nullptr)},
        {"scalars", scalars},
        {"scalar_status_codes",
         {{"0", "not_ready"}, {"1", "valid"}, {"2", "placeholder"}}},
        {"phase_codes",
         {{"0", "initial_accepted_state"},
          {"1", "dense_intermediate_sample"},
          {"2", "accepted_endpoint"}}},
        {"frame_table",
         {{"file", std::string(internal::kFramesFileName)},
          {"encoding", "little-endian binary64 rows"},
          {"frame_count", frame_count_},
          {"row_value_count", row_value_count},
          {"columns",
           {{"time_seconds", internal::kTimeSecondsColumn},
            {"time_nanoseconds", internal::kTimeNanosecondsColumn},
            {"sample_index", internal::kSampleIndexColumn},
            {"phase", internal::kPhaseColumn},
            {"body_states",
             {{"offset", internal::kIdentityValueCount},
              {"body_count", body_count},
              {"values_per_body", internal::kValuesPerBody},
              {"value_layout", body_value_layout}}},
            {"wheel_spin_angles",
             {{"offset", wheel_spin_offset},
              {"count", wheel_spin_angle_count_},
              {"meaning",
               "unwrapped rotation of each wheel placement about its "
               "spin_axis_in_wheel_body_frame, radians, continuous in time; "
               "the body orientation already contains the spin, this only "
               "selects the rotation branch between two samples"}}},
            {"scalar_values",
             {{"offset", scalar_offset},
              {"count", scalar_definitions_.size()}}}}},
          {"absent_integer_identity", -1},
          {"integer_identity_exact_range",
           internal::kIntegerIdentityExactRange}}},
        {"scalar_status_table",
         {{"file", std::string(internal::kStatusesFileName)},
          {"frame_count", frame_count_},
          {"scalar_count", scalar_definitions_.size()}}},
        {"track", track}};

    const std::filesystem::path scene_path =
        directory_ / internal::kSceneFileName;
    std::ofstream scene_file(scene_path, std::ios::out | std::ios::binary);
    if (!scene_file) {
        Reject("could not open '" + scene_path.string() + "'");
    }
    scene_file << scene.dump(2) << '\n';
    CloseChecked(scene_file, scene_path);
    closed_ = true;
}

}  // namespace orvd::scene_record
