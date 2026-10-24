#include "orvd/scene_record/scene_record_writer.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "scene_assets_json.h"
#include "scene_record_layout.h"

namespace orvd::scene_record {
namespace {

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
    scene_observation::SceneDescription description,
    SceneAssets assets, bool with_wheel_spin_angles)
    : directory_(directory),
      description_(std::move(description)),
      assets_(std::move(assets)),
      wheel_spin_angle_count_(
          with_wheel_spin_angles ? description_.topology.wheel_placements.size() : 0U) {
    try {
        (void)scene_observation::EncodeSceneDescription(description_);
    } catch (const std::invalid_argument& error) {
        Reject(error.what());
    }
    internal::ValidateSceneAssets(assets_);
    if (description_.world_frame != assets_.world_frame) {
        Reject("description and assets use different world frames");
    }
    internal::CreateSceneDirectory(directory_);
    internal::WriteVisualDefinition(directory_, assets_);
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
                            internal::kValuesPerBody * description_.topology.bodies.size() +
                            wheel_spin_angle_count_ +
                            description_.scalar_definitions.size()));
}

SceneRecordWriter::~SceneRecordWriter() = default;

void SceneRecordWriter::WriteFrame(const scene_observation::SceneFrame& frame) {
    if (closed_) {
        Reject("the record is already closed");
    }
    if (frame.bodies.size() != description_.topology.bodies.size()) {
        Reject("the frame holds " + std::to_string(frame.bodies.size()) +
               " bodies, but the topology names " +
               std::to_string(description_.topology.bodies.size()));
    }
    if (frame.scalars.values.size() != description_.scalar_definitions.size() ||
        frame.scalars.statuses.size() != description_.scalar_definitions.size()) {
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
    for (std::size_t index = 0; index < description_.scalar_definitions.size(); ++index) {
        const std::uint8_t status = frame.scalars.statuses[index];
        const double value = frame.scalars.values[index];
        if (status > static_cast<std::uint8_t>(ScalarStatus::kPlaceholder)) {
            Reject("scalar '" + description_.scalar_definitions[index].name +
                   "' has an unknown status");
        }
        if (status != static_cast<std::uint8_t>(ScalarStatus::kValid) &&
            value != 0.0) {
            Reject("scalar '" + description_.scalar_definitions[index].name +
                   "' is not valid but carries a nonzero value");
        }
        AppendFinite(row_bytes_, value, "scalar value");
    }
    frames_.write(reinterpret_cast<const char*>(row_bytes_.data()),
                  static_cast<std::streamsize>(row_bytes_.size()));
    if (!frames_) {
        Reject("could not write a frame row");
    }
    if (!description_.scalar_definitions.empty()) {
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
    if (assets_.track.has_value()) {
        Reject("the track table was already written");
    }
    internal::ValidateTrackSampleTable(table);
    assets_.track = table;
}

void SceneRecordWriter::Close() {
    if (closed_) {
        Reject("the record is already closed");
    }
    CloseChecked(frames_, directory_ / internal::kFramesFileName);
    CloseChecked(statuses_, directory_ / internal::kStatusesFileName);

    nlohmann::json body_value_layout = nlohmann::json::array();
    for (const std::string_view name : internal::kBodyValueLayout) {
        body_value_layout.push_back(std::string(name));
    }
    const std::size_t body_count = description_.topology.bodies.size();
    const std::size_t wheel_spin_offset =
        internal::kIdentityValueCount + internal::kValuesPerBody * body_count;
    const std::size_t scalar_offset = wheel_spin_offset + wheel_spin_angle_count_;
    const std::size_t row_value_count = scalar_offset + description_.scalar_definitions.size();
    const nlohmann::json scene = {
        {"schema", std::string(internal::kRecordIdentifier)},
        {"description", nlohmann::json::parse(
             scene_observation::EncodeSceneDescription(description_))},
        {"assets", internal::EncodeSceneAssets(assets_)},
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
              {"count", description_.scalar_definitions.size()}}}}},
          {"absent_integer_identity", -1},
          {"integer_identity_exact_range",
           internal::kIntegerIdentityExactRange}}},
        {"scalar_status_table",
         {{"file", std::string(internal::kStatusesFileName)},
          {"frame_count", frame_count_},
          {"scalar_count", description_.scalar_definitions.size()}}}};

    internal::WriteSceneDocumentLast(directory_, scene);
    closed_ = true;
}

}  // namespace orvd::scene_record
