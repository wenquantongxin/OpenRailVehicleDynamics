#pragma once

#include <cstddef>
#include <string_view>

// The on-disk layout shared by the writer and the reader. The web viewer
// reads the same names from scene.json rather than from this header.

namespace orvd::scene_record::internal {

inline constexpr std::string_view kRecordIdentifier = "orvd.scene_record";
inline constexpr std::string_view kSceneFileName = "scene.json";
inline constexpr std::string_view kFramesFileName = "frames.f64le";
inline constexpr std::string_view kStatusesFileName = "scalar_statuses.u8";
inline constexpr std::string_view kVisualDefinitionFileName =
    "visual_definition.json";

inline constexpr std::size_t kTimeSecondsColumn = 0;
inline constexpr std::size_t kTimeNanosecondsColumn = 1;
inline constexpr std::size_t kSampleIndexColumn = 2;
inline constexpr std::size_t kPhaseColumn = 3;
inline constexpr std::size_t kIdentityValueCount = 4;
inline constexpr std::size_t kValuesPerBody = 13;

// Integer identities (time_nanoseconds, sample_index) travel in the binary64
// table. Every integer whose magnitude does not exceed 2^53 is represented
// exactly; the writer refuses anything beyond that instead of rounding it.
inline constexpr double kIntegerIdentityExactRange = 9007199254740992.0;

inline constexpr std::string_view kBodyValueLayout[kValuesPerBody] = {
    "position_x_meters",
    "position_y_meters",
    "position_z_meters",
    "orientation_w",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "linear_velocity_x_meters_per_second",
    "linear_velocity_y_meters_per_second",
    "linear_velocity_z_meters_per_second",
    "angular_velocity_x_radians_per_second",
    "angular_velocity_y_radians_per_second",
    "angular_velocity_z_radians_per_second",
};

}  // namespace orvd::scene_record::internal
