#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/scene_observation/body_state.h"
#include "orvd/scene_observation/scalar_definition.h"
#include "orvd/scene_observation/scene_description.h"

namespace orvd::scene_stream {

inline constexpr std::size_t kHeaderBytes = 40;
inline constexpr std::size_t kDatagramBytes = 1200;
inline constexpr std::size_t kFragmentPayloadBytes = kDatagramBytes - kHeaderBytes;
inline constexpr std::uint16_t kDescriptionKind = 1;
inline constexpr std::uint16_t kStateKind = 2;

struct Header {
    std::uint16_t kind{};
    std::uint64_t run_id{};
    std::uint64_t sequence{};
    double simulation_time_seconds{};
    std::uint16_t fragment_index{};
    std::uint16_t fragment_count{};
    std::uint32_t payload_bytes{};
};

struct StatePayload {
    std::vector<scene_observation::BodyState> bodies;
    scene_observation::ScalarValues scalars;
};

struct StreamDescription {
    scene_observation::SceneDescription description;
    /// Resource name resolved by a consumer's explicitly configured directory.
    std::optional<std::string> resource_bundle;
};

/// OSTS v1 uses explicit little-endian fields, independent of ABI and padding.
/// Invalid values, statuses or lengths throw std::invalid_argument.
[[nodiscard]] std::vector<std::uint8_t> EncodeState(
    std::span<const scene_observation::BodyState> bodies,
    const scene_observation::ScalarValues& scalars);
[[nodiscard]] StatePayload DecodeState(std::span<const std::uint8_t> payload);

/// Splits a complete message into datagrams. The sequence is supplied by the
/// producer and orders messages within one run and kind; it is not a record's
/// sample index. Wire size overflow throws std::length_error.
[[nodiscard]] std::vector<std::vector<std::uint8_t>> Packetize(
    std::uint16_t kind, std::uint64_t run_id, std::uint64_t sequence,
    double simulation_time_seconds, std::span<const std::uint8_t> payload);
/// Validates one complete datagram, including its fragment length.
[[nodiscard]] Header ParseHeader(std::span<const std::uint8_t> datagram);

[[nodiscard]] std::string EncodeDescription(const StreamDescription& description);
[[nodiscard]] StreamDescription DecodeDescription(std::string_view json);

}  // namespace orvd::scene_stream
