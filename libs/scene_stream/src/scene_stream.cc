#include "orvd/scene_stream/scene_stream.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace orvd::scene_stream {
namespace {
void Unsigned(std::vector<std::uint8_t>& out, std::uint64_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i)
        out.push_back(static_cast<std::uint8_t>(value >> (i * 8)));
}
void Real(std::vector<std::uint8_t>& out, double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite scene stream value");
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
    Unsigned(out, std::bit_cast<std::uint64_t>(value), 8);
}
class Reader {
 public:
    explicit Reader(std::span<const std::uint8_t> bytes) : bytes_(bytes) {}
    std::uint64_t Unsigned(unsigned width) {
        if (width > remaining()) throw std::invalid_argument("truncated scene stream payload");
        std::uint64_t value = 0;
        for (unsigned i = 0; i < width; ++i)
            value |= static_cast<std::uint64_t>(bytes_[offset_++]) << (i * 8);
        return value;
    }
    double Real() {
        const double value = std::bit_cast<double>(Unsigned(8));
        if (!std::isfinite(value)) throw std::invalid_argument("nonfinite scene stream value");
        return value;
    }
    std::size_t remaining() const { return bytes_.size() - offset_; }
 private:
    std::span<const std::uint8_t> bytes_;
    std::size_t offset_{};
};
void ValidateIdentity(std::uint16_t kind, std::uint64_t run_id, double time) {
    if ((kind != kDescriptionKind && kind != kStateKind) || run_id == 0 || !std::isfinite(time))
        throw std::invalid_argument("invalid scene stream kind, run identity or time");
}
void ValidateScalar(double value, std::uint8_t status) {
    constexpr auto valid = static_cast<std::uint8_t>(scene_observation::ScalarStatus::kValid);
    constexpr auto placeholder = static_cast<std::uint8_t>(scene_observation::ScalarStatus::kPlaceholder);
    if (!std::isfinite(value) || status > placeholder || (status != valid && value != 0.0))
        throw std::invalid_argument("scene stream scalar status/value inconsistent");
}
}  // namespace

std::vector<std::uint8_t> EncodeState(
    std::span<const scene_observation::BodyState> bodies,
    const scene_observation::ScalarValues& scalars) {
    if (bodies.size() > std::numeric_limits<std::uint32_t>::max() ||
        scalars.values.size() > std::numeric_limits<std::uint32_t>::max())
        throw std::length_error("scene stream count exceeds wire representation");
    if (scalars.values.size() != scalars.statuses.size())
        throw std::invalid_argument("scene stream scalar value/status counts differ");
    std::vector<std::uint8_t> out;
    out.reserve(8 + bodies.size() * 104 + scalars.values.size() * 9);
    Unsigned(out, bodies.size(), 4);
    for (const auto& body : bodies) {
        for (double value : body.position_meters) Real(out, value);
        for (double value : body.orientation_wxyz) Real(out, value);
        for (double value : body.linear_velocity_meters_per_second) Real(out, value);
        for (double value : body.angular_velocity_radians_per_second) Real(out, value);
    }
    Unsigned(out, scalars.values.size(), 4);
    for (std::size_t i = 0; i < scalars.values.size(); ++i) {
        ValidateScalar(scalars.values[i], scalars.statuses[i]);
        Real(out, scalars.values[i]);
    }
    out.insert(out.end(), scalars.statuses.begin(), scalars.statuses.end());
    return out;
}

StatePayload DecodeState(std::span<const std::uint8_t> payload) {
    Reader in(payload);
    const auto count = in.Unsigned(4);
    if (in.remaining() < 4 || count > (in.remaining() - 4) / 104)
        throw std::invalid_argument("scene stream body count exceeds payload");
    StatePayload state;
    state.bodies.resize(static_cast<std::size_t>(count));
    for (auto& body : state.bodies) {
        for (auto& value : body.position_meters) value = in.Real();
        for (auto& value : body.orientation_wxyz) value = in.Real();
        for (auto& value : body.linear_velocity_meters_per_second) value = in.Real();
        for (auto& value : body.angular_velocity_radians_per_second) value = in.Real();
    }
    const auto scalar_count = in.Unsigned(4);
    if (in.remaining() % 9 != 0 || scalar_count != in.remaining() / 9)
        throw std::invalid_argument("scene stream scalar count differs from payload");
    state.scalars.values.resize(static_cast<std::size_t>(scalar_count));
    state.scalars.statuses.resize(static_cast<std::size_t>(scalar_count));
    for (auto& value : state.scalars.values) value = in.Real();
    for (std::size_t i = 0; i < state.scalars.statuses.size(); ++i) {
        state.scalars.statuses[i] = static_cast<std::uint8_t>(in.Unsigned(1));
        ValidateScalar(state.scalars.values[i], state.scalars.statuses[i]);
    }
    return state;
}

std::vector<std::vector<std::uint8_t>> Packetize(
    std::uint16_t kind, std::uint64_t run_id, std::uint64_t sequence,
    double time, std::span<const std::uint8_t> payload) {
    ValidateIdentity(kind, run_id, time);
    constexpr std::size_t maximum = kFragmentPayloadBytes * std::numeric_limits<std::uint16_t>::max();
    if (payload.size() > maximum)
        throw std::length_error("scene stream message exceeds wire representation");
    const std::size_t parts = std::max<std::size_t>(1, (payload.size() + kFragmentPayloadBytes - 1) / kFragmentPayloadBytes);
    std::vector<std::vector<std::uint8_t>> packets;
    packets.reserve(parts);
    for (std::size_t part = 0; part < parts; ++part) {
        std::vector<std::uint8_t> packet{'O', 'S', 'T', 'S'};
        packet.reserve(kDatagramBytes);
        Unsigned(packet, 1, 2);
        Unsigned(packet, kind, 2);
        Unsigned(packet, run_id, 8);
        Unsigned(packet, sequence, 8);
        Real(packet, time);
        Unsigned(packet, part, 2);
        Unsigned(packet, parts, 2);
        Unsigned(packet, payload.size(), 4);
        const std::size_t start = part * kFragmentPayloadBytes;
        const std::size_t length = std::min(kFragmentPayloadBytes, payload.size() - start);
        packet.insert(packet.end(), payload.begin() + start, payload.begin() + start + length);
        packets.push_back(std::move(packet));
    }
    return packets;
}

Header ParseHeader(std::span<const std::uint8_t> datagram) {
    if (datagram.size() < kHeaderBytes || datagram.size() > kDatagramBytes)
        throw std::invalid_argument("invalid scene stream datagram length");
    Reader in(datagram);
    if (in.Unsigned(4) != 0x5354534f || in.Unsigned(2) != 1)
        throw std::invalid_argument("invalid scene stream magic or version");
    Header header;
    header.kind = static_cast<std::uint16_t>(in.Unsigned(2));
    header.run_id = in.Unsigned(8);
    header.sequence = in.Unsigned(8);
    header.simulation_time_seconds = in.Real();
    header.fragment_index = static_cast<std::uint16_t>(in.Unsigned(2));
    header.fragment_count = static_cast<std::uint16_t>(in.Unsigned(2));
    header.payload_bytes = static_cast<std::uint32_t>(in.Unsigned(4));
    ValidateIdentity(header.kind, header.run_id, header.simulation_time_seconds);
    const std::uint64_t parts = std::max<std::uint64_t>(1,
        (static_cast<std::uint64_t>(header.payload_bytes) + kFragmentPayloadBytes - 1) / kFragmentPayloadBytes);
    if (header.fragment_count != parts || header.fragment_index >= parts)
        throw std::invalid_argument("invalid scene stream fragment indices");
    const auto remaining = header.payload_bytes - static_cast<std::uint64_t>(header.fragment_index) * kFragmentPayloadBytes;
    if (in.remaining() != std::min<std::uint64_t>(kFragmentPayloadBytes, remaining))
        throw std::invalid_argument("invalid scene stream fragment length");
    return header;
}

std::string EncodeDescription(const StreamDescription& description) {
    if (description.resource_bundle && description.resource_bundle->empty())
        throw std::invalid_argument("resource_bundle must be nonempty");
    nlohmann::json document{{"schema", "orvd.scene_description"},
        {"description", nlohmann::json::parse(scene_observation::EncodeSceneDescription(description.description))},
        {"resource_bundle", nullptr}};
    if (description.resource_bundle) document["resource_bundle"] = *description.resource_bundle;
    return document.dump();
}

StreamDescription DecodeDescription(std::string_view json) {
    try {
        const auto document = nlohmann::json::parse(json);
        if (document.at("schema") != "orvd.scene_description")
            throw std::invalid_argument("invalid scene description schema");
        StreamDescription result{scene_observation::DecodeSceneDescription(document.at("description").dump()), std::nullopt};
        if (!document.at("resource_bundle").is_null()) {
            result.resource_bundle = document.at("resource_bundle").get<std::string>();
            if (result.resource_bundle->empty()) throw std::invalid_argument("resource_bundle must be nonempty");
        }
        return result;
    } catch (const nlohmann::json::exception& error) {
        throw std::invalid_argument(std::string("invalid scene description: ") + error.what());
    }
}

}  // namespace orvd::scene_stream
