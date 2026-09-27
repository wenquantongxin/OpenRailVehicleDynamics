#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "orvd/scene_observation/body_state.h"
#include "orvd/scene_observation/scalar_definition.h"

namespace orvd::state_stream {

struct DestinationStatistics {
    std::string destination;
    std::uint64_t sent_datagrams{0};
    std::uint64_t sent_bytes{0};
    std::uint64_t failed_datagrams{0};
    int last_error{0};
};
struct StreamStatistics {
    std::uint64_t run_id{0};
    std::uint64_t attempted_state_frames{0};
    std::vector<DestinationStatistics> destinations;
};

// Explicit little-endian protocol, independent of ABI/padding. A state payload
// holds bodies followed by a scalar count, contiguous binary64 values, and
// contiguous byte statuses. No ABI-dependent casts or padding.
[[nodiscard]] std::vector<std::uint8_t> EncodeState(
    std::span<const scene_observation::BodyState> bodies,
    const scene_observation::ScalarValues& scalars);
[[nodiscard]] std::vector<std::vector<std::uint8_t>> Packetize(
    std::uint16_t kind, std::uint64_t run_id, std::uint64_t sequence,
    double simulation_time_seconds, std::span<const std::uint8_t> payload);

// Linux IPv4 unicast sender: explicit IP:port targets, nonblocking, no worker
// thread, queue, acknowledgements or retries. Runtime network errors are counted.
class UdpStateStream {
 public:
    explicit UdpStateStream(const std::vector<std::string>& destinations);
    ~UdpStateStream();
    UdpStateStream(const UdpStateStream&) = delete;
    UdpStateStream& operator=(const UdpStateStream&) = delete;
    [[nodiscard]] static std::string Describe(
        const std::vector<std::string>& names, const std::string& world_frame,
        const std::vector<scene_observation::ScalarDefinition>& scalars);
    void PublishDescription(std::uint64_t sequence, double time, const std::string& description);
    void PublishState(std::uint64_t sequence, double time,
                      std::span<const scene_observation::BodyState> bodies,
                      const scene_observation::ScalarValues& scalars);
    [[nodiscard]] const StreamStatistics& statistics() const;
 private:
    struct Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::state_stream
