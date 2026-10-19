#pragma once

#include <cstdint>
#include <memory>
#include <optional>
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
    std::uint64_t would_block_events{0};
    std::uint64_t retry_attempts{0};
    /// Datagrams eventually sent after at least one additional send attempt.
    std::uint64_t retried_datagrams{0};
    double retry_wait_seconds{0.0};
};
struct StreamStatistics {
    std::uint64_t run_id{0};
    std::uint64_t attempted_state_frames{0};
    std::vector<DestinationStatistics> destinations;
    std::optional<int> requested_socket_send_buffer_bytes;
    int effective_socket_send_buffer_bytes{0};
};

struct UdpSendOptions {
    /// Positive SO_SNDBUF request, or the operating system default when absent.
    /// The effective value is reported separately; the kernel may adjust it.
    std::optional<int> socket_send_buffer_bytes{};
    /// One wall-clock budget shared by all fragments and destinations of each
    /// Publish call. Zero never waits or retries. Only EAGAIN/EWOULDBLOCK retry.
    double retry_budget_seconds{0.0};
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

// Linux IPv4 unicast sender: explicit IP:port targets and nonblocking socket,
// with optional bounded waiting for local send capacity. No worker thread,
// queue, acknowledgements or receiver-driven retransmission. Runtime network
// errors are counted; a temporary EAGAIN is not a failed datagram if retry succeeds.
class UdpStateStream {
 public:
    // A caller may supply its already allocated run identity; zero is invalid.
    // Otherwise one identity is allocated when this sender is constructed.
    explicit UdpStateStream(const std::vector<std::string>& destinations,
                            std::optional<std::uint64_t> run_id = std::nullopt,
                            const UdpSendOptions& options = {});
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
