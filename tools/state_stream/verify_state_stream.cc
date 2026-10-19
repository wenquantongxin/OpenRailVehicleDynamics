#include "orvd/state_stream/state_stream.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <thread>

#include <poll.h>
#include <sys/socket.h>

namespace {
// Linker wrappers exercise the real socket setup and send loop without relying
// on kernel queue timing or publishing any test packets to another process.
enum class SendOutcome { kSuccess, kBlockSecondOnce, kAlwaysBlock, kNetworkError };
struct SendProbe {
    SendOutcome outcome{SendOutcome::kSuccess};
    bool exhaust_budget{false};
    std::chrono::milliseconds first_send_delay{0};
    std::size_t attempts{0};
    std::vector<int> poll_timeouts_milliseconds{};
    std::vector<std::vector<std::uint8_t>> successful_packets{};
};
SendProbe probe;

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void VerifySendOptions(
    const std::vector<orvd::scene_observation::BodyState>& bodies,
    const orvd::scene_observation::ScalarValues& scalars,
    const std::vector<std::vector<std::uint8_t>>& packets,
    std::uint64_t run_id) {
    using orvd::state_stream::UdpSendOptions;
    using orvd::state_stream::UdpStateStream;
    constexpr auto retry_budget = std::chrono::milliseconds(500);
    const std::vector<std::string> destination{"127.0.0.1:10099"};
    UdpStateStream configured(destination, run_id,
                              {.socket_send_buffer_bytes = 8192,
                               .retry_budget_seconds = std::chrono::duration<double>(retry_budget).count()});
    Require(configured.statistics().requested_socket_send_buffer_bytes == 8192 &&
                configured.statistics().effective_socket_send_buffer_bytes > 0,
            "send buffer request or effective value missing");

    probe = {.outcome = SendOutcome::kBlockSecondOnce};
    configured.PublishState(37, 0.37, bodies, scalars);
    const auto& retried = configured.statistics().destinations.front();
    Require(probe.attempts == 4 && probe.poll_timeouts_milliseconds.size() == 1 &&
                probe.successful_packets == packets,
            "retry changed fragment order or repeated a successful fragment");
    Require(retried.sent_datagrams == 3 && retried.sent_bytes == 3124 &&
                retried.failed_datagrams == 0 && retried.would_block_events == 1 &&
                retried.retry_attempts == 1 && retried.retried_datagrams == 1 &&
                retried.retry_wait_seconds >= 0.0 && configured.statistics().attempted_state_frames == 1,
            "successful retry counters are inconsistent");

    UdpStateStream immediate(destination, run_id);
    Require(!immediate.statistics().requested_socket_send_buffer_bytes &&
                immediate.statistics().effective_socket_send_buffer_bytes > 0,
            "default socket buffer was not reported");
    probe = {.outcome = SendOutcome::kBlockSecondOnce};
    immediate.PublishState(37, 0.37, bodies, scalars);
    const auto& dropped = immediate.statistics().destinations.front();
    Require(probe.attempts == 3 && probe.poll_timeouts_milliseconds.empty() &&
                probe.successful_packets == std::vector{packets.front(), packets.back()} &&
                dropped.failed_datagrams == 1 && dropped.last_error == EAGAIN &&
                dropped.would_block_events == 1 && dropped.retry_attempts == 0 &&
                dropped.retried_datagrams == 0 && dropped.retry_wait_seconds == 0.0,
            "zero budget waited or retried a failed datagram");

    // A delayed first send can leave only part of the budget, or consume it
    // before poll. Neither case grants a fresh budget to subsequent fragments
    // or destinations. A poll, if reached, consumes the remaining budget.
    for (const int delay_milliseconds : {0, 3, 10}) {
        UdpStateStream bounded({destination.front(), "127.0.0.1:10100"}, run_id,
                               {.retry_budget_seconds = 0.005});
        probe = {.outcome = SendOutcome::kAlwaysBlock, .exhaust_budget = true,
                 .first_send_delay = std::chrono::milliseconds(delay_milliseconds)};
        bounded.PublishState(37, 0.37, bodies, scalars);
        Require(probe.attempts == 6 && probe.poll_timeouts_milliseconds.size() <= 1 &&
                    probe.successful_packets.empty(),
                "retry budget was not shared by fragments and destinations");
        for (const auto& counters : bounded.statistics().destinations) {
            Require(counters.failed_datagrams == 3 && counters.would_block_events == 3 &&
                        counters.retry_attempts == 0 && counters.retried_datagrams == 0 &&
                        counters.last_error == EAGAIN,
                    "exhausted retry counters are inconsistent");
        }
        const double wait_seconds = bounded.statistics().destinations.front().retry_wait_seconds;
        if (probe.poll_timeouts_milliseconds.empty()) {
            Require(wait_seconds == 0.0, "wait time recorded without polling");
        } else {
            const int timeout = probe.poll_timeouts_milliseconds.front();
            Require(timeout > 0 && timeout <= 5, "poll timeout exceeded the Publish budget");
            Require(std::isfinite(wait_seconds) &&
                        wait_seconds >= std::chrono::duration<double>(
                            std::chrono::milliseconds(timeout)).count(),
                    "poll wait was not included in destination statistics");
        }
        Require(bounded.statistics().destinations.back().retry_wait_seconds == 0.0,
                "exhausted budget was renewed for another destination");
        if (delay_milliseconds >= 5) {
            Require(probe.poll_timeouts_milliseconds.empty(), "polled after budget exhaustion");
        }
    }

    UdpStateStream network_error(destination, run_id, {.retry_budget_seconds = 0.5});
    probe = {.outcome = SendOutcome::kNetworkError};
    network_error.PublishState(37, 0.37, bodies, scalars);
    Require(probe.attempts == 3 && probe.poll_timeouts_milliseconds.empty() &&
                network_error.statistics().destinations.front().failed_datagrams == 3 &&
                network_error.statistics().destinations.front().last_error == ENETUNREACH &&
                network_error.statistics().destinations.front().would_block_events == 0,
            "non-capacity network error was retried");

    // After the earlier state Publish's entire budget has elapsed, a description
    // Publish must still be able to retry using its own budget. Both use the same
    // send path and retain the exact wire bytes.
    const std::string description = "{\"text\":\"" + std::string(2400, 'x') + "\"}";
    const auto description_packets = orvd::state_stream::Packetize(
        1, run_id, 40, 0.4, std::span(
            reinterpret_cast<const std::uint8_t*>(description.data()), description.size()));
    std::this_thread::sleep_for(retry_budget);
    probe = {.outcome = SendOutcome::kBlockSecondOnce};
    configured.PublishDescription(40, 0.4, description);
    Require(probe.attempts == 4 && probe.poll_timeouts_milliseconds.size() == 1 &&
                probe.successful_packets == description_packets,
            "new Publish did not receive a fresh retry budget or changed its wire bytes");

    for (const auto& options : std::vector<UdpSendOptions>{
             {.socket_send_buffer_bytes = 0}, {.socket_send_buffer_bytes = -1},
             {.retry_budget_seconds = -0.1},
             {.retry_budget_seconds = std::numeric_limits<double>::infinity()},
             {.retry_budget_seconds = std::numeric_limits<double>::quiet_NaN()}}) {
        bool rejected = false;
        try { UdpStateStream invalid(destination, run_id, options); }
        catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected, "invalid UDP send option accepted");
    }
}
}  // namespace

extern "C" ssize_t __wrap_sendto(int, const void* data, std::size_t size, int flags,
                                  const sockaddr*, socklen_t) {
    Require((flags & MSG_DONTWAIT) != 0, "send operation is not nonblocking");
    const auto attempt = probe.attempts++;
    if (attempt == 0 && probe.first_send_delay.count() > 0) {
        std::this_thread::sleep_for(probe.first_send_delay);
    }
    if (probe.outcome == SendOutcome::kNetworkError) { errno = ENETUNREACH; return -1; }
    if (probe.outcome == SendOutcome::kAlwaysBlock ||
        (probe.outcome == SendOutcome::kBlockSecondOnce && attempt == 1)) {
        errno = EAGAIN;
        return -1;
    }
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    probe.successful_packets.emplace_back(bytes, bytes + size);
    return static_cast<ssize_t>(size);
}

extern "C" int __wrap_poll(pollfd* descriptors, nfds_t count, int timeout) {
    Require(count == 1 && descriptors[0].events == POLLOUT && timeout > 0,
            "invalid bounded capacity poll");
    probe.poll_timeouts_milliseconds.push_back(timeout);
    if (probe.exhaust_budget) {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
        return 0;
    }
    descriptors[0].revents = POLLOUT;
    return 1;
}

// The wire protocol and sender boundary of the optional UDP component. Body
// sampling itself is verified with the public scene_observation library.
int main(int argc, char** argv) {
    using namespace orvd;
    try {
        scene_observation::BodyState state;
        state.position_meters = {1.5, -2.25, 0.75};
        state.orientation_wxyz = {std::cos(0.2), 0.0, 0.0, std::sin(0.2)};
        state.angular_velocity_radians_per_second = {0.3, -0.7, 1.2};
        state.linear_velocity_meters_per_second = {2.0, 3.0, -4.0};
        std::vector<scene_observation::BodyState> many(25, state);
        many[0].linear_velocity_meters_per_second[0] = -0.0;
        scene_observation::ScalarValues scalars;
        scalars.values.assign(44, 0.0);
        scalars.statuses.assign(44, 1);
        scalars.values[0] = -0.0;
        scalars.values[1] = 2.75;
        scalars.statuses[2] = 0;
        scalars.statuses[3] = 2;
        const auto payload = state_stream::EncodeState(many, scalars);
        if (payload.size() != 3004 || payload[4 + 7 * 8 + 7] != 128 ||
            payload[2608 + 7] != 128 || payload[2960 + 2] != 0 || payload[2960 + 3] != 2)
            throw std::runtime_error("wire layout lost a signed zero");
        if (state_stream::EncodeState({}, {}).size() != 8)
            throw std::runtime_error("empty state counts missing");
        bool bad_scalar = false;
        try { (void)state_stream::EncodeState({}, {{1.0}, {2}}); }
        catch (const std::invalid_argument&) { bad_scalar = true; }
        if (!bad_scalar) throw std::runtime_error("nonzero placeholder accepted");
        constexpr std::uint64_t run_id = 9223372038880858309ULL;
        const auto packets = state_stream::Packetize(2, run_id, 37, 0.37, payload);
        std::vector<std::uint8_t> recovered;
        for (const auto& packet : packets) {
            if (packet.size() > 1200 || packet.size() < 40)
                throw std::runtime_error("invalid packet size");
            recovered.insert(recovered.end(), packet.begin() + 40, packet.end());
        }
        if (packets.size() != 3 || recovered != payload)
            throw std::runtime_error("fragmentation changed payload");
        const std::string description = state_stream::UdpStateStream::Describe(
            {"test_body"}, "world", {{"speed", "m/s", "world", "speed", "sampled", "instantaneous"}});
        if (description.find("orvd.state-stream") == std::string::npos ||
            description.find("\"speed\"") == std::string::npos)
            throw std::runtime_error("description lost its schema or scalar names");
        bool invalid_rejected = false;
        try { state_stream::UdpStateStream invalid({"127.0.0.1:70000"}); }
        catch (const std::invalid_argument&) { invalid_rejected = true; }
        if (!invalid_rejected) throw std::runtime_error("invalid destination accepted");
        // Socket construction alone does not publish data. Both creation paths
        // use the same sender, and the wire identity retains all 64 bits.
        state_stream::UdpStateStream identified({"127.0.0.1:10099"}, run_id);
        state_stream::UdpStateStream automatic({"127.0.0.1:10099"});
        if (identified.statistics().run_id != run_id || automatic.statistics().run_id == 0)
            throw std::runtime_error("sender did not preserve the selected run identity");
        bool zero_id_rejected = false;
        try { state_stream::UdpStateStream invalid({"127.0.0.1:10099"}, 0); }
        catch (const std::invalid_argument&) { zero_id_rejected = true; }
        if (!zero_id_rejected) throw std::runtime_error("zero run identity accepted");
        std::uint64_t encoded_run_id = 0;
        for (unsigned i = 0; i < 8; ++i)
            encoded_run_id |= static_cast<std::uint64_t>(packets.front()[8 + i]) << (8 * i);
        if (encoded_run_id != identified.statistics().run_id)
            throw std::runtime_error("wire run identity differs from sender identity");
        VerifySendOptions(many, scalars, packets, run_id);
        if (argc == 2) {
            std::filesystem::create_directories(argv[1]);
            for (std::size_t i = 0; i < packets.size(); ++i) {
                std::ofstream out(std::filesystem::path(argv[1]) / (std::to_string(i) + ".bin"), std::ios::binary);
                out.write(reinterpret_cast<const char*>(packets[i].data()), static_cast<std::streamsize>(packets[i].size()));
                if (!out) throw std::runtime_error("fixture write failed");
            }
        }
        std::puts("state_stream encoding, fragments, send buffer and bounded retries passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
