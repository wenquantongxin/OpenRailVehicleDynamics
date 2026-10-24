#include "orvd/state_stream/state_stream.h"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <system_error>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "orvd/scene_stream/scene_stream.h"

namespace orvd::state_stream {
struct UdpStateStream::Implementation {
    using Clock = std::chrono::steady_clock;
    int socket{-1};
    std::vector<sockaddr_in> addresses;
    StreamStatistics statistics;
    double retry_budget_seconds{0.0};
    ~Implementation() { if (socket >= 0) ::close(socket); }
    bool WaitWritable(Clock::time_point started, DestinationStatistics& counters, int& error) {
        for (;;) {
            const double remaining = retry_budget_seconds -
                std::chrono::duration<double>(Clock::now() - started).count();
            if (remaining <= 0.0) return false;
            // poll's timeout is in integer milliseconds. Keep a positive remainder
            // positive, without overflowing for a large caller-selected budget.
            const int timeout = static_cast<int>(std::min(
                std::ceil(remaining * 1000.0),
                static_cast<double>(std::numeric_limits<int>::max())));
            pollfd descriptor{socket, POLLOUT, 0};
            const auto wait_started = Clock::now();
            const int ready = ::poll(&descriptor, 1, timeout);
            const int poll_error = errno;
            counters.retry_wait_seconds +=
                std::chrono::duration<double>(Clock::now() - wait_started).count();
            if (ready < 0 && poll_error == EINTR) continue;
            if (ready < 0) { error = poll_error; return false; }
            if (ready == 0) return false;
            if ((descriptor.revents & POLLOUT) == 0) {
                error = (descriptor.revents & POLLNVAL) != 0 ? EBADF : EIO;
                return false;
            }
            return std::chrono::duration<double>(Clock::now() - started).count() <
                   retry_budget_seconds;
        }
    }
    void Send(std::uint16_t kind, std::uint64_t sequence, double time,
              std::span<const std::uint8_t> payload) {
        const auto packets = scene_stream::Packetize(kind, statistics.run_id, sequence, time, payload);
        const auto started = Clock::now();
        for (std::size_t i = 0; i < addresses.size(); ++i) {
            auto& counters = statistics.destinations[i];
            for (const auto& packet : packets) {
                bool retried = false;
                for (;;) {
                    const auto result = ::sendto(socket, packet.data(), packet.size(), MSG_DONTWAIT,
                        reinterpret_cast<const sockaddr*>(&addresses[i]), sizeof(sockaddr_in));
                    if (result == static_cast<ssize_t>(packet.size())) {
                        ++counters.sent_datagrams;
                        counters.sent_bytes += packet.size();
                        if (retried) ++counters.retried_datagrams;
                        break;
                    }
                    int error = result < 0 ? errno : EIO;
                    if (error == EAGAIN || error == EWOULDBLOCK) {
                        ++counters.would_block_events;
                        if (WaitWritable(started, counters, error)) {
                            ++counters.retry_attempts;
                            retried = true;
                            continue;
                        }
                    }
                    ++counters.failed_datagrams;
                    counters.last_error = error;
                    break;
                }
            }
        }
    }
};

UdpStateStream::UdpStateStream(const std::vector<std::string>& destinations,
                               std::optional<std::uint64_t> run_id,
                               const UdpSendOptions& options)
    : implementation_(std::make_unique<Implementation>()) {
    if (run_id && *run_id == 0) throw std::invalid_argument("state stream run identity must be nonzero");
    if ((options.socket_send_buffer_bytes && *options.socket_send_buffer_bytes <= 0) ||
        !std::isfinite(options.retry_budget_seconds) || options.retry_budget_seconds < 0.0)
        throw std::invalid_argument("send buffer must be positive and retry budget finite and nonnegative");
    if (destinations.empty()) throw std::invalid_argument("state stream needs a destination");
    for (const auto& target : destinations) {
        const auto colon = target.rfind(':');
        if (colon == std::string::npos) throw std::invalid_argument("UDP target must be IPv4:port");
        unsigned port{};
        const auto first = target.data() + colon + 1;
        const auto last = target.data() + target.size();
        const auto [end, error] = std::from_chars(first, last, port);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        if (error != std::errc{} || end != last || port == 0 || port > 65535 ||
            ::inet_pton(AF_INET, target.substr(0, colon).c_str(), &address.sin_addr) != 1)
            throw std::invalid_argument("UDP target must be IPv4:port with port 1..65535");
        address.sin_port = htons(static_cast<std::uint16_t>(port));
        implementation_->addresses.push_back(address);
        implementation_->statistics.destinations.push_back({target});
    }
    implementation_->socket = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (implementation_->socket < 0) throw std::system_error(errno, std::generic_category(), "UDP socket");
    if (options.socket_send_buffer_bytes &&
        ::setsockopt(implementation_->socket, SOL_SOCKET, SO_SNDBUF,
                     &*options.socket_send_buffer_bytes, sizeof(int)) < 0)
        throw std::system_error(errno, std::generic_category(), "UDP send buffer request");
    auto& statistics = implementation_->statistics;
    statistics.requested_socket_send_buffer_bytes = options.socket_send_buffer_bytes;
    socklen_t length = sizeof(statistics.effective_socket_send_buffer_bytes);
    if (::getsockopt(implementation_->socket, SOL_SOCKET, SO_SNDBUF,
                     &statistics.effective_socket_send_buffer_bytes, &length) < 0)
        throw std::system_error(errno, std::generic_category(), "UDP effective send buffer");
    implementation_->retry_budget_seconds = options.retry_budget_seconds;
    // Runtime session label, not a source revision or content digest.
    implementation_->statistics.run_id = run_id ? *run_id : static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}
UdpStateStream::~UdpStateStream() = default;

void UdpStateStream::PublishDescription(std::uint64_t sequence, double time, const std::string& description) {
    implementation_->Send(1, sequence, time, std::span(
        reinterpret_cast<const std::uint8_t*>(description.data()), description.size()));
}
void UdpStateStream::PublishState(std::uint64_t sequence, double time,
                                 std::span<const scene_observation::BodyState> bodies,
                                 const scene_observation::ScalarValues& scalars) {
    const auto payload = scene_stream::EncodeState(bodies, scalars);
    ++implementation_->statistics.attempted_state_frames;
    implementation_->Send(2, sequence, time, payload);
}
const StreamStatistics& UdpStateStream::statistics() const { return implementation_->statistics; }

}  // namespace orvd::state_stream
