#include "orvd/state_stream/state_stream.h"

#include <algorithm>
#include <bit>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <system_error>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

namespace orvd::state_stream {
namespace {
constexpr std::size_t kHeaderBytes = 40;
constexpr std::size_t kPayloadBytes = 1200 - kHeaderBytes;

void Unsigned(std::vector<std::uint8_t>& out, std::uint64_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) out.push_back(static_cast<std::uint8_t>(value >> (i * 8)));
}
void Real(std::vector<std::uint8_t>& out, double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite state stream value");
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
    Unsigned(out, std::bit_cast<std::uint64_t>(value), 8);
}
std::array<double, 3> Triple(const Eigen::Vector3d& value) {
    return {value.x(), value.y(), value.z()};
}
}  // namespace

BodyStateSampler::BodyStateSampler(const multibody_model::MultibodyModel& model) : model_(&model) {
    for (int i = 0; i < model.num_rigid_bodies(); ++i) {
        const auto body = model.GetRigidBody(i);
        bodies_.push_back(body);
        names_.emplace_back(model.GetRigidBodyName(body));
    }
}

std::vector<BodyState> BodyStateSampler::Sample(
    const multibody_model::MultibodyEvaluationContext& context) const {
    std::vector<BodyState> values;
    values.reserve(bodies_.size());
    for (const auto body : bodies_) {
        const auto pose = model_->CalcPoseInWorld(context, body);
        const Eigen::Quaterniond q(pose.rotation());
        const auto velocity = model_->CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(context, body);
        values.push_back({Triple(pose.translation()), {q.w(), q.x(), q.y(), q.z()},
            Triple(velocity.translational_velocity_at_frame_origin_meters_per_second()),
            Triple(velocity.angular_velocity_radians_per_second())});
    }
    return values;
}

std::vector<std::uint8_t> EncodeState(std::span<const BodyState> bodies, const ScalarValues& scalars) {
    if (bodies.size() > std::numeric_limits<std::uint32_t>::max() ||
        scalars.values.size() > std::numeric_limits<std::uint32_t>::max())
        throw std::length_error("state stream count exceeds wire representation");
    if (scalars.values.size() != scalars.statuses.size())
        throw std::invalid_argument("state stream scalar value/status counts differ");
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
        const auto status = scalars.statuses[i];
        if (status > 2 || (status != 1 && scalars.values[i] != 0.0))
            throw std::invalid_argument("state stream scalar status/value inconsistent");
        Real(out, scalars.values[i]);
    }
    out.insert(out.end(), scalars.statuses.begin(), scalars.statuses.end());
    return out;
}

std::vector<std::vector<std::uint8_t>> Packetize(
    std::uint16_t kind, std::uint64_t run_id, std::uint64_t sequence,
    double time, std::span<const std::uint8_t> payload) {
    const std::size_t parts = std::max<std::size_t>(1, (payload.size() + kPayloadBytes - 1) / kPayloadBytes);
    if (parts > std::numeric_limits<std::uint16_t>::max() ||
        payload.size() > std::numeric_limits<std::uint32_t>::max())
        throw std::length_error("state stream message exceeds wire representation");
    std::vector<std::vector<std::uint8_t>> packets;
    packets.reserve(parts);
    for (std::size_t part = 0; part < parts; ++part) {
        std::vector<std::uint8_t> packet{'O', 'S', 'T', 'S'};
        packet.reserve(1200);
        Unsigned(packet, 1, 2);
        Unsigned(packet, kind, 2);
        Unsigned(packet, run_id, 8);
        Unsigned(packet, sequence, 8);
        Real(packet, time);
        Unsigned(packet, part, 2);
        Unsigned(packet, parts, 2);
        Unsigned(packet, payload.size(), 4);
        const std::size_t start = part * kPayloadBytes;
        const std::size_t length = std::min(kPayloadBytes, payload.size() - start);
        packet.insert(packet.end(), payload.begin() + start, payload.begin() + start + length);
        packets.push_back(std::move(packet));
    }
    return packets;
}

struct UdpStateStream::Implementation {
    int socket{-1};
    std::vector<sockaddr_in> addresses;
    StreamStatistics statistics;
    ~Implementation() { if (socket >= 0) ::close(socket); }
    void Send(std::uint16_t kind, std::uint64_t sequence, double time,
              std::span<const std::uint8_t> payload) {
        const auto packets = Packetize(kind, statistics.run_id, sequence, time, payload);
        for (std::size_t i = 0; i < addresses.size(); ++i) {
            auto& counters = statistics.destinations[i];
            for (const auto& packet : packets) {
                const auto result = ::sendto(socket, packet.data(), packet.size(), MSG_DONTWAIT,
                    reinterpret_cast<const sockaddr*>(&addresses[i]), sizeof(sockaddr_in));
                if (result == static_cast<ssize_t>(packet.size())) {
                    ++counters.sent_datagrams;
                    counters.sent_bytes += packet.size();
                } else {
                    ++counters.failed_datagrams;
                    counters.last_error = result < 0 ? errno : EIO;
                }
            }
        }
    }
};

UdpStateStream::UdpStateStream(const std::vector<std::string>& destinations)
    : implementation_(std::make_unique<Implementation>()) {
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
    // Runtime session label, not a source revision or content digest.
    implementation_->statistics.run_id = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}
UdpStateStream::~UdpStateStream() = default;

std::string UdpStateStream::Describe(const std::vector<std::string>& names, const std::string& world_frame,
                                    const std::vector<ScalarDefinition>& scalars) {
    auto definitions = nlohmann::json::array();
    for (const auto& field : scalars) {
        definitions.push_back({{"name", field.name}, {"unit", field.unit},
            {"reference_frame", field.reference_frame}, {"quantity", field.quantity},
            {"method", field.method}, {"sample_semantics", field.sample_semantics}});
    }
    return nlohmann::json{{"schema", "orvd.state-stream"},
        {"world_frame", world_frame}, {"body_names", names},
        {"position", "body-frame origin in world, meters"},
        {"orientation", "quaternion w,x,y,z; body to world; wheel spin included"},
        {"linear_velocity", "body-frame origin in world, meters/second"},
        {"angular_velocity", "body relative to world expressed in world, radians/second"},
        {"body_scalar_count", 13}, {"scalars", definitions},
        {"scalar_statuses", {{"0", "not-ready"}, {"1", "valid"}, {"2", "placeholder"}}}}.dump();
}
void UdpStateStream::PublishDescription(std::uint64_t sequence, double time, const std::string& description) {
    implementation_->Send(1, sequence, time, std::span(
        reinterpret_cast<const std::uint8_t*>(description.data()), description.size()));
}
void UdpStateStream::PublishState(std::uint64_t sequence, double time, std::span<const BodyState> bodies,
                                 const ScalarValues& scalars) {
    const auto payload = EncodeState(bodies, scalars);
    ++implementation_->statistics.attempted_state_frames;
    implementation_->Send(2, sequence, time, payload);
}
const StreamStatistics& UdpStateStream::statistics() const { return implementation_->statistics; }

}  // namespace orvd::state_stream
