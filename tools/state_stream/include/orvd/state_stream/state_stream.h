#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "orvd/multibody_model/multibody_model.h"

namespace orvd::state_stream {

// All quantities refer to the body's declared frame origin, expressed in world.
// Quaternion order is w,x,y,z; rotation maps body coordinates to world.
struct BodyState {
    std::array<double, 3> position_meters{};
    std::array<double, 4> orientation_wxyz{};
    std::array<double, 3> linear_velocity_meters_per_second{};
    std::array<double, 3> angular_velocity_radians_per_second{};
};

enum class ScalarStatus : std::uint8_t { kNotReady = 0, kValid = 1, kPlaceholder = 2 };

struct ScalarDefinition {
    std::string name;
    std::string unit;
    std::string reference_frame;
    std::string quantity;
    std::string method;
    std::string sample_semantics;
};

struct ScalarValues {
    std::vector<double> values;
    std::vector<std::uint8_t> statuses;
};

// Resolves names/handles once. The model must outlive this sampler. Sampling
// only reads accepted kinematics; the caller owns the accepted-state boundary.
class BodyStateSampler {
 public:
    explicit BodyStateSampler(const multibody_model::MultibodyModel& model);
    [[nodiscard]] const std::vector<std::string>& names() const { return names_; }
    [[nodiscard]] std::vector<BodyState> Sample(
        const multibody_model::MultibodyEvaluationContext& context) const;
 private:
    const multibody_model::MultibodyModel* model_;
    std::vector<multibody_model::RigidBodyHandle> bodies_;
    std::vector<std::string> names_;
};

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
    std::span<const BodyState> bodies, const ScalarValues& scalars);
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
        const std::vector<ScalarDefinition>& scalars);
    void PublishDescription(std::uint64_t sequence, double time, const std::string& description);
    void PublishState(std::uint64_t sequence, double time, std::span<const BodyState> bodies,
                      const ScalarValues& scalars);
    [[nodiscard]] const StreamStatistics& statistics() const;
 private:
    struct Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace orvd::state_stream
