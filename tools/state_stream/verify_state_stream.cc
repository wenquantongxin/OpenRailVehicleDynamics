#include "orvd/state_stream/state_stream.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

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
        if (argc == 2) {
            std::filesystem::create_directories(argv[1]);
            for (std::size_t i = 0; i < packets.size(); ++i) {
                std::ofstream out(std::filesystem::path(argv[1]) / (std::to_string(i) + ".bin"), std::ios::binary);
                out.write(reinterpret_cast<const char*>(packets[i].data()), static_cast<std::streamsize>(packets[i].size()));
                if (!out) throw std::runtime_error("fixture write failed");
            }
        }
        std::puts("state_stream endian encoding, fragments and description passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
