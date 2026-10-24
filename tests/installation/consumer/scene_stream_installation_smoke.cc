#include "orvd/scene_stream/scene_stream.h"

#include <stdexcept>

int main() {
    using namespace orvd::scene_stream;
    orvd::scene_observation::BodyState body;
    body.position_meters = {1.0, 2.0, 3.0};
    body.orientation_wxyz = {1.0, 0.0, 0.0, 0.0};
    const auto payload = EncodeState(std::span(&body, 1), {{7.5}, {1}});
    const auto packets = Packetize(kStateKind, 9007199254741009ULL,
                                   9007199254741021ULL, 0.5, payload);
    const auto header = ParseHeader(packets.front());
    const auto decoded = DecodeState(std::span(packets.front()).subspan(kHeaderBytes));
    if (header.run_id != 9007199254741009ULL || header.sequence != 9007199254741021ULL ||
        decoded.bodies.size() != 1 || decoded.bodies[0].position_meters != body.position_meters ||
        decoded.scalars.values != std::vector{7.5} || decoded.scalars.statuses != std::vector<std::uint8_t>{1})
        throw std::runtime_error("installed scene stream round trip failed");
    return 0;
}
