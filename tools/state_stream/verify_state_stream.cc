#include "orvd/state_stream/state_stream.h"

#include <bit>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "orvd/multibody_model/multibody_evaluation_context.h"

int main(int argc, char** argv) {
    using namespace orvd;
    try {
        multibody_model::MultibodyModel model;
        multibody_runtime::RigidBodyInertiaParameters inertia;
        inertia.mass_kilograms = 1.0;
        inertia.unit_inertia_moments = Eigen::Vector3d(0.01, 0.02, 0.02);
        const auto body = model.AddRigidBody("test_body", inertia);
        model.DeclareFreeBody(body);
        model.Finalize();
        auto context = model.CreateDefaultContext();
        Eigen::VectorXd q = Eigen::VectorXd::Zero(model.num_generalized_positions());
        q[0] = std::cos(0.2);
        q[3] = std::sin(0.2);
        q.segment<3>(4) = Eigen::Vector3d(1.5, -2.25, 0.75);
        model.SetGeneralizedPositions(context.get(), q);
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model.num_generalized_velocities());
        const auto range = model.GetFreeBodyVelocityRange(body);
        v.segment<3>(range.start()) = Eigen::Vector3d(0.3, -0.7, 1.2);
        v.segment<3>(range.start() + 3) = Eigen::Vector3d(2.0, 3.0, -4.0);
        model.SetGeneralizedVelocities(context.get(), v);
        state_stream::BodyStateSampler sampler(model);
        const auto states = sampler.Sample(*context);
        if (sampler.names() != std::vector<std::string>{"test_body"} ||
            states.size() != 1 || states[0].position_meters != std::array<double, 3>{1.5, -2.25, 0.75} ||
            std::abs(states[0].orientation_wxyz[0] - std::cos(0.2)) > 1e-14 ||
            std::abs(states[0].orientation_wxyz[3] - std::sin(0.2)) > 1e-14)
            throw std::runtime_error("named sampling differs from the stated free-body pose");
        if (states[0].angular_velocity_radians_per_second != std::array<double, 3>{0.3, -0.7, 1.2} ||
            states[0].linear_velocity_meters_per_second != std::array<double, 3>{2.0, 3.0, -4.0})
            throw std::runtime_error("angular and origin-linear velocity slots differ");
        std::vector<state_stream::BodyState> many(25, states[0]);
        many[0].linear_velocity_meters_per_second[0] = -0.0;
        state_stream::ScalarValues scalars;
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
        std::puts("state_stream sampling, endian encoding and fragments passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
