#include <cstdio>
#include <exception>
#include <filesystem>
#include <random>

#include <Eigen/Dense>

#include "orvd/multibody_model/multibody_model.h"
#include "orvd/scene_observation/body_state.h"
#include "orvd/scene_observation/scene_frame.h"
#include "orvd/scene_observation/scene_topology.h"
#include "orvd/scene_record/scene_record_reader.h"
#include "orvd/scene_record/scene_record_writer.h"

// The installed scene libraries exercised end to end: a one-body model is
// sampled, described, written as a scene record into a fresh temporary
// directory and read back through the installed reader.

namespace {

std::filesystem::path FreshTemporaryDirectory() {
    std::random_device device;
    const auto base = std::filesystem::temp_directory_path();
    for (int attempt = 0; attempt < 16; ++attempt) {
        const std::filesystem::path candidate =
            base / ("orvd-scene-record-smoke-" + std::to_string(device()));
        if (!std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    throw std::runtime_error("could not choose a fresh temporary directory");
}

}  // namespace

int main() {
    try {
        orvd::multibody_model::MultibodyModel model;
        orvd::multibody_runtime::RigidBodyInertiaParameters inertia;
        inertia.mass_kilograms = 2.0;
        inertia.unit_inertia_moments = Eigen::Vector3d(0.1, 0.1, 0.1);
        const auto body = model.AddRigidBody("smoke_body", inertia);
        model.DeclareFreeBody(body);
        model.Finalize();
        auto context = model.CreateDefaultContext();
        Eigen::VectorXd positions =
            Eigen::VectorXd::Zero(model.num_generalized_positions());
        positions[0] = 1.0;
        positions.segment<3>(4) = Eigen::Vector3d(3.0, -1.0, 0.25);
        model.SetGeneralizedPositions(context.get(), positions);

        const orvd::scene_observation::BodyStateSampler sampler(model);
        orvd::scene_observation::SceneFrame frame;
        frame.identity.time_seconds = 0.0;
        frame.identity.sample_index = 0;
        frame.bodies = sampler.Sample(*context);

        const std::filesystem::path directory = FreshTemporaryDirectory();
        {
            orvd::scene_record::SceneRecordWriter writer(
                directory,
                orvd::scene_observation::DescribeSceneTopology(model, nullptr),
                {}, "", false);
            writer.WriteFrame(frame);
            writer.Close();
        }
        const auto record = orvd::scene_record::ReadSceneRecord(directory);
        std::filesystem::remove_all(directory);
        if (record.frames.size() != 1 || record.topology.bodies.size() != 1 ||
            record.topology.bodies[0].name != "smoke_body" ||
            record.frames[0].bodies[0].position_meters !=
                std::array<double, 3>{3.0, -1.0, 0.25}) {
            std::fprintf(stderr,
                         "installed scene record smoke did not read back the "
                         "sampled body\n");
            return 1;
        }
        std::puts("installed scene observation and scene record smoke passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed scene record smoke failed: %s\n",
                     error.what());
        return 1;
    }
}
