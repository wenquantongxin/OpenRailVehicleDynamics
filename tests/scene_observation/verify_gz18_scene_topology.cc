#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Dense>

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/scene_observation/scene_topology.h"
#include "orvd/scene_observation/wheel_spin_sampling.h"

namespace {

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: verify_gz18_scene_topology ORVD_DATA_ROOT\n");
        return 2;
    }
    try {
        using namespace orvd;
        const std::filesystem::path root = argv[1];
        const auto vehicle = configuration::LoadVehicleDefinitionFromJsonFile(
            root / "vehicle_library/gz18/vehicle_definition.json");
        const auto startup = configuration::LoadResolvedStartupStateFromJsonFile(
            root / "vehicle_library/gz18/startup_states/moving_startup_60kmh.json");
        auto line = configuration::LoadTrackGeometryFromJsonFile(
            root / "track_library/geometries/r300_centerline_superelevation_1100m.json");
        const auto scenario = configuration::AssembleGz18ContactScenario(
            vehicle, startup, std::move(line), root, 0.0, nullptr);
        const auto& assembled = scenario->vehicle_system();
        const auto* plan = assembled.contact_force_plan();
        const auto topology = scene_observation::DescribeSceneTopology(assembled.model(), plan);
        Require(topology.bodies.size() == static_cast<std::size_t>(assembled.model().num_rigid_bodies()) &&
                    topology.wheel_placements.size() == 8,
                "GZ18 scene must describe every body and eight wheel placements");
        for (std::size_t i = 0; i < topology.bodies.size(); ++i) {
            const auto body = assembled.model().GetRigidBody(static_cast<int>(i));
            Require(topology.bodies[i].name == assembled.model().GetRigidBodyName(body) &&
                        topology.bodies[i].moves_freely_in_world == assembled.model().IsFreeBody(body),
                    "scene body slots differ from the model");
        }
        std::map<std::string, std::set<wheel_rail_contact::WheelSide>> sides;
        for (int i = 0; i < plan->interface_count(); ++i) {
            const auto& placement = topology.wheel_placements[static_cast<std::size_t>(i)];
            const auto& interface = plan->interface_definition(i);
            Require(placement.interface_name == interface.interface_name &&
                        placement.wheel_body_name == interface.wheel_body_name &&
                        placement.carrier_body_name == placement.wheel_body_name &&
                        placement.side == interface.side && !placement.spin_joint_name.has_value(),
                    "GZ18 wheels must share their rigid wheelset, with no independent spin joint");
            const double lateral = placement.side == wheel_rail_contact::WheelSide::kRight ? 0.7465 : -0.7465;
            Require(placement.datum_in_wheel_body_frame_meters.isApprox(Eigen::Vector3d(0.0, lateral, 0.0), 1e-12) &&
                        placement.spin_axis_in_wheel_body_frame.isApprox(Eigen::Vector3d::UnitY(), 1e-12) &&
                        std::abs(placement.nominal_rolling_radius_meters - 0.42) < 1e-12,
                    "GZ18 wheel datum, axis or rolling radius is incorrect");
            Require(sides[placement.carrier_body_name].insert(placement.side).second,
                    "a wheelset repeats a side");
        }
        Require(sides.size() == 4, "GZ18 wheels must belong to four wheelsets");
        for (const auto& [body, pair] : sides) {
            Require(pair.size() == 2 && assembled.model().IsFreeBody(assembled.model().GetRigidBodyByName(body)),
                    "each free wheelset must own one wheel on each side");
        }
        const scene_observation::WheelSpinAngleSampler spins(assembled.model(), topology);
        Require(!spins.available(), "GZ18 scene must not invent independent wheel angles");
        std::puts("GZ18 scene topology: four rigid wheelsets, eight wheel placements");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
