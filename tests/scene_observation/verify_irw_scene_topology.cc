#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/scene_observation/scene_topology.h"
#include "orvd/scene_observation/wheel_spin_sampling.h"

// The bundled IRW scenario described as a scene: eight wheel placements whose
// datums, derived from the contact constants, land on the axle at the signed
// lateral datum of each side when expressed in the track frame of the resolved
// start-up state. The IRW bodies keep the source basis (y left, z up), so the
// right wheel's datum is at body -y; this test is what stops a hand-written
// sign from placing both wheels on one side.

namespace {

using namespace orvd;

[[noreturn]] void Fail(const std::string& detail) {
    throw std::runtime_error(detail);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: verify_irw_scene_topology ORVD_DATA_ROOT\n");
        return 2;
    }
    try {
        const std::filesystem::path root = argv[1];
        const auto vehicle = configuration::LoadVehicleDefinitionFromJsonFile(
            root / "vehicle_library" / "irw" / "vehicle_definition.json");
        const auto startup = configuration::LoadResolvedStartupStateFromJsonFile(
            root / "vehicle_library" / "irw" / "startup_states" /
            "moving_startup_60kmh.json");
        auto line = configuration::LoadTrackGeometryFromJsonFile(
            root / "track_library" / "geometries" /
            "r300_centerline_superelevation_1100m.json");
        const auto scenario = configuration::AssembleIrwContactScenario(
            vehicle, startup, std::move(line), root, 0.0, nullptr);
        const auto& assembled = scenario->vehicle_system();
        const auto* plan = assembled.contact_force_plan();
        const auto topology = scene_observation::DescribeSceneTopology(
            assembled.model(), plan);

        if (topology.bodies.size() !=
                static_cast<std::size_t>(assembled.model().num_rigid_bodies()) ||
            topology.wheel_placements.size() != 8) {
            Fail("the IRW topology does not list every body and eight wheels");
        }
        auto& context = scenario->initial_context().context();
        const auto component = assembled.system().GetMultibodyComponentView(
            context, assembled.system().multibody_component());
        const scene_observation::WheelSpinAngleSampler spin_sampler(
            assembled.model(), topology);
        if (!spin_sampler.available() || spin_sampler.wheel_count() != 8) {
            Fail("the IRW spin sampler did not bind all eight wheel joints");
        }
        std::vector<double> spin_angles(8);
        spin_sampler.Sample(component.context(), spin_angles);
        for (int index = 0; index < plan->interface_count(); ++index) {
            const auto& placement =
                topology.wheel_placements[static_cast<std::size_t>(index)];
            const auto& definition = plan->interface_definition(index);
            if (placement.interface_name != definition.interface_name ||
                placement.wheel_body_name != definition.wheel_body_name ||
                placement.side != definition.side) {
                Fail("wheel placement identity differs from the plan");
            }
            const double signed_datum =
                placement.side == wheel_rail_contact::WheelSide::kRight
                    ? 0.7465
                    : -0.7465;
            // Source basis: body -y is physically to the right.
            if (!placement.datum_in_wheel_body_frame_meters.isApprox(
                    Eigen::Vector3d(0.0, -signed_datum, 0.0), 1e-12) ||
                !placement.spin_axis_in_wheel_body_frame.isApprox(
                    Eigen::Vector3d(0.0, -1.0, 0.0), 1e-12) ||
                std::abs(placement.nominal_rolling_radius_meters - 0.43) >
                    1e-12) {
                Fail("IRW wheel '" + placement.wheel_body_name +
                     "' datum, axis or radius differs from the contact "
                     "constants in the source body basis");
            }
            // In the world, at the resolved start, the datum must sit at the
            // signed lateral datum of its side in the carrier's track frame.
            const auto wheel_body =
                assembled.model().GetRigidBodyByName(placement.wheel_body_name);
            const auto carrier_body =
                assembled.model().GetRigidBodyByName(definition.carrier_name);
            const auto wheel_pose =
                assembled.model().CalcPoseInWorld(component.context(), wheel_body);
            const auto carrier_pose = assembled.model().CalcPoseInWorld(
                component.context(), carrier_body);
            if ((wheel_pose.translation() - carrier_pose.translation()).norm() >
                1e-12) {
                Fail("IRW wheel origin does not coincide with its axle bridge "
                     "origin at the resolved start");
            }
            // The wheel's orientation relative to its carrier is the spin
            // rotation about the placement's axis by the sampled angle.
            const Eigen::Matrix3d relative_rotation =
                carrier_pose.rotation().transpose() * wheel_pose.rotation();
            const Eigen::Matrix3d spin_rotation =
                Eigen::AngleAxisd(spin_angles[static_cast<std::size_t>(index)],
                                  placement.spin_axis_in_wheel_body_frame)
                    .toRotationMatrix();
            if ((relative_rotation - spin_rotation).cwiseAbs().maxCoeff() >
                1e-9) {
                Fail("IRW wheel '" + placement.wheel_body_name +
                     "': the sampled spin angle about the placement axis does "
                     "not reproduce the wheel orientation relative to its "
                     "carrier");
            }
            const Eigen::Vector3d datum_in_world =
                wheel_pose.translation() +
                wheel_pose.rotation() * placement.datum_in_wheel_body_frame_meters;
            const int carrier_index = [&] {
                for (int carrier = 0; carrier < plan->carrier_count(); ++carrier) {
                    if (plan->carrier_definition(carrier).carrier_name ==
                        definition.carrier_name) {
                        return carrier;
                    }
                }
                Fail("interface names an unknown carrier");
            }();
            const auto track = plan->track_geometry().EvaluateTrackFrame(
                plan->initial_projection_station_meters(carrier_index));
            const Eigen::Vector3d offset_in_track =
                track.pose().rotation_inertial_from_track().transpose() *
                (datum_in_world - carrier_pose.translation());
            if (std::abs(offset_in_track.x()) > 1e-8 ||
                std::abs(offset_in_track.y() - signed_datum) > 1e-8 ||
                std::abs(offset_in_track.z()) > 1e-8) {
                Fail("IRW wheel '" + placement.wheel_body_name +
                     "' datum is not at the signed lateral datum in the track "
                     "frame at the resolved start");
            }
        }
        std::puts("IRW scene topology: eight wheel datums land on the axle at "
                  "the signed lateral datum of their side");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
