// The GZ18 60 km/h resolved start-up state, placed on a line.
//
// argv[1] is the vehicle definition, argv[2] the start-up state, argv[3] a
// level tangent line the vehicle legally starts on.
//
// What is checked here is what only a real vehicle and a real line can show:
// that every named entity of the record reaches the coordinate the model says
// is its own, that the identity and the gravity a state was resolved under are
// compared against the system it is assembled into, that the start-up domain
// contract refuses the three families and a body off the end of the line, and
// that editing one named quantity moves exactly one thing.
//
// What is deliberately not here: any recomputation of a resolved quantity from
// the others, any accounting identity over the target wheel loads, and any time
// integration. A resolved start-up state is an input.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/configuration/assemble_resolved_initial_context.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/track_geometry/track_geometry.h"

namespace {

using orvd::configuration::AssembledVehicleSystem;
using orvd::configuration::AssembleResolvedInitialContext;
using orvd::configuration::AssembleVehicleSystem;
using orvd::configuration::FreeBodyStartupState;
using orvd::configuration::LoadResolvedStartupStateFromJsonFile;
using orvd::configuration::LoadTrackGeometryFromJsonFile;
using orvd::configuration::LoadVehicleDefinitionFromJsonFile;
using orvd::configuration::ResolvedInitialContext;
using orvd::configuration::ResolvedStartupState;
using orvd::configuration::VehicleDefinition;
using orvd::track_geometry::TrackGeometry;
using orvd::track_geometry::TrackScalarProfile;
using orvd::track_geometry::TrackScalarSegment;
using orvd::track_geometry::TrackScalarSegmentShape;

constexpr double kGravity = 9.81;
// Far enough into the level tangent line that all seven bodies stand on it.
constexpr double kLayoutReferenceStation = 20.0;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "GZ18 start-up assembly: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

// A refusal has to be the refusal that was asked for, not merely some
// exception: a message that happened to mention something else would let a
// wrong rejection pass for a right one.
void RequireRefusal(const std::function<void()>& action,
                    std::string_view fragment, std::string_view what) {
    try {
        action();
    } catch (const std::exception& error) {
        const std::string message = error.what();
        if (message.find(fragment) == std::string::npos) {
            std::fprintf(stderr,
                         "GZ18 start-up assembly: %.*s was refused, but for "
                         "another reason: %s\n",
                         static_cast<int>(what.size()), what.data(),
                         message.c_str());
            ++failures;
        }
        return;
    }
    std::fprintf(stderr, "GZ18 start-up assembly: %.*s was accepted\n",
                 static_cast<int>(what.size()), what.data());
    ++failures;
}

TrackScalarProfile LevelProfile(double length_meters) {
    return TrackScalarProfile(
        0.0,
        {TrackScalarSegment{length_meters, TrackScalarSegmentShape::kConstant,
                            0.0, 0.0}},
        {});
}

// Zero up to `support_start`, then a blend that leaves zero. The boundary value
// agrees on both sides, so no seam window is declared — a window would make the
// leading half of it a non-zero piece and move the reported support start half
// a window earlier, quietly turning "exactly at" into "before".
TrackScalarProfile ProfileBeginningAt(double support_start_meters,
                                      double total_length_meters,
                                      double end_value) {
    return TrackScalarProfile(
        0.0,
        {TrackScalarSegment{support_start_meters,
                            TrackScalarSegmentShape::kConstant, 0.0, 0.0},
         TrackScalarSegment{total_length_meters - support_start_meters,
                            TrackScalarSegmentShape::kHermiteCubicBlend, 0.0,
                            end_value}},
        {});
}

double StationOffsetOf(const VehicleDefinition& vehicle,
                       std::string_view body_name) {
    for (const auto& offset :
         vehicle.mechanical_track_station_layout.free_body_station_offsets) {
        if (offset.body_name == body_name) {
            return offset.station_offset_meters;
        }
    }
    Require(false, "the fixture vehicle has no station offset for a body the "
                   "test names");
    return 0.0;
}

FreeBodyStartupState& MutableBodyState(ResolvedStartupState& state,
                                       std::string_view body_name) {
    for (FreeBodyStartupState& body : state.free_body_startup_states) {
        if (body.body_name == body_name) {
            return body;
        }
    }
    Require(false, "the fixture start-up state has no entry for a body the "
                   "test names");
    return state.free_body_startup_states.front();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(stderr,
                     "usage: verify_gz18_startup_assembly <vehicle.json> "
                     "<startup_state.json> <level_line.json>\n");
        return 2;
    }
    const std::filesystem::path vehicle_path = argv[1];
    const std::filesystem::path startup_path = argv[2];
    const std::filesystem::path line_path = argv[3];

    try {
        const VehicleDefinition vehicle =
            LoadVehicleDefinitionFromJsonFile(vehicle_path);
        const ResolvedStartupState startup =
            LoadResolvedStartupStateFromJsonFile(startup_path);
        const TrackGeometry line = LoadTrackGeometryFromJsonFile(line_path);

        Require(startup.vehicle_binding.mechanical_definition_identifier ==
                    vehicle.mechanical_definition_identifier,
                "the shipped start-up state and vehicle record disagree on the "
                "mechanical definition identifier");
        Require(startup.gravitational_acceleration_meters_per_second_squared ==
                    kGravity,
                "the shipped start-up state was not resolved under 9.81 m/s^2");

        const std::unique_ptr<AssembledVehicleSystem> system =
            AssembleVehicleSystem(vehicle, kGravity);

        // --- The legal placement -------------------------------------------
        const ResolvedInitialContext resolved = AssembleResolvedInitialContext(
            *system, startup, line, kLayoutReferenceStation);

        const auto& model = system->model();
        const auto& instance = system->system();
        const int position_count = model.num_generalized_positions();
        const Eigen::VectorXd& positions =
            resolved.context().generalized_positions();
        const Eigen::VectorXd& velocities =
            resolved.context().generalized_velocities();
        const Eigen::VectorXd& series_forces =
            resolved.context().series_spring_damper_forces();

        Require(positions.size() == 57 && velocities.size() == 50 &&
                    series_forces.size() == 2,
                "the assembled context does not have GZ18's coordinate counts");

        // Each named body reached the coordinates the model says are its own.
        // The expected values are formed here from the record and the line,
        // not read back through the same mapping that wrote them.
        for (const FreeBodyStartupState& body :
             startup.free_body_startup_states) {
            const double station =
                kLayoutReferenceStation +
                StationOffsetOf(vehicle, body.body_name) +
                body.resolved_track_station_offset_from_mechanical_layout_meters;
            const auto pose = line.EvaluateTrackFrame(station).pose();
            const Eigen::Quaterniond expected_rotation =
                Eigen::Quaterniond(pose.rotation_inertial_from_track()) *
                body.rotation_local_track_from_body;
            const Eigen::Vector3d expected_origin =
                pose.origin_in_inertial_meters() +
                pose.rotation_inertial_from_track() *
                    Eigen::Vector3d(
                        0.0, body.lateral_offset_in_local_track_frame_meters,
                        body.vertical_offset_in_local_track_frame_meters);
            const Eigen::Vector3d expected_angular_velocity =
                expected_rotation *
                body.
                    body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second;
            const Eigen::Vector3d expected_origin_velocity =
                pose.rotation_inertial_from_track() *
                body.
                    body_origin_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second;

            const auto handle = model.GetRigidBodyByName(body.body_name);
            const auto position_range = model.GetFreeBodyPositionRange(handle);
            const auto velocity_range = model.GetFreeBodyVelocityRange(handle);
            const Eigen::Quaterniond written(
                positions[position_range.start() + 0],
                positions[position_range.start() + 1],
                positions[position_range.start() + 2],
                positions[position_range.start() + 3]);
            Require(written.angularDistance(expected_rotation) < 1.0e-14,
                    "a free body's attitude is not the one the record states "
                    "for it at its own station");
            Require((positions.segment<3>(position_range.start() + 4) -
                     expected_origin)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-12,
                    "a free body's origin is not where the record places it");
            Require((velocities.segment<3>(velocity_range.start()) -
                     expected_angular_velocity)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-12,
                    "a free body's angular velocity was not carried across "
                    "bases as an inertial quantity");
            Require((velocities.segment<3>(velocity_range.start() + 3) -
                     expected_origin_velocity)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-12,
                    "a free body's origin velocity was not carried across "
                    "bases as an inertial quantity");
        }

        for (const auto& joint : startup.revolute_joint_startup_states) {
            const auto handle = model.GetJointByName(joint.joint_name);
            Require(positions[model.GetJointPositionRange(handle).start()] ==
                        joint.position_radians,
                    "a joint coordinate is not the one the record states");
            Require(velocities[model.GetJointVelocityRange(handle).start()] ==
                        joint.rate_radians_per_second,
                    "a joint rate is not the one the record states");
        }

        for (const auto& element :
             startup.series_spring_viscous_damper_force_states) {
            const auto index =
                instance.GetSeriesSpringViscousDamperIndexByName(
                    element.element_name);
            const auto range =
                instance.series_spring_damper_force_state_range(index);
            Eigen::VectorXd whole(instance.continuous_state_size());
            instance.CopyContinuousState(resolved.context(), whole);
            Require(whole[range.start()] ==
                        element.reference_end_axial_force_newtons,
                    "a series force state is not the one the record states");
        }

        // --- The sign gate --------------------------------------------------
        //
        // A wheelset spins about -y and each of its two axlebox carriers turns
        // the other way at the same rate, so in the inertial frame the carriers
        // do not turn at all. Getting either sign wrong leaves them spinning at
        // tens of radians a second. This is the check that catches it, and it
        // needs no rolling radius: it never mentions the speed.
        {
            const auto view = instance.GetMultibodyComponentView(
                resolved.context(), instance.multibody_component());
            double worst_carrier_spin = 0.0;
            int carriers = 0;
            for (const auto& body : vehicle.rigid_bodies) {
                if (body.name.find("axlebox_carrier") == std::string::npos) {
                    continue;
                }
                ++carriers;
                const auto handle = model.GetRigidBodyByName(body.name);
                const Eigen::Vector3d angular_velocity =
                    model
                        .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                            view.context(), handle)
                        .angular_velocity_radians_per_second();
                worst_carrier_spin = std::max(
                    worst_carrier_spin, angular_velocity.cwiseAbs().maxCoeff());
            }
            Require(carriers == 8,
                    "the sign gate did not find GZ18's eight axlebox carriers");
            Require(worst_carrier_spin < 1.0e-12,
                    "the axlebox carriers turn in the inertial frame at "
                    "start-up, so a wheelset spin and its pivot rate do not "
                    "cancel");
        }

        // --- The finite right-hand side ------------------------------------
        {
            Eigen::VectorXd derivatives(instance.continuous_state_size());
            system->compiled_plan().CalcStateTimeDerivatives(resolved.context(),
                                                             derivatives);
            Require(derivatives.allFinite(),
                    "the assembled start-up state does not produce a finite "
                    "right-hand side");
        }

        // --- Placements the assembly derived --------------------------------
        Require(resolved.wheelset_placements().size() == 4,
                "the resolved result does not carry four wheelset placements");
        double foremost = 0.0;
        for (const auto& placement : resolved.wheelset_placements()) {
            foremost = std::max(foremost, placement.track_station_meters);
            Require(placement.left_support_force_newtons == 68295.99375 &&
                        placement.right_support_force_newtons == 68295.99375,
                    "a target wheel support force did not survive assembly");
        }
        Require(foremost ==
                    kLayoutReferenceStation +
                        StationOffsetOf(vehicle, "front_leading_wheelset"),
                "the derived foremost axle station is not the layout "
                "reference plus that body's mechanical offset");
        Require(resolved.rail_profile_reference_vertical_offset_meters() ==
                    startup.rail_profile_reference_vertical_offset_meters,
                "the rail profile reference vertical offset did not survive "
                "assembly unchanged");

        // --- Identity and gravity -------------------------------------------
        {
            ResolvedStartupState edited = startup;
            edited.vehicle_binding.mechanical_definition_identifier =
                "gz18_reference_mechanics_v2";
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, edited, line, kLayoutReferenceStation);
                },
                "mechanical definition",
                "a state resolved for another mechanical definition");
        }
        {
            ResolvedStartupState edited = startup;
            edited.vehicle_binding.vehicle_name = "SH17";
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, edited, line, kLayoutReferenceStation);
                },
                "vehicle", "a state resolved for another vehicle");
        }
        {
            // The preloads, the target loads and the rail offset all follow
            // from the gravity the state was resolved under, so a system built
            // under another one is not the system it describes.
            const std::unique_ptr<AssembledVehicleSystem> lunar_system =
                AssembleVehicleSystem(vehicle, 1.62);
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *lunar_system, startup, line, kLayoutReferenceStation);
                },
                "gravity", "a state resolved under another gravity");
        }

        // --- All and only ---------------------------------------------------
        {
            ResolvedStartupState edited = startup;
            edited.translational_spring_damper_nominal_forces.pop_back();
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, edited, line, kLayoutReferenceStation);
                },
                "states no nominal force",
                "a record that omits one element's preload");
        }
        {
            ResolvedStartupState edited = startup;
            edited.free_body_startup_states.pop_back();
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, edited, line, kLayoutReferenceStation);
                },
                "states no free-body start-up state",
                "a record that omits one free body");
        }
        {
            // The other direction of all-and-only: every real joint is still
            // stated, and one more besides.
            ResolvedStartupState edited = startup;
            edited.revolute_joint_startup_states.push_back(
                orvd::configuration::RevoluteJointStartupState{"no_such_pivot",
                                                               0.0, 0.0});
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, edited, line, kLayoutReferenceStation);
                },
                "no_such_pivot", "a record that names a joint this vehicle "
                                 "does not have");
        }

        // --- The start-up domain --------------------------------------------
        const double foremost_axle_station =
            kLayoutReferenceStation +
            StationOffsetOf(vehicle, "front_leading_wheelset");
        const double counterexample_length = foremost_axle_station + 200.0;
        {
            // Exactly at the foremost axle: the inequality is strict.
            const TrackGeometry curved(
                ProfileBeginningAt(foremost_axle_station,
                                   counterexample_length, 0.002),
                LevelProfile(counterexample_length),
                LevelProfile(counterexample_length), 1.5, 0.5);
            Require(curved.first_curved_track_station_meters().has_value() &&
                        *curved.first_curved_track_station_meters() ==
                            foremost_axle_station,
                    "the curvature counterexample does not begin exactly at "
                    "the foremost axle station");
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, startup, curved, kLayoutReferenceStation);
                },
                "not strictly after the foremost axle station",
                "a line whose curvature begins exactly at the foremost axle");
        }
        {
            const TrackGeometry canted(
                LevelProfile(counterexample_length),
                ProfileBeginningAt(foremost_axle_station - 5.0,
                                   counterexample_length, 0.05),
                LevelProfile(counterexample_length), 1.5, 0.5);
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, startup, canted, kLayoutReferenceStation);
                },
                "superelevation support begins",
                "a line whose superelevation begins before the foremost axle");
        }
        {
            const TrackGeometry graded(
                LevelProfile(counterexample_length),
                LevelProfile(counterexample_length),
                ProfileBeginningAt(foremost_axle_station - 5.0,
                                   counterexample_length, 0.01),
                1.5, 0.5);
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, startup, graded, kLayoutReferenceStation);
                },
                "grade support begins",
                "a line whose grade begins before the foremost axle");
        }
        {
            // Placed so that a body which is not a wheelset falls off the near
            // end first. A domain check that only walked the wheelsets would
            // name a different body, so the diagnostic itself is the evidence
            // that all seven are covered.
            const double crowded_reference =
                -StationOffsetOf(vehicle, "rear_bogie_frame") - 1.0;
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(*system, startup, line,
                                                         crowded_reference);
                },
                "rear_bogie_frame",
                "a placement that puts a non-wheelset free body off the end "
                "of the line");
        }

        // --- One edit moves one thing ---------------------------------------
        {
            ResolvedStartupState edited = startup;
            MutableBodyState(edited, "rear_trailing_wheelset")
                .body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second
                .y() = -30.0;
            const ResolvedInitialContext other =
                AssembleResolvedInitialContext(*system, edited, line,
                                               kLayoutReferenceStation);
            Eigen::VectorXd before(instance.continuous_state_size());
            Eigen::VectorXd after(instance.continuous_state_size());
            instance.CopyContinuousState(resolved.context(), before);
            instance.CopyContinuousState(other.context(), after);
            const auto handle =
                model.GetRigidBodyByName("rear_trailing_wheelset");
            const int changed_index =
                position_count +
                model.GetFreeBodyVelocityRange(handle).start() + 1;
            int changed = 0;
            for (int index = 0; index < before.size(); ++index) {
                if (before[index] != after[index]) {
                    ++changed;
                    Require(index == changed_index,
                            "editing one wheelset's angular velocity moved a "
                            "coordinate that is not its own");
                }
            }
            Require(changed == 1,
                    "editing one wheelset's angular velocity did not move "
                    "exactly one coordinate");
        }
        {
            ResolvedStartupState edited = startup;
            edited.series_spring_viscous_damper_force_states.front()
                .reference_end_axial_force_newtons = 1234.5;
            const ResolvedInitialContext other =
                AssembleResolvedInitialContext(*system, edited, line,
                                               kLayoutReferenceStation);
            Eigen::VectorXd before(instance.continuous_state_size());
            Eigen::VectorXd after(instance.continuous_state_size());
            instance.CopyContinuousState(resolved.context(), before);
            instance.CopyContinuousState(other.context(), after);
            const auto index = instance.GetSeriesSpringViscousDamperIndexByName(
                edited.series_spring_viscous_damper_force_states.front()
                    .element_name);
            const int changed_index =
                instance.series_spring_damper_force_state_range(index).start();
            int changed = 0;
            for (int i = 0; i < before.size(); ++i) {
                if (before[i] != after[i]) {
                    ++changed;
                    Require(i == changed_index,
                            "editing one Maxwell force state moved a "
                            "coordinate that is not its own");
                }
            }
            Require(changed == 1,
                    "editing one Maxwell force state did not move exactly one "
                    "coordinate");
        }
        {
            ResolvedStartupState edited = startup;
            edited.translational_spring_damper_nominal_forces.front()
                .force_on_reference_end_in_reference_frame_newtons =
                Eigen::Vector3d(0.0, 0.0, -55000.0);
            const ResolvedInitialContext other =
                AssembleResolvedInitialContext(*system, edited, line,
                                               kLayoutReferenceStation);
            Eigen::VectorXd before(instance.continuous_state_size());
            Eigen::VectorXd after(instance.continuous_state_size());
            instance.CopyContinuousState(resolved.context(), before);
            instance.CopyContinuousState(other.context(), after);
            Require((before - after).cwiseAbs().maxCoeff() == 0.0,
                    "editing one preload moved the continuous state");
            const Eigen::VectorXd& nominal_before =
                resolved.context().nominal_forces();
            const Eigen::VectorXd& nominal_after =
                other.context().nominal_forces();
            int changed = 0;
            for (int i = 0; i < nominal_before.size(); ++i) {
                if (nominal_before[i] != nominal_after[i]) {
                    ++changed;
                }
            }
            Require(changed == 1,
                    "editing one preload did not move exactly one nominal "
                    "force component");
        }

        // --- The change of basis, on a fixture that can tell ------------------
        //
        // The shipped state is level and tangent and every attitude in it is
        // the identity, so on it the two basis changes below are both the
        // identity too and would pass however they were written. Giving one
        // body a real attitude and an angular velocity that is not along its
        // rotation axis makes the body-to-inertial change discriminating: the
        // expected value is formed here from the quaternion and the stated
        // components, not read back through the mapping that wrote it.
        //
        // The other factor, the track-frame rotation, cannot be made
        // discriminating within this Goal at all. The start-up domain contract
        // requires curvature, superelevation and grade all to begin strictly
        // after the foremost axle, and every body stands at or before it, so
        // the track frame's rotation is the identity at every body's station in
        // every admissible placement. It is written out in full because the
        // formula is the general one, not because a test here could tell.
        {
            ResolvedStartupState edited = startup;
            const Eigen::Quaterniond attitude =
                Eigen::Quaterniond(
                    Eigen::AngleAxisd(0.37, Eigen::Vector3d(0.3, -0.5, 0.81)
                                                .normalized()))
                    .normalized();
            const Eigen::Vector3d body_rate(0.11, -39.2, 0.07);
            FreeBodyStartupState& tilted =
                MutableBodyState(edited, "front_leading_wheelset");
            tilted.rotation_local_track_from_body = attitude;
            tilted
                .body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second =
                body_rate;
            const ResolvedInitialContext other =
                AssembleResolvedInitialContext(*system, edited, line,
                                               kLayoutReferenceStation);
            const auto handle =
                model.GetRigidBodyByName("front_leading_wheelset");
            const auto velocity_range = model.GetFreeBodyVelocityRange(handle);
            const Eigen::Vector3d written =
                other.context().generalized_velocities().segment<3>(
                    velocity_range.start());
            const Eigen::Vector3d expected = attitude * body_rate;
            Require((written - expected).cwiseAbs().maxCoeff() < 1.0e-14,
                    "a body-frame angular velocity was not turned into the "
                    "inertial frame by the body's own attitude");
            Require((written - body_rate).cwiseAbs().maxCoeff() > 1.0e-3,
                    "this fixture cannot tell a change of basis from leaving "
                    "the components alone");

            const auto position_range = model.GetFreeBodyPositionRange(handle);
            const Eigen::Quaterniond written_attitude(
                other.context().generalized_positions()[position_range.start()],
                other.context()
                    .generalized_positions()[position_range.start() + 1],
                other.context()
                    .generalized_positions()[position_range.start() + 2],
                other.context()
                    .generalized_positions()[position_range.start() + 3]);
            Require(written_attitude.angularDistance(attitude) < 1.0e-14,
                    "a stated attitude did not survive assembly on a level "
                    "tangent line, where the track frame contributes nothing");
        }

        // --- The third term of the station sum --------------------------------
        //
        // Every resolved station offset in the shipped state is zero, so on it
        // that term could be dropped without any number changing. A non-zero
        // one moves the body, moves the station the assembly reports for it,
        // and moves nothing else.
        {
            ResolvedStartupState edited = startup;
            MutableBodyState(edited, "front_leading_wheelset")
                .resolved_track_station_offset_from_mechanical_layout_meters =
                0.25;
            const ResolvedInitialContext other =
                AssembleResolvedInitialContext(*system, edited, line,
                                               kLayoutReferenceStation);
            const auto handle =
                model.GetRigidBodyByName("front_leading_wheelset");
            const auto range = model.GetFreeBodyPositionRange(handle);
            const double before =
                resolved.context().generalized_positions()[range.start() + 4];
            const double after =
                other.context().generalized_positions()[range.start() + 4];
            Require(std::abs((after - before) - 0.25) < 1.0e-12,
                    "a resolved station offset did not move the body it "
                    "belongs to along the line");
            for (const auto& placement : other.wheelset_placements()) {
                if (placement.wheelset_body_name == "front_leading_wheelset") {
                    Require(placement.track_station_meters ==
                                kLayoutReferenceStation +
                                    StationOffsetOf(vehicle,
                                                    "front_leading_wheelset") +
                                    0.25,
                            "the station the assembly reports does not include "
                            "the resolved offset");
                }
            }
        }

        // --- Equality is not an error ----------------------------------------
        //
        // A resolved spin that happens to equal the speed over the nominal
        // rolling radius is a legal record, not a suspicious one. Nothing here
        // recomputes the spin, so nothing here may reject it for agreeing.
        {
            ResolvedStartupState edited = startup;
            const double nominal_radius_quotient =
                edited.initial_longitudinal_speed_meters_per_second / 0.42;
            for (FreeBodyStartupState& body : edited.free_body_startup_states) {
                if (body.body_name.find("wheelset") != std::string::npos) {
                    body
                        .body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second
                        .y() = -nominal_radius_quotient;
                }
            }
            const ResolvedInitialContext other =
                AssembleResolvedInitialContext(*system, edited, line,
                                               kLayoutReferenceStation);
            const auto handle =
                model.GetRigidBodyByName("front_leading_wheelset");
            Eigen::VectorXd whole(instance.continuous_state_size());
            instance.CopyContinuousState(other.context(), whole);
            Require(whole[position_count +
                          model.GetFreeBodyVelocityRange(handle).start() + 1] ==
                        -nominal_radius_quotient,
                    "a wheelset spin that equals the speed over the nominal "
                    "rolling radius was not carried through unchanged");
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "GZ18 start-up assembly failed: %s\n",
                     error.what());
        return 1;
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("GZ18 resolved start-up assembly verified");
    return 0;
}
