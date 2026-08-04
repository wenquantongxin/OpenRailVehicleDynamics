// The GZ18 resolved moving start-up, placed on both its qualified demonstration
// line and an explicitly unqualified research line.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
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

constexpr double kLayoutReferenceStation = 20.0;
int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "GZ18 start-up assembly: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireRefusal(const std::function<void()>& action,
                    std::string_view fragment, std::string_view what) {
    try {
        action();
    } catch (const std::exception& error) {
        if (std::string(error.what()).find(fragment) == std::string::npos) {
            std::fprintf(stderr,
                         "GZ18 start-up assembly: %.*s was refused for "
                         "another reason: %s\n",
                         static_cast<int>(what.size()), what.data(),
                         error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

TrackScalarProfile ConstantProfile(double length_meters, double value) {
    return TrackScalarProfile(
        0.0,
        {TrackScalarSegment{length_meters, TrackScalarSegmentShape::kConstant,
                            value, value}},
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
    throw std::logic_error("test fixture has no station offset for " +
                           std::string(body_name));
}

FreeBodyStartupState& MutableBodyState(ResolvedStartupState& state,
                                       std::string_view body_name) {
    for (FreeBodyStartupState& body : state.free_body_startup_states) {
        if (body.body_name == body_name) {
            return body;
        }
    }
    throw std::logic_error("test fixture has no body state for " +
                           std::string(body_name));
}

Eigen::Vector3d GeneratedSpinInBody(const ResolvedStartupState& state,
                                    std::string_view body_name) {
    const double magnitude =
        state.initial_longitudinal_speed_meters_per_second /
        state.common_startup_effective_rolling_radius_meters;
    for (const auto& wheelset : state.wheelset_startup_kinematics) {
        if (wheelset.wheelset_body_name == body_name) {
            return magnitude * wheelset.forward_spin_axis_in_body_frame;
        }
    }
    return Eigen::Vector3d::Zero();
}

Eigen::VectorXd CopyState(const AssembledVehicleSystem& system,
                          const ResolvedInitialContext& resolved) {
    Eigen::VectorXd state(system.system().continuous_state_size());
    system.system().CopyContinuousState(resolved.context(), state);
    return state;
}

void CheckStateMatchesRecord(const VehicleDefinition& vehicle,
                             const ResolvedStartupState& startup,
                             const TrackGeometry& line,
                             const AssembledVehicleSystem& system,
                             const ResolvedInitialContext& resolved) {
    const auto& model = system.model();
    const Eigen::VectorXd& positions =
        resolved.context().generalized_positions();
    const Eigen::VectorXd& velocities =
        resolved.context().generalized_velocities();
    for (const FreeBodyStartupState& body :
         startup.free_body_startup_states) {
        const double station =
            kLayoutReferenceStation + StationOffsetOf(vehicle, body.body_name) +
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
        const Eigen::Vector3d angular_velocity_in_body =
            body.additional_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second +
            GeneratedSpinInBody(startup, body.body_name);
        const Eigen::Vector3d expected_angular_velocity =
            expected_rotation * angular_velocity_in_body;
        const Eigen::Vector3d expected_origin_velocity =
            pose.rotation_inertial_from_track() *
            Eigen::Vector3d(
                startup.initial_longitudinal_speed_meters_per_second,
                body.body_origin_lateral_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second,
                body.body_origin_vertical_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second);

        const auto handle = model.GetRigidBodyByName(body.body_name);
        const auto q = model.GetFreeBodyPositionRange(handle);
        const auto v = model.GetFreeBodyVelocityRange(handle);
        const Eigen::Quaterniond written(positions[q.start() + 0],
                                         positions[q.start() + 1],
                                         positions[q.start() + 2],
                                         positions[q.start() + 3]);
        Require(written.angularDistance(expected_rotation) < 1.0e-13,
                "a body rotation does not equal R_IT times R_TB");
        Require((positions.segment<3>(q.start() + 4) - expected_origin)
                        .cwiseAbs()
                        .maxCoeff() < 1.0e-12,
                "a body origin does not equal p_IT plus R_IT times its offset");
        Require((velocities.segment<3>(v.start()) - expected_angular_velocity)
                        .cwiseAbs()
                        .maxCoeff() < 1.0e-12,
                "a body angular velocity does not match the expanded record");
        Require((velocities.segment<3>(v.start() + 3) -
                 expected_origin_velocity)
                        .cwiseAbs()
                        .maxCoeff() < 1.0e-12,
                "a body origin velocity does not match V0 and its stated "
                "lateral and vertical components");
    }

    const double spin_magnitude =
        startup.initial_longitudinal_speed_meters_per_second /
        startup.common_startup_effective_rolling_radius_meters;
    for (const auto& joint : startup.revolute_joint_startup_states) {
        const auto handle = model.GetJointByName(joint.joint_name);
        Require(positions[model.GetJointPositionRange(handle).start()] ==
                    joint.position_radians,
                "a joint position was not mapped by name");
        Require(velocities[model.GetJointVelocityRange(handle).start()] ==
                    joint.rate_per_common_wheel_spin_magnitude *
                        spin_magnitude,
                "a joint rate was not generated from its named factor");
    }

    const auto& instance = system.system();
    const Eigen::VectorXd complete_state = CopyState(system, resolved);
    for (const auto& element :
         startup.series_spring_viscous_damper_force_states) {
        const auto index =
            instance.GetSeriesSpringViscousDamperIndexByName(
                element.element_name);
        const auto range =
            instance.series_spring_damper_force_state_range(index);
        Require(range.size() == 1 &&
                    complete_state[range.start()] ==
                        element.reference_end_axial_force_newtons,
                "a series force state was not mapped by element name");
    }

    const auto expected_parameters =
        instance.CreateDefaultRuntimeContext(0.0);
    for (const auto& element :
         startup.translational_spring_damper_nominal_forces) {
        instance.SetNominalForce(
            *expected_parameters,
            instance.GetTranslationalSpringDamperIndexByName(
                element.element_name),
            element.force_on_reference_end_in_reference_frame_newtons);
    }
    Require(expected_parameters->nominal_forces() ==
                resolved.context().nominal_forces(),
            "nominal forces do not equal the values resolved by element name");
}

int CountBodiesWithChangedAngularVelocity(
    const VehicleDefinition& vehicle, const AssembledVehicleSystem& system,
    const ResolvedInitialContext& left, const ResolvedInitialContext& right) {
    int changed = 0;
    for (const auto& offset :
         vehicle.mechanical_track_station_layout.free_body_station_offsets) {
        const auto handle = system.model().GetRigidBodyByName(offset.body_name);
        const auto range = system.model().GetFreeBodyVelocityRange(handle);
        if (left.context().generalized_velocities()
                .segment<3>(range.start()) !=
            right.context().generalized_velocities()
                .segment<3>(range.start())) {
            ++changed;
        }
    }
    return changed;
}

int CountBodiesWithChangedOriginVelocity(
    const VehicleDefinition& vehicle, const AssembledVehicleSystem& system,
    const ResolvedInitialContext& left, const ResolvedInitialContext& right) {
    int changed = 0;
    for (const auto& offset :
         vehicle.mechanical_track_station_layout.free_body_station_offsets) {
        const auto handle = system.model().GetRigidBodyByName(offset.body_name);
        const auto range = system.model().GetFreeBodyVelocityRange(handle);
        if (left.context().generalized_velocities()
                .segment<3>(range.start() + 3) !=
            right.context().generalized_velocities()
                .segment<3>(range.start() + 3)) {
            ++changed;
        }
    }
    return changed;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(stderr,
                     "usage: verify_gz18_startup_assembly <vehicle.json> "
                     "<startup.json> <level-line.json>\n");
        return 2;
    }

    try {
        const VehicleDefinition vehicle =
            LoadVehicleDefinitionFromJsonFile(argv[1]);
        const ResolvedStartupState startup =
            LoadResolvedStartupStateFromJsonFile(argv[2]);
        const TrackGeometry line = LoadTrackGeometryFromJsonFile(argv[3]);
        Require(!line.first_curved_track_station_meters().has_value() &&
                    !line.first_superelevated_track_station_meters()
                         .has_value() &&
                    !line.first_graded_track_station_meters().has_value(),
                "the bundled GZ18 demonstration line is not straight, level "
                "and zero-superelevation");

        const std::unique_ptr<AssembledVehicleSystem> system =
            AssembleVehicleSystem(
                vehicle,
                startup.gravitational_acceleration_meters_per_second_squared);
        const ResolvedInitialContext baseline = AssembleResolvedInitialContext(
            *system, startup, line, kLayoutReferenceStation);
        CheckStateMatchesRecord(vehicle, startup, line, *system, baseline);

        Eigen::VectorXd derivatives(system->system().continuous_state_size());
        system->compiled_plan().CalcStateTimeDerivatives(baseline.context(),
                                                         derivatives);
        Require(derivatives.allFinite(),
                "the resolved state does not produce a finite right-hand side");

        // Wheelset spin and the axlebox-carrier pivot have opposite physical
        // directions, leaving all eight carriers inertially still.
        {
            const auto view = system->system().GetMultibodyComponentView(
                baseline.context(), system->system().multibody_component());
            int carriers = 0;
            double worst_spin = 0.0;
            for (const auto& body : vehicle.rigid_bodies) {
                if (body.name.find("axlebox_carrier") == std::string::npos) {
                    continue;
                }
                ++carriers;
                const auto velocity =
                    system->model()
                        .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                            view.context(),
                            system->model().GetRigidBodyByName(body.name));
                worst_spin = std::max(
                    worst_spin,
                    velocity.angular_velocity_radians_per_second()
                        .cwiseAbs()
                        .maxCoeff());
            }
            Require(carriers == 8 && worst_spin < 1.0e-12,
                    "wheelset spin and axlebox-carrier pivot rates do not "
                    "cancel in the inertial frame");
        }

        // Metadata remains paired by wheelset name; the expected values come
        // from the input record, not a duplicate numerical list in this test.
        for (const auto& placement : baseline.wheelset_placements()) {
            const auto expected = std::find_if(
                startup.target_wheel_support_forces.begin(),
                startup.target_wheel_support_forces.end(),
                [&](const auto& value) {
                    return value.wheelset_body_name ==
                           placement.wheelset_body_name;
                });
            Require(expected != startup.target_wheel_support_forces.end() &&
                        placement.left_support_force_newtons ==
                            expected->left_support_force_newtons &&
                        placement.right_support_force_newtons ==
                            expected->right_support_force_newtons,
                    "target support forces did not remain paired by name");
        }
        Require(baseline.wheel_rail_binding().wheel_profile_identifier ==
                    startup.wheel_rail_binding.wheel_profile_identifier &&
                    baseline.wheel_rail_binding().rail_profile_identifier ==
                        startup.wheel_rail_binding.rail_profile_identifier &&
                    baseline.wheel_rail_binding()
                            .wheel_rail_contact_strategy_identifier ==
                        startup.wheel_rail_binding
                            .wheel_rail_contact_strategy_identifier &&
                    baseline.rail_profile_reference_vertical_offset_meters() ==
                        startup.rail_profile_reference_vertical_offset_meters &&
                    baseline.load_condition_identifier() ==
                        startup.load_condition_identifier,
                "resolved identity metadata did not survive assembly");

        // Distinct values and reversed declaration order prove that support
        // loads remain attached to the named wheelset rather than an ordinal.
        {
            ResolvedStartupState edited = startup;
            for (std::size_t i = 0;
                 i < edited.target_wheel_support_forces.size(); ++i) {
                edited.target_wheel_support_forces[i]
                    .left_support_force_newtons =
                    1000.0 + static_cast<double>(i);
                edited.target_wheel_support_forces[i]
                    .right_support_force_newtons =
                    2000.0 + static_cast<double>(i);
                MutableBodyState(
                    edited,
                    edited.target_wheel_support_forces[i].wheelset_body_name)
                    .resolved_track_station_offset_from_mechanical_layout_meters =
                    0.01 * static_cast<double>(i + 1);
            }
            std::reverse(edited.target_wheel_support_forces.begin(),
                         edited.target_wheel_support_forces.end());
            const ResolvedInitialContext other = AssembleResolvedInitialContext(
                *system, edited, line, kLayoutReferenceStation);
            for (const auto& placement : other.wheelset_placements()) {
                const auto expected = std::find_if(
                    edited.target_wheel_support_forces.begin(),
                    edited.target_wheel_support_forces.end(),
                    [&](const auto& value) {
                        return value.wheelset_body_name ==
                               placement.wheelset_body_name;
                    });
                Require(expected !=
                                edited.target_wheel_support_forces.end() &&
                            placement.left_support_force_newtons ==
                                expected->left_support_force_newtons &&
                            placement.right_support_force_newtons ==
                                expected->right_support_force_newtons &&
                            placement.track_station_meters ==
                                kLayoutReferenceStation +
                                    StationOffsetOf(
                                        vehicle,
                                        placement.wheelset_body_name) +
                                    MutableBodyState(
                                        edited,
                                        placement.wheelset_body_name)
                                        .resolved_track_station_offset_from_mechanical_layout_meters,
                        "target loads were mapped by declaration order");
            }
        }

        // V0 is active: changing it changes seven longitudinal velocities,
        // four wheelset spins and eight pivot rates together.
        {
            ResolvedStartupState edited = startup;
            edited.initial_longitudinal_speed_meters_per_second *= 0.73;
            const ResolvedInitialContext other = AssembleResolvedInitialContext(
                *system, edited, line, kLayoutReferenceStation);
            CheckStateMatchesRecord(vehicle, edited, line, *system, other);
            Require(CountBodiesWithChangedOriginVelocity(vehicle, *system,
                                                         baseline, other) ==
                        static_cast<int>(
                            startup.free_body_startup_states.size()),
                    "changing V0 did not reach every GZ18 free body");
            Require(CountBodiesWithChangedAngularVelocity(vehicle, *system,
                                                          baseline, other) ==
                        static_cast<int>(
                            startup.wheelset_startup_kinematics.size()),
                    "changing V0 did not reach exactly the wheelsets' spins");
        }

        // The common effective radius only changes angular rates. It is a
        // consumed input, not a descriptive magic number.
        {
            ResolvedStartupState edited = startup;
            edited.common_startup_effective_rolling_radius_meters *= 1.37;
            const ResolvedInitialContext other = AssembleResolvedInitialContext(
                *system, edited, line, kLayoutReferenceStation);
            CheckStateMatchesRecord(vehicle, edited, line, *system, other);
            Require(CountBodiesWithChangedOriginVelocity(vehicle, *system,
                                                         baseline, other) == 0,
                    "changing effective radius changed a body origin velocity");
            Require(CountBodiesWithChangedAngularVelocity(vehicle, *system,
                                                          baseline, other) ==
                        static_cast<int>(
                            startup.wheelset_startup_kinematics.size()),
                    "changing effective radius did not reach exactly the "
                    "wheelsets' spins");
        }

        // Name, not declaration order, selects every generated angular rate.
        {
            ResolvedStartupState edited = startup;
            const std::vector<Eigen::Vector3d> axes{
                Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(),
                Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitY()};
            for (std::size_t i = 0;
                 i < edited.wheelset_startup_kinematics.size(); ++i) {
                edited.wheelset_startup_kinematics[i]
                    .forward_spin_axis_in_body_frame = axes[i];
            }
            for (std::size_t i = 0;
                 i < edited.revolute_joint_startup_states.size(); ++i) {
                edited.revolute_joint_startup_states[i]
                    .rate_per_common_wheel_spin_magnitude =
                    0.25 + 0.125 * static_cast<double>(i);
            }
            for (std::size_t i = 0;
                 i < edited.series_spring_viscous_damper_force_states.size();
                 ++i) {
                edited.series_spring_viscous_damper_force_states[i]
                    .reference_end_axial_force_newtons =
                    51.0 + static_cast<double>(i);
            }
            const ResolvedInitialContext first = AssembleResolvedInitialContext(
                *system, edited, line, kLayoutReferenceStation);
            CheckStateMatchesRecord(vehicle, edited, line, *system, first);
            std::reverse(edited.wheelset_startup_kinematics.begin(),
                         edited.wheelset_startup_kinematics.end());
            std::reverse(edited.revolute_joint_startup_states.begin(),
                         edited.revolute_joint_startup_states.end());
            std::reverse(
                edited.series_spring_viscous_damper_force_states.begin(),
                edited.series_spring_viscous_damper_force_states.end());
            const ResolvedInitialContext reversed =
                AssembleResolvedInitialContext(*system, edited, line,
                                               kLayoutReferenceStation);
            CheckStateMatchesRecord(vehicle, edited, line, *system, reversed);
            Require(CopyState(*system, first) == CopyState(*system, reversed),
                    "reordering named wheelset, joint or series records "
                    "changed the assembled state");
        }

        // Context-local nominal forces are likewise resolved by element name
        // once; their declaration order is not a hidden slot map.
        {
            ResolvedStartupState edited = startup;
            for (std::size_t i = 0;
                 i < edited.translational_spring_damper_nominal_forces.size();
                 ++i) {
                const double marker = static_cast<double>(i + 1);
                edited.translational_spring_damper_nominal_forces[i]
                    .force_on_reference_end_in_reference_frame_newtons =
                    Eigen::Vector3d(marker, -2.0 * marker, 3.0 * marker);
            }
            const ResolvedInitialContext first = AssembleResolvedInitialContext(
                *system, edited, line, kLayoutReferenceStation);
            CheckStateMatchesRecord(vehicle, edited, line, *system, first);
            std::reverse(
                edited.translational_spring_damper_nominal_forces.begin(),
                edited.translational_spring_damper_nominal_forces.end());
            const ResolvedInitialContext reversed =
                AssembleResolvedInitialContext(*system, edited, line,
                                               kLayoutReferenceStation);
            CheckStateMatchesRecord(vehicle, edited, line, *system, reversed);
            Require(first.context().nominal_forces() ==
                        reversed.context().nominal_forces(),
                    "reordering named nominal forces changed their slots");
        }

        // A curved, graded and superelevated research start is accepted and
        // uses the full non-commuting basis transformation. It is intentionally
        // not claimed as a SIMPACK-qualified demo.
        {
            const double length = 200.0;
            const TrackGeometry research_line(
                ConstantProfile(length, 0.002),
                ConstantProfile(length, 0.02),
                ConstantProfile(length, 0.01), 1.5, 0.5);
            ResolvedStartupState edited = startup;
            FreeBodyStartupState& carbody = MutableBodyState(edited, "carbody");
            carbody.resolved_track_station_offset_from_mechanical_layout_meters =
                20.0;
            carbody.rotation_local_track_from_body =
                Eigen::AngleAxisd(0.31, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(-0.23, Eigen::Vector3d::UnitX());
            carbody.lateral_offset_in_local_track_frame_meters = 0.3;
            carbody.vertical_offset_in_local_track_frame_meters = -0.2;
            carbody.additional_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second =
                Eigen::Vector3d(0.1, -0.2, 0.3);
            carbody.body_origin_lateral_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second =
                0.4;
            carbody.body_origin_vertical_velocity_in_inertial_expressed_in_local_track_frame_meters_per_second =
                -0.5;
            const double carbody_station =
                kLayoutReferenceStation + StationOffsetOf(vehicle, "carbody") +
                carbody
                    .resolved_track_station_offset_from_mechanical_layout_meters;
            double foremost_wheelset_station =
                research_line.start_track_station_meters();
            for (const std::string& wheelset_name :
                 system->binding().wheelset_body_names) {
                foremost_wheelset_station = std::max(
                    foremost_wheelset_station,
                    kLayoutReferenceStation +
                        StationOffsetOf(vehicle, wheelset_name) +
                        MutableBodyState(edited, wheelset_name)
                            .resolved_track_station_offset_from_mechanical_layout_meters);
            }
            Require(carbody_station > foremost_wheelset_station,
                    "the research-line fixture does not place a non-wheel "
                    "body beyond the foremost wheelset");
            const ResolvedInitialContext research =
                AssembleResolvedInitialContext(*system, edited, research_line,
                                               kLayoutReferenceStation);
            CheckStateMatchesRecord(vehicle, edited, research_line, *system,
                                    research);

            const Eigen::Quaterniond rotation_inertial_from_track(
                research_line.EvaluateTrackFrame(carbody_station)
                    .pose()
                    .rotation_inertial_from_track());
            const Eigen::Quaterniond expected =
                rotation_inertial_from_track *
                carbody.rotation_local_track_from_body;
            const Eigen::Quaterniond reversed =
                carbody.rotation_local_track_from_body *
                rotation_inertial_from_track;
            Require(expected.angularDistance(
                        carbody.rotation_local_track_from_body) > 1.0e-3 &&
                        expected.angularDistance(reversed) > 1.0e-3,
                    "the research-line fixture cannot distinguish omitted or "
                    "reversed track/body rotation composition");
        }

        // Public C++ records receive the same layout validation as JSON.
        {
            VehicleDefinition edited = vehicle;
            edited.mechanical_track_station_layout.free_body_station_offsets
                .pop_back();
            RequireRefusal(
                [&] {
                    (void)AssembleVehicleSystem(
                        edited,
                        startup
                            .gravitational_acceleration_meters_per_second_squared);
                },
                "omits the free body",
                "a C++ VehicleDefinition with an incomplete station layout");
        }

        {
            ResolvedStartupState edited = startup;
            edited.wheelset_startup_kinematics.pop_back();
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, edited, line, kLayoutReferenceStation);
                },
                "states no wheelset start-up kinematics",
                "a record omitting one wheelset kinematic mapping");
        }
        {
            ResolvedStartupState edited = startup;
            edited.vehicle_binding.mechanical_definition_identifier =
                "different_mechanics";
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, edited, line, kLayoutReferenceStation);
                },
                "mechanical definition",
                "a state bound to another mechanical definition");
        }
        {
            ResolvedStartupState edited = startup;
            edited.wheel_rail_binding
                .wheel_rail_contact_strategy_identifier = "not/a/name";
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, edited, line, kLayoutReferenceStation);
                },
                "outside [A-Za-z0-9._-]",
                "a C++ state with a path-like contact identifier");
        }
        {
            const std::unique_ptr<AssembledVehicleSystem> other_gravity =
                AssembleVehicleSystem(vehicle, 1.62);
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *other_gravity, startup, line,
                        kLayoutReferenceStation);
                },
                "gravity", "a state resolved under another gravity");
        }
        {
            ResolvedStartupState edited = startup;
            MutableBodyState(edited, "carbody")
                .resolved_track_station_offset_from_mechanical_layout_meters =
                line.end_track_station_meters();
            RequireRefusal(
                [&] {
                    (void)AssembleResolvedInitialContext(
                        *system, edited, line, kLayoutReferenceStation);
                },
                "outside this line's domain",
                "a non-wheel free body outside the line");
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
