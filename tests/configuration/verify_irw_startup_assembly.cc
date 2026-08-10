// The frozen IRW H3 moving state, assembled by names into q81/v74/z2.

#include <algorithm>
#include <array>
#include <cstdio>
#include <functional>
#include <map>
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

namespace {

using orvd::configuration::AssembledVehicleSystem;
using orvd::configuration::AssembleResolvedInitialContext;
using orvd::configuration::AssembleVehicleSystem;
using orvd::configuration::BallRpyJointStartupState;
using orvd::configuration::ExplicitRevoluteJointRate;
using orvd::configuration::FreeBodyStartupState;
using orvd::configuration::LoadResolvedStartupStateFromJsonFile;
using orvd::configuration::LoadTrackGeometryFromJsonFile;
using orvd::configuration::LoadVehicleDefinitionFromJsonFile;
using orvd::configuration::ResolvedInitialContext;
using orvd::configuration::ResolvedStartupState;
using orvd::configuration::VehicleDefinition;
using orvd::track_geometry::TrackGeometry;

constexpr double kLayoutReferenceStationMeters = 0.0;
constexpr double kPrimaryNominalForceNewtons = 21578.11967531337;
constexpr double kAirSpringNominalForceNewtons = -61312.5;
int failure_count = 0;

void Require(bool condition, std::string_view description) {
    if (!condition) {
        std::fprintf(stderr, "IRW start-up assembly: %.*s\n",
                     static_cast<int>(description.size()), description.data());
        ++failure_count;
    }
}

void RequireRefusal(const std::function<void()>& action,
                    std::string_view fragment,
                    std::string_view description) {
    try {
        action();
    } catch (const std::exception& error) {
        if (std::string(error.what()).find(fragment) == std::string::npos) {
            std::fprintf(stderr,
                         "IRW start-up assembly: %.*s was refused for "
                         "another reason: %s\n",
                         static_cast<int>(description.size()),
                         description.data(), error.what());
            ++failure_count;
        }
        return;
    }
    Require(false, description);
}

struct WheelState {
    std::string_view name;
    std::string_view axle_body_name;
    std::string_view wheel_body_name;
    double position_radians;
    double rate_radians_per_second;
};

constexpr std::array<WheelState, 8> kWheelStates{{
    {"rev_wheel_ff_l", "axlebridge_ff", "wheel_ff_l",
     0.31533397347187259, 38.782287098808986},
    {"rev_wheel_ff_r", "axlebridge_ff", "wheel_ff_r",
     -0.06317638965070542, 38.782287098809249},
    {"rev_wheel_fr_l", "axlebridge_fr", "wheel_fr_l",
     -0.025102324383414906, 38.78228709880905},
    {"rev_wheel_fr_r", "axlebridge_fr", "wheel_fr_r",
     -0.33709418242954087, 38.782287098809327},
    {"rev_wheel_rf_l", "axlebridge_rf", "wheel_rf_l",
     -0.002613479380619458, 38.782287098809427},
    {"rev_wheel_rf_r", "axlebridge_rf", "wheel_rf_r",
     0.0055770558701520806, 38.782287098809775},
    {"rev_wheel_rr_l", "axlebridge_rr", "wheel_rr_l",
     0.00053419343930150476, 38.782287098809213},
    {"rev_wheel_rr_r", "axlebridge_rr", "wheel_rr_r",
     0.00021224274270999753, 38.782287098809505},
}};

struct SupportState {
    std::string_view body_name;
    double station_meters;
    double left_newtons;
    double right_newtons;

    bool operator==(const SupportState&) const = default;
};

constexpr std::array<SupportState, 4> kSupportStates{{
    {"axlebridge_ff", 10.000008398803899, 49287.489352631594,
     49287.48935999266},
    {"axlebridge_fr", 7.5000084046506323, 49287.489354519064,
     49287.48936219448},
    {"axlebridge_rf", -7.4999916011961005, 49287.48936481213,
     49287.48937441125},
    {"axlebridge_rr", -9.9999915953493677, 49287.48935887162,
     49287.48936707778},
}};

double MechanicalStationOffset(const VehicleDefinition& vehicle,
                               std::string_view body_name) {
    for (const auto& value :
         vehicle.mechanical_track_station_layout.free_body_station_offsets) {
        if (value.body_name == body_name) {
            return value.station_offset_meters;
        }
    }
    throw std::logic_error("IRW fixture lacks a station offset for " +
                           std::string(body_name));
}

const BallRpyJointStartupState& BallState(const ResolvedStartupState& state,
                                          std::string_view name) {
    for (const BallRpyJointStartupState& value :
         state.ball_rpy_joint_startup_states) {
        if (value.joint_name == name) {
            return value;
        }
    }
    throw std::logic_error("IRW fixture lacks Ball-RPY state " +
                           std::string(name));
}

BallRpyJointStartupState& MutableBallState(ResolvedStartupState& state,
                                          std::string_view name) {
    for (BallRpyJointStartupState& value :
         state.ball_rpy_joint_startup_states) {
        if (value.joint_name == name) {
            return value;
        }
    }
    throw std::logic_error("IRW fixture lacks mutable Ball-RPY state " +
                           std::string(name));
}

Eigen::VectorXd CopyContinuousState(const AssembledVehicleSystem& system,
                                    const ResolvedInitialContext& resolved) {
    Eigen::VectorXd result(system.system().continuous_state_size());
    system.system().CopyContinuousState(resolved.context(), result);
    return result;
}

Eigen::Vector3d ExpectedNominalForce(std::string_view name) {
    if (name.find("_ps_spring_") != std::string_view::npos) {
        return {0.0, 0.0, kPrimaryNominalForceNewtons};
    }
    if (name.find("_ss_airspring_") != std::string_view::npos) {
        return {0.0, 0.0, kAirSpringNominalForceNewtons};
    }
    return Eigen::Vector3d::Zero();
}

std::map<std::string, SupportState> SupportMap(
    const ResolvedInitialContext& resolved) {
    std::map<std::string, SupportState> result;
    for (const auto& value : resolved.wheel_pair_placements()) {
        result.emplace(
            value.station_reference_body_name,
            SupportState{value.station_reference_body_name,
                         value.track_station_meters,
                         value.left_support_force_newtons,
                         value.right_support_force_newtons});
    }
    return result;
}

void CheckRecordIdentity(const ResolvedStartupState& state) {
    Require(state.vehicle_binding.vehicle_name == "IRW" &&
                state.vehicle_binding.mechanical_definition_identifier ==
                    "irw_reference_mechanical_definition",
            "the H3 record is bound to the frozen IRW mechanics");
    Require(!state.common_wheel_spin_generation.has_value(),
            "IRW must not carry a rigid-wheelset common-spin authority");
    Require(state.initial_longitudinal_speed_meters_per_second ==
                16.666666666666668,
            "the H3 station speed is the frozen 60 km/h value");
    Require(state.free_body_startup_states.size() == 7 &&
                state.revolute_joint_startup_states.size() == 8 &&
                state.ball_rpy_joint_startup_states.size() == 8 &&
                state.series_spring_viscous_damper_force_states.size() == 2 &&
                state.translational_spring_damper_nominal_forces.size() == 36,
            "the record has the H3 seven/eight/eight/two/thirty-six families");

    // These two non-degenerate source values distinguish the C-basis
    // transformation from copying SIMPACK XYZ angles into ORVD RPY slots.
    const auto& front_a = BallState(state, "ball_longibar_front_A");
    Require(front_a.roll_pitch_yaw_angles_radians ==
                Eigen::Vector3d(8.3827533763236595e-05,
                                6.0560601352627634e-06,
                                1.7097133997328204) &&
                front_a
                    .angular_velocity_of_child_in_parent_expressed_in_parent_frame_radians_per_second ==
                    Eigen::Vector3d::Zero(),
            "front-A Ball-RPY carries the transformed H3 coordinate and zero "
            "physical angular velocity");
    const auto& rear_d = BallState(state, "ball_longibar_rear_D");
    Require(rear_d.roll_pitch_yaw_angles_radians ==
                Eigen::Vector3d(8.3827533763791706e-05,
                                6.0559934127151918e-06,
                                -1.4318792538569758),
            "rear-D Ball-RPY keeps its distinct transformed H3 coordinate");

    for (const auto& body : state.free_body_startup_states) {
        Require(body.common_wheel_spin_coefficient_in_body_frame ==
                        Eigen::Vector3d::Zero() &&
                    body
                            .explicit_body_angular_velocity_in_inertial_expressed_in_body_frame_radians_per_second ==
                        Eigen::Vector3d::Zero(),
                "every H3 free body states its own zero initial angular "
                "velocity without generated wheel spin");
    }
}

void CheckWrittenState(const VehicleDefinition& vehicle,
                       const ResolvedStartupState& startup,
                       const TrackGeometry& line,
                       const AssembledVehicleSystem& system,
                       const ResolvedInitialContext& resolved) {
    const auto& model = system.model();
    const Eigen::VectorXd& q = resolved.context().generalized_positions();
    const Eigen::VectorXd& v = resolved.context().generalized_velocities();
    Require(q.size() == 81 && v.size() == 74 &&
                system.system().continuous_state_size() == 157,
            "the assembled H3 context is q81/v74/z2");

    for (const FreeBodyStartupState& body : startup.free_body_startup_states) {
        const double station =
            kLayoutReferenceStationMeters +
            MechanicalStationOffset(vehicle, body.body_name) +
            body.resolved_track_station_offset_from_mechanical_layout_meters;
        const auto track_pose = line.EvaluateTrackFrame(station).pose();
        const Eigen::Quaterniond expected_rotation =
            Eigen::Quaterniond(track_pose.rotation_inertial_from_track()) *
            body.rotation_local_track_from_body;
        const Eigen::Vector3d expected_origin =
            track_pose.origin_in_inertial_meters() +
            track_pose.rotation_inertial_from_track() *
                Eigen::Vector3d(
                    0.0, body.lateral_offset_in_local_track_frame_meters,
                    body.vertical_offset_in_local_track_frame_meters);
        const auto handle = model.GetRigidBodyByName(body.body_name);
        const auto q_range = model.GetFreeBodyPositionRange(handle);
        const auto v_range = model.GetFreeBodyVelocityRange(handle);
        const Eigen::Quaterniond actual_rotation(
            q[q_range.start()], q[q_range.start() + 1],
            q[q_range.start() + 2], q[q_range.start() + 3]);
        Require(actual_rotation.angularDistance(expected_rotation) < 1.0e-14 &&
                    (q.segment<3>(q_range.start() + 4) - expected_origin)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-13,
                "free body '" + body.body_name +
                    "' was placed from its named Track-T H3 record");
        Require(v.segment<3>(v_range.start()) == Eigen::Vector3d::Zero() &&
                    v.segment<3>(v_range.start() + 3) ==
                        track_pose.rotation_inertial_from_track() *
                            Eigen::Vector3d(
                                startup
                                    .initial_longitudinal_speed_meters_per_second,
                                0.0, 0.0),
                "free body '" + body.body_name +
                    "' keeps zero body spin and the common longitudinal "
                    "origin velocity");
    }

    for (const WheelState& expected : kWheelStates) {
        const auto handle = model.GetJointByName(std::string(expected.name));
        const auto q_range = model.GetJointPositionRange(handle);
        const auto v_range = model.GetJointVelocityRange(handle);
        Require(q[q_range.start()] == expected.position_radians &&
                    v[v_range.start()] == expected.rate_radians_per_second,
                "wheel joint '" + std::string(expected.name) +
                    "' keeps its independent H3 phase and axle-relative "
                    "speed");
    }

    // These eight rates are wheel-to-axle joint velocities, not copied
    // absolute wheel speeds. Compose each one through the real multibody
    // topology and check that its wheel body, rather than its axle bridge,
    // carries the corresponding inertial spin.
    const auto component = system.system().GetMultibodyComponentView(
        resolved.context(), system.system().multibody_component());
    for (const WheelState& expected : kWheelStates) {
        const auto axle = model.GetRigidBodyByName(
            std::string(expected.axle_body_name));
        const auto wheel = model.GetRigidBodyByName(
            std::string(expected.wheel_body_name));
        const Eigen::Vector3d axle_omega =
            model
                .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                    component.context(), axle)
                .angular_velocity_radians_per_second();
        const Eigen::Vector3d wheel_omega =
            model
                .CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
                    component.context(), wheel)
                .angular_velocity_radians_per_second();
        const Eigen::Vector3d joint_axis_in_world =
            model.CalcPoseInWorld(component.context(), axle).rotation() *
            Eigen::Vector3d::UnitY();
        Require(axle_omega.cwiseAbs().maxCoeff() < 1.0e-14 &&
                    (wheel_omega -
                     joint_axis_in_world * expected.rate_radians_per_second)
                            .cwiseAbs()
                            .maxCoeff() < 1.0e-12,
                "wheel body '" + std::string(expected.wheel_body_name) +
                    "' does not compose its named axle-relative rate while "
                    "the axle bridge remains independently unspun");
    }
    for (const BallRpyJointStartupState& expected :
         startup.ball_rpy_joint_startup_states) {
        const auto handle = model.GetJointByName(expected.joint_name);
        const auto q_range = model.GetJointPositionRange(handle);
        const auto v_range = model.GetJointVelocityRange(handle);
        Require(q.segment<3>(q_range.start()) ==
                        expected.roll_pitch_yaw_angles_radians &&
                    v.segment<3>(v_range.start()) ==
                        expected
                            .angular_velocity_of_child_in_parent_expressed_in_parent_frame_radians_per_second,
                "Ball-RPY joint '" + expected.joint_name +
                    "' was written by name as q and physical omega");
    }

    const Eigen::VectorXd whole_state = CopyContinuousState(system, resolved);
    for (const auto& expected :
         startup.series_spring_viscous_damper_force_states) {
        const auto index =
            system.system().GetSeriesSpringViscousDamperIndexByName(
                expected.element_name);
        const auto range =
            system.system().series_spring_damper_force_state_range(index);
        Require(range.size() == 1 &&
                    whole_state[range.start()] ==
                        expected.reference_end_axial_force_newtons,
                "Maxwell state '" + expected.element_name +
                    "' was written by name");
    }

    auto expected_parameters = system.system().CreateDefaultRuntimeContext(0.0);
    int primary_count = 0;
    int air_count = 0;
    int zero_count = 0;
    for (const auto& element :
         startup.translational_spring_damper_nominal_forces) {
        const Eigen::Vector3d expected = ExpectedNominalForce(element.element_name);
        Require(element.force_on_reference_end_in_reference_frame_newtons ==
                    expected,
                "nominal force '" + element.element_name +
                    "' has the frozen H3 value and sign");
        primary_count += expected.z() == kPrimaryNominalForceNewtons ? 1 : 0;
        air_count += expected.z() == kAirSpringNominalForceNewtons ? 1 : 0;
        zero_count += expected.isZero() ? 1 : 0;
        system.system().SetNominalForce(
            *expected_parameters,
            system.system().GetTranslationalSpringDamperIndexByName(
                element.element_name),
            expected);
    }
    Require(primary_count == 16 && air_count == 4 && zero_count == 16,
            "the 36 nominal slots divide into 16 primary, four air and 16 "
            "zero entries");
    Require(expected_parameters->nominal_forces() ==
                resolved.context().nominal_forces(),
            "all 36 nominal forces were installed by element name");
}

void CheckSupportTargets(const ResolvedInitialContext& resolved) {
    const auto actual = SupportMap(resolved);
    Require(actual.size() == kSupportStates.size(),
            "the four axle station references remain paired with eight "
            "target wheel loads");
    for (const SupportState& expected : kSupportStates) {
        const auto found = actual.find(std::string(expected.body_name));
        Require(found != actual.end() &&
                    found->second.station_meters == expected.station_meters &&
                    found->second.left_newtons == expected.left_newtons &&
                    found->second.right_newtons == expected.right_newtons,
                "support target for '" + std::string(expected.body_name) +
                    "' keeps its source station and distinct left/right "
                    "loads");
    }
}

void ReverseEveryNamedFamily(ResolvedStartupState* state) {
    std::reverse(state->wheel_pair_target_support_forces.begin(),
                 state->wheel_pair_target_support_forces.end());
    std::reverse(state->free_body_startup_states.begin(),
                 state->free_body_startup_states.end());
    std::reverse(state->revolute_joint_startup_states.begin(),
                 state->revolute_joint_startup_states.end());
    std::reverse(state->ball_rpy_joint_startup_states.begin(),
                 state->ball_rpy_joint_startup_states.end());
    std::reverse(state->series_spring_viscous_damper_force_states.begin(),
                 state->series_spring_viscous_damper_force_states.end());
    std::reverse(state->translational_spring_damper_nominal_forces.begin(),
                 state->translational_spring_damper_nominal_forces.end());
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(stderr,
                     "usage: verify_irw_startup_assembly <vehicle.json> "
                     "<startup.json> <r300-line.json>\n");
        return 2;
    }

    try {
        const VehicleDefinition vehicle =
            LoadVehicleDefinitionFromJsonFile(argv[1]);
        const ResolvedStartupState startup =
            LoadResolvedStartupStateFromJsonFile(argv[2]);
        const TrackGeometry line = LoadTrackGeometryFromJsonFile(argv[3]);
        CheckRecordIdentity(startup);

        const std::unique_ptr<AssembledVehicleSystem> system =
            AssembleVehicleSystem(
                vehicle,
                startup.gravitational_acceleration_meters_per_second_squared);
        const ResolvedInitialContext baseline = AssembleResolvedInitialContext(
            *system, startup, line, kLayoutReferenceStationMeters);
        CheckWrittenState(vehicle, startup, line, *system, baseline);
        CheckSupportTargets(baseline);

        Eigen::VectorXd derivatives(system->system().continuous_state_size());
        system->compiled_plan().CalcStateTimeDerivatives(baseline.context(),
                                                         derivatives);
        Require(derivatives.allFinite(),
                "the real passive IRW H3 state produces a finite RHS before "
                "wheel-rail contact is connected");

        // Every family is deliberately reversed together. The model's q/v/z
        // layout does not change, so an ordinal-driven writer would now fail.
        ResolvedStartupState reordered = startup;
        ReverseEveryNamedFamily(&reordered);
        const ResolvedInitialContext reordered_context =
            AssembleResolvedInitialContext(
                *system, reordered, line, kLayoutReferenceStationMeters);
        Require(CopyContinuousState(*system, baseline) ==
                    CopyContinuousState(*system, reordered_context) &&
                    baseline.context().nominal_forces() ==
                        reordered_context.context().nominal_forces() &&
                    SupportMap(baseline) == SupportMap(reordered_context),
                "reordering all six named families changed the assembled "
                "H3 state, parameters or support targets");

        // The frozen H3 Ball velocities happen to be zero. Exercise the same
        // product writer once with a physical nonzero parent-frame omega so
        // the v side of the new three-component family is non-degenerate.
        ResolvedStartupState moving_ball = startup;
        const Eigen::Vector3d stated_omega(0.125, -0.25, 0.375);
        MutableBallState(moving_ball, "ball_longibar_front_A")
            .angular_velocity_of_child_in_parent_expressed_in_parent_frame_radians_per_second =
            stated_omega;
        const ResolvedInitialContext moving_ball_context =
            AssembleResolvedInitialContext(
                *system, moving_ball, line, kLayoutReferenceStationMeters);
        const auto ball =
            system->model().GetJointByName("ball_longibar_front_A");
        const auto velocity_range =
            system->model().GetJointVelocityRange(ball);
        Require(moving_ball_context.context()
                        .generalized_velocities()
                        .segment<3>(velocity_range.start()) == stated_omega &&
                    baseline.context()
                            .generalized_velocities()
                            .segment<3>(velocity_range.start()) ==
                        Eigen::Vector3d::Zero(),
                "a nonzero Ball physical omega was not written to its named "
                "three-component velocity range");

        ResolvedStartupState missing_ball = startup;
        missing_ball.ball_rpy_joint_startup_states.pop_back();
        RequireRefusal(
            [&] {
                (void)AssembleResolvedInitialContext(
                    *system, missing_ball, line,
                    kLayoutReferenceStationMeters);
            },
            "states no Ball-RPY-joint start-up state",
            "a missing Ball-RPY q/v family member was accepted");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW start-up assembly failed: %s\n",
                     error.what());
        return 1;
    }

    if (failure_count != 0) {
        return 1;
    }
    std::puts("IRW H3 resolved start-up assembly verified");
    return 0;
}
