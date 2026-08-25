#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/irw_full_state_control_event_session.h"
#include "orvd/configuration/irw_full_state_control_observation_binding.h"
#include "orvd/configuration/load_irw_full_state_wheel_speed_guidance_controller.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_track_irregularity_field.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/load_wheel_drive_torque_command_conditioner.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/wheel_rail_contact/roll_yaw_pitch.h"
#include "system_continuous_state_integration_access.h"

namespace {

using orvd::configuration::IrwFullStateControlEventKind;
using orvd::configuration::IrwFullStateControlEventSession;
using orvd::configuration::IrwFullStateControlObservationBinding;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "IRW control event session: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

template <class Function>
bool Throws(Function&& function) {
    try {
        function();
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

bool Equal(const Eigen::VectorXd& left, const Eigen::VectorXd& right) {
    return left.size() == right.size() &&
           (left.array() == right.array()).all();
}

orvd::integrators::ContinuousStateErrorTolerances MakeTolerances(
    const orvd::configuration::AssembledVehicleSystem& assembled) {
    const auto& system = assembled.system();
    Eigen::VectorXd absolute =
        Eigen::VectorXd::Constant(system.continuous_state_size(), 1.0e-8);
    const auto q = system.generalized_positions_state_range();
    const auto z = system.series_spring_damper_force_state_range();
    absolute.segment(q.start(), q.size()).setConstant(1.0e-9);
    absolute.segment(z.start(), z.size()).setConstant(1.0e-6);
    return orvd::integrators::ContinuousStateErrorTolerances(
        1.0e-7, std::move(absolute));
}

void Run(const std::filesystem::path& vehicle_path,
         const std::filesystem::path& startup_path,
         const std::filesystem::path& line_path,
         const std::filesystem::path& data_root,
         const std::filesystem::path& controller_path,
         const std::filesystem::path& conditioner_path) {
    const auto vehicle =
        orvd::configuration::LoadVehicleDefinitionFromJsonFile(vehicle_path);
    const auto startup =
        orvd::configuration::LoadResolvedStartupStateFromJsonFile(
            startup_path);
    auto line = orvd::configuration::LoadTrackGeometryFromJsonFile(line_path);
    auto irregularity =
        std::make_unique<orvd::wheel_rail_contact::TrackIrregularityField>(
            orvd::configuration::LoadTrackIrregularityFieldFromDataRoot(
                data_root, "aar5_irregularity"));
    auto scenario = orvd::configuration::AssembleIrwContactScenario(
        vehicle, startup, std::move(line), data_root, 0.0,
        std::move(irregularity));
    auto& assembled = scenario->vehicle_system();
    auto& accepted = scenario->initial_context().context();

    // The controller consumes the frozen WRL/SIMPACK physical axle-bridge
    // body basis. Exercise the fixed half-turn conversion at a realistic
    // nonzero yaw before restoring the exact H3 state used below.
    Eigen::VectorXd h3_state(assembled.system().continuous_state_size());
    assembled.system().CopyContinuousState(accepted, h3_state);
    Eigen::VectorXd yaw_state = h3_state;
    const auto front_axle =
        assembled.model().GetRigidBodyByName("axlebridge_ff");
    const auto front_positions =
        assembled.model().GetFreeBodyPositionRange(front_axle);
    const double front_station =
        accepted.wheel_rail_projection_station_hints_meters().front();
    const auto track = assembled.contact_force_plan()
                           ->track_geometry()
                           .EvaluateTrackFrame(front_station);
    auto yaw_component = assembled.system().GetMultibodyComponentView(
        accepted, assembled.system().multibody_component());
    const auto original_pose =
        assembled.model().CalcPoseInWorld(yaw_component.context(), front_axle);
    constexpr double kTestYawRadians = 0.003;
    const Eigen::Matrix3d yawed_rotation =
        track.pose().rotation_inertial_from_track() *
        Eigen::AngleAxisd(kTestYawRadians, Eigen::Vector3d::UnitZ()) *
        track.pose().rotation_inertial_from_track().transpose() *
        original_pose.rotation();
    const Eigen::Quaterniond yawed_quaternion(yawed_rotation);
    yaw_state.segment<4>(front_positions.start()) << yawed_quaternion.w(),
        yawed_quaternion.x(), yawed_quaternion.y(), yawed_quaternion.z();
    assembled.system().SetContinuousState(accepted, yaw_state);
    const IrwFullStateControlObservationBinding yaw_binding(assembled);
    const auto yaw_observation = yaw_binding.Observe(accepted);
    yaw_component = assembled.system().GetMultibodyComponentView(
        accepted, assembled.system().multibody_component());
    const Eigen::Matrix3d rotation_track_from_orvd_body =
        track.pose().rotation_inertial_from_track().transpose() *
        assembled.model()
            .CalcPoseInWorld(yaw_component.context(), front_axle)
            .rotation();
    const Eigen::Matrix3d body_basis_half_turn =
        Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
    const double expected_source_yaw =
        orvd::wheel_rail_contact::ResolveRollYawPitch(
            rotation_track_from_orvd_body * body_basis_half_turn)
            .yaw_radians;
    Require(std::abs(expected_source_yaw) > 0.002 &&
                yaw_observation.mechanical_input.axle_yaw_angles_radians
                        .front() ==
                    expected_source_yaw,
            "the nonzero axle-bridge yaw was not restored to the frozen "
            "source body basis");
    assembled.system().SetContinuousState(accepted, h3_state);

    IrwFullStateControlEventSession session(
        assembled,
        orvd::configuration::
            LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile(
                controller_path),
        orvd::configuration::
            LoadWheelDriveTorqueCommandConditionerFromJsonFile(
                conditioner_path));
    const auto direct_observation = yaw_binding.Observe(accepted);
    const auto adapted_observation = session.ObserveMechanicalInput(accepted);
    Require(
        direct_observation.axle_track_stations_meters ==
                adapted_observation.axle_track_stations_meters &&
            direct_observation.mechanical_input
                    .axle_lateral_displacements_meters ==
                adapted_observation.axle_lateral_displacements_meters &&
            direct_observation.mechanical_input.axle_yaw_angles_radians ==
                adapted_observation.axle_yaw_angles_radians &&
            direct_observation.mechanical_input
                    .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second ==
                adapted_observation
                    .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        "the frozen session did not delegate to the public mechanical "
        "observation binding");
    Require(session.sample_period_seconds() == 0.01 &&
                session.next_periodic_event_ordinal() == 0 &&
                session.next_periodic_event_time_seconds() == 0.0,
            "the frozen period or initial integer event ordinal is wrong");
    Require(Throws([&] { session.RequireReadyToAdvance(); }),
            "advance was admitted before the initialization and U0 event");

    const Eigen::VectorXd zero_held =
        accepted.held_independent_wheel_active_torques_newton_metres();
    const auto initialization = session.ApplyInitializationUpdate(accepted);
    Require(initialization.kind ==
                    IrwFullStateControlEventKind::kInitialization &&
                initialization.event_time_seconds == 0.0 &&
                initialization.periodic_event_ordinal == 0,
            "the first recurrence is not an initialization audit");
    Require(Equal(zero_held,
                  accepted
                      .held_independent_wheel_active_torques_newton_metres()),
            "the initialization-only output entered the held vehicle torque");
    Require(session.controller_state().initialized &&
                Throws([&] {
                    (void)session.ApplyInitializationUpdate(accepted);
                }),
            "the initialization memory was not committed exactly once");

    const auto u0 = session.ApplyPeriodicUpdate(accepted);
    Require(u0.kind == IrwFullStateControlEventKind::kPeriodic &&
                u0.periodic_event_ordinal == 0 &&
                u0.event_time_seconds == 0.0 &&
                u0.mechanical_input.axle_lateral_displacements_meters ==
                    initialization.mechanical_input
                        .axle_lateral_displacements_meters &&
                u0.mechanical_input.axle_yaw_angles_radians ==
                    initialization.mechanical_input.axle_yaw_angles_radians &&
                u0.mechanical_input
                        .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second ==
                    initialization.mechanical_input
                        .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
            "U0 did not reuse the same accepted H3 mechanical observation");
    for (std::size_t wheel = 0;
         wheel < u0.conditioning_result.actual_wheel_torques_newton_metres
                     .size();
         ++wheel) {
        Require(
            accepted
                    .held_independent_wheel_active_torques_newton_metres()
                        [static_cast<Eigen::Index>(wheel)] ==
                u0.conditioning_result
                    .actual_wheel_torques_newton_metres[wheel],
            "U0 did not publish the complete conditioned torque vector");
    }
    Require(session.synchronization_required() &&
                session.next_periodic_event_ordinal() == 1 &&
                session.next_periodic_event_time_seconds() == 0.01,
            "U0 did not advance the integer event session exactly once");

    // The backend is constructed only after U0 is in the accepted context, so
    // no artificial t=0 reinitialization is introduced.
    auto advancer = orvd::integrators::internal::
        SystemContinuousStateIntegrationAccess::Make(
            orvd::integrators::internal::
                SystemContinuousStateIntegrationRecipe::kRadau5,
            assembled.system(), assembled.compiled_plan(), accepted,
            MakeTolerances(assembled),
            orvd::integrators::NoCallTimeAppliedForces{});
    Require(orvd::integrators::internal::
                    SystemContinuousStateIntegrationAccess::ConfiguredRecipe(
                        *advancer) ==
                orvd::integrators::internal::
                    SystemContinuousStateIntegrationRecipe::kRadau5 &&
                advancer->integration_statistics()
                        .requested_dense_finite_difference_jacobian_worker_count ==
                    1,
            "the 100 Hz event qualification did not construct serial "
            "Radau5");
    session.ConfirmBackendSynchronized();
    session.RequireReadyToAdvance();

    advancer->AdvanceTo(session.next_periodic_event_time_seconds());
    const auto u1 = session.ApplyPeriodicUpdate(accepted);
    Require(u1.periodic_event_ordinal == 1 &&
                u1.event_time_seconds == 0.01 &&
                session.synchronization_required(),
            "U1 is not the first positive-time integer-grid event");
    Require(Throws([&] { session.RequireReadyToAdvance(); }),
            "advance was admitted before U1 backend synchronization");
    advancer->SynchronizeAfterAcceptedContextChange();
    session.ConfirmBackendSynchronized();
    session.RequireReadyToAdvance();

    advancer->AdvanceTo(session.next_periodic_event_time_seconds());
    const auto u2 = session.ApplyPeriodicUpdate(accepted);
    Require(u2.periodic_event_ordinal == 2 &&
                u2.event_time_seconds == 0.02 &&
                accepted.time_seconds() == 0.02 &&
                session.synchronization_required(),
            "the terminal U2 audit is not at the second integer boundary");

    Eigen::VectorXd rhs(assembled.system().continuous_state_size());
    assembled.compiled_plan().CalcStateTimeDerivatives(accepted, rhs);
    Require(rhs.size() == 157 && rhs.allFinite(),
            "the controlled IRW terminal 157-state RHS is not finite");

    auto component = assembled.system().GetMultibodyComponentView(
        accepted, assembled.system().multibody_component());
    auto workspace = assembled.contact_force_plan()->CreateWorkspace();
    std::vector<orvd::multibody_model::AppliedBodyWrench> contact_wrenches(8);
    std::array<orvd::forces::WheelRailContactInterfaceObservation, 8>
        observations{};
    assembled.contact_force_plan()->CalcAppliedForcesAndObservations(
        component.context(), *workspace,
        accepted.wheel_rail_projection_station_hints_meters(),
        contact_wrenches, observations);
    for (const auto& observation : observations) {
        Require(std::isfinite(observation.normal_force_newtons) &&
                    observation.contact_patch_count > 0,
                "a real controlled IRW wheel has no finite contact result");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::fprintf(
            stderr,
            "usage: verify_irw_full_state_control_event_session VEHICLE "
            "STARTUP LINE DATA_ROOT CONTROLLER CONDITIONER\n");
        return 2;
    }
    try {
        Run(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW control event session threw: %s\n",
                     error.what());
        ++failures;
    }
    if (failures != 0) {
        std::fprintf(stderr, "%d IRW control event assertion(s) failed\n",
                     failures);
        return 1;
    }
    std::puts("IRW 100 Hz control event session verified");
    return 0;
}
