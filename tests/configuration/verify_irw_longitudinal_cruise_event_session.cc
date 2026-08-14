#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <string_view>
#include <utility>

#include <Eigen/Core>

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/irw_longitudinal_cruise_event_session.h"
#include "orvd/configuration/load_irw_longitudinal_cruise_controller.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/load_wheel_drive_torque_command_conditioner.h"
#include "orvd/integrators/system_continuous_state_advancer.h"

namespace {

using orvd::configuration::IrwLongitudinalCruiseEventKind;
using orvd::configuration::IrwLongitudinalCruiseEventSession;

int failures = 0;

void Require(bool condition, std::string_view message) {
    if (!condition) {
        std::fprintf(stderr, "IRW longitudinal cruise session: %.*s\n",
                     static_cast<int>(message.size()), message.data());
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

bool Equal(
    const orvd::control::SampledLongitudinalCruiseControllerState& left,
    const orvd::control::SampledLongitudinalCruiseControllerState& right) {
    return left.speed_pi.integral == right.speed_pi.integral &&
           left.speed_pi.filtered_output == right.speed_pi.filtered_output;
}

bool Equal(
    const orvd::actuation::WheelDriveTorqueChannelValues& left,
    const orvd::actuation::WheelDriveTorqueChannelValues& right) {
    return left == right;
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

void RequireCommonRequest(
    const orvd::configuration::IrwLongitudinalCruiseEventAudit& audit) {
    const double common =
        audit.controller_result.requested_common_wheel_torque_newton_metres;
    for (const double requested :
         audit.requested_wheel_torques_newton_metres) {
        Require(requested == common,
                "the scalar PI request was not copied identically to all "
                "eight wheels");
    }
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
    auto scenario = orvd::configuration::AssembleIrwContactScenario(
        vehicle, startup, std::move(line), data_root, 0.0, 0.01, nullptr);
    auto& assembled = scenario->vehicle_system();
    auto& accepted = scenario->initial_context().context();
    auto cruise_asset = orvd::configuration::
        LoadIrwLongitudinalCruiseControllerFromJsonFile(controller_path);
    const double target_speed =
        cruise_asset.controller.config().target_speed_meters_per_second;
    IrwLongitudinalCruiseEventSession session(
        assembled, std::move(cruise_asset.controller),
        cruise_asset.nominal_rolling_radius_meters,
        cruise_asset.forward_joint_rate_signs,
        orvd::configuration::
            LoadWheelDriveTorqueCommandConditionerFromJsonFile(
                conditioner_path));

    Require(session.sample_period_seconds() == 0.01 &&
                session.nominal_rolling_radius_meters() == 0.43 &&
                session.next_periodic_event_ordinal() == 0 &&
                session.next_periodic_event_time_seconds() == 0.0,
            "the initial cruise clock or rolling-radius authority is wrong");
    Require(Throws([&] { session.RequireReadyToAdvance(); }),
            "advance was admitted before U0");

    const auto initial_observation = session.ObserveWheelSpeeds(accepted);
    double manual_sum = 0.0;
    for (std::size_t wheel = 0;
         wheel < initial_observation.raw_joint_rates_radians_per_second.size();
         ++wheel) {
        const double raw =
            initial_observation.raw_joint_rates_radians_per_second[wheel];
        const double forward = initial_observation
                                   .forward_wheel_circumferential_speeds_meters_per_second
                                       [wheel];
        Require(raw > 0.0 &&
                    initial_observation
                            .conditioner_scalar_wheel_speeds_radians_per_second
                                [wheel] == -raw &&
                    forward == raw * 0.43,
                "raw, forward and conditioner wheel-rate conventions were "
                "not kept distinct");
        manual_sum += forward;
    }
    Require(initial_observation
                    .common_forward_wheel_circumferential_speed_meters_per_second ==
                manual_sum / 8.0 &&
                initial_observation
                        .common_forward_wheel_circumferential_speed_meters_per_second >
                    16.0 &&
                initial_observation
                        .common_forward_wheel_circumferential_speed_meters_per_second <
                    17.0,
            "the eight-wheel arithmetic mean is not the 60 km/h observation");

    const Eigen::VectorXd zero_held =
        accepted.held_independent_wheel_active_torques_newton_metres();
    const auto u0 = session.ApplyInitializationUpdate(accepted);
    Require(u0.kind == IrwLongitudinalCruiseEventKind::kInitialization &&
                u0.periodic_event_ordinal == 0 &&
                u0.event_time_seconds == 0.0 &&
                u0.controller_result.speed_error_meters_per_second ==
                    target_speed -
                        u0.wheel_speed_observation
                            .common_forward_wheel_circumferential_speed_meters_per_second &&
                u0.controller_result
                        .requested_common_wheel_torque_newton_metres <
                    0.0,
            "U0 did not brake the slightly overspeed resolved startup state");
    RequireCommonRequest(u0);
    Require(!Equal(zero_held,
                   accepted
                       .held_independent_wheel_active_torques_newton_metres()),
            "U0 did not enter the accepted held-torque state");
    for (std::size_t wheel = 0;
         wheel < u0.conditioning_result.actual_wheel_torques_newton_metres
                     .size();
         ++wheel) {
        Require(
            accepted
                    .held_independent_wheel_active_torques_newton_metres()
                        [static_cast<Eigen::Index>(wheel)] ==
                u0.conditioning_result
                    .actual_wheel_torques_newton_metres[wheel] &&
                u0.conditioning_result
                        .actual_wheel_torques_newton_metres[wheel] <
                    0.0,
            "U0 did not atomically publish braking torque for the "
            "overspeed startup state");
    }
    const Eigen::VectorXd held_u0 =
        accepted.held_independent_wheel_active_torques_newton_metres();
    const auto controller_state_u0 = session.controller_state();
    const auto conditioner_memory_u0 =
        session.conditioner_memory_newton_metres();
    Require(session.synchronization_required() &&
                session.next_periodic_event_ordinal() == 1 &&
                session.next_periodic_event_time_seconds() == 0.01 &&
                Throws([&] {
                    (void)session.ApplyInitializationUpdate(accepted);
                }) &&
                Equal(held_u0,
                      accepted.held_independent_wheel_active_torques_newton_metres()) &&
                Equal(controller_state_u0, session.controller_state()) &&
                Equal(conditioner_memory_u0,
                      session.conditioner_memory_newton_metres()),
            "U0 was not committed exactly once");
    Require(Throws([&] { (void)session.ApplyPeriodicUpdate(accepted); }) &&
                Equal(held_u0,
                      accepted.held_independent_wheel_active_torques_newton_metres()) &&
                Equal(controller_state_u0, session.controller_state()) &&
                Equal(conditioner_memory_u0,
                      session.conditioner_memory_newton_metres()),
            "an unsynchronized periodic event changed accepted control "
            "state");

    orvd::integrators::SystemContinuousStateAdvancer advancer(
        assembled.system(), assembled.compiled_plan(), accepted,
        MakeTolerances(assembled),
        orvd::integrators::NoCallTimeAppliedForces{});
    session.ConfirmBackendSynchronized();
    session.RequireReadyToAdvance();

    for (std::uint64_t ordinal = 1; ordinal <= 10; ++ordinal) {
        advancer.AdvanceTo(session.next_periodic_event_time_seconds());
        const auto event = session.ApplyPeriodicUpdate(accepted);
        Require(event.kind == IrwLongitudinalCruiseEventKind::kPeriodic &&
                    event.periodic_event_ordinal == ordinal &&
                    event.event_time_seconds ==
                        static_cast<double>(ordinal) * 0.01,
                "a periodic cruise event left the integer 100 Hz grid");
        RequireCommonRequest(event);
        if (ordinal < 10) {
            advancer.SynchronizeAfterAcceptedContextChange();
            session.ConfirmBackendSynchronized();
            session.RequireReadyToAdvance();
        }
    }

    const auto terminal_observation = session.ObserveWheelSpeeds(accepted);
    Require(accepted.time_seconds() == 0.1 &&
                std::isfinite(
                    terminal_observation
                        .common_forward_wheel_circumferential_speed_meters_per_second) &&
                session.synchronization_required(),
            "the 100 ms real consumer did not finish at a finite accepted "
            "event");
    Eigen::VectorXd rhs(assembled.system().continuous_state_size());
    assembled.compiled_plan().CalcStateTimeDerivatives(accepted, rhs);
    Require(rhs.size() == 157 && rhs.allFinite(),
            "the terminal cruise-controlled 157-state RHS is not finite");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 7) {
        std::fprintf(
            stderr,
            "usage: verify_irw_longitudinal_cruise_event_session VEHICLE "
            "STARTUP LINE DATA_ROOT CONTROLLER CONDITIONER\n");
        return 2;
    }
    try {
        Run(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "IRW longitudinal cruise session threw: %s\n",
                     error.what());
        ++failures;
    }
    if (failures != 0) {
        std::fprintf(stderr,
                     "%d IRW longitudinal cruise assertion(s) failed\n",
                     failures);
        return 1;
    }
    std::puts("IRW 100 Hz longitudinal cruise session verified");
    return 0;
}
