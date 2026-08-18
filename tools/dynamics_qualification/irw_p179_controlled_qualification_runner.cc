#include "irw_qualification_runner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "atomic_qualification_directory.h"
#include "irw_integration_recipe.h"
#include "qualification_sample_clock.h"
#include "vehicle_qualification_runner_internal.h"

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/irw_full_state_control_event_session.h"
#include "orvd/configuration/load_irw_full_state_wheel_speed_guidance_controller.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_track_irregularity_field.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/configuration/load_wheel_drive_torque_command_conditioner.h"
#include "orvd/forces/independent_wheel_active_torque_plan.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/multibody_model/multibody_applied_forces.h"
#include "orvd/multibody_model/multibody_model.h"

namespace orvd::dynamics_qualification {
namespace {

using Clock = std::chrono::steady_clock;
using configuration::IrwFullStateControlEventAudit;
using forces::WheelRailContactInterfaceObservation;
using forces::WheelRailContactPatchObservation;
using multibody_model::AppliedBodyWrench;

constexpr std::int64_t kEventPeriodNanoseconds = 10'000'000;
constexpr std::int64_t kObservationPeriodNanoseconds = 500'000;
constexpr std::uint64_t kObservationsPerEventPeriod =
    static_cast<std::uint64_t>(kEventPeriodNanoseconds /
                               kObservationPeriodNanoseconds);
static_assert(kEventPeriodNanoseconds % kObservationPeriodNanoseconds == 0);
constexpr double kVehicleReferenceTrackStationMeters = 0.0;
constexpr double kProjectionSearchHalfWidthMeters = 0.01;
constexpr std::string_view kIrregularityIdentifier =
    "aar5_irregularity";
constexpr std::size_t kAxleCount = 4;
constexpr std::size_t kWheelCount = 8;

struct ResolvedConfiguration final {
    std::filesystem::path vehicle_definition_path;
    std::filesystem::path resolved_startup_state_path;
    std::filesystem::path track_geometry_path;
    std::filesystem::path orvd_data_root;
    std::filesystem::path controller_configuration_path;
    std::filesystem::path torque_conditioner_configuration_path;
    std::filesystem::path output_directory;
    std::int64_t duration_nanoseconds{};
};

struct ControlledObservation final {
    std::uint64_t sample_index{};
    std::uint64_t time_nanoseconds{};
    double time_seconds{};
    control::IrwGuidanceAxleValues track_stations_meters{};
    control::IrwGuidanceAxleValues lateral_displacements_meters{};
    control::IrwGuidanceAxleValues yaw_angles_radians{};
    std::array<std::size_t, kWheelCount> contact_patch_counts{};
    std::array<double, kWheelCount> vertical_support_forces_newtons{};
    std::array<double, kWheelCount> normal_forces_newtons{};
    std::array<Eigen::Vector3d, kWheelCount>
        total_forces_in_carrier_track_frame_newtons{};
};

struct ControlledEndpointDiagnostics final {
    std::uint64_t held_torque_event_ordinal{};
    double time_seconds{};
    double state_derivative_inf_norm{};
    double generalized_force_residual_inf_norm{};
    double virtual_power_residual_watts{};
};

struct ControlledPatchObservation final {
    std::uint64_t sample_index{};
    std::uint64_t time_nanoseconds{};
    double time_seconds{};
    std::size_t interface_ordinal{};
    std::size_t patch_ordinal{};
    WheelRailContactPatchObservation patch;
};

[[noreturn]] void Reject(const std::string& detail) {
    throw std::runtime_error("IRW P179 controlled qualification: " + detail);
}

[[nodiscard]] double ElapsedSeconds(Clock::time_point begin,
                                    Clock::time_point end) {
    return std::chrono::duration<double>(end - begin).count();
}

[[nodiscard]] std::filesystem::path CanonicalExistingInput(
    const std::filesystem::path& input, std::string_view name) {
    std::error_code error;
    const std::filesystem::path resolved =
        std::filesystem::canonical(input, error);
    if (error) {
        Reject("could not resolve " + std::string(name) + " '" +
               input.string() + "': " + error.message());
    }
    return resolved;
}

[[nodiscard]] ResolvedConfiguration ResolveConfiguration(
    const IrwP179ControlledQualificationRunConfiguration& input) {
    if (input.duration_nanoseconds <= 0 ||
        input.duration_nanoseconds % kEventPeriodNanoseconds != 0) {
        throw std::invalid_argument(
            "IRW P179 controlled qualification duration must be a positive "
            "integer multiple of 10,000,000 ns");
    }
    return ResolvedConfiguration{
        CanonicalExistingInput(input.vehicle_definition_path,
                               "vehicle definition"),
        CanonicalExistingInput(input.resolved_startup_state_path,
                               "resolved start-up state"),
        CanonicalExistingInput(input.track_geometry_path, "track geometry"),
        CanonicalExistingInput(input.orvd_data_root, "ORVD data root"),
        CanonicalExistingInput(input.controller_configuration_path,
                               "controller configuration"),
        CanonicalExistingInput(input.torque_conditioner_configuration_path,
                               "torque conditioner configuration"),
        input.output_directory,
        input.duration_nanoseconds,
    };
}

void CloseChecked(std::ofstream* stream, const std::filesystem::path& path) {
    stream->flush();
    if (!*stream) {
        Reject("could not flush '" + path.string() + "'");
    }
    stream->close();
    if (!*stream) {
        Reject("could not close '" + path.string() + "'");
    }
}

[[nodiscard]] std::string JsonString(std::string_view input) {
    std::ostringstream output;
    output << '"';
    for (const char character : input) {
        if (character == '"' || character == '\\') {
            output << '\\';
        }
        output << character;
    }
    output << '"';
    return output.str();
}

[[nodiscard]] integrators::ContinuousStateErrorTolerances MakeTolerances(
    const configuration::AssembledVehicleSystem& assembled) {
    constexpr auto& recipe = internal::kIrwP179ControlledIntegrationRecipe;
    const auto& system = assembled.system();
    Eigen::VectorXd absolute = Eigen::VectorXd::Constant(
        system.continuous_state_size(),
        recipe.velocity_absolute_tolerance);
    const auto q = system.generalized_positions_state_range();
    const auto z = system.series_spring_damper_force_state_range();
    absolute.segment(q.start(), q.size())
        .setConstant(recipe.position_absolute_tolerance);
    absolute.segment(z.start(), z.size())
        .setConstant(recipe.series_force_absolute_tolerance_newtons);
    return integrators::ContinuousStateErrorTolerances(
        recipe.relative_tolerance, std::move(absolute));
}

void AccumulateStatistics(
    const integrators::ContinuousStateIntegrationStatistics& source,
    integrators::ContinuousStateIntegrationStatistics* destination) {
    destination->successful_internal_step_count +=
        source.successful_internal_step_count;
    destination->right_hand_side_evaluation_count +=
        source.right_hand_side_evaluation_count;
    destination->linear_solver_right_hand_side_evaluation_count +=
        source.linear_solver_right_hand_side_evaluation_count;
    destination->error_test_failure_count += source.error_test_failure_count;
    destination->nonlinear_solver_iteration_count +=
        source.nonlinear_solver_iteration_count;
    destination->nonlinear_solver_convergence_failure_count +=
        source.nonlinear_solver_convergence_failure_count;
    destination->linear_solver_setup_count += source.linear_solver_setup_count;
    destination->jacobian_evaluation_count +=
        source.jacobian_evaluation_count;
    const int requested_workers =
        source.requested_dense_finite_difference_jacobian_worker_count;
    if (destination
            ->requested_dense_finite_difference_jacobian_worker_count == 0) {
        destination
            ->requested_dense_finite_difference_jacobian_worker_count =
            requested_workers;
    } else if (destination
                   ->requested_dense_finite_difference_jacobian_worker_count !=
               requested_workers) {
        Reject("the requested dense-Jacobian worker count changed between "
               "control intervals");
    }
}

[[nodiscard]] double WrenchPower(
    const multibody_model::MultibodyModel& model,
    const multibody_model::MultibodyEvaluationContext& context,
    const AppliedBodyWrench& wrench) {
    const auto pose = model.CalcPoseInWorld(context, wrench.body);
    const auto velocity =
        model.CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld(
            context, wrench.body);
    const Eigen::Vector3d point_offset_in_world =
        pose.rotation() * wrench.point_position_in_body_frame_meters;
    const Eigen::Vector3d point_velocity =
        velocity.translational_velocity_at_frame_origin_meters_per_second() +
        velocity.angular_velocity_radians_per_second().cross(
            point_offset_in_world);
    return wrench.torque_about_point_newton_metres.dot(
               velocity.angular_velocity_radians_per_second()) +
           wrench.force_newtons.dot(point_velocity);
}

[[nodiscard]] ControlledObservation ObserveControlledSample(
    const configuration::AssembledVehicleSystem& assembled,
    const configuration::IrwFullStateControlEventSession& event_session,
    system_assembly::SystemRuntimeContext& context,
    std::uint64_t sample_index, std::uint64_t time_nanoseconds,
    double time_seconds,
    forces::WheelRailContactForceWorkspace& contact_workspace,
    std::span<AppliedBodyWrench> contact_wrenches,
    std::span<WheelRailContactInterfaceObservation> contact_observations,
    std::vector<ControlledPatchObservation>* patch_observations) {
    auto component = assembled.system().GetMultibodyComponentView(
        context, assembled.system().multibody_component());
    assembled.contact_force_plan()->CalcAppliedForcesAndObservations(
        component.context(), contact_workspace,
        context.wheel_rail_projection_station_hints_meters(),
        contact_wrenches, contact_observations);

    ControlledObservation observation;
    observation.sample_index = sample_index;
    observation.time_nanoseconds = time_nanoseconds;
    observation.time_seconds = time_seconds;
    const auto mechanical_input = event_session.ObserveMechanicalInput(context);
    for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
        observation.track_stations_meters[axle] =
            mechanical_input.axle_track_stations_meters[axle];
        observation.lateral_displacements_meters[axle] =
            mechanical_input.axle_lateral_displacements_meters[axle];
        observation.yaw_angles_radians[axle] =
            mechanical_input.axle_yaw_angles_radians[axle];
        if (!std::isfinite(observation.track_stations_meters[axle]) ||
            !std::isfinite(observation.lateral_displacements_meters[axle]) ||
            !std::isfinite(observation.yaw_angles_radians[axle])) {
            Reject("a controlled mechanical observation is not finite");
        }
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        const auto& contact = contact_observations[wheel];
        observation.contact_patch_counts[wheel] = contact.contact_patch_count;
        observation.vertical_support_forces_newtons[wheel] =
            contact.vertical_support_force_on_wheel_newtons;
        observation.normal_forces_newtons[wheel] = contact.normal_force_newtons;
        observation.total_forces_in_carrier_track_frame_newtons[wheel] =
            contact.total_force_on_wheel_in_carrier_track_frame_newtons;
        if (!std::isfinite(observation.vertical_support_forces_newtons[wheel]) ||
            !std::isfinite(observation.normal_forces_newtons[wheel]) ||
            !observation
                 .total_forces_in_carrier_track_frame_newtons[wheel]
                 .allFinite()) {
            Reject("a controlled contact observation is not finite");
        }
        for (std::size_t patch = 0; patch < contact.contact_patch_count;
             ++patch) {
            patch_observations->push_back(ControlledPatchObservation{
                sample_index, time_nanoseconds, time_seconds, wheel, patch,
                contact.patches[patch]});
        }
    }
    return observation;
}

[[nodiscard]] ControlledEndpointDiagnostics CalcControlledEndpointDiagnostics(
    const configuration::AssembledVehicleSystem& assembled,
    system_assembly::SystemRuntimeContext& context,
    std::uint64_t held_torque_event_ordinal,
    forces::WheelRailContactForceWorkspace& contact_workspace,
    std::span<AppliedBodyWrench> vehicle_wrenches,
    std::span<AppliedBodyWrench> contact_wrenches,
    std::span<AppliedBodyWrench> active_wrenches,
    Eigen::VectorXd* series_derivatives) {
    auto component = assembled.system().GetMultibodyComponentView(
        context, assembled.system().multibody_component());
    assembled.force_plan().CalcAppliedForces(
        component.context(), context.series_spring_damper_forces(),
        context.nominal_forces(), vehicle_wrenches, *series_derivatives);
    assembled.contact_force_plan()->CalcAppliedForces(
        component.context(), contact_workspace,
        context.wheel_rail_projection_station_hints_meters(),
        contact_wrenches);
    assembled.active_torque_plan()->CalcAppliedForces(
        component.context(),
        std::span<const double>(
            context
                .held_independent_wheel_active_torques_newton_metres()
                .data(),
            static_cast<std::size_t>(
                context
                    .held_independent_wheel_active_torques_newton_metres()
                    .size())),
        active_wrenches);

    Eigen::VectorXd rhs(assembled.system().continuous_state_size());
    assembled.compiled_plan().CalcStateTimeDerivatives(context, rhs);
    if (!rhs.allFinite()) {
        Reject("a controlled endpoint produced a non-finite 157-state RHS");
    }

    const int nq = assembled.model().num_generalized_positions();
    const int nv = assembled.model().num_generalized_velocities();
    Eigen::VectorXd projected_generalized_force = Eigen::VectorXd::Zero(nv);
    Eigen::MatrixXd angular_jacobian(3, nv);
    Eigen::MatrixXd translational_jacobian(3, nv);
    double spatial_power = 0.0;
    const auto accumulate_wrenches = [&](std::span<const AppliedBodyWrench> set) {
        for (const AppliedBodyWrench& wrench : set) {
            assembled.model()
                .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                    component.context(), wrench.body,
                    wrench.point_position_in_body_frame_meters,
                    &angular_jacobian, &translational_jacobian);
            projected_generalized_force.noalias() +=
                angular_jacobian.transpose() *
                    wrench.torque_about_point_newton_metres +
                translational_jacobian.transpose() * wrench.force_newtons;
            spatial_power +=
                WrenchPower(assembled.model(), component.context(), wrench);
        }
    };
    accumulate_wrenches(vehicle_wrenches);
    accumulate_wrenches(contact_wrenches);
    accumulate_wrenches(active_wrenches);
    Eigen::VectorXd required_generalized_force(nv);
    assembled.model().CalcRequiredGeneralizedForces(
        component.context(), rhs.segment(nq, nv),
        required_generalized_force);

    ControlledEndpointDiagnostics diagnostics;
    diagnostics.held_torque_event_ordinal = held_torque_event_ordinal;
    diagnostics.time_seconds = context.time_seconds();
    diagnostics.state_derivative_inf_norm = rhs.lpNorm<Eigen::Infinity>();
    diagnostics.generalized_force_residual_inf_norm =
        (projected_generalized_force - required_generalized_force)
            .lpNorm<Eigen::Infinity>();
    diagnostics.virtual_power_residual_watts =
        projected_generalized_force.dot(context.generalized_velocities()) -
        spatial_power;
    if (!std::isfinite(diagnostics.state_derivative_inf_norm) ||
        !std::isfinite(diagnostics.generalized_force_residual_inf_norm) ||
        !std::isfinite(diagnostics.virtual_power_residual_watts)) {
        Reject("a controlled endpoint diagnostic is not finite");
    }
    return diagnostics;
}

template <std::size_t Size>
void WriteArrayHeader(std::ofstream* output, std::string_view prefix,
                      const std::array<std::string_view, Size>& names) {
    for (const std::string_view name : names) {
        *output << '\t' << prefix << name;
    }
}

template <std::size_t Size>
void WriteArray(std::ofstream* output,
                const std::array<double, Size>& values) {
    for (const double value : values) {
        *output << '\t' << value;
    }
}

void WriteControllerStateHeader(std::ofstream* output,
                                std::string_view prefix) {
    *output << '\t' << prefix << "initialized";
    constexpr std::array<std::string_view, 4> kAxles{"ff", "fr", "rf",
                                                   "rr"};
    constexpr std::array<std::string_view, 8> kWheels{
        "ff_l", "ff_r", "fr_l", "fr_r", "rf_l", "rf_r", "rr_l",
        "rr_r"};
    for (const std::string_view family :
         {std::string_view("previous_y."), std::string_view("previous_yaw."),
          std::string_view("filtered_y_dot."),
          std::string_view("filtered_yaw_rate."),
          std::string_view("lateral_integral."),
          std::string_view("filtered_delta_omega_command.")}) {
        WriteArrayHeader(output, std::string(prefix) + std::string(family),
                         kAxles);
    }
    WriteArrayHeader(output, std::string(prefix) + "pi_integral.", kWheels);
    WriteArrayHeader(output, std::string(prefix) + "pi_filtered_torque.",
                     kWheels);
}

void WriteControllerState(
    std::ofstream* output,
    const control::IrwFullStateWheelSpeedGuidanceControllerState& state) {
    *output << '\t' << (state.initialized ? 1 : 0);
    WriteArray(output, state.previous_lateral_displacements_meters);
    WriteArray(output, state.previous_yaw_angles_radians);
    WriteArray(output, state.filtered_lateral_velocities_meters_per_second);
    WriteArray(output, state.filtered_yaw_rates_radians_per_second);
    WriteArray(output, state.lateral_error_integrals_meter_seconds);
    WriteArray(
        output,
        state.filtered_wheel_speed_difference_commands_radians_per_second);
    WriteArray(output, state.wheel_speed_pi_integrals_meters);
    WriteArray(output,
               state.wheel_speed_pi_filtered_torques_newton_metres);
}

void WriteControlAudits(
    const std::filesystem::path& path,
    const std::vector<IrwFullStateControlEventAudit>& audits) {
    constexpr std::array<std::string_view, 4> kAxles{"ff", "fr", "rf",
                                                   "rr"};
    constexpr std::array<std::string_view, 8> kWheels{
        "ff_l", "ff_r", "fr_l", "fr_r", "rf_l", "rf_r", "rr_l",
        "rr_r"};
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "event_kind\tperiodic_event_ordinal\tevent_time_seconds";
    WriteArrayHeader(&output, "input.station.", kAxles);
    WriteArrayHeader(&output, "input.lateral.", kAxles);
    WriteArrayHeader(&output, "input.source_body_yaw.", kAxles);
    WriteArrayHeader(&output, "input.frozen_wheel_speed.", kWheels);
    WriteArrayHeader(&output, "controller.request.", kWheels);
    WriteArrayHeader(&output, "controller.base_speed_reference.", kWheels);
    WriteArrayHeader(&output, "controller.speed_reference.", kWheels);
    WriteArrayHeader(&output, "controller.delta_omega_reference.", kAxles);
    WriteArrayHeader(&output, "controller.delta_omega_measured.", kAxles);
    WriteArrayHeader(&output, "controller.delta_omega_equilibrium.", kAxles);
    WriteArrayHeader(&output, "controller.filtered_lateral_velocity.",
                     kAxles);
    WriteArrayHeader(&output, "controller.filtered_yaw_rate.", kAxles);
    WriteArrayHeader(&output, "controller.guidance_active.", kAxles);
    WriteArrayHeader(&output, "conditioner.actual_torque.", kWheels);
    WriteArrayHeader(&output, "conditioner.dynamic_limit.", kWheels);
    WriteArrayHeader(&output, "conditioner.limit_flag.", kWheels);
    WriteArrayHeader(&output, "conditioner.drive_speed_rpm.", kWheels);
    WriteArrayHeader(&output, "conditioner.memory_before.", kWheels);
    WriteArrayHeader(&output, "conditioner.memory_after.", kWheels);
    WriteControllerStateHeader(&output, "controller_state_before.");
    WriteControllerStateHeader(&output, "controller_state_after.");
    output << '\n';

    for (const auto& audit : audits) {
        output << (audit.kind ==
                           configuration::IrwFullStateControlEventKind::
                               kInitialization
                       ? "initialization"
                       : "periodic")
               << '\t' << audit.periodic_event_ordinal << '\t'
               << audit.event_time_seconds;
        WriteArray(&output,
                   audit.mechanical_input.axle_track_stations_meters);
        WriteArray(&output,
                   audit.mechanical_input.axle_lateral_displacements_meters);
        WriteArray(&output, audit.mechanical_input.axle_yaw_angles_radians);
        WriteArray(
            &output,
            audit.mechanical_input
                .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second);
        WriteArray(
            &output,
            audit.controller_result.requested_wheel_torques_newton_metres);
        WriteArray(&output,
                   audit.controller_result.observations
                       .base_wheel_speed_references_meters_per_second);
        WriteArray(&output,
                   audit.controller_result.observations
                       .wheel_speed_references_meters_per_second);
        WriteArray(
            &output,
            audit.controller_result.observations
                .wheel_speed_difference_references_radians_per_second);
        WriteArray(
            &output,
            audit.controller_result.observations
                .measured_wheel_speed_differences_radians_per_second);
        WriteArray(
            &output,
            audit.controller_result.observations
                .equilibrium_wheel_speed_differences_radians_per_second);
        WriteArray(
            &output,
            audit.controller_result.observations
                .filtered_lateral_velocities_meters_per_second);
        WriteArray(&output,
                   audit.controller_result.observations
                       .filtered_yaw_rates_radians_per_second);
        for (const bool active :
             audit.controller_result.observations.guidance_active) {
            output << '\t' << (active ? 1 : 0);
        }
        WriteArray(
            &output,
            audit.conditioning_result.actual_wheel_torques_newton_metres);
        WriteArray(
            &output,
            audit.conditioning_result
                .wheel_dynamic_torque_limits_newton_metres);
        for (const auto flag : audit.conditioning_result.limit_flags) {
            output << '\t' << static_cast<std::uint16_t>(flag);
        }
        WriteArray(&output,
                   audit.conditioning_result
                       .equivalent_drive_side_speeds_revolutions_per_minute);
        WriteArray(&output, audit.conditioner_memory_before_newton_metres);
        WriteArray(
            &output,
            audit.conditioning_result
                .next_drive_side_torque_memory_newton_metres);
        WriteControllerState(&output, audit.controller_state_before);
        WriteControllerState(&output, audit.controller_result.next_state);
        output << '\n';
    }
    CloseChecked(&output, path);
}

void WriteObservations(const std::filesystem::path& path,
                       const std::vector<ControlledObservation>& observations) {
    constexpr std::array<std::string_view, 4> kAxles{"ff", "fr", "rf",
                                                   "rr"};
    constexpr std::array<std::string_view, 8> kWheels{
        "ff_l", "ff_r", "fr_l", "fr_r", "rf_l", "rf_r", "rr_l",
        "rr_r"};
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "sample_index\ttime_nanoseconds\ttime_seconds";
    WriteArrayHeader(&output, "station.", kAxles);
    WriteArrayHeader(&output, "lateral.", kAxles);
    WriteArrayHeader(&output, "source_body_yaw.", kAxles);
    WriteArrayHeader(&output, "patch_count.", kWheels);
    WriteArrayHeader(&output, "Q.", kWheels);
    WriteArrayHeader(&output, "N.", kWheels);
    for (const std::string_view wheel : kWheels) {
        output << '\t' << "force_x." << wheel << '\t' << "force_y." << wheel
               << '\t' << "force_z." << wheel;
    }
    output << '\n';
    for (const auto& observation : observations) {
        output << observation.sample_index << '\t'
               << observation.time_nanoseconds << '\t'
               << observation.time_seconds;
        WriteArray(&output, observation.track_stations_meters);
        WriteArray(&output, observation.lateral_displacements_meters);
        WriteArray(&output, observation.yaw_angles_radians);
        for (const std::size_t count : observation.contact_patch_counts) {
            output << '\t' << count;
        }
        WriteArray(&output, observation.vertical_support_forces_newtons);
        WriteArray(&output, observation.normal_forces_newtons);
        for (const Eigen::Vector3d& force :
             observation.total_forces_in_carrier_track_frame_newtons) {
            output << '\t' << force.x() << '\t' << force.y() << '\t'
                   << force.z();
        }
        output << '\n';
    }
    CloseChecked(&output, path);
}

void WritePatchObservations(
    const std::filesystem::path& path,
    const std::vector<ControlledPatchObservation>& observations,
    const configuration::AssembledVehicleSystem& assembled) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "sample_index\ttime_nanoseconds\ttime_seconds\tinterface_name"
              "\tpatch_ordinal\tnormal_force_newtons"
              "\tlongitudinal_force_on_wheel_in_contact_frame_newtons"
              "\tlateral_force_on_wheel_in_contact_frame_newtons"
              "\tcontact_frame_angle_radians"
              "\tcontact_point_in_carrier_track_frame_x_meters"
              "\tcontact_point_in_carrier_track_frame_y_meters"
              "\tcontact_point_in_carrier_track_frame_z_meters"
              "\tforce_on_wheel_in_carrier_track_frame_x_newtons"
              "\tforce_on_wheel_in_carrier_track_frame_y_newtons"
              "\tforce_on_wheel_in_carrier_track_frame_z_newtons\n";
    for (const auto& observation : observations) {
        const auto& patch = observation.patch;
        output << observation.sample_index << '\t'
               << observation.time_nanoseconds << '\t'
               << observation.time_seconds << '\t'
               << assembled.contact_force_plan()->interface_name(
                      static_cast<int>(observation.interface_ordinal))
               << '\t' << observation.patch_ordinal << '\t'
               << patch.normal_force_newtons << '\t'
               << patch.longitudinal_force_on_wheel_in_contact_frame_newtons
               << '\t'
               << patch.lateral_force_on_wheel_in_contact_frame_newtons
               << '\t' << patch.contact_frame_angle_radians << '\t'
               << patch.contact_point_in_carrier_track_frame_meters.x()
               << '\t'
               << patch.contact_point_in_carrier_track_frame_meters.y()
               << '\t'
               << patch.contact_point_in_carrier_track_frame_meters.z()
               << '\t'
               << patch.force_on_wheel_in_carrier_track_frame_newtons.x()
               << '\t'
               << patch.force_on_wheel_in_carrier_track_frame_newtons.y()
               << '\t'
               << patch.force_on_wheel_in_carrier_track_frame_newtons.z()
               << '\n';
    }
    CloseChecked(&output, path);
}

void WriteEndpointDiagnostics(
    const std::filesystem::path& path,
    const std::vector<ControlledEndpointDiagnostics>& diagnostics) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "held_torque_event_ordinal\ttime_seconds"
              "\tstate_derivative_inf_norm"
              "\tgeneralized_force_residual_inf_norm"
              "\tvirtual_power_residual_watts\n";
    for (const auto& entry : diagnostics) {
        output << entry.held_torque_event_ordinal << '\t'
               << entry.time_seconds << '\t'
               << entry.state_derivative_inf_norm << '\t'
               << entry.generalized_force_residual_inf_norm << '\t'
               << entry.virtual_power_residual_watts << '\n';
    }
    CloseChecked(&output, path);
}

void WriteMetadata(
    const std::filesystem::path& path,
    const ResolvedConfiguration& configuration,
    const configuration::ResolvedStartupState& startup,
    const configuration::AssembledVehicleSystem& assembled,
    const configuration::IrwFullStateControlEventSession& session,
    const IrwP179ControlledQualificationSummary& summary,
    int contact_batch_parallel_team_probe_worker_count,
    std::string_view controller_identifier,
    std::string_view conditioner_identifier) {
    constexpr auto& recipe = internal::kIrwP179ControlledIntegrationRecipe;
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"completed\": true,\n"
           << "  \"qualification_vehicle_recipe\": "
           << JsonString("IRW_P179_100HZ_CONTROLLED") << ",\n"
           << "  \"vehicle_name\": "
           << JsonString(startup.vehicle_binding.vehicle_name) << ",\n"
           << "  \"track_irregularity_identifier\": "
           << JsonString(kIrregularityIdentifier) << ",\n"
           << "  \"controller_identifier\": "
           << JsonString(controller_identifier) << ",\n"
           << "  \"torque_conditioner_identifier\": "
           << JsonString(conditioner_identifier) << ",\n"
           << "  \"control_event_period_seconds\": "
           << session.sample_period_seconds() << ",\n"
           << "  \"mechanical_observation_period_nanoseconds\": "
           << kObservationPeriodNanoseconds << ",\n"
           << "  \"mechanical_observation_time_rule\": "
           << JsonString(
                  "integer 500,000-nanosecond clock; adjacent control "
                  "interval boundaries are published once")
           << ",\n"
           << "  \"contact_observation_contract\": {"
           << "\"interface_totals_frame\": "
           << JsonString("carrier-projection Track-T axes")
           << ", \"patch_local_components\": "
           << JsonString(
                  "one row per returned patch; local N/Tx/Ty are not "
                  "summed across contact frames")
           << ", \"maximum_returned_patch_count\": "
           << wheel_rail_contact::kMaxContactPatches << "},\n"
           << "  \"event_time_rule\": "
           << JsonString(
                  "integer event ordinal multiplied by control event period")
           << ",\n"
           << "  \"contact_projection_search_half_width_meters\": "
           << kProjectionSearchHalfWidthMeters << ",\n"
           << "  \"control_observation_basis\": "
           << JsonString(
                  "lateral displacement in Track-T; yaw in the frozen P179 "
                  "source physical axle-bridge body basis")
           << ",\n"
           << "  \"startup_rule\": "
           << JsonString(
                  "one full-period initialization recurrence followed by U0 "
                  "at the same accepted H3 state before backend construction")
           << ",\n"
           << "  \"terminal_event_rule\": "
           << JsonString("audit and commit without backend reinitialization")
           << ",\n"
           << "  \"duration_nanoseconds\": "
           << configuration.duration_nanoseconds << ",\n"
           << "  \"observation_count\": " << summary.observation_count
           << ",\n"
           << "  \"control_audit_count\": " << summary.control_audit_count
           << ",\n"
           << "  \"positive_hold_interval_count\": "
           << summary.positive_hold_interval_count << ",\n"
           << "  \"backend_synchronization_count\": "
           << summary.backend_synchronization_count << ",\n"
           << "  \"assembled_state_and_force_layout\": {"
           << "\"generalized_position_count\": "
           << assembled.model().num_generalized_positions()
           << ", \"generalized_velocity_count\": "
           << assembled.model().num_generalized_velocities()
           << ", \"series_force_state_count\": "
           << assembled.force_plan().series_spring_damper_force_state_count()
           << ", \"vehicle_body_wrench_count\": "
           << assembled.force_plan().body_wrench_count()
           << ", \"contact_body_wrench_count\": "
           << assembled.contact_force_plan()->body_wrench_count()
           << ", \"active_torque_body_wrench_count\": "
           << assembled.active_torque_plan()->body_wrench_count() << "},\n"
           << "  \"numerical_execution_contract\": {\n"
           << "    \"maximum_bdf_order\": "
           << summary.maximum_bdf_order << ",\n"
           << "    \"relative_tolerance\": "
           << recipe.relative_tolerance << ",\n"
           << "    \"generalized_position_absolute_tolerance\": "
           << recipe.position_absolute_tolerance << ",\n"
           << "    \"generalized_velocity_absolute_tolerance\": "
           << recipe.velocity_absolute_tolerance << ",\n"
           << "    \"series_force_absolute_tolerance_newtons\": "
           << recipe.series_force_absolute_tolerance_newtons << ",\n"
           << "    \"contact_batch_parallel_team_probe_worker_count\": "
           << contact_batch_parallel_team_probe_worker_count << "\n"
           << "  },\n"
           << "  \"input_paths\": {\n"
           << "    \"vehicle_definition\": "
           << JsonString(configuration.vehicle_definition_path.string())
           << ",\n"
           << "    \"resolved_startup_state\": "
           << JsonString(
                  configuration.resolved_startup_state_path.string())
           << ",\n"
           << "    \"track_geometry\": "
           << JsonString(configuration.track_geometry_path.string())
           << ",\n"
           << "    \"orvd_data_root\": "
           << JsonString(configuration.orvd_data_root.string()) << ",\n"
           << "    \"controller_configuration\": "
           << JsonString(
                  configuration.controller_configuration_path.string())
           << ",\n"
           << "    \"torque_conditioner_configuration\": "
           << JsonString(configuration.torque_conditioner_configuration_path
                             .string())
           << "\n  }\n}\n";
    CloseChecked(&output, path);
}

void WritePerformance(
    const std::filesystem::path& path,
    const IrwP179ControlledQualificationSummary& summary,
    std::int64_t duration_nanoseconds, std::size_t dense_state_peak_bytes,
    std::size_t observation_buffer_bytes,
    std::size_t patch_observation_buffer_bytes,
    std::size_t endpoint_diagnostics_buffer_bytes) {
    const auto& stats = summary.integration_statistics;
    const double simulated_duration_seconds =
        static_cast<double>(duration_nanoseconds) * 1.0e-9;
    const double integrated_dynamics_wall_seconds =
        summary.advance_and_synchronization_wall_seconds +
        summary.control_wall_seconds;
    const double qualification_compute_wall_seconds =
        integrated_dynamics_wall_seconds + summary.observation_wall_seconds;
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"advance_and_synchronization_wall_seconds\": "
           << summary.advance_and_synchronization_wall_seconds << ",\n"
           << "  \"control_wall_seconds\": "
           << summary.control_wall_seconds << ",\n"
           << "  \"observation_wall_seconds\": "
           << summary.observation_wall_seconds << ",\n"
           << "  \"data_and_metadata_write_wall_seconds\": "
           << summary.data_and_metadata_write_wall_seconds << ",\n"
           << "  \"simulated_duration_seconds\": "
           << simulated_duration_seconds << ",\n"
           << "  \"integrated_dynamics_wall_seconds\": "
           << integrated_dynamics_wall_seconds
           << ",\n"
           << "  \"qualification_compute_wall_seconds\": "
           << qualification_compute_wall_seconds
           << ",\n"
           << "  \"integrated_dynamics_realtime_factor\": "
           << simulated_duration_seconds / integrated_dynamics_wall_seconds
           << ",\n"
           << "  \"qualification_compute_realtime_factor\": "
           << simulated_duration_seconds / qualification_compute_wall_seconds
           << ",\n"
           << "  \"dense_state_peak_bytes\": " << dense_state_peak_bytes
           << ",\n"
           << "  \"observation_buffer_bytes\": "
           << observation_buffer_bytes << ",\n"
           << "  \"patch_observation_buffer_bytes\": "
           << patch_observation_buffer_bytes << ",\n"
           << "  \"endpoint_diagnostics_buffer_bytes\": "
           << endpoint_diagnostics_buffer_bytes << ",\n"
           << "  \"maximum_generalized_force_residual_inf_norm\": "
           << summary.maximum_generalized_force_residual_inf_norm << ",\n"
           << "  \"maximum_absolute_virtual_power_residual_watts\": "
           << summary.maximum_absolute_virtual_power_residual_watts << ",\n"
           << "  \"integration_statistics\": {"
           << "\"successful_internal_step_count\": "
           << stats.successful_internal_step_count
           << ", \"right_hand_side_evaluation_count\": "
           << stats.right_hand_side_evaluation_count
           << ", \"linear_solver_right_hand_side_evaluation_count\": "
           << stats.linear_solver_right_hand_side_evaluation_count
           << ", \"error_test_failure_count\": "
           << stats.error_test_failure_count
           << ", \"nonlinear_solver_iteration_count\": "
           << stats.nonlinear_solver_iteration_count
           << ", \"nonlinear_solver_convergence_failure_count\": "
           << stats.nonlinear_solver_convergence_failure_count
           << ", \"linear_solver_setup_count\": "
           << stats.linear_solver_setup_count
           << ", \"jacobian_evaluation_count\": "
           << stats.jacobian_evaluation_count
           << ", \"requested_dense_finite_difference_jacobian_worker_count\": "
           << stats.requested_dense_finite_difference_jacobian_worker_count
           << "}\n}\n";
    CloseChecked(&output, path);
}

}  // namespace

IrwP179ControlledQualificationSummary RunIrwP179ControlledQualification(
    const IrwP179ControlledQualificationRunConfiguration& input) {
    const ResolvedConfiguration resolved = ResolveConfiguration(input);
    const int contact_batch_parallel_team_probe_worker_count =
        internal::RequireRealContactBatchParallelTeam();
    AtomicQualificationDirectory output_directory(
        resolved.output_directory);
    const auto vehicle = configuration::LoadVehicleDefinitionFromJsonFile(
        resolved.vehicle_definition_path);
    const auto startup =
        configuration::LoadResolvedStartupStateFromJsonFile(
            resolved.resolved_startup_state_path);
    auto line = configuration::LoadTrackGeometryFromJsonFile(
        resolved.track_geometry_path);
    auto irregularity =
        std::make_unique<wheel_rail_contact::TrackIrregularityField>(
            configuration::LoadTrackIrregularityFieldFromDataRoot(
                resolved.orvd_data_root,
                std::string(kIrregularityIdentifier)));
    auto scenario = configuration::AssembleIrwContactScenario(
        vehicle, startup, std::move(line), resolved.orvd_data_root,
        kVehicleReferenceTrackStationMeters,
        kProjectionSearchHalfWidthMeters, std::move(irregularity));
    auto& assembled = scenario->vehicle_system();
    auto& accepted = scenario->initial_context().context();
    if (assembled.model().num_generalized_positions() != 81 ||
        assembled.model().num_generalized_velocities() != 74 ||
        assembled.force_plan().series_spring_damper_force_state_count() != 2 ||
        assembled.force_plan().body_wrench_count() != 96 ||
        assembled.contact_force_plan() == nullptr ||
        assembled.contact_force_plan()->body_wrench_count() != 8 ||
        assembled.active_torque_plan() == nullptr ||
        assembled.active_torque_plan()->body_wrench_count() != 16 ||
        accepted.time_seconds() != 0.0) {
        Reject("the assembled H3 IRW state or 96+8+16 wrench topology is "
               "not the frozen controlled recipe");
    }

    auto controller = configuration::
        LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile(
            resolved.controller_configuration_path);
    const std::string controller_identifier = controller.config().identifier;
    auto conditioner =
        configuration::LoadWheelDriveTorqueCommandConditionerFromJsonFile(
            resolved.torque_conditioner_configuration_path);
    const std::string conditioner_identifier =
        conditioner.config().identifier;
    configuration::IrwFullStateControlEventSession event_session(
        assembled, std::move(controller), std::move(conditioner));

    const std::uint64_t terminal_event_ordinal =
        static_cast<std::uint64_t>(resolved.duration_nanoseconds /
                                   kEventPeriodNanoseconds);
    const QualificationSampleClock observation_clock(
        static_cast<std::uint64_t>(resolved.duration_nanoseconds),
        static_cast<std::uint64_t>(kObservationPeriodNanoseconds));
    std::vector<double> observation_times_seconds =
        observation_clock.MakeSampleTimesSeconds();
    if (observation_clock.sample_count() !=
        static_cast<std::size_t>(terminal_event_ordinal *
                                 kObservationsPerEventPeriod +
                                 1U)) {
        Reject("the 0.5 ms mechanical clock does not tile the 10 ms event "
               "clock");
    }
    for (std::uint64_t ordinal = 0; ordinal <= terminal_event_ordinal;
         ++ordinal) {
        observation_times_seconds[static_cast<std::size_t>(
            ordinal * kObservationsPerEventPeriod)] =
            static_cast<double>(ordinal) *
            event_session.sample_period_seconds();
    }
    std::vector<IrwFullStateControlEventAudit> audits;
    audits.reserve(static_cast<std::size_t>(terminal_event_ordinal + 2));
    std::vector<ControlledObservation> observations;
    observations.reserve(observation_clock.sample_count());
    std::vector<ControlledPatchObservation> patch_observations;
    patch_observations.reserve(observation_clock.sample_count() *
                               kWheelCount);
    std::vector<ControlledEndpointDiagnostics> endpoint_diagnostics;
    endpoint_diagnostics.reserve(
        static_cast<std::size_t>(terminal_event_ordinal + 1));

    IrwP179ControlledQualificationSummary summary;
    const Clock::time_point startup_control_begin = Clock::now();
    audits.push_back(event_session.ApplyInitializationUpdate(accepted));
    audits.push_back(event_session.ApplyPeriodicUpdate(accepted));
    summary.control_wall_seconds +=
        ElapsedSeconds(startup_control_begin, Clock::now());
    integrators::SystemContinuousStateAdvancer advancer(
        assembled.system(), assembled.compiled_plan(), accepted,
        MakeTolerances(assembled), integrators::NoCallTimeAppliedForces{});
    summary.maximum_bdf_order =
        integrators::internal::BdfIntegrationAccess::
            ConfiguredMaximumBdfOrder(advancer);
    if (summary.maximum_bdf_order !=
        integrators::internal::MaximumBdfOrderValue(
            internal::kIrwP179ControlledIntegrationRecipe
                .maximum_bdf_order)) {
        Reject("the constructed integrator does not match the controlled "
               "qualification recipe's maximum BDF order");
    }
    event_session.ConfirmBackendSynchronized();

    auto contact_workspace = assembled.contact_force_plan()->CreateWorkspace();
    std::vector<AppliedBodyWrench> vehicle_wrenches(
        static_cast<std::size_t>(assembled.force_plan().body_wrench_count()));
    std::vector<AppliedBodyWrench> contact_wrenches(
        static_cast<std::size_t>(
            assembled.contact_force_plan()->body_wrench_count()));
    std::vector<AppliedBodyWrench> active_wrenches(
        static_cast<std::size_t>(
            assembled.active_torque_plan()->body_wrench_count()));
    Eigen::VectorXd series_derivatives(
        assembled.force_plan().series_spring_damper_force_state_count());
    std::array<WheelRailContactInterfaceObservation, kWheelCount>
        contact_observations{};
    auto observation_context =
        assembled.system().CreateDefaultRuntimeContext(0.0);
    assembled.system().CopyContextLocalData(accepted, *observation_context);
    Eigen::VectorXd initial_state(assembled.system().continuous_state_size());
    assembled.system().CopyContinuousState(accepted, initial_state);
    assembled.system().SetTimeContinuousStateAndWheelRailProjectionHints(
        *observation_context, 0.0, initial_state,
        accepted.wheel_rail_projection_station_hints_meters());
    const Clock::time_point initial_observation_begin = Clock::now();
    observations.push_back(ObserveControlledSample(
        assembled, event_session, *observation_context, 0, 0, 0.0,
        *contact_workspace, contact_wrenches, contact_observations,
        &patch_observations));
    endpoint_diagnostics.push_back(CalcControlledEndpointDiagnostics(
        assembled, *observation_context, 0, *contact_workspace,
        vehicle_wrenches, contact_wrenches, active_wrenches,
        &series_derivatives));
    summary.observation_wall_seconds +=
        ElapsedSeconds(initial_observation_begin, Clock::now());
    std::size_t dense_state_peak_bytes{};
    for (std::uint64_t ordinal = 1; ordinal <= terminal_event_ordinal;
         ++ordinal) {
        event_session.RequireReadyToAdvance();
        const std::size_t interval_begin = static_cast<std::size_t>(
            (ordinal - 1U) * kObservationsPerEventPeriod);
        const std::span<const double> interval_sample_times(
            observation_times_seconds.data() + interval_begin,
            static_cast<std::size_t>(kObservationsPerEventPeriod + 1U));
        const Clock::time_point advance_begin = Clock::now();
        const Eigen::MatrixXd dense_states =
            advancer.AdvanceToWithDenseStateSamples(
                event_session.next_periodic_event_time_seconds(),
                interval_sample_times);
        summary.advance_and_synchronization_wall_seconds +=
            ElapsedSeconds(advance_begin, Clock::now());
        dense_state_peak_bytes = std::max(
            dense_state_peak_bytes,
            static_cast<std::size_t>(dense_states.size()) * sizeof(double));
        AccumulateStatistics(advancer.integration_statistics(),
                             &summary.integration_statistics);
        const Clock::time_point observation_begin = Clock::now();
        for (std::uint64_t local_sample = 1;
             local_sample <= kObservationsPerEventPeriod; ++local_sample) {
            const std::uint64_t sample_index =
                (ordinal - 1U) * kObservationsPerEventPeriod + local_sample;
            const std::size_t global_index =
                static_cast<std::size_t>(sample_index);
            assembled.system().SetTimeAndContinuousState(
                *observation_context,
                observation_times_seconds[global_index],
                dense_states.col(static_cast<Eigen::Index>(local_sample)));
            assembled.system().UpdateWheelRailProjectionStationHints(
                *observation_context);
            observations.push_back(ObserveControlledSample(
                assembled, event_session, *observation_context,
                sample_index,
                observation_clock.TargetTimeNanoseconds(sample_index),
                observation_times_seconds[global_index], *contact_workspace,
                contact_wrenches, contact_observations,
                &patch_observations));
        }
        endpoint_diagnostics.push_back(CalcControlledEndpointDiagnostics(
            assembled, *observation_context, ordinal - 1U,
            *contact_workspace, vehicle_wrenches, contact_wrenches,
            active_wrenches, &series_derivatives));
        summary.observation_wall_seconds +=
            ElapsedSeconds(observation_begin, Clock::now());
        const Clock::time_point control_begin = Clock::now();
        audits.push_back(event_session.ApplyPeriodicUpdate(accepted));
        assembled.system().CopyContextLocalData(accepted,
                                                *observation_context);
        summary.control_wall_seconds +=
            ElapsedSeconds(control_begin, Clock::now());
        if (ordinal < terminal_event_ordinal) {
            const Clock::time_point synchronization_begin = Clock::now();
            advancer.SynchronizeAfterAcceptedContextChange();
            summary.advance_and_synchronization_wall_seconds +=
                ElapsedSeconds(synchronization_begin, Clock::now());
            event_session.ConfirmBackendSynchronized();
            ++summary.backend_synchronization_count;
        }
    }

    summary.observation_count = observations.size();
    summary.control_audit_count = audits.size();
    summary.positive_hold_interval_count =
        static_cast<std::size_t>(terminal_event_ordinal);
    summary.terminal_continuous_state.resize(
        assembled.system().continuous_state_size());
    assembled.system().CopyContinuousState(
        accepted, summary.terminal_continuous_state);
    for (const ControlledEndpointDiagnostics& diagnostics :
         endpoint_diagnostics) {
        summary.maximum_generalized_force_residual_inf_norm = std::max(
            summary.maximum_generalized_force_residual_inf_norm,
            diagnostics.generalized_force_residual_inf_norm);
        summary.maximum_absolute_virtual_power_residual_watts = std::max(
            summary.maximum_absolute_virtual_power_residual_watts,
            std::abs(diagnostics.virtual_power_residual_watts));
    }

    const Clock::time_point write_begin = Clock::now();
    WriteControlAudits(output_directory.working_path() / "control_events.tsv",
                       audits);
    WriteObservations(output_directory.working_path() / "observations.tsv",
                      observations);
    WritePatchObservations(
        output_directory.working_path() / "contact_patches.tsv",
        patch_observations, assembled);
    WriteEndpointDiagnostics(
        output_directory.working_path() / "endpoint_diagnostics.tsv",
        endpoint_diagnostics);
    WriteMetadata(output_directory.working_path() / "metadata.json",
                  resolved, startup, assembled, event_session, summary,
                  contact_batch_parallel_team_probe_worker_count,
                  controller_identifier, conditioner_identifier);
    const std::filesystem::path complete_path =
        output_directory.working_path() / "COMPLETE";
    std::ofstream complete(complete_path, std::ios::out | std::ios::trunc);
    if (!complete) {
        Reject("could not open '" + complete_path.string() + "'");
    }
    complete << summary.observation_count << " controlled observations\n";
    CloseChecked(&complete, complete_path);
    const Clock::time_point write_end = Clock::now();
    summary.data_and_metadata_write_wall_seconds =
        ElapsedSeconds(write_begin, write_end);
    WritePerformance(
        output_directory.working_path() / "performance.json", summary,
        resolved.duration_nanoseconds, dense_state_peak_bytes,
        observations.capacity() * sizeof(ControlledObservation),
        patch_observations.capacity() * sizeof(ControlledPatchObservation),
        endpoint_diagnostics.capacity() *
            sizeof(ControlledEndpointDiagnostics));
    output_directory.Publish();
    return summary;
}

}  // namespace orvd::dynamics_qualification
