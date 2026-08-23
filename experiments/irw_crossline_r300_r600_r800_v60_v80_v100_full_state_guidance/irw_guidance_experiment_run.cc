#include "irw_guidance_experiment_run.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Core>

#include "irw_guidance_control_event_session.h"
#include "irw_crossline_operating_point_schedule.h"
#include "irw_single_curve_full_state_guidance_schedule.h"
#include "irw_single_curve_normal_differential_wheel_speed_schedule.h"

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/irw_full_state_control_observation_binding.h"
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

namespace orvd::experiments::irw_crossline_full_state_guidance {
namespace {

using Clock = std::chrono::steady_clock;
using forces::WheelRailContactInterfaceObservation;
using multibody_model::AppliedBodyWrench;

constexpr std::uint64_t kCrosslineDurationNanoseconds =
    100'000'000'000ULL;
constexpr std::uint64_t kSingleCurveSafetyDurationNanoseconds =
    45'000'000'000ULL;
constexpr std::uint64_t kControlPeriodNanoseconds = 10'000'000ULL;
constexpr std::uint64_t kObservationPeriodNanoseconds = 500'000ULL;
constexpr std::size_t kSamplesPerControlInterval =
    static_cast<std::size_t>(kControlPeriodNanoseconds /
                             kObservationPeriodNanoseconds);
constexpr std::size_t kAxleCount = control::kIrwGuidanceAxleCount;
constexpr std::size_t kWheelCount = control::kIrwGuidanceWheelCount;
constexpr double kVehicleReferenceTrackStationMeters = 0.0;
constexpr double kMaximumScheduledSpeedMetersPerSecond = 30.0;
constexpr double kProjectionSearchHalfWidthMeters =
    kMaximumScheduledSpeedMetersPerSecond * kControlSamplePeriodSeconds;
constexpr double kRelativeTolerance = 1.0e-6;
constexpr double kPositionAbsoluteTolerance = 1.0e-6;
constexpr double kVelocityAbsoluteTolerance = 1.0e-5;
constexpr double kSeriesForceAbsoluteToleranceNewtons = 1.0e-6;
constexpr std::string_view kIrregularityIdentifier = "aar5_irregularity";
constexpr std::string_view kConditionerIdentifier =
    "irw_reference_wheel_drive_torque_conditioner";

constexpr std::array<std::string_view, kAxleCount> kAxleNames{
    "axle_1", "axle_2", "axle_3", "axle_4"};
constexpr std::array<std::string_view, kWheelCount> kWheelNames{
    "wheel_1", "wheel_2", "wheel_3", "wheel_4",
    "wheel_5", "wheel_6", "wheel_7", "wheel_8"};

static_assert(kCrosslineDurationNanoseconds % kControlPeriodNanoseconds == 0);
static_assert(kSingleCurveSafetyDurationNanoseconds %
                  kControlPeriodNanoseconds ==
              0);
static_assert(kControlPeriodNanoseconds % kObservationPeriodNanoseconds == 0);
static_assert(kProjectionSearchHalfWidthMeters == 0.30);

struct ResolvedInputs final {
    std::filesystem::path data_root;
    std::filesystem::path vehicle_definition;
    std::filesystem::path startup_state;
    std::filesystem::path track_geometry;
    std::filesystem::path torque_conditioner;
    std::filesystem::path output_directory;
};

struct IrwGuidanceExperimentDefinition final {
    std::string_view experiment_name;
    std::string_view control_profile_identity;
    double initial_longitudinal_speed_meters_per_second{};
    std::string_view track_geometry_relative_path;
    std::string_view track_geometry_label;
    std::string_view schedule_scope;
    std::uint64_t maximum_duration_nanoseconds{};
    std::optional<double> terminal_minimum_axle_station_meters;
    control::IrwFullStateWheelSpeedGuidanceRecurrenceConfig recurrence_config;
    IrwOperatingPointEvaluator operating_point_evaluator;
};

struct EndpointDiagnostics final {
    std::uint64_t held_torque_event_ordinal{};
    double time_seconds{};
    double state_derivative_inf_norm{};
    double generalized_force_residual_inf_norm{};
    double virtual_power_residual_watts{};
};

class AtomicOutputDirectory final {
   public:
    explicit AtomicOutputDirectory(std::filesystem::path final_path)
        : final_path_(std::filesystem::absolute(std::move(final_path))
                          .lexically_normal()) {
        if (final_path_.filename().empty() ||
            final_path_.filename() == "." ||
            final_path_.filename() == "..") {
            throw std::invalid_argument(
                "IRW guidance output directory must have a file name");
        }
        std::error_code error;
        if (std::filesystem::exists(final_path_, error) || error) {
            throw std::invalid_argument(
                "IRW guidance output directory already exists or cannot be "
                "examined: " +
                final_path_.string());
        }
        const auto parent = final_path_.parent_path();
        if (!std::filesystem::is_directory(parent, error) || error) {
            throw std::invalid_argument(
                "IRW guidance output parent is not an existing directory: " +
                parent.string());
        }
        working_path_ =
            parent / (final_path_.filename().string() + ".partial");
        if (std::filesystem::exists(working_path_, error) || error) {
            throw std::invalid_argument(
                "IRW guidance partial output already exists or cannot be "
                "examined: " +
                working_path_.string());
        }
        if (!std::filesystem::create_directory(working_path_, error) || error) {
            throw std::runtime_error(
                "could not create IRW guidance partial output directory: " +
                error.message());
        }
        owns_working_path_ = true;
    }

    ~AtomicOutputDirectory() {
        if (!published_ && owns_working_path_) {
            std::error_code ignored;
            std::filesystem::remove_all(working_path_, ignored);
        }
    }

    AtomicOutputDirectory(const AtomicOutputDirectory&) = delete;
    AtomicOutputDirectory& operator=(const AtomicOutputDirectory&) = delete;

    [[nodiscard]] const std::filesystem::path& working_path() const noexcept {
        return working_path_;
    }

    void Publish() {
        if (published_ || !owns_working_path_) {
            throw std::logic_error(
                "IRW guidance output directory was already published");
        }
        std::error_code error;
        std::filesystem::rename(working_path_, final_path_, error);
        if (error) {
            throw std::runtime_error(
                "could not atomically publish IRW guidance output: " +
                error.message());
        }
        owns_working_path_ = false;
        published_ = true;
    }

   private:
    std::filesystem::path final_path_;
    std::filesystem::path working_path_;
    bool owns_working_path_{false};
    bool published_{false};
};

[[noreturn]] void Reject(const std::string& detail) {
    throw std::runtime_error("IRW guidance experiment: " + detail);
}

[[nodiscard]] double ElapsedSeconds(Clock::time_point begin,
                                    Clock::time_point end) {
    return std::chrono::duration<double>(end - begin).count();
}

[[nodiscard]] std::filesystem::path CanonicalExisting(
    const std::filesystem::path& path, std::string_view label) {
    std::error_code error;
    const auto canonical = std::filesystem::canonical(path, error);
    if (error) {
        Reject("could not resolve " + std::string(label) + " '" +
               path.string() + "': " + error.message());
    }
    return canonical;
}

[[nodiscard]] ResolvedInputs ResolveInputs(
    const std::filesystem::path& data_root,
    const std::filesystem::path& output_directory,
    const IrwGuidanceExperimentDefinition& definition) {
    const auto root = CanonicalExisting(data_root, "ORVD data root");
    if (!std::filesystem::is_directory(root)) {
        Reject("ORVD data root is not a directory");
    }
    return ResolvedInputs{
        .data_root = root,
        .vehicle_definition = CanonicalExisting(
            root / "vehicle_library/irw/vehicle_definition.json",
            "IRW vehicle definition"),
        .startup_state = CanonicalExisting(
            root /
                "vehicle_library/irw/startup_states/moving_startup_60kmh.json",
            "IRW 60 km/h startup state"),
        .track_geometry = CanonicalExisting(
            root / definition.track_geometry_relative_path,
            definition.track_geometry_label),
        .torque_conditioner = CanonicalExisting(
            root /
                "vehicle_library/irw/drive_torque_conditioners/"
                "irw_reference_wheel_drive_torque_conditioner.json",
            "IRW torque conditioner"),
        .output_directory = output_directory,
    };
}

void FlushAndClose(std::ofstream* stream,
                   const std::filesystem::path& path) {
    stream->flush();
    if (!*stream) {
        Reject("could not flush '" + path.string() + "'");
    }
    stream->close();
    if (!*stream) {
        Reject("could not close '" + path.string() + "'");
    }
}

[[nodiscard]] std::ofstream OpenOutput(const std::filesystem::path& path) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17);
    return output;
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

template <typename Value, std::size_t Size>
void WriteJsonArray(std::ofstream* output,
                    const std::array<Value, Size>& values) {
    *output << '[';
    for (std::size_t index = 0; index < Size; ++index) {
        if (index != 0) {
            *output << ", ";
        }
        *output << values[index];
    }
    *output << ']';
}

template <std::size_t Size>
void WriteNamedHeader(std::ofstream* output, std::string_view prefix,
                      const std::array<std::string_view, Size>& names) {
    for (const auto name : names) {
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

template <std::size_t Size>
void RequireFiniteArray(const std::array<double, Size>& values,
                        std::string_view label) {
    if (!std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); })) {
        Reject(std::string(label) + " contains a non-finite value");
    }
}

void WriteObservationHeader(std::ofstream* output) {
    *output << "sample_index\ttime_nanoseconds\ttime_seconds";
    for (const auto axle : kAxleNames) {
        *output << '\t' << axle << "_track_station_meters";
    }
    for (const auto axle : kAxleNames) {
        *output << '\t' << axle
                << "_longitudinal_speed_meters_per_second";
    }
    for (const auto axle : kAxleNames) {
        *output << '\t' << axle << "_lateral_displacement_meters";
    }
    for (const auto axle : kAxleNames) {
        *output << '\t' << axle << "_yaw_angle_radians";
    }
    for (const auto wheel : kWheelNames) {
        *output << '\t' << wheel
                << "_frozen_angular_speed_radians_per_second";
    }
    for (const auto wheel : kWheelNames) {
        *output << '\t' << wheel
                << "_actual_drive_torque_newton_metres";
    }
    for (const auto wheel : kWheelNames) {
        *output << '\t' << wheel << "_contact_patch_count";
    }
    for (const auto wheel : kWheelNames) {
        *output << '\t' << wheel << "_vertical_support_force_newtons";
    }
    for (const auto wheel : kWheelNames) {
        *output << '\t' << wheel << "_normal_force_newtons";
    }
    for (const auto wheel : kWheelNames) {
        *output << '\t' << wheel << "_track_force_x_newtons" << '\t'
                << wheel << "_track_force_y_newtons" << '\t' << wheel
                << "_track_force_z_newtons";
    }
    *output << '\n';
}

void WritePatchHeader(std::ofstream* output) {
    *output
        << "sample_index\ttime_nanoseconds\ttime_seconds"
           "\twheel_index\tinterface_name\tpatch_ordinal"
           "\tnormal_force_newtons"
           "\tlongitudinal_force_on_wheel_in_contact_frame_newtons"
           "\tlateral_force_on_wheel_in_contact_frame_newtons"
           "\tlongitudinal_creepage\tlateral_creepage"
           "\tcontact_frame_angle_radians"
           "\tcontact_point_in_carrier_track_frame_x_meters"
           "\tcontact_point_in_carrier_track_frame_y_meters"
           "\tcontact_point_in_carrier_track_frame_z_meters"
           "\tforce_on_wheel_in_carrier_track_frame_x_newtons"
           "\tforce_on_wheel_in_carrier_track_frame_y_newtons"
           "\tforce_on_wheel_in_carrier_track_frame_z_newtons\n";
}

void WriteEndpointHeader(std::ofstream* output) {
    *output << "held_torque_event_ordinal\ttime_seconds"
               "\tstate_derivative_inf_norm"
               "\tgeneralized_force_residual_inf_norm"
               "\tvirtual_power_residual_watts\n";
}

void WriteControllerStateHeader(std::ofstream* output,
                                std::string_view prefix) {
    *output << '\t' << prefix << "initialized";
    for (const std::string_view family :
         {std::string_view("previous_lateral_displacement."),
          std::string_view("previous_yaw_angle."),
          std::string_view("filtered_lateral_velocity."),
          std::string_view("filtered_yaw_rate."),
          std::string_view("lateral_error_integral."),
          std::string_view("filtered_delta_omega_command.")}) {
        WriteNamedHeader(output,
                         std::string(prefix) + std::string(family),
                         kAxleNames);
    }
    WriteNamedHeader(output, std::string(prefix) + "pi_integral.",
                     kWheelNames);
    WriteNamedHeader(output, std::string(prefix) + "pi_filtered_torque.",
                     kWheelNames);
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

void WriteControlEventHeader(std::ofstream* output) {
    *output << "event_kind\tperiodic_event_ordinal\tevent_time_seconds";
    WriteNamedHeader(output, "input.station.", kAxleNames);
    WriteNamedHeader(output, "input.lateral.", kAxleNames);
    WriteNamedHeader(output, "input.yaw.", kAxleNames);
    WriteNamedHeader(output, "input.frozen_wheel_speed.", kWheelNames);
    WriteNamedHeader(output, "operating_point.base_speed_reference.",
                     kWheelNames);
    WriteNamedHeader(output, "operating_point.feedforward_gain.",
                     kAxleNames);
    WriteNamedHeader(output, "operating_point.yaw_rate_feedback_gain.",
                     kAxleNames);
    WriteNamedHeader(output, "operating_point.yaw_feedback_gain.",
                     kAxleNames);
    WriteNamedHeader(output, "operating_point.lateral_velocity_gain.",
                     kAxleNames);
    WriteNamedHeader(output, "operating_point.lateral_displacement_gain.",
                     kAxleNames);
    WriteNamedHeader(output, "operating_point.lateral_integral_gain.",
                     kAxleNames);
    WriteNamedHeader(output, "operating_point.delta_omega_feedback_gain.",
                     kAxleNames);
    WriteNamedHeader(output, "controller.speed_reference.", kWheelNames);
    WriteNamedHeader(output, "controller.delta_omega_reference.",
                     kAxleNames);
    WriteNamedHeader(output, "controller.delta_omega_measured.",
                     kAxleNames);
    WriteNamedHeader(output, "controller.delta_omega_equilibrium.",
                     kAxleNames);
    WriteNamedHeader(output, "controller.filtered_lateral_velocity.",
                     kAxleNames);
    WriteNamedHeader(output, "controller.filtered_yaw_rate.", kAxleNames);
    WriteNamedHeader(output, "controller.guidance_active.", kAxleNames);
    WriteNamedHeader(output, "controller.raw_torque_request.", kWheelNames);
    WriteNamedHeader(output, "conditioner.common_probe_request.",
                     kWheelNames);
    WriteNamedHeader(output, "conditioner.common_probe_dynamic_limit.",
                     kWheelNames);
    WriteNamedHeader(output, "conditioner.prioritized_request.",
                     kWheelNames);
    WriteNamedHeader(output, "conditioner.actual_torque.", kWheelNames);
    WriteNamedHeader(output, "conditioner.dynamic_limit.", kWheelNames);
    WriteNamedHeader(output, "conditioner.limit_flag.", kWheelNames);
    WriteNamedHeader(output, "conditioner.drive_speed_rpm.", kWheelNames);
    WriteNamedHeader(output, "conditioner.memory_before.", kWheelNames);
    WriteNamedHeader(output, "conditioner.memory_after.", kWheelNames);
    WriteControllerStateHeader(output, "controller_state_before.");
    WriteControllerStateHeader(output, "controller_state_after.");
    *output << '\n';
}

void WriteControlEvent(std::ofstream* output,
                       const IrwGuidanceControlEventAudit& audit) {
    std::string_view event_kind;
    switch (audit.kind) {
        case IrwGuidanceControlEventKind::kInitialization:
            event_kind = "initialization";
            break;
        case IrwGuidanceControlEventKind::kPeriodic:
            event_kind = "periodic";
            break;
        case IrwGuidanceControlEventKind::kTerminal:
            event_kind = "terminal";
            break;
    }
    *output << event_kind << '\t' << audit.periodic_event_ordinal << '\t'
            << audit.event_time_seconds;
    WriteArray(output,
               audit.mechanical_observation.axle_track_stations_meters);
    WriteArray(output,
               audit.mechanical_observation.mechanical_input
                   .axle_lateral_displacements_meters);
    WriteArray(output,
               audit.mechanical_observation.mechanical_input
                   .axle_yaw_angles_radians);
    WriteArray(
        output,
        audit.mechanical_observation.mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second);
    WriteArray(output,
               audit.operating_point
                   .base_wheel_speed_references_meters_per_second);
    WriteArray(output, audit.operating_point.feedforward_gains);
    WriteArray(output, audit.operating_point.yaw_rate_feedback_gains);
    WriteArray(output, audit.operating_point.yaw_feedback_gains);
    WriteArray(output,
               audit.operating_point.lateral_velocity_feedback_gains);
    WriteArray(output,
               audit.operating_point.lateral_displacement_feedback_gains);
    WriteArray(output,
               audit.operating_point.lateral_integral_feedback_gains);
    WriteArray(output,
               audit.operating_point
                   .wheel_speed_difference_feedback_gains);
    WriteArray(output,
               audit.controller_result.observations
                   .wheel_speed_references_meters_per_second);
    WriteArray(output,
               audit.controller_result.observations
                   .wheel_speed_difference_references_radians_per_second);
    WriteArray(output,
               audit.controller_result.observations
                   .measured_wheel_speed_differences_radians_per_second);
    WriteArray(output,
               audit.controller_result.observations
                   .equilibrium_wheel_speed_differences_radians_per_second);
    WriteArray(output,
               audit.controller_result.observations
                   .filtered_lateral_velocities_meters_per_second);
    WriteArray(output,
               audit.controller_result.observations
                   .filtered_yaw_rates_radians_per_second);
    for (const bool active :
         audit.controller_result.observations.guidance_active) {
        *output << '\t' << (active ? 1 : 0);
    }
    WriteArray(
        output,
        audit.controller_result.requested_wheel_torques_newton_metres);
    WriteArray(output, audit.common_mode_probe_requests_newton_metres);
    WriteArray(output,
               audit.common_mode_conditioning_probe
                   .wheel_dynamic_torque_limits_newton_metres);
    WriteArray(output,
               audit.prioritized_wheel_torque_requests_newton_metres);
    WriteArray(output,
               audit.conditioning_result.actual_wheel_torques_newton_metres);
    WriteArray(output,
               audit.conditioning_result
                   .wheel_dynamic_torque_limits_newton_metres);
    for (const auto flag : audit.conditioning_result.limit_flags) {
        *output << '\t' << static_cast<std::uint16_t>(flag);
    }
    WriteArray(output,
               audit.conditioning_result
                   .equivalent_drive_side_speeds_revolutions_per_minute);
    WriteArray(output, audit.conditioner_memory_before_newton_metres);
    WriteArray(output,
               audit.conditioning_result
                   .next_drive_side_torque_memory_newton_metres);
    WriteControllerState(output, audit.controller_state_before);
    WriteControllerState(output, audit.controller_result.next_state);
    *output << '\n';
}

[[nodiscard]] integrators::ContinuousStateErrorTolerances MakeTolerances(
    const configuration::AssembledVehicleSystem& assembled) {
    const auto& system = assembled.system();
    Eigen::VectorXd absolute = Eigen::VectorXd::Constant(
        system.continuous_state_size(), kVelocityAbsoluteTolerance);
    const auto q = system.generalized_positions_state_range();
    const auto z = system.series_spring_damper_force_state_range();
    absolute.segment(q.start(), q.size())
        .setConstant(kPositionAbsoluteTolerance);
    absolute.segment(z.start(), z.size())
        .setConstant(kSeriesForceAbsoluteToleranceNewtons);
    return integrators::ContinuousStateErrorTolerances(
        kRelativeTolerance, std::move(absolute));
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
    const int workers =
        source.requested_dense_finite_difference_jacobian_worker_count;
    if (destination
            ->requested_dense_finite_difference_jacobian_worker_count == 0) {
        destination
            ->requested_dense_finite_difference_jacobian_worker_count =
            workers;
    } else if (destination
                   ->requested_dense_finite_difference_jacobian_worker_count !=
               workers) {
        Reject("dense-Jacobian worker count changed between control "
               "intervals");
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
    const Eigen::Vector3d offset =
        pose.rotation() * wrench.point_position_in_body_frame_meters;
    const Eigen::Vector3d point_velocity =
        velocity.translational_velocity_at_frame_origin_meters_per_second() +
        velocity.angular_velocity_radians_per_second().cross(offset);
    return wrench.torque_about_point_newton_metres.dot(
               velocity.angular_velocity_radians_per_second()) +
           wrench.force_newtons.dot(point_velocity);
}

[[nodiscard]] EndpointDiagnostics CalcEndpointDiagnostics(
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
            context.held_independent_wheel_active_torques_newton_metres()
                .data(),
            static_cast<std::size_t>(
                context.held_independent_wheel_active_torques_newton_metres()
                    .size())),
        active_wrenches);

    Eigen::VectorXd rhs(assembled.system().continuous_state_size());
    assembled.compiled_plan().CalcStateTimeDerivatives(context, rhs);
    if (!rhs.allFinite()) {
        Reject("endpoint produced a non-finite continuous-state RHS");
    }

    const int nq = assembled.model().num_generalized_positions();
    const int nv = assembled.model().num_generalized_velocities();
    Eigen::VectorXd projected_generalized_force = Eigen::VectorXd::Zero(nv);
    Eigen::MatrixXd angular_jacobian(3, nv);
    Eigen::MatrixXd translational_jacobian(3, nv);
    double spatial_power = 0.0;
    const auto accumulate = [&](std::span<const AppliedBodyWrench> wrenches) {
        for (const auto& wrench : wrenches) {
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
    accumulate(vehicle_wrenches);
    accumulate(contact_wrenches);
    accumulate(active_wrenches);
    Eigen::VectorXd required_generalized_force(nv);
    assembled.model().CalcRequiredGeneralizedForces(
        component.context(), rhs.segment(nq, nv),
        required_generalized_force);

    EndpointDiagnostics result{
        .held_torque_event_ordinal = held_torque_event_ordinal,
        .time_seconds = context.time_seconds(),
        .state_derivative_inf_norm = rhs.lpNorm<Eigen::Infinity>(),
        .generalized_force_residual_inf_norm =
            (projected_generalized_force - required_generalized_force)
                .lpNorm<Eigen::Infinity>(),
        .virtual_power_residual_watts =
            projected_generalized_force.dot(
                context.generalized_velocities()) -
            spatial_power,
    };
    if (!std::isfinite(result.state_derivative_inf_norm) ||
        !std::isfinite(result.generalized_force_residual_inf_norm) ||
        !std::isfinite(result.virtual_power_residual_watts)) {
        Reject("endpoint diagnostics contain a non-finite value");
    }
    return result;
}

void WriteEndpoint(std::ofstream* output,
                   const EndpointDiagnostics& diagnostics) {
    *output << diagnostics.held_torque_event_ordinal << '\t'
            << diagnostics.time_seconds << '\t'
            << diagnostics.state_derivative_inf_norm << '\t'
            << diagnostics.generalized_force_residual_inf_norm << '\t'
            << diagnostics.virtual_power_residual_watts << '\n';
}

struct ObservationWriterState final {
    bool has_previous_station{false};
    control::IrwGuidanceAxleValues previous_stations_meters{};
    std::size_t patch_observation_count{};
};

void ObserveAndWrite(
    const configuration::AssembledVehicleSystem& assembled,
    const configuration::IrwFullStateControlObservationBinding& binding,
    system_assembly::SystemRuntimeContext& context,
    std::uint64_t sample_index, std::uint64_t time_nanoseconds,
    double initial_longitudinal_speed_meters_per_second,
    forces::WheelRailContactForceWorkspace& contact_workspace,
    std::span<AppliedBodyWrench> contact_wrenches,
    std::span<WheelRailContactInterfaceObservation> contact_observations,
    std::ofstream* observation_output, std::ofstream* patch_output,
    ObservationWriterState* writer_state) {
    const double time_seconds =
        static_cast<double>(time_nanoseconds) * 1.0e-9;
    auto component = assembled.system().GetMultibodyComponentView(
        context, assembled.system().multibody_component());
    assembled.contact_force_plan()->CalcAppliedForcesAndObservations(
        component.context(), contact_workspace,
        context.wheel_rail_projection_station_hints_meters(),
        contact_wrenches, contact_observations);
    const auto mechanical = binding.Observe(context);

    control::IrwGuidanceAxleValues longitudinal_velocities{};
    if (!writer_state->has_previous_station) {
        longitudinal_velocities.fill(
            initial_longitudinal_speed_meters_per_second);
    } else {
        constexpr double kObservationPeriodSeconds =
            static_cast<double>(kObservationPeriodNanoseconds) * 1.0e-9;
        for (std::size_t axle = 0; axle < kAxleCount; ++axle) {
            longitudinal_velocities[axle] =
                (mechanical.axle_track_stations_meters[axle] -
                 writer_state->previous_stations_meters[axle]) /
                kObservationPeriodSeconds;
        }
    }
    writer_state->has_previous_station = true;
    writer_state->previous_stations_meters =
        mechanical.axle_track_stations_meters;
    RequireFiniteArray(mechanical.axle_track_stations_meters,
                       "axle stations");
    RequireFiniteArray(longitudinal_velocities,
                       "axle longitudinal velocities");
    RequireFiniteArray(
        mechanical.mechanical_input.axle_lateral_displacements_meters,
        "axle lateral displacements");
    RequireFiniteArray(mechanical.mechanical_input.axle_yaw_angles_radians,
                       "axle yaw angles");
    RequireFiniteArray(
        mechanical.mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second,
        "wheel speeds");

    std::array<double, kWheelCount> held_torques{};
    const auto& held =
        context.held_independent_wheel_active_torques_newton_metres();
    if (held.size() != static_cast<Eigen::Index>(kWheelCount)) {
        Reject("accepted held-torque vector is not eight channels");
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        held_torques[wheel] = held[static_cast<Eigen::Index>(wheel)];
    }
    RequireFiniteArray(held_torques, "held wheel torques");

    *observation_output << sample_index << '\t' << time_nanoseconds << '\t'
                        << time_seconds;
    WriteArray(observation_output, mechanical.axle_track_stations_meters);
    WriteArray(observation_output, longitudinal_velocities);
    WriteArray(observation_output,
               mechanical.mechanical_input
                   .axle_lateral_displacements_meters);
    WriteArray(observation_output,
               mechanical.mechanical_input.axle_yaw_angles_radians);
    WriteArray(
        observation_output,
        mechanical.mechanical_input
            .wheel_angular_speeds_in_frozen_scalar_convention_radians_per_second);
    WriteArray(observation_output, held_torques);
    for (const auto& contact : contact_observations) {
        *observation_output << '\t' << contact.contact_patch_count;
    }
    for (const auto& contact : contact_observations) {
        *observation_output << '\t'
                            << contact.vertical_support_force_on_wheel_newtons;
    }
    for (const auto& contact : contact_observations) {
        *observation_output << '\t' << contact.normal_force_newtons;
    }
    for (const auto& contact : contact_observations) {
        if (!std::isfinite(contact.vertical_support_force_on_wheel_newtons) ||
            !std::isfinite(contact.normal_force_newtons) ||
            !contact.total_force_on_wheel_in_carrier_track_frame_newtons
                 .allFinite()) {
            Reject("contact summary contains a non-finite value");
        }
        const auto& force =
            contact.total_force_on_wheel_in_carrier_track_frame_newtons;
        *observation_output << '\t' << force.x() << '\t' << force.y()
                            << '\t' << force.z();
    }
    *observation_output << '\n';

    for (std::size_t interface = 0; interface < kWheelCount; ++interface) {
        const auto& contact = contact_observations[interface];
        for (std::size_t patch_ordinal = 0;
             patch_ordinal < contact.contact_patch_count; ++patch_ordinal) {
            const auto& patch = contact.patches[patch_ordinal];
            if (!std::isfinite(patch.normal_force_newtons) ||
                !std::isfinite(
                    patch.longitudinal_force_on_wheel_in_contact_frame_newtons) ||
                !std::isfinite(
                    patch.lateral_force_on_wheel_in_contact_frame_newtons) ||
                !std::isfinite(patch.longitudinal_creepage) ||
                !std::isfinite(patch.lateral_creepage) ||
                !std::isfinite(patch.contact_frame_angle_radians) ||
                !patch.contact_point_in_carrier_track_frame_meters.allFinite() ||
                !patch.force_on_wheel_in_carrier_track_frame_newtons
                     .allFinite()) {
                Reject("contact patch observation contains a non-finite "
                       "value");
            }
            *patch_output
                << sample_index << '\t' << time_nanoseconds << '\t'
                << time_seconds << '\t' << interface << '\t'
                << assembled.contact_force_plan()->interface_name(
                       static_cast<int>(interface))
                << '\t' << patch_ordinal << '\t'
                << patch.normal_force_newtons << '\t'
                << patch.longitudinal_force_on_wheel_in_contact_frame_newtons
                << '\t'
                << patch.lateral_force_on_wheel_in_contact_frame_newtons
                << '\t' << patch.longitudinal_creepage << '\t'
                << patch.lateral_creepage << '\t'
                << patch.contact_frame_angle_radians << '\t'
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
            ++writer_state->patch_observation_count;
        }
    }
}

void WritePerformance(
    const std::filesystem::path& path,
    const IrwGuidanceExperimentRunSummary& summary,
    const integrators::ContinuousStateIntegrationStatistics& statistics,
    std::size_t dense_state_peak_bytes,
    double maximum_generalized_force_residual_inf_norm,
    double maximum_absolute_virtual_power_residual_watts) {
    auto output = OpenOutput(path);
    const double integrated_wall_seconds =
        summary.advance_and_synchronization_wall_seconds +
        summary.control_wall_seconds;
    const double total_compute_wall_seconds =
        integrated_wall_seconds +
        summary.observation_and_streaming_wall_seconds;
    output << "{\n"
           << "  \"simulated_duration_seconds\": "
           << summary.simulated_duration_seconds
           << ",\n"
           << "  \"advance_and_synchronization_wall_seconds\": "
           << summary.advance_and_synchronization_wall_seconds << ",\n"
           << "  \"control_wall_seconds\": "
           << summary.control_wall_seconds << ",\n"
           << "  \"observation_and_streaming_wall_seconds\": "
           << summary.observation_and_streaming_wall_seconds << ",\n"
           << "  \"finalization_wall_seconds\": "
           << summary.finalization_wall_seconds << ",\n"
           << "  \"integrated_dynamics_realtime_factor\": "
           << summary.simulated_duration_seconds / integrated_wall_seconds
           << ",\n"
           << "  \"complete_compute_realtime_factor\": "
           << summary.simulated_duration_seconds / total_compute_wall_seconds
           << ",\n"
           << "  \"dense_state_peak_bytes\": " << dense_state_peak_bytes
           << ",\n"
           << "  \"maximum_generalized_force_residual_inf_norm\": "
           << maximum_generalized_force_residual_inf_norm << ",\n"
           << "  \"maximum_absolute_virtual_power_residual_watts\": "
           << maximum_absolute_virtual_power_residual_watts << ",\n"
           << "  \"integration_statistics\": {\n"
           << "    \"successful_internal_step_count\": "
           << statistics.successful_internal_step_count << ",\n"
           << "    \"right_hand_side_evaluation_count\": "
           << statistics.right_hand_side_evaluation_count << ",\n"
           << "    \"linear_solver_right_hand_side_evaluation_count\": "
           << statistics.linear_solver_right_hand_side_evaluation_count
           << ",\n"
           << "    \"error_test_failure_count\": "
           << statistics.error_test_failure_count << ",\n"
           << "    \"nonlinear_solver_iteration_count\": "
           << statistics.nonlinear_solver_iteration_count << ",\n"
           << "    \"nonlinear_solver_convergence_failure_count\": "
           << statistics.nonlinear_solver_convergence_failure_count
           << ",\n"
           << "    \"linear_solver_setup_count\": "
           << statistics.linear_solver_setup_count << ",\n"
           << "    \"jacobian_evaluation_count\": "
           << statistics.jacobian_evaluation_count << ",\n"
           << "    \"requested_dense_finite_difference_jacobian_worker_count\": "
           << statistics
                  .requested_dense_finite_difference_jacobian_worker_count
           << "\n  }\n}\n";
    FlushAndClose(&output, path);
}

void WriteMetadata(
    const std::filesystem::path& path,
    const configuration::ResolvedStartupState& startup,
    const configuration::AssembledVehicleSystem& assembled,
    const IrwGuidanceExperimentDefinition& definition,
    const IrwGuidanceExperimentRunSummary& summary) {
    auto output = OpenOutput(path);
    output
        << "{\n"
        << "  \"completed\": true,\n"
        << "  \"experiment\": "
        << JsonString(definition.experiment_name)
        << ",\n"
        << "  \"control_profile_identity\": "
        << JsonString(definition.control_profile_identity)
        << ",\n"
        << "  \"vehicle_name\": "
        << JsonString(startup.vehicle_binding.vehicle_name) << ",\n"
        << "  \"initial_longitudinal_speed_meters_per_second\": "
        << startup.initial_longitudinal_speed_meters_per_second << ",\n"
        << "  \"startup_derivation\": "
        << JsonString(
               "bundled resolved V60 state with only longitudinal speed and eight explicit independent-wheel rates scaled by their common speed ratio")
        << ",\n"
        << "  \"duration_seconds\": " << summary.simulated_duration_seconds
        << ",\n"
        << "  \"control_period_nanoseconds\": "
        << kControlPeriodNanoseconds << ",\n"
        << "  \"mechanical_observation_period_nanoseconds\": "
        << kObservationPeriodNanoseconds << ",\n"
        << "  \"projection_search_half_width_meters\": "
        << kProjectionSearchHalfWidthMeters << ",\n"
        << "  \"projection_search_half_width_derivation\": "
        << JsonString("30 m/s * 0.01 s") << ",\n"
        << "  \"track_irregularity_identifier\": "
        << JsonString(kIrregularityIdentifier) << ",\n"
        << "  \"track_irregularity_scope\": "
        << JsonString(
               "current public ORVD AAR5 contract; not the historical random field of the source cross-line project")
        << ",\n"
        << "  \"schedule_scope\": "
        << JsonString(definition.schedule_scope)
        << ",\n"
        << "  \"termination_contract\": {\n"
        << "    \"maximum_duration_seconds\": "
        << static_cast<double>(definition.maximum_duration_nanoseconds) *
               1.0e-9
        << ",\n"
        << "    \"terminal_condition\": "
        << JsonString(definition.terminal_minimum_axle_station_meters
                          ? "first control boundary at which every axle has reached the minimum station"
                          : "fixed maximum duration")
        << ",\n"
        << "    \"minimum_all_axle_station_meters\": ";
    if (definition.terminal_minimum_axle_station_meters) {
        output << *definition.terminal_minimum_axle_station_meters;
    } else {
        output << "null";
    }
    output
        << "\n  },\n"
        << "  \"wheel_speed_recurrence\": {\n"
        << "    \"sample_period_seconds\": "
        << definition.recurrence_config.sample_period_seconds << ",\n"
        << "    \"rolling_radius_meters\": "
        << definition.recurrence_config.rolling_radius_meters << ",\n"
        << "    \"proportional_gain\": "
        << definition.recurrence_config.wheel_speed_pi.proportional_gain
        << ",\n"
        << "    \"integral_time_seconds\": "
        << definition.recurrence_config.wheel_speed_pi.integral_time_seconds
        << ",\n"
        << "    \"output_filter_time_constant_seconds\": "
        << definition.recurrence_config.wheel_speed_pi
               .output_filter_time_constant_seconds
        << ",\n"
        << "    \"integral_absolute_limit\": "
        << definition.recurrence_config.wheel_speed_pi
               .integral_absolute_limit
        << ",\n"
        << "    \"raw_output_absolute_limit\": "
        << definition.recurrence_config.wheel_speed_pi
               .raw_output_absolute_limit
        << ",\n"
        << "    \"lateral_integral_absolute_limit_meter_seconds\": "
        << definition.recurrence_config
               .lateral_integral_absolute_limit_meter_seconds
        << ",\n"
        << "    \"guidance_axle_signs\": ";
    WriteJsonArray(&output, definition.recurrence_config.guidance_axle_signs);
    output << ",\n    \"guidance_wheel_signs\": ";
    WriteJsonArray(&output, definition.recurrence_config.guidance_wheel_signs);
    output << ",\n    \"wheel_speed_pi_wheel_signs\": ";
    WriteJsonArray(&output,
                   definition.recurrence_config.wheel_speed_pi_wheel_signs);
    output
        << "\n  },\n"
        << "  \"contact_interface_ordinal_base\": 0,\n"
        << "  \"longitudinal_velocity_observation\": "
        << JsonString(
               "backward difference of each axle projection station on the 0.5 ms clock; initial sample uses the resolved startup longitudinal speed")
        << ",\n"
        << "  \"counts\": {\n"
        << "    \"observations\": " << summary.observation_count << ",\n"
        << "    \"contact_patch_observations\": "
        << summary.contact_patch_observation_count << ",\n"
        << "    \"control_events\": " << summary.control_event_count
        << ",\n"
        << "    \"backend_synchronizations\": "
        << summary.backend_synchronization_count << "\n  },\n"
        << "  \"final_axle_track_stations_meters\": ["
        << summary.final_axle_track_stations_meters[0] << ", "
        << summary.final_axle_track_stations_meters[1] << ", "
        << summary.final_axle_track_stations_meters[2] << ", "
        << summary.final_axle_track_stations_meters[3] << "],\n"
        << "  \"assembled_topology\": {\n"
        << "    \"generalized_positions\": "
        << assembled.model().num_generalized_positions() << ",\n"
        << "    \"generalized_velocities\": "
        << assembled.model().num_generalized_velocities() << ",\n"
        << "    \"vehicle_wrenches\": "
        << assembled.force_plan().body_wrench_count() << ",\n"
        << "    \"contact_wrenches\": "
        << assembled.contact_force_plan()->body_wrench_count() << ",\n"
        << "    \"active_torque_wrenches\": "
        << assembled.active_torque_plan()->body_wrench_count() << "\n  },\n"
        << "  \"numerical_execution_contract\": {\n"
        << "    \"maximum_bdf_order\": 2,\n"
        << "    \"relative_tolerance\": " << kRelativeTolerance << ",\n"
        << "    \"generalized_position_absolute_tolerance\": "
        << kPositionAbsoluteTolerance << ",\n"
        << "    \"generalized_velocity_absolute_tolerance\": "
        << kVelocityAbsoluteTolerance << ",\n"
        << "    \"series_force_absolute_tolerance_newtons\": "
        << kSeriesForceAbsoluteToleranceNewtons << "\n  },\n"
        << "  \"input_assets\": {\n"
        << "    \"vehicle_definition\": "
        << JsonString("vehicle_library/irw/vehicle_definition.json")
        << ",\n"
        << "    \"resolved_startup_state\": "
        << JsonString(
               "vehicle_library/irw/startup_states/moving_startup_60kmh.json")
        << ",\n"
        << "    \"track_geometry\": "
        << JsonString(definition.track_geometry_relative_path)
        << ",\n"
        << "    \"torque_conditioner\": "
        << JsonString(
               "vehicle_library/irw/drive_torque_conditioners/irw_reference_wheel_drive_torque_conditioner.json")
        << "\n  }\n}\n";
    FlushAndClose(&output, path);
}

}  // namespace

namespace {

IrwGuidanceExperimentRunSummary RunIrwGuidanceExperiment(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory,
    const IrwGuidanceExperimentDefinition& definition) {
    if (definition.maximum_duration_nanoseconds == 0 ||
        definition.maximum_duration_nanoseconds %
                kControlPeriodNanoseconds !=
            0 ||
        !std::isfinite(
            definition.initial_longitudinal_speed_meters_per_second) ||
        !(definition.initial_longitudinal_speed_meters_per_second > 0.0) ||
        definition.control_profile_identity.empty() ||
        !definition.operating_point_evaluator) {
        Reject("the run definition has an invalid duration or empty "
               "operating-point evaluator");
    }
    const ResolvedInputs inputs =
        ResolveInputs(orvd_data_root, output_directory, definition);
    AtomicOutputDirectory output(inputs.output_directory);

    const auto vehicle = configuration::LoadVehicleDefinitionFromJsonFile(
        inputs.vehicle_definition);
    auto startup = configuration::LoadResolvedStartupStateFromJsonFile(
        inputs.startup_state);
    constexpr double kSourceSpeedMetersPerSecond = 60.0 / 3.6;
    if (startup.initial_longitudinal_speed_meters_per_second !=
            kSourceSpeedMetersPerSecond ||
        startup.common_wheel_spin_generation.has_value() ||
        startup.revolute_joint_startup_states.size() != kWheelCount) {
        Reject("resolved startup state is not the frozen 60 km/h identity");
    }
    const double startup_speed_scale =
        definition.initial_longitudinal_speed_meters_per_second /
        kSourceSpeedMetersPerSecond;
    for (auto& joint_state : startup.revolute_joint_startup_states) {
        auto* explicit_rate =
            std::get_if<configuration::ExplicitRevoluteJointRate>(
                &joint_state.rate);
        if (explicit_rate == nullptr ||
            !std::isfinite(explicit_rate->angular_rate_radians_per_second)) {
            Reject("resolved IRW startup does not contain eight finite "
                   "explicit independent-wheel rates");
        }
        explicit_rate->angular_rate_radians_per_second *= startup_speed_scale;
    }
    startup.initial_longitudinal_speed_meters_per_second =
        definition.initial_longitudinal_speed_meters_per_second;
    auto line = configuration::LoadTrackGeometryFromJsonFile(
        inputs.track_geometry);
    auto irregularity =
        std::make_unique<wheel_rail_contact::TrackIrregularityField>(
            configuration::LoadTrackIrregularityFieldFromDataRoot(
                inputs.data_root, kIrregularityIdentifier));
    auto scenario = configuration::AssembleIrwContactScenario(
        vehicle, startup, std::move(line), inputs.data_root,
        kVehicleReferenceTrackStationMeters,
        kProjectionSearchHalfWidthMeters, std::move(irregularity));
    const auto& assembled = scenario->vehicle_system();
    auto& accepted = scenario->initial_context().context();
    if (assembled.model().num_generalized_positions() != 81 ||
        assembled.model().num_generalized_velocities() != 74 ||
        assembled.force_plan().series_spring_damper_force_state_count() != 2 ||
        assembled.force_plan().body_wrench_count() != 96 ||
        assembled.contact_force_plan() == nullptr ||
        assembled.contact_force_plan()->interface_count() !=
            static_cast<int>(kWheelCount) ||
        assembled.active_torque_plan() == nullptr ||
        assembled.active_torque_plan()->channel_count() !=
            static_cast<int>(kWheelCount) ||
        accepted.time_seconds() != 0.0) {
        Reject("assembled IRW startup or 96+8+16 wrench topology differs "
               "from the experiment identity");
    }

    auto conditioner =
        configuration::LoadWheelDriveTorqueCommandConditionerFromJsonFile(
            inputs.torque_conditioner);
    if (conditioner.config().identifier != kConditionerIdentifier) {
        Reject("loaded torque conditioner has the wrong identity");
    }
    IrwGuidanceControlEventSession event_session(
        assembled, definition.recurrence_config,
        definition.operating_point_evaluator,
        std::move(conditioner));
    configuration::IrwFullStateControlObservationBinding observation_binding(
        assembled);

    auto observations_path = output.working_path() / "observations.tsv";
    auto patches_path = output.working_path() / "contact_patches.tsv";
    auto events_path = output.working_path() / "control_events.tsv";
    auto diagnostics_path =
        output.working_path() / "endpoint_diagnostics.tsv";
    auto observation_stream = OpenOutput(observations_path);
    auto patch_stream = OpenOutput(patches_path);
    auto event_stream = OpenOutput(events_path);
    auto diagnostics_stream = OpenOutput(diagnostics_path);
    WriteObservationHeader(&observation_stream);
    WritePatchHeader(&patch_stream);
    WriteControlEventHeader(&event_stream);
    WriteEndpointHeader(&diagnostics_stream);

    IrwGuidanceExperimentRunSummary summary;
    const Clock::time_point initialization_begin = Clock::now();
    const auto initialization_audit =
        event_session.ApplyInitializationUpdate(accepted);
    WriteControlEvent(&event_stream, initialization_audit);
    ++summary.control_event_count;
    summary.control_wall_seconds +=
        ElapsedSeconds(initialization_begin, Clock::now());

    integrators::SystemContinuousStateAdvancer advancer(
        assembled.system(), assembled.compiled_plan(), accepted,
        MakeTolerances(assembled), integrators::NoCallTimeAppliedForces{});
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

    ObservationWriterState writer_state;
    double maximum_generalized_force_residual_inf_norm = 0.0;
    double maximum_absolute_virtual_power_residual_watts = 0.0;
    const Clock::time_point initial_observation_begin = Clock::now();
    ObserveAndWrite(
        assembled, observation_binding, *observation_context, 0, 0,
        startup.initial_longitudinal_speed_meters_per_second,
        *contact_workspace, contact_wrenches, contact_observations,
        &observation_stream, &patch_stream, &writer_state);
    ++summary.observation_count;
    const auto initial_diagnostics = CalcEndpointDiagnostics(
        assembled, *observation_context, 0, *contact_workspace,
        vehicle_wrenches, contact_wrenches, active_wrenches,
        &series_derivatives);
    WriteEndpoint(&diagnostics_stream, initial_diagnostics);
    maximum_generalized_force_residual_inf_norm =
        initial_diagnostics.generalized_force_residual_inf_norm;
    maximum_absolute_virtual_power_residual_watts =
        std::abs(initial_diagnostics.virtual_power_residual_watts);
    summary.observation_and_streaming_wall_seconds +=
        ElapsedSeconds(initial_observation_begin, Clock::now());

    integrators::ContinuousStateIntegrationStatistics integration_statistics;
    std::size_t dense_state_peak_bytes = 0;
    std::array<double, kSamplesPerControlInterval + 1U> sample_times{};
    const std::uint64_t maximum_control_event_ordinal =
        definition.maximum_duration_nanoseconds / kControlPeriodNanoseconds;
    for (std::uint64_t ordinal = 1;
         ordinal <= maximum_control_event_ordinal; ++ordinal) {
        event_session.RequireReadyToAdvance();
        sample_times.front() = accepted.time_seconds();
        for (std::size_t local_sample = 1;
             local_sample <= kSamplesPerControlInterval; ++local_sample) {
            const std::uint64_t sample_index =
                (ordinal - 1U) * kSamplesPerControlInterval + local_sample;
            sample_times[local_sample] =
                static_cast<double>(sample_index *
                                    kObservationPeriodNanoseconds) *
                1.0e-9;
        }
        sample_times.back() = event_session.next_periodic_event_time_seconds();

        const Clock::time_point advance_begin = Clock::now();
        const Eigen::MatrixXd dense_states =
            advancer.AdvanceToWithDenseStateSamples(sample_times.back(),
                                                    sample_times);
        summary.advance_and_synchronization_wall_seconds +=
            ElapsedSeconds(advance_begin, Clock::now());
        dense_state_peak_bytes =
            std::max(dense_state_peak_bytes,
                     static_cast<std::size_t>(dense_states.size()) *
                         sizeof(double));
        AccumulateStatistics(advancer.integration_statistics(),
                             &integration_statistics);

        const Clock::time_point observation_begin = Clock::now();
        for (std::size_t local_sample = 1;
             local_sample <= kSamplesPerControlInterval; ++local_sample) {
            const std::uint64_t sample_index =
                (ordinal - 1U) * kSamplesPerControlInterval + local_sample;
            const std::uint64_t time_nanoseconds =
                sample_index * kObservationPeriodNanoseconds;
            assembled.system().SetTimeAndContinuousState(
                *observation_context, sample_times[local_sample],
                dense_states.col(static_cast<Eigen::Index>(local_sample)));
            assembled.system().UpdateWheelRailProjectionStationHints(
                *observation_context);
            ObserveAndWrite(
                assembled, observation_binding, *observation_context,
                sample_index, time_nanoseconds,
                startup.initial_longitudinal_speed_meters_per_second,
                *contact_workspace, contact_wrenches, contact_observations,
                &observation_stream, &patch_stream, &writer_state);
            ++summary.observation_count;
        }
        const auto diagnostics = CalcEndpointDiagnostics(
            assembled, *observation_context, ordinal - 1U,
            *contact_workspace, vehicle_wrenches, contact_wrenches,
            active_wrenches, &series_derivatives);
        WriteEndpoint(&diagnostics_stream, diagnostics);
        maximum_generalized_force_residual_inf_norm =
            std::max(maximum_generalized_force_residual_inf_norm,
                     diagnostics.generalized_force_residual_inf_norm);
        maximum_absolute_virtual_power_residual_watts =
            std::max(maximum_absolute_virtual_power_residual_watts,
                     std::abs(diagnostics.virtual_power_residual_watts));
        summary.observation_and_streaming_wall_seconds +=
            ElapsedSeconds(observation_begin, Clock::now());

        const auto terminal_observation =
            event_session.ObserveMechanicalInput(accepted);
        const bool terminal_station_reached =
            definition.terminal_minimum_axle_station_meters.has_value() &&
            std::all_of(
                terminal_observation.axle_track_stations_meters.begin(),
                terminal_observation.axle_track_stations_meters.end(),
                [&](double station) {
                    return station >=
                           *definition.terminal_minimum_axle_station_meters;
                });
        const bool maximum_duration_reached =
            ordinal == maximum_control_event_ordinal;
        if (maximum_duration_reached &&
            definition.terminal_minimum_axle_station_meters.has_value() &&
            !terminal_station_reached) {
            Reject("the safety duration elapsed before every axle reached "
                   "the terminal station");
        }
        const bool terminal_event = terminal_station_reached ||
                                    maximum_duration_reached;
        const Clock::time_point control_begin = Clock::now();
        const auto audit = terminal_event
                               ? event_session.ApplyTerminalUpdate(accepted)
                               : event_session.ApplyPeriodicUpdate(accepted);
        WriteControlEvent(&event_stream, audit);
        ++summary.control_event_count;
        assembled.system().CopyContextLocalData(accepted,
                                                *observation_context);
        summary.control_wall_seconds +=
            ElapsedSeconds(control_begin, Clock::now());
        if (!terminal_event) {
            const Clock::time_point synchronization_begin = Clock::now();
            advancer.SynchronizeAfterAcceptedContextChange();
            summary.advance_and_synchronization_wall_seconds +=
                ElapsedSeconds(synchronization_begin, Clock::now());
            event_session.ConfirmBackendSynchronized();
            ++summary.backend_synchronization_count;
        } else {
            summary.simulated_duration_seconds = audit.event_time_seconds;
            summary.final_axle_track_stations_meters =
                audit.mechanical_observation.axle_track_stations_meters;
            break;
        }
    }
    if (!event_session.terminal_event_committed()) {
        Reject("terminal event was not committed");
    }
    summary.contact_patch_observation_count =
        writer_state.patch_observation_count;

    const Clock::time_point finalization_begin = Clock::now();
    FlushAndClose(&observation_stream, observations_path);
    FlushAndClose(&patch_stream, patches_path);
    FlushAndClose(&event_stream, events_path);
    FlushAndClose(&diagnostics_stream, diagnostics_path);
    WriteMetadata(output.working_path() / "metadata.json", startup, assembled,
                  definition, summary);
    summary.finalization_wall_seconds =
        ElapsedSeconds(finalization_begin, Clock::now());
    WritePerformance(output.working_path() / "performance.json", summary,
                     integration_statistics, dense_state_peak_bytes,
                     maximum_generalized_force_residual_inf_norm,
                     maximum_absolute_virtual_power_residual_watts);
    const auto complete_path = output.working_path() / "COMPLETE";
    auto complete = OpenOutput(complete_path);
    complete << summary.observation_count << " observations\n"
             << summary.contact_patch_observation_count
             << " contact patches\n"
             << summary.control_event_count << " control events\n";
    FlushAndClose(&complete, complete_path);
    output.Publish();
    return summary;
}

IrwGuidanceExperimentDefinition MakeCrosslineDefinition() {
    const IrwCrosslineOperatingPointSchedule schedule;
    return IrwGuidanceExperimentDefinition{
        .experiment_name =
            "IRW cross-line R300/R600/R800 V60/V80/V100 full-state guidance",
        .control_profile_identity = "better2_crossline_schedule",
        .initial_longitudinal_speed_meters_per_second = 60.0 / 3.6,
        .track_geometry_relative_path =
            "track_library/geometries/"
            "crossline_r300_r600_r800_superelevation_2396p9m.json",
        .track_geometry_label = "cross-line geometry",
        .schedule_scope =
            "track geometry and control schedule transcribed from the source cross-line project",
        .maximum_duration_nanoseconds = kCrosslineDurationNanoseconds,
        .terminal_minimum_axle_station_meters = std::nullopt,
        .recurrence_config = schedule.MakeRecurrenceConfig(),
        .operating_point_evaluator =
            [schedule](const control::IrwGuidanceAxleValues& stations) {
                return schedule.EvaluateOperatingPoint(stations);
            },
    };
}

IrwGuidanceExperimentDefinition MakeSingleCurveNormalDefinition(
    std::string_view experiment_name,
    std::string_view track_geometry_relative_path,
    std::string_view track_geometry_label,
    double initial_speed_meters_per_second,
    double curve_radius_meters) {
    const IrwSingleCurveNormalDifferentialWheelSpeedSchedule schedule({
        .base_speed_meters_per_second = initial_speed_meters_per_second,
        .curve_radius_meters = curve_radius_meters,
    });
    return IrwGuidanceExperimentDefinition{
        .experiment_name = experiment_name,
        .control_profile_identity =
            "normal_curvature_differential_wheel_speed",
        .initial_longitudinal_speed_meters_per_second =
            initial_speed_meters_per_second,
        .track_geometry_relative_path = track_geometry_relative_path,
        .track_geometry_label = track_geometry_label,
        .schedule_scope =
            "historical normal baseline: per-axle planned-curvature wheel-speed references and eight wheel-speed PI recurrences; all outer guidance gains are zero",
        .maximum_duration_nanoseconds =
            kSingleCurveSafetyDurationNanoseconds,
        .terminal_minimum_axle_station_meters = 600.0,
        .recurrence_config = schedule.MakeRecurrenceConfig(),
        .operating_point_evaluator =
            [schedule](const control::IrwGuidanceAxleValues& stations) {
                return schedule.EvaluateOperatingPoint(stations);
            },
    };
}

IrwGuidanceExperimentDefinition MakeR300NormalDefinition() {
    return MakeSingleCurveNormalDefinition(
        "IRW R300 V60 normal curvature-differential wheel-speed baseline",
        "track_library/geometries/"
        "r300_centerline_superelevation_1100m.json",
        "R300 centerline-superelevation geometry", 60.0 / 3.6, 300.0);
}

IrwGuidanceExperimentDefinition MakeR600NormalDefinition() {
    return MakeSingleCurveNormalDefinition(
        "IRW R600 V80 normal curvature-differential wheel-speed baseline",
        "track_library/geometries/"
        "r600_centerline_superelevation_1100m.json",
        "R600 centerline-superelevation geometry", 80.0 / 3.6, 600.0);
}

IrwGuidanceExperimentDefinition MakeR800NormalDefinition() {
    return MakeSingleCurveNormalDefinition(
        "IRW R800 V100 normal curvature-differential wheel-speed baseline",
        "track_library/geometries/"
        "r800_centerline_superelevation_1100m.json",
        "R800 centerline-superelevation geometry", 100.0 / 3.6, 800.0);
}

IrwGuidanceExperimentDefinition MakeSingleCurveFullStateGuidanceDefinition(
    std::string_view experiment_name,
    std::string_view track_geometry_relative_path,
    std::string_view track_geometry_label,
    double initial_speed_meters_per_second,
    double curve_radius_meters,
    IrwCurveFullStateGuidanceProfile profile) {
    const IrwSingleCurveFullStateGuidanceSchedule schedule({
        .base_speed_meters_per_second = initial_speed_meters_per_second,
        .curve_radius_meters = curve_radius_meters,
        .profile = profile,
    });
    return IrwGuidanceExperimentDefinition{
        .experiment_name = experiment_name,
        .control_profile_identity = profile.identity,
        .initial_longitudinal_speed_meters_per_second =
            initial_speed_meters_per_second,
        .track_geometry_relative_path = track_geometry_relative_path,
        .track_geometry_label = track_geometry_label,
        .schedule_scope =
            "isolated-curve full-state guidance profile: per-axle planned-curvature reference and gains ramp from zero over 50--100 m, then remain at the selected curve operating point",
        .maximum_duration_nanoseconds = kSingleCurveSafetyDurationNanoseconds,
        .terminal_minimum_axle_station_meters = 600.0,
        .recurrence_config = schedule.MakeRecurrenceConfig(),
        .operating_point_evaluator =
            [schedule](const control::IrwGuidanceAxleValues& stations) {
                return schedule.EvaluateOperatingPoint(stations);
            },
    };
}

IrwGuidanceExperimentDefinition MakeR300Better2Definition() {
    return MakeSingleCurveFullStateGuidanceDefinition(
        "IRW R300 V60 better2 full-state guidance",
        "track_library/geometries/"
        "r300_centerline_superelevation_1100m.json",
        "R300 centerline-superelevation geometry", 60.0 / 3.6, 300.0,
        kBetter2R300Profile);
}

IrwGuidanceExperimentDefinition MakeR600Better2Definition() {
    return MakeSingleCurveFullStateGuidanceDefinition(
        "IRW R600 V80 better2 full-state guidance",
        "track_library/geometries/"
        "r600_centerline_superelevation_1100m.json",
        "R600 centerline-superelevation geometry", 80.0 / 3.6, 600.0,
        kBetter2R600Profile);
}

IrwGuidanceExperimentDefinition MakeR800Better2Definition() {
    return MakeSingleCurveFullStateGuidanceDefinition(
        "IRW R800 V100 better2 full-state guidance",
        "track_library/geometries/"
        "r800_centerline_superelevation_1100m.json",
        "R800 centerline-superelevation geometry", 100.0 / 3.6, 800.0,
        kBetter2R800Profile);
}

IrwGuidanceExperimentDefinition MakeR300ScbpDefinition() {
    return MakeSingleCurveFullStateGuidanceDefinition(
        "IRW R300 V60 SCBP recorded wear champion full-state guidance",
        "track_library/geometries/"
        "r300_centerline_superelevation_1100m.json",
        "R300 centerline-superelevation geometry", 60.0 / 3.6, 300.0,
        kScbpR300Profile);
}

IrwGuidanceExperimentDefinition MakeR600ScbpDefinition() {
    return MakeSingleCurveFullStateGuidanceDefinition(
        "IRW R600 V80 SCBP recorded wear champion full-state guidance",
        "track_library/geometries/"
        "r600_centerline_superelevation_1100m.json",
        "R600 centerline-superelevation geometry", 80.0 / 3.6, 600.0,
        kScbpR600Profile);
}

IrwGuidanceExperimentDefinition MakeR800ScbpDefinition() {
    return MakeSingleCurveFullStateGuidanceDefinition(
        "IRW R800 V100 SCBP recorded wear champion full-state guidance",
        "track_library/geometries/"
        "r800_centerline_superelevation_1100m.json",
        "R800 centerline-superelevation geometry", 100.0 / 3.6, 800.0,
        kScbpR800Profile);
}

}  // namespace

IrwGuidanceExperimentRunSummary
RunIrwCrosslineR300R600R800V60V80V100FullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeCrosslineDefinition());
}

IrwGuidanceExperimentRunSummary
RunIrwR300Aar5V60NormalDifferentialWheelSpeed(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeR300NormalDefinition());
}

IrwGuidanceExperimentRunSummary
RunIrwR600Aar5V80NormalDifferentialWheelSpeed(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeR600NormalDefinition());
}

IrwGuidanceExperimentRunSummary
RunIrwR800Aar5V100NormalDifferentialWheelSpeed(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeR800NormalDefinition());
}

IrwGuidanceExperimentRunSummary
RunIrwR300Aar5V60Better2FullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeR300Better2Definition());
}

IrwGuidanceExperimentRunSummary
RunIrwR600Aar5V80Better2FullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeR600Better2Definition());
}

IrwGuidanceExperimentRunSummary
RunIrwR800Aar5V100Better2FullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeR800Better2Definition());
}

IrwGuidanceExperimentRunSummary
RunIrwR300Aar5V60ScbpRecordedWearChampionFullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeR300ScbpDefinition());
}

IrwGuidanceExperimentRunSummary
RunIrwR600Aar5V80ScbpRecordedWearChampionFullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeR600ScbpDefinition());
}

IrwGuidanceExperimentRunSummary
RunIrwR800Aar5V100ScbpRecordedWearChampionFullStateGuidance(
    const std::filesystem::path& orvd_data_root,
    const std::filesystem::path& output_directory) {
    return RunIrwGuidanceExperiment(orvd_data_root, output_directory,
                                    MakeR800ScbpDefinition());
}

}  // namespace orvd::experiments::irw_crossline_full_state_guidance
