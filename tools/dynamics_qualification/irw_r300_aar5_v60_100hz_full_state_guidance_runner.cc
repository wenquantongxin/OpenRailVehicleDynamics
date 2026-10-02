#include "irw_r300_aar5_v60_100hz_full_state_guidance_run.h"

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
#include "qualification_continuous_state_writer.h"
#include "qualification_integration_configuration.h"
#include "qualification_integration_run.h"
#include "irw_integration_recipes.h"
#include "qualification_sample_clock.h"
#include "strict_floating_point_qualification.h"
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
constexpr std::string_view kIrregularityIdentifier =
    "aar5_irregularity";
constexpr std::string_view kTrackGeometryFilename =
    "r300_centerline_superelevation_1100m.json";
constexpr std::string_view kControllerIdentifier =
    "irw_r300_v60_full_state_wheel_speed_guidance_controller";
constexpr std::string_view kTorqueConditionerIdentifier =
    "irw_reference_wheel_drive_torque_conditioner";
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
    std::array<double, kWheelCount> rail_profile_reference_marker_track_station_meters{};
    std::array<Eigen::Vector3d, kWheelCount>
        total_forces_in_carrier_track_frame_newtons{};
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
    throw std::runtime_error(
        "IRW R300/AAR5/60 km/h 100 Hz full-state guidance run: " + detail);
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
    const IrwR300Aar5V60At100HzFullStateGuidanceRunConfiguration& input) {
    if (input.duration_nanoseconds <= 0 ||
        input.duration_nanoseconds % kEventPeriodNanoseconds != 0) {
        throw std::invalid_argument(
            "IRW R300/AAR5/60 km/h 100 Hz guidance duration must be a "
            "positive "
            "integer multiple of 10,000,000 ns");
    }
    if (input.track_geometry_path.filename() != kTrackGeometryFilename) {
        throw std::invalid_argument(
            "IRW R300/AAR5/60 km/h 100 Hz guidance requires the closed R300 "
            "track geometry");
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
        observation.rail_profile_reference_marker_track_station_meters[wheel] =
            contact.rail_profile_reference_marker_track_station_meters;
        observation.total_forces_in_carrier_track_frame_newtons[wheel] =
            contact.total_force_on_wheel_in_carrier_track_frame_newtons;
        if (!std::isfinite(observation.vertical_support_forces_newtons[wheel]) ||
            !std::isfinite(observation.normal_forces_newtons[wheel]) ||
            !std::isfinite(observation.rail_profile_reference_marker_track_station_meters[wheel]) ||
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
    WriteArrayHeader(&output, "rail_profile_reference_marker_track_station_meters.", kWheels);
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
        WriteArray(&output, observation.rail_profile_reference_marker_track_station_meters);
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

[[nodiscard]] nlohmann::json PhysicalObservationContract(
    const configuration::AssembledVehicleSystem& assembled) {
    constexpr std::array<std::string_view, kAxleCount> kAxles{"ff", "fr", "rf", "rr"};
    constexpr std::array<std::string_view, kWheelCount> kWheels{
        "ff_l", "ff_r", "fr_l", "fr_r", "rf_l", "rf_r", "rr_l", "rr_r"};
    nlohmann::json carriers = nlohmann::json::array();
    nlohmann::json interfaces = nlohmann::json::array();
    for (std::size_t i = 0; i < kAxles.size(); ++i) {
        const std::string suffix(kAxles[i]);
        carriers.push_back({
            {"name", assembled.contact_force_plan()->carrier_name(static_cast<int>(i))},
            {"station_column", "station." + suffix},
            {"lateral_column", "lateral." + suffix},
            {"yaw_column", "source_body_yaw." + suffix},
            {"yaw_basis", "physical_axle_bridge_source_body_relative_to_track_T"}});
    }
    for (std::size_t i = 0; i < kWheels.size(); ++i) {
        const std::string suffix(kWheels[i]);
        interfaces.push_back({
            {"name", assembled.contact_force_plan()->interface_name(static_cast<int>(i))},
            {"station_column", "rail_profile_reference_marker_track_station_meters." + suffix},
            {"patch_count_column", "patch_count." + suffix},
            {"normal_force_column", "N." + suffix},
            {"support_force_column", "Q." + suffix},
            {"force_columns", {{"x", "force_x." + suffix},
                               {"y", "force_y." + suffix},
                               {"z", "force_z." + suffix}}}});
    }
    return {{"schema_identifier", "orvd.qualification_physical_observations.v1"},
            {"file", "observations.tsv"},
            {"row_join_key", {"sample_index", "time_nanoseconds"}},
            {"time_seconds_role", "audit_only"},
            {"force_frame", "carrier_projection_track_T"},
            {"carriers", std::move(carriers)}, {"interfaces", std::move(interfaces)}};
}

void WriteMetadata(
    const std::filesystem::path& path,
    const ResolvedConfiguration& configuration,
    const configuration::ResolvedStartupState& startup,
    const configuration::AssembledVehicleSystem& assembled,
    const configuration::IrwFullStateControlEventSession& session,
    const IrwR300Aar5V60At100HzFullStateGuidanceRunSummary& summary,
    const nlohmann::json& numerical_metadata,
    std::string_view controller_identifier,
    std::string_view conditioner_identifier,
    const QualificationSampleClock& observation_clock) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"completed\": true,\n"
           << "  \"artifact_schema_identifier\": \"orvd.controlled_vehicle_qualification.v1\",\n"
           << "  \"continuous_state_observation_contract\": "
           << ContinuousStateObservationContract(observation_clock).dump() << ",\n"
           << "  \"physical_observation_contract\": "
           << PhysicalObservationContract(assembled).dump() << ",\n"
           << "  \"qualification_vehicle_recipe\": "
           << JsonString(
                  "IRW_R300_AAR5_V60_100HZ_FULL_STATE_WHEEL_SPEED_GUIDANCE")
           << ",\n"
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
           << "  \"control_observation_basis\": "
           << JsonString(
                  "lateral displacement in Track-T; yaw in the physical "
                  "axle-bridge body basis")
           << ",\n"
           << "  \"startup_rule\": "
           << JsonString(
                  "one full-period initialization recurrence followed by U0 "
                  "at the same accepted startup state before backend "
                  "construction")
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
           << "  \"numerical_execution_contract\": {\n";
    for (const auto& [key, value] : numerical_metadata.items()) {
        output << "    " << JsonString(key) << ": ";
        // Preserve the existing scalar double text contract (17 digits).
        if (value.is_number_float()) output << value.get<double>();
        else output << value.dump();
        output << ",\n";
    }
    output
           << "    \"floating_point_compilation_contract\": {\n"
           << "      \"identifier\": "
           << JsonString(
                  internal::kStrictFloatingPointSemanticsIdentifier)
           << ",\n"
           << "      \"cmake_external_flag_audit_passed\": true,\n"
           << "      \"compile_command_audit_enabled\": true,\n"
           << "      \"fast_math_macro_defined\": false,\n"
           << "      \"finite_math_only_enabled\": false,\n"
           << "      \"build_type\": "
           << JsonString(internal::kQualificationBuildType) << ",\n"
           << "      \"compiler_id\": "
           << JsonString(internal::kQualificationCxxCompilerId) << ",\n"
           << "      \"compiler_version\": "
           << JsonString(internal::kQualificationCxxCompilerVersion)
           << "\n"
           << "    }\n"
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
    const IrwR300Aar5V60At100HzFullStateGuidanceRunSummary& summary,
    std::int64_t duration_nanoseconds) {
    const double duration_seconds = static_cast<double>(duration_nanoseconds) * 1.0e-9;
    const double dynamics_seconds = summary.advance_and_synchronization_wall_seconds +
        summary.control_wall_seconds;
    const double compute_seconds = dynamics_seconds + summary.observation_wall_seconds;
    const nlohmann::json result{
        {"integrator_recipe_identifier", summary.integrator_recipe_identifier},
        {"advance_and_synchronization_wall_seconds", summary.advance_and_synchronization_wall_seconds},
        {"control_wall_seconds", summary.control_wall_seconds},
        {"observation_wall_seconds", summary.observation_wall_seconds},
        {"data_and_metadata_write_wall_seconds", summary.data_and_metadata_write_wall_seconds},
        {"simulated_duration_seconds", duration_seconds},
        {"integrated_dynamics_wall_seconds", dynamics_seconds},
        {"qualification_compute_wall_seconds", compute_seconds},
        {"integrated_dynamics_realtime_factor", duration_seconds / dynamics_seconds},
        {"qualification_compute_realtime_factor", duration_seconds / compute_seconds},
        {"numerical_timings", summary.numerical_timings.ToJson()},
        {"integration_work", summary.integration_work.ToJson()}};
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) Reject("could not open '" + path.string() + "'");
    output << result.dump(2) << '\n';
    CloseChecked(&output, path);
}

}  // namespace

IrwR300Aar5V60At100HzFullStateGuidanceRunSummary
RunIrwR300Aar5V60At100HzFullStateGuidance(
    const IrwR300Aar5V60At100HzFullStateGuidanceRunConfiguration& input) {
    const ResolvedConfiguration resolved = ResolveConfiguration(input);
    const auto integration_request = RequestIntegrationConfiguration(
        input.integration_config_path);
    RequireFailureResultDestinationAvailable(resolved.output_directory);
    AtomicQualificationDirectory output_directory(
        resolved.output_directory);
    const auto vehicle = configuration::LoadVehicleDefinitionFromJsonFile(
        resolved.vehicle_definition_path);
    const auto startup =
        configuration::LoadResolvedStartupStateFromJsonFile(
            resolved.resolved_startup_state_path);
    if (startup.initial_longitudinal_speed_meters_per_second != 60.0 / 3.6) {
        Reject("the resolved startup state is not the closed 60 km/h "
               "identity");
    }
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
        std::move(irregularity));
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
        Reject("the assembled IRW startup state or 96+8+16 wrench topology "
               "does not match the closed guidance recipe");
    }

    auto controller = configuration::
        LoadIrwFullStateWheelSpeedGuidanceControllerFromJsonFile(
            resolved.controller_configuration_path);
    const std::string controller_identifier = controller.config().identifier;
    if (controller_identifier != kControllerIdentifier) {
        Reject("the loaded controller does not match the closed guidance "
               "identity");
    }
    auto conditioner =
        configuration::LoadWheelDriveTorqueCommandConditionerFromJsonFile(
            resolved.torque_conditioner_configuration_path);
    const std::string conditioner_identifier =
        conditioner.config().identifier;
    if (conditioner_identifier != kTorqueConditionerIdentifier) {
        Reject("the loaded torque conditioner does not match the closed "
               "guidance identity");
    }
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
    const Clock::time_point startup_control_begin = Clock::now();
    audits.push_back(event_session.ApplyInitializationUpdate(accepted));
    audits.push_back(event_session.ApplyPeriodicUpdate(accepted));
    const double startup_control_wall_seconds =
        ElapsedSeconds(startup_control_begin, Clock::now());
    constexpr auto& numerical_recipe =
        internal::
            kIrwR300Aar5V60At100HzFullStateGuidanceIntegrationRecipe;
    Eigen::VectorXd integration_initial_state(assembled.system().continuous_state_size());
    assembled.system().CopyContinuousState(accepted, integration_initial_state);
    auto numerics = ResolveIntegrationConfiguration(
        integration_request, assembled, integration_initial_state,
        ExplicitOdeRequest{numerical_recipe.default_integration_recipe,
            numerical_recipe.relative_tolerance,
            numerical_recipe.position_absolute_tolerance,
            numerical_recipe.velocity_absolute_tolerance,
            numerical_recipe.series_force_absolute_tolerance_newtons});
    numerics.metadata["execution_conditions_at_start"] =
        CaptureQualificationExecutionConditions(
            *assembled.contact_force_plan());
    auto failure_metadata = numerics.metadata;
    failure_metadata["input_paths"] = {
        {"vehicle_definition", resolved.vehicle_definition_path.string()},
        {"resolved_startup_state", resolved.resolved_startup_state_path.string()},
        {"track_geometry", resolved.track_geometry_path.string()},
        {"orvd_data_root", resolved.orvd_data_root.string()},
        {"controller_configuration", resolved.controller_configuration_path.string()},
        {"torque_conditioner_configuration", resolved.torque_conditioner_configuration_path.string()}};
    failure_metadata["track_irregularity_identifier"] = kIrregularityIdentifier;
    failure_metadata["sample_period_nanoseconds"] = kObservationPeriodNanoseconds;
    QualificationIntegrationRun integration(resolved.output_directory, std::move(failure_metadata),
        std::move(numerics.configuration), assembled, accepted,
        observation_clock.terminal_time_seconds());
    const auto method_identifier = integration.advancer().method_identifier();
    IrwR300Aar5V60At100HzFullStateGuidanceRunSummary summary(method_identifier);
    summary.control_wall_seconds = startup_control_wall_seconds;
    if (method_identifier == "cvode_bdf2") summary.maximum_bdf_order = 2;
    if (method_identifier == "cvode_bdf5") summary.maximum_bdf_order = 5;
    event_session.ConfirmBackendSynchronized();

    auto contact_workspace = assembled.contact_force_plan()->CreateWorkspace();
    std::vector<AppliedBodyWrench> contact_wrenches(
        static_cast<std::size_t>(
            assembled.contact_force_plan()->body_wrench_count()));
    std::array<WheelRailContactInterfaceObservation, kWheelCount>
        contact_observations{};
    auto observation_context =
        assembled.system().CreateDefaultRuntimeContext(0.0);
    assembled.system().CopyContextLocalData(accepted, *observation_context);
    assembled.system().SetTimeContinuousStateAndWheelRailProjectionHints(
        *observation_context, 0.0, integration_initial_state,
        accepted.wheel_rail_projection_station_hints_meters());
    const Clock::time_point initial_observation_begin = Clock::now();
    observations.push_back(ObserveControlledSample(
        assembled, event_session, *observation_context, 0, 0, 0.0,
        *contact_workspace, contact_wrenches, contact_observations,
        &patch_observations));
    summary.observation_wall_seconds +=
        ElapsedSeconds(initial_observation_begin, Clock::now());
    // Persist only the already requested dense samples, outside all numerical
    // and observation timing scopes. Adjacent event boundaries are written once.
    const Clock::time_point state_writer_begin = Clock::now();
    QualificationContinuousStateWriter state_writer(
        output_directory.working_path() / "continuous_states.tsv",
        observation_clock, observation_times_seconds,
        assembled.model().num_generalized_positions(),
        assembled.model().num_generalized_velocities(),
        assembled.force_plan().series_spring_damper_force_state_count());
    state_writer.Append(0, integration_initial_state);
    summary.data_and_metadata_write_wall_seconds +=
        ElapsedSeconds(state_writer_begin, Clock::now());
    for (std::uint64_t ordinal = 1; ordinal <= terminal_event_ordinal;
         ++ordinal) {
        event_session.RequireReadyToAdvance();
        const std::size_t interval_begin = static_cast<std::size_t>(
            (ordinal - 1U) * kObservationsPerEventPeriod);
        const std::span<const double> interval_sample_times(
            observation_times_seconds.data() + interval_begin,
            static_cast<std::size_t>(kObservationsPerEventPeriod + 1U));
        const Eigen::MatrixXd dense_states =
            integration.Advance(
                event_session.next_periodic_event_time_seconds(),
                interval_sample_times);
        const Clock::time_point state_write_begin = Clock::now();
        for (std::uint64_t local_sample = 1;
             local_sample <= kObservationsPerEventPeriod; ++local_sample) {
            state_writer.Append(interval_begin + static_cast<std::size_t>(local_sample),
                dense_states.col(static_cast<Eigen::Index>(local_sample)));
        }
        summary.data_and_metadata_write_wall_seconds +=
            ElapsedSeconds(state_write_begin, Clock::now());
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
        summary.observation_wall_seconds +=
            ElapsedSeconds(observation_begin, Clock::now());
        const Clock::time_point control_begin = Clock::now();
        audits.push_back(event_session.ApplyPeriodicUpdate(accepted));
        assembled.system().CopyContextLocalData(accepted,
                                                *observation_context);
        summary.control_wall_seconds +=
            ElapsedSeconds(control_begin, Clock::now());
        if (ordinal < terminal_event_ordinal) {
            integration.Synchronize();
            event_session.ConfirmBackendSynchronized();
            ++summary.backend_synchronization_count;
        }
    }

    summary.integration_work = integration.ledger();
    summary.numerical_timings = integration.timings();
    summary.advance_and_synchronization_wall_seconds =
        summary.numerical_timings.advance_wall_seconds +
        summary.numerical_timings.synchronization_wall_seconds;
    summary.observation_count = observations.size();
    summary.control_audit_count = audits.size();
    summary.positive_hold_interval_count =
        static_cast<std::size_t>(terminal_event_ordinal);
    summary.terminal_continuous_state.resize(
        assembled.system().continuous_state_size());
    assembled.system().CopyContinuousState(
        accepted, summary.terminal_continuous_state);
    const Clock::time_point write_begin = Clock::now();
    state_writer.Close();
    WriteControlAudits(output_directory.working_path() / "control_events.tsv",
                       audits);
    WriteObservations(output_directory.working_path() / "observations.tsv",
                      observations);
    WritePatchObservations(
        output_directory.working_path() / "contact_patches.tsv",
        patch_observations, assembled);
    WriteMetadata(output_directory.working_path() / "metadata.json",
                  resolved, startup, assembled, event_session, summary,
                  numerics.metadata,
                  controller_identifier, conditioner_identifier,
                  observation_clock);
    const std::filesystem::path complete_path =
        output_directory.working_path() / "COMPLETE";
    std::ofstream complete(complete_path, std::ios::out | std::ios::trunc);
    if (!complete) {
        Reject("could not open '" + complete_path.string() + "'");
    }
    complete << summary.observation_count << " controlled observations\n";
    CloseChecked(&complete, complete_path);
    const Clock::time_point write_end = Clock::now();
    summary.data_and_metadata_write_wall_seconds +=
        ElapsedSeconds(write_begin, write_end);
    WritePerformance(
        output_directory.working_path() / "performance.json", summary,
        resolved.duration_nanoseconds);
    output_directory.Publish();
    return summary;
}

}  // namespace orvd::dynamics_qualification
