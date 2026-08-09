#include "gz18_qualification_runner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "atomic_qualification_directory.h"
#include "qualification_sample_clock.h"

#include "orvd/configuration/assembled_gz18_contact_scenario.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_track_irregularity_field.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/multibody_model/multibody_applied_forces.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/track_geometry/track_geometry.h"
#include "orvd/wheel_rail_contact/profile_track_roll_transport.h"

namespace orvd::dynamics_qualification {
namespace {

using Clock = std::chrono::steady_clock;
using configuration::AssembledGz18ContactScenario;
using forces::WheelRailContactInterfaceObservation;
using multibody_model::AppliedBodyWrench;
using track_geometry::TrackStationRegion;

constexpr double kVehicleReferenceTrackStationMeters = 0.0;
constexpr double kContactProjectionHalfWidthMeters = 0.01;
constexpr double kBodyObservationProjectionHalfWidthMeters = 0.02;
constexpr std::size_t kCarrierCount = 4;
constexpr std::size_t kInterfaceCount = 8;
constexpr std::size_t kRepresentativeBodyCount = 3;
constexpr std::array<std::string_view, kRepresentativeBodyCount>
    kRepresentativeBodyNames{
        "carbody", "front_bogie_frame", "rear_bogie_frame"};

struct CarrierObservation final {
    double track_station_meters{};
    double lateral_meters{};
    double yaw_radians{};
};

struct RepresentativeBodyObservation final {
    double track_station_meters{};
    double lateral_meters{};
    double yaw_radians{};
};

struct QualificationObservation final {
    std::uint64_t sample_index{};
    double time_seconds{};
    std::array<CarrierObservation, kCarrierCount> carriers{};
    std::array<WheelRailContactInterfaceObservation, kInterfaceCount>
        interfaces{};
    std::array<RepresentativeBodyObservation, kRepresentativeBodyCount>
        representative_bodies{};
    double vehicle_suspension_spatial_power_watts{};
    double wheel_rail_spatial_power_watts{};
};

struct BoundaryUse final {
    bool observed{false};
    std::string carrier_name;
    std::uint64_t sample_index{};
    double track_station_meters{};
    double definition_boundary_meters{};
};

struct EndpointDiagnostics final {
    double generalized_force_residual_inf_norm{};
    double virtual_power_residual_watts{};
    double position_derivative_residual_inf_norm{};
    double maxwell_derivative_residual_inf_norm{};
};

[[nodiscard]] double ElapsedSeconds(Clock::time_point begin,
                                    Clock::time_point end) {
    return std::chrono::duration<double>(end - begin).count();
}

[[noreturn]] void Reject(std::string detail) {
    throw std::runtime_error("GZ18 dynamics qualification: " + detail);
}

void RequireFinite(double value, std::string_view name) {
    if (!std::isfinite(value)) {
        Reject(std::string(name) + " is not finite");
    }
}

[[nodiscard]] std::string JsonString(std::string_view input) {
    std::ostringstream output;
    output << '"';
    constexpr char kHex[] = "0123456789abcdef";
    for (const unsigned char character : input) {
        switch (character) {
            case '"':
                output << "\\\"";
                break;
            case '\\':
                output << "\\\\";
                break;
            case '\b':
                output << "\\b";
                break;
            case '\f':
                output << "\\f";
                break;
            case '\n':
                output << "\\n";
                break;
            case '\r':
                output << "\\r";
                break;
            case '\t':
                output << "\\t";
                break;
            default:
                if (character < 0x20U) {
                    output << "\\u00" << kHex[character >> 4U]
                           << kHex[character & 0x0fU];
                } else {
                    output << static_cast<char>(character);
                }
                break;
        }
    }
    output << '"';
    return output.str();
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

[[nodiscard]] integrators::ContinuousStateErrorTolerances
MakeGz18Tolerances(const configuration::AssembledVehicleSystem& assembled) {
    const auto& system = assembled.system();
    Eigen::VectorXd absolute =
        Eigen::VectorXd::Constant(system.continuous_state_size(), 1.0e-9);
    const auto q = system.generalized_positions_state_range();
    const auto z = system.series_spring_damper_force_state_range();
    absolute.segment(q.start(), q.size()).setConstant(1.0e-10);
    absolute.segment(z.start(), z.size()).setConstant(1.0e-2);
    return integrators::ContinuousStateErrorTolerances(1.0e-8,
                                                       std::move(absolute));
}

[[nodiscard]] double InitialBodyTrackStation(
    const configuration::VehicleDefinition& vehicle,
    const configuration::ResolvedStartupState& startup,
    std::string_view body_name) {
    const auto mechanical = std::find_if(
        vehicle.mechanical_track_station_layout.free_body_station_offsets
            .begin(),
        vehicle.mechanical_track_station_layout.free_body_station_offsets.end(),
        [body_name](const auto& entry) { return entry.body_name == body_name; });
    const auto resolved = std::find_if(
        startup.free_body_startup_states.begin(),
        startup.free_body_startup_states.end(),
        [body_name](const auto& entry) { return entry.body_name == body_name; });
    if (mechanical ==
            vehicle.mechanical_track_station_layout.free_body_station_offsets
                .end() ||
        resolved == startup.free_body_startup_states.end()) {
        Reject("representative body '" + std::string(body_name) +
               "' is absent from the mechanical or resolved station layout");
    }
    const double station =
        mechanical->station_offset_meters +
        resolved->resolved_track_station_offset_from_mechanical_layout_meters;
    RequireFinite(station, "representative-body initial station");
    return station;
}

[[nodiscard]] double WrenchPower(
    const multibody_model::MultibodyModel& model,
    const multibody_model::MultibodyEvaluationContext& context,
    const AppliedBodyWrench& wrench) {
    if (wrench.expressed_in_frame != model.world_frame()) {
        Reject("an observed wrench is not expressed in the world frame");
    }
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
    const double power =
        wrench.torque_about_point_newton_metres.dot(
            velocity.angular_velocity_radians_per_second()) +
        wrench.force_newtons.dot(point_velocity);
    RequireFinite(power, "wrench power");
    return power;
}

[[nodiscard]] CarrierObservation ObserveCarrier(
    const configuration::AssembledVehicleSystem& assembled,
    const multibody_model::MultibodyEvaluationContext& context,
    int carrier_index, double track_station_meters) {
    const auto* contact_plan = assembled.contact_force_plan();
    const auto track =
        contact_plan->track_geometry().EvaluateTrackFrame(track_station_meters);
    const auto body = assembled.model().GetRigidBodyByName(
        contact_plan->carrier_name(carrier_index));
    const auto body_pose = assembled.model().CalcPoseInWorld(context, body);
    const Eigen::Matrix3d rotation_track_from_inertial =
        track.pose().rotation_inertial_from_track().transpose();
    const Eigen::Vector3d origin_in_track =
        rotation_track_from_inertial *
        (body_pose.translation() -
         track.pose().origin_in_inertial_meters());
    const auto angles = wheel_rail_contact::ResolveRollYawPitch(
        rotation_track_from_inertial * body_pose.rotation());
    CarrierObservation observation{
        track_station_meters, origin_in_track.y(), angles.yaw_radians};
    RequireFinite(observation.lateral_meters, "wheelset lateral position");
    RequireFinite(observation.yaw_radians, "wheelset yaw");
    return observation;
}

[[nodiscard]] RepresentativeBodyObservation ObserveRepresentativeBody(
    const configuration::AssembledVehicleSystem& assembled,
    const multibody_model::MultibodyEvaluationContext& context,
    multibody_model::RigidBodyHandle body, double* projection_seed_meters) {
    const auto* contact_plan = assembled.contact_force_plan();
    const auto body_pose = assembled.model().CalcPoseInWorld(context, body);
    const auto projection = contact_plan->track_geometry().ProjectPointNearSeed(
        body_pose.translation(), *projection_seed_meters,
        kBodyObservationProjectionHalfWidthMeters);
    *projection_seed_meters = projection.track_station_meters();
    const auto track = contact_plan->track_geometry().EvaluateTrackFrame(
        *projection_seed_meters);
    const Eigen::Matrix3d rotation_track_from_inertial =
        track.pose().rotation_inertial_from_track().transpose();
    const Eigen::Vector3d origin_in_track =
        rotation_track_from_inertial *
        (body_pose.translation() -
         track.pose().origin_in_inertial_meters());
    const auto angles = wheel_rail_contact::ResolveRollYawPitch(
        rotation_track_from_inertial * body_pose.rotation());
    RepresentativeBodyObservation observation{
        *projection_seed_meters, origin_in_track.y(), angles.yaw_radians};
    RequireFinite(observation.track_station_meters,
                  "representative-body station");
    RequireFinite(observation.lateral_meters,
                  "representative-body lateral position");
    RequireFinite(observation.yaw_radians, "representative-body yaw");
    return observation;
}

void RecordBoundaryUse(
    const configuration::AssembledVehicleSystem& assembled,
    const QualificationObservation& observation, BoundaryUse* before,
    BoundaryUse* after) {
    const auto* plan = assembled.contact_force_plan();
    const auto& line = plan->track_geometry();
    for (std::size_t carrier = 0; carrier < observation.carriers.size();
         ++carrier) {
        const double station = observation.carriers[carrier].track_station_meters;
        const TrackStationRegion region = line.ClassifyTrackStation(station);
        BoundaryUse* destination = nullptr;
        double boundary{};
        if (region == TrackStationRegion::kBeforeDefinedInterval) {
            destination = before;
            boundary = line.start_track_station_meters();
        } else if (region == TrackStationRegion::kAfterDefinedInterval) {
            destination = after;
            boundary = line.end_track_station_meters();
        }
        if (destination != nullptr && !destination->observed) {
            destination->observed = true;
            destination->carrier_name =
                std::string(plan->carrier_name(static_cast<int>(carrier)));
            destination->sample_index = observation.sample_index;
            destination->track_station_meters = station;
            destination->definition_boundary_meters = boundary;
        }
    }
}

[[nodiscard]] EndpointDiagnostics CalcEndpointDiagnostics(
    const configuration::AssembledVehicleSystem& assembled,
    system_assembly::SystemRuntimeContext& context,
    std::span<const AppliedBodyWrench> wrenches,
    const Eigen::VectorXd& series_derivatives) {
    const int nq = assembled.model().num_generalized_positions();
    const int nv = assembled.model().num_generalized_velocities();
    Eigen::VectorXd rhs(assembled.system().continuous_state_size());
    assembled.compiled_plan().CalcStateTimeDerivatives(context, rhs);
    auto component = assembled.system().GetMultibodyComponentView(
        context, assembled.system().multibody_component());

    Eigen::VectorXd projected_generalized_force = Eigen::VectorXd::Zero(nv);
    Eigen::MatrixXd angular_jacobian(3, nv);
    Eigen::MatrixXd translational_jacobian(3, nv);
    double spatial_power = 0.0;
    for (const AppliedBodyWrench& wrench : wrenches) {
        assembled.model()
            .CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld(
                component.context(), wrench.body,
                wrench.point_position_in_body_frame_meters, &angular_jacobian,
                &translational_jacobian);
        projected_generalized_force.noalias() +=
            angular_jacobian.transpose() *
                wrench.torque_about_point_newton_metres +
            translational_jacobian.transpose() * wrench.force_newtons;
        spatial_power +=
            WrenchPower(assembled.model(), component.context(), wrench);
    }

    Eigen::VectorXd required_generalized_force(nv);
    assembled.model().CalcRequiredGeneralizedForces(
        component.context(), rhs.segment(nq, nv),
        required_generalized_force);
    Eigen::VectorXd mapped_qdot(nq);
    assembled.model().MapGeneralizedVelocitiesToPositionDerivatives(
        component.context(), context.generalized_velocities(), &mapped_qdot);

    EndpointDiagnostics result;
    result.generalized_force_residual_inf_norm =
        (projected_generalized_force - required_generalized_force)
            .lpNorm<Eigen::Infinity>();
    result.virtual_power_residual_watts =
        projected_generalized_force.dot(context.generalized_velocities()) -
        spatial_power;
    result.position_derivative_residual_inf_norm =
        (rhs.head(nq) - mapped_qdot).lpNorm<Eigen::Infinity>();
    result.maxwell_derivative_residual_inf_norm =
        (rhs.tail(series_derivatives.size()) - series_derivatives)
            .lpNorm<Eigen::Infinity>();
    RequireFinite(result.generalized_force_residual_inf_norm,
                  "endpoint generalized-force residual");
    RequireFinite(result.virtual_power_residual_watts,
                  "endpoint virtual-power residual");
    RequireFinite(result.position_derivative_residual_inf_norm,
                  "endpoint position-derivative residual");
    RequireFinite(result.maxwell_derivative_residual_inf_norm,
                  "endpoint Maxwell-derivative residual");
    return result;
}

void WriteObservationHeader(
    std::ofstream* output,
    const configuration::AssembledVehicleSystem& assembled) {
    *output << "sample_index\ttime_seconds";
    const auto* contact_plan = assembled.contact_force_plan();
    for (int carrier = 0; carrier < contact_plan->carrier_count(); ++carrier) {
        const std::string name(contact_plan->carrier_name(carrier));
        *output << '\t' << name << ".track_station_meters"
                << '\t' << name << ".lateral_meters"
                << '\t' << name << ".yaw_radians";
    }
    for (int interface = 0; interface < contact_plan->interface_count();
         ++interface) {
        const std::string name(contact_plan->interface_name(interface));
        *output << '\t' << name << ".contact_patch_count"
                << '\t' << name
                << ".vertical_support_force_on_wheel_newtons"
                << '\t' << name << ".normal_force_newtons"
                << '\t' << name
                << ".longitudinal_force_on_wheel_newtons"
                << '\t' << name << ".lateral_force_on_wheel_newtons";
    }
    for (const std::string_view name : kRepresentativeBodyNames) {
        *output << '\t' << name << ".track_station_meters"
                << '\t' << name << ".lateral_meters"
                << '\t' << name << ".yaw_radians";
    }
    *output << "\tvehicle_suspension_spatial_power_watts"
            << "\twheel_rail_spatial_power_watts\n";
}

void WriteObservation(std::ofstream* output,
                      const QualificationObservation& observation) {
    *output << observation.sample_index << '\t' << observation.time_seconds;
    for (const CarrierObservation& carrier : observation.carriers) {
        *output << '\t' << carrier.track_station_meters << '\t'
                << carrier.lateral_meters << '\t' << carrier.yaw_radians;
    }
    for (const WheelRailContactInterfaceObservation& interface :
         observation.interfaces) {
        *output << '\t' << interface.contact_patch_count << '\t'
                << interface.vertical_support_force_on_wheel_newtons << '\t'
                << interface.normal_force_newtons << '\t'
                << interface.longitudinal_force_on_wheel_newtons << '\t'
                << interface.lateral_force_on_wheel_newtons;
    }
    for (const RepresentativeBodyObservation& body :
         observation.representative_bodies) {
        *output << '\t' << body.track_station_meters << '\t'
                << body.lateral_meters << '\t' << body.yaw_radians;
    }
    *output << '\t' << observation.vehicle_suspension_spatial_power_watts
            << '\t' << observation.wheel_rail_spatial_power_watts << '\n';
}

void WriteBoundaryUse(std::ofstream* output, std::string_view key,
                      const BoundaryUse& use, bool trailing_comma) {
    *output << "    " << JsonString(key) << ": ";
    if (!use.observed) {
        *output << "null";
    } else {
        *output << "{\"carrier_name\": " << JsonString(use.carrier_name)
                << ", \"sample_index\": " << use.sample_index
                << ", \"track_station_meters\": "
                << use.track_station_meters
                << ", \"definition_boundary_meters\": "
                << use.definition_boundary_meters << '}';
    }
    *output << (trailing_comma ? ",\n" : "\n");
}

void WriteMetadata(
    const std::filesystem::path& path,
    const Gz18QualificationRunConfiguration& configuration,
    const configuration::ResolvedStartupState& startup,
    const configuration::AssembledVehicleSystem& assembled,
    const QualificationSampleClock& clock, const BoundaryUse& before,
    const BoundaryUse& after,
    const std::array<std::size_t, kInterfaceCount>& longest_zero_contact_runs,
    const EndpointDiagnostics& diagnostics) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"internal_format_revision\": 1,\n"
           << "  \"completed\": true,\n"
           << "  \"vehicle_name\": "
           << JsonString(startup.vehicle_binding.vehicle_name) << ",\n"
           << "  \"mechanical_definition_identifier\": "
           << JsonString(
                  startup.vehicle_binding.mechanical_definition_identifier)
           << ",\n"
           << "  \"load_condition_identifier\": "
           << JsonString(startup.load_condition_identifier) << ",\n"
           << "  \"wheel_profile_identifier\": "
           << JsonString(startup.wheel_rail_binding.wheel_profile_identifier)
           << ",\n"
           << "  \"rail_profile_identifier\": "
           << JsonString(startup.wheel_rail_binding.rail_profile_identifier)
           << ",\n"
           << "  \"contact_strategy_identifier\": "
           << JsonString(startup.wheel_rail_binding
                             .wheel_rail_contact_strategy_identifier)
           << ",\n"
           << "  \"track_irregularity_identifier\": "
           << JsonString(configuration.track_irregularity_identifier) << ",\n"
           << "  \"initial_longitudinal_speed_meters_per_second\": "
           << startup.initial_longitudinal_speed_meters_per_second << ",\n"
           << "  \"vehicle_layout_reference_track_station_meters\": "
           << kVehicleReferenceTrackStationMeters << ",\n"
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
           << JsonString(configuration.orvd_data_root.string()) << "\n"
           << "  },\n"
           << "  \"sample_period_nanoseconds\": "
           << clock.sample_period_nanoseconds() << ",\n"
           << "  \"terminal_time_nanoseconds\": "
           << clock.terminal_time_nanoseconds() << ",\n"
           << "  \"sample_count\": " << clock.sample_count() << ",\n"
           << "  \"base_track_definition_interval_meters\": ["
           << assembled.contact_force_plan()
                  ->track_geometry()
                  .start_track_station_meters()
           << ", "
           << assembled.contact_force_plan()
                  ->track_geometry()
                  .end_track_station_meters()
           << "],\n"
           << "  \"boundary_use\": {\n";
    WriteBoundaryUse(&output, "before_definition_interval", before, true);
    WriteBoundaryUse(&output, "after_definition_interval", after, false);
    output << "  },\n"
           << "  \"longest_zero_contact_run_samples\": [";
    for (std::size_t index = 0; index < longest_zero_contact_runs.size();
         ++index) {
        output << (index == 0 ? "" : ", ")
               << longest_zero_contact_runs[index];
    }
    output << "],\n"
           << "  \"endpoint_diagnostics\": {\n"
           << "    \"generalized_force_residual_inf_norm\": "
           << diagnostics.generalized_force_residual_inf_norm << ",\n"
           << "    \"virtual_power_residual_watts\": "
           << diagnostics.virtual_power_residual_watts << ",\n"
           << "    \"position_derivative_residual_inf_norm\": "
           << diagnostics.position_derivative_residual_inf_norm << ",\n"
           << "    \"maxwell_derivative_residual_inf_norm\": "
           << diagnostics.maxwell_derivative_residual_inf_norm << "\n"
           << "  }\n"
           << "}\n";
    CloseChecked(&output, path);
}

void WritePerformance(const std::filesystem::path& path,
                      const Gz18QualificationRunSummary& summary,
                      std::size_t dense_state_bytes,
                      std::size_t observation_bytes) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"advance_wall_seconds\": "
           << summary.advance_wall_seconds << ",\n"
           << "  \"observation_wall_seconds\": "
           << summary.observation_wall_seconds << ",\n"
           << "  \"data_and_metadata_write_wall_seconds\": "
           << summary.data_and_metadata_write_wall_seconds << ",\n"
           << "  \"dense_state_bytes\": " << dense_state_bytes << ",\n"
           << "  \"observation_buffer_bytes\": " << observation_bytes
           << "\n"
           << "}\n";
    CloseChecked(&output, path);
}

void ReportBoundaryWarning(std::string_view side, const BoundaryUse& use) {
    if (!use.observed) {
        return;
    }
    std::fprintf(
        stderr,
        "warning: the base-track asset definition interval %.*s side was "
        "used; straight continuation along the boundary three-dimensional "
        "tangent is active and track irregularity is suppressed outside the "
        "definition interval (first carrier=%s, sample=%llu, station=%.17g "
        "m, boundary=%.17g m)\n",
        static_cast<int>(side.size()), side.data(), use.carrier_name.c_str(),
        static_cast<unsigned long long>(use.sample_index),
        use.track_station_meters, use.definition_boundary_meters);
}

}  // namespace

Gz18QualificationRunSummary RunGz18Qualification(
    const Gz18QualificationRunConfiguration& run_configuration) {
    if (run_configuration.track_irregularity_identifier.empty()) {
        throw std::invalid_argument(
            "GZ18 dynamics qualification: track-irregularity identifier is "
            "empty");
    }
    if (run_configuration.duration_nanoseconds <= 0 ||
        run_configuration.sample_period_nanoseconds <= 0) {
        throw std::invalid_argument(
            "GZ18 dynamics qualification: duration and sample period must be "
            "positive integer nanoseconds");
    }
    const QualificationSampleClock sample_clock(
        static_cast<std::uint64_t>(run_configuration.duration_nanoseconds),
        static_cast<std::uint64_t>(
            run_configuration.sample_period_nanoseconds));
    const std::vector<double> sample_times =
        sample_clock.MakeSampleTimesSeconds();
    AtomicQualificationDirectory output_directory(
        run_configuration.output_directory);

    const auto vehicle = configuration::LoadVehicleDefinitionFromJsonFile(
        run_configuration.vehicle_definition_path);
    const auto startup =
        configuration::LoadResolvedStartupStateFromJsonFile(
            run_configuration.resolved_startup_state_path);
    auto line = configuration::LoadTrackGeometryFromJsonFile(
        run_configuration.track_geometry_path);
    auto irregularity =
        std::make_unique<wheel_rail_contact::TrackIrregularityField>(
            configuration::LoadTrackIrregularityFieldFromDataRoot(
                run_configuration.orvd_data_root,
                run_configuration.track_irregularity_identifier));
    std::unique_ptr<AssembledGz18ContactScenario> scenario =
        configuration::AssembleGz18ContactScenario(
            vehicle, startup, std::move(line),
            run_configuration.orvd_data_root,
            kVehicleReferenceTrackStationMeters,
            kContactProjectionHalfWidthMeters, std::move(irregularity));
    auto& assembled = scenario->vehicle_system();
    auto& accepted = scenario->initial_context().context();
    if (accepted.time_seconds() != 0.0 || sample_times.front() != 0.0) {
        Reject("the qualification clock and resolved context must start at "
               "exactly zero seconds");
    }

    integrators::SystemContinuousStateAdvancer advancer(
        assembled.system(), assembled.compiled_plan(), accepted,
        MakeGz18Tolerances(assembled),
        integrators::NoCallTimeAppliedForces{});
    const Clock::time_point advance_begin = Clock::now();
    const Eigen::MatrixXd dense_states =
        advancer.AdvanceToWithDenseStateSamples(
            sample_clock.terminal_time_seconds(), sample_times);
    const Clock::time_point advance_end = Clock::now();
    if (dense_states.rows() != assembled.system().continuous_state_size() ||
        dense_states.cols() !=
            static_cast<Eigen::Index>(sample_clock.sample_count()) ||
        !dense_states.allFinite() ||
        accepted.time_seconds() != sample_clock.terminal_time_seconds()) {
        Reject("the dense state batch is incomplete, non-finite or detached "
               "from its accepted endpoint");
    }

    auto observation_context = assembled.system().CreateDefaultRuntimeContext(0.0);
    assembled.system().CopyContextLocalParameters(
        scenario->initial_context().context(), *observation_context);
    auto contact_workspace = assembled.contact_force_plan()->CreateWorkspace();
    std::vector<AppliedBodyWrench> vehicle_wrenches(
        static_cast<std::size_t>(assembled.force_plan().body_wrench_count()));
    std::vector<AppliedBodyWrench> contact_wrenches(
        static_cast<std::size_t>(
            assembled.contact_force_plan()->body_wrench_count()));
    Eigen::VectorXd series_derivatives(
        assembled.force_plan().series_spring_damper_force_state_count());
    std::array<WheelRailContactInterfaceObservation, kInterfaceCount>
        interface_observations{};
    if (assembled.contact_force_plan()->carrier_count() !=
            static_cast<int>(kCarrierCount) ||
        assembled.contact_force_plan()->interface_count() !=
            static_cast<int>(kInterfaceCount)) {
        Reject("the assembled GZ18 contact topology is not four carriers and "
               "eight interfaces");
    }

    const std::array<multibody_model::RigidBodyHandle,
                     kRepresentativeBodyCount>
        representative_bodies{
            assembled.model().GetRigidBodyByName(kRepresentativeBodyNames[0]),
            assembled.model().GetRigidBodyByName(kRepresentativeBodyNames[1]),
            assembled.model().GetRigidBodyByName(kRepresentativeBodyNames[2])};
    std::array<double, kRepresentativeBodyCount> body_projection_seeds{};
    for (std::size_t index = 0; index < body_projection_seeds.size(); ++index) {
        body_projection_seeds[index] = InitialBodyTrackStation(
            vehicle, startup, kRepresentativeBodyNames[index]);
    }

    std::vector<QualificationObservation> observations;
    observations.reserve(sample_clock.sample_count());
    BoundaryUse before_definition_interval;
    BoundaryUse after_definition_interval;
    std::array<std::size_t, kInterfaceCount> current_zero_contact_runs{};
    std::array<std::size_t, kInterfaceCount> longest_zero_contact_runs{};
    const Clock::time_point observation_begin = Clock::now();
    for (std::size_t sample = 0; sample < sample_clock.sample_count(); ++sample) {
        assembled.system().SetTimeAndContinuousState(
            *observation_context, sample_times[sample],
            dense_states.col(static_cast<Eigen::Index>(sample)));
        assembled.system().UpdateWheelRailProjectionStationHints(
            *observation_context);
        auto component = assembled.system().GetMultibodyComponentView(
            *observation_context, assembled.system().multibody_component());
        assembled.force_plan().CalcAppliedForces(
            component.context(),
            observation_context->series_spring_damper_forces(),
            observation_context->nominal_forces(), vehicle_wrenches,
            series_derivatives);
        assembled.contact_force_plan()->CalcAppliedForcesAndObservations(
            component.context(), *contact_workspace,
            observation_context
                ->wheel_rail_projection_station_hints_meters(),
            contact_wrenches, interface_observations);

        QualificationObservation observation;
        observation.sample_index = static_cast<std::uint64_t>(sample);
        observation.time_seconds = sample_times[sample];
        const auto& station_hints =
            observation_context
                ->wheel_rail_projection_station_hints_meters();
        for (std::size_t carrier = 0; carrier < kCarrierCount; ++carrier) {
            observation.carriers[carrier] = ObserveCarrier(
                assembled, component.context(), static_cast<int>(carrier),
                station_hints[carrier]);
        }
        observation.interfaces = interface_observations;
        for (std::size_t body = 0; body < kRepresentativeBodyCount; ++body) {
            observation.representative_bodies[body] =
                ObserveRepresentativeBody(
                    assembled, component.context(),
                    representative_bodies[body],
                    &body_projection_seeds[body]);
        }
        for (const AppliedBodyWrench& wrench : vehicle_wrenches) {
            observation.vehicle_suspension_spatial_power_watts +=
                WrenchPower(assembled.model(), component.context(), wrench);
        }
        for (const AppliedBodyWrench& wrench : contact_wrenches) {
            observation.wheel_rail_spatial_power_watts +=
                WrenchPower(assembled.model(), component.context(), wrench);
        }
        RequireFinite(observation.vehicle_suspension_spatial_power_watts,
                      "vehicle suspension power");
        RequireFinite(observation.wheel_rail_spatial_power_watts,
                      "wheel-rail power");
        for (std::size_t interface = 0; interface < kInterfaceCount;
             ++interface) {
            const auto& value = observation.interfaces[interface];
            RequireFinite(value.vertical_support_force_on_wheel_newtons,
                          "wheel vertical support force");
            RequireFinite(value.normal_force_newtons,
                          "wheel normal force");
            RequireFinite(value.longitudinal_force_on_wheel_newtons,
                          "wheel longitudinal force");
            RequireFinite(value.lateral_force_on_wheel_newtons,
                          "wheel lateral force");
            if (value.contact_patch_count == 0) {
                ++current_zero_contact_runs[interface];
                longest_zero_contact_runs[interface] = std::max(
                    longest_zero_contact_runs[interface],
                    current_zero_contact_runs[interface]);
            } else {
                current_zero_contact_runs[interface] = 0;
            }
        }
        RecordBoundaryUse(assembled, observation,
                          &before_definition_interval,
                          &after_definition_interval);
        observations.push_back(observation);
    }

    std::vector<AppliedBodyWrench> all_wrenches;
    all_wrenches.reserve(vehicle_wrenches.size() + contact_wrenches.size());
    all_wrenches.insert(all_wrenches.end(), vehicle_wrenches.begin(),
                        vehicle_wrenches.end());
    all_wrenches.insert(all_wrenches.end(), contact_wrenches.begin(),
                        contact_wrenches.end());
    const EndpointDiagnostics endpoint_diagnostics = CalcEndpointDiagnostics(
        assembled, *observation_context, all_wrenches, series_derivatives);
    const Clock::time_point observation_end = Clock::now();

    Gz18QualificationRunSummary summary;
    summary.sample_count = sample_clock.sample_count();
    summary.advance_wall_seconds =
        ElapsedSeconds(advance_begin, advance_end);
    summary.observation_wall_seconds =
        ElapsedSeconds(observation_begin, observation_end);
    summary.used_before_track_definition_interval =
        before_definition_interval.observed;
    summary.used_after_track_definition_interval =
        after_definition_interval.observed;

    const Clock::time_point write_begin = Clock::now();
    const std::filesystem::path observation_path =
        output_directory.working_path() / "observations.tsv";
    std::ofstream observation_output(observation_path,
                                     std::ios::out | std::ios::trunc);
    if (!observation_output) {
        Reject("could not open '" + observation_path.string() + "'");
    }
    observation_output << std::setprecision(17);
    WriteObservationHeader(&observation_output, assembled);
    for (const QualificationObservation& observation : observations) {
        WriteObservation(&observation_output, observation);
    }
    CloseChecked(&observation_output, observation_path);

    WriteMetadata(output_directory.working_path() / "metadata.json",
                  run_configuration, startup, assembled, sample_clock,
                  before_definition_interval, after_definition_interval,
                  longest_zero_contact_runs, endpoint_diagnostics);
    const Clock::time_point write_end = Clock::now();
    summary.data_and_metadata_write_wall_seconds =
        ElapsedSeconds(write_begin, write_end);
    WritePerformance(
        output_directory.working_path() / "performance.json", summary,
        static_cast<std::size_t>(dense_states.size()) * sizeof(double),
        observations.size() * sizeof(QualificationObservation));
    const std::filesystem::path complete_path =
        output_directory.working_path() / "COMPLETE";
    std::ofstream complete(complete_path, std::ios::out | std::ios::trunc);
    if (!complete) {
        Reject("could not open '" + complete_path.string() + "'");
    }
    complete << summary.sample_count << " samples\n";
    CloseChecked(&complete, complete_path);
    output_directory.Publish();

    ReportBoundaryWarning("left", before_definition_interval);
    ReportBoundaryWarning("right", after_definition_interval);
    return summary;
}

}  // namespace orvd::dynamics_qualification
