#include "vehicle_qualification_runner_internal.h"

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
#include <system_error>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <omp.h>

#include "atomic_qualification_directory.h"
#include "qualification_sample_clock.h"

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
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

namespace orvd::dynamics_qualification::internal {
namespace {

using Clock = std::chrono::steady_clock;
using configuration::AssembledVehicleContactScenario;
using forces::WheelRailContactInterfaceObservation;
using forces::WheelRailContactPatchObservation;
using multibody_model::AppliedBodyWrench;
using track_geometry::TrackStationRegion;

constexpr double kVehicleReferenceTrackStationMeters = 0.0;
constexpr double kRhsContactProjectionHalfWidthMeters = 0.01;
constexpr double kCarrierObservationProjectionBaseHalfWidthMeters = 0.01;
constexpr double kRepresentativeBodyObservationProjectionBaseHalfWidthMeters =
    0.02;
constexpr std::size_t kCarrierCount = 4;
constexpr std::size_t kInterfaceCount = 8;
constexpr int kMaximumContactWorkerCount = 8;

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

struct QualificationInterfaceObservation final {
    std::size_t contact_patch_count{};
    double rail_profile_reference_marker_track_station_meters{};
    double vertical_support_force_on_wheel_newtons{};
    double normal_force_newtons{};
    Eigen::Vector3d total_force_on_wheel_in_carrier_track_frame_newtons{
        Eigen::Vector3d::Zero()};
    int primary_patch_ordinal{-1};
    double primary_patch_normal_force_newtons{};
    double primary_patch_longitudinal_force_on_wheel_in_contact_frame_newtons{};
    double primary_patch_lateral_force_on_wheel_in_contact_frame_newtons{};
};

struct QualificationPatchObservation final {
    std::uint64_t sample_index{};
    std::uint64_t time_nanoseconds{};
    double time_seconds{};
    std::size_t interface_ordinal{};
    std::size_t patch_ordinal{};
    WheelRailContactPatchObservation patch;
};

struct QualificationObservation final {
    std::uint64_t sample_index{};
    std::uint64_t time_nanoseconds{};
    double time_seconds{};
    std::array<CarrierObservation, kCarrierCount> carriers{};
    std::array<QualificationInterfaceObservation, kInterfaceCount>
        interfaces{};
    std::array<RepresentativeBodyObservation, kRepresentativeBodyCount>
        representative_bodies{};
};

QualificationInterfaceObservation SummarizeInterfaceObservation(
    const WheelRailContactInterfaceObservation& source) {
    QualificationInterfaceObservation summary;
    summary.contact_patch_count = source.contact_patch_count;
    summary.rail_profile_reference_marker_track_station_meters =
        source.rail_profile_reference_marker_track_station_meters;
    summary.vertical_support_force_on_wheel_newtons =
        source.vertical_support_force_on_wheel_newtons;
    summary.normal_force_newtons = source.normal_force_newtons;
    summary.total_force_on_wheel_in_carrier_track_frame_newtons =
        source.total_force_on_wheel_in_carrier_track_frame_newtons;
    for (std::size_t patch = 0; patch < source.contact_patch_count; ++patch) {
        if (patch == 0 ||
            source.patches[patch].normal_force_newtons >
            source.patches[static_cast<std::size_t>(
                summary.primary_patch_ordinal)]
                    .normal_force_newtons) {
            summary.primary_patch_ordinal = static_cast<int>(patch);
        }
    }
    if (source.contact_patch_count != 0) {
        const auto& primary = source.patches[static_cast<std::size_t>(
            summary.primary_patch_ordinal)];
        summary.primary_patch_normal_force_newtons =
            primary.normal_force_newtons;
        summary
            .primary_patch_longitudinal_force_on_wheel_in_contact_frame_newtons =
            primary.longitudinal_force_on_wheel_in_contact_frame_newtons;
        summary.primary_patch_lateral_force_on_wheel_in_contact_frame_newtons =
            primary.lateral_force_on_wheel_in_contact_frame_newtons;
    }
    return summary;
}

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
    double position_derivative_slice_consistency_inf_norm{};
    double series_force_derivative_slice_consistency_inf_norm{};
};

struct ProjectionHistory final {
    double station_seed_meters{};
    double base_half_width_meters{};
    double maximum_half_width_meters{};
    Eigen::Vector3d previous_body_origin_in_inertial_meters{
        Eigen::Vector3d::Zero()};
    bool has_previous_body_origin{false};
};

[[nodiscard]] double ElapsedSeconds(Clock::time_point begin,
                                    Clock::time_point end) {
    return std::chrono::duration<double>(end - begin).count();
}

[[noreturn]] void Reject(std::string detail) {
    throw std::runtime_error("vehicle dynamics qualification: " + detail);
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

[[nodiscard]] QualificationRunConfiguration ResolveInputPaths(
    const QualificationRunConfiguration& input) {
    QualificationRunConfiguration resolved = input;
    resolved.vehicle_definition_path = CanonicalExistingInput(
        input.vehicle_definition_path, "vehicle definition");
    resolved.resolved_startup_state_path = CanonicalExistingInput(
        input.resolved_startup_state_path, "resolved start-up state");
    resolved.track_geometry_path =
        CanonicalExistingInput(input.track_geometry_path, "track geometry");
    resolved.orvd_data_root =
        CanonicalExistingInput(input.orvd_data_root, "ORVD data root");
    return resolved;
}

void RequireFinite(double value, std::string_view name) {
    if (!std::isfinite(value)) {
        Reject(std::string(name) + " is not finite");
    }
}

void RequireFinite(const Eigen::Vector3d& value, std::string_view name) {
    if (!value.allFinite()) {
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
MakeTolerances(const configuration::AssembledVehicleSystem& assembled,
               const VehicleQualificationRecipe& recipe) {
    const auto& system = assembled.system();
    Eigen::VectorXd absolute =
        Eigen::VectorXd::Constant(system.continuous_state_size(),
                                  recipe.generalized_velocity_absolute_tolerance);
    const auto q = system.generalized_positions_state_range();
    const auto z = system.series_spring_damper_force_state_range();
    absolute.segment(q.start(), q.size())
        .setConstant(recipe.generalized_position_absolute_tolerance);
    absolute.segment(z.start(), z.size())
        .setConstant(recipe.series_force_absolute_tolerance_newtons);
    return integrators::ContinuousStateErrorTolerances(
        recipe.relative_tolerance, std::move(absolute));
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

[[nodiscard]] double ProjectBodyOriginForObservation(
    const configuration::AssembledVehicleSystem& assembled,
    const Eigen::Vector3d& body_origin_in_inertial_meters,
    ProjectionHistory* history) {
    double half_width_meters = history->base_half_width_meters;
    if (history->has_previous_body_origin) {
        half_width_meters +=
            (body_origin_in_inertial_meters -
             history->previous_body_origin_in_inertial_meters)
                .norm();
    }
    RequireFinite(half_width_meters, "observation projection half width");
    const auto projection = assembled.contact_force_plan()
                                ->track_geometry()
                                .ProjectPointNearSeed(
                                    body_origin_in_inertial_meters,
                                    history->station_seed_meters,
                                    half_width_meters);
    history->station_seed_meters = projection.track_station_meters();
    history->previous_body_origin_in_inertial_meters =
        body_origin_in_inertial_meters;
    history->has_previous_body_origin = true;
    history->maximum_half_width_meters =
        std::max(history->maximum_half_width_meters, half_width_meters);
    return history->station_seed_meters;
}

[[nodiscard]] CarrierObservation ObserveCarrier(
    const configuration::AssembledVehicleSystem& assembled,
    const multibody_model::MultibodyEvaluationContext& context,
    multibody_model::RigidBodyHandle body, double track_station_meters) {
    const auto* contact_plan = assembled.contact_force_plan();
    const auto track =
        contact_plan->track_geometry().EvaluateTrackFrame(track_station_meters);
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
    multibody_model::RigidBodyHandle body, ProjectionHistory* history) {
    const auto* contact_plan = assembled.contact_force_plan();
    const auto body_pose = assembled.model().CalcPoseInWorld(context, body);
    const double track_station_meters = ProjectBodyOriginForObservation(
        assembled, body_pose.translation(), history);
    const auto track = contact_plan->track_geometry().EvaluateTrackFrame(
        track_station_meters);
    const Eigen::Matrix3d rotation_track_from_inertial =
        track.pose().rotation_inertial_from_track().transpose();
    const Eigen::Vector3d origin_in_track =
        rotation_track_from_inertial *
        (body_pose.translation() -
         track.pose().origin_in_inertial_meters());
    const auto angles = wheel_rail_contact::ResolveRollYawPitch(
        rotation_track_from_inertial * body_pose.rotation());
    RepresentativeBodyObservation observation{
        track_station_meters, origin_in_track.y(), angles.yaw_radians};
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
    result.position_derivative_slice_consistency_inf_norm =
        (rhs.head(nq) - mapped_qdot).lpNorm<Eigen::Infinity>();
    result.series_force_derivative_slice_consistency_inf_norm =
        (rhs.tail(series_derivatives.size()) - series_derivatives)
            .lpNorm<Eigen::Infinity>();
    RequireFinite(result.generalized_force_residual_inf_norm,
                  "endpoint generalized-force residual");
    RequireFinite(result.virtual_power_residual_watts,
                  "endpoint virtual-power residual");
    RequireFinite(result.position_derivative_slice_consistency_inf_norm,
                  "endpoint position-derivative slice consistency");
    RequireFinite(result.series_force_derivative_slice_consistency_inf_norm,
                  "endpoint series-force derivative slice consistency");
    return result;
}

void WriteObservationHeader(
    std::ofstream* output,
    const configuration::AssembledVehicleSystem& assembled,
    const VehicleQualificationRecipe& recipe) {
    *output << "sample_index\ttime_nanoseconds\ttime_seconds";
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
        *output << '\t' << name
                << ".rail_profile_reference_marker_track_station_meters"
                << '\t' << name << ".contact_patch_count"
                << '\t' << name
                << ".vertical_support_force_on_wheel_newtons"
                << '\t' << name << ".normal_force_newtons"
                << '\t' << name
                << ".total_force_on_wheel_in_carrier_track_frame_x_newtons"
                << '\t' << name
                << ".total_force_on_wheel_in_carrier_track_frame_y_newtons"
                << '\t' << name
                << ".total_force_on_wheel_in_carrier_track_frame_z_newtons"
                << '\t' << name << ".primary_patch_ordinal"
                << '\t' << name << ".primary_patch_normal_force_newtons"
                << '\t' << name
                << ".longitudinal_force_on_wheel_newtons"
                << '\t' << name << ".lateral_force_on_wheel_newtons";
    }
    for (const std::string_view name : recipe.representative_body_names) {
        *output << '\t' << name << ".track_station_meters"
                << '\t' << name << ".lateral_meters"
                << '\t' << name << ".yaw_radians";
    }
    *output << '\n';
}

void WriteObservation(std::ofstream* output,
                      const QualificationObservation& observation) {
    *output << observation.sample_index << '\t'
            << observation.time_nanoseconds << '\t'
            << observation.time_seconds;
    for (const CarrierObservation& carrier : observation.carriers) {
        *output << '\t' << carrier.track_station_meters << '\t'
                << carrier.lateral_meters << '\t' << carrier.yaw_radians;
    }
    for (const QualificationInterfaceObservation& interface :
         observation.interfaces) {
        *output << '\t'
                << interface
                       .rail_profile_reference_marker_track_station_meters
                << '\t' << interface.contact_patch_count << '\t'
                << interface.vertical_support_force_on_wheel_newtons << '\t'
                << interface.normal_force_newtons << '\t'
                << interface
                       .total_force_on_wheel_in_carrier_track_frame_newtons.x()
                << '\t'
                << interface
                       .total_force_on_wheel_in_carrier_track_frame_newtons.y()
                << '\t'
                << interface
                       .total_force_on_wheel_in_carrier_track_frame_newtons.z()
                << '\t' << interface.primary_patch_ordinal << '\t'
                << interface.primary_patch_normal_force_newtons << '\t'
                << interface
                       .primary_patch_longitudinal_force_on_wheel_in_contact_frame_newtons
                << '\t'
                << interface
                       .primary_patch_lateral_force_on_wheel_in_contact_frame_newtons;
    }
    for (const RepresentativeBodyObservation& body :
         observation.representative_bodies) {
        *output << '\t' << body.track_station_meters << '\t'
                << body.lateral_meters << '\t' << body.yaw_radians;
    }
    *output << '\n';
}

void WritePatchObservationHeader(std::ofstream* output) {
    *output
        << "sample_index\ttime_nanoseconds\ttime_seconds\tinterface_name"
        << "\tpatch_ordinal"
        << "\tnormal_force_newtons"
        << "\tlongitudinal_force_on_wheel_in_contact_frame_newtons"
        << "\tlateral_force_on_wheel_in_contact_frame_newtons"
        << "\tcontact_frame_angle_radians"
        << "\tcontact_point_in_carrier_track_frame_x_meters"
        << "\tcontact_point_in_carrier_track_frame_y_meters"
        << "\tcontact_point_in_carrier_track_frame_z_meters"
        << "\tforce_on_wheel_in_carrier_track_frame_x_newtons"
        << "\tforce_on_wheel_in_carrier_track_frame_y_newtons"
        << "\tforce_on_wheel_in_carrier_track_frame_z_newtons\n";
}

void WritePatchObservation(
    std::ofstream* output, const QualificationPatchObservation& observation,
    const configuration::AssembledVehicleSystem& assembled) {
    const auto& patch = observation.patch;
    *output << observation.sample_index << '\t'
            << observation.time_nanoseconds << '\t'
            << observation.time_seconds << '\t'
            << assembled.contact_force_plan()->interface_name(
                   static_cast<int>(observation.interface_ordinal))
            << '\t' << observation.patch_ordinal << '\t'
            << patch.normal_force_newtons << '\t'
            << patch.longitudinal_force_on_wheel_in_contact_frame_newtons
            << '\t' << patch.lateral_force_on_wheel_in_contact_frame_newtons
            << '\t' << patch.contact_frame_angle_radians << '\t'
            << patch.contact_point_in_carrier_track_frame_meters.x() << '\t'
            << patch.contact_point_in_carrier_track_frame_meters.y() << '\t'
            << patch.contact_point_in_carrier_track_frame_meters.z() << '\t'
            << patch.force_on_wheel_in_carrier_track_frame_newtons.x() << '\t'
            << patch.force_on_wheel_in_carrier_track_frame_newtons.y() << '\t'
            << patch.force_on_wheel_in_carrier_track_frame_newtons.z() << '\n';
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
    const QualificationRunConfiguration& configuration,
    const VehicleQualificationRecipe& recipe,
    const configuration::ResolvedStartupState& startup,
    const configuration::AssembledVehicleSystem& assembled,
    const QualificationSampleClock& clock, const BoundaryUse& before,
    const BoundaryUse& after,
    const std::array<std::size_t, kInterfaceCount>& longest_zero_contact_runs,
    double maximum_carrier_observation_projection_half_width_meters,
    double maximum_representative_body_observation_projection_half_width_meters,
    const EndpointDiagnostics& diagnostics) {
    const int requested_contact_worker_count =
        std::min({kMaximumContactWorkerCount,
                  static_cast<int>(kInterfaceCount), omp_get_max_threads()});
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"completed\": true,\n"
           << "  \"qualification_vehicle_recipe\": "
           << JsonString(recipe.vehicle_label) << ",\n"
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
           << "  \"track_irregularity_identifier\": ";
    if (configuration.track_irregularity_identifier.has_value()) {
        output << JsonString(*configuration.track_irregularity_identifier);
    } else {
        output << "null";
    }
    output << ",\n"
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
           << "  \"assembled_state_and_force_layout\": {\n"
           << "    \"generalized_position_count\": "
           << assembled.model().num_generalized_positions() << ",\n"
           << "    \"generalized_velocity_count\": "
           << assembled.model().num_generalized_velocities() << ",\n"
           << "    \"series_force_state_count\": "
           << assembled.force_plan()
                  .series_spring_damper_force_state_count()
           << ",\n"
           << "    \"vehicle_body_wrench_count\": "
           << assembled.force_plan().body_wrench_count() << ",\n"
           << "    \"contact_body_wrench_count\": "
           << assembled.contact_force_plan()->body_wrench_count() << "\n"
           << "  },\n"
           << "  \"numerical_execution_contract\": {\n"
           << "    \"relative_tolerance\": " << recipe.relative_tolerance
           << ",\n"
           << "    \"generalized_position_absolute_tolerance\": "
           << recipe.generalized_position_absolute_tolerance << ",\n"
           << "    \"generalized_velocity_absolute_tolerance\": "
           << recipe.generalized_velocity_absolute_tolerance << ",\n"
           << "    \"series_force_absolute_tolerance_newtons\": "
           << recipe.series_force_absolute_tolerance_newtons << ",\n"
           << "    \"openmp_dynamic_teams_enabled\": "
           << (omp_get_dynamic() != 0 ? "true" : "false") << ",\n"
           << "    \"openmp_runtime_maximum_threads\": "
           << omp_get_max_threads() << ",\n"
           << "    \"contact_batch_worker_cap\": "
           << kMaximumContactWorkerCount << ",\n"
           << "    \"contact_batch_requested_worker_count\": "
           << requested_contact_worker_count
           << ",\n"
           << "    \"rhs_contact_projection_half_width_meters\": "
           << kRhsContactProjectionHalfWidthMeters << ",\n"
           << "    \"observation_projection_rule\": "
           << JsonString(
                  "base half width plus previous-to-current body-origin "
                  "chord")
           << ",\n"
           << "    \"carrier_observation_projection_base_half_width_meters\": "
           << kCarrierObservationProjectionBaseHalfWidthMeters << ",\n"
           << "    \"representative_body_observation_projection_base_half_width_meters\": "
           << kRepresentativeBodyObservationProjectionBaseHalfWidthMeters
           << ",\n"
           << "    \"maximum_carrier_observation_projection_half_width_meters\": "
           << maximum_carrier_observation_projection_half_width_meters
           << ",\n"
           << "    \"maximum_representative_body_observation_projection_half_width_meters\": "
           << maximum_representative_body_observation_projection_half_width_meters
           << "\n"
           << "  },\n"
           << "  \"sample_period_nanoseconds\": "
           << clock.sample_period_nanoseconds() << ",\n"
           << "  \"local_sample_refinement\": ";
    if (!clock.local_refinement().has_value()) {
        output << "null";
    } else {
        const auto& refinement = *clock.local_refinement();
        output << "{\"begin_time_nanoseconds\": "
               << refinement.begin_time_nanoseconds
               << ", \"end_time_nanoseconds\": "
               << refinement.end_time_nanoseconds
               << ", \"sample_period_nanoseconds\": "
               << refinement.sample_period_nanoseconds << '}';
    }
    output << ",\n"
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
           << "  \"contact_observation_contract\": {\n"
           << "    \"interface_totals_frame\": \"carrier-projection "
              "Track-T axes\",\n"
           << "    \"contact_point_origin\": \"carrier-projection "
              "Track-T origin; compliant wheel-surface point P\",\n"
           << "    \"local_tangential_forces\": \"one row per patch in "
              "contact_patches.tsv\",\n"
           << "    \"primary_patch_rule\": \"maximum normal force; "
              "summary convenience only\",\n"
           << "    \"maximum_returned_patch_count\": "
           << wheel_rail_contact::kMaxContactPatches << "\n"
           << "  },\n"
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
           << "  \"endpoint_assembly_and_state_slice_diagnostics\": {\n"
           << "    \"generalized_force_residual_inf_norm\": "
           << diagnostics.generalized_force_residual_inf_norm << ",\n"
           << "    \"virtual_power_residual_watts\": "
           << diagnostics.virtual_power_residual_watts << ",\n"
           << "    \"position_derivative_slice_consistency_inf_norm\": "
           << diagnostics.position_derivative_slice_consistency_inf_norm
           << ",\n"
           << "    \"series_force_derivative_slice_consistency_inf_norm\": "
           << diagnostics.series_force_derivative_slice_consistency_inf_norm
           << "\n"
           << "  }\n"
           << "}\n";
    CloseChecked(&output, path);
}

void WritePerformance(const std::filesystem::path& path,
                      const QualificationRunSummary& summary,
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
           << "  \"endpoint_diagnostics_wall_seconds\": "
           << summary.endpoint_diagnostics_wall_seconds << ",\n"
           << "  \"data_and_metadata_write_wall_seconds\": "
           << summary.data_and_metadata_write_wall_seconds << ",\n"
           << "  \"dense_state_bytes\": " << dense_state_bytes << ",\n"
           << "  \"observation_buffer_bytes\": " << observation_bytes
           << ",\n"
           << "  \"integration_statistics\": {\n"
           << "    \"successful_internal_step_count\": "
           << summary.integration_statistics.successful_internal_step_count
           << ",\n"
           << "    \"right_hand_side_evaluation_count\": "
           << summary.integration_statistics.right_hand_side_evaluation_count
           << ",\n"
           << "    \"linear_solver_right_hand_side_evaluation_count\": "
           << summary.integration_statistics
                  .linear_solver_right_hand_side_evaluation_count
           << ",\n"
           << "    \"error_test_failure_count\": "
           << summary.integration_statistics.error_test_failure_count
           << ",\n"
           << "    \"nonlinear_solver_iteration_count\": "
           << summary.integration_statistics.nonlinear_solver_iteration_count
           << ",\n"
           << "    \"nonlinear_solver_convergence_failure_count\": "
           << summary.integration_statistics
                  .nonlinear_solver_convergence_failure_count
           << ",\n"
           << "    \"linear_solver_setup_count\": "
           << summary.integration_statistics.linear_solver_setup_count
           << ",\n"
           << "    \"jacobian_evaluation_count\": "
           << summary.integration_statistics.jacobian_evaluation_count
           << "\n"
           << "  }\n"
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

QualificationRunSummary RunVehicleQualification(
    const QualificationRunConfiguration& run_configuration,
    const VehicleQualificationRecipe& recipe) {
    if (recipe.track_irregularity_requirement ==
            TrackIrregularityRequirement::kRequired &&
        (!run_configuration.track_irregularity_identifier.has_value() ||
         run_configuration.track_irregularity_identifier->empty())) {
        throw std::invalid_argument(
            std::string(recipe.vehicle_label) +
            " dynamics qualification: track-irregularity identifier is "
            "empty");
    }
    if (recipe.track_irregularity_requirement ==
            TrackIrregularityRequirement::kForbidden &&
        run_configuration.track_irregularity_identifier.has_value()) {
        throw std::invalid_argument(
            std::string(recipe.vehicle_label) +
            " dynamics qualification: a track irregularity was supplied to "
            "the explicit no-track-irregularity recipe");
    }
    if (run_configuration.duration_nanoseconds <= 0 ||
        run_configuration.sample_period_nanoseconds <= 0) {
        throw std::invalid_argument(
            std::string(recipe.vehicle_label) +
            " dynamics qualification: duration and sample period must be "
            "positive integer nanoseconds");
    }
    const QualificationSampleClock sample_clock(
        static_cast<std::uint64_t>(run_configuration.duration_nanoseconds),
        static_cast<std::uint64_t>(
            run_configuration.sample_period_nanoseconds),
        run_configuration.local_sample_refinement);
    const std::vector<double> sample_times =
        sample_clock.MakeSampleTimesSeconds();
    const QualificationRunConfiguration resolved_run_configuration =
        ResolveInputPaths(run_configuration);
    AtomicQualificationDirectory output_directory(
        resolved_run_configuration.output_directory);

    const auto vehicle = configuration::LoadVehicleDefinitionFromJsonFile(
        resolved_run_configuration.vehicle_definition_path);
    const auto startup =
        configuration::LoadResolvedStartupStateFromJsonFile(
            resolved_run_configuration.resolved_startup_state_path);
    auto line = configuration::LoadTrackGeometryFromJsonFile(
        resolved_run_configuration.track_geometry_path);
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField> irregularity;
    if (resolved_run_configuration.track_irregularity_identifier.has_value()) {
        irregularity =
            std::make_unique<wheel_rail_contact::TrackIrregularityField>(
                configuration::LoadTrackIrregularityFieldFromDataRoot(
                    resolved_run_configuration.orvd_data_root,
                    *resolved_run_configuration
                         .track_irregularity_identifier));
    }
    std::unique_ptr<AssembledVehicleContactScenario> scenario =
        recipe.assemble_scenario(
            vehicle, startup, std::move(line),
            resolved_run_configuration.orvd_data_root,
            kVehicleReferenceTrackStationMeters,
            kRhsContactProjectionHalfWidthMeters, std::move(irregularity));
    auto& assembled = scenario->vehicle_system();
    auto& accepted = scenario->initial_context().context();
    if (assembled.model().num_generalized_positions() !=
            recipe.expected_generalized_position_count ||
        assembled.model().num_generalized_velocities() !=
            recipe.expected_generalized_velocity_count ||
        assembled.force_plan().series_spring_damper_force_state_count() !=
            recipe.expected_series_force_state_count ||
        assembled.force_plan().body_wrench_count() !=
            recipe.expected_vehicle_wrench_count) {
        Reject(std::string(recipe.vehicle_label) +
               " assembled state or vehicle-wrench topology differs from "
               "its typed qualification recipe");
    }
    if (accepted.time_seconds() != 0.0 || sample_times.front() != 0.0) {
        Reject("the qualification clock and resolved context must start at "
               "exactly zero seconds");
    }

    integrators::SystemContinuousStateAdvancer advancer(
        assembled.system(), assembled.compiled_plan(), accepted,
        MakeTolerances(assembled, recipe),
        integrators::NoCallTimeAppliedForces{});
    const Clock::time_point advance_begin = Clock::now();
    const Eigen::MatrixXd dense_states =
        advancer.AdvanceToWithDenseStateSamples(
            sample_clock.terminal_time_seconds(), sample_times);
    const Clock::time_point advance_end = Clock::now();
    const auto integration_statistics = advancer.integration_statistics();
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
        Reject(std::string("the assembled ") +
               std::string(recipe.vehicle_label) +
               " contact topology is not four carriers and eight interfaces");
    }

    const std::array<multibody_model::RigidBodyHandle, kCarrierCount>
        carrier_bodies{
            assembled.model().GetRigidBodyByName(
                assembled.contact_force_plan()->carrier_name(0)),
            assembled.model().GetRigidBodyByName(
                assembled.contact_force_plan()->carrier_name(1)),
            assembled.model().GetRigidBodyByName(
                assembled.contact_force_plan()->carrier_name(2)),
            assembled.model().GetRigidBodyByName(
                assembled.contact_force_plan()->carrier_name(3))};
    std::array<ProjectionHistory, kCarrierCount> carrier_projection_histories{};
    for (std::size_t index = 0; index < carrier_projection_histories.size();
         ++index) {
        carrier_projection_histories[index].station_seed_meters =
            assembled.contact_force_plan()->initial_projection_station_meters(
                static_cast<int>(index));
        carrier_projection_histories[index].base_half_width_meters =
            kCarrierObservationProjectionBaseHalfWidthMeters;
    }

    const std::array<multibody_model::RigidBodyHandle,
                     kRepresentativeBodyCount>
        representative_bodies{
            assembled.model().GetRigidBodyByName(
                recipe.representative_body_names[0]),
            assembled.model().GetRigidBodyByName(
                recipe.representative_body_names[1]),
            assembled.model().GetRigidBodyByName(
                recipe.representative_body_names[2])};
    std::array<ProjectionHistory, kRepresentativeBodyCount>
        representative_body_projection_histories{};
    for (std::size_t index = 0;
         index < representative_body_projection_histories.size(); ++index) {
        representative_body_projection_histories[index].station_seed_meters =
            InitialBodyTrackStation(vehicle, startup,
                                    recipe.representative_body_names[index]);
        representative_body_projection_histories[index]
            .base_half_width_meters =
            kRepresentativeBodyObservationProjectionBaseHalfWidthMeters;
    }

    std::vector<QualificationObservation> observations;
    observations.reserve(sample_clock.sample_count());
    std::vector<QualificationPatchObservation> patch_observations;
    patch_observations.reserve(sample_clock.sample_count() * kInterfaceCount);
    BoundaryUse before_definition_interval;
    BoundaryUse after_definition_interval;
    std::array<std::size_t, kInterfaceCount> current_zero_contact_runs{};
    std::array<std::size_t, kInterfaceCount> longest_zero_contact_runs{};
    const Clock::time_point observation_begin = Clock::now();
    for (std::size_t sample = 0; sample < sample_clock.sample_count(); ++sample) {
        assembled.system().SetTimeAndContinuousState(
            *observation_context, sample_times[sample],
            dense_states.col(static_cast<Eigen::Index>(sample)));
        auto projection_component =
            assembled.system().GetMultibodyComponentView(
            *observation_context, assembled.system().multibody_component());

        std::array<ProjectionHistory, kCarrierCount>
            pending_carrier_projection_histories =
                carrier_projection_histories;
        std::array<double, kCarrierCount> station_hints{};
        for (std::size_t carrier = 0; carrier < kCarrierCount; ++carrier) {
            const Eigen::Vector3d body_origin_in_inertial_meters =
                assembled.model()
                    .CalcPoseInWorld(projection_component.context(),
                                     carrier_bodies[carrier])
                    .translation();
            station_hints[carrier] = ProjectBodyOriginForObservation(
                assembled, body_origin_in_inertial_meters,
                &pending_carrier_projection_histories[carrier]);
        }
        assembled.system().SetTimeContinuousStateAndWheelRailProjectionHints(
            *observation_context, sample_times[sample],
            dense_states.col(static_cast<Eigen::Index>(sample)),
            station_hints);
        carrier_projection_histories =
            pending_carrier_projection_histories;
        auto component = assembled.system().GetMultibodyComponentView(
            *observation_context, assembled.system().multibody_component());

        assembled.force_plan().CalcAppliedForces(
            component.context(),
            observation_context->series_spring_damper_forces(),
            observation_context->nominal_forces(), vehicle_wrenches,
            series_derivatives);
        assembled.contact_force_plan()->CalcAppliedForcesAndObservations(
            component.context(), *contact_workspace,
            station_hints,
            contact_wrenches, interface_observations);

        QualificationObservation observation;
        observation.sample_index = static_cast<std::uint64_t>(sample);
        observation.time_nanoseconds = sample_clock.TargetTimeNanoseconds(
            static_cast<std::uint64_t>(sample));
        observation.time_seconds = sample_times[sample];
        for (std::size_t carrier = 0; carrier < kCarrierCount; ++carrier) {
            observation.carriers[carrier] = ObserveCarrier(
                assembled, component.context(), carrier_bodies[carrier],
                station_hints[carrier]);
        }
        for (std::size_t interface = 0; interface < kInterfaceCount;
             ++interface) {
            observation.interfaces[interface] =
                SummarizeInterfaceObservation(
                    interface_observations[interface]);
            for (std::size_t patch = 0;
                 patch < interface_observations[interface].contact_patch_count;
                 ++patch) {
                patch_observations.push_back(QualificationPatchObservation{
                    static_cast<std::uint64_t>(sample),
                    observation.time_nanoseconds, observation.time_seconds,
                    interface, patch,
                    interface_observations[interface].patches[patch]});
            }
        }
        std::array<ProjectionHistory, kRepresentativeBodyCount>
            pending_representative_body_projection_histories =
                representative_body_projection_histories;
        for (std::size_t body = 0; body < kRepresentativeBodyCount; ++body) {
            observation.representative_bodies[body] =
                ObserveRepresentativeBody(
                    assembled, component.context(),
                    representative_bodies[body],
                    &pending_representative_body_projection_histories[body]);
        }
        representative_body_projection_histories =
            pending_representative_body_projection_histories;
        for (std::size_t interface = 0; interface < kInterfaceCount;
             ++interface) {
            const auto& value = observation.interfaces[interface];
            RequireFinite(
                value.rail_profile_reference_marker_track_station_meters,
                "rail-profile reference track station");
            RequireFinite(value.vertical_support_force_on_wheel_newtons,
                          "wheel vertical support force");
            RequireFinite(value.normal_force_newtons,
                          "wheel normal force");
            RequireFinite(
                value.total_force_on_wheel_in_carrier_track_frame_newtons,
                          "total wheel force in Track-T");
            RequireFinite(value.primary_patch_normal_force_newtons,
                          "primary-patch normal force");
            RequireFinite(
                value
                    .primary_patch_longitudinal_force_on_wheel_in_contact_frame_newtons,
                "primary-patch longitudinal force");
            RequireFinite(
                value
                    .primary_patch_lateral_force_on_wheel_in_contact_frame_newtons,
                "primary-patch lateral force");
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
    const Clock::time_point observation_end = Clock::now();

    const Clock::time_point endpoint_diagnostics_begin = Clock::now();
    std::vector<AppliedBodyWrench> all_wrenches;
    all_wrenches.reserve(vehicle_wrenches.size() + contact_wrenches.size());
    all_wrenches.insert(all_wrenches.end(), vehicle_wrenches.begin(),
                        vehicle_wrenches.end());
    all_wrenches.insert(all_wrenches.end(), contact_wrenches.begin(),
                        contact_wrenches.end());
    const EndpointDiagnostics endpoint_diagnostics = CalcEndpointDiagnostics(
        assembled, *observation_context, all_wrenches, series_derivatives);
    const Clock::time_point endpoint_diagnostics_end = Clock::now();

    double maximum_carrier_projection_half_width_meters = 0.0;
    for (const ProjectionHistory& history : carrier_projection_histories) {
        maximum_carrier_projection_half_width_meters =
            std::max(maximum_carrier_projection_half_width_meters,
                     history.maximum_half_width_meters);
    }
    double maximum_representative_body_projection_half_width_meters = 0.0;
    for (const ProjectionHistory& history :
         representative_body_projection_histories) {
        maximum_representative_body_projection_half_width_meters = std::max(
            maximum_representative_body_projection_half_width_meters,
            history.maximum_half_width_meters);
    }

    QualificationRunSummary summary;
    summary.sample_count = sample_clock.sample_count();
    summary.advance_wall_seconds =
        ElapsedSeconds(advance_begin, advance_end);
    summary.observation_wall_seconds =
        ElapsedSeconds(observation_begin, observation_end);
    summary.endpoint_diagnostics_wall_seconds = ElapsedSeconds(
        endpoint_diagnostics_begin, endpoint_diagnostics_end);
    summary.endpoint_generalized_force_residual_inf_norm =
        endpoint_diagnostics.generalized_force_residual_inf_norm;
    summary.endpoint_virtual_power_residual_watts =
        endpoint_diagnostics.virtual_power_residual_watts;
    summary.endpoint_position_derivative_slice_consistency_inf_norm =
        endpoint_diagnostics.position_derivative_slice_consistency_inf_norm;
    summary.endpoint_series_force_derivative_slice_consistency_inf_norm =
        endpoint_diagnostics
            .series_force_derivative_slice_consistency_inf_norm;
    summary.integration_statistics = integration_statistics;
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
    WriteObservationHeader(&observation_output, assembled, recipe);
    for (const QualificationObservation& observation : observations) {
        WriteObservation(&observation_output, observation);
    }
    CloseChecked(&observation_output, observation_path);

    const std::filesystem::path patch_observation_path =
        output_directory.working_path() / "contact_patches.tsv";
    std::ofstream patch_observation_output(
        patch_observation_path, std::ios::out | std::ios::trunc);
    if (!patch_observation_output) {
        Reject("could not open '" + patch_observation_path.string() + "'");
    }
    patch_observation_output << std::setprecision(17);
    WritePatchObservationHeader(&patch_observation_output);
    for (const QualificationPatchObservation& observation :
         patch_observations) {
        WritePatchObservation(&patch_observation_output, observation,
                              assembled);
    }
    CloseChecked(&patch_observation_output, patch_observation_path);

    WriteMetadata(output_directory.working_path() / "metadata.json",
                  resolved_run_configuration, recipe, startup, assembled,
                  sample_clock, before_definition_interval,
                  after_definition_interval,
                  longest_zero_contact_runs,
                  maximum_carrier_projection_half_width_meters,
                  maximum_representative_body_projection_half_width_meters,
                  endpoint_diagnostics);
    const Clock::time_point write_end = Clock::now();
    summary.data_and_metadata_write_wall_seconds =
        ElapsedSeconds(write_begin, write_end);
    WritePerformance(
        output_directory.working_path() / "performance.json", summary,
        static_cast<std::size_t>(dense_states.size()) * sizeof(double),
        observations.capacity() * sizeof(QualificationObservation) +
            patch_observations.capacity() *
                sizeof(QualificationPatchObservation));
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

}  // namespace orvd::dynamics_qualification::internal
