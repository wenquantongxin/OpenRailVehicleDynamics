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
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
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
#include "qualification_integration_configuration.h"
#include "qualification_continuous_state_writer.h"
#include "qualification_integration_run.h"
#include "qualification_sample_clock.h"
#include "strict_floating_point_qualification.h"

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_track_irregularity_field.h"
#include "orvd/configuration/load_vehicle_definition.h"
#include "orvd/forces/wheel_rail_contact_force_plan.h"
#include "orvd/integrators/system_continuous_state_advancer.h"
#include "orvd/multibody_model/multibody_applied_forces.h"
#include "orvd/multibody_model/multibody_model.h"
#include "orvd/scene_observation/body_state.h"
#include "orvd/scene_observation/scene_frame.h"
#include "orvd/scene_observation/scene_topology.h"
#include "orvd/scene_observation/track_sampling.h"
#include "orvd/scene_observation/wheel_spin_sampling.h"
#include "orvd/scene_record/scene_record_writer.h"
#include "orvd/track_geometry/track_geometry.h"
#include "orvd/wheel_rail_contact/roll_yaw_pitch.h"

namespace orvd::dynamics_qualification::internal {
namespace {

using Clock = std::chrono::steady_clock;
using configuration::AssembledVehicleContactScenario;
using forces::WheelRailContactInterfaceObservation;
using forces::WheelRailContactPatchObservation;
using multibody_model::AppliedBodyWrench;
using track_geometry::TrackStationRegion;

constexpr double kVehicleReferenceTrackStationMeters = 0.0;
constexpr std::size_t kCarrierCount = 4;
constexpr std::size_t kInterfaceCount = 8;

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

struct ProjectionHistory final {
    double station_seed_meters{};
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

[[nodiscard]] double ProjectBodyOriginForObservation(
    const configuration::AssembledVehicleSystem& assembled,
    const Eigen::Vector3d& body_origin_in_inertial_meters,
    ProjectionHistory* history) {
    const auto projection = assembled.contact_force_plan()
                                ->track_geometry()
                                .ProjectPointOntoSeededBranch(
                                    body_origin_in_inertial_meters,
                                    history->station_seed_meters);
    history->station_seed_meters = projection.track_station_meters();
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

// The optional scene record: the same dense samples and observation context
// the qualification observes, sampled once more as world-frame body states and
// a few named scalars. It adds no integrator stop and no RHS evaluation.
struct SceneRecordExport final {
    SceneRecordExport(const configuration::AssembledVehicleSystem& assembled,
                      const std::filesystem::path& directory,
                      scene_observation::SceneTopology topology,
                      std::vector<scene_observation::ScalarDefinition>
                          scalar_definitions,
                      std::string visual_definition_json)
        : sampler(assembled.model()),
          spin_sampler(assembled.model(), topology),
          wheel_body_slots(ResolveWheelBodySlots(topology)),
          writer(directory, std::move(topology),
                 std::move(scalar_definitions),
                 std::move(visual_definition_json),
                 spin_sampler.available()) {}

    [[nodiscard]] static std::vector<std::size_t> ResolveWheelBodySlots(
        const scene_observation::SceneTopology& topology) {
        std::vector<std::size_t> slots;
        for (const auto& wheel : topology.wheel_placements) {
            for (std::size_t slot = 0; slot < topology.bodies.size(); ++slot) {
                if (topology.bodies[slot].name == wheel.wheel_body_name) {
                    slots.push_back(slot);
                    break;
                }
            }
        }
        return slots;
    }

    scene_observation::BodyStateSampler sampler;
    scene_observation::WheelSpinAngleSampler spin_sampler;
    std::vector<std::size_t> wheel_body_slots;
    scene_record::SceneRecordWriter writer;
    // The largest wheel rotation, in radians, that the recorded angular
    // velocities imply between two consecutive samples. Above a half turn a
    // display cannot pick the rotation branch from the orientations alone.
    double maximum_wheel_rotation_between_samples_radians{0.0};
    double previous_sample_time_seconds{0.0};
    double minimum_carrier_station_meters{
        std::numeric_limits<double>::infinity()};
    double maximum_carrier_station_meters{
        -std::numeric_limits<double>::infinity()};
    double wall_seconds{0.0};
};

[[nodiscard]] std::string ReadVisualDefinitionText(
    const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input) {
        Reject("the scene record needs the parametric visual definition '" +
               path.string() + "', which could not be opened");
    }
    std::ostringstream text;
    text << input.rdbuf();
    return text.str();
}

[[nodiscard]] std::vector<scene_observation::ScalarDefinition>
MakeSceneScalarDefinitions(
    const configuration::AssembledVehicleSystem& assembled,
    const VehicleQualificationRecipe& recipe) {
    using scene_observation::ScalarDefinition;
    constexpr const char* kDenseSample =
        "dense integrator sample replayed in a private observation context";
    constexpr const char* kCarrierTrackFrame =
        "Track-T at the body's projected station; +y right, +z down";
    std::vector<ScalarDefinition> definitions;
    const auto* contact_plan = assembled.contact_force_plan();
    const auto add_track_coordinates = [&](const std::string& body) {
        definitions.push_back(ScalarDefinition{
            body + ".track_station_meters", "m", "track inertial frame",
            "planar projected track station of the body origin",
            "seeded local centerline projection", kDenseSample});
        definitions.push_back(ScalarDefinition{
            body + ".lateral_meters", "m", kCarrierTrackFrame,
            "lateral position of the body origin",
            "body origin expressed in Track-T", kDenseSample});
        definitions.push_back(ScalarDefinition{
            body + ".yaw_radians", "rad", kCarrierTrackFrame,
            "yaw of the body about the track vertical",
            "X-Z-Y roll-yaw-pitch resolution of R_TB in the ORVD body basis",
            kDenseSample});
    };
    for (int carrier = 0; carrier < contact_plan->carrier_count(); ++carrier) {
        add_track_coordinates(std::string(contact_plan->carrier_name(carrier)));
    }
    for (int interface = 0; interface < contact_plan->interface_count();
         ++interface) {
        const std::string name(contact_plan->interface_name(interface));
        definitions.push_back(ScalarDefinition{
            name + ".contact_patch_count", "1", "wheel-rail contact evaluation",
            "number of loaded contact patches",
            "wheel-rail contact evaluated at the sample state", kDenseSample});
        definitions.push_back(ScalarDefinition{
            name + ".vertical_support_force_on_wheel_newtons", "N",
            "Track-T at the carrier projection station; positive upward "
            "support",
            "vertical support force on the wheel",
            "minus the z component of the patch force total", kDenseSample});
        definitions.push_back(ScalarDefinition{
            name + ".normal_force_newtons", "N", "patch normal directions",
            "total normal force on the wheel", "sum over loaded patches",
            kDenseSample});
    }
    for (const std::string_view body : recipe.representative_body_names) {
        add_track_coordinates(std::string(body));
    }
    return definitions;
}

void FillSceneScalarValues(const QualificationObservation& observation,
                           scene_observation::ScalarValues* values) {
    values->values.clear();
    for (const CarrierObservation& carrier : observation.carriers) {
        values->values.push_back(carrier.track_station_meters);
        values->values.push_back(carrier.lateral_meters);
        values->values.push_back(carrier.yaw_radians);
    }
    for (const QualificationInterfaceObservation& interface :
         observation.interfaces) {
        values->values.push_back(
            static_cast<double>(interface.contact_patch_count));
        values->values.push_back(
            interface.vertical_support_force_on_wheel_newtons);
        values->values.push_back(interface.normal_force_newtons);
    }
    for (const RepresentativeBodyObservation& body :
         observation.representative_bodies) {
        values->values.push_back(body.track_station_meters);
        values->values.push_back(body.lateral_meters);
        values->values.push_back(body.yaw_radians);
    }
    values->statuses.assign(
        values->values.size(),
        static_cast<std::uint8_t>(scene_observation::ScalarStatus::kValid));
}

[[nodiscard]] scene_observation::SamplePhase SceneSamplePhase(
    std::size_t sample, std::size_t sample_count) {
    if (sample == 0) {
        return scene_observation::SamplePhase::kInitialAcceptedState;
    }
    if (sample + 1 == sample_count) {
        return scene_observation::SamplePhase::kAcceptedEndpoint;
    }
    return scene_observation::SamplePhase::kDenseIntermediateSample;
}

// Samples the line over the stations the carriers actually visited plus a
// margin that covers the vehicle's own length, so the short record carries
// its own track without describing the whole line.
void WriteSceneTrack(const configuration::AssembledVehicleSystem& assembled,
                     SceneRecordExport* scene_export) {
    constexpr double kMarginMeters = 25.0;
    constexpr double kSpacingMeters = 0.5;
    const auto* contact_plan = assembled.contact_force_plan();
    const double begin =
        scene_export->minimum_carrier_station_meters - kMarginMeters;
    const double end =
        scene_export->maximum_carrier_station_meters + kMarginMeters;
    if (!std::isfinite(begin) || !std::isfinite(end) || !(end > begin)) {
        Reject("the scene record observed no finite carrier station range");
    }
    const std::size_t count =
        static_cast<std::size_t>(std::floor((end - begin) / kSpacingMeters)) +
        1U;
    std::vector<double> stations(count);
    for (std::size_t index = 0; index < count; ++index) {
        stations[index] =
            begin + static_cast<double>(index) * kSpacingMeters;
    }
    const auto& left =
        contact_plan->pose_constants(wheel_rail_contact::WheelSide::kLeft);
    const auto& right =
        contact_plan->pose_constants(wheel_rail_contact::WheelSide::kRight);
    scene_observation::TrackSampleTable table;
    scene_observation::SampleTrackGeometry(
        contact_plan->track_geometry(), stations,
        scene_observation::RailDatumPlacement{
            left.rail_lateral_datum_meters, left.rail_vertical_datum_meters},
        scene_observation::RailDatumPlacement{
            right.rail_lateral_datum_meters, right.rail_vertical_datum_meters},
        table);
    scene_export->writer.WriteTrackSampleTable(table);
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

nlohmann::json PhysicalObservationContract(
    const configuration::AssembledVehicleSystem& assembled) {
    nlohmann::json carriers = nlohmann::json::array();
    nlohmann::json interfaces = nlohmann::json::array();
    const auto& contact = *assembled.contact_force_plan();
    for (int i = 0; i < contact.carrier_count(); ++i) {
        const std::string name(contact.carrier_name(i));
        carriers.push_back({{"name", name},
                            {"station_column", name + ".track_station_meters"},
                            {"lateral_column", name + ".lateral_meters"},
                            {"yaw_column", name + ".yaw_radians"},
                            {"yaw_basis", "assembled_carrier_body_relative_to_track_T"}});
    }
    for (int i = 0; i < contact.interface_count(); ++i) {
        const std::string name(contact.interface_name(i));
        const std::string prefix = name + ".";
        interfaces.push_back({{"name", name},
            {"station_column", prefix + "rail_profile_reference_marker_track_station_meters"},
            {"patch_count_column", prefix + "contact_patch_count"},
            {"normal_force_column", prefix + "normal_force_newtons"},
            {"support_force_column", prefix + "vertical_support_force_on_wheel_newtons"},
            {"force_columns", {{"x", prefix + "total_force_on_wheel_in_carrier_track_frame_x_newtons"},
                               {"y", prefix + "total_force_on_wheel_in_carrier_track_frame_y_newtons"},
                               {"z", prefix + "total_force_on_wheel_in_carrier_track_frame_z_newtons"}}}});
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
    const QualificationRunConfiguration& configuration,
    const VehicleQualificationRecipe& recipe,
    const nlohmann::json& numerical_metadata,
    const configuration::ResolvedStartupState& startup,
    const configuration::AssembledVehicleSystem& assembled,
    const QualificationSampleClock& clock, const BoundaryUse& before,
    const BoundaryUse& after,
    const std::array<std::size_t, kInterfaceCount>& longest_zero_contact_runs) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) {
        Reject("could not open '" + path.string() + "'");
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"completed\": true,\n"
           << "  \"artifact_schema_identifier\": "
           << JsonString("orvd.passive_vehicle_qualification.v2") << ",\n"
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
           << "  \"continuous_state_observation_contract\": "
           << ContinuousStateObservationContract(clock).dump() << ",\n"
           << "  \"physical_observation_contract\": "
           << PhysicalObservationContract(assembled).dump() << ",\n"
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
           << JsonString(kStrictFloatingPointSemanticsIdentifier) << ",\n"
           << "      \"cmake_external_flag_audit_passed\": true,\n"
           << "      \"compile_command_audit_enabled\": true,\n"
           << "      \"fast_math_macro_defined\": false,\n"
           << "      \"finite_math_only_enabled\": false,\n"
           << "      \"build_type\": "
           << JsonString(kQualificationBuildType) << ",\n"
           << "      \"compiler_id\": "
           << JsonString(kQualificationCxxCompilerId) << ",\n"
           << "      \"compiler_version\": "
           << JsonString(kQualificationCxxCompilerVersion) << "\n"
           << "    }\n"
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
    output << "]\n}\n";
    CloseChecked(&output, path);
}

void WritePerformance(const std::filesystem::path& path,
                      const QualificationRunSummary& summary) {
    const nlohmann::json result{
        {"integrator_recipe_identifier", summary.integrator_recipe_identifier},
        {"advance_wall_seconds", summary.advance_wall_seconds},
        {"observation_wall_seconds", summary.observation_wall_seconds},
        {"scene_record_frame_count", summary.scene_record_frame_count},
        {"scene_record_wall_seconds", summary.scene_record_wall_seconds},
        {"data_and_metadata_write_wall_seconds", summary.data_and_metadata_write_wall_seconds},
        {"numerical_timings", summary.numerical_timings.ToJson()},
        {"integration_work", summary.integration_work.ToJson()}};
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) Reject("could not open '" + path.string() + "'");
    output << result.dump(2) << '\n';
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
            run_configuration.sample_period_nanoseconds));
    const std::vector<double> sample_times =
        sample_clock.MakeSampleTimesSeconds();
    const QualificationRunConfiguration resolved_run_configuration =
        ResolveInputPaths(run_configuration);
    const auto integration_request = RequestIntegrationConfiguration(
        run_configuration.integration_config_path);
    RequireFailureResultDestinationAvailable(resolved_run_configuration.output_directory);
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
            std::move(irregularity));
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
    Eigen::VectorXd initial_continuous_state(
        assembled.system().continuous_state_size());
    assembled.system().CopyContinuousState(accepted,
                                           initial_continuous_state);

    auto numerics = ResolveIntegrationConfiguration(
        integration_request, assembled, initial_continuous_state,
        ExplicitOdeRequest{recipe.default_integration_recipe, recipe.relative_tolerance,
            recipe.generalized_position_absolute_tolerance,
            recipe.generalized_velocity_absolute_tolerance,
            recipe.series_force_absolute_tolerance_newtons});
    numerics.metadata["execution_conditions_at_start"] =
        CaptureQualificationExecutionConditions(
            *assembled.contact_force_plan());
    auto failure_metadata = numerics.metadata;
    failure_metadata["input_paths"] = {
        {"vehicle_definition", resolved_run_configuration.vehicle_definition_path.string()},
        {"resolved_startup_state", resolved_run_configuration.resolved_startup_state_path.string()},
        {"track_geometry", resolved_run_configuration.track_geometry_path.string()},
        {"orvd_data_root", resolved_run_configuration.orvd_data_root.string()}};
    failure_metadata["qualification_vehicle_recipe"] = recipe.vehicle_label;
    failure_metadata["sample_period_nanoseconds"] = run_configuration.sample_period_nanoseconds;
    failure_metadata["track_irregularity_identifier"] =
        run_configuration.track_irregularity_identifier
            ? nlohmann::json(*run_configuration.track_irregularity_identifier) : nlohmann::json(nullptr);
    QualificationIntegrationRun integration(
        resolved_run_configuration.output_directory, std::move(failure_metadata),
        std::move(numerics.configuration), assembled, accepted,
        sample_clock.terminal_time_seconds());
    const auto method_identifier = integration.advancer().method_identifier();
    const Eigen::MatrixXd dense_states = integration.Advance(
        sample_clock.terminal_time_seconds(), sample_times);
    if (dense_states.rows() != assembled.system().continuous_state_size() ||
        dense_states.cols() !=
            static_cast<Eigen::Index>(sample_clock.sample_count()) ||
        !dense_states.allFinite() ||
        accepted.time_seconds() != sample_clock.terminal_time_seconds()) {
        Reject("the dense state batch is incomplete, non-finite or detached "
               "from its accepted endpoint");
    }
    Eigen::VectorXd accepted_terminal_state(
        assembled.system().continuous_state_size());
    assembled.system().CopyContinuousState(accepted,
                                           accepted_terminal_state);
    if (dense_states.col(0) != initial_continuous_state ||
        dense_states.col(dense_states.cols() - 1) !=
            accepted_terminal_state) {
        Reject("the dense state batch did not preserve its exact initial or "
               "accepted terminal state");
    }

    auto observation_context = assembled.system().CreateDefaultRuntimeContext(0.0);
    assembled.system().CopyContextLocalData(
        scenario->initial_context().context(), *observation_context);
    auto contact_workspace = assembled.contact_force_plan()->CreateWorkspace();
    std::vector<AppliedBodyWrench> contact_wrenches(
        static_cast<std::size_t>(
            assembled.contact_force_plan()->body_wrench_count()));
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
    }

    std::optional<SceneRecordExport> scene_export;
    if (run_configuration.publish_scene_record) {
        const std::filesystem::path visual_definition_path =
            resolved_run_configuration.vehicle_definition_path.parent_path() /
            "visualization" / "visual_definition.json";
        scene_export.emplace(
            assembled, output_directory.working_path() / "scene_record",
            scene_observation::DescribeSceneTopology(
                assembled.model(), assembled.contact_force_plan()),
            MakeSceneScalarDefinitions(assembled, recipe),
            ReadVisualDefinitionText(visual_definition_path));
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
        if (scene_export.has_value()) {
            const Clock::time_point scene_begin = Clock::now();
            scene_observation::SceneFrame frame;
            frame.identity.time_seconds = sample_times[sample];
            frame.identity.time_nanoseconds =
                static_cast<std::int64_t>(observation.time_nanoseconds);
            frame.identity.sample_index = static_cast<std::int64_t>(sample);
            frame.identity.phase =
                SceneSamplePhase(sample, sample_clock.sample_count());
            frame.bodies = scene_export->sampler.Sample(component.context());
            if (scene_export->spin_sampler.available()) {
                frame.wheel_spin_angles_radians.resize(
                    scene_export->spin_sampler.wheel_count());
                scene_export->spin_sampler.Sample(
                    component.context(), frame.wheel_spin_angles_radians);
            }
            FillSceneScalarValues(observation, &frame.scalars);
            if (sample > 0) {
                const double interval =
                    sample_times[sample] -
                    scene_export->previous_sample_time_seconds;
                for (const std::size_t slot : scene_export->wheel_body_slots) {
                    const auto& omega =
                        frame.bodies[slot].angular_velocity_radians_per_second;
                    const double rate = std::sqrt(omega[0] * omega[0] +
                                                  omega[1] * omega[1] +
                                                  omega[2] * omega[2]);
                    scene_export->maximum_wheel_rotation_between_samples_radians =
                        std::max(scene_export
                                     ->maximum_wheel_rotation_between_samples_radians,
                                 rate * interval);
                }
            }
            scene_export->previous_sample_time_seconds = sample_times[sample];
            scene_export->writer.WriteFrame(frame);
            for (const CarrierObservation& carrier : observation.carriers) {
                scene_export->minimum_carrier_station_meters =
                    std::min(scene_export->minimum_carrier_station_meters,
                             carrier.track_station_meters);
                scene_export->maximum_carrier_station_meters =
                    std::max(scene_export->maximum_carrier_station_meters,
                             carrier.track_station_meters);
            }
            scene_export->wall_seconds +=
                ElapsedSeconds(scene_begin, Clock::now());
        }
    }
    if (scene_export.has_value()) {
        const Clock::time_point scene_begin = Clock::now();
        WriteSceneTrack(assembled, &*scene_export);
        scene_export->writer.Close();
        scene_export->wall_seconds += ElapsedSeconds(scene_begin, Clock::now());
        if (!scene_export->spin_sampler.available() &&
            scene_export->maximum_wheel_rotation_between_samples_radians >
                0.5 * std::numbers::pi) {
            std::fprintf(
                stderr,
                "warning: the scene record samples wheel orientations up to "
                "%.3f rad apart and carries no unwrapped spin angles; a display "
                "cannot choose the spin branch between such samples\n",
                scene_export->maximum_wheel_rotation_between_samples_radians);
        }
    }
    const Clock::time_point observation_end = Clock::now();

    QualificationRunSummary summary(method_identifier);
    if (method_identifier == "cvode_bdf2") summary.maximum_bdf_order = 2;
    if (method_identifier == "cvode_bdf5") summary.maximum_bdf_order = 5;
    summary.sample_count = sample_clock.sample_count();
    summary.advance_wall_seconds =
        integration.timings().advance_wall_seconds;
    summary.observation_wall_seconds =
        ElapsedSeconds(observation_begin, observation_end);
    if (scene_export.has_value()) {
        summary.scene_record_frame_count = scene_export->writer.frame_count();
        summary.scene_record_wall_seconds = scene_export->wall_seconds;
    }
    summary.integration_work = integration.ledger();
    summary.numerical_timings = integration.timings();
    summary.terminal_continuous_state.resize(
        assembled.system().continuous_state_size());
    assembled.system().CopyContinuousState(
        accepted, summary.terminal_continuous_state);
    summary.used_before_track_definition_interval =
        before_definition_interval.observed;
    summary.used_after_track_definition_interval =
        after_definition_interval.observed;

    const Clock::time_point write_begin = Clock::now();
    WriteQualificationContinuousStates(
        output_directory.working_path() / "continuous_states.tsv",
        sample_clock, sample_times, dense_states,
        assembled.model().num_generalized_positions(),
        assembled.model().num_generalized_velocities(),
        assembled.force_plan().series_spring_damper_force_state_count());

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
                  resolved_run_configuration, recipe, numerics.metadata,
                  startup, assembled,
                  sample_clock, before_definition_interval,
                  after_definition_interval,
                  longest_zero_contact_runs);
    const Clock::time_point write_end = Clock::now();
    summary.data_and_metadata_write_wall_seconds =
        ElapsedSeconds(write_begin, write_end);
    WritePerformance(output_directory.working_path() / "performance.json", summary);
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
