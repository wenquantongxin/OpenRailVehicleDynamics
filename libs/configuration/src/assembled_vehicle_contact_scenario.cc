#include "orvd/configuration/assembled_vehicle_contact_scenario.h"

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "orvd/configuration/gz18_wheel_rail_contact.h"
#include "orvd/configuration/irw_wheel_rail_contact.h"
#include "resolved_startup_state_invariants.h"
#include "vehicle_definition_invariants.h"

namespace orvd::configuration {
namespace {

using forces::WheelRailContactCarrierDefinition;
using forces::WheelRailContactInterfaceDefinition;
using forces::IndependentWheelActiveTorqueCoupleDefinition;
using wheel_rail_contact::WheelSide;

struct FrozenContactTopology {
    std::vector<WheelRailContactCarrierDefinition> carriers;
    std::vector<WheelRailContactInterfaceDefinition> interfaces;
    std::vector<IndependentWheelActiveTorqueCoupleDefinition>
        active_torque_couples;
};

[[noreturn]] void Reject(std::string_view scenario,
                         const std::string& detail) {
    throw std::invalid_argument(std::string(scenario) +
                                " contact scenario: " + detail);
}

struct StationOffsets {
    std::unordered_map<std::string, double> mechanical;
    std::unordered_map<std::string, double> resolved;
};

StationOffsets CaptureStationOffsets(
    const VehicleDefinition& vehicle,
    const ResolvedStartupState& startup_state) {
    StationOffsets offsets;
    const auto& layout = vehicle.mechanical_track_station_layout;
    offsets.mechanical.reserve(layout.free_body_station_offsets.size());
    for (const VehicleFreeBodyStationOffsetDefinition& entry :
         layout.free_body_station_offsets) {
        offsets.mechanical.emplace(entry.body_name,
                                   entry.station_offset_meters);
    }
    offsets.resolved.reserve(startup_state.free_body_startup_states.size());
    for (const FreeBodyStartupState& entry :
         startup_state.free_body_startup_states) {
        offsets.resolved.emplace(
            entry.body_name,
            entry.resolved_track_station_offset_from_mechanical_layout_meters);
    }
    return offsets;
}

WheelRailContactCarrierDefinition MakeCarrier(
    std::string_view scenario, std::string_view body_name,
    const StationOffsets& offsets,
    double vehicle_layout_reference_track_station_meters,
    const Eigen::Matrix3d&
        rotation_body_from_nonspinning_wheel_profile =
            Eigen::Matrix3d::Identity()) {
    const auto mechanical = offsets.mechanical.find(std::string(body_name));
    const auto resolved = offsets.resolved.find(std::string(body_name));
    if (mechanical == offsets.mechanical.end() ||
        resolved == offsets.resolved.end()) {
        Reject(scenario, "station carrier '" + std::string(body_name) +
                             "' is missing from the mechanical layout or "
                             "resolved free-body start-up family");
    }
    return WheelRailContactCarrierDefinition{
        std::string(body_name), std::string(body_name),
        vehicle_layout_reference_track_station_meters + mechanical->second +
            resolved->second,
        rotation_body_from_nonspinning_wheel_profile};
}

FrozenContactTopology BuildGz18ContactTopology(
    const VehicleDefinition& vehicle,
    const ResolvedStartupState& startup_state,
    double vehicle_layout_reference_track_station_meters) {
    constexpr std::string_view kScenario = "GZ18";
    if (!std::isfinite(vehicle_layout_reference_track_station_meters)) {
        Reject(kScenario,
               "the vehicle layout reference track station is not finite");
    }
    const auto& wheelsets = vehicle.mechanical_track_station_layout
                                .wheel_pair_station_reference_body_names;
    if (wheelsets.size() != 4) {
        Reject(kScenario,
               "the mechanical layout must name four wheel-pair station "
               "reference bodies");
    }

    const StationOffsets offsets = CaptureStationOffsets(vehicle,
                                                          startup_state);
    FrozenContactTopology topology;
    topology.carriers.reserve(wheelsets.size());
    topology.interfaces.reserve(2 * wheelsets.size());
    for (const std::string& wheelset : wheelsets) {
        topology.carriers.push_back(MakeCarrier(
            kScenario, wheelset, offsets,
            vehicle_layout_reference_track_station_meters));
        topology.interfaces.push_back(WheelRailContactInterfaceDefinition{
            .interface_name = wheelset + ".right",
            .carrier_name = wheelset,
            .wheel_body_name = wheelset,
            .side = WheelSide::kRight,
            .independent_wheel_revolute_joint = std::nullopt,
        });
        topology.interfaces.push_back(WheelRailContactInterfaceDefinition{
            .interface_name = wheelset + ".left",
            .carrier_name = wheelset,
            .wheel_body_name = wheelset,
            .side = WheelSide::kLeft,
            .independent_wheel_revolute_joint = std::nullopt,
        });
    }
    return topology;
}

struct IrwInterfaceTopology {
    std::string_view interface_name;
    std::string_view carrier_body_name;
    std::string_view wheel_body_name;
    std::string_view revolute_joint_name;
    std::string_view reaction_frame_body_name;
    WheelSide side;
};

constexpr std::array<std::string_view, 4> kIrwCarrierNames{
    "axlebridge_ff", "axlebridge_fr", "axlebridge_rf", "axlebridge_rr"};

// Frozen WRL order: each axle from front to rear, left before right.
constexpr std::array<IrwInterfaceTopology, 8> kIrwInterfaces{{
    {"wheel_ff_l", "axlebridge_ff", "wheel_ff_l", "rev_wheel_ff_l",
     "frame_front",
     WheelSide::kLeft},
    {"wheel_ff_r", "axlebridge_ff", "wheel_ff_r", "rev_wheel_ff_r",
     "frame_front",
     WheelSide::kRight},
    {"wheel_fr_l", "axlebridge_fr", "wheel_fr_l", "rev_wheel_fr_l",
     "frame_front",
     WheelSide::kLeft},
    {"wheel_fr_r", "axlebridge_fr", "wheel_fr_r", "rev_wheel_fr_r",
     "frame_front",
     WheelSide::kRight},
    {"wheel_rf_l", "axlebridge_rf", "wheel_rf_l", "rev_wheel_rf_l",
     "frame_rear",
     WheelSide::kLeft},
    {"wheel_rf_r", "axlebridge_rf", "wheel_rf_r", "rev_wheel_rf_r",
     "frame_rear",
     WheelSide::kRight},
    {"wheel_rr_l", "axlebridge_rr", "wheel_rr_l", "rev_wheel_rr_l",
     "frame_rear",
     WheelSide::kLeft},
    {"wheel_rr_r", "axlebridge_rr", "wheel_rr_r", "rev_wheel_rr_r",
     "frame_rear",
     WheelSide::kRight},
}};

void RequireIrwMechanicalTopology(const VehicleDefinition& vehicle) {
    constexpr std::string_view kScenario = "IRW";
    const auto& carrier_names = vehicle.mechanical_track_station_layout
                                    .wheel_pair_station_reference_body_names;
    if (carrier_names.size() != kIrwCarrierNames.size()) {
        Reject(kScenario,
               "the mechanical layout must name exactly four axle-bridge "
               "station carriers");
    }
    std::unordered_set<std::string> stated_carriers;
    stated_carriers.reserve(carrier_names.size());
    for (const std::string& carrier_name : carrier_names) {
        stated_carriers.emplace(carrier_name);
    }
    for (const std::string_view expected : kIrwCarrierNames) {
        if (!stated_carriers.contains(std::string(expected))) {
            Reject(kScenario,
                   "the mechanical layout does not name the required "
                   "station carrier '" + std::string(expected) + "'");
        }
    }

    if (vehicle.revolute_joints.size() != kIrwInterfaces.size()) {
        Reject(kScenario,
               "the mechanical definition must contain exactly the eight "
               "independent-wheel revolute joints");
    }
    std::unordered_map<std::string,
                       const VehicleRevoluteJointDefinition*>
        joints_by_name;
    joints_by_name.reserve(vehicle.revolute_joints.size());
    for (const VehicleRevoluteJointDefinition& joint :
         vehicle.revolute_joints) {
        if (!joints_by_name.emplace(joint.name, &joint).second) {
            Reject(kScenario, "duplicate revolute joint name '" + joint.name +
                                  "'");
        }
    }
    for (const IrwInterfaceTopology& expected : kIrwInterfaces) {
        const auto found =
            joints_by_name.find(std::string(expected.revolute_joint_name));
        if (found == joints_by_name.end()) {
            Reject(kScenario,
                   "missing independent-wheel revolute joint '" +
                       std::string(expected.revolute_joint_name) + "'");
        }
        const VehicleRevoluteJointDefinition& joint = *found->second;
        const bool is_positive_y_axis =
            joint.axis_in_parent_frame.x() == 0.0 &&
            joint.axis_in_parent_frame.y() == 1.0 &&
            joint.axis_in_parent_frame.z() == 0.0;
        if (joint.parent_frame_name != expected.carrier_body_name ||
            joint.child_frame_name != expected.wheel_body_name ||
            !is_positive_y_axis) {
            Reject(kScenario,
                   "revolute joint '" + joint.name +
                       "' is not the required axle-bridge-to-wheel +Y "
                       "relation");
        }
    }
}

FrozenContactTopology BuildIrwContactTopology(
    const VehicleDefinition& vehicle,
    const ResolvedStartupState& startup_state,
    double vehicle_layout_reference_track_station_meters) {
    constexpr std::string_view kScenario = "IRW";
    if (!std::isfinite(vehicle_layout_reference_track_station_meters)) {
        Reject(kScenario,
               "the vehicle layout reference track station is not finite");
    }
    RequireIrwMechanicalTopology(vehicle);
    const StationOffsets offsets = CaptureStationOffsets(vehicle,
                                                          startup_state);

    FrozenContactTopology topology;
    topology.carriers.reserve(kIrwCarrierNames.size());
    topology.interfaces.reserve(kIrwInterfaces.size());
    topology.active_torque_couples.reserve(kIrwInterfaces.size());
    // The frozen WRL straight Track-T basis and the IRW axle-bridge body basis
    // differ by C = diag(1,-1,-1). ORVD's Track-T basis is already the
    // physical track basis, so this fixed R_BP keeps the non-spinning wheel
    // profile upright without rotating it by the wheel's joint phase.
    const Eigen::Matrix3d rotation_body_from_profile =
        Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
    for (const std::string_view carrier : kIrwCarrierNames) {
        topology.carriers.push_back(MakeCarrier(
            kScenario, carrier, offsets,
            vehicle_layout_reference_track_station_meters,
            rotation_body_from_profile));
    }
    for (const IrwInterfaceTopology& interface : kIrwInterfaces) {
        topology.interfaces.push_back(WheelRailContactInterfaceDefinition{
            .interface_name = std::string(interface.interface_name),
            .carrier_name = std::string(interface.carrier_body_name),
            .wheel_body_name = std::string(interface.wheel_body_name),
            .side = interface.side,
            .independent_wheel_revolute_joint =
                forces::IndependentWheelRevoluteJointDefinition{
                    std::string(interface.revolute_joint_name)},
        });
        topology.active_torque_couples.push_back(
            IndependentWheelActiveTorqueCoupleDefinition{
                .channel_name = std::string(interface.interface_name),
                .axis_provider_body_name =
                    std::string(interface.carrier_body_name),
                .wheel_body_name = std::string(interface.wheel_body_name),
                .reaction_frame_body_name =
                    std::string(interface.reaction_frame_body_name),
            });
    }
    return topology;
}

void RequireAnchorsMatchResolvedPlacements(
    std::string_view scenario,
    const forces::WheelRailContactForcePlan& contact_plan,
    const ResolvedInitialContext& initial_context) {
    std::unordered_map<std::string, double> resolved_station;
    resolved_station.reserve(initial_context.wheel_pair_placements().size());
    for (const ResolvedWheelPairPlacement& placement :
         initial_context.wheel_pair_placements()) {
        resolved_station.emplace(placement.station_reference_body_name,
                                 placement.track_station_meters);
    }
    if (contact_plan.carrier_count() !=
        static_cast<int>(resolved_station.size())) {
        throw std::logic_error(
            std::string(scenario) +
            " contact scenario: contact carriers and resolved wheel-pair "
            "placements differ in count");
    }
    for (int ordinal = 0; ordinal < contact_plan.carrier_count(); ++ordinal) {
        const std::string name(contact_plan.carrier_name(ordinal));
        const auto placement = resolved_station.find(name);
        if (placement == resolved_station.end() ||
            placement->second !=
                contact_plan.initial_projection_station_meters(ordinal)) {
            throw std::logic_error(
                std::string(scenario) +
                " contact scenario: initial projection anchor for '" + name +
                "' differs from its resolved wheel-pair station");
        }
    }
}

}  // namespace

AssembledVehicleContactScenario::AssembledVehicleContactScenario(
    std::unique_ptr<AssembledVehicleSystem> vehicle_system,
    ResolvedInitialContext initial_context)
    : vehicle_system_(std::move(vehicle_system)),
      initial_context_(std::move(initial_context)) {}

AssembledVehicleContactScenario::~AssembledVehicleContactScenario() = default;

std::unique_ptr<AssembledVehicleContactScenario>
AssembleGz18ContactScenario(
    const VehicleDefinition& vehicle,
    const ResolvedStartupState& startup_state,
    track_geometry::TrackGeometry line,
    const std::filesystem::path& orvd_data_root,
    double vehicle_layout_reference_track_station_meters,
    double projection_search_half_width_meters,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField>
        track_irregularity) {
    internal::RequireVehicleMechanicalTrackStationLayoutInvariants(
        vehicle, "vehicle definition");
    internal::RequireResolvedStartupStateInvariants(startup_state,
                                                    "resolved start-up state");
    FrozenContactTopology topology = BuildGz18ContactTopology(
        vehicle, startup_state,
        vehicle_layout_reference_track_station_meters);

    auto contact = AssembleGz18WheelRailContact(
        orvd_data_root, startup_state.wheel_rail_binding,
        startup_state.rail_profile_reference_vertical_offset_meters);
    auto vehicle_system =
        AssembledVehicleSystem::AssembleWithWheelRailContact(
            vehicle,
            startup_state
                .gravitational_acceleration_meters_per_second_squared,
            std::move(line), contact->ReleaseRuntimePersonality(),
            std::move(track_irregularity), std::move(topology.carriers),
            std::move(topology.interfaces),
            std::move(topology.active_torque_couples),
            projection_search_half_width_meters);
    ResolvedInitialContext initial_context = AssembleResolvedInitialContext(
        *vehicle_system, startup_state,
        vehicle_layout_reference_track_station_meters);
    RequireAnchorsMatchResolvedPlacements(
        "GZ18", *vehicle_system->contact_force_plan(), initial_context);

    return std::unique_ptr<AssembledVehicleContactScenario>(
        new AssembledVehicleContactScenario(std::move(vehicle_system),
                                            std::move(initial_context)));
}

std::unique_ptr<AssembledVehicleContactScenario>
AssembleIrwContactScenario(
    const VehicleDefinition& vehicle,
    const ResolvedStartupState& startup_state,
    track_geometry::TrackGeometry line,
    const std::filesystem::path& orvd_data_root,
    double vehicle_layout_reference_track_station_meters,
    double projection_search_half_width_meters,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField>
        track_irregularity) {
    internal::RequireVehicleMechanicalTrackStationLayoutInvariants(
        vehicle, "vehicle definition");
    internal::RequireResolvedStartupStateInvariants(startup_state,
                                                    "resolved start-up state");
    FrozenContactTopology topology = BuildIrwContactTopology(
        vehicle, startup_state,
        vehicle_layout_reference_track_station_meters);

    auto contact = AssembleIrwWheelRailContact(
        orvd_data_root, startup_state.wheel_rail_binding,
        startup_state.rail_profile_reference_vertical_offset_meters);
    auto vehicle_system =
        AssembledVehicleSystem::AssembleWithWheelRailContact(
            vehicle,
            startup_state
                .gravitational_acceleration_meters_per_second_squared,
            std::move(line), contact->ReleaseRuntimePersonality(),
            std::move(track_irregularity), std::move(topology.carriers),
            std::move(topology.interfaces),
            std::move(topology.active_torque_couples),
            projection_search_half_width_meters);
    ResolvedInitialContext initial_context = AssembleResolvedInitialContext(
        *vehicle_system, startup_state,
        vehicle_layout_reference_track_station_meters);
    RequireAnchorsMatchResolvedPlacements(
        "IRW", *vehicle_system->contact_force_plan(), initial_context);

    return std::unique_ptr<AssembledVehicleContactScenario>(
        new AssembledVehicleContactScenario(std::move(vehicle_system),
                                            std::move(initial_context)));
}

}  // namespace orvd::configuration
