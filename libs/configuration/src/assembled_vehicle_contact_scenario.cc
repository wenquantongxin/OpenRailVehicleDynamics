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

#include "irw_closed_topology.h"
#include "orvd/configuration/gz18_wheel_rail_contact.h"
#include "orvd/configuration/irw_wheel_rail_contact.h"
#include "resolved_startup_state_invariants.h"
#include "vehicle_definition_invariants.h"
#include "vehicle_instance_assembly.h"

namespace orvd::configuration {
namespace {

using forces::WheelRailContactCarrierDefinition;
using forces::WheelRailContactInterfaceDefinition;
using forces::IndependentWheelActiveTorqueCoupleDefinition;
using wheel_rail_contact::WheelSide;

[[noreturn]] void Reject(std::string_view scenario,
                         const std::string& detail) {
    throw std::invalid_argument(std::string(scenario) +
                                " contact scenario: " + detail);
}

VehicleContactTopology BuildGz18ContactTopology(const VehicleDefinition& vehicle) {
    constexpr std::string_view kScenario = "GZ18";
    const auto& wheelsets = vehicle.mechanical_track_station_layout
                                .wheel_pair_station_reference_body_names;
    if (wheelsets.size() != 4) {
        Reject(kScenario,
               "the mechanical layout must name four wheel-pair station "
               "reference bodies");
    }

    VehicleContactTopology topology;
    topology.carriers.reserve(wheelsets.size());
    topology.interfaces.reserve(2 * wheelsets.size());
    for (const std::string& wheelset : wheelsets) {
        topology.carriers.push_back({wheelset, wheelset, 0.0, Eigen::Matrix3d::Identity()});
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

void RequireIrwMechanicalTopology(const VehicleDefinition& vehicle) {
    constexpr std::string_view kScenario = "IRW";
    const auto& carrier_names = vehicle.mechanical_track_station_layout
                                    .wheel_pair_station_reference_body_names;
    if (carrier_names.size() != internal::kIrwClosedCarrierNames.size()) {
        Reject(kScenario,
               "the mechanical layout must name exactly four axle-bridge "
               "station carriers");
    }
    std::unordered_set<std::string> stated_carriers;
    stated_carriers.reserve(carrier_names.size());
    for (const std::string& carrier_name : carrier_names) {
        stated_carriers.emplace(carrier_name);
    }
    for (const std::string_view expected :
         internal::kIrwClosedCarrierNames) {
        if (!stated_carriers.contains(std::string(expected))) {
            Reject(kScenario,
                   "the mechanical layout does not name the required "
                   "station carrier '" + std::string(expected) + "'");
        }
    }

    if (vehicle.revolute_joints.size() !=
        internal::kIrwClosedInterfaces.size()) {
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
    for (const internal::IrwClosedInterfaceTopology& expected :
         internal::kIrwClosedInterfaces) {
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

VehicleContactTopology BuildIrwContactTopology(const VehicleDefinition& vehicle) {
    RequireIrwMechanicalTopology(vehicle);

    VehicleContactTopology topology;
    topology.carriers.reserve(internal::kIrwClosedCarrierNames.size());
    topology.interfaces.reserve(internal::kIrwClosedInterfaces.size());
    topology.active_torque_couples.reserve(
        internal::kIrwClosedInterfaces.size());
    // The frozen WRL straight Track-T basis and the IRW axle-bridge body basis
    // differ by C = diag(1,-1,-1). ORVD's Track-T basis is already the
    // physical track basis, so this fixed R_BP keeps the non-spinning wheel
    // profile upright without rotating it by the wheel's joint phase.
    const Eigen::Matrix3d& rotation_body_from_profile =
        internal::kIrwBodyBasisHalfTurn;
    for (const std::string_view carrier :
         internal::kIrwClosedCarrierNames) {
        topology.carriers.push_back(
            {std::string(carrier), std::string(carrier), 0.0, rotation_body_from_profile});
    }
    for (const internal::IrwClosedInterfaceTopology& interface :
         internal::kIrwClosedInterfaces) {
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

PreparedVehicleContact PrepareGz18VehicleContact(const VehicleDefinition& vehicle,
                                                 const ResolvedStartupState& startup_state,
                                                 const std::filesystem::path& orvd_data_root) {
    internal::RequireVehicleMechanicalTrackStationLayoutInvariants(
        vehicle, "vehicle definition");
    internal::RequireResolvedStartupStateInvariants(startup_state, "resolved start-up state");
    auto contact = AssembleGz18WheelRailContact(
        orvd_data_root, startup_state.wheel_rail_binding,
        startup_state.rail_profile_reference_vertical_offset_meters);
    return {BuildGz18ContactTopology(vehicle), contact->ReleaseRuntimePersonality()};
}

std::unique_ptr<AssembledVehicleContactScenario>
internal::VehicleInstanceAssembler::AssembleContactScenario(
    const VehicleDefinition& vehicle, const ResolvedStartupState& startup_state,
    const ConsistDefinition& consist, PreparedVehicleContact contact,
    track_geometry::TrackGeometry line, double layout_station,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField> irregularity) {
    internal::RequireResolvedStartupStateInvariants(startup_state, "resolved start-up state");
    auto bindings = CaptureVehicleInstanceBindings(vehicle, consist);
    const auto stations = ResolveInitialBodyStations(bindings, startup_state, layout_station);
    VehicleContactTopology expanded;
    for (const auto& binding : bindings) {
        const auto qualified = [&](std::string& name) {
            name = QualifyVehicleName(binding.instance_name, name);
        };
        for (auto carrier : contact.topology.carriers) {
            qualified(carrier.carrier_name);
            qualified(carrier.body_name);
            const auto station = stations.find(carrier.body_name);
            if (station == stations.end()) {
                Reject(vehicle.vehicle_name,
                       "station carrier '" + carrier.body_name +
                           "' is missing from the resolved free-body start-up family");
            }
            carrier.initial_projection_station_meters = station->second;
            expanded.carriers.push_back(std::move(carrier));
        }
        for (auto interface : contact.topology.interfaces) {
            qualified(interface.interface_name);
            qualified(interface.carrier_name);
            qualified(interface.wheel_body_name);
            if (interface.independent_wheel_revolute_joint) {
                qualified(interface.independent_wheel_revolute_joint->joint_name);
            }
            expanded.interfaces.push_back(std::move(interface));
        }
        for (auto torque : contact.topology.active_torque_couples) {
            qualified(torque.channel_name);
            qualified(torque.axis_provider_body_name);
            qualified(torque.wheel_body_name);
            qualified(torque.reaction_frame_body_name);
            expanded.active_torque_couples.push_back(std::move(torque));
        }
    }
    auto vehicle_system = AssembledVehicleSystem::AssembleWithWheelRailContact(
        vehicle, std::move(bindings), consist.couplers,
        startup_state.gravitational_acceleration_meters_per_second_squared, std::move(line),
        std::move(contact.personality), std::move(irregularity), std::move(expanded.carriers),
        std::move(expanded.interfaces), std::move(expanded.active_torque_couples));
    auto initial_context =
        AssembleInitialContext(*vehicle_system, startup_state,
                               vehicle_system->contact_force_plan()->track_geometry(), stations);
    RequireAnchorsMatchResolvedPlacements(vehicle.vehicle_name,
                                          *vehicle_system->contact_force_plan(), initial_context);
    return std::unique_ptr<AssembledVehicleContactScenario>(
        new AssembledVehicleContactScenario(std::move(vehicle_system), std::move(initial_context)));
}

std::unique_ptr<AssembledVehicleContactScenario> AssembleConsistContactScenario(
    const VehicleDefinition& vehicle, const ResolvedStartupState& startup_state,
    const ConsistDefinition& consist, PreparedVehicleContact contact,
    track_geometry::TrackGeometry line, double layout_reference_track_station_meters,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField> track_irregularity) {
    return internal::VehicleInstanceAssembler::AssembleContactScenario(
        vehicle, startup_state, consist, std::move(contact), std::move(line),
        layout_reference_track_station_meters, std::move(track_irregularity));
}

std::unique_ptr<AssembledVehicleContactScenario> AssembleGz18ContactScenario(
    const VehicleDefinition& vehicle, const ResolvedStartupState& startup_state,
    track_geometry::TrackGeometry line, const std::filesystem::path& orvd_data_root,
    double vehicle_layout_reference_track_station_meters,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField> track_irregularity) {
    ConsistDefinition single{
        {{"", vehicle.vehicle_name, vehicle.mechanical_definition_identifier, 0.0}}, {}};
    return internal::VehicleInstanceAssembler::AssembleContactScenario(
        vehicle, startup_state, single,
        PrepareGz18VehicleContact(vehicle, startup_state, orvd_data_root), std::move(line),
        vehicle_layout_reference_track_station_meters, std::move(track_irregularity));
}

std::unique_ptr<AssembledVehicleContactScenario> AssembleIrwContactScenario(
    const VehicleDefinition& vehicle, const ResolvedStartupState& startup_state,
    track_geometry::TrackGeometry line, const std::filesystem::path& orvd_data_root,
    double vehicle_layout_reference_track_station_meters,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField> track_irregularity) {
    internal::RequireVehicleMechanicalTrackStationLayoutInvariants(
        vehicle, "vehicle definition");
    internal::RequireResolvedStartupStateInvariants(startup_state, "resolved start-up state");
    auto contact = AssembleIrwWheelRailContact(
        orvd_data_root, startup_state.wheel_rail_binding,
        startup_state.rail_profile_reference_vertical_offset_meters);
    PreparedVehicleContact prepared{BuildIrwContactTopology(vehicle),
                                    contact->ReleaseRuntimePersonality()};
    ConsistDefinition single{
        {{"", vehicle.vehicle_name, vehicle.mechanical_definition_identifier, 0.0}}, {}};
    return internal::VehicleInstanceAssembler::AssembleContactScenario(
        vehicle, startup_state, single, std::move(prepared), std::move(line),
        vehicle_layout_reference_track_station_meters, std::move(track_irregularity));
}

}  // namespace orvd::configuration
