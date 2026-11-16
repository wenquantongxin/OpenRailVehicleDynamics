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
#include "orvd/configuration/load_vehicle_contact_record.h"
#include "orvd/configuration/vehicle_contact_record.h"
#include "resolved_startup_state_invariants.h"
#include "vehicle_definition_invariants.h"
#include "vehicle_instance_assembly.h"
#include "wheel_rail_contact_personality_assembly.h"

namespace orvd::configuration {
namespace {

using forces::IndependentWheelActiveTorqueCoupleDefinition;

[[noreturn]] void Reject(std::string_view scenario,
                         const std::string& detail) {
    throw std::invalid_argument(std::string(scenario) +
                                " contact scenario: " + detail);
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
        const std::string& name = contact_plan.carrier_definition(ordinal).body_name;
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

PreparedVehicleContact PrepareVehicleContact(const VehicleDefinition& vehicle,
                                             const ResolvedStartupState& startup_state,
                                             const VehicleContactRecord& record,
                                             const std::filesystem::path& orvd_data_root) {
    internal::RequireVehicleMechanicalTrackStationLayoutInvariants(vehicle, "vehicle definition");
    internal::RequireResolvedStartupStateInvariants(startup_state, "resolved start-up state");
    auto topology = BuildVehicleContactTopology(vehicle, record.declaration);
    auto contact = internal::AssembleBoundWheelRailContact(
        orvd_data_root, startup_state.wheel_rail_binding,
        startup_state.rail_profile_reference_vertical_offset_meters, record);
    return {std::move(topology), std::move(contact->runtime_personality)};
}

PreparedVehicleContact PrepareGz18VehicleContact(const VehicleDefinition& vehicle,
                                                 const ResolvedStartupState& startup_state,
                                                 const std::filesystem::path& orvd_data_root) {
    return PrepareVehicleContact(vehicle, startup_state,
                                 LoadVehicleContactRecordFromJsonFile(
                                     orvd_data_root / internal::kGz18ContactRecordRelativePath),
                                 orvd_data_root);
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
        vehicle, std::move(bindings), consist.point_to_point_linear_spring_dampers,
        consist.point_to_point_hysteretic_springs,
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

std::unique_ptr<AssembledVehicleContactScenario> AssembleVehicleContactScenario(
    const VehicleDefinition& vehicle, const ResolvedStartupState& startup_state,
    const VehicleContactRecord& record, track_geometry::TrackGeometry line,
    const std::filesystem::path& orvd_data_root, double layout_station,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField> irregularity) {
    ConsistDefinition single{
        {{"", vehicle.vehicle_name, vehicle.mechanical_definition_identifier, 0.0}}, {}, {}};
    return internal::VehicleInstanceAssembler::AssembleContactScenario(
        vehicle, startup_state, single,
        PrepareVehicleContact(vehicle, startup_state, record, orvd_data_root), std::move(line),
        layout_station, std::move(irregularity));
}

std::unique_ptr<AssembledVehicleContactScenario> AssembleGz18ContactScenario(
    const VehicleDefinition& vehicle, const ResolvedStartupState& startup_state,
    track_geometry::TrackGeometry line, const std::filesystem::path& orvd_data_root,
    double vehicle_layout_reference_track_station_meters,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField> track_irregularity) {
    ConsistDefinition single{
        {{"", vehicle.vehicle_name, vehicle.mechanical_definition_identifier, 0.0}}, {}, {}};
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
    PreparedVehicleContact prepared =
        PrepareVehicleContact(vehicle, startup_state,
                              LoadVehicleContactRecordFromJsonFile(
                                  orvd_data_root / internal::kIrwContactRecordRelativePath),
                              orvd_data_root);
    // Control and actuation keep their established channel order and reaction
    // bodies. The contact declaration itself contains no control bindings.
    for (const internal::IrwClosedInterfaceTopology& interface : internal::kIrwClosedInterfaces) {
        prepared.topology.active_torque_couples.push_back(
            IndependentWheelActiveTorqueCoupleDefinition{
                .channel_name = std::string(interface.interface_name),
                .axis_provider_body_name = std::string(interface.carrier_body_name),
                .wheel_body_name = std::string(interface.wheel_body_name),
                .reaction_frame_body_name = std::string(interface.reaction_frame_body_name),
            });
    }
    ConsistDefinition single{
        {{"", vehicle.vehicle_name, vehicle.mechanical_definition_identifier, 0.0}}, {}, {}};
    return internal::VehicleInstanceAssembler::AssembleContactScenario(
        vehicle, startup_state, single, std::move(prepared), std::move(line),
        vehicle_layout_reference_track_station_meters, std::move(track_irregularity));
}

}  // namespace orvd::configuration
