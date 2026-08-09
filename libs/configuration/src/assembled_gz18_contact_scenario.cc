#include "orvd/configuration/assembled_gz18_contact_scenario.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "orvd/configuration/gz18_wheel_rail_contact.h"
#include "resolved_startup_state_invariants.h"
#include "vehicle_definition_invariants.h"

namespace orvd::configuration {
namespace {

using forces::WheelRailContactCarrierDefinition;
using forces::WheelRailContactInterfaceDefinition;
using wheel_rail_contact::WheelSide;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("GZ18 contact scenario: " + detail);
}

struct FrozenContactTopology {
    std::vector<WheelRailContactCarrierDefinition> carriers;
    std::vector<WheelRailContactInterfaceDefinition> interfaces;
};

FrozenContactTopology BuildContactTopology(
    const VehicleDefinition& vehicle,
    const ResolvedStartupState& startup_state,
    double vehicle_layout_reference_track_station_meters) {
    if (!std::isfinite(vehicle_layout_reference_track_station_meters)) {
        Reject("the vehicle layout reference track station is not finite");
    }
    const auto& layout = vehicle.mechanical_track_station_layout;
    if (layout.wheelset_body_names.size() != 4) {
        Reject("the GZ18 mechanical layout must name four wheelset bodies");
    }

    std::unordered_map<std::string, double> mechanical_offset;
    mechanical_offset.reserve(layout.free_body_station_offsets.size());
    for (const VehicleFreeBodyStationOffsetDefinition& entry :
         layout.free_body_station_offsets) {
        mechanical_offset.emplace(entry.body_name, entry.station_offset_meters);
    }
    std::unordered_map<std::string, double> resolved_offset;
    resolved_offset.reserve(startup_state.free_body_startup_states.size());
    for (const FreeBodyStartupState& entry :
         startup_state.free_body_startup_states) {
        resolved_offset.emplace(
            entry.body_name,
            entry.resolved_track_station_offset_from_mechanical_layout_meters);
    }

    FrozenContactTopology topology;
    topology.carriers.reserve(layout.wheelset_body_names.size());
    topology.interfaces.reserve(2 * layout.wheelset_body_names.size());
    for (const std::string& wheelset : layout.wheelset_body_names) {
        const auto mechanical = mechanical_offset.find(wheelset);
        const auto resolved = resolved_offset.find(wheelset);
        if (mechanical == mechanical_offset.end() ||
            resolved == resolved_offset.end()) {
            Reject("wheelset '" + wheelset +
                   "' is missing from the mechanical layout or resolved "
                   "free-body start-up family");
        }
        const double initial_station =
            vehicle_layout_reference_track_station_meters +
            mechanical->second + resolved->second;
        topology.carriers.push_back(WheelRailContactCarrierDefinition{
            wheelset, wheelset, initial_station});
        topology.interfaces.push_back(WheelRailContactInterfaceDefinition{
            wheelset + ".right", wheelset, wheelset, WheelSide::kRight});
        topology.interfaces.push_back(WheelRailContactInterfaceDefinition{
            wheelset + ".left", wheelset, wheelset, WheelSide::kLeft});
    }
    return topology;
}

void RequireAnchorsMatchResolvedPlacements(
    const forces::WheelRailContactForcePlan& contact_plan,
    const ResolvedInitialContext& initial_context) {
    std::unordered_map<std::string, double> resolved_station;
    resolved_station.reserve(initial_context.wheelset_placements().size());
    for (const ResolvedWheelsetPlacement& placement :
         initial_context.wheelset_placements()) {
        resolved_station.emplace(placement.wheelset_body_name,
                                 placement.track_station_meters);
    }
    if (contact_plan.carrier_count() !=
        static_cast<int>(resolved_station.size())) {
        throw std::logic_error(
            "GZ18 contact scenario: contact carriers and resolved wheelset "
            "placements differ in count");
    }
    for (int ordinal = 0; ordinal < contact_plan.carrier_count(); ++ordinal) {
        const std::string name(contact_plan.carrier_name(ordinal));
        const auto placement = resolved_station.find(name);
        if (placement == resolved_station.end() ||
            placement->second !=
                contact_plan.initial_projection_station_meters(ordinal)) {
            throw std::logic_error(
                "GZ18 contact scenario: initial projection anchor for '" +
                name + "' differs from its resolved wheelset station");
        }
    }
}

}  // namespace

AssembledGz18ContactScenario::AssembledGz18ContactScenario(
    std::unique_ptr<AssembledVehicleSystem> vehicle_system,
    ResolvedInitialContext initial_context)
    : vehicle_system_(std::move(vehicle_system)),
      initial_context_(std::move(initial_context)) {}

AssembledGz18ContactScenario::~AssembledGz18ContactScenario() = default;

std::unique_ptr<AssembledGz18ContactScenario>
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
    FrozenContactTopology topology = BuildContactTopology(
        vehicle, startup_state,
        vehicle_layout_reference_track_station_meters);

    auto gz18_contact = AssembleGz18WheelRailContact(
        orvd_data_root, startup_state.wheel_rail_binding,
        startup_state.rail_profile_reference_vertical_offset_meters);
    auto vehicle_system =
        AssembledVehicleSystem::AssembleWithWheelRailContact(
            vehicle,
            startup_state
                .gravitational_acceleration_meters_per_second_squared,
            std::move(line), gz18_contact->ReleaseRuntimePersonality(),
            std::move(track_irregularity),
            std::move(topology.carriers), std::move(topology.interfaces),
            projection_search_half_width_meters);
    ResolvedInitialContext initial_context = AssembleResolvedInitialContext(
        *vehicle_system, startup_state,
        vehicle_layout_reference_track_station_meters);
    RequireAnchorsMatchResolvedPlacements(
        *vehicle_system->contact_force_plan(), initial_context);

    return std::unique_ptr<AssembledGz18ContactScenario>(
        new AssembledGz18ContactScenario(std::move(vehicle_system),
                                         std::move(initial_context)));
}

}  // namespace orvd::configuration
