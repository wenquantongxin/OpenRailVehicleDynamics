#pragma once

#include <span>
#include <unordered_map>

#include "orvd/configuration/assembled_vehicle_contact_scenario.h"

namespace orvd::configuration::internal {

void AppendVehicleMultibodyModel(const VehicleDefinition& vehicle, std::string_view instance_name,
                                 multibody_model::MultibodyModel& model);
void AppendVehicleForceElements(const VehicleDefinition& vehicle, std::string_view instance_name,
                                const multibody_model::MultibodyModel& model,
                                forces::VehicleForceElementCollection& elements);
void AppendPointToPointLinearSpringDampers(
    std::span<const PointToPointLinearSpringDamperDefinition> definitions,
    std::string_view name_prefix, const multibody_model::MultibodyModel& model,
    forces::VehicleForceElementCollection& elements);
void AppendPointToPointHystereticSprings(
    std::span<const PointToPointHystereticSpringDefinition> definitions,
    std::string_view name_prefix, const multibody_model::MultibodyModel& model,
    forces::VehicleForceElementCollection& elements);
std::vector<VehicleInstanceBinding> CaptureVehicleInstanceBindings(
    const VehicleDefinition& vehicle, const ConsistDefinition& consist);
using InitialBodyStations = std::unordered_map<std::string, double>;
InitialBodyStations ResolveInitialBodyStations(const std::vector<VehicleInstanceBinding>& bindings,
                                               const ResolvedStartupState& startup_state,
                                               double layout_station);

class VehicleInstanceAssembler {
  public:
    static ResolvedInitialContext AssembleInitialContext(const AssembledVehicleSystem& system,
                                                         const ResolvedStartupState& startup_state,
                                                         const track_geometry::TrackGeometry& line,
                                                         const InitialBodyStations& stations);
    static std::unique_ptr<AssembledVehicleContactScenario> AssembleContactScenario(
        const VehicleDefinition& vehicle, const ResolvedStartupState& startup_state,
        const ConsistDefinition& consist, PreparedVehicleContact contact,
        track_geometry::TrackGeometry line, double layout_station,
        std::unique_ptr<wheel_rail_contact::TrackIrregularityField> irregularity);
};

}  // namespace orvd::configuration::internal
