#pragma once

#include <filesystem>
#include <memory>

#include "orvd/configuration/assemble_resolved_initial_context.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/consist_definition.h"
#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/track_geometry/track_geometry.h"
#include "orvd/wheel_rail_contact/track_irregularity_field.h"

namespace orvd::configuration {

struct VehicleContactTopology {
    std::vector<forces::WheelRailContactCarrierDefinition> carriers;
    std::vector<forces::WheelRailContactInterfaceDefinition> interfaces;
    std::vector<forces::IndependentWheelActiveTorqueCoupleDefinition> active_torque_couples;
};

struct PreparedVehicleContact {
    VehicleContactTopology topology;
    std::unique_ptr<wheel_rail_contact::WheelRailContactRuntimePersonality> personality;
};

[[nodiscard]] PreparedVehicleContact PrepareGz18VehicleContact(
    const VehicleDefinition& vehicle, const ResolvedStartupState& startup_state,
    const std::filesystem::path& orvd_data_root);

// One transactionally assembled vehicle contact scenario.
//
// The system owns the vehicle model, vehicle-force plan, wheel--rail force
// plan, optional active-torque plan, line and immutable contact personality.
// The resolved context is created only after that ownership chain is complete,
// and is destroyed first. Thus a successful return cannot contain a context
// whose line or contact assets disagree with the force plan that will evaluate
// it.
class AssembledVehicleContactScenario {
   public:
    ~AssembledVehicleContactScenario();

    AssembledVehicleContactScenario(
        const AssembledVehicleContactScenario&) = delete;
    AssembledVehicleContactScenario& operator=(
        const AssembledVehicleContactScenario&) = delete;
    AssembledVehicleContactScenario(AssembledVehicleContactScenario&&) =
        delete;
    AssembledVehicleContactScenario& operator=(
        AssembledVehicleContactScenario&&) = delete;

    [[nodiscard]] const AssembledVehicleSystem& vehicle_system() const {
        return *vehicle_system_;
    }
    [[nodiscard]] const ResolvedInitialContext& initial_context() const {
        return initial_context_;
    }

   private:
     friend class internal::VehicleInstanceAssembler;

     AssembledVehicleContactScenario(std::unique_ptr<AssembledVehicleSystem> vehicle_system,
                                     ResolvedInitialContext initial_context);

     // Destruction is reverse declaration order: the context goes away before
     // the system and the model-bound workspaces it borrows from.
     std::unique_ptr<AssembledVehicleSystem> vehicle_system_;
     ResolvedInitialContext initial_context_;
};

// Builds the GZ18 vehicle, its installed contact personality, one owned line,
// four immutable initial projection anchors and a resolved start-up context as
// one operation. Each rigid wheelset is both a station carrier and the body
// receiving its left/right contact wrenches.
[[nodiscard]] std::unique_ptr<AssembledVehicleContactScenario>
AssembleGz18ContactScenario(
    const VehicleDefinition& vehicle,
    const ResolvedStartupState& startup_state,
    track_geometry::TrackGeometry line,
    const std::filesystem::path& orvd_data_root,
    double vehicle_layout_reference_track_station_meters,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField>
        track_irregularity = nullptr);

// Builds the IRW vehicle and its installed contact personality as one
// operation. The closed topology has four axle-bridge station carriers and
// eight independently jointed wheel bodies in frozen axle/side order. The same
// closed table binds each wheel's active-torque axis provider and reaction
// frame. The factory validates that every named wheel joint is exactly the
// expected axle-bridge-to-wheel +Y revolute relation before any runtime object
// is published.
[[nodiscard]] std::unique_ptr<AssembledVehicleContactScenario>
AssembleIrwContactScenario(
    const VehicleDefinition& vehicle,
    const ResolvedStartupState& startup_state,
    track_geometry::TrackGeometry line,
    const std::filesystem::path& orvd_data_root,
    double vehicle_layout_reference_track_station_meters,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField>
        track_irregularity = nullptr);

[[nodiscard]] std::unique_ptr<AssembledVehicleContactScenario> AssembleConsistContactScenario(
    const VehicleDefinition& vehicle, const ResolvedStartupState& startup_state,
    const ConsistDefinition& consist, PreparedVehicleContact contact,
    track_geometry::TrackGeometry line, double layout_reference_track_station_meters,
    std::unique_ptr<wheel_rail_contact::TrackIrregularityField> track_irregularity = nullptr);

// The layout reference station is a scene property, not a start-up JSON field.
// A non-null irregularity field is moved into the immutable force plan, while
// null selects the explicit zero-irregularity scenario. Neither factory reads
// files during right-hand-side evaluation.

}  // namespace orvd::configuration
