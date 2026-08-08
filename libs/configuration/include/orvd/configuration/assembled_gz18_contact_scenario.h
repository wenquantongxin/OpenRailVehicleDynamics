#pragma once

#include <filesystem>
#include <memory>

#include "orvd/configuration/assemble_resolved_initial_context.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/resolved_startup_state.h"
#include "orvd/configuration/vehicle_definition.h"
#include "orvd/track_geometry/track_geometry.h"

namespace orvd::configuration {

// One transactionally assembled GZ18 contact scenario.
//
// The system owns the vehicle model, vehicle-force plan, wheel--rail force
// plan, line and immutable contact personality.  The resolved context is
// created only after that ownership chain is complete, and is destroyed first.
// Thus a successful return cannot contain a context whose line or contact
// assets disagree with the force plan that will evaluate it.
class AssembledGz18ContactScenario {
   public:
    ~AssembledGz18ContactScenario();

    AssembledGz18ContactScenario(const AssembledGz18ContactScenario&) = delete;
    AssembledGz18ContactScenario& operator=(
        const AssembledGz18ContactScenario&) = delete;
    AssembledGz18ContactScenario(AssembledGz18ContactScenario&&) = delete;
    AssembledGz18ContactScenario& operator=(
        AssembledGz18ContactScenario&&) = delete;

    [[nodiscard]] const AssembledVehicleSystem& vehicle_system() const {
        return *vehicle_system_;
    }
    [[nodiscard]] const ResolvedInitialContext& initial_context() const {
        return initial_context_;
    }

   private:
    friend std::unique_ptr<AssembledGz18ContactScenario>
    AssembleGz18ContactScenario(
        const VehicleDefinition&, const ResolvedStartupState&,
        track_geometry::TrackGeometry, const std::filesystem::path&, double,
        double);

    AssembledGz18ContactScenario(
        std::unique_ptr<AssembledVehicleSystem> vehicle_system,
        ResolvedInitialContext initial_context);

    // Destruction is reverse declaration order: the context goes away before
    // the system and the model-bound workspaces it borrows from.
    std::unique_ptr<AssembledVehicleSystem> vehicle_system_;
    ResolvedInitialContext initial_context_;
};

// Builds the GZ18 vehicle, installed contact personality, single owned line,
// four immutable initial projection anchors and resolved start-up context as
// one operation.
//
// `vehicle_layout_reference_track_station_meters` remains a scene property,
// not a start-up JSON field.  The projection half width is a local branch
// isolation width, not a physical response limit.  G53 keeps its four initial
// anchors immutable; accepted/candidate projection history belongs to G54.
[[nodiscard]] std::unique_ptr<AssembledGz18ContactScenario>
AssembleGz18ContactScenario(
    const VehicleDefinition& vehicle,
    const ResolvedStartupState& startup_state,
    track_geometry::TrackGeometry line,
    const std::filesystem::path& orvd_data_root,
    double vehicle_layout_reference_track_station_meters,
    double projection_search_half_width_meters);

}  // namespace orvd::configuration
