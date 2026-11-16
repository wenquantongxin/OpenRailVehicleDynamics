#include <Eigen/Core>
#include <cmath>
#include <cstdio>
#include <exception>
#include <utility>

#include "orvd/vehicle_project_editing/vehicle_project_editing_session.h"
#include "orvd/vehicle_project_editing/vehicle_project_element_properties.h"
#include "orvd/vehicle_project_editing/vehicle_project_mechanical_check_report.h"
#include "orvd/vehicle_project_editing/vehicle_project_preview.h"

// The installed editing session used through its public headers and archive
// alone: a one-body project has its body's placement edited, which leaves an
// unsaved change, passes the mechanical check under its own gravity, is drawn
// where the placement puts it, and has the edit undone, which leaves none.

int main() {
    using namespace orvd;
    try {
        configuration::VehicleDefinition vehicle;
        vehicle.vehicle_name = "smoke_vehicle";
        vehicle.mechanical_definition_identifier = "smoke_vehicle_mechanics";
        vehicle.mechanical_track_station_layout = {
            .reference_body_name = "carbody",
            .free_body_station_offsets = {{.body_name = "carbody", .station_offset_meters = 0.0}},
            .wheel_pair_station_reference_body_names = {}};
        vehicle.rigid_bodies.push_back(
            {.name = "carbody",
             .moves_freely_in_world = true,
             .mass_kilograms = 1.0,
             .inertia_moments_about_center_of_mass_kilogram_square_meters =
                 Eigen::Vector3d::Ones()});
        vehicle_project::VehicleProject project{
            .vehicle_definition = std::move(vehicle),
            .gravitational_acceleration_meters_per_second_squared = 9.81,
            .free_body_placements = {{.body_name = "carbody"}}};

        vehicle_project_editing::VehicleProjectEditingSession session(std::move(project));
        const vehicle_project::ElementReference carbody{vehicle_project::ElementKind::kRigidBody,
                                                        "carbody"};
        const bool edited =
            session
                .SetElementProperty(carbody,
                                    "placement.vertical_offset_in_local_track_frame_meters", -1.0)
                .has_value();
        const bool unsaved_after_edit = session.has_unsaved_changes();
        const vehicle_project_editing::MechanicalCheckReport report = session.CheckMechanics();
        const auto& preview = session.preview();
        const bool drawn_at_placement =
            preview.has_value() && preview->rigid_bodies.size() == 1 &&
            std::abs(preview->rigid_bodies[0].origin_in_drawing_frame_meters.z() + 1.0) <= 1.0e-12;
        session.Undo();
        const auto readings =
            vehicle_project_editing::ReadElementProperties(session.project(), carbody);
        if (!edited || !unsaved_after_edit || !report.refusals.empty() || !drawn_at_placement ||
            session.revision() != 2 || !session.can_redo() || session.has_unsaved_changes() ||
            readings.empty() ||
            session.project().free_body_placements[0].vertical_offset_in_local_track_frame_meters !=
                0.0) {
            std::fprintf(stderr,
                         "installed vehicle project editing did not edit, check, draw and undo the "
                         "project it was given\n");
            return 1;
        }
        std::puts("installed vehicle project editing smoke passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed vehicle project editing smoke failed: %s\n", error.what());
        return 1;
    }
}
