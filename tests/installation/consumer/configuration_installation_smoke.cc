#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>

#include <Eigen/Core>

#include "orvd/configuration/assemble_resolved_initial_context.h"
#include "orvd/configuration/assemble_vehicle_multibody_model.h"
#include "orvd/configuration/assembled_vehicle_system.h"
#include "orvd/configuration/load_resolved_startup_state.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"

int main(int argc, char* argv[]) {
    try {
        if (argc != 4) {
            throw std::invalid_argument(
                "expected the installed track geometry, vehicle definition and "
                "resolved start-up state paths");
        }
        const auto line =
            orvd::configuration::LoadTrackGeometryFromJsonFile(argv[1]);
        const auto point = line.CenterlinePositionInInertialMeters(40.0);
        if (std::abs(point.x() - 40.0) > 1.0e-12 || point.y() != 0.0 ||
            point.z() != 0.0) {
            std::fprintf(stderr,
                         "installed configuration smoke produced an invalid "
                         "straight-line point\n");
            return 1;
        }
        // The installed vehicle record, read from where the install put it and
        // assembled with only the installed public headers and export set.
        const auto vehicle =
            orvd::configuration::LoadVehicleDefinitionFromJsonFile(argv[2]);
        double total_mass_kilograms = 0.0;
        for (const auto& body : vehicle.rigid_bodies) {
            total_mass_kilograms += body.mass_kilograms;
        }
        const auto model =
            orvd::configuration::AssembleVehicleMultibodyModel(vehicle, 9.81);
        const auto force_plan =
            orvd::configuration::BuildVehicleForcePlan(vehicle, *model);
        if (!model->is_finalized() || total_mass_kilograms != 55695.0 ||
            model->num_generalized_positions() != 57 ||
            model->num_generalized_velocities() != 50 ||
            force_plan->translational_spring_damper_count() != 20 ||
            force_plan->series_spring_viscous_damper_count() != 2 ||
            force_plan->body_wrench_count() != 56) {
            std::fprintf(stderr,
                         "installed vehicle record did not assemble and "
                         "compile into the vehicle it describes\n");
            return 1;
        }

        // The installed start-up state, placed on the installed line. This is
        // the whole chain a consumer of the package walks: three installed
        // records in, one runtime context out.
        const auto startup =
            orvd::configuration::LoadResolvedStartupStateFromJsonFile(argv[3]);
        const auto system = orvd::configuration::AssembleVehicleSystem(
            vehicle,
            startup.gravitational_acceleration_meters_per_second_squared);
        const auto resolved =
            orvd::configuration::AssembleResolvedInitialContext(
                *system, startup, line, 20.0);
        if (resolved.wheelset_placements().size() != 4 ||
            resolved.context().generalized_positions().size() != 57 ||
            resolved.context().series_spring_damper_forces().size() != 2) {
            std::fprintf(stderr,
                         "installed start-up state did not assemble into the "
                         "context it describes\n");
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed configuration smoke failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
