#include <cmath>
#include <cstdio>
#include <exception>
#include <stdexcept>

#include <Eigen/Core>

#include "orvd/configuration/assemble_vehicle_multibody_model.h"
#include "orvd/configuration/load_track_geometry.h"
#include "orvd/configuration/load_vehicle_definition.h"

int main(int argc, char* argv[]) {
    try {
        if (argc != 3) {
            throw std::invalid_argument(
                "expected the installed track geometry and vehicle definition "
                "paths");
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
        if (!model->is_finalized() || total_mass_kilograms != 55695.0 ||
            model->num_generalized_positions() != 57 ||
            model->num_generalized_velocities() != 50) {
            std::fprintf(stderr,
                         "installed vehicle record did not assemble into the "
                         "vehicle it describes\n");
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed configuration smoke failed: %s\n",
                     error.what());
        return 1;
    }
    return 0;
}
