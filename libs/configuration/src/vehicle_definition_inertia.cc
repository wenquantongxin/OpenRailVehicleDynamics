#include "vehicle_definition_inertia.h"

#include <stdexcept>
#include <string>

#include <Eigen/Eigenvalues>

namespace orvd::configuration::internal {

void ThrowIfSingularFreeBodyCenterOfMassInertia(
    const VehicleRigidBodyDefinition& body, std::string_view subject) {
    if (!body.moves_freely_in_world) {
        return;
    }
    const Eigen::Vector3d& moments =
        body.inertia_moments_about_center_of_mass_kilogram_square_meters;
    const Eigen::Vector3d& products =
        body.inertia_products_about_center_of_mass_kilogram_square_meters;
    Eigen::Matrix3d inertia;
    inertia << moments.x(), products.x(), products.y(), products.x(),
        moments.y(), products.z(), products.y(), products.z(), moments.z();

    if (!inertia.allFinite()) {
        throw std::invalid_argument(std::string(subject) +
                                    " must state a finite centre-of-mass "
                                    "inertia tensor");
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
        inertia, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success ||
        !(solver.eigenvalues().minCoeff() > 0.0)) {
        throw std::invalid_argument(std::string(subject) +
                                    " must state a positive-definite "
                                    "centre-of-mass inertia tensor");
    }
}

}  // namespace orvd::configuration::internal
