#include "orvd/multibody_runtime/multibody_physical_parameter_validation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Eigenvalues>

namespace orvd::multibody_runtime {
namespace {

// A rotation supplied by a caller has usually been composed from a few
// elementary rotations, so it carries a few units of round-off. This bound
// admits that and rejects anything that is not a rotation by any margin that
// matters — a transposed sign, a scaled axis, a shear. It is not a repair
// threshold: nothing is orthonormalised, the value is simply refused.
constexpr double kRotationOrthonormalityTolerance =
    128 * std::numeric_limits<double>::epsilon();

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("multibody parameters: " + detail);
}

void RequireFinite(double value, const std::string& what) {
    if (!std::isfinite(value)) {
        Reject(what + " is " + std::to_string(value) + ", which is not finite");
    }
}

void RequireFinite(const Eigen::Vector3d& values, const std::string& what) {
    for (int i = 0; i < 3; ++i) {
        RequireFinite(values[i], what + " component " + std::to_string(i));
    }
}

void RequireNonNegative(double value, const std::string& what) {
    RequireFinite(value, what);
    if (value < 0.0) {
        Reject(what + " is " + std::to_string(value) +
               ", which cannot be negative");
    }
}

}  // namespace

void ThrowIfNotARotation(const Eigen::Matrix3d& rotation,
                         const std::string& what) {
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            RequireFinite(rotation(row, column),
                          what + " element (" + std::to_string(row) + "," +
                              std::to_string(column) + ")");
        }
    }
    const Eigen::Matrix3d deviation =
        rotation * rotation.transpose() - Eigen::Matrix3d::Identity();
    const double orthonormality_error = deviation.cwiseAbs().maxCoeff();
    if (orthonormality_error > kRotationOrthonormalityTolerance) {
        Reject(what + " is not orthonormal: max|RRᵀ - I| is " +
               std::to_string(orthonormality_error) + ", above " +
               std::to_string(kRotationOrthonormalityTolerance));
    }
    if (rotation.determinant() <= 0.0) {
        Reject(what + " has determinant " +
               std::to_string(rotation.determinant()) +
               "; a rotation's determinant is +1, and a negative one means the "
               "basis is left-handed");
    }
}

// Checks the complete spatial-inertia condition. Checking only the three
// diagonal entries misses impossible products of inertia; checking about the
// body origin misses an inertia that becomes impossible after shifting to the
// centre of mass.
void ThrowIfNotRealisableInertia(const RigidBodyInertiaParameters& parameters,
                                 const std::string& what) {
    const std::string body = what + "'s ";
    RequireNonNegative(parameters.mass_kilograms, body + "mass");
    RequireFinite(parameters.center_of_mass_in_body_frame,
                  body + "centre of mass");
    RequireFinite(parameters.unit_inertia_moments,
                  body + "unit inertia moment");
    RequireFinite(parameters.unit_inertia_products, body + "unit inertia product");

    const Eigen::Vector3d& moments = parameters.unit_inertia_moments;
    const Eigen::Vector3d& products = parameters.unit_inertia_products;
    Eigen::Matrix3d unit_inertia_about_body_origin;
    unit_inertia_about_body_origin << moments[0], products[0], products[1],
        products[0], moments[1], products[2], products[1], products[2],
        moments[2];

    const Eigen::Vector3d& center_of_mass =
        parameters.center_of_mass_in_body_frame;
    const Eigen::Matrix3d point_mass_unit_inertia =
        center_of_mass.squaredNorm() * Eigen::Matrix3d::Identity() -
        center_of_mass * center_of_mass.transpose();
    const Eigen::Matrix3d central_rotational_inertia =
        parameters.mass_kilograms *
        (unit_inertia_about_body_origin - point_mass_unit_inertia);
    if (!central_rotational_inertia.allFinite()) {
        Reject(body + "central rotational inertia is not finite");
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> principal_moment_solver(
        central_rotational_inertia, Eigen::EigenvaluesOnly);
    if (principal_moment_solver.info() != Eigen::Success) {
        Reject(body + "central principal moments could not be computed");
    }
    const Eigen::Vector3d principal_moments =
        principal_moment_solver.eigenvalues();
    const double scale =
        std::max(1.0, 0.5 * std::abs(central_rotational_inertia.trace()));
    const double tolerance =
        16 * std::numeric_limits<double>::epsilon() * scale;
    if (principal_moments[0] < -tolerance ||
        principal_moments[0] + principal_moments[1] + tolerance <
            principal_moments[2]) {
        Reject(body + "central principal moments (" +
               std::to_string(principal_moments[0]) + ", " +
               std::to_string(principal_moments[1]) + ", " +
               std::to_string(principal_moments[2]) +
               ") are negative or violate the triangle inequality");
    }
}

}  // namespace orvd::multibody_runtime
