#include "orvd/wheel_rail_contact/normal_contact_force.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace orvd::wheel_rail_contact {
namespace {

// The bisection's lower bracket. Not zero: the circle through a chord and a
// segment apex has a radius that divides by the apex height, so a height of
// exactly zero names no circle.
constexpr double kSegmentHeightFloor = 1.0e-15;
constexpr double kSegmentHeightTolerance = 1.0e-14;
constexpr int kSegmentHeightIterations = 80;

// How close to a degenerate ellipse the parameter is allowed to get. At the
// limit the integral diverges logarithmically; the clamp keeps it finite at a
// value no contact patch reaches.
constexpr double kEllipticParameterCeiling = 1.0 - 1.0e-15;
constexpr double kAgmTolerance = 1.0e-15;
constexpr int kAgmIterations = 80;

// The floor on the cosine of the contact angle, applied to its magnitude so
// that an angle past a right angle does not reverse the effective radius's
// sign. Only the estimated length reads it.
constexpr double kContactAngleCosineFloor = 1.0e-6;

// The radius below which the wheel is not a wheel.
constexpr double kRollingRadiusFloor = 1.0e-9;

}  // namespace

double CompleteEllipticIntegralFirstKind(double parameter) {
    const double clamped =
        std::min(std::max(parameter, 0.0), kEllipticParameterCeiling);
    double arithmetic = 1.0;
    double geometric = std::sqrt(std::max(0.0, 1.0 - clamped));
    for (int step = 0; step < kAgmIterations; ++step) {
        const double next_arithmetic = 0.5 * (arithmetic + geometric);
        const double next_geometric = std::sqrt(arithmetic * geometric);
        if (std::abs(next_arithmetic - next_geometric) < kAgmTolerance) {
            arithmetic = next_arithmetic;
            break;
        }
        arithmetic = next_arithmetic;
        geometric = next_geometric;
    }
    return std::numbers::pi / (2.0 * arithmetic);
}

double CircularSegmentArea(double height_meters, double chord_meters) {
    if (!(height_meters > 0.0) || !(chord_meters > 0.0)) {
        return 0.0;
    }
    // The radius of the circle through the two chord ends and the apex.
    const double radius =
        (chord_meters * chord_meters) / (8.0 * height_meters) + 0.5 * height_meters;
    const double cosine = std::min(std::max((radius - height_meters) / radius, -1.0), 1.0);
    const double angle = 2.0 * std::acos(cosine);
    return 0.5 * radius * radius * (angle - std::sin(angle));
}

double SolveCircularSegmentHeight(double chord_meters, double area_square_meters) {
    if (!(area_square_meters > 0.0) || !(chord_meters > 0.0)) {
        return 0.0;
    }
    double low = kSegmentHeightFloor;
    double high = chord_meters;
    for (int step = 0; step < kSegmentHeightIterations; ++step) {
        const double middle = 0.5 * (low + high);
        if (CircularSegmentArea(middle, chord_meters) > area_square_meters) {
            high = middle;
        } else {
            low = middle;
        }
        if (high - low < kSegmentHeightTolerance) {
            break;
        }
    }
    return 0.5 * (low + high);
}

NormalContactLaw::NormalContactLaw(
    const NormalContactConfiguration& configuration)
    : configuration_(configuration) {
    const ContactMaterial& material = configuration_.material;
    if (!std::isfinite(material.youngs_modulus_pascals) ||
        !(material.youngs_modulus_pascals > 0.0)) {
        throw std::invalid_argument(
            "NormalContactLaw: Young's modulus must be a positive finite number, "
            "and " + std::to_string(material.youngs_modulus_pascals) + " is not");
    }
    if (!(material.poisson_ratio >= 0.0) || !(material.poisson_ratio < 0.5)) {
        throw std::invalid_argument(
            "NormalContactLaw: a Poisson ratio of " +
            std::to_string(material.poisson_ratio) +
            " is outside [0, 0.5); at one half the material is incompressible "
            "and the plane-strain modulus is unbounded");
    }
    if (!(configuration_.reference_stiffness_newtons_per_meter > 0.0) ||
        !(configuration_.reference_damping_newton_seconds_per_meter > 0.0) ||
        !std::isfinite(configuration_.penetration_equivalence_factor)) {
        throw std::invalid_argument(
            "NormalContactLaw: the reference stiffness and damping must both be "
            "positive and the penetration equivalence factor finite");
    }

    // Two identical bodies in plane strain: 1/E* = 2(1-nu^2)/E. Written as the
    // reciprocal of that sum rather than as E/(2(1-nu^2)) — the two are the same
    // number in exact arithmetic and not always the same double, and the sum
    // form is the one that stays right if the two bodies are ever given
    // different materials.
    equivalent_modulus_pascals_ =
        1.0 / ((2.0 * (1.0 - material.poisson_ratio * material.poisson_ratio)) /
               material.youngs_modulus_pascals);
}

NormalContactResult NormalContactLaw::Solve(
    const NormalContactGeometry& geometry,
    double approach_speed_meters_per_second) const {
    NormalContactResult result;

    const double width = geometry.arc_width_meters;
    const double area = geometry.cross_section_area_square_meters;
    const double depth = geometry.vertical_penetration_meters;
    const double rolling_radius = geometry.rolling_radius_meters;
    if (!std::isfinite(width) || !std::isfinite(area) || !std::isfinite(depth) ||
        !std::isfinite(rolling_radius) || !(depth > 0.0) || !(area > 0.0) ||
        !(width > 0.0) || !(rolling_radius > kRollingRadiusFloor)) {
        return result;
    }

    const double lateral_semi_axis = 0.5 * width;

    // The longitudinal extent. The circular estimate is formed first and always:
    // it is what the patch length would be if the wheel were a cylinder of the
    // local radius meeting a plane, which is the leading behaviour, and it is
    // what stands when the geometry could not resolve the true extent.
    const double angle_cosine =
        std::max(std::abs(std::cos(geometry.common_normal_angle_radians)),
                 kContactAngleCosineFloor);
    const double effective_radius = rolling_radius / angle_cosine;
    double length = 2.0 * std::sqrt(std::max(0.0, 2.0 * effective_radius * depth));
    result.used_estimated_length = true;
    if (std::isfinite(geometry.longitudinal_length_meters) &&
        geometry.longitudinal_length_meters > 0.0) {
        length = geometry.longitudinal_length_meters;
        result.used_estimated_length = false;
    }
    if (!(length > 0.0)) {
        result.used_estimated_length = false;
        return result;
    }
    const double longitudinal_semi_axis = 0.5 * length;

    // The overlap's cross-section, replaced by the circular segment of equal
    // area on the same chord, and that segment's height scaled into the
    // approach Hertz's solution is written in terms of.
    const double segment_height = SolveCircularSegmentHeight(width, area);
    const double equivalent_penetration =
        segment_height * configuration_.penetration_equivalence_factor;
    if (!(equivalent_penetration > 0.0)) {
        result.used_estimated_length = false;
        return result;
    }

    // Hertz. The sort here is local: the integral needs to know which semi-axis
    // is the larger, and the emitted pair must stay in its fixed order.
    const double major = std::max(longitudinal_semi_axis, lateral_semi_axis);
    const double minor = std::min(longitudinal_semi_axis, lateral_semi_axis);
    const double ratio = minor / major;
    const double eccentricity = std::sqrt(std::max(0.0, 1.0 - ratio * ratio));
    // The elliptic *parameter*, which is the square of the eccentricity. Passing
    // the eccentricity itself would give a larger integral and a force some
    // percent low, silently.
    const double parameter = eccentricity * eccentricity;
    const double integral = CompleteEllipticIntegralFirstKind(parameter);
    if (!(integral > 0.0)) {
        result.used_estimated_length = false;
        return result;
    }

    const double peak_pressure =
        equivalent_penetration * equivalent_modulus_pascals_ / (minor * integral);
    const double elastic_force =
        (2.0 / 3.0) * std::numbers::pi * major * minor * peak_pressure;
    if (!(elastic_force > 0.0)) {
        result.used_estimated_length = false;
        return result;
    }

    // The damping. Its coefficient follows the square root of the contact's
    // current secant stiffness, so that a stiffening contact damps proportionally
    // rather than becoming ever more lightly damped.
    double damping_force = 0.0;
    double total_force = elastic_force;
    if (equivalent_penetration > configuration_.damping_activation_penetration_meters) {
        const double secant_stiffness = elastic_force / equivalent_penetration;
        if (secant_stiffness > 0.0) {
            const double coefficient =
                configuration_.reference_damping_newton_seconds_per_meter *
                std::sqrt(secant_stiffness /
                          configuration_.reference_stiffness_newtons_per_meter);
            damping_force = coefficient * approach_speed_meters_per_second;
            // A contact cannot pull. Separating fast enough simply drives the
            // total to zero; the patch is not removed, and the tangential
            // solution then sees no load rather than a negative one.
            total_force = std::max(0.0, elastic_force + damping_force);
            if (total_force == 0.0) {
                damping_force = -elastic_force;
            }
        }
    }

    result.normal_force_newtons = total_force;
    result.elastic_force_newtons = elastic_force;
    result.damping_force_newtons = damping_force;
    result.longitudinal_semi_axis_meters = longitudinal_semi_axis;
    result.lateral_semi_axis_meters = lateral_semi_axis;
    result.equivalent_penetration_meters = equivalent_penetration;
    result.maximum_pressure_pascals = peak_pressure;
    result.in_contact = true;
    return result;
}

}  // namespace orvd::wheel_rail_contact
