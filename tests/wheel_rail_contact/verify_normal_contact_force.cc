// The normal contact law: the elliptic integral, the segment inversion, the
// Hertz force and the damping.
//
// The elliptic integral is checked against its own power series, summed here.
// The Hertz force is checked against the classical closed form for a sphere on
// a plane, which the general expression must reduce to when the patch comes out
// circular. The segment inversion is checked against the one segment whose area
// is elementary — the semicircle — and against a round trip. Nothing is
// compared against a remembered number.

#include <cmath>
#include <cstdio>
#include <functional>
#include <numbers>
#include <string>
#include <string_view>

#include "allocation_probe.h"
#include "orvd/wheel_rail_contact/normal_contact_force.h"

namespace {

using orvd::test::AllocationScope;
using orvd::wheel_rail_contact::CircularSegmentArea;
using orvd::wheel_rail_contact::CompleteEllipticIntegralFirstKind;
using orvd::wheel_rail_contact::NormalContactConfiguration;
using orvd::wheel_rail_contact::NormalContactGeometry;
using orvd::wheel_rail_contact::NormalContactLaw;
using orvd::wheel_rail_contact::NormalContactResult;
using orvd::wheel_rail_contact::SolveCircularSegmentHeight;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "normal contact force: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireClose(double actual, double expected, double relative_tolerance,
                  std::string_view what) {
    const double scale = std::max(std::abs(expected), 1.0e-30);
    if (!(std::abs(actual - expected) <= relative_tolerance * scale)) {
        std::fprintf(stderr,
                     "normal contact force: %.*s (expected %.17g, got %.17g, "
                     "relative %.3g)\n",
                     static_cast<int>(what.size()), what.data(), expected, actual,
                     std::abs(actual - expected) / scale);
        ++failures;
    }
}

void RequireRefusal(const std::function<void()>& action, std::string_view fragment,
                    std::string_view what) {
    try {
        action();
    } catch (const std::exception& error) {
        if (std::string(error.what()).find(fragment) == std::string::npos) {
            std::fprintf(stderr,
                         "normal contact force: %.*s was refused for another "
                         "reason: %s\n",
                         static_cast<int>(what.size()), what.data(), error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

// The complete elliptic integral of the first kind, from its hypergeometric
// series. Slow and completely independent of the arithmetic-geometric mean the
// implementation uses, which is what makes it worth summing here.
double EllipticFirstKindBySeries(double parameter) {
    double term = 1.0;
    double total = 1.0;
    for (int order = 1; order < 20000; ++order) {
        const double odd = 2.0 * static_cast<double>(order) - 1.0;
        const double even = 2.0 * static_cast<double>(order);
        const double factor = (odd / even) * (odd / even);
        term *= factor * parameter;
        total += term;
        if (term < 1.0e-18 * total) {
            break;
        }
    }
    return 0.5 * std::numbers::pi * total;
}

NormalContactConfiguration SteelConfiguration() {
    NormalContactConfiguration configuration;
    configuration.material.youngs_modulus_pascals = 210.0e9;
    configuration.material.poisson_ratio = 0.28;
    configuration.penetration_equivalence_factor = 1.8181818181818181;
    configuration.reference_damping_newton_seconds_per_meter = 100000.0;
    configuration.reference_stiffness_newtons_per_meter = 500000000.0;
    configuration.damping_activation_penetration_meters = 1.0e-12;
    return configuration;
}

// A patch of the given width, length and overlap area, deep enough and on a
// wheel large enough to pass every gate.
NormalContactGeometry Patch(double width, double length, double area) {
    NormalContactGeometry geometry;
    geometry.arc_width_meters = width;
    geometry.longitudinal_length_meters = length;
    geometry.cross_section_area_square_meters = area;
    geometry.vertical_penetration_meters = 1.5 * area / width;
    geometry.rolling_radius_meters = 0.42;
    geometry.common_normal_angle_radians = 0.2;
    return geometry;
}

}  // namespace

int main() {
    {
        // The elliptic integral, against its series.
        RequireClose(CompleteEllipticIntegralFirstKind(0.0), 0.5 * std::numbers::pi,
                     0.0, "a circular patch does not give the integral its value "
                          "at zero exactly");
        for (const double parameter : {0.01, 0.2098765432098765, 0.5, 0.8, 0.95}) {
            RequireClose(CompleteEllipticIntegralFirstKind(parameter),
                         EllipticFirstKindBySeries(parameter), 1.0e-13,
                         "the elliptic integral disagrees with its own series");
        }
        Require(CompleteEllipticIntegralFirstKind(0.5) >
                    CompleteEllipticIntegralFirstKind(0.4),
                "the elliptic integral is not increasing in its parameter");
        // The argument is the parameter, not the modulus, and the two are far
        // enough apart at a realistic eccentricity that no tolerance hides it.
        const double eccentricity = 0.45812284729085112;
        Require(std::abs(CompleteEllipticIntegralFirstKind(eccentricity) -
                         CompleteEllipticIntegralFirstKind(eccentricity *
                                                           eccentricity)) > 0.1,
                "the parameter and the modulus give the same integral at a "
                "realistic eccentricity, so this fixture cannot tell them apart");
        Require(std::isfinite(CompleteEllipticIntegralFirstKind(1.0)),
                "a degenerate ellipse gave a non-finite integral rather than the "
                "clamped one");
    }

    {
        // The one circular segment whose area is elementary: the semicircle,
        // whose height is its radius and whose chord is its diameter.
        const double radius = 0.037;
        RequireClose(CircularSegmentArea(radius, 2.0 * radius),
                     0.5 * std::numbers::pi * radius * radius, 1.0e-15,
                     "the area of a semicircular segment is not half a circle");

        // The inversion is the forward map's inverse.
        for (const double height : {1.0e-5, 1.0e-4, 1.0e-3, 2.0e-3}) {
            const double chord = 0.012;
            const double area = CircularSegmentArea(height, chord);
            RequireClose(SolveCircularSegmentHeight(chord, area), height, 1.0e-8,
                         "the segment inversion does not undo the segment area");
        }

        // Shallow segments approach two thirds of the enclosing rectangle, and
        // approach it from below — but they are not equal to it at the depths a
        // rail contact reaches, which is why the exact expression is used.
        const double chord = 0.012;
        const double height = 1.0e-4;
        const double shallow = (2.0 / 3.0) * chord * height;
        const double exact = CircularSegmentArea(height, chord);
        Require(exact > shallow,
                "a circular segment is not larger than two thirds of its "
                "bounding rectangle");
        Require((exact - shallow) / exact > 1.0e-5,
                "the shallow approximation is indistinguishable from the exact "
                "area at this depth, so substituting it would be invisible here");

        Require(SolveCircularSegmentHeight(0.0, 1.0e-6) == 0.0 &&
                    SolveCircularSegmentHeight(0.012, 0.0) == 0.0,
                "a segment with no chord or no area was given a height");
    }

    const NormalContactLaw law{SteelConfiguration()};

    {
        // The equivalent penetration, pinned against the overlap that produced
        // it rather than against itself. A shallow segment of area A on a chord
        // W has height very near 3A/2W, so the approach comes out at that times
        // the equivalence factor — and a law that forgot to apply the factor,
        // or applied its reciprocal, lands nowhere near.
        const NormalContactConfiguration configuration = SteelConfiguration();
        const double width = 0.012;
        const double area = 8.0e-7;
        const NormalContactResult result = law.Solve(Patch(width, 0.0135, area), 0.0);
        RequireClose(result.equivalent_penetration_meters,
                     1.5 * area / width *
                         configuration.penetration_equivalence_factor,
                     1.0e-3,
                     "the equivalent penetration is not the shallow segment's "
                     "height times the equivalence factor");
        Require(result.equivalent_penetration_meters >
                    1.5 * SolveCircularSegmentHeight(width, area),
                "the equivalence factor was not applied at all");
    }

    {
        // The plane-strain modulus of two identical bodies.
        const double poisson = 0.28;
        const double modulus = 210.0e9;
        RequireClose(law.equivalent_modulus_pascals(),
                     1.0 / (2.0 * (1.0 - poisson * poisson) / modulus), 0.0,
                     "the plane-strain modulus of the pair is not the reciprocal "
                     "of twice the single-body compliance");
    }

    {
        // A circular patch. The general expression must then reduce to the
        // classical Hertz result for a sphere on a plane, which is written here
        // from the textbook rather than from the implementation.
        const double width = 0.012;
        const double length = 0.012;
        const NormalContactResult result = law.Solve(Patch(width, length, 8.0e-7), 0.0);
        Require(result.in_contact, "a well-formed patch was rejected");
        RequireClose(result.longitudinal_semi_axis_meters, 0.5 * length, 0.0,
                     "the longitudinal semi-axis is not half the patch length");
        RequireClose(result.lateral_semi_axis_meters, 0.5 * width, 0.0,
                     "the lateral semi-axis is not half the patch width");
        const double semi_axis = 0.5 * width;
        RequireClose(result.elastic_force_newtons,
                     (4.0 / 3.0) * law.equivalent_modulus_pascals() * semi_axis *
                         result.equivalent_penetration_meters,
                     1.0e-14,
                     "a circular patch does not reproduce the Hertz force for a "
                     "sphere on a plane");
        RequireClose(result.normal_force_newtons, result.elastic_force_newtons, 0.0,
                     "a patch with no closing speed acquired a damping force");
    }

    {
        // An elongated patch. The force is then the same expression with the
        // minor semi-axis cancelled and the elliptic integral in the
        // denominator — an algebraically distinct rearrangement, evaluated here
        // with the series rather than with the implementation's integral.
        const double width = 0.008;
        const double length = 0.020;
        const NormalContactResult result = law.Solve(Patch(width, length, 6.0e-7), 0.0);
        Require(result.in_contact, "an elongated patch was rejected");
        const double major = std::max(result.longitudinal_semi_axis_meters,
                                      result.lateral_semi_axis_meters);
        const double minor = std::min(result.longitudinal_semi_axis_meters,
                                      result.lateral_semi_axis_meters);
        const double ratio = minor / major;
        const double parameter = 1.0 - ratio * ratio;
        const double expected =
            (2.0 * std::numbers::pi / 3.0) * major *
            result.equivalent_penetration_meters * law.equivalent_modulus_pascals() /
            EllipticFirstKindBySeries(parameter);
        RequireClose(result.elastic_force_newtons, expected, 1.0e-12,
                     "the elastic force of an elongated patch does not match the "
                     "closed form with the minor semi-axis cancelled");

        // The semi-axes keep their meaning rather than being sorted: swapping
        // the patch's width and length must swap which of the two is larger.
        const NormalContactResult across = law.Solve(Patch(length, width, 6.0e-7), 0.0);
        Require(across.in_contact, "the transposed patch was rejected");
        Require(result.longitudinal_semi_axis_meters >
                    result.lateral_semi_axis_meters &&
                    across.longitudinal_semi_axis_meters <
                        across.lateral_semi_axis_meters,
                "the two semi-axes were sorted, so a patch long along the track "
                "cannot be told from one long across it");
    }

    {
        // The damping is linear in the closing speed and its coefficient
        // follows the square root of the contact's secant stiffness.
        const NormalContactGeometry geometry = Patch(0.012, 0.0135, 8.0e-7);
        const NormalContactResult still = law.Solve(geometry, 0.0);
        const NormalContactResult closing = law.Solve(geometry, 0.01);
        const NormalContactResult faster = law.Solve(geometry, 0.02);
        Require(still.in_contact && closing.in_contact && faster.in_contact,
                "the damping fixture lost contact");
        RequireClose(closing.elastic_force_newtons, still.elastic_force_newtons, 0.0,
                     "the closing speed changed the elastic force");
        RequireClose(faster.damping_force_newtons,
                     2.0 * closing.damping_force_newtons, 1.0e-14,
                     "the damping force is not linear in the closing speed");
        const NormalContactConfiguration configuration = SteelConfiguration();
        const double expected_coefficient =
            configuration.reference_damping_newton_seconds_per_meter *
            std::sqrt(still.elastic_force_newtons /
                      still.equivalent_penetration_meters /
                      configuration.reference_stiffness_newtons_per_meter);
        RequireClose(closing.damping_force_newtons, expected_coefficient * 0.01,
                     1.0e-14,
                     "the damping coefficient does not follow the square root of "
                     "the secant stiffness");

        // Separating reduces the force, and separating fast enough drives it to
        // exactly zero. The contact never pulls.
        const NormalContactResult separating = law.Solve(geometry, -0.01);
        Require(separating.normal_force_newtons < still.normal_force_newtons,
                "separation did not reduce the normal force");
        const NormalContactResult flying = law.Solve(geometry, -1000.0);
        Require(flying.normal_force_newtons == 0.0,
                "a fast separation produced a tensile contact force");
        RequireClose(flying.elastic_force_newtons + flying.damping_force_newtons, 0.0,
                     1.0e-12,
                     "the two parts of a saturated force do not sum to the total");
    }

    {
        // The analytic baseline is the qualified WRL expression: vertical
        // penetration and the local rolling radius divided by the common
        // normal's cosine. A nonzero angle makes omitting that projection
        // visible. The three-dimensional measurement, when present, wins
        // exactly and clears the flag.
        NormalContactGeometry geometry = Patch(0.012, 0.0135, 8.0e-7);
        const NormalContactResult resolved = law.Solve(geometry, 0.0);
        Require(resolved.in_contact &&
                    !resolved.used_analytic_longitudinal_length_fallback,
                "a resolved three-dimensional length was reported as a fallback");
        Require(resolved.longitudinal_semi_axis_meters ==
                    0.5 * geometry.longitudinal_length_meters,
                "the explicit three-dimensional length did not override the "
                "analytic baseline exactly");

        geometry.longitudinal_length_meters = 0.0;
        const NormalContactResult fallback = law.Solve(geometry, 0.0);
        const double effective_radius =
            geometry.rolling_radius_meters /
            std::abs(std::cos(geometry.common_normal_angle_radians));
        const double expected_semi_axis =
            std::sqrt(2.0 * effective_radius *
                      geometry.vertical_penetration_meters);
        Require(fallback.in_contact &&
                    fallback.used_analytic_longitudinal_length_fallback,
                "an unavailable three-dimensional length did not report its "
                "analytic fallback");
        RequireClose(fallback.longitudinal_semi_axis_meters, expected_semi_axis,
                     1.0e-14,
                     "the fallback semi-axis is not the qualified analytic "
                     "baseline");
    }

    {
        // The gates. A patch that has stopped touching comes back empty, not as
        // an exception: it happens on every step.
        const auto rejected = [&](NormalContactGeometry geometry,
                                  std::string_view what) {
            const NormalContactResult result = law.Solve(geometry, 0.0);
            Require(!result.in_contact && result.normal_force_newtons == 0.0 &&
                        result.longitudinal_semi_axis_meters == 0.0 &&
                        result.lateral_semi_axis_meters == 0.0 &&
                        result.equivalent_penetration_meters == 0.0 &&
                        !result.used_analytic_longitudinal_length_fallback,
                    what);
        };
        NormalContactGeometry lifted = Patch(0.012, 0.0135, 8.0e-7);
        lifted.vertical_penetration_meters = 0.0;
        rejected(lifted, "a patch with no depth produced a force");
        NormalContactGeometry empty = Patch(0.012, 0.0135, 8.0e-7);
        empty.cross_section_area_square_meters = 0.0;
        rejected(empty, "a patch with no area produced a force");
        NormalContactGeometry narrow = Patch(0.012, 0.0135, 8.0e-7);
        narrow.arc_width_meters = 0.0;
        rejected(narrow, "a patch with no width produced a force");
        NormalContactGeometry broken = Patch(0.012, 0.0135, 8.0e-7);
        broken.cross_section_area_square_meters = std::nan("");
        rejected(broken, "a patch with a non-finite area produced a force");
        NormalContactGeometry radiusless = Patch(0.012, 0.0135, 8.0e-7);
        radiusless.rolling_radius_meters = 0.0;
        rejected(radiusless, "a patch with no local rolling radius produced a force");
    }

    {
        // At a fixed ellipse the Hertz force is exactly linear in the approach,
        // and this law inherits that: the whole of its nonlinearity in load
        // lives in the patch growing, which is the geometry's business and not
        // this module's. A test that expected a stiffening here would be
        // expecting this module to do the geometry's job.
        const NormalContactResult light = law.Solve(Patch(0.012, 0.0135, 4.0e-7), 0.0);
        const NormalContactResult heavy = law.Solve(Patch(0.012, 0.0135, 8.0e-7), 0.0);
        Require(light.in_contact && heavy.in_contact,
                "the stiffness fixture lost contact");
        RequireClose(heavy.elastic_force_newtons / heavy.equivalent_penetration_meters,
                     light.elastic_force_newtons / light.equivalent_penetration_meters,
                     1.0e-14,
                     "the force is not linear in the equivalent penetration at a "
                     "fixed contact ellipse");
        Require(heavy.equivalent_penetration_meters >
                    light.equivalent_penetration_meters,
                "a larger overlap area did not give a deeper equivalent "
                "penetration");
        Require(heavy.maximum_pressure_pascals > light.maximum_pressure_pascals,
                "a deeper contact is not at a higher peak pressure");

        // A wider patch at the same equivalent penetration carries more: that
        // is where the load dependence actually comes from once the geometry
        // supplies it.
        const NormalContactResult narrow_patch =
            law.Solve(Patch(0.008, 0.0135, 4.0e-7), 0.0);
        const NormalContactResult wide_patch =
            law.Solve(Patch(0.016, 0.0135, 8.0e-7), 0.0);
        Require(narrow_patch.in_contact && wide_patch.in_contact,
                "the width fixture lost contact");
        RequireClose(wide_patch.equivalent_penetration_meters,
                     narrow_patch.equivalent_penetration_meters, 1.0e-3,
                     "the two width fixtures are not at the same equivalent "
                     "penetration, so they compare nothing");
        // It does increase — but by less than the width ratio. Widening the
        // patch also changes the ellipse's aspect, and the elliptic integral in
        // the denominator grows with it, so the two effects work against each
        // other. Asserting a proportional increase would be asserting the
        // integral away.
        Require(wide_patch.elastic_force_newtons >
                    1.2 * narrow_patch.elastic_force_newtons,
                "doubling the patch width at the same penetration did not "
                "increase the force");
        Require(wide_patch.elastic_force_newtons <
                    2.0 * narrow_patch.elastic_force_newtons,
                "the force grew at least in proportion to the width, so the "
                "elliptic integral is not moderating the aspect change");
        // The peak pressure and the force agree with the paraboloidal
        // distribution the Hertz solution has.
        RequireClose(heavy.maximum_pressure_pascals,
                     3.0 * heavy.elastic_force_newtons /
                         (2.0 * std::numbers::pi *
                          heavy.longitudinal_semi_axis_meters *
                          heavy.lateral_semi_axis_meters),
                     1.0e-14,
                     "the peak pressure is not three halves the mean pressure "
                     "over the ellipse");
    }

    RequireRefusal(
        [&] {
            NormalContactConfiguration configuration = SteelConfiguration();
            configuration.material.poisson_ratio = 0.5;
            (void)NormalContactLaw{configuration};
        },
        "outside [0, 0.5)", "an incompressible material");
    RequireRefusal(
        [&] {
            NormalContactConfiguration configuration = SteelConfiguration();
            configuration.material.youngs_modulus_pascals = 0.0;
            (void)NormalContactLaw{configuration};
        },
        "Young's modulus", "a material with no stiffness");
    RequireRefusal(
        [&] {
            NormalContactConfiguration configuration = SteelConfiguration();
            configuration.reference_stiffness_newtons_per_meter = 0.0;
            (void)NormalContactLaw{configuration};
        },
        "reference stiffness", "a damping reference with no stiffness");

    {
        // The law runs once per patch per derivative evaluation.
        const NormalContactGeometry geometry = Patch(0.012, 0.0135, 8.0e-7);
        double sink = 0.0;
        const AllocationScope scope;
        for (int step = 0; step < 200; ++step) {
            sink += law.Solve(geometry, 0.001 * static_cast<double>(step))
                        .normal_force_newtons;
        }
        Require(scope.allocations() == 0,
                "the normal contact law allocated on the integrator's path");
        Require(sink > 0.0, "the allocation sweep produced no force");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("normal contact force verified");
    return 0;
}
