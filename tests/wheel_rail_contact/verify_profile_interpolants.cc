// The two interpolants a profile surface is read through, checked by the
// properties they claim rather than against a table of remembered numbers.

#include <cmath>
#include <cstdio>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "orvd/wheel_rail_contact/monotone_cubic_interpolant.h"
#include "orvd/wheel_rail_contact/natural_cubic_spline.h"

namespace {

using orvd::wheel_rail_contact::ComputeShapePreservingNodalSlopes;
using orvd::wheel_rail_contact::MonotoneCubicInterpolant;
using orvd::wheel_rail_contact::NaturalCubicSpline;
using orvd::wheel_rail_contact::OutsideDerivativeRule;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "profile interpolants: %.*s\n",
                     static_cast<int>(what.size()), what.data());
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
                         "profile interpolants: %.*s was refused for another "
                         "reason: %s\n",
                         static_cast<int>(what.size()), what.data(), error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

// The one-sided second derivative at a knot, obtained by extrapolating along
// the segment rather than by evaluating at the knot itself. On a cubic the
// second derivative is linear in the segment parameter, so two interior samples
// determine it exactly and the result is free of the snapping the evaluators
// apply at knots.
double OneSidedSecondDerivative(const NaturalCubicSpline& spline, double from,
                                double to) {
    const double first = from + 0.25 * (to - from);
    const double second = from + 0.75 * (to - from);
    const double at_first = spline.EvaluateSecondDerivative(first);
    const double at_second = spline.EvaluateSecondDerivative(second);
    const double slope = (at_second - at_first) / (second - first);
    return at_first + slope * (from - first);
}

void CheckNaturalCubicProperties(const std::vector<double>& knots,
                                 const std::vector<double>& values,
                                 std::string_view label) {
    const NaturalCubicSpline spline(knots, values);
    const std::string prefix(label);

    for (std::size_t node = 0; node < knots.size(); ++node) {
        Require(spline.Evaluate(knots[node]) == values[node],
                prefix + ": the spline does not reproduce a tabulated value");
    }

    // C2 at every interior knot: the segment on each side must agree in value,
    // slope and curvature. Value and slope are structural; curvature is what
    // the natural cubic is for and the only one that can actually fail.
    for (std::size_t node = 1; node + 1 < knots.size(); ++node) {
        const double from_left =
            OneSidedSecondDerivative(spline, knots[node], knots[node - 1]);
        const double from_right =
            OneSidedSecondDerivative(spline, knots[node], knots[node + 1]);
        const double scale =
            std::max(1.0, std::max(std::abs(from_left), std::abs(from_right)));
        Require(std::abs(from_left - from_right) < 1.0e-9 * scale,
                prefix + ": the curvature jumps at an interior knot");
    }

    const double left_curvature =
        OneSidedSecondDerivative(spline, knots.front(), knots[1]);
    const double right_curvature = OneSidedSecondDerivative(
        spline, knots.back(), knots[knots.size() - 2]);
    const double curvature_scale = std::max(
        1.0, std::abs(OneSidedSecondDerivative(spline, knots[1], knots[2])));
    Require(std::abs(left_curvature) < 1.0e-9 * curvature_scale &&
                std::abs(right_curvature) < 1.0e-9 * curvature_scale,
            prefix + ": the curvature at a boundary knot is not zero, so the "
                     "natural boundary condition does not hold");

    Require(spline.EvaluateSecondDerivative(knots.front()) == 0.0 &&
                spline.EvaluateSecondDerivative(knots.back()) == 0.0,
            prefix + ": the reported curvature at a boundary knot is not "
                     "exactly zero");

    const double span = knots.back() - knots.front();
    Require(spline.Evaluate(knots.front() - span) == values.front() &&
                spline.Evaluate(knots.back() + span) == values.back(),
            prefix + ": the value is not held flat outside the knots");
    Require(spline.EvaluateFirstDerivative(knots.front() - span) == 0.0 &&
                spline.EvaluateFirstDerivative(knots.back() + span) == 0.0,
            prefix + ": the slope is not zero strictly outside the knots");

    // At a boundary knot the slope is the interior one-sided value, not the
    // zero the flat extrapolation would suggest.
    Require(std::abs(spline.EvaluateFirstDerivative(knots.front()) -
                     spline.nodal_slopes().front()) < 1.0e-12,
            prefix + ": the slope at the first knot is not the nodal slope");
}

}  // namespace

int main() {
    // An equally spaced grid takes one construction path and an unequally
    // spaced one takes the other. Both are exercised because they solve
    // different systems and only agree analytically.
    const std::vector<double> uniform_knots{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> uniform_values{0.0, 1.0, 0.0, 1.0, 0.0};
    CheckNaturalCubicProperties(uniform_knots, uniform_values, "equally spaced");

    const std::vector<double> ragged_knots{0.0, 0.5, 2.0, 2.25, 5.0};
    const std::vector<double> ragged_values{1.0, -2.0, 0.5, 3.0, -1.0};
    CheckNaturalCubicProperties(ragged_knots, ragged_values, "unequally spaced");

    {
        const NaturalCubicSpline uniform(uniform_knots, uniform_values);
        Require(uniform.is_uniform(),
                "an equally spaced grid was not detected as one");
        const NaturalCubicSpline ragged(ragged_knots, ragged_values);
        Require(!ragged.is_uniform(),
                "an unequally spaced grid was detected as equally spaced");

        // The two paths must agree on data both can take. Perturbing the
        // interior knots below the detection tolerance forces the general path
        // onto data the uniform path also accepts.
        std::vector<double> nudged = uniform_knots;
        nudged[2] += 3.0e-9;
        const NaturalCubicSpline general(nudged, uniform_values);
        Require(!general.is_uniform(),
                "a grid perturbed past the detection tolerance was still "
                "treated as equally spaced");
        double worst = 0.0;
        for (int sample = 0; sample <= 40; ++sample) {
            const double at = 4.0 * static_cast<double>(sample) / 40.0;
            worst = std::max(worst,
                             std::abs(uniform.Evaluate(at) - general.Evaluate(at)));
        }
        Require(worst < 1.0e-7,
                "the two construction paths disagree on data both accept");
    }

    {
        // A profile surface is read many times per contact evaluation, so the
        // interpolant has to be independent of query order.
        const NaturalCubicSpline spline(ragged_knots, ragged_values);
        const double ascending = spline.Evaluate(1.0);
        (void)spline.Evaluate(4.9);
        (void)spline.Evaluate(0.1);
        Require(spline.Evaluate(1.0) == ascending,
                "the spline's answer depends on what was asked before it");
    }

    RequireRefusal([] { NaturalCubicSpline(std::vector<double>{0.0},
                                           std::vector<double>{1.0}); },
                   "at least two points", "a one-point spline");
    RequireRefusal(
        [] {
            NaturalCubicSpline(std::vector<double>{0.0, 1.0, 1.0},
                               std::vector<double>{0.0, 1.0, 2.0});
        },
        "strictly increasing", "a spline over a repeated knot");
    RequireRefusal(
        [] {
            NaturalCubicSpline(std::vector<double>{0.0, 1.0},
                               std::vector<double>{
                                   0.0, std::numeric_limits<double>::quiet_NaN()});
        },
        "not finite", "a spline over a value that is not a number");

    {
        // Shape preservation: a monotone table must produce a monotone curve,
        // and the curve must not leave the box its two end values span. A
        // natural cubic on the same table does neither, which is why both
        // interpolants exist.
        const std::vector<double> knots{0.0, 1.0, 2.0, 3.0, 4.0};
        const std::vector<double> values{0.0, 0.0, 0.0, 1.0, 1.0};
        const MonotoneCubicInterpolant shaped =
            MonotoneCubicInterpolant::FromValues(knots, values);
        const NaturalCubicSpline smooth(knots, values);
        double worst_shaped = 0.0;
        double worst_smooth = 0.0;
        double previous = shaped.Evaluate(knots.front());
        bool monotone = true;
        for (int sample = 0; sample <= 400; ++sample) {
            const double at = 4.0 * static_cast<double>(sample) / 400.0;
            const double here = shaped.Evaluate(at);
            if (here < previous - 1.0e-15) {
                monotone = false;
            }
            previous = here;
            worst_shaped = std::max(worst_shaped, std::max(-here, here - 1.0));
            const double smooth_here = smooth.Evaluate(at);
            worst_smooth =
                std::max(worst_smooth, std::max(-smooth_here, smooth_here - 1.0));
        }
        Require(monotone,
                "the shape-preserving interpolant is not monotone on a monotone "
                "table");
        Require(worst_shaped <= 0.0,
                "the shape-preserving interpolant leaves the range its values "
                "span");
        Require(worst_smooth > 1.0e-3,
                "the fixture does not distinguish the two interpolants: the "
                "natural cubic did not overshoot either");
    }

    {
        // The weighted harmonic mean on a non-uniform grid. Swapping the two
        // interval weights is invisible on a uniform table, so pin the formula
        // with unequal spacings and unequal secants.
        const std::vector<double> knots{0.0, 1.0, 4.0};
        const std::vector<double> values{0.0, 2.0, 5.0};
        std::vector<double> slopes(knots.size(), 0.0);
        ComputeShapePreservingNodalSlopes(knots, values, slopes);
        const double expected = 12.0 / (7.0 / 2.0 + 5.0 / 1.0);
        const double swapped = 12.0 / (5.0 / 2.0 + 7.0 / 1.0);
        Require(std::abs(slopes[1] - expected) < 4.0e-15,
                "the non-uniform interior slope uses the wrong harmonic "
                "weights");
        Require(std::abs(expected - swapped) > 0.1,
                "the non-uniform fixture cannot distinguish swapped weights");
    }

    {
        // The endpoint limiter. On a table whose first two secants straddle a
        // turning point sharply enough, the unlimited one-sided formula
        // overshoots three times the first secant; the limit is what stops it.
        const std::vector<double> knots{0.0, 1.0, 2.0};
        const std::vector<double> values{0.0, 1.0, -3.0};
        std::vector<double> slopes(knots.size(), 0.0);
        ComputeShapePreservingNodalSlopes(knots, values, slopes);
        const double first_secant = (values[1] - values[0]) / (knots[1] - knots[0]);
        const double second_secant = (values[2] - values[1]) / (knots[2] - knots[1]);
        const double unlimited =
            ((2.0 * 1.0 + 1.0) * first_secant - 1.0 * second_secant) / (1.0 + 1.0);
        Require(unlimited > 3.0 * first_secant,
                "the fixture does not reach the endpoint limit");
        Require(slopes.front() == 3.0 * first_secant,
                "the endpoint slope was not limited to three times its secant");
    }

    {
        // A flat run adjacent to a rising one must produce a zero derivative at
        // the joint, which is what keeps the flat part flat.
        const std::vector<double> knots{0.0, 1.0, 2.0};
        const std::vector<double> values{1.0, 1.0, 2.0};
        std::vector<double> slopes(knots.size(), 0.0);
        ComputeShapePreservingNodalSlopes(knots, values, slopes);
        Require(slopes[1] == 0.0,
                "the derivative at the joint between a flat and a rising run is "
                "not zero");
    }

    {
        // Borrowed derivatives. A cubic segment is fixed by its two end values
        // and two end slopes, so handing the natural spline's nodal slopes to
        // the Hermite evaluator reproduces the spline inside the knots exactly.
        // That is the point of the construction: what changes is not the curve
        // but what happens at its ends and how it is queried.
        const NaturalCubicSpline donor(ragged_knots, ragged_values);
        const MonotoneCubicInterpolant borrowed =
            MonotoneCubicInterpolant::FromNodalSlopes(ragged_knots, ragged_values,
                                                      donor.nodal_slopes());
        double worst_inside = 0.0;
        for (int sample = 0; sample <= 500; ++sample) {
            const double at = ragged_knots.front() +
                              (ragged_knots.back() - ragged_knots.front()) *
                                  static_cast<double>(sample) / 500.0;
            worst_inside =
                std::max(worst_inside, std::abs(borrowed.Evaluate(at) -
                                                donor.Evaluate(at)));
        }
        Require(worst_inside < 1.0e-15,
                "a borrowed-slope interpolant is not its donor between knots");

        // The shape-preserving slopes are genuinely different slopes, so the
        // same points read through them give a different curve. Without this
        // the check above would be vacuous.
        const MonotoneCubicInterpolant own =
            MonotoneCubicInterpolant::FromValues(ragged_knots, ragged_values);
        const double interior = 0.5 * (ragged_knots[1] + ragged_knots[2]);
        Require(std::abs(own.Evaluate(interior) - donor.Evaluate(interior)) > 1.0e-3,
                "the two slope rules produce the same curve, so the borrowing "
                "check proves nothing");
    }

    {
        // The two rules for what a derivative means outside the knots.
        const std::vector<double> knots{0.0, 0.7, 1.4, 2.0};
        const std::vector<double> values{0.0, 1.2, -0.4, 0.8};
        const MonotoneCubicInterpolant holding =
            MonotoneCubicInterpolant::FromValues(knots, values);
        const MonotoneCubicInterpolant zeroing = MonotoneCubicInterpolant::FromValues(
            knots, values, OutsideDerivativeRule::kZeroOutsideKnots);
        Require(holding.EvaluateFirstDerivative(-0.3) != 0.0,
                "the holding rule reported a zero slope outside the knots");
        Require(zeroing.EvaluateFirstDerivative(-0.3) == 0.0 &&
                    zeroing.EvaluateFirstDerivative(2.3) == 0.0,
                "the zeroing rule reported a slope outside the knots");
        Require(zeroing.EvaluateSecondDerivative(knots.front()) == 0.0 &&
                    zeroing.EvaluateSecondDerivative(knots.back()) == 0.0,
                "the zeroing rule reported a curvature at a boundary knot");
        // The first derivative's rule is the strict one, so the boundary knot
        // itself keeps its interior slope under both rules.
        Require(zeroing.EvaluateFirstDerivative(knots.front()) ==
                    holding.EvaluateFirstDerivative(knots.front()),
                "the zeroing rule changed the slope at a boundary knot");
        Require(zeroing.Evaluate(0.9) == holding.Evaluate(0.9),
                "the outside rule changed an interior value");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("profile interpolants verified");
    return 0;
}
