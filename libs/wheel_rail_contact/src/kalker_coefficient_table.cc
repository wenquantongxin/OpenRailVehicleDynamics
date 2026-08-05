#include "orvd/wheel_rail_contact/kalker_coefficient_table.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace orvd::wheel_rail_contact {
namespace {

constexpr std::size_t kNodes = KalkerCoefficientTable::kNodeCount;

// The three tabulated Poisson ratios. Three is the whole tabulation, not a
// sample of it, which is why the collapse below is a quadratic and not a spline.
constexpr std::array<double, 3> kPoissonNodes{0.0, 0.25, 0.5};

// The tabulated semi-axis ratios. The upper half is the reciprocal of the lower
// half and is written that way: typing the decimal expansions instead would
// give the same doubles here but would hide that the grid is symmetric under
// a/b -> b/a even though the interpolant over it is not.
constexpr std::array<double, kNodes> kSemiAxisRatioNodes{
    0.1,       0.2,       0.3,       0.4,       0.5,       0.6,      0.7,
    0.8,       0.9,       1.0,       1.0 / 0.9, 1.0 / 0.8, 1.0 / 0.7,
    1.0 / 0.6, 1.0 / 0.5, 1.0 / 0.4, 1.0 / 0.3, 1.0 / 0.2, 1.0 / 0.1};

// Kalker's tabulated coefficients, one row per Poisson node.
//
// These are measurements of a solved elasticity problem, transcribed. Nothing
// derives them and nothing should adjust them. The longitudinal and lateral
// rows coincide at zero Poisson ratio because the two directions are then
// indistinguishable — that is a property of the underlying solution, and a
// transcription error that broke it would be caught by it.
constexpr std::array<std::array<double, kNodes>, 3> kLongitudinal{{
    {2.51, 2.59, 2.68, 2.78, 2.88, 2.98, 3.09, 3.19, 3.29, 3.4, 3.51, 3.65,
     3.82, 4.06, 4.37, 4.84, 5.57, 6.96, 10.7},
    {3.31, 3.37, 3.44, 3.53, 3.62, 3.72, 3.81, 3.91, 4.01, 4.12, 4.22, 4.36,
     4.54, 4.78, 5.1, 5.57, 6.34, 7.78, 11.7},
    {4.85, 4.81, 4.8, 4.82, 4.83, 4.91, 4.97, 5.05, 5.12, 5.2, 5.3, 5.42, 5.58,
     5.8, 6.11, 6.57, 7.34, 8.82, 12.9},
}};

constexpr std::array<std::array<double, kNodes>, 3> kLateral{{
    {2.51, 2.59, 2.68, 2.78, 2.88, 2.98, 3.09, 3.19, 3.29, 3.4, 3.51, 3.65,
     3.82, 4.06, 4.37, 4.84, 5.57, 6.96, 10.7},
    {2.52, 2.63, 2.75, 2.88, 3.01, 3.14, 3.28, 3.41, 3.54, 3.67, 3.81, 3.99,
     4.21, 4.5, 4.9, 5.48, 6.4, 8.14, 12.8},
    {2.53, 2.66, 2.81, 2.98, 3.14, 3.31, 3.48, 3.65, 3.82, 3.98, 4.16, 4.39,
     4.67, 5.04, 5.56, 6.31, 7.51, 9.79, 16.0},
}};

constexpr std::array<std::array<double, kNodes>, 3> kLateralSpin{{
    {0.334, 0.483, 0.607, 0.72, 0.827, 0.93, 1.03, 1.13, 1.23, 1.33, 1.44, 1.58,
     1.76, 2.01, 2.35, 2.88, 3.79, 5.72, 12.2},
    {0.473, 0.603, 0.715, 0.823, 0.929, 1.03, 1.14, 1.25, 1.36, 1.47, 1.59,
     1.75, 1.95, 2.23, 2.62, 3.24, 4.32, 6.63, 14.6},
    {0.731, 0.809, 0.889, 0.977, 1.07, 1.18, 1.29, 1.4, 1.51, 1.63, 1.77, 1.94,
     2.18, 2.5, 2.96, 3.7, 5.01, 7.89, 18.0},
}};

// The unique quadratic through three points, evaluated at one abscissa.
//
// Written as an accumulation of Lagrange weights rather than as solved
// coefficients: the weights are exactly the Kronecker delta at the nodes, so
// the interpolant reproduces a tabulated Poisson ratio without a special case.
double QuadraticThroughThree(const std::array<double, 3>& abscissae,
                             const std::array<double, 3>& values, double at) {
    double result = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
        double weight = 1.0;
        for (std::size_t j = 0; j < 3; ++j) {
            if (j != i) {
                weight *= (at - abscissae[j]) / (abscissae[i] - abscissae[j]);
            }
        }
        result += weight * values[i];
    }
    return result;
}

std::array<double, kNodes> CollapsePoissonAxis(
    const std::array<std::array<double, kNodes>, 3>& rows, double poisson_ratio) {
    std::array<double, kNodes> collapsed{};
    for (std::size_t column = 0; column < kNodes; ++column) {
        const std::array<double, 3> values{rows[0][column], rows[1][column],
                                           rows[2][column]};
        collapsed[column] =
            QuadraticThroughThree(kPoissonNodes, values, poisson_ratio);
    }
    return collapsed;
}

// Piecewise-linear lookup with the ends held flat.
//
// The clamping arms are unreachable for anything but the exact endpoints,
// because the caller gates the ratio to the tabulated interval first. They are
// here so this function is total on its own terms rather than on its caller's.
double InterpolateAcrossNodes(const std::array<double, kNodes>& nodes,
                              const std::array<double, kNodes>& values,
                              double at) {
    if (at <= nodes.front()) {
        return values.front();
    }
    if (at >= nodes.back()) {
        return values.back();
    }
    const auto upper = std::upper_bound(nodes.begin(), nodes.end(), at);
    const std::size_t high = static_cast<std::size_t>(upper - nodes.begin());
    const std::size_t low = high - 1;
    if (nodes[high] == nodes[low]) {
        return values[low];
    }
    const double fraction = (at - nodes[low]) / (nodes[high] - nodes[low]);
    return values[low] + (values[high] - values[low]) * fraction;
}

// The slender-ellipse expansion, for a contact patch so elongated that the
// tabulation has nothing to say about it.
//
// Two regimes, and they are not each other's mirror image. When the patch is
// slender across the rolling direction the longitudinal and lateral
// coefficients lose their shape dependence entirely and become constants; when
// it is slender along it they grow without bound. The asymmetry is physical.
KalkerCoefficients AsymptoticCoefficients(double semi_axis_ratio,
                                          double poisson_ratio) {
    const double pi = std::numbers::pi;
    const double slenderness =
        std::min(semi_axis_ratio, 1.0 / semi_axis_ratio);
    const double logarithm = std::log(16.0 / (slenderness * slenderness));

    KalkerCoefficients coefficients;
    if (semi_axis_ratio < KalkerCoefficientTable::kSmallestTabulatedRatio) {
        coefficients.longitudinal = (pi * pi) / 4.0 / (1.0 - poisson_ratio);
        coefficients.lateral = (pi * pi) / 4.0;
        coefficients.lateral_spin =
            pi * std::sqrt(slenderness) / 3.0 / (1.0 - poisson_ratio) *
            (1.0 + poisson_ratio * (0.5 * logarithm + std::log(4.0) - 5.0));
        return coefficients;
    }

    const double shifted = logarithm - 2.0 * poisson_ratio;
    const double weighted = (1.0 - poisson_ratio) * logarithm + 2.0 * poisson_ratio;
    coefficients.longitudinal = 2.0 * pi / shifted / slenderness *
                                (1.0 + (3.0 - std::log(4.0)) / shifted);
    coefficients.lateral =
        2.0 * pi / slenderness *
        (1.0 + ((1.0 - poisson_ratio) * (3.0 - std::log(4.0))) / weighted) /
        weighted;
    coefficients.lateral_spin =
        2.0 * pi / 3.0 / std::pow(slenderness, 1.5) /
        ((1.0 - poisson_ratio) * logarithm - 2.0 + 4.0 * poisson_ratio);
    return coefficients;
}

}  // namespace

KalkerCoefficientTable KalkerCoefficientTable::ForPoissonRatio(
    double poisson_ratio, OutsideTableRule outside_rule) {
    if (!(poisson_ratio >= 0.0) || !(poisson_ratio <= 0.5)) {
        throw std::invalid_argument(
            "KalkerCoefficientTable::ForPoissonRatio: the Poisson ratio " +
            std::to_string(poisson_ratio) +
            " is outside the tabulated range [0, 0.5]; the collapse "
            "interpolates between the three tabulated ratios and does not "
            "extrapolate past them");
    }
    KalkerCoefficientTable table;
    table.longitudinal_ = CollapsePoissonAxis(kLongitudinal, poisson_ratio);
    table.lateral_ = CollapsePoissonAxis(kLateral, poisson_ratio);
    table.lateral_spin_ = CollapsePoissonAxis(kLateralSpin, poisson_ratio);
    table.poisson_ratio_ = poisson_ratio;
    table.outside_rule_ = outside_rule;
    return table;
}

KalkerCoefficients KalkerCoefficientTable::At(double semi_axis_ratio) const {
    if (semi_axis_ratio >= kSmallestTabulatedRatio &&
        semi_axis_ratio <= kLargestTabulatedRatio) {
        KalkerCoefficients coefficients;
        coefficients.longitudinal = InterpolateAcrossNodes(
            kSemiAxisRatioNodes, longitudinal_, semi_axis_ratio);
        coefficients.lateral =
            InterpolateAcrossNodes(kSemiAxisRatioNodes, lateral_, semi_axis_ratio);
        coefficients.lateral_spin = InterpolateAcrossNodes(
            kSemiAxisRatioNodes, lateral_spin_, semi_axis_ratio);
        return coefficients;
    }
    if (outside_rule_ == OutsideTableRule::kRefuse) {
        throw std::out_of_range(
            "KalkerCoefficientTable::At: the semi-axis ratio " +
            std::to_string(semi_axis_ratio) +
            " is outside the tabulated range [0.1, 10] and this table was "
            "built to refuse rather than extrapolate");
    }
    // A ratio of zero or a non-finite one would make the slenderness and its
    // logarithm meaningless, and neither is a contact ellipse. The comparison
    // is written so that a NaN falls through to the refusal.
    if (!(semi_axis_ratio > 0.0) || !std::isfinite(semi_axis_ratio)) {
        throw std::out_of_range(
            "KalkerCoefficientTable::At: the semi-axis ratio " +
            std::to_string(semi_axis_ratio) +
            " is not a positive finite number, so no contact ellipse has it");
    }
    return AsymptoticCoefficients(semi_axis_ratio, poisson_ratio_);
}

std::span<const double, KalkerCoefficientTable::kNodeCount>
KalkerCoefficientTable::semi_axis_ratio_nodes() const {
    // One grid for every table. Storing a copy per table would be one more
    // thing that could be edited into disagreeing with the coefficients.
    return std::span<const double, kNodeCount>(kSemiAxisRatioNodes);
}

}  // namespace orvd::wheel_rail_contact
