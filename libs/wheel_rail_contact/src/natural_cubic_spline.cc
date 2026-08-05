#include "orvd/wheel_rail_contact/natural_cubic_spline.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace orvd::wheel_rail_contact {
namespace {

// The spacing-equality tolerance. Absolute in the abscissa's own units plus a
// relative part referenced to the first spacing, because a profile stated in
// metres and a station coordinate stated in metres differ by four orders of
// magnitude in spacing and one fixed number cannot serve both.
constexpr double kUniformSpacingAbsoluteTolerance = 1.0e-12;
constexpr double kUniformSpacingRelativeTolerance = 1.0e-9;

// The pivot below which the tridiagonal solve is refused. Absolute and
// unscaled: a scaled threshold would accept systems whose solution is noise.
constexpr double kSingularPivot = 1.0e-15;

// Snapping tolerance on the local parameter, so that evaluating at a knot from
// either side returns the tabulated value exactly rather than the polynomial's
// last-bit approximation to it.
constexpr double kLocalParameterSnap = 1.0e-12;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("natural cubic spline: " + detail);
}

[[noreturn]] void RejectSingular(const std::string& where) {
    throw std::runtime_error("natural cubic spline: singular tridiagonal system" +
                             where);
}

bool SpacingsAreUniform(std::span<const double> spacings) {
    const double reference = spacings.front();
    const double tolerance = kUniformSpacingAbsoluteTolerance +
                             kUniformSpacingRelativeTolerance * std::abs(reference);
    for (const double spacing : spacings) {
        if (!(std::abs(spacing - reference) <= tolerance)) {
            return false;
        }
    }
    return true;
}

// Thomas elimination without pivoting. Row i reads
// sub[i]*x[i-1] + diagonal[i]*x[i] + super[i]*x[i+1] = right_hand_side[i].
std::vector<double> SolveTridiagonal(std::span<const double> sub,
                                     std::span<const double> diagonal,
                                     std::span<const double> super,
                                     std::span<const double> right_hand_side) {
    const std::size_t rows = diagonal.size();
    std::vector<double> solution(rows, 0.0);
    if (rows == 1) {
        if (std::abs(diagonal[0]) < kSingularPivot) {
            RejectSingular(" at row 0");
        }
        solution[0] = right_hand_side[0] / diagonal[0];
        return solution;
    }

    std::vector<double> super_prime(rows, 0.0);
    std::vector<double> right_prime(rows, 0.0);
    if (std::abs(diagonal[0]) < kSingularPivot) {
        RejectSingular(" at row 0");
    }
    super_prime[0] = super[0] / diagonal[0];
    right_prime[0] = right_hand_side[0] / diagonal[0];
    for (std::size_t row = 1; row < rows; ++row) {
        const double pivot = diagonal[row] - sub[row] * super_prime[row - 1];
        if (std::abs(pivot) < kSingularPivot) {
            RejectSingular("");
        }
        super_prime[row] = (row + 1 < rows) ? (super[row] / pivot) : 0.0;
        right_prime[row] =
            (right_hand_side[row] - sub[row] * right_prime[row - 1]) / pivot;
    }

    solution[rows - 1] = right_prime[rows - 1];
    for (std::size_t row = rows - 1; row-- > 0;) {
        solution[row] = right_prime[row] - super_prime[row] * solution[row + 1];
    }
    return solution;
}

// Equally spaced knots: the natural boundary condition is expressible directly
// in slopes, so the system is solved for what the segment form wants and no
// moment ever appears. Its diagonal does not depend on the spacing, which is
// why this path survives spacings the general one refuses.
std::vector<double> SolveNodalSlopesUniform(std::span<const double> values,
                                            double spacing) {
    const std::size_t count = values.size();
    if (count == 2) {
        const double slope = (values[1] - values[0]) / spacing;
        return {slope, slope};
    }

    std::vector<double> sub(count, 1.0);
    std::vector<double> diagonal(count, 4.0);
    std::vector<double> super(count, 1.0);
    std::vector<double> right_hand_side(count, 0.0);
    sub[0] = 0.0;
    diagonal[0] = 2.0;
    diagonal[count - 1] = 2.0;
    super[count - 1] = 0.0;

    const double inverse_spacing = 1.0 / spacing;
    right_hand_side[0] = 3.0 * (values[1] - values[0]) * inverse_spacing;
    for (std::size_t node = 1; node + 1 < count; ++node) {
        right_hand_side[node] =
            3.0 * (values[node + 1] - values[node - 1]) * inverse_spacing;
    }
    right_hand_side[count - 1] =
        3.0 * (values[count - 1] - values[count - 2]) * inverse_spacing;

    return SolveTridiagonal(sub, diagonal, super, right_hand_side);
}

// Unequally spaced knots: the classical moment system over the interior nodes.
// The natural boundary condition is imposed by leaving the two end moments out
// of the system rather than by adding rows for them, so they are exactly zero.
std::vector<double> SolveNodalSlopesGeneral(std::span<const double> values,
                                            std::span<const double> spacings) {
    const std::size_t count = values.size();
    std::vector<double> moments(count, 0.0);
    if (count > 2) {
        const std::size_t interior = count - 2;
        std::vector<double> sub(interior, 0.0);
        std::vector<double> diagonal(interior, 0.0);
        std::vector<double> super(interior, 0.0);
        std::vector<double> right_hand_side(interior, 0.0);
        for (std::size_t row = 0; row < interior; ++row) {
            const std::size_t node = row + 1;
            const double before = spacings[node - 1];
            const double after = spacings[node];
            sub[row] = (row > 0) ? before : 0.0;
            diagonal[row] = 2.0 * (before + after);
            super[row] = (row + 1 < interior) ? after : 0.0;
            const double forward_slope = (values[node + 1] - values[node]) / after;
            const double backward_slope = (values[node] - values[node - 1]) / before;
            right_hand_side[row] = 6.0 * (forward_slope - backward_slope);
        }
        const std::vector<double> interior_moments =
            SolveTridiagonal(sub, diagonal, super, right_hand_side);
        for (std::size_t row = 0; row < interior; ++row) {
            moments[row + 1] = interior_moments[row];
        }
    }

    std::vector<double> slopes(count, 0.0);
    for (std::size_t node = 0; node + 1 < count; ++node) {
        const double spacing = spacings[node];
        const double rise = values[node + 1] - values[node];
        slopes[node] = rise / spacing -
                       (spacing / 6.0) * (2.0 * moments[node] + moments[node + 1]);
    }
    const double last_spacing = spacings[count - 2];
    const double last_rise = values[count - 1] - values[count - 2];
    slopes[count - 1] =
        last_rise / last_spacing +
        (last_spacing / 6.0) * (moments[count - 2] + 2.0 * moments[count - 1]);
    return slopes;
}

}  // namespace

NaturalCubicSpline::NaturalCubicSpline(std::span<const double> knots,
                                       std::span<const double> values) {
    if (knots.size() != values.size()) {
        Reject("the knots and the values must have the same length, but there are " +
               std::to_string(knots.size()) + " and " +
               std::to_string(values.size()));
    }
    if (knots.size() < 2) {
        Reject("at least two points are required, but " +
               std::to_string(knots.size()) + " were given");
    }
    for (const double knot : knots) {
        if (!std::isfinite(knot)) {
            Reject("a knot is not finite");
        }
    }
    for (const double value : values) {
        if (!std::isfinite(value)) {
            Reject("a value is not finite");
        }
    }

    const std::size_t count = knots.size();
    std::vector<double> spacings(count - 1, 0.0);
    for (std::size_t node = 0; node + 1 < count; ++node) {
        const double spacing = knots[node + 1] - knots[node];
        if (!(spacing > 0.0)) {
            Reject("the knots must be strictly increasing, but the one at index " +
                   std::to_string(node + 1) + " does not exceed its predecessor");
        }
        spacings[node] = spacing;
    }

    values_.assign(values.begin(), values.end());
    first_knot_ = knots.front();
    last_knot_ = knots.back();
    is_uniform_ = SpacingsAreUniform(spacings);

    if (is_uniform_) {
        uniform_spacing_ = spacings.front();
        inverse_uniform_spacing_ = 1.0 / uniform_spacing_;
        nodal_slopes_ = SolveNodalSlopesUniform(values_, uniform_spacing_);
    } else {
        knots_.assign(knots.begin(), knots.end());
        nodal_slopes_ = SolveNodalSlopesGeneral(values_, spacings);
    }

    segments_.resize(count - 1);
    for (std::size_t segment = 0; segment + 1 < count; ++segment) {
        const double left_value = values_[segment];
        const double right_value = values_[segment + 1];
        const double left_slope = nodal_slopes_[segment];
        const double right_slope = nodal_slopes_[segment + 1];
        const double spacing = is_uniform_ ? uniform_spacing_ : spacings[segment];
        const double inverse_spacing =
            is_uniform_ ? inverse_uniform_spacing_ : (1.0 / spacing);
        SegmentCoefficients& coefficients = segments_[segment];
        coefficients.constant = left_value;
        coefficients.linear = spacing * left_slope;
        coefficients.quadratic = -3.0 * left_value + 3.0 * right_value -
                                 2.0 * spacing * left_slope - spacing * right_slope;
        coefficients.cubic = 2.0 * left_value - 2.0 * right_value +
                             spacing * left_slope + spacing * right_slope;
        coefficients.inverse_spacing = inverse_spacing;
        coefficients.inverse_spacing_squared = inverse_spacing * inverse_spacing;
    }
}

NaturalCubicSpline::Location NaturalCubicSpline::Locate(double abscissa) const {
    const std::size_t last_segment = segments_.size() - 1;
    if (abscissa <= first_knot_) {
        return Location{0, 0.0};
    }
    if (abscissa >= last_knot_) {
        return Location{last_segment, 1.0};
    }

    std::size_t segment = 0;
    double local_parameter = 0.0;
    if (is_uniform_) {
        const double scaled = (abscissa - first_knot_) * inverse_uniform_spacing_;
        const double floored = std::floor(scaled);
        if (!(floored < static_cast<double>(segments_.size()))) {
            return Location{last_segment, 1.0};
        }
        segment = static_cast<std::size_t>(floored);
        local_parameter = scaled - floored;
    } else {
        const auto upper =
            std::upper_bound(knots_.begin(), knots_.end(), abscissa);
        const std::size_t upper_index =
            static_cast<std::size_t>(upper - knots_.begin());
        if (upper_index == 0) {
            return Location{0, 0.0};
        }
        segment = upper_index - 1;
        if (segment > last_segment) {
            return Location{last_segment, 1.0};
        }
        local_parameter =
            (abscissa - knots_[segment]) * segments_[segment].inverse_spacing;
    }

    if (local_parameter <= kLocalParameterSnap) {
        return Location{segment, 0.0};
    }
    if (local_parameter >= 1.0 - kLocalParameterSnap) {
        if (segment + 1 > last_segment) {
            return Location{last_segment, 1.0};
        }
        return Location{segment + 1, 0.0};
    }
    return Location{segment, local_parameter};
}

double NaturalCubicSpline::Evaluate(double abscissa) const {
    if (abscissa <= first_knot_) {
        return values_.front();
    }
    if (abscissa >= last_knot_) {
        return values_.back();
    }
    const Location location = Locate(abscissa);
    const SegmentCoefficients& coefficients = segments_[location.segment];
    const double t = location.local_parameter;
    return ((coefficients.cubic * t + coefficients.quadratic) * t +
            coefficients.linear) *
               t +
           coefficients.constant;
}

double NaturalCubicSpline::EvaluateFirstDerivative(double abscissa) const {
    // Strictly outside only. At a boundary knot the interior one-sided slope is
    // the answer a consumer needs, so the flat hold of `Evaluate` deliberately
    // does not extend to the derivative there.
    if (abscissa < first_knot_ || abscissa > last_knot_) {
        return 0.0;
    }
    const Location location = Locate(abscissa);
    const SegmentCoefficients& coefficients = segments_[location.segment];
    const double t = location.local_parameter;
    return coefficients.inverse_spacing *
           (coefficients.linear + 2.0 * coefficients.quadratic * t +
            3.0 * coefficients.cubic * t * t);
}

double NaturalCubicSpline::EvaluateSecondDerivative(double abscissa) const {
    // Inclusive at both boundary knots, which makes the natural boundary
    // condition exact instead of leaving rounding noise where the contract says
    // zero.
    if (abscissa <= first_knot_ || abscissa >= last_knot_) {
        return 0.0;
    }
    const Location location = Locate(abscissa);
    const SegmentCoefficients& coefficients = segments_[location.segment];
    const double t = location.local_parameter;
    return coefficients.inverse_spacing_squared *
           (2.0 * coefficients.quadratic + 6.0 * coefficients.cubic * t);
}

}  // namespace orvd::wheel_rail_contact
