#include "orvd/wheel_rail_contact/monotone_cubic_interpolant.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace orvd::wheel_rail_contact {
namespace {

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("monotone cubic interpolant: " + detail);
}

void RequireUsableNodes(std::span<const double> knots,
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
    for (std::size_t node = 0; node + 1 < knots.size(); ++node) {
        if (!(knots[node + 1] - knots[node] > 0.0)) {
            Reject("the knots must be strictly increasing, but the one at index " +
                   std::to_string(node + 1) + " does not exceed its predecessor");
        }
    }
}

// The one-sided three-point derivative at an end, limited twice: it is dropped
// to zero when it disagrees in sign with the secant it belongs to, and clipped
// to three times that secant when the two secants straddle a turning point. The
// second limit is what keeps an end segment from overshooting the way an
// unlimited one-sided formula does.
double EndpointSlope(double near_spacing, double far_spacing, double near_secant,
                     double far_secant) {
    const double slope =
        ((2.0 * near_spacing + far_spacing) * near_secant - near_spacing * far_secant) /
        (near_spacing + far_spacing);
    if (slope * near_secant <= 0.0) {
        return 0.0;
    }
    if (near_secant * far_secant < 0.0 &&
        std::abs(slope) > std::abs(3.0 * near_secant)) {
        return 3.0 * near_secant;
    }
    return slope;
}

}  // namespace

void ComputeShapePreservingNodalSlopes(std::span<const double> knots,
                                       std::span<const double> values,
                                       std::span<double> slopes_out) {
    // The allocating form. The rule itself lives in the overload below; this one
    // exists so that a caller who is not on a hot path does not have to think
    // about where two temporary arrays come from.
    const std::size_t count = knots.size();
    std::vector<double> spacings(count == 0 ? 0 : count - 1, 0.0);
    std::vector<double> secants(count == 0 ? 0 : count - 1, 0.0);
    ComputeShapePreservingNodalSlopes(knots, values, slopes_out, spacings, secants);
}

void ComputeShapePreservingNodalSlopes(std::span<const double> knots,
                                       std::span<const double> values,
                                       std::span<double> slopes_out,
                                       std::span<double> spacings,
                                       std::span<double> secants) {
    RequireUsableNodes(knots, values);
    if (slopes_out.size() != knots.size()) {
        Reject("the slope output must have one entry per knot, but there are " +
               std::to_string(slopes_out.size()) + " and " +
               std::to_string(knots.size()));
    }

    const std::size_t count = knots.size();
    if (spacings.size() < count - 1 || secants.size() < count - 1) {
        Reject("the two scratch spans must each hold one entry per interval, "
               "which is " + std::to_string(count - 1) + ", but they hold " +
               std::to_string(spacings.size()) + " and " +
               std::to_string(secants.size()));
    }
    if (count == 2) {
        const double secant = (values[1] - values[0]) / (knots[1] - knots[0]);
        slopes_out[0] = secant;
        slopes_out[1] = secant;
        return;
    }

    for (std::size_t node = 0; node + 1 < count; ++node) {
        spacings[node] = knots[node + 1] - knots[node];
        secants[node] = (values[node + 1] - values[node]) / spacings[node];
    }

    slopes_out[0] =
        EndpointSlope(spacings[0], spacings[1], secants[0], secants[1]);
    slopes_out[count - 1] =
        EndpointSlope(spacings[count - 2], spacings[count - 3], secants[count - 2],
                      secants[count - 3]);

    for (std::size_t node = 1; node + 1 < count; ++node) {
        const double left_secant = secants[node - 1];
        const double right_secant = secants[node];
        if (left_secant * right_secant <= 0.0) {
            slopes_out[node] = 0.0;
            continue;
        }
        // The doubled interval on each weight is the one on the far side of the
        // secant that weight divides. Pairing them the other way round is
        // invisible on an equally spaced grid and wrong everywhere else.
        const double left_spacing = spacings[node - 1];
        const double right_spacing = spacings[node];
        const double left_weight = 2.0 * right_spacing + left_spacing;
        const double right_weight = right_spacing + 2.0 * left_spacing;
        slopes_out[node] = (left_weight + right_weight) /
                           (left_weight / left_secant + right_weight / right_secant);
    }
}

MonotoneCubicInterpolant::MonotoneCubicInterpolant(
    std::vector<double> knots, std::vector<double> values,
    std::vector<double> nodal_slopes, OutsideDerivativeRule outside_rule)
    : knots_(std::move(knots)),
      values_(std::move(values)),
      nodal_slopes_(std::move(nodal_slopes)),
      outside_rule_(outside_rule) {
    const std::size_t count = knots_.size();
    segments_.resize(count - 1);
    for (std::size_t segment = 0; segment + 1 < count; ++segment) {
        const double spacing = knots_[segment + 1] - knots_[segment];
        const double inverse_spacing = 1.0 / spacing;
        const double left_value = values_[segment];
        const double right_value = values_[segment + 1];
        const double left_slope = nodal_slopes_[segment];
        const double right_slope = nodal_slopes_[segment + 1];
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

MonotoneCubicInterpolant MonotoneCubicInterpolant::FromValues(
    std::span<const double> knots, std::span<const double> values,
    OutsideDerivativeRule outside_rule) {
    RequireUsableNodes(knots, values);
    std::vector<double> slopes(knots.size(), 0.0);
    ComputeShapePreservingNodalSlopes(knots, values, slopes);
    return MonotoneCubicInterpolant(
        std::vector<double>(knots.begin(), knots.end()),
        std::vector<double>(values.begin(), values.end()), std::move(slopes),
        outside_rule);
}

MonotoneCubicInterpolant MonotoneCubicInterpolant::FromNodalSlopes(
    std::span<const double> knots, std::span<const double> values,
    std::span<const double> nodal_slopes, OutsideDerivativeRule outside_rule) {
    RequireUsableNodes(knots, values);
    if (nodal_slopes.size() != knots.size()) {
        Reject("the nodal slopes must have one entry per knot, but there are " +
               std::to_string(nodal_slopes.size()) + " and " +
               std::to_string(knots.size()));
    }
    for (const double slope : nodal_slopes) {
        if (!std::isfinite(slope)) {
            Reject("a nodal slope is not finite");
        }
    }
    return MonotoneCubicInterpolant(
        std::vector<double>(knots.begin(), knots.end()),
        std::vector<double>(values.begin(), values.end()),
        std::vector<double>(nodal_slopes.begin(), nodal_slopes.end()),
        outside_rule);
}

MonotoneCubicInterpolant::Location MonotoneCubicInterpolant::Locate(
    double abscissa) const {
    const std::size_t last_segment = segments_.size() - 1;
    if (!(abscissa > knots_.front())) {
        // Not-a-number lands here rather than indexing past the last segment,
        // which is what an unguarded upper bound would do with it.
        return Location{0, 0.0};
    }
    if (abscissa >= knots_.back()) {
        return Location{last_segment, 1.0};
    }
    const auto upper = std::upper_bound(knots_.begin(), knots_.end(), abscissa);
    const std::size_t upper_index = static_cast<std::size_t>(upper - knots_.begin());
    const std::size_t segment = std::min(upper_index - 1, last_segment);
    const double local_parameter = std::clamp(
        (abscissa - knots_[segment]) * segments_[segment].inverse_spacing, 0.0, 1.0);
    return Location{segment, local_parameter};
}

double MonotoneCubicInterpolant::Evaluate(double abscissa) const {
    const Location location = Locate(abscissa);
    const SegmentCoefficients& coefficients = segments_[location.segment];
    const double t = location.local_parameter;
    return ((coefficients.cubic * t + coefficients.quadratic) * t +
            coefficients.linear) *
               t +
           coefficients.constant;
}

double MonotoneCubicInterpolant::EvaluateWithSegmentHint(
    double abscissa, std::size_t& segment_hint) const {
    const std::size_t last_segment = segments_.size() - 1;
    Location location;
    if (!(abscissa > knots_.front())) {
        // Preserve `Locate`'s left-end and not-a-number behaviour.
        location = Location{0, 0.0};
    } else if (abscissa >= knots_.back()) {
        // `Locate` makes the right endpoint exactly one. Recomputing the local
        // parameter on the adjacent fast path can instead round one ulp below
        // one for a non-binary interval spacing.
        location = Location{last_segment, 1.0};
    } else if (segment_hint <= last_segment &&
               abscissa >= knots_[segment_hint] &&
               abscissa < knots_[segment_hint + 1]) {
        location = Location{
            segment_hint,
            std::clamp((abscissa - knots_[segment_hint]) *
                           segments_[segment_hint].inverse_spacing,
                       0.0, 1.0)};
    } else if (segment_hint < last_segment &&
               abscissa >= knots_[segment_hint + 1] &&
               (segment_hint + 1 == last_segment ||
                abscissa < knots_[segment_hint + 2])) {
        const std::size_t segment = segment_hint + 1;
        location = Location{
            segment,
            std::clamp((abscissa - knots_[segment]) *
                           segments_[segment].inverse_spacing,
                       0.0, 1.0)};
    } else if (segment_hint > 0 && segment_hint <= last_segment &&
               abscissa >= knots_[segment_hint - 1] &&
               abscissa < knots_[segment_hint]) {
        const std::size_t segment = segment_hint - 1;
        location = Location{
            segment,
            std::clamp((abscissa - knots_[segment]) *
                           segments_[segment].inverse_spacing,
                       0.0, 1.0)};
    } else {
        location = Locate(abscissa);
    }
    segment_hint = location.segment;
    const SegmentCoefficients& coefficients = segments_[location.segment];
    const double t = location.local_parameter;
    return ((coefficients.cubic * t + coefficients.quadratic) * t +
            coefficients.linear) *
               t +
           coefficients.constant;
}

double MonotoneCubicInterpolant::EvaluateFirstDerivative(double abscissa) const {
    if (outside_rule_ == OutsideDerivativeRule::kZeroOutsideKnots &&
        (abscissa < knots_.front() || abscissa > knots_.back())) {
        return 0.0;
    }
    const Location location = Locate(abscissa);
    const SegmentCoefficients& coefficients = segments_[location.segment];
    const double t = location.local_parameter;
    return coefficients.inverse_spacing *
           (coefficients.linear +
            t * (2.0 * coefficients.quadratic + 3.0 * coefficients.cubic * t));
}

double MonotoneCubicInterpolant::EvaluateSecondDerivative(double abscissa) const {
    if (outside_rule_ == OutsideDerivativeRule::kZeroOutsideKnots &&
        (abscissa <= knots_.front() || abscissa >= knots_.back())) {
        return 0.0;
    }
    const Location location = Locate(abscissa);
    const SegmentCoefficients& coefficients = segments_[location.segment];
    const double t = location.local_parameter;
    return coefficients.inverse_spacing_squared *
           (2.0 * coefficients.quadratic + 6.0 * coefficients.cubic * t);
}

}  // namespace orvd::wheel_rail_contact
