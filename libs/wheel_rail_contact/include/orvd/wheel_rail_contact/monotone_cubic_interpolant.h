#pragma once

#include <cstddef>
#include <span>
#include <utility>
#include <vector>

// A shape-preserving piecewise cubic Hermite interpolant over owned knots.
//
// Two things distinguish it from the natural cubic spline beside it. It is only
// C1, not C2, and in exchange it never overshoots: between two knots the curve
// stays inside the interval the two values span, and a monotone run of values
// produces a monotone curve. A profile measured across a flange root has a
// nearly discontinuous change of slope there, and a C2 interpolant answers that
// with a ripple on both sides; the ripple is small in the value and large in the
// derivative, which is the quantity a contact angle is made of.
//
// The nodal derivatives can come from two places. `FromValues` derives them by
// the weighted harmonic mean that produces the shape preservation, zeroing the
// derivative wherever the two neighbouring secants disagree in sign or either
// is zero. `FromNodalSlopes` takes them from the caller, which is how a natural
// cubic spline's nodal slopes are given this evaluator: the result then passes
// through every knot with the spline's own value and slope while behaving like
// a Hermite cubic in between.
//
// Outside the knot range the value is held flat. The derivatives are, by
// default, held at their endpoint values rather than dropped to zero, which is
// deliberately inconsistent with the flat value and is what a consumer wants
// when it is extrapolating a surface it knows continues. A consumer that wants
// the consistent reading asks for it at construction, and then the first
// derivative is zero strictly outside and the second derivative is zero at and
// outside the boundary knots. The asymmetry between those two comparisons is
// the same one the natural spline makes and for the same reason.

namespace orvd::wheel_rail_contact {

// Whether the reported derivatives outside the knot range follow the flat value
// or keep the interior endpoint derivative.
enum class OutsideDerivativeRule {
    kHoldEndpointDerivative,
    kZeroOutsideKnots,
};

class MonotoneCubicInterpolant {
   public:
    // Derives the nodal derivatives from the values themselves.
    //
    // Throws std::invalid_argument when the two spans differ in length, when
    // fewer than two points are given, when any number is not finite, or when
    // the knots are not strictly increasing.
    [[nodiscard]] static MonotoneCubicInterpolant FromValues(
        std::span<const double> knots, std::span<const double> values,
        OutsideDerivativeRule outside_rule =
            OutsideDerivativeRule::kHoldEndpointDerivative);

    // Uses the nodal derivatives the caller supplies. The shape preservation is
    // then the caller's responsibility: nothing here limits the slopes it was
    // given.
    //
    // A cubic segment is fixed by its two end values and two end slopes, so
    // handing this the nodal slopes of another interpolant over the same points
    // reproduces that interpolant exactly between the knots. What the caller
    // gains is not a different curve but a different treatment of its ends.
    //
    // Throws under the same conditions as `FromValues`, and additionally when
    // the slopes span differs in length from the knots.
    [[nodiscard]] static MonotoneCubicInterpolant FromNodalSlopes(
        std::span<const double> knots, std::span<const double> values,
        std::span<const double> nodal_slopes,
        OutsideDerivativeRule outside_rule =
            OutsideDerivativeRule::kHoldEndpointDerivative);

    [[nodiscard]] double Evaluate(double abscissa) const;
    [[nodiscard]] double EvaluateFirstDerivative(double abscissa) const;
    [[nodiscard]] double EvaluateSecondDerivative(double abscissa) const;

    [[nodiscard]] std::span<const double> knots() const & { return knots_; }
    [[nodiscard]] std::vector<double> knots() && { return std::move(knots_); }
    [[nodiscard]] std::vector<double> knots() const && { return knots_; }
    [[nodiscard]] std::span<const double> values() const & { return values_; }
    [[nodiscard]] std::vector<double> values() && { return std::move(values_); }
    [[nodiscard]] std::vector<double> values() const && { return values_; }
    [[nodiscard]] std::span<const double> nodal_slopes() const & {
        return nodal_slopes_;
    }
    [[nodiscard]] std::vector<double> nodal_slopes() && {
        return std::move(nodal_slopes_);
    }
    [[nodiscard]] std::vector<double> nodal_slopes() const && {
        return nodal_slopes_;
    }
    [[nodiscard]] std::size_t size() const { return knots_.size(); }

   private:
    struct SegmentCoefficients {
        double constant{0.0};
        double linear{0.0};
        double quadratic{0.0};
        double cubic{0.0};
        double inverse_spacing{0.0};
        double inverse_spacing_squared{0.0};
    };

    struct Location {
        std::size_t segment{0};
        double local_parameter{0.0};
    };

    MonotoneCubicInterpolant(std::vector<double> knots, std::vector<double> values,
                             std::vector<double> nodal_slopes,
                             OutsideDerivativeRule outside_rule);

    [[nodiscard]] Location Locate(double abscissa) const;

    std::vector<double> knots_;
    std::vector<double> values_;
    std::vector<double> nodal_slopes_;
    std::vector<SegmentCoefficients> segments_;
    OutsideDerivativeRule outside_rule_{
        OutsideDerivativeRule::kHoldEndpointDerivative};
};

// The nodal derivative rule `FromValues` applies, exposed because a consumer
// building its own evaluator on the same nodes needs the same slopes and must
// not re-derive them from a paraphrase of this comment.
//
// `slopes_out` must have exactly the length of `knots`. Throws
// std::invalid_argument under the same conditions as the class factories.
void ComputeShapePreservingNodalSlopes(std::span<const double> knots,
                                       std::span<const double> values,
                                       std::span<double> slopes_out);

// The same rule, over memory the caller owns.
//
// The rule needs one spacing and one secant per interval. Where the nodes move
// every step — a curve rebuilt each time a solver is called — allocating those
// two arrays per call is the difference between a solver that allocates nothing
// and one that allocates twice per wheel per derivative evaluation. Both spans
// must hold at least `knots.size() - 1` entries; their contents on entry are
// ignored and on exit are unspecified.
//
// Throws under the same conditions as the allocating form, and additionally
// when either scratch span is too short.
void ComputeShapePreservingNodalSlopes(std::span<const double> knots,
                                       std::span<const double> values,
                                       std::span<double> slopes_out,
                                       std::span<double> spacing_scratch,
                                       std::span<double> secant_scratch);

}  // namespace orvd::wheel_rail_contact
