#pragma once

#include <cstddef>
#include <span>
#include <utility>
#include <vector>

// The natural cubic spline the wheel and rail profile surfaces are built on.
//
// It owns its knots and values, is immutable once constructed, and evaluates
// without allocating. Two construction paths exist and both are reachable: a
// grid whose spacings are equal to within a stated tolerance solves for nodal
// slopes on a spacing-independent tridiagonal system, and any other grid solves
// the classical moment system and derives the slopes from it. The two agree
// analytically and differ in the last bits; which one ran is observable through
// `is_uniform()` because a caller comparing two profiles needs to know whether a
// difference came from the data or from the path.
//
// The uniform path is not an optimisation switch a caller may request. It is a
// property of the data, detected here, and the detection tolerance is part of
// the contract: a grid that passes it is evaluated on the idealised grid
// `x0 + i*h`, so a caller whose spacings differ by more than rounding must not
// expect its own knots back. The wheel profile shipped with a reference vehicle
// is uniform to a part in 1e12 and therefore takes that path; a profile
// resampled at equal arc length is not, and takes the other.
//
// Outside the knot range the value is held flat at the end value. That is a
// deliberate choice and not the usual polynomial continuation: a profile has no
// meaning beyond its last measured point, and a cubic continued past the flange
// root produces a surface that looks plausible and is not there. The first
// derivative is zero strictly outside, and at the boundary knots themselves it
// is the one-sided interior slope, so the derivative has a step at each end.
// The second derivative is exactly zero at and outside both boundary knots,
// which is the natural boundary condition made exact rather than approximated.

namespace orvd::wheel_rail_contact {

class NaturalCubicSpline {
   public:
    // Builds the spline over the given knots and values.
    //
    // Throws std::invalid_argument when the two spans differ in length, when
    // fewer than two points are given, when any value is not finite, or when
    // the knots are not strictly increasing. Throws std::runtime_error when the
    // resulting tridiagonal system is singular; the threshold is absolute, so a
    // grid whose spacings are far below the scale of its values can be refused
    // on the general path while an equally spaced one is not.
    NaturalCubicSpline(std::span<const double> knots,
                       std::span<const double> values);

    [[nodiscard]] double Evaluate(double abscissa) const;
    [[nodiscard]] double EvaluateFirstDerivative(double abscissa) const;
    [[nodiscard]] double EvaluateSecondDerivative(double abscissa) const;

    // The nodal first derivatives the construction solved for. A Hermite
    // interpolant built from these reproduces this spline's values and slopes
    // at every knot while giving its own behaviour in between, which is how a
    // profile representation is handed to a consumer that wants a different
    // interpolant on the same nodes.
    [[nodiscard]] std::span<const double> nodal_slopes() const & {
        return nodal_slopes_;
    }
    [[nodiscard]] std::vector<double> nodal_slopes() && {
        return std::move(nodal_slopes_);
    }
    [[nodiscard]] std::vector<double> nodal_slopes() const && {
        return nodal_slopes_;
    }

    // Which construction path ran. See the class comment: this is data, not a
    // caller preference.
    [[nodiscard]] bool is_uniform() const { return is_uniform_; }
    [[nodiscard]] std::size_t size() const { return values_.size(); }

   private:
    friend class ContactGeometrySolver;

    struct ValueAndFirstDerivative {
        double value{0.0};
        double first_derivative{0.0};
    };

    // One segment's cubic in the local parameter t = (s - x_i) / h_i, stored in
    // the scaled Hermite form so that evaluation never divides.
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

    [[nodiscard]] Location Locate(double abscissa) const;
    [[nodiscard]] ValueAndFirstDerivative
    EvaluateValueAndFirstDerivativeForFiniteAbscissa(double abscissa) const;

    // Empty on the uniform path: the idealised grid replaces it, and keeping a
    // copy would invite an evaluation that mixed the two.
    std::vector<double> knots_;
    std::vector<double> values_;
    std::vector<double> nodal_slopes_;
    std::vector<SegmentCoefficients> segments_;
    double first_knot_{0.0};
    double last_knot_{0.0};
    double uniform_spacing_{0.0};
    double inverse_uniform_spacing_{0.0};
    bool is_uniform_{false};
};

}  // namespace orvd::wheel_rail_contact
