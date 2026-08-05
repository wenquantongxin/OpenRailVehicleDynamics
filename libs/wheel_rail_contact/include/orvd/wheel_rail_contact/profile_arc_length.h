#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "orvd/wheel_rail_contact/natural_cubic_spline.h"

// Arc length along a profile curve, and the inverse question.
//
// The curve is the graph of a spline over its lateral coordinate, so its arc
// length between two abscissae is the integral of sqrt(1 + z'^2). The integrand
// is smooth inside a spline segment and only continuous in its third derivative
// across a knot, which decides the whole design here: the rule is applied once
// per knot interval and never across one. A single rule spread over the whole
// profile would lose most of its accuracy at every knot, and — more to the
// point — the cumulative table and the station search must integrate the same
// way or the stations they agree on drift apart.
//
// The station search is deliberately a fixed number of halvings rather than a
// tolerance loop. A tolerance loop on a monotone function of a bracket this
// small ends at the same place, and ends there after an input-dependent number
// of steps; the fixed count makes the result a function of the input alone.

namespace orvd::wheel_rail_contact {

// Integrates sqrt(1 + z'^2) over [from, to] with a sixteen-point Gauss-Legendre
// rule. Returns zero when the interval is empty or reversed. The caller is
// responsible for not spanning a knot: this function does not know where the
// knots are.
[[nodiscard]] double IntegrateProfileArcLength(const NaturalCubicSpline& profile,
                                               double from_lateral_meters,
                                               double to_lateral_meters);

// The cumulative arc length at each knot, starting at zero, integrated one knot
// interval at a time.
//
// Throws std::invalid_argument when fewer than two knots are given or when the
// knots are not strictly increasing.
[[nodiscard]] std::vector<double> AccumulateProfileArcLength(
    const NaturalCubicSpline& profile, std::span<const double> knots);

// The lateral coordinate at which the arc length measured from the first knot
// reaches `target_arc_length_meters`.
//
// `cumulative` must be the table `AccumulateProfileArcLength` produced for the
// same spline and knots. The search brackets the answer inside one knot
// interval and halves that bracket a fixed number of times, which reaches the
// last representable digit for any interval a profile asset contains.
//
// Throws std::invalid_argument when the table does not match the knots, or when
// the target lies outside the table's range.
[[nodiscard]] double FindLateralCoordinateAtArcLength(
    const NaturalCubicSpline& profile, std::span<const double> knots,
    std::span<const double> cumulative, double target_arc_length_meters);

}  // namespace orvd::wheel_rail_contact
