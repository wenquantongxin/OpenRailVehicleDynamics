#include "orvd/wheel_rail_contact/profile_arc_length.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace orvd::wheel_rail_contact {
namespace {

// Sixteen-point Gauss-Legendre, stored as the eight positive abscissae and
// their weights; the rule is symmetric, so each pair is evaluated once on each
// side of the interval midpoint. The weights sum to two over the full set,
// which is the identity a reader can check against this table.
constexpr std::array<double, 8> kGaussLegendreAbscissae{
    0.095012509837637440185319335424958,
    0.281603550779258913230460501460496,
    0.458016777657227386342419442983577,
    0.617876244402643748446671764048791,
    0.755404408355003033895101194847442,
    0.865631202387831743880467897712393,
    0.944575023073232576077988415534608,
    0.989400934991649932596154173450333};

constexpr std::array<double, 8> kGaussLegendreWeights{
    0.189450610455068496285396723208283,
    0.182603415044923588866763667969219,
    0.169156519395002538189312079030359,
    0.149595988816576732081501730547479,
    0.124628971255533872052476282192017,
    0.095158511682492784809925107602246,
    0.062253523938647892862843836994378,
    0.027152459411754094851780572456018};

// The bracket is halved this many times. Sixty-four halvings take any interval
// a profile asset contains below the spacing of the doubles inside it, so the
// answer is the converged root rather than an approximation to it.
constexpr int kBracketHalvings = 64;

[[noreturn]] void Reject(const std::string& detail) {
    throw std::invalid_argument("profile arc length: " + detail);
}

}  // namespace

double IntegrateProfileArcLength(const NaturalCubicSpline& profile,
                                 double from_lateral_meters,
                                 double to_lateral_meters) {
    if (!(to_lateral_meters > from_lateral_meters)) {
        return 0.0;
    }
    const double midpoint = 0.5 * (from_lateral_meters + to_lateral_meters);
    const double half_width = 0.5 * (to_lateral_meters - from_lateral_meters);
    double sum = 0.0;
    for (std::size_t node = 0; node < kGaussLegendreAbscissae.size(); ++node) {
        const double offset = half_width * kGaussLegendreAbscissae[node];
        const double below =
            std::hypot(1.0, profile.EvaluateFirstDerivative(midpoint - offset));
        const double above =
            std::hypot(1.0, profile.EvaluateFirstDerivative(midpoint + offset));
        sum += kGaussLegendreWeights[node] * (below + above);
    }
    return half_width * sum;
}

std::vector<double> AccumulateProfileArcLength(const NaturalCubicSpline& profile,
                                               std::span<const double> knots) {
    if (knots.size() < 2) {
        Reject("at least two knots are required, but " +
               std::to_string(knots.size()) + " were given");
    }
    for (std::size_t node = 0; node + 1 < knots.size(); ++node) {
        if (!(knots[node + 1] > knots[node])) {
            Reject("the knots must be strictly increasing, but the one at index " +
                   std::to_string(node + 1) + " does not exceed its predecessor");
        }
    }

    std::vector<double> cumulative(knots.size(), 0.0);
    for (std::size_t node = 0; node + 1 < knots.size(); ++node) {
        cumulative[node + 1] =
            cumulative[node] +
            IntegrateProfileArcLength(profile, knots[node], knots[node + 1]);
    }
    return cumulative;
}

double FindLateralCoordinateAtArcLength(const NaturalCubicSpline& profile,
                                        std::span<const double> knots,
                                        std::span<const double> cumulative,
                                        double target_arc_length_meters) {
    if (cumulative.size() != knots.size()) {
        Reject("the cumulative table has " + std::to_string(cumulative.size()) +
               " entries and the knots have " + std::to_string(knots.size()) +
               "; they must describe the same curve");
    }
    if (knots.size() < 2) {
        Reject("at least two knots are required, but " +
               std::to_string(knots.size()) + " were given");
    }
    if (!std::isfinite(target_arc_length_meters) ||
        target_arc_length_meters < cumulative.front() ||
        target_arc_length_meters > cumulative.back()) {
        Reject("the target arc length " +
               std::to_string(target_arc_length_meters) +
               " m is outside the curve's range [" +
               std::to_string(cumulative.front()) + ", " +
               std::to_string(cumulative.back()) + "] m");
    }

    const auto upper = std::upper_bound(cumulative.begin(), cumulative.end(),
                                        target_arc_length_meters);
    const std::size_t interval =
        std::min(static_cast<std::size_t>(upper - cumulative.begin()) - 1,
                 knots.size() - 2);
    const double local_target = target_arc_length_meters - cumulative[interval];

    double lower = knots[interval];
    double upper_bound_lateral = knots[interval + 1];
    for (int halving = 0; halving < kBracketHalvings; ++halving) {
        const double midpoint = 0.5 * (lower + upper_bound_lateral);
        const double local_arc =
            IntegrateProfileArcLength(profile, knots[interval], midpoint);
        if (local_arc < local_target) {
            lower = midpoint;
        } else {
            upper_bound_lateral = midpoint;
        }
    }
    return 0.5 * (lower + upper_bound_lateral);
}

}  // namespace orvd::wheel_rail_contact
