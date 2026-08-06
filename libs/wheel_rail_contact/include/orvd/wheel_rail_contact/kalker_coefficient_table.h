#pragma once

#include <array>
#include <cstddef>
#include <span>

// Kalker's linear creep coefficients, as a function of the contact ellipse's
// shape and the material's Poisson ratio.
//
// These three numbers are what turns a creepage into a tangential stress at the
// leading edge of the contact patch. They come from Kalker's tabulation of the
// exact linear-elastic half-space solution: there is no closed form inside the
// table's domain, so the values are read off a grid and interpolated, and that
// grid is reproduced here rather than recomputed.
//
// ## Two axes, collapsed to one
//
// The tabulation is two-dimensional — semi-axis ratio against Poisson ratio —
// but a material's Poisson ratio does not change while a vehicle runs. So the
// Poisson axis is collapsed once, when the table is built, and what remains is
// a one-dimensional lookup that the contact solver can call once per patch per
// step without thinking about it.
//
// The collapse is a quadratic through the three tabulated Poisson nodes
// (0, 1/4, 1/2). Three nodes admit exactly one quadratic, so this is not a
// fitting choice; the choice is to interpolate rather than to round to the
// nearest node, and it matters: steel sits at 0.28, between two nodes.
//
// ## The domain edge is a real discontinuity
//
// Outside a semi-axis ratio of [0.1, 10] the table stops and an asymptotic
// slender-ellipse expansion takes over. The two do not agree at the join — the
// jump is a few percent in places. That is a property of the underlying
// tabulation, not an artefact of this implementation, and it is not smoothed
// over. A caller that wants to know it has left the tabulated region can ask
// for a refusal instead.
//
// The gate is on the ratio itself, at the literal bounds, not on the first and
// last grid node. They coincide here; stating which one is authoritative
// removes the question.

namespace orvd::wheel_rail_contact {

// The three coefficients: longitudinal, lateral, and the lateral/spin coupling.
//
// There is no fourth. Kalker's C33 (the pure spin coefficient) does not appear
// because the tangential solution this feeds resolves spin by marching across
// the patch rather than by a linear coefficient.
struct KalkerCoefficients {
    double longitudinal{0.0};
    double lateral{0.0};
    double lateral_spin{0.0};
};

// What to do when the semi-axis ratio leaves the tabulated domain.
enum class OutsideTableRule {
    // Refuse. Appropriate when a ratio that far from unity means the contact
    // geometry has gone wrong and continuing would hide it.
    kRefuse,
    // Continue with the slender-ellipse asymptotic expansion, discontinuously.
    kAsymptotic,
};

// The collapsed table, ready for lookup.
//
// Fixed-size throughout: constructing one allocates nothing and looking one up
// allocates nothing, which is what lets it live on the contact solver's hot
// path. Copying one is cheap enough to pass by value, but it is passed by
// reference everywhere for consistency with the rest of this layer.
class KalkerCoefficientTable {
  public:
    static constexpr std::size_t kNodeCount = 19;

    // Collapses the Poisson axis at the given ratio.
    //
    // Throws if the Poisson ratio is not in [0, 0.5]. The quadratic is exact at
    // the three nodes and interpolating within them is what it is for;
    // extrapolating outside the physical range of a Poisson ratio is not.
    [[nodiscard]] static KalkerCoefficientTable ForPoissonRatio(
        double poisson_ratio, OutsideTableRule outside_rule);

    // The coefficients at one semi-axis ratio a/b.
    //
    // Piecewise linear in the ratio itself — not in its logarithm, and not in
    // min(a/b, b/a). The grid above unity is the reciprocal of the grid below
    // it, so interpolating in the ratio and interpolating in its reciprocal are
    // different functions; this is the first.
    [[nodiscard]] KalkerCoefficients At(double semi_axis_ratio) const;

    [[nodiscard]] double poisson_ratio() const { return poisson_ratio_; }
    [[nodiscard]] OutsideTableRule outside_rule() const { return outside_rule_; }

    // The tabulated semi-axis ratios, ascending. Exposed because a caller that
    // wants to know where the interpolant's corners are should read them rather
    // than re-derive them.
    [[nodiscard]] std::span<const double, kNodeCount> semi_axis_ratio_nodes() const;

    // The inclusive bounds outside which `outside_rule` applies.
    static constexpr double kSmallestTabulatedRatio = 0.1;
    static constexpr double kLargestTabulatedRatio = 10.0;

  private:
    KalkerCoefficientTable() = default;

    std::array<double, kNodeCount> longitudinal_{};
    std::array<double, kNodeCount> lateral_{};
    std::array<double, kNodeCount> lateral_spin_{};
    double poisson_ratio_{0.0};
    OutsideTableRule outside_rule_{OutsideTableRule::kRefuse};
};

}  // namespace orvd::wheel_rail_contact
