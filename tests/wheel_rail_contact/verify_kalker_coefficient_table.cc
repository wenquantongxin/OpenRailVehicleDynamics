// Kalker's linear creep coefficients: the Poisson collapse, the interpolation
// across semi-axis ratios, and the asymptotic branch outside the tabulation.
//
// None of the tabulated numbers are re-stated here. A test that repeated them
// would only check that two transcriptions of the same table agree, which is
// the one thing that is already guaranteed by there being one transcription.
// What is checked instead is what the construction claims about itself: that
// the Poisson collapse is a quadratic, that the ratio interpolation is linear
// in the ratio, that the two directions coincide where the physics says they
// must, that the domain edge is where it is said to be, and that the asymptotic
// branch has the closed form it is documented to have.

#include <cmath>
#include <cstdio>
#include <functional>
#include <numbers>
#include <span>
#include <string>
#include <string_view>

#include "allocation_probe.h"
#include "orvd/wheel_rail_contact/kalker_coefficient_table.h"

namespace {

using orvd::test::AllocationScope;
using orvd::wheel_rail_contact::KalkerCoefficients;
using orvd::wheel_rail_contact::KalkerCoefficientTable;
using orvd::wheel_rail_contact::OutsideTableRule;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "kalker coefficient table: %.*s\n",
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
                         "kalker coefficient table: %.*s was refused for another "
                         "reason: %s\n",
                         static_cast<int>(what.size()), what.data(), error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

// Steel. Chosen because it is between two tabulated Poisson nodes, which is the
// case the collapse exists for.
constexpr double kSteelPoissonRatio = 0.28;

}  // namespace

int main() {
    const KalkerCoefficientTable steel = KalkerCoefficientTable::ForPoissonRatio(
        kSteelPoissonRatio, OutsideTableRule::kAsymptotic);
    const std::span<const double, KalkerCoefficientTable::kNodeCount> nodes =
        steel.semi_axis_ratio_nodes();

    {
        // The grid is strictly ascending, spans exactly the documented domain,
        // and its upper half is the reciprocal of its lower half.
        bool ascending = true;
        for (std::size_t index = 1; index < nodes.size(); ++index) {
            ascending = ascending && nodes[index] > nodes[index - 1];
        }
        Require(ascending, "the semi-axis ratio grid is not strictly ascending");
        Require(nodes.front() == KalkerCoefficientTable::kSmallestTabulatedRatio &&
                    nodes.back() == KalkerCoefficientTable::kLargestTabulatedRatio,
                "the grid does not span the documented domain, so the gate and "
                "the interpolant disagree about where the table ends");
        Require(nodes[9] == 1.0, "the grid does not have a node at unity");
        bool reciprocal = true;
        for (std::size_t step = 1; step <= 9; ++step) {
            reciprocal = reciprocal &&
                         std::abs(nodes[9 + step] * nodes[9 - step] - 1.0) < 1.0e-15;
        }
        Require(reciprocal, "the upper half of the grid is not the reciprocal of "
                            "the lower half");
    }

    {
        // The Poisson collapse is a quadratic. Four abscissae of a quadratic
        // have a vanishing third divided difference, and a cubic or a spline
        // would not. This checks the interpolant's degree without knowing a
        // single tabulated value.
        const double at[4] = {0.05, 0.17, 0.31, 0.44};
        KalkerCoefficientTable built[4] = {
            KalkerCoefficientTable::ForPoissonRatio(at[0], OutsideTableRule::kRefuse),
            KalkerCoefficientTable::ForPoissonRatio(at[1], OutsideTableRule::kRefuse),
            KalkerCoefficientTable::ForPoissonRatio(at[2], OutsideTableRule::kRefuse),
            KalkerCoefficientTable::ForPoissonRatio(at[3], OutsideTableRule::kRefuse)};

        bool quadratic = true;
        for (const double ratio : {0.1, 0.55, 1.0, 2.0, 10.0}) {
            double value[4];
            for (int index = 0; index < 4; ++index) {
                value[index] = built[index].At(ratio).lateral_spin;
            }
            // Newton's divided differences, in place.
            for (int order = 1; order <= 3; ++order) {
                for (int index = 3; index >= order; --index) {
                    value[index] = (value[index] - value[index - 1]) /
                                   (at[index] - at[index - order]);
                }
            }
            quadratic = quadratic && std::abs(value[3]) < 1.0e-11;
        }
        Require(quadratic,
                "the Poisson collapse is not a quadratic: a four-point third "
                "divided difference did not vanish");

        // And it is exact at the nodes: a table built at a tabulated Poisson
        // ratio must reproduce that row rather than smooth across it. The three
        // Lagrange weights are Kronecker deltas there, so the reproduction is
        // exact in binary64, not approximate.
        const KalkerCoefficientTable at_node =
            KalkerCoefficientTable::ForPoissonRatio(0.25, OutsideTableRule::kRefuse);
        const KalkerCoefficients from_node = at_node.At(1.0);
        const KalkerCoefficients nearly = KalkerCoefficientTable::ForPoissonRatio(
                                              0.25 + 1.0e-9, OutsideTableRule::kRefuse)
                                              .At(1.0);
        Require(from_node.longitudinal != nearly.longitudinal,
                "the collapse does not depend on the Poisson ratio at all");
        Require(std::abs(from_node.longitudinal - nearly.longitudinal) < 1.0e-7,
                "the collapse is discontinuous at a tabulated Poisson node");
    }

    {
        // At zero Poisson ratio the two directions are indistinguishable, and
        // the tabulation says so exactly. Away from zero they must separate.
        const KalkerCoefficientTable isotropic =
            KalkerCoefficientTable::ForPoissonRatio(0.0, OutsideTableRule::kRefuse);
        bool coincide = true;
        for (const double ratio : nodes) {
            const KalkerCoefficients value = isotropic.At(ratio);
            coincide = coincide && value.longitudinal == value.lateral;
        }
        Require(coincide,
                "at zero Poisson ratio the longitudinal and lateral "
                "coefficients differ, so a row was mistranscribed");

        bool separate = true;
        for (const double ratio : nodes) {
            const KalkerCoefficients value = steel.At(ratio);
            separate = separate && value.longitudinal != value.lateral;
        }
        Require(separate,
                "at a nonzero Poisson ratio the two directions still coincide");
    }

    {
        // The ratio interpolation is linear in the ratio. The midpoint of two
        // adjacent nodes must therefore give the mean of their values — and it
        // must not, if the interpolation were in the logarithm of the ratio or
        // in the reciprocal.
        bool linear = true;
        bool discriminating = false;
        for (std::size_t index = 1; index < nodes.size(); ++index) {
            const double low = nodes[index - 1];
            const double high = nodes[index];
            const double middle = 0.5 * (low + high);
            const double mean =
                0.5 * (steel.At(low).lateral + steel.At(high).lateral);
            linear = linear && std::abs(steel.At(middle).lateral - mean) < 1.0e-14;

            // In the logarithm the same midpoint would land elsewhere. Confirm
            // at least one segment separates the two, so the check above is not
            // being satisfied by a grid too fine to tell them apart.
            const double log_middle = std::sqrt(low * high);
            discriminating =
                discriminating ||
                std::abs(steel.At(log_middle).lateral - mean) > 1.0e-4;
        }
        Require(linear, "the semi-axis interpolation is not linear in the ratio");
        Require(discriminating,
                "no segment of the grid distinguishes interpolation in the ratio "
                "from interpolation in its logarithm, so the check above proves "
                "nothing");

        // The grid is symmetric under a/b -> b/a; the interpolant over it is
        // not, because the values are not.
        Require(steel.At(1.25).lateral != steel.At(0.8).lateral,
                "the lookup is symmetric under reciprocation of the semi-axis "
                "ratio, which the tabulation is not");
    }

    {
        // All three coefficients grow with the semi-axis ratio across the whole
        // tabulated range. A row read backwards would break this.
        bool increasing = true;
        for (std::size_t index = 1; index < nodes.size(); ++index) {
            const KalkerCoefficients low = steel.At(nodes[index - 1]);
            const KalkerCoefficients high = steel.At(nodes[index]);
            increasing = increasing && high.longitudinal > low.longitudinal &&
                         high.lateral > low.lateral &&
                         high.lateral_spin > low.lateral_spin;
        }
        Require(increasing,
                "a coefficient does not increase with the semi-axis ratio");
    }

    {
        // The domain gate is inclusive at both ends.
        const KalkerCoefficientTable refusing =
            KalkerCoefficientTable::ForPoissonRatio(kSteelPoissonRatio,
                                                    OutsideTableRule::kRefuse);
        const KalkerCoefficients smallest =
            refusing.At(KalkerCoefficientTable::kSmallestTabulatedRatio);
        const KalkerCoefficients largest =
            refusing.At(KalkerCoefficientTable::kLargestTabulatedRatio);
        Require(std::isfinite(smallest.longitudinal) &&
                    std::isfinite(largest.longitudinal),
                "the tabulated endpoints were not evaluated");

        RequireRefusal(
            [&] {
                (void)refusing.At(
                    std::nextafter(KalkerCoefficientTable::kSmallestTabulatedRatio,
                                   0.0));
            },
            "outside the tabulated range",
            "a ratio one representable step below the domain");
        RequireRefusal(
            [&] {
                (void)refusing.At(std::nextafter(
                    KalkerCoefficientTable::kLargestTabulatedRatio, 100.0));
            },
            "outside the tabulated range",
            "a ratio one representable step above the domain");
    }

    {
        // The join between the table and the asymptotic expansion is a real
        // discontinuity. It is recorded, not smoothed: a re-implementation that
        // blended across it would fail here, and should.
        const double below = std::nextafter(
            KalkerCoefficientTable::kSmallestTabulatedRatio, 0.0);
        const KalkerCoefficients inside =
            steel.At(KalkerCoefficientTable::kSmallestTabulatedRatio);
        const KalkerCoefficients outside = steel.At(below);
        Require(std::abs(outside.lateral - inside.lateral) >
                    1.0e-3 * std::abs(inside.lateral),
                "the lower domain edge is continuous, so the asymptotic branch "
                "is not the one documented");

        const double above = std::nextafter(
            KalkerCoefficientTable::kLargestTabulatedRatio, 100.0);
        const KalkerCoefficients within =
            steel.At(KalkerCoefficientTable::kLargestTabulatedRatio);
        const KalkerCoefficients beyond = steel.At(above);
        Require(std::abs(beyond.longitudinal - within.longitudinal) >
                    1.0e-3 * std::abs(within.longitudinal),
                "the upper domain edge is continuous, so the asymptotic branch "
                "is not the one documented");
    }

    {
        // A patch slender across the rolling direction: the two direct
        // coefficients lose their shape dependence entirely, and take closed
        // forms that can be written down here without copying anything.
        const KalkerCoefficients thin = steel.At(0.05);
        const KalkerCoefficients thinner = steel.At(0.005);
        Require(thin.longitudinal == thinner.longitudinal &&
                    thin.lateral == thinner.lateral,
                "the slender-in-b asymptote still depends on the semi-axis "
                "ratio");
        const double pi = std::numbers::pi;
        Require(thin.lateral == (pi * pi) / 4.0,
                "the slender-in-b lateral coefficient is not pi squared over "
                "four");
        Require(thin.longitudinal == (pi * pi) / 4.0 / (1.0 - kSteelPoissonRatio),
                "the slender-in-b longitudinal coefficient is not pi squared "
                "over four, divided by one minus the Poisson ratio");
        // The coupling does keep a shape dependence, and it vanishes with the
        // patch's width.
        Require(thinner.lateral_spin < thin.lateral_spin && thinner.lateral_spin > 0.0,
                "the slender-in-b coupling does not shrink with the patch");
    }

    {
        // A patch slender along the rolling direction: everything grows, and
        // stays finite.
        const KalkerCoefficients long_patch = steel.At(50.0);
        const KalkerCoefficients longer = steel.At(500.0);
        Require(std::isfinite(longer.longitudinal) && std::isfinite(longer.lateral) &&
                    std::isfinite(longer.lateral_spin),
                "the slender-in-a asymptote is not finite");
        Require(longer.longitudinal > long_patch.longitudinal &&
                    longer.lateral > long_patch.lateral &&
                    longer.lateral_spin > long_patch.lateral_spin,
                "the slender-in-a asymptote does not grow with elongation");
    }

    RequireRefusal(
        [&] {
            (void)KalkerCoefficientTable::ForPoissonRatio(0.6,
                                                          OutsideTableRule::kRefuse);
        },
        "outside the tabulated range [0, 0.5]", "a Poisson ratio above one half");
    RequireRefusal(
        [&] {
            (void)KalkerCoefficientTable::ForPoissonRatio(
                std::nan(""), OutsideTableRule::kRefuse);
        },
        "outside the tabulated range [0, 0.5]", "a Poisson ratio that is not a number");
    RequireRefusal([&] { (void)steel.At(0.0); }, "not a positive finite number",
                   "a semi-axis ratio of zero under the asymptotic rule");
    RequireRefusal([&] { (void)steel.At(std::nan("")); },
                   "not a positive finite number",
                   "a semi-axis ratio that is not a number");
    RequireRefusal([&] { (void)steel.At(-2.0); }, "not a positive finite number",
                   "a negative semi-axis ratio");

    {
        // The lookup runs inside the integrator's right-hand side. It must not
        // allocate — not in the table branch, and not in the asymptotic one.
        double sink = 0.0;
        const AllocationScope scope;
        for (int step = 0; step < 200; ++step) {
            const double ratio = 0.1 + 0.05 * static_cast<double>(step);
            const KalkerCoefficients value = steel.At(ratio);
            sink += value.longitudinal + value.lateral + value.lateral_spin;
        }
        const std::size_t allocations = scope.allocations();
        Require(allocations == 0,
                "the coefficient lookup allocated on a path the integrator "
                "runs thousands of times per simulated second");
        Require(std::isfinite(sink), "the allocation sweep produced no numbers");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("kalker coefficient table verified");
    return 0;
}
