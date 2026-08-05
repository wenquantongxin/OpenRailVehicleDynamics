// The tangential contact solution: the friction law, the strip layout, the
// march, and the bound the resultant respects.
//
// The load-bearing check here is the small-creepage limit. When the creepages
// are small enough that the whole patch adheres, the march's answer has a
// closed form — the classical linear creep force, times a factor that is
// exactly the error of a midpoint rule integrating the patch's chord across the
// strips. Both are written out here from the geometry, so the check covers the
// flexibilities, the strip layout, the cell weights, the march's half first
// step and the sign convention all at once.
//
// There is no spin moment in the result and none is checked for. The direct
// moment a patch's creep would exert about its own normal is suppressed in this
// formulation — not computed and discarded, but absent — and the way to assert
// that is for the type to have nowhere to put one.

#include <cmath>
#include <cstdio>
#include <functional>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

#include "allocation_probe.h"
#include "orvd/wheel_rail_contact/tangential_contact_force.h"

namespace {

using orvd::test::AllocationScope;
using orvd::wheel_rail_contact::Creepages;
using orvd::wheel_rail_contact::FrictionCoefficientAt;
using orvd::wheel_rail_contact::FrictionLaw;
using orvd::wheel_rail_contact::KalkerCoefficients;
using orvd::wheel_rail_contact::KalkerCoefficientTable;
using orvd::wheel_rail_contact::OutsideTableRule;
using orvd::wheel_rail_contact::TangentialContactConfiguration;
using orvd::wheel_rail_contact::TangentialContactPatch;
using orvd::wheel_rail_contact::TangentialContactResult;
using orvd::wheel_rail_contact::TangentialContactSolver;
using orvd::wheel_rail_contact::TangentialContactWorkspace;

int failures = 0;

void Require(bool condition, std::string_view what) {
    if (!condition) {
        std::fprintf(stderr, "tangential contact force: %.*s\n",
                     static_cast<int>(what.size()), what.data());
        ++failures;
    }
}

void RequireClose(double actual, double expected, double relative_tolerance,
                  std::string_view what) {
    const double scale = std::max(std::abs(expected), 1.0e-30);
    if (!(std::abs(actual - expected) <= relative_tolerance * scale)) {
        std::fprintf(stderr,
                     "tangential contact force: %.*s (expected %.17g, got %.17g, "
                     "relative %.3g)\n",
                     static_cast<int>(what.size()), what.data(), expected, actual,
                     std::abs(actual - expected) / scale);
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
                         "tangential contact force: %.*s was refused for another "
                         "reason: %s\n",
                         static_cast<int>(what.size()), what.data(), error.what());
            ++failures;
        }
        return;
    }
    Require(false, what);
}

constexpr std::size_t kLongitudinalCells = 21;
constexpr std::size_t kLateralStrips = 21;
constexpr double kNormalLoad = 68000.0;
constexpr double kSemiLongitudinal = 0.00675;
constexpr double kSemiLateral = 0.006;
constexpr double kFriction = 0.3;

TangentialContactConfiguration Configuration() {
    TangentialContactConfiguration configuration;
    configuration.longitudinal_cells = kLongitudinalCells;
    configuration.lateral_strips = kLateralStrips;
    configuration.refinement_width = 0.01;
    configuration.material.youngs_modulus_pascals = 210.0e9;
    configuration.material.poisson_ratio = 0.28;
    return configuration;
}

TangentialContactPatch Patch() {
    TangentialContactPatch patch;
    patch.normal_force_newtons = kNormalLoad;
    patch.longitudinal_semi_axis_meters = kSemiLongitudinal;
    patch.lateral_semi_axis_meters = kSemiLateral;
    patch.friction_coefficient = kFriction;
    return patch;
}

Creepages Slip(double longitudinal, double lateral, double spin) {
    Creepages creepages;
    creepages.longitudinal = longitudinal;
    creepages.lateral = lateral;
    creepages.spin_per_meter = spin;
    creepages.reference_speed_meters_per_second = 16.666666666666668;
    return creepages;
}

// The strip layout, laid down again here from the description in the solver's
// own header rather than read out of it.
//
// This exists for one reason: the spin's contribution to the *longitudinal*
// stress rate is proportional to a strip's distance from the patch's centreline,
// so over any layout that is symmetric about that line its resultant cancels
// exactly — and every layout is symmetric unless the spin pole is off centre.
// Removing that whole term therefore changes no force this suite would otherwise
// look at. Placing the pole off centre makes the layout asymmetric, and then the
// term survives into the resultant, where it can be checked against a closed
// form. Reproducing the layout is the price of that check; it also pins the
// layout itself, which is otherwise only seen through an integral.
struct Strip {
    double centre{0.0};
    double width{0.0};
};

std::vector<Strip> LayStrips(std::size_t strips, double refinement_width,
                             double pole_longitudinal, double pole_lateral) {
    std::vector<Strip> laid;
    const double divisions = static_cast<double>(strips);
    if (!(pole_longitudinal * pole_longitudinal + pole_lateral * pole_lateral <
          1.0)) {
        const double width = 2.0 / divisions;
        for (std::size_t index = 0; index < strips; ++index) {
            laid.push_back({-1.0 + (static_cast<double>(index) + 0.5) * width, width});
        }
        return laid;
    }
    for (const double side : {1.0, -1.0}) {
        const double pole = pole_lateral * side;
        const double span = 1.0 - pole;
        const std::size_t count = static_cast<std::size_t>(
            std::max(1.0, std::floor(span * divisions / 2.0) + 1.0));
        double width = span / static_cast<double>(count);
        const double refinement_start = pole + width;
        double centre = 1.0 - 0.5 * width;
        while (true) {
            bool terminal = false;
            if (centre < refinement_start) {
                if (refinement_width < 0.5 * width) {
                    width *= 0.5;
                    centre += 0.5 * width;
                } else {
                    terminal = true;
                }
            }
            laid.push_back({centre * side, width});
            if (terminal) {
                break;
            }
            centre -= width;
        }
    }
    return laid;
}

}  // namespace

int main() {
    {
        // The friction law: highest at rest, falling towards its floor as the
        // surfaces slide faster, and never below it.
        const FrictionLaw law;
        RequireClose(FrictionCoefficientAt(law, Slip(0.0, 0.0, 0.0)),
                     law.static_coefficient, 1.0e-15,
                     "friction at zero creepage is not the static coefficient");
        const double gentle = FrictionCoefficientAt(law, Slip(0.001, 0.0, 0.0));
        const double harsh = FrictionCoefficientAt(law, Slip(0.05, 0.0, 0.0));
        Require(gentle < law.static_coefficient && harsh < gentle,
                "friction does not fall with sliding speed");
        Require(harsh > law.static_coefficient * law.limiting_fraction,
                "friction fell below its own floor");
        RequireClose(FrictionCoefficientAt(law, Slip(0.0, 0.0, 50.0)),
                     law.static_coefficient, 1.0e-15,
                     "the spin creepage reached the friction law, which it must "
                     "not");
        // The lateral and longitudinal creepages enter only through their
        // magnitude, so rotating the slip vector must not change the answer.
        RequireClose(FrictionCoefficientAt(law, Slip(0.03, 0.04, 0.0)),
                     FrictionCoefficientAt(law, Slip(0.05, 0.0, 0.0)), 1.0e-15,
                     "the friction law is not isotropic in the two translational "
                     "creepages");
    }

    const KalkerCoefficientTable table = KalkerCoefficientTable::ForPoissonRatio(
        0.28, OutsideTableRule::kAsymptotic);
    const TangentialContactSolver solver{Configuration(), table};
    TangentialContactWorkspace workspace;
    const KalkerCoefficients kalker =
        table.At(kSemiLongitudinal / kSemiLateral);
    const double shear = solver.shear_modulus_pascals();
    // The midpoint rule's error integrating the patch's chord across uniformly
    // laid strips. It is a property of the layout, not a fudge: with `n` strips
    // over a span of two, the rule overshoots by exactly this fraction.
    const double strip_width = 2.0 / static_cast<double>(kLateralStrips);
    const double quadrature_factor = 1.0 + strip_width * strip_width / 8.0;

    {
        // Nothing creeping, nothing transmitted. Exactly zero, not nearly.
        const TangentialContactResult result =
            solver.Solve(Patch(), Slip(0.0, 0.0, 0.0), workspace);
        Require(result.longitudinal_force_newtons == 0.0 &&
                    result.lateral_force_newtons == 0.0,
                "a patch with no creepage transmitted a tangential force");
        Require(result.strip_count == kLateralStrips,
                "no spin did not give the uniform strip layout");
        Require(result.cell_count == kLateralStrips * kLongitudinalCells,
                "the cell count is not the product of the two resolutions");
        Require(result.friction_bound_newtons > 0.0,
                "a loaded patch reported no friction capacity");
    }

    {
        // The small-creepage limit, longitudinally. The whole patch adheres, so
        // the march integrates a stress that grows linearly with distance from
        // the leading edge, and the answer is the classical linear creep force.
        const double creepage = 1.0e-7;
        const TangentialContactResult result =
            solver.Solve(Patch(), Slip(creepage, 0.0, 0.0), workspace);
        const double expected = -creepage * shear * kSemiLongitudinal *
                                kSemiLateral * kalker.longitudinal *
                                quadrature_factor;
        RequireClose(result.longitudinal_force_newtons, expected, 1.0e-12,
                     "the small-creepage longitudinal force is not the linear "
                     "creep force");
        Require(std::abs(result.lateral_force_newtons) <
                    1.0e-9 * std::abs(expected),
                "a purely longitudinal creepage produced a lateral force");

        // Small creepages are linear in the creepage, and the sign follows.
        const TangentialContactResult doubled =
            solver.Solve(Patch(), Slip(2.0 * creepage, 0.0, 0.0), workspace);
        RequireClose(doubled.longitudinal_force_newtons,
                     2.0 * result.longitudinal_force_newtons, 1.0e-12,
                     "the small-creepage force is not linear in the creepage");
        const TangentialContactResult reversed =
            solver.Solve(Patch(), Slip(-creepage, 0.0, 0.0), workspace);
        RequireClose(reversed.longitudinal_force_newtons,
                     -result.longitudinal_force_newtons, 1.0e-12,
                     "reversing the creepage did not reverse the force");
    }

    {
        // The same limit laterally, which reads a different tabulated
        // coefficient. A implementation that used one coefficient for both
        // directions would pass the check above and fail this one.
        const double creepage = 1.0e-7;
        const TangentialContactResult result =
            solver.Solve(Patch(), Slip(0.0, creepage, 0.0), workspace);
        const double expected = -creepage * shear * kSemiLongitudinal *
                                kSemiLateral * kalker.lateral * quadrature_factor;
        RequireClose(result.lateral_force_newtons, expected, 1.0e-12,
                     "the small-creepage lateral force is not the linear creep "
                     "force");
        Require(kalker.lateral != kalker.longitudinal,
                "the two tabulated coefficients coincide at this aspect ratio, "
                "so this fixture cannot tell them apart");
        Require(std::abs(result.longitudinal_force_newtons) <
                    1.0e-9 * std::abs(expected),
                "a purely lateral creepage produced a longitudinal force");
    }

    {
        // Spin alone still produces a force. It has to: the rigid slip a spin
        // imposes does not vanish just because the patch is not translating
        // relative to the rail, and a solver that returned zero here would lose
        // a real part of the guidance force.
        const TangentialContactResult result =
            solver.Solve(Patch(), Slip(0.0, 0.0, -0.19), workspace);
        Require(std::abs(result.lateral_force_newtons) > 1.0,
                "a patch under pure spin transmitted no lateral force");
        Require(std::abs(result.longitudinal_force_newtons) <
                    1.0e-9 * std::abs(result.lateral_force_newtons),
                "pure spin produced a longitudinal force, which the layout's "
                "symmetry forbids");
        // With the pole inside the patch the strips refine, so there are more
        // of them than the uniform layout would give.
        Require(result.strip_count > kLateralStrips,
                "spin did not refine the strip layout");
    }

    {
        // The spin's longitudinal stress, made visible by an off-centre pole.
        //
        // Everything stays in full adhesion, where each strip's stress grows
        // linearly with distance from the leading edge and the march's midpoint
        // rule integrates that linear growth exactly. So the resultant is a
        // plain sum over the strips, written out below from the layout and the
        // flexibilities — and the spin's contribution to the longitudinal rate
        // is one of the two terms in it.
        const TangentialContactResult probe =
            solver.Solve(Patch(), Slip(0.0, 0.0, -1.0), workspace);
        const double pole = 0.5;
        for (const double scale : {1.0, 0.5}) {
            const double spin = -0.002 * scale;
            const double longitudinal = pole * spin *
                                        probe.longitudinal_flexibility *
                                        kSemiLateral / probe.spin_flexibility;
            const TangentialContactResult result =
                solver.Solve(Patch(), Slip(longitudinal, 0.0, spin), workspace);

            const std::vector<Strip> laid =
                LayStrips(kLateralStrips, 0.01, 0.0, pole);
            Require(laid.size() == result.strip_count,
                    "the strip layout laid here does not match the solver's");
            double total_width = 0.0;
            double predicted = 0.0;
            for (const Strip& strip : laid) {
                total_width += strip.width;
                const double rate =
                    longitudinal / result.longitudinal_flexibility -
                    spin * (kSemiLateral * strip.centre) / result.spin_flexibility;
                predicted -= rate * kSemiLongitudinal * kSemiLongitudinal *
                             kSemiLateral * strip.width * 2.0 *
                             (1.0 - strip.centre * strip.centre);
            }
            RequireClose(total_width, 2.0, 1.0e-15,
                         "the strips do not tile the patch");
            RequireClose(result.longitudinal_force_newtons, predicted, 1.0e-12,
                         "the longitudinal force over an asymmetric layout is not "
                         "the sum of the two stress rates over the strips");

            // Confirm the fixture would notice the spin term going missing: the
            // two contributions must be comparable, not one dwarfing the other.
            double without_spin = 0.0;
            for (const Strip& strip : laid) {
                without_spin -= (longitudinal / result.longitudinal_flexibility) *
                                kSemiLongitudinal * kSemiLongitudinal * kSemiLateral *
                                strip.width * 2.0 * (1.0 - strip.centre * strip.centre);
            }
            // The spin's share of this force is small in absolute terms —
            // parts in a thousand — and that is not an accident: over a layout
            // tiling the whole patch it is a quadrature residue of an integral
            // that vanishes exactly, so a finer layout makes it smaller still.
            // What matters is that it is enormously larger than the tolerance
            // the comparison above is made at, which is what makes that
            // comparison able to see it go missing.
            Require(std::abs(predicted - without_spin) >
                        1.0e-6 * std::abs(predicted),
                    "the spin's longitudinal stress is below the scale this "
                    "fixture can resolve, so the check above would not notice it "
                    "going missing");
        }
    }

    {
        // The bound the resultant actually respects. Every cell is limited by
        // its own friction capacity, so the resultant cannot exceed the sum of
        // those capacities — and at full saturation it reaches it.
        for (const Creepages creepages :
             {Slip(0.5, 0.0, 0.0), Slip(0.0, 0.5, 0.0), Slip(0.3, 0.4, 0.0),
              Slip(0.2, 0.2, -5.0)}) {
            const TangentialContactResult result =
                solver.Solve(Patch(), creepages, workspace);
            const double magnitude = std::hypot(result.longitudinal_force_newtons,
                                                result.lateral_force_newtons);
            Require(magnitude <= result.friction_bound_newtons * (1.0 + 1.0e-14),
                    "the tangential resultant exceeded the sum of the per-cell "
                    "friction capacities");
        }

        // Saturated, and in one direction, the resultant is the whole capacity.
        const TangentialContactResult saturated =
            solver.Solve(Patch(), Slip(1.0, 0.0, 0.0), workspace);
        RequireClose(std::abs(saturated.longitudinal_force_newtons),
                     saturated.friction_bound_newtons, 1.0e-12,
                     "a fully saturated patch did not reach its own friction "
                     "capacity");

        // That capacity is slightly more than the continuum Coulomb product,
        // because the pressure is integrated by a midpoint rule. It is recorded
        // rather than normalised away: normalising would move every force in
        // the model, not only the saturated ones.
        const double coulomb = kFriction * kNormalLoad;
        Require(saturated.friction_bound_newtons > coulomb,
                "the discrete friction capacity does not exceed the continuum "
                "product, so the pressure is not being integrated as documented");
        Require(saturated.friction_bound_newtons < coulomb * 1.01,
                "the discrete friction capacity is more than one percent above "
                "the continuum product");
    }

    {
        // The strip layout tiles the patch under every branch it can take, and
        // the friction capacity is the check: a gap or an overlap would change
        // the integrated pressure, and the capacity is that integral.
        const TangentialContactResult uniform =
            solver.Solve(Patch(), Slip(1.0, 0.0, 0.0), workspace);

        // Placing the spin pole takes the flexibilities, so they are read off a
        // probe solve rather than guessed. With no lateral creepage the pole
        // sits on the patch's own lateral axis, and its position is the
        // longitudinal creepage over the spin, scaled by the two flexibilities
        // and the patch's half width.
        const TangentialContactResult probe =
            solver.Solve(Patch(), Slip(0.001, 0.0, -1.0), workspace);
        const double spin = -1.0;
        const auto creepage_placing_pole_at = [&](double pole) {
            return pole * spin * probe.longitudinal_flexibility * kSemiLateral /
                   probe.spin_flexibility;
        };

        // A pole halfway out: the march subdivides towards it from both sides.
        const TangentialContactResult refined = solver.Solve(
            Patch(), Slip(creepage_placing_pole_at(0.5), 0.0, spin), workspace);
        // A pole almost exactly on the patch's edge: too little room to
        // subdivide, so the layout falls back to a plain one at the refinement
        // width.
        const TangentialContactResult fallback = solver.Solve(
            Patch(), Slip(creepage_placing_pole_at(0.995), 0.0, spin), workspace);
        RequireClose(refined.friction_bound_newtons, uniform.friction_bound_newtons,
                     2.0e-3,
                     "the refined strip layout does not integrate the same "
                     "pressure as the uniform one");
        RequireClose(fallback.friction_bound_newtons, uniform.friction_bound_newtons,
                     2.0e-3,
                     "the fallback strip layout does not integrate the same "
                     "pressure as the uniform one");
        Require(refined.strip_count != uniform.strip_count &&
                    fallback.strip_count != uniform.strip_count &&
                    fallback.strip_count != refined.strip_count,
                "the three strip layouts did not produce three different strip "
                "counts, so this fixture does not reach all three");
        Require(fallback.strip_count > 90,
                "the fallback layout did not lay strips at the refinement width");
    }

    {
        // Nothing to transmit through.
        const auto empty = [&](TangentialContactPatch patch, std::string_view what) {
            const TangentialContactResult result =
                solver.Solve(patch, Slip(0.1, 0.1, 0.0), workspace);
            Require(result.longitudinal_force_newtons == 0.0 &&
                        result.lateral_force_newtons == 0.0 &&
                        result.friction_bound_newtons == 0.0 &&
                        result.cell_count == 0,
                    what);
        };
        TangentialContactPatch unloaded = Patch();
        unloaded.normal_force_newtons = 0.0;
        empty(unloaded, "an unloaded patch transmitted a tangential force");
        TangentialContactPatch flat = Patch();
        flat.longitudinal_semi_axis_meters = 0.0;
        empty(flat, "a patch with no length transmitted a tangential force");
        TangentialContactPatch narrow = Patch();
        narrow.lateral_semi_axis_meters = 0.0;
        empty(narrow, "a patch with no width transmitted a tangential force");
        TangentialContactPatch slippery = Patch();
        slippery.friction_coefficient = 0.0;
        empty(slippery, "a frictionless patch transmitted a tangential force");
    }

    RequireRefusal(
        [&] {
            TangentialContactConfiguration configuration = Configuration();
            configuration.longitudinal_cells = 2;
            (void)TangentialContactSolver(configuration, table);
        },
        "at least three divisions", "a march with no interior");
    RequireRefusal(
        [&] {
            TangentialContactConfiguration configuration = Configuration();
            configuration.refinement_width = 0.0;
            (void)TangentialContactSolver(configuration, table);
        },
        "refinement width", "a refinement that never stops");
    RequireRefusal(
        [&] {
            TangentialContactConfiguration configuration = Configuration();
            configuration.material.poisson_ratio = 0.5;
            (void)TangentialContactSolver(configuration, table);
        },
        "Poisson ratio", "an incompressible material");

    {
        // The solve runs once per patch per derivative evaluation, under every
        // strip layout.
        double sink = 0.0;
        const Creepages sweep[3] = {Slip(0.001, 0.0, 0.0), Slip(0.01, 0.02, -2.0),
                                    Slip(0.9999, 0.0, -1.0e-4)};
        for (const Creepages& creepages : sweep) {
            (void)solver.Solve(Patch(), creepages, workspace);
        }
        const AllocationScope scope;
        for (int step = 0; step < 60; ++step) {
            const TangentialContactResult result =
                solver.Solve(Patch(), sweep[step % 3], workspace);
            sink += result.longitudinal_force_newtons + result.lateral_force_newtons;
        }
        Require(scope.allocations() == 0,
                "the tangential solve allocated after its workspace was warm");
        Require(std::isfinite(sink), "the allocation sweep produced no force");
    }

    if (failures != 0) {
        return 1;
    }
    std::puts("tangential contact force verified");
    return 0;
}
