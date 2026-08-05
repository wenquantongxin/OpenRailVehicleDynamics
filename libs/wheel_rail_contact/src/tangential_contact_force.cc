#include "orvd/wheel_rail_contact/tangential_contact_force.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace orvd::wheel_rail_contact {
namespace {

// A hard ceiling on how many times a strip may be halved. The refinement stops
// on its own at the configured width; this only bounds the loop if a
// pathological width ever made it not to.
constexpr int kMaximumHalvings = 64;

// The smallest resolution that has an interior to march through.
constexpr std::size_t kMinimumResolution = 3;

}  // namespace

double FrictionCoefficientAt(const FrictionLaw& law, const Creepages& creepages) {
    const double reference_speed =
        std::max(std::abs(creepages.reference_speed_meters_per_second),
                 law.minimum_reference_speed_meters_per_second);
    const double sliding_speed =
        reference_speed * std::hypot(creepages.longitudinal, creepages.lateral);
    return law.static_coefficient *
           ((1.0 - law.limiting_fraction) *
                std::exp(-law.decay_per_meter_per_second * sliding_speed) +
            law.limiting_fraction);
}

TangentialContactSolver::TangentialContactSolver(
    const TangentialContactConfiguration& configuration,
    const KalkerCoefficientTable& coefficients)
    : configuration_(configuration), coefficients_(coefficients) {
    if (configuration_.longitudinal_cells < kMinimumResolution ||
        configuration_.lateral_strips < kMinimumResolution) {
        throw std::invalid_argument(
            "TangentialContactSolver: both resolutions need at least three "
            "divisions for the march to have an interior, and they are " +
            std::to_string(configuration_.longitudinal_cells) + " and " +
            std::to_string(configuration_.lateral_strips));
    }
    if (!std::isfinite(configuration_.refinement_width) ||
        !(configuration_.refinement_width > 0.0) ||
        configuration_.refinement_width > 2.0) {
        throw std::invalid_argument(
            "TangentialContactSolver: the refinement width must be finite and "
            "in (0, 2], the patch being two wide in these units, and " +
            std::to_string(configuration_.refinement_width) + " is not");
    }
    const ContactMaterial& material = configuration_.material;
    if (!std::isfinite(material.youngs_modulus_pascals) ||
        !(material.youngs_modulus_pascals > 0.0) ||
        !(material.poisson_ratio >= 0.0) || !(material.poisson_ratio < 0.5)) {
        throw std::invalid_argument(
            "TangentialContactSolver: the material needs a positive finite "
            "Young's modulus and a Poisson ratio in [0, 0.5)");
    }
    shear_modulus_pascals_ = material.youngs_modulus_pascals /
                             (2.0 * (1.0 + material.poisson_ratio));

    longitudinal_cells_ = configuration_.longitudinal_cells;
    lateral_strips_ = configuration_.lateral_strips;

    // The worst case the layout can produce. The refined march emits at most
    // one strip per initial division plus one per halving, on each of two
    // sides; the fallback layout emits one per refinement width plus two.
    const std::size_t fallback_strips =
        static_cast<std::size_t>(std::floor(2.0 / configuration_.refinement_width)) +
        2;
    const std::size_t march_strips =
        2 * (lateral_strips_ + 1 + static_cast<std::size_t>(kMaximumHalvings));
    strip_capacity_ = std::max(fallback_strips, march_strips);
}

void TangentialContactSolver::PrepareWorkspace(
    TangentialContactWorkspace& workspace) const {
    workspace.strips_.reserve(strip_capacity_);
}

void TangentialContactSolver::LayStrips(
    double spin_pole_longitudinal, double spin_pole_lateral,
    TangentialContactWorkspace& workspace) const {
    std::vector<TangentialContactWorkspace::Strip>& strips = workspace.strips_;
    strips.reserve(strip_capacity_);
    strips.clear();

    const double tolerance = configuration_.refinement_width;
    const double divisions = static_cast<double>(lateral_strips_);

    // The layout refines only when the spin pole lies inside the patch. Outside
    // it — which includes the whole of the no-spin case, where the pole runs
    // away to infinity — there is nothing to bunch strips around and the
    // uniform layout is the right one.
    const bool refine = std::isfinite(spin_pole_longitudinal) &&
                        std::isfinite(spin_pole_lateral) &&
                        spin_pole_longitudinal * spin_pole_longitudinal +
                                spin_pole_lateral * spin_pole_lateral <
                            1.0;
    if (!refine) {
        const double width = 2.0 / divisions;
        for (std::size_t index = 0; index < lateral_strips_; ++index) {
            strips.push_back(
                {-1.0 + (static_cast<double>(index) + 0.5) * width, width});
        }
        return;
    }

    // Two marches, each running inward from one edge of the patch toward the
    // pole. Working in a coordinate that starts at one on the marching side
    // makes both marches the same code.
    for (const double side : {1.0, -1.0}) {
        const double pole = spin_pole_lateral * side;
        const double span = 1.0 - pole;
        const std::size_t initial_count = static_cast<std::size_t>(
            std::max(1.0, std::floor(span * divisions / 2.0) + 1.0));
        double width = span / static_cast<double>(initial_count);
        // Where the refinement begins, frozen at the initial width. It does not
        // move as the width halves: once the march is inside this band it stays
        // inside, because each halving steps the next centre back into it.
        const double refinement_start = pole + width;
        double centre = 1.0 - 0.5 * width;
        int halvings = 0;
        while (true) {
            bool terminal = false;
            if (centre < refinement_start) {
                if (tolerance < 0.5 * width && halvings < kMaximumHalvings) {
                    width *= 0.5;
                    // Half of the *halved* width. The emitted strip is then the
                    // outer half of the cell that would have been, flush
                    // against the one before it. Using the width before the
                    // halving opens a gap.
                    centre += 0.5 * width;
                    ++halvings;
                } else {
                    terminal = true;
                }
            }
            strips.push_back({centre * side, width});
            if (terminal) {
                break;
            }
            centre -= width;
        }
    }

    // If the march could not reach the refinement width — which happens when
    // the pole sits almost exactly on the patch's edge and there is no room to
    // subdivide — fall back to a plain layout at that width, splitting whichever
    // strip contains the pole.
    double narrowest = strips.front().width;
    for (const TangentialContactWorkspace::Strip& strip : strips) {
        narrowest = std::min(narrowest, strip.width);
    }
    if (narrowest >= tolerance && narrowest <= 2.0 * tolerance) {
        return;
    }

    strips.clear();
    if (tolerance >= 1.0) {
        strips.push_back({0.0, 2.0});
        return;
    }
    const std::size_t base_count = static_cast<std::size_t>(
        std::max(1.0, std::floor(1.0 / tolerance)));
    const double base_width = 2.0 / static_cast<double>(base_count);
    for (std::size_t index = 0; index < base_count; ++index) {
        const double centre = -1.0 + (static_cast<double>(index) + 0.5) * base_width;
        const double left = centre - 0.5 * base_width;
        const double right = centre + 0.5 * base_width;
        if (left <= spin_pole_lateral && spin_pole_lateral <= right) {
            const double half = 0.5 * base_width;
            strips.push_back({centre - 0.5 * half, half});
            strips.push_back({centre + 0.5 * half, half});
        } else {
            strips.push_back({centre, base_width});
        }
    }
}

TangentialContactResult TangentialContactSolver::Solve(
    const TangentialContactPatch& patch, const Creepages& creepages,
    TangentialContactWorkspace& workspace) const {
    TangentialContactResult result;
    const double load = patch.normal_force_newtons;
    const double semi_longitudinal = patch.longitudinal_semi_axis_meters;
    const double semi_lateral = patch.lateral_semi_axis_meters;
    const double friction = patch.friction_coefficient;
    if (!(load > 0.0) || !(semi_longitudinal > 0.0) || !(semi_lateral > 0.0) ||
        !(friction > 0.0)) {
        return result;
    }

    const double aspect = semi_longitudinal / semi_lateral;
    const KalkerCoefficients kalker = coefficients_.At(aspect);

    // The three flexibilities: how much shear stress a unit of each creepage
    // builds per unit of travel through the patch. Each is the patch's size
    // divided by the material's stiffness and the corresponding tabulated
    // coefficient.
    const double longitudinal_flexibility =
        8.0 * semi_longitudinal / (3.0 * kalker.longitudinal * shear_modulus_pascals_);
    const double lateral_flexibility =
        8.0 * semi_longitudinal / (3.0 * kalker.lateral * shear_modulus_pascals_);
    const double spin_flexibility =
        std::numbers::pi * semi_longitudinal * std::sqrt(aspect) /
        (4.0 * kalker.lateral_spin * shear_modulus_pascals_);
    result.longitudinal_flexibility = longitudinal_flexibility;
    result.lateral_flexibility = lateral_flexibility;
    result.spin_flexibility = spin_flexibility;

    // Where the rigid slip vanishes, in the patch's own normalised coordinates.
    // With no spin there is no such point and both come out non-finite, which
    // is what selects the uniform strip layout.
    double spin_pole_longitudinal = std::numeric_limits<double>::quiet_NaN();
    double spin_pole_lateral = std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(creepages.spin_per_meter) && creepages.spin_per_meter != 0.0) {
        spin_pole_longitudinal =
            -(creepages.lateral * spin_flexibility) /
            (creepages.spin_per_meter * lateral_flexibility * semi_longitudinal);
        spin_pole_lateral = (creepages.longitudinal * spin_flexibility) /
                            (creepages.spin_per_meter * longitudinal_flexibility *
                             semi_lateral);
    }
    LayStrips(spin_pole_longitudinal, spin_pole_lateral, workspace);

    // The pressure the friction limit is taken from: a paraboloid over the
    // patch, normalised so that it integrates to the normal load. See the
    // header note — this is not the Hertz distribution the normal force came
    // from, and it is not meant to be.
    const double pressure_scale =
        2.0 * load / (std::numbers::pi * semi_longitudinal * semi_lateral);
    const double cells = static_cast<double>(longitudinal_cells_);

    for (const TangentialContactWorkspace::Strip& strip : workspace.strips_) {
        const double half_chord_squared = 1.0 - strip.centre * strip.centre;
        if (!(half_chord_squared > 0.0)) {
            continue;
        }
        const double half_chord = std::sqrt(half_chord_squared);
        const double lateral_position = semi_lateral * strip.centre;

        // The longitudinal stress rate is constant along a strip; the lateral
        // one is not, because the spin's contribution turns with position.
        const double longitudinal_rate =
            creepages.longitudinal / longitudinal_flexibility -
            creepages.spin_per_meter * lateral_position / spin_flexibility;

        const double normalised_step = 2.0 * half_chord / cells;
        const double cell_length = semi_longitudinal * normalised_step;
        const double cell_area = cell_length * strip.width * semi_lateral;

        // Material enters at the leading edge carrying nothing.
        double stress_longitudinal = 0.0;
        double stress_lateral = 0.0;
        double previous_position = half_chord;
        double current_position = half_chord - 0.5 * normalised_step;

        for (std::size_t cell = 0; cell < longitudinal_cells_; ++cell) {
            // The first step is a half step: from the leading edge to the first
            // cell's centre. Every step after it is a full cell. The area
            // weight, though, is the full cell for every cell including the
            // first — the step is how far the material travelled, the weight is
            // how much of the patch the cell covers, and they are not the same
            // question.
            const double step_length =
                semi_longitudinal * (previous_position - current_position);
            const double transport_position =
                0.5 * (previous_position + current_position);
            const double lateral_rate =
                creepages.lateral / lateral_flexibility +
                creepages.spin_per_meter * (semi_longitudinal * transport_position) /
                    spin_flexibility;

            const double trial_longitudinal =
                stress_longitudinal - longitudinal_rate * step_length;
            const double trial_lateral = stress_lateral - lateral_rate * step_length;

            const double depth =
                half_chord_squared - current_position * current_position;
            const double pressure = depth <= 0.0 ? 0.0 : pressure_scale * depth;
            const double limit = friction * pressure;

            stress_longitudinal = 0.0;
            stress_lateral = 0.0;
            if (limit > 0.0) {
                const double trial_magnitude =
                    std::hypot(trial_longitudinal, trial_lateral);
                if (trial_magnitude <= limit) {
                    // Still adhering: the surfaces have not yet slipped here.
                    stress_longitudinal = trial_longitudinal;
                    stress_lateral = trial_lateral;
                } else {
                    // Slipping: the stress follows the friction limit, keeping
                    // the direction the build-up gave it. A radial projection,
                    // not a per-component clamp — clamping each component
                    // separately would let the magnitude exceed the limit by up
                    // to a factor of the square root of two.
                    const double scale = limit / trial_magnitude;
                    stress_longitudinal = trial_longitudinal * scale;
                    stress_lateral = trial_lateral * scale;
                }
                result.friction_bound_newtons += limit * cell_area;
            }

            result.longitudinal_force_newtons += stress_longitudinal * cell_area;
            result.lateral_force_newtons += stress_lateral * cell_area;
            ++result.cell_count;

            previous_position = current_position;
            current_position -= normalised_step;
        }
        ++result.strip_count;
    }
    return result;
}

}  // namespace orvd::wheel_rail_contact
