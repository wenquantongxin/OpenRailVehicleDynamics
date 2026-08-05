#pragma once

#include <cstddef>
#include <vector>

#include "orvd/wheel_rail_contact/contact_creepage.h"
#include "orvd/wheel_rail_contact/kalker_coefficient_table.h"
#include "orvd/wheel_rail_contact/normal_contact_force.h"

// The force the contact patch transmits along the rail, given how fast the two
// surfaces are creeping past each other.
//
// ## How the patch is solved
//
// The patch is an ellipse. Material enters at its leading edge carrying no
// shear stress, and as it travels through it the surfaces deform elastically
// and the stress builds — linearly, at a rate the creepages set. It builds
// until it reaches what friction can hold, and from there the surfaces slide
// and the stress follows the friction limit instead. So the patch has an
// adhesion region at the front and a slip region behind it, and where the
// boundary falls is the whole answer.
//
// Marching across the patch one strip at a time and integrating that build-up
// is the simplification this solver is: it replaces the elastic coupling
// between strips with three flexibility coefficients, one per creepage. Those
// come from a tabulated solution of the exact elasticity problem.
//
// ## The strips are not uniform
//
// With spin, the stress direction rotates across the patch and there is a point
// — the spin pole — where the rigid slip vanishes. The solution changes fastest
// near it, so the strips are laid out to bunch up there: full width out at the
// edges, halving repeatedly as the march approaches the pole, from both sides.
//
// A consequence worth stating plainly: the strip layout depends on the
// creepages, so it changes from step to step. The tangential force is therefore
// not a smooth function of the creepages — it has small kinks wherever the
// layout changes. Smoothing that out would mean giving up the refinement, which
// is what makes the spin case accurate at all.
//
// ## Two things that look like errors and are not
//
// The pressure the friction limit is taken from is a **paraboloid**, not the
// Hertz semi-ellipsoid the normal force was computed with. Its peak is a third
// higher and its shape is different. It integrates to exactly the normal load,
// which is what the tangential solution needs of it, and it is the distribution
// this family of solvers has always used.
//
// The discrete resultant slightly **exceeds** the Coulomb product of friction
// and normal load — by about one part in seven hundred at the usual cell count.
// That is the midpoint rule integrating a quadratic, not a violated friction
// law: every individual cell respects its own limit exactly. The per-cell sum
// of those limits is reported alongside the force so that a consumer can check
// the real bound rather than the wrong one.

namespace orvd::wheel_rail_contact {

// Friction that falls off with sliding speed.
//
// Dry wheel-on-rail friction is not constant: it drops as the surfaces slide
// faster, towards a floor. The two shape parameters below set where the floor
// is, as a fraction of the static value, and how fast it is approached.
struct FrictionLaw {
    double static_coefficient{0.3};
    // The fraction of the static coefficient left at unbounded sliding speed.
    double limiting_fraction{0.4};
    // How fast the fall-off happens, per unit sliding speed.
    double decay_per_meter_per_second{0.55};
    // The reference speed is floored at this before the sliding speed is formed
    // from it, so that a creepage divided by a floored speed and then
    // multiplied back does not reintroduce the floor's discontinuity.
    double minimum_reference_speed_meters_per_second{0.01};
};

// The friction coefficient at this patch's operating point.
//
// The spin does not enter: only the two translational creepages do, through
// their magnitude. That is the law as formulated, not a simplification made
// here.
[[nodiscard]] double FrictionCoefficientAt(const FrictionLaw& law,
                                           const Creepages& creepages);

struct TangentialContactConfiguration {
    // Cells along the running direction within each strip, and strips across
    // the patch. Both are resolutions, not physical quantities.
    std::size_t longitudinal_cells{21};
    std::size_t lateral_strips{21};
    // The width the strip refinement stops at, in units where the patch spans
    // two. Refinement halves the strip width until it is no finer than this.
    double refinement_width{0.01};
    ContactMaterial material;
};

// The buffers one tangential solve needs. Sized once and reused; a solve
// allocates nothing after the first.
class TangentialContactWorkspace {
  public:
    TangentialContactWorkspace() = default;

  private:
    friend class TangentialContactSolver;

    struct Strip {
        double centre{0.0};
        double width{0.0};
    };
    std::vector<Strip> strips_;
};

struct TangentialContactPatch {
    double normal_force_newtons{0.0};
    double longitudinal_semi_axis_meters{0.0};
    double lateral_semi_axis_meters{0.0};
    // The friction coefficient over the whole patch. One number: the law is
    // evaluated once per patch, not per cell.
    double friction_coefficient{0.0};
};

struct TangentialContactResult {
    // The two in-plane force components, in the contact frame.
    double longitudinal_force_newtons{0.0};
    double lateral_force_newtons{0.0};

    // The sum over cells of each cell's own friction limit times its area. This
    // is the bound the resultant actually respects; the Coulomb product is a
    // continuum idealisation the discrete quadrature is allowed to exceed.
    double friction_bound_newtons{0.0};

    // The three flexibilities the creepages were multiplied by. Carried because
    // they are the whole of this solver's dependence on the tabulated
    // elasticity solution, and a consumer checking that dependence needs them.
    double longitudinal_flexibility{0.0};
    double lateral_flexibility{0.0};
    double spin_flexibility{0.0};

    // How many strips the layout ended up with, and how many cells were
    // marched. A count that moves between steps is the layout changing, which
    // is expected; a count that moves by an order of magnitude is the fallback
    // layout firing.
    std::size_t strip_count{0};
    std::size_t cell_count{0};
};

class TangentialContactSolver {
  public:
    // Throws std::invalid_argument when a resolution is below three, when the
    // refinement width is not in (0, 2], or when the material is unusable.
    TangentialContactSolver(const TangentialContactConfiguration& configuration,
                            const KalkerCoefficientTable& coefficients);

    // Solves one patch. Allocates nothing once the workspace has been through
    // one solve.
    //
    // A patch with no load, no extent or no friction returns all zeros: there
    // is nothing there to transmit a force, and that is not an error.
    [[nodiscard]] TangentialContactResult Solve(
        const TangentialContactPatch& patch, const Creepages& creepages,
        TangentialContactWorkspace& workspace) const;

    // Sizes a workspace for this solver without solving. The strip layout's
    // worst case is a property of the two resolutions and the refinement width
    // alone — never of the load, the patch's shape or the creepages — so it can
    // be reserved once and never grown.
    void PrepareWorkspace(TangentialContactWorkspace& workspace) const;

    // The shear modulus the flexibilities are formed with, derived from the
    // material rather than configured separately so that the two cannot drift
    // apart.
    [[nodiscard]] double shear_modulus_pascals() const {
        return shear_modulus_pascals_;
    }

  private:
    void LayStrips(double spin_pole_longitudinal, double spin_pole_lateral,
                   TangentialContactWorkspace& workspace) const;

    TangentialContactConfiguration configuration_;
    KalkerCoefficientTable coefficients_;
    double shear_modulus_pascals_{0.0};
    std::size_t longitudinal_cells_{3};
    std::size_t lateral_strips_{3};
    std::size_t strip_capacity_{0};
};

}  // namespace orvd::wheel_rail_contact
