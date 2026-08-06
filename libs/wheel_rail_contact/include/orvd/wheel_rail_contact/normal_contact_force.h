#pragma once

// How hard a wheel and a rail push each other apart, given the shape of their
// overlap.
//
// The contact geometry says how much of the wheel is inside the rail. This says
// what force that costs. The two are kept apart because the geometry depends on
// no material property and this depends on nothing but the overlap's shape,
// the material, and how fast the two are closing.
//
// ## Why an equivalent penetration
//
// Hertz's solution for two elastic bodies pressed together relates the force to
// the *approach* of two distant points — how much closer their far ends come.
// The geometry does not measure that. What it measures is an interpenetration:
// how deeply one undeformed surface lies inside the other. The two are not the
// same number, and using one where the other belongs gives a force wrong by a
// factor near two.
//
// The bridge is to replace the overlap's cross-section with the circular
// segment of the same area on the same chord, take that segment's height, and
// scale it. The scale factor is the reciprocal of the shape factor a full
// strip-wise solve of the same patch would produce, held constant here rather
// than recomputed per patch — which is exactly the approximation this contact
// law is, and the reason it is fast enough to run inside an integrator.
//
// ## What the two semi-axes are, and why they are not sorted
//
// `longitudinal` comes from how far the overlap reaches along the running
// direction; `lateral` from how wide it is across the rail. Either may be the
// larger. They are emitted in that fixed order and never sorted, because the
// tangential solution reads their ratio and the ratio's *sense* — long-and-thin
// along the track versus across it — is a different physical situation with
// different creep coefficients. Sorting them here would silently merge the two.
//
// The elliptic integral inside the force does need them sorted, and it sorts a
// local copy.
//
// ## The damping is not Hunt–Crossley
//
// The damping force is linear in the closing speed, with a coefficient that
// scales as the square root of the contact's current secant stiffness. That
// keeps the damping ratio roughly constant as the contact stiffens rather than
// making the damping vanish at light load. It is not a `c·δⁿ·δ̇` law and does
// not reduce to one.

namespace orvd::wheel_rail_contact {

// The elastic properties of the two bodies, taken as identical.
struct ContactMaterial {
    double youngs_modulus_pascals{210.0e9};
    double poisson_ratio{0.28};
};

struct NormalContactConfiguration {
    ContactMaterial material;
    // The factor turning the equal-area segment's height into the approach
    // Hertz's solution wants. See the header note: it is the reciprocal of a
    // shape factor, held constant instead of solved per patch.
    double penetration_equivalence_factor{1.8181818181818181};
    // The reference pair the damping coefficient is scaled from. Only their
    // ratio and the square root matter: the coefficient is
    // `reference_damping * sqrt(stiffness / reference_stiffness)`.
    double reference_damping_newton_seconds_per_meter{100000.0};
    double reference_stiffness_newtons_per_meter{500000000.0};
    // Below this equivalent penetration the damping is not formed at all. It is
    // not a smoothing threshold: it keeps the secant stiffness, which divides
    // by the penetration, from being formed at a penetration of zero.
    double damping_activation_penetration_meters{1.0e-12};
};

// The shape of one overlap, as the geometry measured it.
struct NormalContactGeometry {
    // The overlap's cross-sectional area and the wheel's arc across it. These
    // two together fix the equivalent penetration; nothing else does.
    double cross_section_area_square_meters{0.0};
    double arc_width_meters{0.0};
    // How far the overlap reaches along the running direction. A positive
    // contact requires this value to be finite and positive; the selected
    // model admits no estimated substitute.
    double longitudinal_length_meters{0.0};
    // The deepest vertical overlap. It gates the whole evaluation.
    double vertical_penetration_meters{0.0};
};

struct NormalContactResult {
    // The force pushing the two bodies apart, along the contact normal. Never
    // negative: a contact cannot pull.
    double normal_force_newtons{0.0};
    // Its two parts, which always sum to the total. The elastic one is what the
    // overlap costs; the damping one is what the closing speed costs, and it is
    // negative while separating.
    //
    // When separation is fast enough to drive the total to zero, the damping
    // reported is the part that was actually applied — the exact negative of
    // the elastic force — not the larger value the coefficient and the speed
    // would give. The sum is kept an invariant because a consumer adding the
    // two and getting something other than the total is a worse failure than
    // not knowing how much damping was clipped.
    double elastic_force_newtons{0.0};
    double damping_force_newtons{0.0};

    // The contact ellipse. Not sorted — see the header note.
    double longitudinal_semi_axis_meters{0.0};
    double lateral_semi_axis_meters{0.0};

    double equivalent_penetration_meters{0.0};
    // The greatest pressure in the patch, at its centre.
    double maximum_pressure_pascals{0.0};

    // Whether the patch produced any force at all. False means one of the
    // validity gates rejected it, and every number above is zero.
    bool in_contact{false};
};

// The complete elliptic integral of the first kind.
//
// The argument is the **parameter** m = k², not the modulus k and not the
// modular angle. Feeding it a modulus produces a smaller integral and therefore
// a force several percent low, with nothing to warn that it happened — and the
// error vanishes as the contact patch becomes circular, so a round-patch check
// will not find it. Any test of this must use a deliberately elongated patch.
//
// Evaluated by the arithmetic-geometric mean, which converges quadratically:
// four iterations at the eccentricities a rail contact produces.
[[nodiscard]] double CompleteEllipticIntegralFirstKind(double parameter);

// The area of a circular segment of the given chord and height.
//
// Exposed because the inversion below is a bisection on it, and a caller
// checking that inversion should check it against this rather than against a
// shallow-segment approximation. The approximation `2·chord·height/3` differs
// by about 6e-5 relative at the depths a rail contact reaches, which is far
// more than the inversion's own tolerance.
[[nodiscard]] double CircularSegmentArea(double height_meters, double chord_meters);

// The height of the circular segment with the given chord and area.
//
// Bisection, because the forward map has no elementary inverse. The bracket
// starts just above zero rather than at zero: the segment's radius divides by
// the height, so a height of exactly zero has no circle through it.
//
// Two limits worth knowing rather than guarding: an area larger than the widest
// segment on that chord returns the chord itself, and a segment shallower than
// about three parts in a billion of the chord cannot be resolved at all,
// because the cosine of its half-angle rounds to one. Both are far outside what
// a real contact produces.
[[nodiscard]] double SolveCircularSegmentHeight(double chord_meters,
                                                double area_square_meters);

class NormalContactLaw {
  public:
    // Throws std::invalid_argument when a material or reference constant is not
    // usable — a non-positive modulus, a Poisson ratio outside [0, 0.5), a
    // non-positive reference stiffness or damping.
    explicit NormalContactLaw(const NormalContactConfiguration& configuration);

    // `approach_speed_meters_per_second` is positive while the two bodies close
    // on each other along the contact normal, negative while they separate.
    //
    // Allocates nothing on the successful path. A patch with no positive depth,
    // area or width comes back with `in_contact` false and every number zero:
    // stopping contact is an ordinary event. Throws std::invalid_argument when
    // those contact measures are positive but the three-dimensional
    // longitudinal length is not finite and positive; guessing a replacement
    // length is forbidden.
    [[nodiscard]] NormalContactResult Solve(
        const NormalContactGeometry& geometry,
        double approach_speed_meters_per_second) const;

    // The plane-strain modulus of the pair: the single elastic constant the
    // Hertz solution for two identical bodies depends on.
    [[nodiscard]] double equivalent_modulus_pascals() const {
        return equivalent_modulus_pascals_;
    }

  private:
    NormalContactConfiguration configuration_;
    double equivalent_modulus_pascals_{0.0};
};

}  // namespace orvd::wheel_rail_contact
