[中文](NORMAL_CONTACT_FORCE.md)

# Normal contact force

This document explains how ORVD converts the geometry of one contact patch into a normal load. It constructs an equivalent penetration from the overlap cross-section, obtains elastic force and peak pressure from an elliptical Hertz relation, and adds velocity damping scaled by the current secant stiffness. The core law is `NormalContactLaw::Solve` in [normal_contact_force.cc](../../../libs/wheel_rail_contact/src/normal_contact_force.cc), and its place in the complete contact chain is `WheelRailContactModel::Evaluate` in [wheel_rail_contact_model.cc](../../../libs/wheel_rail_contact/src/wheel_rail_contact_model.cc).

## 1. Scope

The normal law reads six geometric quantities: cross-sectional overlap area, wheel-envelope arc width, three-dimensional longitudinal length, vertical penetration, local rolling radius and common-normal angle. It also reads normal approach speed. Its principal mechanical outputs are total normal force, elastic and damping components, longitudinal and lateral semi-axes, equivalent penetration and peak pressure.

See [Contact geometry](CONTACT_GEOMETRY.en.md) for construction of the geometric quantities, [Creepages and the contact frame](CREEPAGE_AND_CONTACT_FRAME.en.md) for approach speed and the contact frame, and [Tangential contact](TANGENTIAL_CONTACT_FASTSIM.en.md) for tangential consumption of the axes and normal load.

The model assumes two deformable elastic bodies of identical material, the wheel and the rail, and a constant penetration-equivalence factor. It is neither a full three-dimensional elastic free-boundary solution nor a Hunt–Crossley penetration-power damping law.

## 2. Notation

| Symbol | Meaning | Implementation quantity |
|---|---|---|
| $A$ | Planar cross-sectional overlap area | `cross_section_area_square_meters` |
| $W$ | Wheel-envelope arc width, used as the equal-area segment chord | `arc_width_meters` |
| $L_3$ | Three-dimensional longitudinal length from contact geometry | `longitudinal_length_meters` |
| $\delta_v$ | Vertical penetration | `vertical_penetration_meters` |
| $r$ | Local rolling radius at contact | `rolling_radius_meters` |
| $\gamma$ | Common-normal angle | `common_normal_angle_radians` |
| $r_{\mathrm{eff}}$ | Effective radius for the analytic longitudinal baseline | `effective_radius` |
| $h$ | Circular-segment height with equal area and chord $(A,W)$ | `segment_height` |
| $f$ | Penetration-equivalence factor | `penetration_equivalence_factor` |
| $\delta_{\mathrm{eq}}$ | Equivalent penetration | `equivalent_penetration_meters` |
| $E,\nu$ | Common Young's modulus and Poisson ratio of both bodies | `ContactMaterial` |
| $E^*$ | Effective contact modulus for two identical materials | `equivalent_modulus_pascals` |
| $a,b$ | Longitudinal and lateral semi-axes | `longitudinal_semi_axis_meters`, `lateral_semi_axis_meters` |
| $K(m)$ | Complete elliptic integral of the first kind with parameter $m$ | `CompleteEllipticIntegralFirstKind` |
| $p_0^{\mathrm H}$ | Peak pressure of the Hertzian elliptical contact | `maximum_pressure_pascals` |
| $F_e,F_d,N$ | Elastic, damping and total normal force | `elastic_force_newtons`, `damping_force_newtons`, `normal_force_newtons` |
| $v_n$ | Normal relative speed, positive in approach | `approach_speed_meters_per_second` |
| $k_s$ | Current secant stiffness $F_e/\delta_{\mathrm{eq}}$ | `secant_stiffness` |
| $c_{\mathrm{ref}},k_{\mathrm{ref}}$ | Reference damping and reference stiffness of the damping scaling | `reference_damping_newton_seconds_per_meter`, `reference_stiffness_newtons_per_meter` |

## 3. Model

### 3.1 From geometric overlap to equivalent penetration

The quantities $\delta_v$ and $A$ from contact geometry describe interpenetration of the two undeformed profiles. The approach in Hertz theory describes relative motion of the remote parts of two elastic bodies. The model connects them through an equal-area circular segment.

For a chord $W$ and segment height $h$, the radius of the circle through the two chord endpoints and the segment apex is

$$
r_{\mathrm{seg}}=\frac{W^2}{8h}+\frac h2.
$$

The corresponding central angle and segment area are

$$
\vartheta=2\arccos\frac{r_{\mathrm{seg}}-h}{r_{\mathrm{seg}}},
$$

$$
A_{\mathrm{seg}}(h,W)
=\frac12r_{\mathrm{seg}}^2(\vartheta-\sin\vartheta).
$$

On the height interval $0<h\leq W$ used by this model, the height $h$ is obtained from the monotone inverse problem

$$
A_{\mathrm{seg}}(h,W)=A
$$

and equivalent penetration is defined by

$$
\delta_{\mathrm{eq}}=f h.
$$

This exact inverse requires that the target area not exceed

$$
A_{\max}(W)=A_{\mathrm{seg}}(W,W).
$$

The area here is the planar area $A$, not the arc-weighted area. The constant $f$ compresses the stripwise shape effect into one penetration-equivalence factor and is the main approximation of this normal law. In the shallow-segment limit,

$$
A_{\mathrm{seg}}\sim\frac23Wh,
\qquad
h\sim\frac{3A}{2W},
$$

but the implementation evaluates the full circular-segment expression.

### 3.2 Longitudinal scale and analytic baseline

The lateral semi-axis follows directly from arc width:

$$
b=\frac W2.
$$

The model first constructs an analytic longitudinal baseline for every patch. With a positive cosine floor $\epsilon_{\cos}>0$ regularizing a nearly perpendicular common normal,

$$
r_{\mathrm{eff}}
=
\frac{r}
{\max(|\cos\gamma|,\epsilon_{\cos})},
$$

$$
L_{\mathrm{base}}
=
2\sqrt{\max(0,2r_{\mathrm{eff}}\delta_v)}.
$$

If $L_3$ is finite and positive, the model uses $L=L_3$; otherwise it uses $L=L_{\mathrm{base}}$. The longitudinal semi-axis is

$$
a=\frac L2.
$$

Thus $L_3=0$ means that the three-dimensional geometric length is unavailable, not that the normal patch must have zero length; the analytic baseline still defines the longitudinal semi-axis in that case.

### 3.3 Elliptical Hertz relation

For two bodies with the same $E,\nu$,

$$
\frac1{E^*}=
\frac{2(1-\nu^2)}{E}.
$$

The output axes retain fixed meanings: $a$ is longitudinal and $b$ is lateral, irrespective of their relative magnitudes. Sorting is local to the normal law and does not change the fixed meaning of the emitted axes; the sorted copies enter the elliptic parameter, $p_0^{\mathrm H}$ and $F_e$ alike:

$$
a_{\max}=\max(a,b),
\qquad
a_{\min}=\min(a,b),
$$

$$
\rho=\frac{a_{\min}}{a_{\max}},
\qquad
m=1-\rho^2.
$$

The quantity $m$ is eccentricity squared—the elliptic parameter—not the modulus. The complete elliptic integral of the first kind is

$$
K(m)=
\int_0^{\pi/2}
\frac{d\varphi}
{\sqrt{1-m\sin^2\varphi}}.
$$

The $\varphi$ in this integral is a dummy variable of integration, unrelated to the pose roll.

Peak pressure and elastic normal force are

$$
p_0^{\mathrm H}=
\frac{\delta_{\mathrm{eq}}E^*}
     {a_{\min}K(m)},
$$

$$
F_e=
\frac23\pi a_{\max}a_{\min}p_0^{\mathrm H}.
$$

Equivalently,

$$
F_e=
\frac{2\pi}{3}
\frac{a_{\max}E^*\delta_{\mathrm{eq}}}{K(m)}.
$$

Inverting $F_e=\frac23\pi a_{\max}a_{\min}p_0^{\mathrm H}$ gives $p_0^{\mathrm H}=3F_e/(2\pi ab)$. [Tangential contact](TANGENTIAL_CONTACT_FASTSIM.en.md) instead denotes the peak of its FASTSIM paraboloidal pressure by $p_0^{\mathrm F}=2N/(\pi ab)$. On the same pair of semi-axes their ratio is

$$
\frac{p_0^{\mathrm F}}{p_0^{\mathrm H}}=\frac{4N}{3F_e},
$$

which equals $4/3$ only when $N=F_e$. The superscripts explicitly distinguish the peaks of the two pressure shapes.

In the circular limit $a=b$, $m=0$ and $K(0)=\pi/2$, giving

$$
F_e=\frac43E^*a\,\delta_{\mathrm{eq}}.
$$

For fixed axes, $F_e$ is linear in $\delta_{\mathrm{eq}}$. The complete wheel-rail system is also nonlinear because contact geometry changes patch shape and scale with state.

### 3.4 Damping and the no-tension condition

The current secant stiffness is

$$
k_s=\frac{F_e}{\delta_{\mathrm{eq}}}.
$$

The damping coefficient scales with its square root:

$$
c=c_{\mathrm{ref}}
\sqrt{\frac{k_s}{k_{\mathrm{ref}}}},
\qquad
F_d=cv_n.
$$

This term is formed only above a small positive equivalent-penetration activation threshold, avoiding a secant stiffness at vanishing penetration. The total force obeys the no-tension condition

$$
N=\max(0,F_e+F_d).
$$

If separation is fast enough to make the unclipped sum negative, the local normal law sets $N=0$ and rewrites the reported damping component as $F_d=-F_e$, preserving

$$
F_e+F_d=N.
$$

For the local normal law, this remains a unilateral-contact result with valid geometry and zero normal load. The complete physical chain passes only patches with $N>0$ into the creepage and tangential-contact stages. Thus $N=0$ means that a geometric candidate becomes inactive during force assembly, not that it remains a contact patch with Coulomb-friction capacity.

### 3.5 Mathematical domain

The normal law admits a patch on the geometric domain

$$
A>0,\qquad W>0,\qquad \delta_v>0,\qquad r>\epsilon_R,
$$

with all four quantities finite. Here $\epsilon_R>0$ is a rolling-radius floor that excludes a degenerate local radius. Circular-segment equivalence additionally requires $f>0$; the material domain is $E>0$ and $0\leq\nu<1/2$; the damping scales require $c_{\mathrm{ref}}>0$ and $k_{\mathrm{ref}}>0$. The value $L_3$ is not required because the analytic baseline covers an unavailable measurement.

The bound $A\leq A_{\max}(W)$ of §3.1 is a condition of a different kind: it is the solvability condition of the exact circular-segment inverse on the height interval $0<h\leq W$ used here, not an admission gate. When $A$ crosses that bound the inverted height saturates at $h=W$, so $\delta_{\mathrm{eq}}$ and the normal force remain continuous in $A$ and merely lose differentiability there; the contact state does not change branch.

On the admissible domain, $a,b,\delta_{\mathrm{eq}},K(m),p_0^{\mathrm H},F_e$ are positive. When the common normal approaches a right angle, the cosine floor replaces the unbounded $r_{\mathrm{eff}}$ by a finite regularization. This is a model continuation rather than an exact asymptotic solution for perpendicular contact.

## 4. Numerical algorithms

### 4.1 Inverting circular-segment height

`SolveCircularSegmentHeight` bisects $A_{\mathrm{seg}}(h,W)-A$ on $[h_{\min},W]$, where $h_{\min}=10^{-15}\,\mathrm m$ because the expression for $r_{\mathrm{seg}}$ degenerates at $h=0$. If the midpoint area exceeds the target, the upper half is discarded; otherwise the lower half is discarded. Iteration stops when the height bracket is smaller than an absolute length tolerance or a finite iteration limit is reached. If $A$ lies outside the area range mapped by this numerical interval, the returned height saturates near the corresponding endpoint instead of satisfying the exact inverse equation.

The forward area calculation clamps the inverse-cosine argument to $[-1,1]$. Bisection uses an absolute rather than a relative height tolerance, so its resolution scale does not vary with chord length.

### 4.2 Complete elliptic integral of the first kind

The implementation first restricts $m$ to a finite closed subinterval of $[0,1)$, avoiding the logarithmic divergence as $m\to1$. It then uses the arithmetic-geometric mean:

$$
a_0=1,\qquad g_0=\sqrt{1-m},
$$

$$
a_{j+1}=\frac{a_j+g_j}{2},
\qquad
g_{j+1}=\sqrt{a_jg_j},
$$

$$
K(m)=
\frac{\pi}
{2\,\operatorname{AGM}(1,\sqrt{1-m})}.
$$

The arithmetic-geometric mean converges quadratically. The difference between its arithmetic and geometric iterates is the stopping measure, with a finite iteration cap.

### 4.3 Per-patch evaluation order

The theoretical order of `NormalContactLaw::Solve` is:

1. Determine whether the geometry lies in the admissible domain.
2. Form $b$ and the analytic longitudinal baseline, then override the baseline with an available $L_3$ to obtain $a$.
3. Invert the equal-area circular segment and form $\delta_{\mathrm{eq}}$.
4. Locally sort the axes and compute $m$ and $K(m)$.
5. Compute $p_0^{\mathrm H}$, $F_e$ and the secant stiffness.
6. Add damping in its active region and impose the no-tension clipping.
7. Continue to creepage and tangential force only when the complete contact model sees $N>0$.

The work per patch consists of one bounded bisection, one bounded arithmetic-geometric-mean iteration and a constant number of scalar operations.

## 5. Non-smoothness and theoretical applicability

- When $A$, $W$, $\delta_v$ or $r$ crosses the boundary of the admissible geometric domain, the normal-contact state changes branch. Crossing $A_{\max}(W)$ is not such a boundary; see §3.5.
- When $L_3$ changes between available and unavailable, $a$ switches between the three-dimensional measurement and analytic baseline; the force need not remain continuous.
- Damping is piecewise-defined at the positive equivalent-penetration activation threshold. The secant stiffness does not vanish as the penetration approaches that threshold, so for $v_n\neq0$ both $F_d$ and $N$ jump discontinuously there rather than merely losing differentiability.
- No-tension clipping is continuous but nondifferentiable at $F_e+F_d=0$; the assembly layer also changes the number of retained patches at that boundary.
- Local major/minor-axis sorting changes branch at $a=b$, but the symmetric elliptic integral and force remain continuous.
- The model assumes that each geometric island can be represented by an equivalent ellipse with axes $(a,b)$ and that an equal-area cross-sectional circular segment can stand in for elastic approach. Strongly conformal contact, coupled multiple patches, plasticity, roughness and material nonlinearity are outside this theory.
- Damping is a linear velocity term scaled by current secant stiffness. It is a low-order closure for contact dissipation and should not be interpreted as the unique law derived from material viscoelasticity.

## 6. Implementation mapping

| Theoretical object | Main implementation |
|---|---|
| Circular-segment area and height inversion | `CircularSegmentArea` and `SolveCircularSegmentHeight` in [normal_contact_force.cc](../../../libs/wheel_rail_contact/src/normal_contact_force.cc) |
| Effective modulus, Hertz force and damping | The `NormalContactLaw` constructor and `NormalContactLaw::Solve` in [normal_contact_force.cc](../../../libs/wheel_rail_contact/src/normal_contact_force.cc) |
| Source of geometric quantities | `ContactGeometrySolver::Solve` in [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
| Approach speed, positive-load gate and tangential handoff | `WheelRailContactModel::Evaluate` in [wheel_rail_contact_model.cc](../../../libs/wheel_rail_contact/src/wheel_rail_contact_model.cc) |
