[中文](TANGENTIAL_CONTACT_FASTSIM.md)

# Tangential contact force: FASTSIM strip marching

This chapter explains how ORVD obtains the longitudinal and lateral tangential-force components over an elliptical contact patch from normal load, creepages, Kalker coefficients and a friction law. The implementation follows the local-flexibility and strip-marching idea represented by [Kalker's FASTSIM](https://doi.org/10.1080/00423118208968684); the pressure shape, strip refinement and discrete march used by this repository are defined by [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc).

## 1. Model scope and notation

The contact patch is an ellipse with semi-axes $a$ and $b$ in the rolling and lateral directions. Introduce normalized coordinates

$$
u=\frac{x}{a},\qquad v=\frac{y}{b},\qquad u^2+v^2\le1.
$$

Let $N$ be the normal load, taken as the total normal force including damping, and let $\xi_x$, $\xi_y$ and $\xi_{sp}$ be the longitudinal, lateral and spin creepages defined in [Creepages and the contact frame](CREEPAGE_AND_CONTACT_FRAME.en.md). The coefficient of friction $\mu$ is evaluated once per contact patch and shared by every cell in it. The Kalker coefficients determine three flexibility scales $L_x$, $L_y$ and $L_{sp}$; see [Kalker linear creepage coefficients](KALKER_COEFFICIENTS.en.md). This chapter assumes positive $N$, $a$, $b$ and $\mu$, with a semi-axis ratio in the selected domain of the Kalker coefficient functions.

## 2. Stress build-up model

### 2.1 Local flexibilities

With $G=E/[2(1+\nu)]$ and $\kappa=a/b$, the implementation uses the following expressions. Here $\kappa$ follows the local notation of the Kalker-coefficient document and denotes the semi-axis ratio, not planar track curvature.

$$
L_x=\frac{8a}{3C_{11}G},\qquad
L_y=\frac{8a}{3C_{22}G},\qquad
L_{sp}=\frac{\pi a\sqrt{\kappa}}{4C_{23}G}.
$$

These quantities convert creepages into shear-stress build-up rates along material travel. On a strip at lateral coordinate $y$,

$$
r_x(y)=\frac{\xi_x}{L_x}-\frac{\xi_{sp} y}{L_{sp}},
\qquad
r_y(x)=\frac{\xi_y}{L_y}+\frac{\xi_{sp} x}{L_{sp}}.
$$

$r_x$ is constant along one strip; with spin, $r_y$ varies linearly with longitudinal position. The statement that stress grows linearly along travel therefore applies only to $\tau_x$, or to $\tau_y$ when $\xi_{sp}=0$; it cannot be applied unconditionally to both components.

### 2.2 Continuous expression in adhesion

On the strip at normalized lateral coordinate $v$, the half-chord is

$$
h(v)=\sqrt{1-v^2}.
$$

Material enters with zero shear stress at the leading edge $x_\ell=ah$ and travels toward the trailing edge. Before the friction limit is reached,

$$
\begin{aligned}
\tau_x(x,y)
&=-\int_x^{x_\ell}r_x(y)\,dx'
=-r_x(y)(x_\ell-x),\\
\tau_y(x,y)
&=-\int_x^{x_\ell}r_y(x')\,dx'\\
&=-\frac{\xi_y}{L_y}(x_\ell-x)
-\frac{\xi_{sp}}{2L_{sp}}(x_\ell^2-x^2).
\end{aligned}
$$

Thus $\tau_y$ is quadratic in $x$ when spin is present. The source evaluates $r_y$ at the midpoint of every transport step. Because $r_y$ is linear in $x$, midpoint integration exactly integrates this linear rate over an individual adhering step.

The minus sign follows from the relation between the marching and coordinate directions: a positive translational creepage produces a negative corresponding tangential stress and resultant.

### 2.3 Zero of the stress build-up rates

For $\xi_{sp}\ne0$, solving $r_x=0$ and $r_y=0$ simultaneously gives

$$
x_p=-\frac{\xi_yL_{sp}}{\xi_{sp}L_y},
\qquad
y_p=\frac{\xi_xL_{sp}}{\xi_{sp}L_x},
$$

or, in normalized coordinates,

$$
u_p=-\frac{\xi_yL_{sp}}{\xi_{sp}L_y\,a},
\qquad
v_p=\frac{\xi_xL_{sp}}{\xi_{sp}L_x\,b}.
$$

The source names these quantities `spin_pole_longitudinal` and `spin_pole_lateral`. Their implemented formulas identify the common zero of the two stress build-up rates, not unconditionally the zero of rigid slip: the rigid-slip field contains none of $L_x$, $L_y$ or $L_{sp}$. The two points coincide only under additional relations among the flexibilities.

## 3. Pressure, friction and adhesion-slip partition

### 3.1 Paraboloid pressure

The tangential solver uses the normalized paraboloid pressure

$$
p(u,v)=p_0^{\mathrm F}(1-u^2-v^2),
\qquad
p_0^{\mathrm F}=\frac{2N}{\pi ab},
$$

which satisfies

$$
\iint_{u^2+v^2<1}p(u,v)\,ab\,du\,dv=N.
$$

This is not the semi-ellipsoidal pressure of the normal Hertz solution; it is an independent shape chosen by this tangential approximation. This document denotes the peak of the FASTSIM paraboloid by $p_0^{\mathrm F}$; [Normal contact force](NORMAL_CONTACT_FORCE.en.md) instead denotes the Hertz peak by $p_0^{\mathrm H}=3F_e/(2\pi ab)$. Their ratio is

$$
\frac{p_0^{\mathrm F}}{p_0^{\mathrm H}}=\frac{4N}{3F_e},
$$

which equals $4/3$ only when $N=F_e$. The normal chapter's peak is built from the elastic normal force $F_e$ alone, whereas the $N$ of this chapter is the total normal force including damping, so the former must not be substituted into the friction bound below. Pressure must do more than integrate to $N$: pointwise it determines the friction bound

$$
\tau_{\max}(u,v)=\mu p(u,v),
$$

and therefore determines when each cell changes from adhesion to slip, where the adhesion-slip boundary lies and the resultant before full saturation. Two pressure fields with the same integral but different spatial shapes do not in general produce the same tangential solution.

### 3.2 Adhesion and slip

For one transport step, a trial stress $\boldsymbol\tau^*$ is first formed from the build-up rates. If

$$
\|\boldsymbol\tau^*\|\le\mu p,
$$

the cell adheres and accepts the trial value. Otherwise it is projected radially onto the friction circle while preserving the trial direction:

$$
\boldsymbol\tau=\mu p\,
\frac{\boldsymbol\tau^*}{\|\boldsymbol\tau^*\|}.
$$

The radial projection preserves stress direction and makes every cell satisfy its own local friction bound. The final resultant is the sum of cell stresses multiplied by their areas.

Write the discrete tangential resultant as $\mathbf F_t=[F_x,F_y]^{\mathsf T}$. When the whole patch is in full slip, each cell has $\|\boldsymbol\tau_{ij}\|=\mu p_{ij}$, so

$$
\|\mathbf F_t\|\leq\sum_{i,j}\mu p_{ij}\,\Delta A_{ij}.
$$

Equality requires all cell stresses to have the same direction, as they do in full slip without spin. Midpoint quadrature over the uniform strips does not preserve the continuous pressure normalization: its scalar sum can exceed $\mu N$, and so can the discrete resultant. With spin, cell directions need not coincide, leaving only the scalar-quadrature bound above. What is exceeded is the discrete approximation of the continuous pressure integral, not any cell's local friction bound; $\mu N$ is therefore not a strict upper bound on the discrete resultant.

### 3.3 Friction decreasing with sliding speed

`FrictionCoefficientAt` first forms sliding speed from the two translational creepages,

$$
v_s=\max(|v_{\mathrm{ref}}|,v_{\min})
\sqrt{\xi_x^2+\xi_y^2},
$$

where $v_{\mathrm{ref}}$ is the already floored reference speed $V$ of [Creepages and the contact frame](CREEPAGE_AND_CONTACT_FRAME.en.md), that is, the divisor in the definition of the creepages, and $v_{\min}$ is a second reference-speed floor carried by the friction law itself, independent of the creepage-side floor $V_{\min}$. Which reading the expression takes depends on how the two floors compare. Whenever $v_{\min}\le V_{\min}$, the bound $|v_{\mathrm{ref}}|\ge V_{\min}$ makes the outer maximum return $|v_{\mathrm{ref}}|$, which cancels against the divisor inside the creepages:

$$
v_s=|V|\sqrt{\left(\frac{v_{C,x}}{V}\right)^2+\left(\frac{v_{C,y}}{V}\right)^2}
=\sqrt{v_{C,x}^2+v_{C,y}^2},
$$

so $v_s$ is the magnitude of the in-plane translational relative velocity at the patch reference point, and the creepage-side floor cancels exactly. If instead $v_{\min}>V_{\min}$, the outer maximum becomes active over the range $|v_{\mathrm{ref}}|<v_{\min}$, where $v_s=v_{\min}\sqrt{\xi_x^2+\xi_y^2}$ no longer equals that translational relative-speed magnitude. The maximum factor has a derivative corner at $|v_{\mathrm{ref}}|=v_{\min}$ and generally passes it into $v_s$.

Given $v_s$, the coefficient is

$$
\mu(v_s)=\mu_0\left[(1-A)e^{-Bv_s}+A\right].
$$

$\mu(0)=\mu_0$. When $B>0$, $\mu\to A\mu_0$ as $v_s\to\infty$; if in addition $\mu_0>0$ and $0\leq A<1$, the curve decreases strictly with $v_s$ (for $A=1$ it degenerates to a constant). The physical reading of “falling friction” therefore assumes these parameter conditions; the formula itself does not enforce them. Spin creepage does not enter this patch-level friction law. The function has the same form as the falling-friction curve in the [Polach wheel-rail contact model](https://komunikacie.uniza.sk/artkey/csl-200101-0002_contact-of-wheel-and-rail-in-computer-simulation-of-vehicle-dynamics-and-axle-drive-dynamics.php); ORVD's discrete contact solution remains defined by the other equations in this chapter.

## 4. Strip and cell discretization

### 4.1 Strip layout

`LayStrips` lays strips over $v\in[-1,1]$. Three discrete quantities appear in this section: the number of longitudinal cells per strip $n_x$, the number of lateral strips $n_y$ and the refinement target width $w_\ast$, measured in units where the patch spans two. All three are quadrature resolutions rather than physical quantities: they fix the grid and do not enter the continuous model. Equal-width strips, $n_y$ of them, are used when the stress-build-up-rate zero does not exist or does not lie inside the unit disk. If

$$
u_p^2+v_p^2<1,
$$

the algorithm marches inward from both lateral edges and repeatedly halves strip width near $v_p$ until the width falls in $[w_\ast,2w_\ast]$. If the geometric-progression march cannot bring the narrowest strip into that interval, it switches to a fallback layout over the whole patch: one full-width strip is used when $w_\ast$ is at least the patch half-width; otherwise it uses nearly uniform strips and bisects the one containing $v_p$. Refinement changes only the lateral quadrature grid; it does not alter the continuous $r_x$, $r_y$ or the friction law.

Because $(u_p,v_p)$ depends on creepages, the lateral quadrature nodes also depend on state. The strict inside/outside branch at the unit circle and wholesale replacement by the fallback grid have no interpolation between layouts. A change in the strip set can therefore introduce either a derivative corner or a finite jump into the discrete resultant, with no uniform smallness bound on its magnitude.

### 4.2 Longitudinal march

For a strip centered at $v_j$ with normalized width $w_j$, let $h_j=\sqrt{1-v_j^2}$. The longitudinal step and cell area are

$$
\Delta u=\frac{2h_j}{n_x},
\qquad
\Delta A=a\Delta u\,b w_j.
$$

Material starts at $u=h_j$. The first transport step reaches the first cell center and is half a cell long; later steps travel one full cell between adjacent centers. Every cell nevertheless carries the full area weight $\Delta A$. The core recurrence is


```text
p0_F = 2 * N / (pi * a * b)
tau_x = 0; tau_y = 0
u_previous = h
u = h - 0.5 * delta_u
for each longitudinal cell:
    step = a * (u_previous - u)
    u_mid = 0.5 * (u_previous + u)
    r_x = xi_x / L_x - xi_sp * (b * v) / L_sp
    r_y = xi_y / L_y + xi_sp * (a * u_mid) / L_sp
    trial_x = tau_x - r_x * step
    trial_y = tau_y - r_y * step
    p = p0_F * max(0, 1 - v*v - u*u)
    (tau_x, tau_y) = radial_projection_to_radius_mu_p(trial_x, trial_y)
    F_x += tau_x * delta_A
    F_y += tau_y * delta_A
    u_previous = u
    u = u - delta_u
```

### 4.3 Small-creepage limit

With no spin and full adhesion over the patch, the continuous model returns to Kalker's linear force. Equal-width strip midpoint quadrature gives the finite-resolution factor

$$
F_x=-G\,a\,b\,C_{11}\xi_x\left(1+\frac{w^2}{8}\right),
\qquad
F_y=-G\,a\,b\,C_{22}\xi_y\left(1+\frac{w^2}{8}\right),
\qquad
w=\frac{2}{n_y}.
$$

As $n_y\to\infty$, the factor approaches $1$, recovering the continuous linear limit.

## 5. Approximations and non-smoothness

The principal approximations are: three local flexibilities replace full elastic coupling; paraboloid pressure supplies the local friction bound; one $\mu$ is evaluated from patch-level translational sliding speed and shared across the patch; midpoint strip-and-cell quadrature discretizes the patch; and the lateral strips adapt near the stress-build-up-rate zero.

The corresponding sources of non-smoothness include: the derivative change when a cell switches between adhesion and slip; the stress-build-up-rate zero crossing the unit disk or changing strip layout; the corner of the friction law's reference-speed floor when $v_{\min}>V_{\min}$, that maximum being an identity when $v_{\min}\le V_{\min}$ as shown in section 3.3; and the piecewise-linear Kalker nodes and table-asymptotic junctions. The non-smoothness of the creepages themselves at $V_0=0$ and $|V_0|=V_{\min}$ belongs to [Creepages and the contact frame](CREEPAGE_AND_CONTACT_FRAME.en.md) and reaches this chapter through $\xi_x$, $\xi_y$ and $\xi_{sp}$. Force is continuous across an individual adhesion-slip projection, but its derivative with respect to state is generally not.

The model outputs $F_x$ and $F_y$. It does not output a direct spin moment about the patch normal and does not solve nonlocal elastic coupling between strips. These are limits of the model, not omitted quantities that can be recovered from the discrete result.

## 6. Source mapping

| Theoretical object | Primary implementation |
|---|---|
| Friction law and patch-level coefficient | `FrictionLaw`, `FrictionCoefficientAt`; see [`tangential_contact_force.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/tangential_contact_force.h) and [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
| Flexibilities, build-up rates and cell march | `TangentialContactSolver::Solve`; see [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
| Stress-build-up-rate zero and strip layout | `spin_pole_longitudinal`, `spin_pole_lateral`, `TangentialContactSolver::LayStrips`; see [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
| Kalker coefficients | `KalkerCoefficientTable::At`; see [`kalker_coefficient_table.cc`](../../../libs/wheel_rail_contact/src/kalker_coefficient_table.cc) |
| Discrete resolutions and strip refinement | `TangentialContactConfiguration`, `TangentialContactSolver::LayStrips`; see [`tangential_contact_force.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/tangential_contact_force.h) and [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
| Input and output quantities | `TangentialContactPatch`, `TangentialContactResult`; see [`tangential_contact_force.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/tangential_contact_force.h) |
