[中文](CONTACT_GEOMETRY.md)

# Contact geometry

This document describes the geometric part of ORVD wheel-rail contact. Given the four pose scalars produced by pose reduction, `ContactGeometrySolver::Solve` returns a set of contact patches. Each patch contains position, penetration, transverse width, three-dimensional longitudinal length, local angles, wheel and rail curvatures and a rail material reference point. Forces, materials and loads are handled by subsequent models. The core implementation is in [`contact_geometry.cc`](../../../libs/wheel_rail_contact/src/contact_geometry.cc).

## 1. Scope

The model answers three questions in sequence:

1. What visible outline does a surface-of-revolution wheel present to the rail cross-section at the given yaw?
2. Where does that projected outline interpenetrate the rail, and how is the interpenetrating region partitioned into contact islands?
3. What are the geometric dimensions, centroid, normal, curvatures and longitudinal chord of each island?

The model does not compute pressure distributions, normal force or tangential force. Irregularities have already entered the relative placement through pose reduction. See [Wheel-rail pose reduction and irregularity inputs](WHEEL_RAIL_POSE_REDUCTION.en.md) for the input pose, [Normal contact force](NORMAL_CONTACT_FORCE.en.md) for normal loading and [Creepages and the contact frame](CREEPAGE_AND_CONTACT_FRAME.en.md) for the contact frame and creepages.

## 2. Notation

The profile transverse coordinate is $Y$, and the vertical direction is positive downward. Pose roll $\varphi$, pose yaw $\beta$, pose lateral offset $d_y$ and upward-positive vertical raise $d_z^{\uparrow}$ are the shared pose-scalar symbols and are not repeated in the table below. The decoded planar translation is written $t_y,t_z$ here, and it is not the same pair of quantities as the pose scalars $d_y,d_z^{\uparrow}$. The table below gives the remaining symbols used in this document.

| Symbol | Meaning | Implementation quantity |
|---|---|---|
| $\eta$ | Side-resolved transverse wheel-profile station | `wheel_station_meters` |
| $h(\eta)$ | Wheel height represented by a natural cubic spline | `wheel_spline_` |
| $\hat h(\eta)$ | Hermite wheel surface built from node values and natural-spline nodal slopes | `wheel_surface_` |
| $r_0$ | Nominal rolling radius | `nominal_rolling_radius_meters` |
| $r(\eta)$ | Local rolling radius $r_0+h(\eta)$ | `local_radius` |
| $\tau$ | Circumferential angle measured from directly below the axle | `angle` |
| $\tau_s(\eta)$ | Circumferential angle of the visible outline | `silhouette_angle` |
| $t_y,t_z$ | Decoded transverse and vertical translation of the wheel-profile datum relative to the rail-profile datum, in $T_c$ | `lateral_offset`, `vertical_offset` |
| $z_r(Y)$ | Rail cross-section height | `rail_surface_` |
| $H(Y)$ | Single-valued upper envelope of the projected wheel outline | cubic segments fixed by `envelope_vertical` and `envelope_vertical_slopes` |
| $\eta(Y)$ | Piecewise-linear map from envelope coordinate to wheel station | `envelope_station_` |
| $g(Y)$ | Vertical interpenetration: wheel envelope minus rail surface | `union_gap_` |
| $\epsilon$ | Gap threshold defining contact | `contact_gap_epsilon_meters` |
| $\delta_m$ | Valley-depth threshold for merging islands | `island_merge_gap_tolerance_meters` |
| $\delta_v,\delta_n$ | Vertical and normal penetration | `vertical_penetration_meters`, `normal_penetration_meters` |
| $Y_c,Z_c$ | Area centroid of the overlap region | `centroid_lateral_meters`, `centroid_vertical_meters` |
| $\alpha_r,\alpha,\gamma$ | Rail-surface angle, canted contact-frame angle and common-normal angle | $\alpha_r$ is internal to contact geometry; `rail_slope_angle_radians`, `common_normal_angle_radians` |
| $c_r$ | Cant magnitude carried by contact geometry itself | `rail_cant_radians` |
| $\varsigma$ | Side sign, $+1$ on the right and $-1$ on the left | mathematical encoding of `WheelSide`, opposite in sign to the internal `side_sign_` |
| $L$ | Longest three-dimensional longitudinal chord | `longitudinal_length_meters` |

## 3. Model

### 3.1 Surface interpolation, pose and projection

The authored wheel-profile point list is laid into a node set along one of two paths. When equal-arc-length rescanning is disabled, the authored points are resolved directly for the requested side: the abscissae take the side sign and are reordered. When rescanning is enabled, the authored points are first resolved to the physical right-hand side and the equal-arc-length stations are laid on that right-hand ordering. A right-side request retains those nodes directly; only a left-side request mirrors them by negating the abscissae and reversing the order. Anchoring the station phase on the physical right-hand ordering makes the two sides share one grid phase. The construction is given in [Profiles and interpolants](PROFILES_AND_INTERPOLANTS.en.md), section 3.5. Either path yields a node set ascending in that side's transverse station. Two related wheel representations with different responsibilities are built from these nodes:

- `wheel_spline_` is a natural cubic spline used for outline height and slope and for the three-dimensional longitudinal resolution.
- `wheel_surface_` is constructed through `FromNodalSlopes` with the same node values and the natural spline's nodal slopes. It supplies the shared surface-value, derivative and curvature interface. `FromNodalSlopes` uses the supplied slopes directly and performs no shape-preserving slope limiting.

Both representations hold the endpoint value constant outside the knot interval. The first derivative is zero strictly outside only, the interior one-sided slope being returned at a boundary knot, while the second derivative is zero at the boundary knots and outside them. That strict-versus-nonstrict asymmetry is a contract both share, and their coexistence is due to consumer and interface separation, not different extrapolation rules. If the natural spline has not replaced an approximately uniform point list by an ideal grid, so that the two representations actually use the same knot abscissae and interval lengths, their within-range Hermite cubic segments are identical because they share values and slopes. The natural spline can use an idealized grid $x_0+ih$ for a nearly uniform input node set, while `wheel_surface_` still receives the actual control nodes supplied to the solver; on that path the two representations are not guaranteed to agree point for point. The rail surface $z_r(Y)$ is built through the same `FromNodalSlopes` path from natural-spline nodal slopes and is not a shape-limited curve either.

In the wheel-profile datum frame, the surface of revolution is

$$
\mathbf p_w(\eta,\tau)=
\begin{bmatrix}
r(\eta)\sin\tau\\
\eta\\
r(\eta)\cos\tau-r_0
\end{bmatrix},
\qquad
r(\eta)=r_0+h(\eta).
$$

The encoded pose offsets are first decoded into planar translation:

$$
t_y=d_y\cos\varphi+d_z^{\uparrow}\sin\varphi,
\qquad
t_z=d_y\sin\varphi-d_z^{\uparrow}\cos\varphi.
$$

Here $t_y,t_z$ are the transverse and vertical translation of the wheel-profile datum relative to the rail-profile datum, expressed in the rail-cant frame $T_c$. They are not the pose scalars: at $\varphi=0$ one has $t_y=d_y$ and $t_z=-d_z^{\uparrow}$, at a general pose $t_y$ and $d_y$ also differ, and this document does not interchange them. The two-dimensional map is a self-inverse reflection with determinant $-1$. Let $x_w=r\sin\tau$ and $z_w=r\cos\tau-r_0$. Yaw and roll project the point into the cross-section of $T_c$ as

$$
\widetilde y=\sin\beta\,x_w+\cos\beta\,\eta,
$$

$$
Y=\cos\varphi\,\widetilde y-\sin\varphi\,z_w+t_y,
\qquad
Z=\sin\varphi\,\widetilde y+\cos\varphi\,z_w+t_z.
$$

### 3.2 Visible outline

Let $\chi(\eta)=-\tan\beta\,h'(\eta)$. When $|\chi|\leq1$, orthogonality of the surface-of-revolution normal to the track longitudinal direction gives $\sin\tau_s=\chi$. The implementation extends the same expression to every finite pose as

$$
\sin\tau_s(\eta)=
\operatorname{clamp}\!\left(-\tan\beta\,h'(\eta),-1,1\right),
$$

$$
\cos\tau_s=\sqrt{\max(0,1-\sin^2\tau_s)}.
$$

When $\beta=0$, $\tau_s=0$, and the visible outline is the wheel profile at its lowest circumferential position. Nonzero yaw moves sloped profile points circumferentially and changes both projected coordinates $Y$ and $Z$. If $|\chi|>1$, clamping sets $\sin\tau_s=\pm1$; this is a numerical continuation that selects a circumferential boundary direction, not a real root of the original normal-orthogonality equation.

Values $h(\eta_i)$ and slopes $h'(\eta_i)$ are evaluated in advance at uniformly spaced wheel stations. Both come from `wheel_spline_`, so the visible outline and the three-dimensional longitudinal resolver use the same natural-spline evaluation semantics. `wheel_surface_` serves the common surface and curvature calculations; there is no shape-preserving rule that flattens its supplied wheel slopes.

### 3.3 Binned upper envelope

The projected points $(Y_i,Z_i,\eta_i)$ may fold in $Y$ under yaw and roll and therefore need not define a single-valued function. The model bins them with width $w_b$:

$$
n_b=\left\lceil\frac{Y_{\max}-Y_{\min}}{w_b}\right\rceil+1,
$$

$$
k(Y)=
\min\!\left(
\left\lfloor\frac{Y-Y_{\min}}{w_b}\right\rfloor,
n_b-1
\right).
$$

Each bin retains only the point with greatest $Z$, namely the outermost branch capable of contacting the rail. The retained coordinates are the sample's own $(Y_i,Z_i,\eta_i)$, not the bin center. Thus $w_b$ is a competition scale for folded branches rather than a resampling step.

Shape-preserving cubic slopes are computed for the envelope nodes. Shape limiting applies only to this projected upper envelope $H(Y)$, not to `wheel_surface_` or `rail_surface_` from the preceding section. Adjacent envelope nodes are joined by a Hermite cubic. With

$$
t=\frac{Y-Y_j}{\Delta_j},\qquad \Delta_j=Y_{j+1}-Y_j,
$$

the segment is

$$
H(Y)=c_0+c_1t+c_2t^2+c_3t^3,
$$

$$
\begin{aligned}
c_0&=Z_j,&
c_1&=\Delta_jm_j,\\
c_2&=-3Z_j+3Z_{j+1}-2\Delta_jm_j-\Delta_jm_{j+1},\\
c_3&=2Z_j-2Z_{j+1}+\Delta_jm_j+\Delta_jm_{j+1}.
\end{aligned}
$$

The inverse station map from $Y$ to wheel station, $\eta(Y)$, is piecewise linear. Wheel station may jump where the projection folds; smoothing this map with a cubic would create intermediate stations occupied by no point on the selected wheel branch.

### 3.4 Interpenetration and contact islands

Over the common support of the wheel envelope and rail surface, their node abscissae are merged into an ordered union grid. Vertical interpenetration on that grid is

$$
g_i=H(Y_i)-z_r(Y_i).
$$

The strict condition $g_i>\epsilon$ defines contact. Maximal contiguous runs satisfying it form the raw contact islands; within the representable range, they are retained in transverse scan order.

The valley between two adjacent islands is

$$
v_k=\min_{e_k\leq i\leq s_{k+1}}g_i.
$$

The islands merge when $v_k>-\delta_m$. This criterion measures vertical valley depth rather than transverse distance. An island edge in the interior of the common support is obtained by one secant interpolation between adjacent grid points of opposite classification, that is, by the secant root of $g-\epsilon$:

$$
Y_e=Y_a+
\frac{(\epsilon-g_a)(Y_b-Y_a)}{g_b-g_a}.
$$

Since the root sought is that of $g=\epsilon$, the values $g_a,g_b$ on either side of an edge need not be of opposite sign when $\epsilon\neq0$. This single interpolation is consistent with the piecewise-linear sign model used while discovering islands. If an island reaches an endpoint of the common support, that endpoint itself is the corresponding edge and no secant interpolation is applied there.

### 3.5 Per-island quadrature

Each retained merged island has its own uniform quadrature grid on $[Y_L,Y_R]$. Define

$$
o(Y)=\max(0,H(Y)-z_r(Y)).
$$

Vertical penetration $\delta_v$ is the greatest sampled $o$, with the first maximizer selected. Composite trapezoidal quadrature over the same station sequence gives

$$
A=\int_{Y_L}^{Y_R}o\,dY,
\qquad
W_a=\int_{Y_L}^{Y_R}\sqrt{1+H'^2}\,dY,
$$

$$
A_a=\int_{Y_L}^{Y_R}o\sqrt{1+H'^2}\,dY,
\qquad
S_y=\int_{Y_L}^{Y_R}Yo\,dY,
$$

$$
S_z=\int_{Y_L}^{Y_R}
\frac{(z_r+o)^2-z_r^2}{2}\,dY.
$$

Hence

$$
Y_c=\frac{S_y}{A},
\qquad
Z_c=\frac{S_z}{A}.
$$

Here $A$ is cross-sectional overlap area, $W_a$ is wheel-envelope arc width, $A_a$ is arc-weighted overlap area, $S_y,S_z$ are the first area moments of the overlap region and $(Y_c,Z_c)$ is its area centroid. The island-discovery grid and the per-island quadrature grid discretize different questions, so the deepest quadrature point need not coincide with the maximum interpenetration on the union grid.

The quadrature grid carries a second threshold: a merged island with $\delta_v\leq\epsilon$ emits no contact patch, and neither does one whose secant-located width $Y_R-Y_L$ or whose quadrature area $A$ comes out nonpositive. Each retained merged island emits at most one patch, and the two are not in one-to-one correspondence: the quadrature grid is usually coarser than the union grid, so a narrow island can satisfy $g_i>\epsilon$ on the union grid and still miss at every quadrature station.

### 3.6 Normal penetration, angles and curvatures

Rail slope at the deepest quadrature station projects vertical penetration onto the local normal:

$$
\delta_n=
\frac{\delta_v}
{\sqrt{1+z_r'(Y_d)^2}}.
$$

Wheel station and local radius at the centroid are

$$
\eta_c=\eta(Y_c),
\qquad
r_c=r_0+\hat h(\eta_c).
$$

Define the rail-surface angle, common-normal angle and canted contact-frame angle by

$$
\alpha_r=\arctan z_r'(Y_c),
$$

$$
\gamma=
\operatorname{atan2}\!\left(
\cos\beta\,\sin(\alpha_r-\varphi),
\cos(\alpha_r-\varphi)
\right),
$$

$$
\alpha=\alpha_r-\varsigma c_r,
$$

where $\varsigma=+1$ denotes the right side and $\varsigma=-1$ the left. Here $c_r$ is the cant magnitude carried by contact geometry itself; it and the cant magnitude $\phi_c$ used by pose reduction are two separately given constants and should not be read as one quantity even where they agree numerically. Cant is not added again to $\gamma$ because the pose roll already contains the rail attitude. The angle $\alpha$ is used to construct the contact frame.

The patch longitudinal coordinate on the wheel is

$$
x_c=r_c\,
\operatorname{clamp}\!\left(-\tan\beta\tan\gamma,-1,1\right).
$$

Both profile curvatures use the planar-curve expression

$$
\kappa=\frac{z''}{(1+z'^2)^{3/2}}.
$$

Wheel curvature is evaluated on $\hat h$ at $\eta_c$, and rail curvature on $z_r$ at $Y_c$. They belong to their respective profile coordinates and should not be read as the same global curvature component.

### 3.7 Rail material reference point

Each patch carries a point on the undeformed rail surface at which subsequent stages form the two body velocities. Call that point `R`, with position vector $\mathbf x_R=(x_R,Y_R,Z_R)$. The wheel-side representative point is projected longitudinally and transversely into $T_c$, then dropped vertically onto the rail:

$$
x_R=\cos\beta\,x_c-\sin\beta\,\eta_c,
$$

$$
Y_R=
\cos\varphi(\sin\beta\,x_c+\cos\beta\,\eta_c)
-\sin\varphi(r_c-r_0)+t_y,
$$

$$
Z_R=z_r(Y_R).
$$

This is a rail reference point, not a wheel point or a pressure centroid, so its vertical coordinate is the undeformed rail-profile height.

### 3.8 Three-dimensional longitudinal length

The cross-sectional envelope removes the wheel's circumferential degree of freedom. At each quadrature station, the longitudinal resolver restores $\tau$ as a free variable and defines

$$
o(\eta,\tau)=Z(\eta,\tau)-z_r(Y(\eta,\tau)).
$$

A station enters the longitudinal resolution only if its local rolling radius $r(\eta)$ is finite and positive and the exact surface of revolution gives a finite, strictly positive $o(\eta,\tau_s)$ at the visible-outline angle. The cross-sectional envelope reads a shape-preserving fit of a projection while the longitudinal resolver reads the surface itself, so the two can disagree at the same station: where the envelope says the station penetrates and the exact surface of revolution says it does not, that station contributes no $\ell$ and does not enter the maximum defining $L$. This is also what makes the square root below well defined.

Starting from $\tau_s$, the resolver searches in both circumferential directions until each side brackets a root of $o=0$. The initial angular step combines a local circular estimate with a finite lower bound:

$$
\Delta_0=
\max\!\left(
\Delta_{\min},
\sqrt{\frac{2o(\eta,\tau_s)}{r(\eta)}}
\right),
$$

and is then doubled up to the prescribed angular-domain limit. Once both roots are bracketed, each is refined by bisection. The station chord is

$$
\ell(\eta)=
\left|
r(\eta)\cos\beta\,
(\sin\tau_a-\sin\tau_b)
\right|,
$$

and the patch longitudinal length is

$$
L=\max_\eta\ell(\eta).
$$

If no station yields a valid pair of root brackets, $L=0$. Its theoretical meaning is that the geometric longitudinal measurement is unavailable, not that the physical patch necessarily has zero length.

Bisection stops against an absolute longitudinal-chord error. Let $p=|r\cos\beta|$. Because sine is 1-Lipschitz,

$$
|x(\tau)-x(\tau')|\leq p|\tau-\tau'|.
$$

In exact arithmetic, if both true roots remain in their respective sign-changing brackets and refinement terminates through the $p\Delta\tau$ condition, taking the midpoints bounds their combined chord error by the prescribed resolution. The current model uses 20 nm as its absolute chord-length target. Each side also has a hard ceiling of 36 bisections, so 20 nm is not an unconditional error guarantee when that ceiling is reached first. This conditional bound uses the actual local radius at each station and does not depend on a particular wheel-profile curvature.

To reduce midpoint trigonometric evaluations, bisection can normalize the sum of the endpoint direction vectors with an even polynomial. Let

$$
h=\frac{\tau_{\mathrm{in}}-\tau_{\mathrm{out}}}{2},
$$

$$
P_8(h)=
\frac12\left[
1+h^2\left(
\frac12+h^2\left(
\frac5{24}+h^2\left(
\frac{61}{720}+\frac{1385}{40320}h^2
\right)\right)\right)\right]
\approx\frac1{2\cos h}.
$$

Then

$$
\sin\tau_m\approx
(\sin\tau_{\mathrm{out}}+\sin\tau_{\mathrm{in}})P_8(h),
$$

$$
\cos\tau_m\approx
(\cos\tau_{\mathrm{out}}+\cos\tau_{\mathrm{in}})P_8(h).
$$

The approximation has an explicit local error bound over the prescribed search width. When overlap is extremely close to zero it may change one bisection branch, so it is a budgeted numerical approximation rather than an algebraic identity.

For a station that has not yet been fully resolved, let $\tau_{\min}$ and $\tau_{\max}$ be the least and greatest angles among all endpoints of the two root brackets. Provided that each true root remains in its bracket, exact arithmetic gives the geometric competitive upper bound

$$
U=p(\tau_{\max}-\tau_{\min}).
$$

If $U$ is already no greater than the current longest chord, the station cannot change $L$ under the stated conditions and needs no further refinement. The implementation evaluates $U$ directly in ordinary binary64 without directed rounding; screening therefore belongs to the finite-precision algorithm, like the midpoint approximation, rather than constituting an interval-arithmetic proof.

## 4. Algorithm structure

`ContactGeometrySolver::Solve` proceeds in this order:

1. Decode the pose offsets and project the pre-sampled wheel-outline points.
2. Bin by transverse coordinate and retain the outermost sample in each bin, which yields the envelope nodes and the nodes of the piecewise-linear $\eta(Y)$ map.
3. Merge wheel-envelope and rail-profile nodes into an ordered union grid.
4. Compute shape-preserving cubic slopes for the envelope, then interpenetration on the union grid, and discover the raw islands that can be retained.
5. Merge adjacent islands by valley depth; locate interior edges by secant interpolation and use a common-support endpoint directly when an island reaches it.
6. Integrate each retained merged island to obtain area, widths, centroid, penetration and deepest station; an island that fails a threshold stops here and emits no patch.
7. Still inside the same per-island body, compute angles and local radius, then restore the circumferential degree of freedom and resolve the longest three-dimensional longitudinal chord, then compute the wheel and rail curvatures.
8. After the per-island loop, sort the patches by ascending centroid transverse coordinate, then form the rail material reference points in a separate pass.

For $n_s$ outline samples, $n_e$ envelope nodes, $n_r$ rail nodes, $n_i$ merged islands and $N_q$ quadrature stations per island, the main work consists of $O(n_s)$ projection and binning, $O(n_e+n_r)$ union-grid merging and $O(n_iN_q)$ quadrature and longitudinal resolution. Bisection depth depends on the length-error target and local projected radius.

## 5. Discrete approximations, non-smoothness and applicability

- The visible outline is sampled at finitely many wheel stations; profile features narrower than that scale can be missed.
- The exact normal-orthogonality form requires $|\tan\beta\,h'(\eta)|\leq1$; outside that domain, the model uses the clamped continuation $\sin\tau_s=\pm1$.
- Envelope binning uses $w_b$ as the transverse competition scale between projected branches; changing $w_b$ can change which folded branch survives.
- The union grid discovers islands, whereas a separate uniform grid approximates patch integrals. The two grids must not be treated as one discretization.
- The discrete solver retains at most 64 raw islands in scan order and emits at most 16 merged patches. The formulas in this chapter fully cover the model domain in which neither ceiling is reached. Beyond it, the algorithm merges only the retained 64-island prefix and forms at most 16 patches from it in scan order.
- The strict contact threshold, island-merge threshold, first-maximizer rule, bin winner and maximum longitudinal chord all introduce branch switching and non-smoothness.
- The map $\eta(Y)$ is allowed to jump where the projection folds; piecewise-linear interpolation preserves that geometric fact.
- Curvature requires meaningful local first and second derivatives and a nondegenerate curvature denominator. Near corners, near-vertical tangents or nonsmooth measured profiles make local curvature an unstable descriptor.
- Longitudinal resolution assumes that moving forward and backward from the visible outline finds one separation root on each side within a finite angular domain. $L=0$ denotes an unavailable measurement, after which the normal model uses its own analytic longitudinal scale.
- The model describes a patch as a cross-sectional overlap island combined with circumferential chords at sampled stations. It does not solve a three-dimensional elastic free-boundary problem; the normal and tangential models construct equivalent contact scales from this geometry.

## 6. Implementation mapping

| Theoretical object | Main implementation |
|---|---|
| Natural cubic spline values and derivatives | [natural_cubic_spline.cc](../../../libs/wheel_rail_contact/src/natural_cubic_spline.cc) |
| Shape-preserving cubic values, derivatives and curvature | [monotone_cubic_interpolant.cc](../../../libs/wheel_rail_contact/src/monotone_cubic_interpolant.cc) |
| Side resolution and equal-arc-length rescan of the wheel-profile nodes | [wheel_profile_preprocessing.cc](../../../libs/wheel_rail_contact/src/wheel_profile_preprocessing.cc) |
| Outline projection, envelope, islands and quadrature | `ContactGeometrySolver::Solve` in [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
| Longitudinal bracketing, bisection, screening and chord length | `ResolveLongitudinalLength`, called by `ContactGeometrySolver::Solve` in [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
| Geometric input pose | [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) |
| Normal consumption of geometric patches | [normal_contact_force.cc](../../../libs/wheel_rail_contact/src/normal_contact_force.cc) |
