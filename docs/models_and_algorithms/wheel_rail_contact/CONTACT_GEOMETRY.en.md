[中文](CONTACT_GEOMETRY.md)

# Contact Geometry

This document describes the geometric part of ORVD wheel–rail contact. Given the four pose scalars produced by pose reduction, `ContactGeometrySolver::Solve` returns a set of contact patches. Each patch contains position, penetration, transverse width, three-dimensional longitudinal length, local angles, wheel and rail curvatures, and a rail material reference point. Forces, materials, and loads are handled by subsequent models. The core implementation is in [`contact_geometry.cc`](../../../libs/wheel_rail_contact/src/contact_geometry.cc).

## 1. Scope

The model answers three questions in sequence:

1. What visible outline does a surface-of-revolution wheel present to the rail cross-section at the given yaw?
2. Where does that projected outline interpenetrate the rail, and how is the interpenetrating region partitioned into contact islands?
3. What are the geometric dimensions, centroid, normal, curvatures, and longitudinal chord of each island?

The model does not compute pressure distributions, normal force, or tangential force. Irregularities have already entered the relative placement through pose reduction. See [Wheel–Rail Pose Reduction and Irregularity Inputs](WHEEL_RAIL_POSE_REDUCTION.en.md) for the input pose, [Normal Contact Force](NORMAL_CONTACT_FORCE.en.md) for normal loading, and [Creepages and the Contact Frame](CREEPAGE_AND_CONTACT_FRAME.en.md) for the contact frame and creepages.

## 2. Notation

The profile transverse coordinate is $Y$, and the vertical direction is positive downward. To avoid confusion with track attitude, pose roll is denoted by $\varphi$ and pose yaw by $\beta$.

| Symbol | Meaning | Implementation quantity |
|---|---|---|
| $\eta$ | Side-resolved transverse wheel-profile station | wheel_station_meters |
| $h(\eta)$ | Wheel height represented by a natural cubic spline | wheel_spline_ |
| $\hat h(\eta)$ | Hermite wheel surface built from node values and natural-spline nodal slopes | wheel_surface_ |
| $R_0$ | Nominal rolling radius | nominal_rolling_radius_meters |
| $r(\eta)$ | Local wheel radius $R_0+h(\eta)$ | local_radius |
| $\tau$ | Circumferential angle measured from directly below the axle | angle |
| $\tau_s(\eta)$ | Circumferential angle of the visible outline | silhouette_angle |
| $u_y,u_z^\uparrow$ | Pose lateral offset and upward-positive vertical raise | ContactPoseScalars |
| $d_y,d_z$ | Planar translation of the wheel datum relative to the rail datum | lateral_offset, vertical_offset |
| $z_r(Y)$ | Rail cross-section height | rail_surface_ |
| $H(Y)$ | Single-valued upper envelope of the projected wheel outline | envelope cubic segments |
| $\eta(Y)$ | Piecewise-linear map from envelope coordinate to wheel station | envelope_station_ |
| $g(Y)$ | Vertical interpenetration: wheel envelope minus rail surface | union_gap_ |
| $\epsilon$ | Gap threshold defining contact | contact_gap_epsilon_meters |
| $\delta_m$ | Valley-depth threshold for merging islands | island_merge_gap_tolerance_meters |
| $\delta_v,\delta_n$ | Vertical and normal penetration | patch penetration fields |
| $Y_c,Z_c$ | Area centroid of the overlap region | centroid fields |
| $\alpha_r,\alpha_s,\gamma$ | Rail-surface angle, canted contact-frame angle, and common-normal angle | patch angle fields |
| $\varsigma$ | Side sign, $+1$ on the right and $-1$ on the left | mathematical encoding of `WheelSide` |
| $L$ | Longest three-dimensional longitudinal chord | longitudinal_length_meters |

## 3. Model

### 3.1 Surface interpolation, pose, and projection

Wheel-profile points are first resolved for the selected side and ordered by transverse station. Optional equal-arc-length resampling changes only the node set used thereafter. Two related wheel representations with different responsibilities are built from these nodes:

- `wheel_spline_` is a natural cubic spline used for outline height and slope and for the three-dimensional longitudinal resolution.
- `wheel_surface_` is constructed through `FromNodalSlopes` with the same node values and the natural spline's nodal slopes. It supplies the shared surface-value, derivative, and curvature interface. `FromNodalSlopes` uses the supplied slopes directly and performs no shape-preserving slope limiting.

Both representations hold the endpoint value constant outside the knot interval and return zero first and second derivatives strictly outside. Their coexistence is therefore due to consumer and interface separation, not different extrapolation rules. If the natural spline has not replaced an approximately uniform point list by an ideal grid, so that the two representations actually use the same knot abscissae and interval lengths, their within-range Hermite cubic segments are identical because they share values and slopes. The natural spline can use an idealized grid $x_0+ih$ for a nearly uniform input grid, while `wheel_surface_` receives the authored knots; on that path the two representations are not guaranteed to agree point for point. The rail surface $z_r(Y)$ is built through the same `FromNodalSlopes` path from natural-spline nodal slopes and is not a shape-limited curve either.

In the wheel-profile datum frame, the surface of revolution is

$$
\mathbf p_w(\eta,\tau)=
\begin{bmatrix}
r(\eta)\sin\tau\\
\eta\\
r(\eta)\cos\tau-R_0
\end{bmatrix},
\qquad
r(\eta)=R_0+h(\eta).
$$

The encoded pose offsets are first decoded into planar translation:

$$
d_y=u_y\cos\varphi+u_z^\uparrow\sin\varphi,
\qquad
d_z=u_y\sin\varphi-u_z^\uparrow\cos\varphi.
$$

This two-dimensional map is a self-inverse reflection with determinant $-1$. Let $x_w=r\sin\tau$ and $z_w=r\cos\tau-R_0$. Yaw and roll project the point into the rail-profile cross-section as

$$
\widetilde y=\sin\beta\,x_w+\cos\beta\,\eta,
$$

$$
Y=\cos\varphi\,\widetilde y-\sin\varphi\,z_w+d_y,
\qquad
Z=\sin\varphi\,\widetilde y+\cos\varphi\,z_w+d_z.
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

Each bin retains only the point with greatest $Z$, namely the outermost branch capable of contacting the rail. The retained coordinates are the sample's own $(Y_i,Z_i,\eta_i)$, not the bin centre. Thus $w_b$ is a competition scale for folded branches rather than a resampling step.

Shape-preserving cubic slopes are computed for the envelope nodes. Shape limiting applies only to this projected upper envelope $H(Y)$, not to wheel_surface_ or rail_surface_ from the preceding section. Adjacent envelope nodes are joined by a Hermite cubic. With

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

The islands merge when $v_k>-\delta_m$. This criterion measures vertical valley depth rather than transverse distance. A merged island edge is obtained by one secant interpolation between adjacent grid values of opposite classification:

$$
Y_e=Y_a+
\frac{(\epsilon-g_a)(Y_b-Y_a)}{g_b-g_a}.
$$

The midpoint is used when the denominator vanishes. This single interpolation is consistent with the piecewise-linear sign model used while discovering islands.

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
M_y=\int_{Y_L}^{Y_R}Yo\,dY,
$$

$$
M_z=\int_{Y_L}^{Y_R}
\frac{(z_r+o)^2-z_r^2}{2}\,dY.
$$

Hence

$$
Y_c=\frac{M_y}{A},
\qquad
Z_c=\frac{M_z}{A}.
$$

Here $A$ is cross-sectional overlap area, $W_a$ is wheel-envelope arc width, $A_a$ is arc-weighted overlap area, and $(Y_c,Z_c)$ is the overlap region's area centroid. The island-discovery grid and the per-island quadrature grid discretize different questions, so the deepest quadrature point need not coincide with the maximum interpenetration on the union grid.

### 3.6 Normal penetration, angles, and curvatures

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
r_c=R_0+\hat h(\eta_c).
$$

Define the rail-surface angle, common-normal angle, and canted contact-frame angle by

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
\alpha_s=\alpha_r-\varsigma c_r,
$$

where $\varsigma=+1$ denotes the right side and $\varsigma=-1$ the left, and $c_r$ is the cant magnitude used by geometry. Cant is not added again to $\gamma$ because the pose roll already contains the rail attitude. The angle $\alpha_s$ is used to construct the contact-force frame.

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

Each patch carries a point on the undeformed rail surface at which subsequent stages form the two body velocities. The wheel-side representative point is projected longitudinally and transversely into the rail frame, then dropped vertically onto the rail:

$$
x_R=\cos\beta\,x_c-\sin\beta\,\eta_c,
$$

$$
Y_R=
\cos\varphi(\sin\beta\,x_c+\cos\beta\,\eta_c)
-\sin\varphi(r_c-R_0)+d_y,
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

Starting from the visible-outline angle $\tau_s$, it searches in both circumferential directions until each side brackets a root of $o=0$. The initial angular step combines a local circular estimate with a finite lower bound:

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

Provided that the true roots remain bracketed, exact arithmetic gives the following geometric competitive upper bound for a station that has not yet been fully resolved:

$$
U=p(\tau_{\max}-\tau_{\min}).
$$

If $U$ is already no greater than the current longest chord, the station cannot change $L$ under the stated conditions and needs no further refinement. The implementation evaluates $U$ directly in ordinary binary64 without directed rounding; screening therefore belongs to the finite-precision algorithm, like the midpoint approximation, rather than constituting an interval-arithmetic proof.

## 4. Algorithm structure

`ContactGeometrySolver::Solve` proceeds in this order:

1. Decode the pose offsets and project the pre-sampled wheel-outline points.
2. Bin by transverse coordinate and retain the outermost sample in each bin.
3. Construct shape-preserving cubic segments for the envelope and the piecewise-linear $\eta(Y)$ map.
4. Merge wheel-envelope and rail-profile nodes, compute interpenetration on the union grid, and discover the raw islands that can be retained.
5. Merge adjacent islands by valley depth and locate island edges with secant interpolation.
6. Integrate each retained merged island to obtain area, widths, centroid, penetration, and deepest station.
7. Compute angles, local radius, wheel and rail curvatures, and the rail material reference point.
8. Restore the circumferential degree of freedom and resolve the longest three-dimensional longitudinal chord.

For $n_s$ outline samples, $n_e$ envelope nodes, $n_r$ rail nodes, $n_i$ merged islands, and $N_q$ quadrature stations per island, the main work consists of $O(n_s)$ projection and binning, $O(n_e+n_r)$ union-grid merging, and $O(n_iN_q)$ quadrature and longitudinal resolution. Bisection depth depends on the length-error target and local projected radius.

## 5. Discrete approximations, non-smoothness, and applicability

- The visible outline is sampled at finitely many wheel stations; profile features narrower than that scale can be missed.
- The exact normal-orthogonality form requires $|\tan\beta\,h'(\eta)|\leq1$; outside that domain, the model uses the clamped continuation $\sin\tau_s=\pm1$.
- Envelope binning uses $w_b$ as the transverse competition scale between projected branches; changing $w_b$ can change which folded branch survives.
- The union grid discovers islands, whereas a separate uniform grid approximates patch integrals. The two grids must not be treated as one discretization.
- The discrete solver retains at most 64 raw islands in scan order and emits at most the first 16 merged patches. The formulas in this chapter fully cover the model domain in which neither ceiling is reached. Beyond it, the algorithm merges only the retained 64-island prefix and forms at most 16 patches from the resulting merged islands.
- The strict contact threshold, island-merge threshold, first-maximizer rule, bin winner, and maximum longitudinal chord all introduce branch switching and non-smoothness.
- The map $\eta(Y)$ is allowed to jump where the projection folds; piecewise-linear interpolation preserves that geometric fact.
- Curvature requires meaningful local first and second derivatives and a nondegenerate curvature denominator. Near corners, near-vertical tangents, or nonsmooth measured profiles make local curvature an unstable descriptor.
- Longitudinal resolution assumes that moving forward and backward from the visible outline finds one separation root on each side within a finite angular domain. $L=0$ denotes an unavailable measurement, after which the normal model uses its own analytic longitudinal scale.
- The model describes a patch as a cross-sectional overlap island combined with circumferential chords at sampled stations. It does not solve a three-dimensional elastic free-boundary problem; the normal and tangential models construct equivalent contact scales from this geometry.

## 6. Implementation mapping

| Theoretical object | Main implementation |
|---|---|
| Wheel and rail profile values and derivatives | [natural_cubic_spline.cc](../../../libs/wheel_rail_contact/src/natural_cubic_spline.cc), [monotone_cubic_interpolant.cc](../../../libs/wheel_rail_contact/src/monotone_cubic_interpolant.cc) |
| Outline projection, envelope, islands, and quadrature | [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
| Longitudinal bracketing, bisection, screening, and chord length | ResolveLongitudinalLength in [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
| Geometric input pose | [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) |
| Normal consumption of geometric patches | [normal_contact_force.cc](../../../libs/wheel_rail_contact/src/normal_contact_force.cc) |
