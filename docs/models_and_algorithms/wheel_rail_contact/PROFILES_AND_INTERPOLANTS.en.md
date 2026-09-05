[中文](PROFILES_AND_INTERPOLANTS.md)

# Profiles and interpolants

This document describes the mathematical steps by which the ORVD wheel-rail contact layer turns a measured profile into an evaluable surface: side resolution of the point list, two piecewise-cubic interpolants, profile arc-length quadrature and inversion, equal-arc resampling of a wheel profile, and a gauge datum derived from rail-profile geometry. Each formula either corresponds directly to a source computation or states its mathematical definition; a compact table at the end connects the theoretical objects to their implementation sites.

## 1. Scope

This document covers the six implementation units in `orvd::wheel_rail_contact` that carry those mathematical objects: `ProfilePoints` and `SideResolvedProfile`, `NaturalCubicSpline`, `MonotoneCubicInterpolant`, the profile arc-length algorithms, `WheelProfilePreprocessing`, and `ComputeRailGaugeDatum`. It ends with the three kinds of quantities consumed by contact geometry: wheel control nodes, curve representations built on the nodes, and one gauge datum per rail. File formats, configuration fields and interface catalogues are outside the scope.

The following are explicitly out of scope. How contact geometry projects the two surfaces, bins them, takes the upper envelope and cuts contact islands and patches belongs to [`CONTACT_GEOMETRY.en.md`](CONTACT_GEOMETRY.en.md); how wheelset-to-rail pose reduction consumes the gauge datum belongs to [`WHEEL_RAIL_POSE_REDUCTION.en.md`](WHEEL_RAIL_POSE_REDUCTION.en.md); normal force, creepages, Kalker coefficients and FASTSIM each have their own document. Interpolation of track-irregularity sequences is not a profile matter; see [`TRACK_IRREGULARITY_SPECTRA.md`](../track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.md).

The theoretical data flow in the implementation is as follows. The authored points are first resolved to a physical side. Wheel points may be resampled at equal arc length along a first natural spline, while rail points are side-resolved directly. Each node set forms a `NaturalCubicSpline`, whose nodal slopes in turn form a `MonotoneCubicInterpolant`. Independently, the authored rail polyline is rolled and intersected at the specified measuring depth to obtain `RailGaugeDatum`. Contact geometry then consumes these derived representations rather than querying the authored point list directly.

## 2. Notation

Frames, wheel sides and signs follow section 2.5 of [Conventions and Notation](../CONVENTIONS_AND_NOTATION.en.md): profile coordinates are metres in the profile's own frame, lateral across the track, vertical positive downward; `WheelSide::kRight` keeps the authored sign and `WheelSide::kLeft` is the mirror image. The notation added by this document is listed below.

| Symbol | Meaning | Code |
|---|---|---|
| $y$, $z$ | lateral and vertical profile coordinates, in m, vertical positive downward | `lateral_meters`, `vertical_meters` |
| $n$, $y_0\lt y_1\lt\cdots\lt y_{n-1}$, $z_i$ | number of knots, the strictly increasing knots and their values | `knots`, `values` |
| $h_i=y_{i+1}-y_i$ | width of the $i$-th knot interval | `spacings` |
| $\delta_i=(z_{i+1}-z_i)/h_i$ | secant slope over the $i$-th interval | `secants` |
| $m_i$ | nodal first derivative, the nodal slope | `nodal_slopes` |
| $M_i$ | nodal second derivative, the moment | `moments` |
| $t=(y-y_i)/h_i$ | local parameter inside a segment | `local_parameter` |
| $\varsigma$ | side sign, $+1$ on the right and $-1$ on the left | `sign` in `ResolveForSide` |
| $\sigma(a,b)$, $\Sigma_j$, $L$ | arc length over an interval, cumulative arc length at each knot, total arc length | `IntegrateProfileArcLength`, `cumulative`, `total` |
| $\Delta\sigma$ | equal arc-length rescan step | `equal_arc_length_rescan_step_meters` |
| $G$, $d$, $\gamma$, $\rho$ | track gauge, gauge measuring depth, laying-cant magnitude, side-signed rail roll | `track_gauge_meters`, `gauge_measuring_depth_meters`, `rail_cant_radians`, `roll_radians` |

## 3. Model

### 3.1 Point list, role and side resolution

A profile is the point list. `ProfilePoints` retains the authored order and signs together with a role `ProfileRole` (`kWheel` or `kRail`) and an identifier. The role selects the geometric operations that apply: equal-arc preprocessing is defined for a wheel profile, the gauge datum for a rail profile, and contact geometry distinguishes wheel from rail surfaces. A profile itself carries no station, side or vehicle.

A usable list has equal column lengths, at least two finite points, a non-empty identifier, and a lateral coordinate that is strictly monotone in authored order, either ascending or descending. A list that doubles back cannot first be sorted and then interpreted, because sorting changes the original geometric polyline into a different surface; repeated abscissae also make $z(y)$ multivalued. This prerequisite means the model cannot express an undercut shape (Section 6).

`ResolveForSide(side)` produces a `SideResolvedProfile`: the same points sorted ascending in the signed lateral coordinate. Writing $\pi$ for the permutation that orders $\varsigma\,y$ ascending,

$$
y^{(\varsigma)}_i=\varsigma\,y_{\pi(i)},\qquad z^{(\varsigma)}_i=z_{\pi(i)},\qquad \varsigma=\begin{cases}+1,&\texttt{kRight}\\ -1,&\texttt{kLeft}\end{cases}
$$

which is `sign * lateral_meters_[order[index]]` in `ResolveForSide`. The right side can only change the order and never a sign; the left side negates the lateral coordinate and then sorts ascending. The track frame's lateral axis pointing right is the basis of the convention. Every interpolant in this module is built on a side-resolved list and never on the authored list, because an interpolant needs a strictly increasing abscissa and the authored list is not required to have one.

Wheel preprocessing rescans only on the physical right side and mirrors the result to the left (Section 3.5). The reason is grid phase: an equal-arc rescan starts at the first point and places nodes at integer multiples of the step, leaving the indivisible remainder in the final interval. Rescanning an already mirrored left list independently would move that remainder to the other end, so the two sides would no longer share one grid phase.

### 3.2 Natural cubic spline

`NaturalCubicSpline` is the base representation of a profile surface. Each segment is a cubic in the local parameter $t=(y-y_i)/h_i$, stored in scaled Hermite form:

$$
p_i(t)=c_{0,i}+c_{1,i}\,t+c_{2,i}\,t^{2}+c_{3,i}\,t^{3},\qquad t=\frac{y-y_i}{h_i}\in[0,1]
$$

$$
\begin{aligned}
c_{0,i}&=z_i,\\
c_{1,i}&=h_i\,m_i,\\
c_{2,i}&=-3z_i+3z_{i+1}-2h_i\,m_i-h_i\,m_{i+1},\\
c_{3,i}&=2z_i-2z_{i+1}+h_i\,m_i+h_i\,m_{i+1}
\end{aligned}
$$

The four coefficients are, in order, `constant`, `linear`, `quadratic` and `cubic` of `SegmentCoefficients`. The first and second derivatives come from the same coefficients, in `EvaluateFirstDerivative` and `EvaluateSecondDerivative`:

$$
z'(y)=\frac{1}{h_i}\left(c_{1,i}+2c_{2,i}\,t+3c_{3,i}\,t^{2}\right),\qquad
z''(y)=\frac{1}{h_i^{2}}\left(2c_{2,i}+6c_{3,i}\,t\right)
$$

The nodal slopes $m_i$ come from one of two construction paths. On strictly equally spaced data the two are analytically equivalent. If data merely fall within the finite-precision uniformity criterion below, however, the uniform path still substitutes $h_0$ for every actual spacing and the two paths no longer solve exactly the same interpolation problem. `SpacingsAreUniform` selects the path from the data:

$$
\left|h_i-h_0\right|\le\tau_{\mathrm{abs}}+\tau_{\mathrm{rel}}\,|h_0|\qquad\forall\,i
$$

where $\tau_{\mathrm{abs}}$ and $\tau_{\mathrm{rel}}$ are the absolute and relative tolerances. A grid that passes is constructed and located on the idealised grid $y_0+i\,h_0$. The difference between actual $y_i$ and idealised nodes is therefore a modelling approximation of this fast path, not merely a storage detail.

The uniform path (`SolveNodalSlopesUniform`) solves a spacing-independent tridiagonal system directly for the slopes; the natural boundary condition is expressed in slopes and no moment ever appears:

$$
\begin{aligned}
2m_0+m_1&=\frac{3}{h}\,(z_1-z_0),\\
m_{i-1}+4m_i+m_{i+1}&=\frac{3}{h}\,(z_{i+1}-z_{i-1}),\qquad 1\le i\le n-2,\\
m_{n-2}+2m_{n-1}&=\frac{3}{h}\,(z_{n-1}-z_{n-2})
\end{aligned}
$$

For $n=2$ both end slopes are the secant. The diagonal does not depend on spacing, avoiding direct placement of a very small $h$ on the diagonal of the moment system.

The general path (`SolveNodalSlopesGeneral`) solves the classical moment system over the interior nodes; the two end moments are left out of the system rather than added as rows, so the natural boundary condition holds exactly instead of approximately:

$$
h_{i-1}M_{i-1}+2\,(h_{i-1}+h_i)\,M_i+h_iM_{i+1}=6\,(\delta_i-\delta_{i-1}),\qquad 1\le i\le n-2,\qquad M_0=M_{n-1}=0
$$

The slopes the segment form needs are then derived from the moments:

$$
m_i=\delta_i-\frac{h_i}{6}\,(2M_i+M_{i+1}),\quad 0\le i\le n-2,\qquad
m_{n-1}=\delta_{n-2}+\frac{h_{n-2}}{6}\,(M_{n-2}+2M_{n-1})
$$

The behaviour outside the knot range is a deliberate choice and not polynomial continuation. A profile has no meaning beyond its last measured point, and a cubic continued past the flange root produces a surface that looks plausible and is not there. Hence:

$$
z(y)=\begin{cases}z_0,&y\le y_0\\ z_{n-1},&y\ge y_{n-1}\end{cases},\qquad
z'(y)=0\quad(y\lt y_0\ \lor\ y\gt y_{n-1}),\qquad
z''(y)=0\quad(y\le y_0\ \lor\ y\ge y_{n-1})
$$

The value is held flat outside; the first derivative is zero only strictly outside and takes the one-sided interior slope at a boundary knot itself, so it has a step at an end when that slope is non-zero; the second derivative is exactly zero at and outside both boundary knots, which makes the natural boundary condition an exact value rather than rounding noise. The strict-versus-inclusive difference among the three tests is the implementation's contract, not an oversight.

`nodal_slopes()` provides the slopes obtained during construction so that another Hermite representation may reuse them on the same nodes. Knot values, knot slopes and interval widths together uniquely determine each cubic segment. Section 3.3 distinguishes the equality of these ideal piecewise polynomials from pointwise equality of their finite-precision public evaluators.

### 3.3 Shape-preserving cubic interpolant

`MonotoneCubicInterpolant` uses the same scaled Hermite segment form as the natural cubic spline (the four coefficient formulas of Section 3.2 are identical); the differences lie in the source of nodal slopes and the rule for derivatives outside the knots. When constructed by the limited-slope rule of `FromValues`, it is generally $C^1$ rather than $C^2$, but every monotone data interval remains monotone and stays within the interval spanned by its endpoint values. The same conclusion holds for `FromNodalSlopes` only when the supplied slopes already satisfy the corresponding monotone-Hermite conditions; arbitrary finite slopes may overshoot. This trade-off targets the nearly discontinuous change in slope around a flange root: a $C^2$ interpolant may respond with a ripple small in value but large in derivative, while the derivative directly sets the contact angle.

`FromValues` derives the nodal slopes from the values themselves, with the rule `ComputeShapePreservingNodalSlopes`. At an interior node:

$$
m_i=\begin{cases}
0,&\delta_{i-1}\,\delta_i\le 0\\
\dfrac{w_{\mathrm L}+w_{\mathrm R}}{\dfrac{w_{\mathrm L}}{\delta_{i-1}}+\dfrac{w_{\mathrm R}}{\delta_i}},&\text{otherwise}
\end{cases},\qquad
w_{\mathrm L}=2h_i+h_{i-1},\quad w_{\mathrm R}=h_i+2h_{i-1}
$$

Where the two neighbouring secants disagree in sign or either is zero the slope is set to zero, which is what keeps a flat run flat and a monotone run from reversing; otherwise it is a weighted harmonic mean, corresponding to `left_weight`, `right_weight` and `(left_weight + right_weight) / (left_weight / left_secant + right_weight / right_secant)` in the source. The source comment stresses that the doubled interval on each weight is the one on the far side of the secant that weight divides, and that pairing them the other way round is invisible on an equally spaced grid and wrong everywhere else. The endpoints use the one-sided three-point formula limited twice, in `EndpointSlope`:

$$
\tilde m_0=\frac{(2h_0+h_1)\,\delta_0-h_0\,\delta_1}{h_0+h_1},\qquad
m_0=\begin{cases}0,&\tilde m_0\,\delta_0\le 0\\ 3\delta_0,&\delta_0\,\delta_1\lt 0\ \land\ |\tilde m_0|\gt 3|\delta_0|\\ \tilde m_0,&\text{otherwise}\end{cases}
$$

The right end applies the same rule with $(h_{n-2},h_{n-3},\delta_{n-2},\delta_{n-3})$; for $n=2$ both end slopes are the secant. The second limit is what keeps an end segment from overshooting the way an unlimited one-sided formula does. The source gives this slope rule no method name and this document adds none.

`FromNodalSlopes` uses the supplied slopes directly, so those slopes determine whether the result is shape preserving. In exact arithmetic, if donor and recipient use the same knots, values and interval widths, the two endpoint values and slopes uniquely determine the same Hermite cubic on every interval: their **ideal piecewise polynomials** are identical. When the donor natural spline uses the uniform path, its segments use the ideal width $h_0$, while `MonotoneCubicInterpolant` uses the actual $h_i$, and even the segment polynomials generally differ.

Even when the ideal piecewise polynomials are identical, the two public evaluators need not agree bit for bit at every floating-point input. `NaturalCubicSpline::Locate` snaps a local parameter within `kLocalParameterSnap` of a knot onto that knot, whereas `MonotoneCubicInterpolant::Locate` only clamps the parameter to $[0,1]$. They can therefore use different $t$ values near knots, and the right-end evaluation paths also differ. `SurfaceThroughNodes` in contact geometry still deliberately builds a `NaturalCubicSpline` and then a `MonotoneCubicInterpolant` from `spline.nodal_slopes()` with `OutsideDerivativeRule::kZeroOutsideKnots`: the latter unconditionally shares the former's nodal derivatives; only when their actual knots and interval widths also agree do they share ideal segment shapes; in any case the two representations are not the same pointwise finite-precision function. `wheel_spline_` supplies profile values and slopes, while `wheel_surface_` supplies local radius and curvature at contact.

There are two rules for derivatives outside the knots, chosen by the construction parameter `OutsideDerivativeRule`. The default `kHoldEndpointDerivative` holds the endpoint derivative outside, deliberately inconsistent with the flat value, which is the reading a consumer wants when it is extrapolating a surface it knows continues; `kZeroOutsideKnots` gives the consistent reading: the first derivative is zero strictly outside and the second derivative is zero at and outside the boundary knots. One test is strict and the other inclusive, the same asymmetry the natural spline makes and for the same reason. The value is held flat under both rules.

### 3.4 Profile arc length

The profile curve is the graph of the spline $z(y)$, and its arc length is

$$
\sigma(a,b)=\int_a^b\sqrt{1+z'(y)^2}\,dy
$$

The integrand is smooth inside a spline segment and loses higher-order smoothness across a knot, which decides the whole design: the quadrature rule is applied once per knot interval and never across one. A single rule spread over the whole profile would lose most of its accuracy at every knot, and, more to the point, the cumulative table and the station search must integrate the same way or the stations they agree on drift apart. `IntegrateProfileArcLength(profile, from, to)` applies a sixteen-point Gauss–Legendre rule over one interval; the rule is symmetric, and each of the eight positive nodes is evaluated once on each side of the interval midpoint:

$$
\sigma(a,b)\approx\eta\sum_{k=1}^{8}w_k\left[\sqrt{1+z'(c-\eta\,\xi_k)^2}+\sqrt{1+z'(c+\eta\,\xi_k)^2}\right],\qquad c=\frac{a+b}{2},\quad \eta=\frac{b-a}{2}
$$

which is `half_width * sum`, each term formed with `std::hypot(1.0, profile.EvaluateFirstDerivative(...))`. An empty or reversed interval (`!(to > from)`) returns zero; the function does not know where the knots are, and not spanning one is the caller's responsibility. The cumulative table is formed interval by interval in `AccumulateProfileArcLength`, $\Sigma_0=0$, $\Sigma_{j+1}=\Sigma_j+\sigma(y_j,y_{j+1})$. The inverse question, the lateral coordinate at which the arc length from the first knot reaches $\sigma^\star$, is answered by `FindLateralCoordinateAtArcLength`, which brackets the answer inside one knot interval and halves that bracket a fixed number of times (section 4.4).

### 3.5 Wheel profile preprocessing

`WheelProfilePreprocessing` turns an authored wheel profile into the outline the contact geometry samples. A zero step preserves the authored nodes; a positive step replaces them with nodes laid at constant arc length along a first natural cubic spline and then builds a second natural cubic spline through the replacement nodes. Where the profile turns sharply the equal-arc nodes crowd together, so the outline resolves the flange root at the same curve-length density as the tread instead of at the density the asset author happened to write.

The rescan always runs on the physical right-hand ordering and the mirror is applied to its result; running it on an already-mirrored list would move the remainder interval, and with it the phase of the whole grid, to the other end of the left profile (section 3.1). The left nodes are the right nodes reversed with the lateral coordinate negated:

$$
y^{\mathrm L}_j=-\,y^{\mathrm R}_{N-1-j},\qquad z^{\mathrm L}_j=z^{\mathrm R}_{N-1-j},\qquad 0\le j\le N-1
$$

The target arc lengths are integer multiples of the step, $\sigma_k=k\,\Delta\sigma$, $k=1,2,\dots$; a node is laid as long as $\sigma_k\lt L-\tau_{\mathrm{end}}$, where $\tau_{\mathrm{end}}$ is the termination tolerance from `RescanEndTolerance`. The first and last nodes copy the physical endpoints without participating in arc-length inversion, thereby preserving the support endpoints declared by the asset; the spline's public evaluator also returns the stored knot values directly at both ends.

A second natural spline is constructed through the resampled nodes and becomes the continuous wheel outline consumed by contact geometry. It passes through the resampled nodes but generally not through the authored interior nodes. Equal-arc resampling is therefore a rediscretisation followed by reinterpolation, not a lossless reparameterisation of the first spline.

### 3.6 Rail gauge datum

A track gauge is measured between gauge faces at a stated depth below the rail crowns, not between the crowns. The lateral position of a rail-profile origin is therefore not simply half the gauge but half the gauge plus the distance from the gauge face to that origin. This distance depends jointly on rail shape, measuring depth and laying cant.

The side-signed roll is $\rho=-\varsigma\,\gamma$: under a positive cant the right rail leans toward the track centre, which with the lateral axis pointing right and the vertical axis pointing down is a negative roll about the forward axis, and the left rail is its mirror. The authored points are first rolled about the origin:

$$
\begin{bmatrix}y'_i\\ z'_i\end{bmatrix}=\begin{bmatrix}\cos\rho&-\sin\rho\\ \sin\rho&\cos\rho\end{bmatrix}\begin{bmatrix}y_i\\ z_i\end{bmatrix}
$$

which are `rolled_lateral` and `rolled_vertical`. The crown is the least rolled vertical coordinate (the vertical axis points down), and the measuring level is $z_{\mathrm{level}}=\min_i z'_i+d$. The construction is deliberately piecewise linear on the profile polyline and not on any interpolant built over it: the gauge face is a measurement convention applied to measured points, and interpolating it would make the answer depend on which interpolant happened to be selected. The points are sorted by rolled lateral coordinate, and for each neighbouring pair $(a,b)$ with $(z'_a-z_{\mathrm{level}})(z'_b-z_{\mathrm{level}})\le 0$ one crossing is recorded:

$$
y_\times=\begin{cases}\dfrac{y'_a+y'_b}{2},&z'_a=z'_b\\ y'_a+\dfrac{(z_{\mathrm{level}}-z'_a)\,(y'_b-y'_a)}{z'_b-z'_a},&\text{otherwise}\end{cases}
$$

A segment lying exactly on the measuring level contributes its midpoint. The gauge face is the crossing facing the track centre: the right rail takes the least crossing coordinate and the left rail the greatest. The three outputs are

$$
\text{offset}=-\varsigma\,y_{\mathrm{face}},\qquad
y_{\mathrm{datum}}=\varsigma\left(\frac{G}{2}+\text{offset}\right),\qquad
\rho=-\varsigma\,\gamma
$$

namely `gauge_face_offset_meters`, `lateral_datum_meters` and `roll_radians`. The construction assumes $G>0$, $d>0$, and at least one intersection between the rolled polyline and the measuring level. The two rolls always satisfy $\rho_{\mathrm L}=-\rho_{\mathrm R}$, but equal left/right offsets and opposite lateral datums are not invariants of an arbitrary authored profile. They follow only when the authored rail is mirror-symmetric about $y=0$—or, more generally, when the selected gauge-face intersections under opposite rolls obey the corresponding mirror relation:

$$
\operatorname{offset}_{\mathrm L}=\operatorname{offset}_{\mathrm R},\qquad
y_{\mathrm{datum,L}}=-y_{\mathrm{datum,R}}.
$$

## 4. Algorithm

### 4.1 Side resolution

The mathematical prerequisites of an authored point list are equal column lengths, at least two finite points, and a lateral coordinate that is strictly monotone in authored order. `ResolveForSide` builds an index array, sorts it by the key $\varsigma y$, and reads the points in that order. Pairwise-distinct lateral coordinates make the result unique. The process is $O(n\log n)$ and produces strictly increasing abscissae for every downstream interpolant.

### 4.2 Spline construction and evaluation

Spline construction forms positive spacings $h_i$, detects uniformity, solves for nodal slopes and forms the segment coefficients. The general-path `SolveTridiagonal` uses Thomas elimination without pivoting and a fixed pivot threshold to exclude numerically singular elimination. Because the threshold is absolute, a general grid with extremely small spacing can be affected while the dimensionless diagonal system of the uniform path is not scaled in the same way. The uniform system has size $n$, the general moment system size $n-2$; for $n=2$ no system is required. Both construction paths are $O(n)$.

`Locate` maps an abscissa to a segment and local parameter. The uniform path takes `std::floor` of $(y-y_0)/h_0$ in $O(1)$; the general path uses `std::upper_bound` over the knots in $O(\log n)$. It then snaps $t\le$ `kLocalParameterSnap` to the current segment start $t=0$ and $t\ge1-$ `kLocalParameterSnap` to the next segment start (or to $t=1$ on the final segment). On the general path this stabilises the tabulated value at a knot. Uniform-path location uses the idealised grid, so the same conclusion holds at an actual input knot only when its local displacement from the ideal knot is within the snap threshold. Values are held flat outside the domain; the first derivative is zero strictly outside but takes the one-sided interior slope at a boundary; the second derivative is zero at and outside both boundary knots.

The flat continuation may create non-smoothness at the two boundary knots: when an interior one-sided slope is non-zero, the first derivative jumps to zero outside that boundary; when the slope is zero, the first derivative is continuous. Returning zero for the second derivative at the boundary agrees with the natural boundary condition; the general path sets $M_0=M_{n-1}=0$ directly, and the two boundary equations of the uniform path are equivalent in exact arithmetic. The two construction paths are analytically equivalent on strictly uniform data, but their location formulas and rounding sequences differ, so their finite-precision results can still differ.

### 4.3 Shape-preserving slopes and segment location

`ComputeShapePreservingNodalSlopes` first forms every $h_i$ and $\delta_i$, then applies the one-sided endpoint formulas and weighted harmonic means of Section 3.3 to obtain $m_i$, in $O(n)$. Contact geometry uses the same mathematical kernel for temporary nodes whose finiteness and strict ordering have already been established; repeating or eliding those prerequisite checks does not change the slope formula.

`MonotoneCubicInterpolant::Locate` always performs a binary search over actual knots. It has no uniform-grid path and no knot-snap tolerance. The local parameter is only restricted by

$$
t=\operatorname{clamp}\!\left(\frac{y-y_i}{h_i},0,1\right)
$$

The left end uses the first segment at $t=0$ and the right end the last segment at $t=1$. The left value is therefore formed as $c_0=z_0$, while the right value is formed as $c_0+c_1+c_2+c_3$. The latter equals $z_{n-1}$ in exact arithmetic but floating-point summation does not promise bitwise equality with the stored endpoint value. Segment location is $O(\log n)$.

### 4.4 Arc-length quadrature and inverse search

The sixteen-point rule in `IntegrateProfileArcLength` is stored as eight positive nodes and weights, used symmetrically about the interval midpoint; the weights summing to two is the identity a reader can check against the table. The literals of the two arrays `kGaussLegendreAbscissae` and `kGaussLegendreWeights` are:

| $k$ | $\xi_k$ | $w_k$ |
|---|---|---|
| 1 | `0.095012509837637440185319335424958` | `0.189450610455068496285396723208283` |
| 2 | `0.281603550779258913230460501460496` | `0.182603415044923588866763667969219` |
| 3 | `0.458016777657227386342419442983577` | `0.169156519395002538189312079030359` |
| 4 | `0.617876244402643748446671764048791` | `0.149595988816576732081501730547479` |
| 5 | `0.755404408355003033895101194847442` | `0.124628971255533872052476282192017` |
| 6 | `0.865631202387831743880467897712393` | `0.095158511682492784809925107602246` |
| 7 | `0.944575023073232576077988415534608` | `0.062253523938647892862843836994378` |
| 8 | `0.989400934991649932596154173450333` | `0.027152459411754094851780572456018` |

`AccumulateProfileArcLength` applies the single-interval rule once to every strictly increasing knot interval, requiring $O(16\,n)$ derivative evaluations.

Given $\sigma^\star\in[\Sigma_0,\Sigma_{n-1}]$, `FindLateralCoordinateAtArcLength` uses `std::upper_bound` to find the unique cumulative-length interval $j$, sets the local target to $\sigma^\star-\Sigma_j$, and performs 64 bisections over $[y_j,y_{j+1}]$. Each step compares $\sigma(y_j,y_{\mathrm{mid}})$ with the local target and returns the final bracket midpoint. Since $\sqrt{1+z'^2}>0$, arc length is strictly increasing in $y$ and the root in this interval is unique; every quadrature remains inside one spline segment. A fixed iteration count makes the discrete inverse a deterministic function of its inputs, but its result is still affected by quadrature and floating-point rounding and is not an analytic exact root.

### 4.5 Equal arc-length rescan

Let the rescan step satisfy $\Delta\sigma\ge0$. At $\Delta\sigma=0$, contact geometry uses the side-resolved authored nodes directly. For $\Delta\sigma>0$, `LayControlNodes` follows this procedure:

```
physical = authored.ResolveForSide(kRight)
first_pass = NaturalCubicSpline(physical.y, physical.z)
cumulative = AccumulateProfileArcLength(first_pass, physical.y)
L = cumulative.back()
tol = 16 * eps * max(1, |L|, |step|)
nodes = [(physical.y.front, physical.z.front)]
for k = 1, 2, ...:
    target = k * step
    if target >= L - tol: break
    y_k = FindLateralCoordinateAtArcLength(first_pass, physical.y, cumulative, target)
    nodes.append((y_k, first_pass.Evaluate(y_k)))
nodes.append((physical.y.back, physical.z.back))        // copied, not re-evaluated
if side == kLeft: nodes = reverse(nodes) with y -> -y
```

The termination tolerance `RescanEndTolerance(total, step)` is $16\,\varepsilon\max(1,|L|,|\Delta\sigma|)$. It prevents construction of a node almost coincident with the physical endpoint when total arc length is an integer multiple of the step. The first spline is built on the authored point list after physical-right side resolution and may take either spline path according to the data. The final interval is a remainder: stopping at $k\Delta\sigma\ge L-\tau_{\mathrm{end}}$ bounds its arc length by $\Delta\sigma+\tau_{\mathrm{end}}$. There are about $L/\Delta\sigma$ nodes, and each interior node requires one fixed-iteration inverse arc-length search.

### 4.6 Constructing the gauge datum

`ComputeRailGaugeDatum` forms $\rho$ from the side, rolls the authored points and sorts them by rolled lateral coordinate. It then forms the measuring level as the least rolled vertical coordinate plus $d$, scans crossings of adjacent sorted point pairs, and selects the extreme crossing facing the track centre. A segment coincident with the measuring level contributes its midpoint, avoiding a zero denominator. Sorting costs $O(n\log n)$ and the scan costs $O(n)$. The resulting object is the section of a polyline reconnected in rolled-lateral order; if roll changes the authored adjacency order, it is not topologically identical to a rigidly rotated authored polyline.

## 5. Implementation mapping

The table locates only the implementations that carry the central mathematical relations; it is not an interface or configuration reference.

| Theoretical object or algorithm | Main implementation | Source |
|---|---|---|
| Authored points and left/right side resolution | `ProfilePoints`, `SideResolvedProfile` | [`profile_points.cc`](../../../libs/wheel_rail_contact/src/profile_points.cc) |
| Natural cubic spline, uniformity test and knot snapping | `NaturalCubicSpline`, `SpacingsAreUniform`, `Locate` | [`natural_cubic_spline.cc`](../../../libs/wheel_rail_contact/src/natural_cubic_spline.cc) |
| Shape-preserving slopes and Hermite interpolation | `ComputeShapePreservingNodalSlopes`, `MonotoneCubicInterpolant` | [`monotone_cubic_interpolant.cc`](../../../libs/wheel_rail_contact/src/monotone_cubic_interpolant.cc) |
| Single-segment arc length, cumulative length and inversion | `IntegrateProfileArcLength`, `AccumulateProfileArcLength`, `FindLateralCoordinateAtArcLength` | [`profile_arc_length.cc`](../../../libs/wheel_rail_contact/src/profile_arc_length.cc) |
| Equal-arc resampling and left/right mirroring | `WheelProfilePreprocessing::LayControlNodes` | [`wheel_profile_preprocessing.cc`](../../../libs/wheel_rail_contact/src/wheel_profile_preprocessing.cc) |
| Gauge face and gauge datum on the rolled polyline | `ComputeRailGaugeDatum` | [`rail_gauge_datum.cc`](../../../libs/wheel_rail_contact/src/rail_gauge_datum.cc) |
| Combination of spline and Hermite surface in contact geometry | `SurfaceThroughNodes`, `ResolveWheelNodes` | [`contact_geometry.cc`](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |

## 6. Theoretical assumptions and range of applicability

- A profile is represented as a single-valued graph $z(y)$, and authored lateral coordinates must be strictly monotone. An undercut rail head or flange for which one $y$ maps to several $z$ values cannot be represented as one profile in this model.
- The natural cubic spline is $C^2$ in its interior and has zero endpoint moments. Flat continuation beyond the measured span generally makes the first derivative jump at both boundaries. This continuation does not imply material outside the measured profile.
- The uniform fast path idealises every grid that passes its tolerance as $y_0+i h_0$. When actual knots differ from that ideal grid, this is a finite-precision geometric approximation; knot snapping also makes its pointwise floating-point function differ from the general path.
- The slope rule of `FromValues` produces a shape-preserving piecewise Hermite curve. `FromNodalSlopes` has the same property only when the supplied slopes satisfy the corresponding monotone-Hermite conditions; arbitrary finite slopes may overshoot.
- When natural-spline nodal slopes are reused, identical knots, values and interval widths guarantee only that the exact-arithmetic piecewise polynomials agree. Knot snapping, local-parameter clamping, right-end evaluation order and uniform-grid idealisation can all prevent bitwise equality of public evaluation results.
- Arc length is approximated by fixed sixteen-point Gauss–Legendre quadrature on each spline segment, and inversion by 64 fixed bisections. Because the integrand is strictly positive, the arc-length map is strictly increasing and its inverse is unique within the knot interval; this does not turn the quadrature result into analytic arc length.
- Equal-arc resampling selects nodes along a first spline and then constructs a second spline through them. The second spline generally passes through neither the authored interior points nor exactly the same curve as the first spline.
- The gauge datum is defined by a prescribed-depth section of the measured polyline after roll and sorting by lateral coordinate, not by the natural spline or shape-preserving interpolant. Left and right roll angles necessarily have opposite signs; equal offsets and opposite datums additionally require mirror symmetry of the authored rail or an equivalent symmetry of the selected intersections.
