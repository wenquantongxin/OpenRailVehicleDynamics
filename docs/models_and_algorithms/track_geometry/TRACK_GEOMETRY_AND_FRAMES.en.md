[中文](TRACK_GEOMETRY_AND_FRAMES.md)

# Line geometry and track frames

This document describes the geometric model and algorithms of the ORVD line layer `orvd::track_geometry`: scalar profiles laid out along track station, the three-dimensional centerline obtained by integrating curvature and the vertical profile, the track inertial frame and two frames that ride along the line, tangent continuation outside the finite definition interval, and local branch projection of a point onto the centerline. Each formula either corresponds directly to a source computation or states its mathematical definition; a compact table at the end connects the theoretical objects to their implementation sites.

## 1. Scope

The objects modelled here are the scalar profile `TrackScalarProfile` under [`libs/track_geometry`](../../../libs/track_geometry/README.md) (planar curvature and superelevation share one representation), the line `TrackGeometry`, the pose and kinematics quantities `TrackFramePose` and `TrackFrameKinematics`, and local branch projection represented by `TrackStationProjection`. The emphasis is on their physical quantities, analytic relations and numerical discretisation; configuration-file formats and interface catalogues are outside the scope.

This document deliberately leaves out the constant-grade, parabolic-vertical-curve and circular-vertical-curve formulas of `TrackVerticalProfile`, which are derived in [`TRACK_VERTICAL_PROFILE_MODELLING.md`](TRACK_VERTICAL_PROFILE_MODELLING.md); only the way that profile enters the centerline and track frame is treated here. Track irregularity is not line geometry; see [`TRACK_IRREGULARITY_SPECTRA.md`](../track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.md). How wheel-rail pose reduction consumes `TrackFrameKinematics` belongs to [`WHEEL_RAIL_POSE_REDUCTION.en.md`](../wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.en.md). Time advancement and rollback of projection seeds belong to dynamics and numerical methods; this document treats only one geometric projection.

Frames, signs, the definition of the track station and the unit suffixes all follow [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md); this document does not repeat the conclusions of its Sections 2 and 3 and only cites them where needed.

## 2. Notation

Only the notation this document adds is listed below; for $s$, $\psi$, $\kappa$, $g$, $u$, $b$, $\phi$, $\mathbf C(s)$, $R_{IT}$, $R_{I0}$, $\mathbf x_0,\mathbf y_0,\mathbf z_0$, $\boldsymbol\omega_{IT}$ and the definition interval $[s_{\min},s_{\max}]$ see Sections 2 and 3 of [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md).

| Symbol | Meaning | Code identifier |
|---|---|---|
| $v(s)$ | The value of one scalar profile: $\kappa$ when the profile is the curvature, $u$ when it is the superelevation | `TrackScalarProfile::Value` |
| $s_i$, $L_i$ | Start station and length of segment $i$; the start accumulates `length_meters` over the segments from `start_track_station_meters` | `TrackScalarSegment::length_meters` |
| $o$, $\hat s=s-o$ | Polynomial origin of a piece and its local coordinate | `polynomial_origin_track_station_meters` |
| $x=\hat s/L_i$ | Normalised coordinate of a Hermite blend segment | `kHermiteCubicBlend` |
| $\Delta v$ | End value minus start value of a Hermite blend segment | `end_value`, `start_value` |
| $c_k$ | Ascending polynomial coefficients of a piece in its local coordinate | `coefficients` |
| $F(\ell)$, $I_i$ | Antiderivative of a piece polynomial that vanishes at a zero local coordinate; integral of the profile from $s_{\min}$ to the start of piece $i$ | `integral_at_piece_start_` |
| $s_b$, $w$ | Seam boundary station and full width of the seam window | `TrackSeamTransition::window_length_meters` |
| $\xi$, $d_r$, $H_r(\xi)$ | Normalised window coordinate, the six boundary data, the quintic Hermite basis functions | `kQuinticBasis` |
| $\Delta s$ | Declared station node spacing | `station_node_spacing_meters` |
| $B_j$, $n_j$ | Breakpoint $j$ of the ordered union of curvature and vertical breakpoints; number of panels a breakpoint interval is divided into | `nodes_` |
| $\xi_q$, $w_q$ | Abscissae and weights of the eight-point Gauss–Legendre rule | `kQuadratureAbscissae`, `kQuadratureWeights` |
| $h$ | Half turn angle of a constant-curvature panel | `half_turn` |
| $\mathbf t$, $n$ | The unnormalised three-dimensional tangent $\mathbf C'(s)$ and its norm $\sqrt{1+g^2}$ | `tangent`, `slope_norm` |
| $\boldsymbol\omega_0$ | Station rotation rate of the roll-free tangent frame relative to the inertial frame | `roll_free_rate` |
| $\mathbf p$, $f(s)$ | The point in space to project; the projection objective, written objective in the README | `EvaluateObjectiveDerivatives` |
| $\tau$, $\ell_0$, $\beta$ | Relative tolerance of the first-derivative residual, its absolute scale floor, and the resulting residual bound | `kObjectiveGradientRelativeTolerance`, constant `1.0`, `gradient_bound` |
| $s_{\text{seed}}$ | Branch seed station supplied by the caller | `branch_seed_track_station_meters` |

## 3. Model

### 3.1 Scalar profiles: constant segments and Hermite cubic blends

A scalar profile is a sequence of `TrackScalarSegment` laid end to end from `start_track_station_meters`; each segment length `length_meters` must be finite and strictly positive, and the segment end must be a finite station strictly after its start. A segment has only two shapes. `TrackScalarSegmentShape::kConstant` holds `start_value` over the whole segment; `TrackScalarSegmentShape::kHermiteCubicBlend` blends the value from `start_value` to `end_value` through the normalised coordinate $x$:

$$
v(s)=v_{\text{start}}+\Delta v\,\left(3x^{2}-2x^{3}\right),\qquad x=\frac{s-s_i}{L_i},\qquad \Delta v=v_{\text{end}}-v_{\text{start}}
$$

The implementation does not store the polynomial in $x$; it stores ascending coefficients in the physical local coordinate $\hat s$ whose origin is the segment start. The constructor writes `piece.coefficients[0] = segment.start_value`, `piece.coefficients[2] = 3.0 * change / (length * length)` and `piece.coefficients[3] = -2.0 * change / (length * length * length)`, and the linear coefficient stays zero:

$$
v(s)=c_0+c_2\,\hat s^{2}+c_3\,\hat s^{3},\qquad c_0=v_{\text{start}},\quad c_2=\frac{3\,\Delta v}{L_i^{2}},\quad c_3=-\frac{2\,\Delta v}{L_i^{3}}
$$

Differentiating with respect to $x$ exposes the end behaviour of this blend:

$$
v'(s)=\frac{6\,\Delta v}{L_i}\,x\,(1-x),\qquad v''(s)=\frac{6\,\Delta v}{L_i^{2}}\,(1-2x)
$$

The first derivative vanishes at $x=0$ and at $x=1$, and the second derivative takes $6\Delta v/L_i^{2}$ and $-6\Delta v/L_i^{2}$ at the two ends. When $\Delta v\ne0$, it therefore jumps against the zero second derivative of an adjacent constant segment; when $\Delta v=0$, the blend degenerates to a constant segment. This curve is consequently not a non-trivial clothoid: a clothoid's curvature is linear in station, and its non-zero curvature rate does not fall to zero at the ends. The header [`track_geometry_segments.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_geometry_segments.h) states that this is a deliberate choice of the library ("It is deliberately not a clothoid"), and the module [`README.md`](../../../libs/track_geometry/README.md) further rules that downstream consumers must not read the transition curve here by clothoid standards; used on a curvature profile, this segment is the transition curve of this library.

Adjacent segments without a seam window obey a continuous-profile assumption: the declared end value of the preceding segment equals the declared start value of the following one. The implementation compares the declared values instead of re-evaluating the polynomials, because a Hermite blend need not reproduce its stated end value bit for bit in floating-point arithmetic. If value jumps were admitted, the integrated heading would remain continuous but the track frame and its station derivatives would lose their usual differential meaning at the jump; that case is therefore outside this model.

### 3.2 The quintic seam transition

Two adjacent segments may additionally declare a `TrackSeamTransition`: the index `preceding_segment_index` names the boundary $s_b$ (the end station of that segment), the full window width is `window_length_meters`, written $w$, and the window $[s_b-w/2,\ s_b+w/2]$ is centred on the boundary. A seam is identified by topology rather than by a written-out station, so nothing has to equal an accumulated sum of segment lengths in floating point. Inside the window the profile follows a quintic polynomial that matches the value, the first derivative and the second derivative of each original segment's own polynomial at the two window ends. The boundary data are evaluated on the original segment polynomials at the window endpoints: on the left, $(f_0,f_0',f_0'')$ of the preceding segment at $s_b-w/2$; on the right, $(f_1,f_1',f_1'')$ of the following segment at $s_b+w/2$. In the normalised window coordinate $\xi=(s-s_b+w/2)/w\in[0,1]$ the quintic is

$$
p(\xi)=\sum_{r=0}^{5} d_r\,H_r(\xi),\qquad
\mathbf d=\begin{bmatrix} f_0 & w f_0' & w^{2} f_0'' & f_1 & w f_1' & w^{2} f_1''\end{bmatrix}
$$

where the ascending coefficients of the six basis functions $H_r$ are exactly the constant table `kQuinticBasis` in [`track_profile_quintic.cc`](../../../libs/track_geometry/src/track_profile_quintic.cc), whose rows correspond in order to the value, the first derivative and the second derivative at the start, then the same three at the end:

```text
kQuinticBasis[6][6] = {
    {1.0, 0.0, 0.0, -10.0, 15.0, -6.0},
    {0.0, 1.0, 0.0, -6.0, 8.0, -3.0},
    {0.0, 0.0, 0.5, -1.5, 1.5, -0.5},
    {0.0, 0.0, 0.0, 10.0, -15.0, 6.0},
    {0.0, 0.0, 0.0, -4.0, 7.0, -3.0},
    {0.0, 0.0, 0.0, 0.5, -1.0, 0.5},
};
```

The normalised coefficients $a_k=\sum_r d_r\,K_{rk}$ ($K$ being that table) are then rescaled by power back into the physical local coordinate whose origin is the window start:

$$
c_k=\frac{a_k}{w^{k}},\qquad k=0,\dots,5,\qquad p(s)=\sum_{k=0}^{5}c_k\,\left(s-s_b+\tfrac{w}{2}\right)^{k}
$$

That rescaling is the whole of `internal::BuildQuinticHermiteCoefficients`. It is the single quintic boundary kernel shared by the scalar profile and the vertical profile: `TrackScalarProfile` uses it in [`track_geometry_segments.cc`](../../../libs/track_geometry/src/track_geometry_segments.cc), and `TrackVerticalProfile` calls the same function in [`track_vertical_profile.cc`](../../../libs/track_geometry/src/track_vertical_profile.cc) with the analytic $(g,g',g'')$ of its original segments. A seam is a declared interval of the geometry, not a smoothing applied silently after construction; any statement about a segment's "own value" holds only for the part of that segment outside the windows.

The definition of the quintic bridge requires a finite, strictly positive width $w$, a window centred on an internal boundary with adjacent segments on both sides, at most one window per boundary, and a half-width $w/2$ no greater than either adjacent segment length. Windows must not overlap, although touching endpoints are allowed. These conditions give every set of six boundary data a unique source and make the resulting piece partition single-valued over the whole station domain.

### 3.3 The three-dimensional centerline

The planar curvature profile $\kappa(s)$ and the vertical profile $g(s)$ share one station axis. The heading is the exact integral of the curvature, the horizontal components of the centerline integrate along the heading, and the elevation is the negative of the vertical-profile integral:

$$
\psi(s)=\int_{s_{\min}}^{s}\kappa(\sigma)\,d\sigma,\qquad
\begin{bmatrix}x(s)\\ y(s)\end{bmatrix}=\int_{s_{\min}}^{s}\begin{bmatrix}\cos\psi(\sigma)\\ \sin\psi(\sigma)\end{bmatrix}d\sigma,\qquad
z(s)=-\int_{s_{\min}}^{s}g(\sigma)\,d\sigma
$$

In the implementation $\psi(s)$ is `curvature_.IntegralFromStart(s)` and $z(s)$ is `-grade_.IntegralFromStart(s)`, both analytic; only the horizontal components need quadrature, and only where the curvature is not constant (Section 4.2). The centerline leaves the origin of the inertial frame with zero heading: the constructor sets the first node's `heading_radians` to `0.0` and its position to the zero vector. The first and second station derivatives are

$$
\mathbf C'(s)=\begin{bmatrix}\cos\psi\\ \sin\psi\\ -g\end{bmatrix},\qquad
\mathbf C''(s)=\begin{bmatrix}-\kappa\sin\psi\\ \kappa\cos\psi\\ -g'\end{bmatrix},\qquad
\left\lVert\mathbf C'(s)\right\rVert=\sqrt{1+g^{2}}
$$

formed by `CenterlineDerivativeUnchecked` and `CenterlineSecondDerivativeUnchecked` respectively, where $g'$ is `grade_.FirstDerivativePerMeter`. The station is planar projected mileage, so the horizontal projection of $\mathbf C'$ is a unit vector, while the full three-dimensional derivative has norm $\sqrt{1+g^2}$, which is not one wherever the grade is non-zero; $\mathbf C''$ is used only by the projection's second-order condition and therefore stays private. Closed forms for $g$, $g'$, $g''$ and $\int g$ on each vertical segment are in Section 4 of [`TRACK_VERTICAL_PROFILE_MODELLING.en.md`](TRACK_VERTICAL_PROFILE_MODELLING.en.md); this document only uses the fact that they are analytic.

### 3.4 Track frames: the inertial frame I, the roll-free tangent frame and the track frame T

The inertial frame `I` is defined by the line: its origin is the start of the line, `+x` points along increasing station there, `+y` points to the right, `+z` points downward, and `DownwardUnitVectorInInertial()` returns `(0.0, 0.0, 1.0)`. Two frames ride along the line. The axes of the roll-free tangent frame are

$$
\mathbf x_0=\frac{\mathbf t}{n},\qquad \mathbf t=\mathbf C'(s),\qquad n=\sqrt{1+g^{2}},\qquad
\mathbf y_0=\begin{bmatrix}-\sin\psi\\ \cos\psi\\ 0\end{bmatrix},\qquad
\mathbf z_0=\mathbf x_0\times\mathbf y_0
$$

corresponding to `roll_free_x`, `roll_free_y` and `roll_free_z` in `EvaluateTrackFrame`. $\mathbf y_0$ is always horizontal and points to the track's right; where the grade is non-zero, $\mathbf z_0$ is not the inertial vertical. The track frame `T` is the roll-free frame rolled about its own $\mathbf x_0$ axis by the superelevation roll angle $\phi$:

$$
R_{IT}=R_{I0}\,R_x(\phi),\qquad
R_x(\phi)=\begin{bmatrix}1&0&0\\ 0&\cos\phi&-\sin\phi\\ 0&\sin\phi&\cos\phi\end{bmatrix},\qquad
\phi=\arcsin\frac{u}{b}
$$

where the three columns of $R_{I0}$ are $\mathbf x_0,\mathbf y_0,\mathbf z_0$ in order, implemented as `rotation = rotation_roll_free * roll_about_x`; `TrackRollRadians` returns `std::asin(superelevation_.Value(definition_station) / superelevation_reference_baselength_meters_)`. Here $u$ is the value of the superelevation profile, `SuperelevationMeters`, and $b$ is the constructor argument `superelevation_reference_baselength_meters`.

Definition of the signed superelevation: take two abstract reference points on the track-frame transverse axis $\mathbf y_T$ (the second column of $R_{IT}$) at $b/2$ on either side of the centerline, $\mathbf p_{\text{right}}=+\tfrac{b}{2}\mathbf y_T$ and $\mathbf p_{\text{left}}=-\tfrac{b}{2}\mathbf y_T$; then $u$ is their signed separation along the roll-free vertical axis $\mathbf z_0$:

$$
u=(\mathbf p_{\text{right}}-\mathbf p_{\text{left}})\cdot\mathbf z_0=b\sin\phi,\qquad
(\mathbf p_{\text{right}}-\mathbf p_{\text{left}})\cdot\mathbf e_z=\frac{u}{\sqrt{1+g^{2}}}
$$

The first identity follows directly from $\mathbf y_T=\cos\phi\,\mathbf y_0+\sin\phi\,\mathbf z_0$; the second ($\mathbf e_z$ being the inertial vertical unit vector) says that $u$ equals the inertial vertical height difference only when the grade is zero. A positive superelevation means the right reference point is lower, which is a positive roll about $\mathbf x_0$: it turns `+y` toward `+z`, and `+z` is down. These two reference points do not assert that actual rails or profile reference points sit exactly there; $b$ is neither nominal gauge nor the reference-point spacing used to place wheel and rail profiles. ORVD implements a centerline datum only: the centerline does not move with superelevation, and displacement under a fixed inner- or outer-rail datum is outside the current model.

The roll definition applies where $\lvert u\rvert<b$ at every point of the profile, without clipping. A quarter turn of roll is no longer meaningful as line geometry, and the roll rate $\phi'$ is singular there, so the admissible interval is open rather than closed. `TrackScalarProfile::MaximumAbsoluteValue` evaluates each piece at both ends and at every real root of its derivative polynomial (Section 4.1), rather than sampling, so an interior extremum of a quintic seam is included as well.

`TrackFrameKinematics` returns the pose and its first station derivative in one evaluation: the pose `TrackFramePose` carries $R_{IT}$ and the centerline position; `centerline_derivative_in_inertial_meters_per_meter()` returns $\mathbf t=\mathbf C'(s)$, whose norm is $\sqrt{1+g^2}$ rather than one; and `track_frame_rotation_rate_in_inertial_radians_per_meter()` returns $\boldsymbol\omega_{IT}$ expressed in the inertial frame, which satisfies

$$
\frac{dR_{IT}}{ds}=\operatorname{skew}(\boldsymbol\omega_{IT})\,R_{IT}
$$

This identity defines the relation between the returned rotation rate and the attitude derivative. Section 4.3 derives $\boldsymbol\omega_{IT}$. Pose and derivative are formed in one evaluation so that both use the same geometric state.

### 3.5 The definition interval and the three-dimensional tangent continuation

The line geometry is defined on the finite station interval $[s_{\min},s_{\max}]$, whose endpoints are the start and end stations of the curvature profile (all three profiles have been canonicalised to a common end at construction, Section 4.2). `ClassifyTrackStation` returns `TrackStationRegion::kBeforeDefinedInterval` for $s<s_{\min}$, `kAfterDefinedInterval` for $s>s_{\max}$ and `kWithinDefinedInterval` otherwise; both endpoints belong to the interval. For any finite station outside it, `TrackGeometry` continues along the three-dimensional tangent of the nearest definition boundary $s_b$ as a straight line:

$$
\mathbf C(s)=\mathbf C(s_b)+(s-s_b)\,\mathbf C'(s_b),\qquad
\kappa(s)=0,\qquad \mathbf C''(s)=\mathbf 0,\qquad
\psi(s)=\psi(s_b),\quad g(s)=g(s_b),\quad u(s)=u(s_b),\quad g'(s)=u'(s)=0
$$

In the implementation `CenterlinePositionUnchecked` combines the boundary position and boundary derivative for an out-of-domain station. Heading, superelevation, grade and roll take their boundary values, while curvature, the centerline second derivative, and the rates of grade and superelevation become zero. This tangent continuation belongs to the geometric definition of `TrackGeometry`; the underlying scalar profiles remain defined only on their finite intervals. It is an explicit continuation choice, not an extrapolation of the final analytic profile piece.

`support_start_track_station_meters()` is the implementation's representation of the profile support: it is the start station of the first piece whose polynomial coefficients are not all zero, and it has no value when the profile is identically zero. Because it examines analytic pieces rather than samples, it returns the window start when the first non-zero piece is a seam, not the original segment boundary. The vertical profile follows the same idea: a constant-grade original segment is classified by its start grade and a seam piece by whether all its coefficients vanish.

### 3.6 The objective of the station projection

The projection of a point $\mathbf p$ onto the centerline uses half the squared point-to-centerline distance as its objective, whose first and second station derivatives are

$$
f(s)=\tfrac12\left\lVert\mathbf p-\mathbf C(s)\right\rVert^{2},\qquad
f'(s)=-(\mathbf p-\mathbf C)\cdot\mathbf C',\qquad
f''(s)=\left\lVert\mathbf C'\right\rVert^{2}-(\mathbf p-\mathbf C)\cdot\mathbf C''
$$

corresponding to `gradient` and `hessian` in `EvaluateObjectiveDerivatives` (written objective, objective′ and objective″ in the README). An admissible station must put the first derivative inside a scaled residual bound and keep the second derivative strictly positive:

$$
\lvert f'(s)\rvert\le\beta(s),\qquad \beta(s)=\tau\left(\lVert\mathbf p-\mathbf C\rVert\,\lVert\mathbf C'\rVert+\ell_0\right),\qquad \ell_0=1\,\mathrm m,\qquad f''(s)>0
$$

Here $\tau$ is `kObjectiveGradientRelativeTolerance`, whose value is `1.0e-10`. The residual bound scales with distance because $f'$ has units of length; the numeric constant `1.0` in the SI-coordinate implementation represents $\ell_0=1\,\mathrm m$ and provides a dimensionally consistent absolute floor at zero distance. Requiring $f''>0$ excludes the case where the distance stationary point is a maximum, for instance when the point lies beyond the curvature centre of a circular curve. The model deliberately describes only a local branch: the caller supplies a station seed $s_{\text{seed}}$ that identifies the current branch, and this layer performs no whole-line closest-point search, scans no nodes and never reselects a distant root; the reasons and the algorithm are in Section 4.4.

## 4. Algorithm

### 4.1 Constructing and evaluating a scalar profile

Profile construction has five steps. First, it accumulates station segment by segment and forms each original local polynomial as in Section 3.1. Second, it forms all seam intervals from internal boundaries and window widths and checks the geometric prerequisites of Section 3.2 as a group. Third, every boundary not covered by a window must have matching declared values. Fourth, it sorts the windows by start station and cuts the piece sequence along a station cursor: the part of an original segment before a window, the window, and the part after it become pieces in turn. A segment cut by a window retains the original segment start as its polynomial origin, avoiding extra rounding from re-expansion about the cut; a seam piece uses the window start. Fifth, it computes the accumulated integral $I_i$ at every piece start, records the breakpoint table (all piece starts plus the profile end), and finds the support start.

Every piece stores at most six ascending coefficients plus the coefficient count (one term for a constant, four for a Hermite blend, six for a seam), and evaluation runs in Horner form on the local coordinate $\hat s=s-o$, with `Value`, `FirstDerivativePerMeter` and `SecondDerivativePerMeterSquared` mapping to `EvaluatePolynomial`, `EvaluateFirstDerivative` and `EvaluateSecondDerivative`. The integral is analytic: each piece's antiderivative is taken to vanish at a zero local coordinate,

$$
F(\ell)=\sum_{k}\frac{c_k}{k+1}\,\ell^{\,k+1},\qquad
\int_{s_{\min}}^{s}v(\sigma)\,d\sigma=I_i+F(s-o_i)-F\!\left(s_{i,\text{start}}-o_i\right)
$$

where $i$ is the piece containing $s$. `IntegralFromStart` implements this expression. `PieceIndexAt` performs a binary search over piece starts and takes the last piece whose start is not after $s$, hence the right-hand piece at an interior boundary and the last piece at the profile end. Derivatives are consequently right-hand one-sided values at a boundary without a seam; for example, `SecondDerivativePerMeterSquared` returns the right piece's second derivative at a constant-to-blend junction. The mathematical domain of the underlying profile is strictly $[s_{\min},s_{\max}]$; clamping and tangent continuation occur only in the higher-level `TrackGeometry`.

`LocalPolynomialDegree` reports the local degree after dropping trailing zero coefficients, and degree zero selects the closed-form constant-curvature path; a zero-change Hermite blend and a seam between equal constants are therefore recognised as constant. `MaximumAbsoluteValue` evaluates every piece at both endpoints and at all real roots of its derivative polynomial, yielding the magnitude extremum of the analytic piece. The derivative degree is at most four; `FindPolynomialRootsInClosedInterval` recursively finds roots of the next lower derivative, partitions the interval into monotone stretches, and closes sign-changing roots by bisection. Scale-aware zero detection at endpoints and partition points preserves even-multiplicity roots, and nearby roots are merged by a scale-aware distance. This converts whole-interval conditions such as $|u|<b$ into checks on finitely many candidate points.

When the three profile ends differ only by final-place rounding, `ShortenDomainEndTo` canonicalises the longer profiles' last pieces and breakpoint tables to the common shortest end; no earlier piece changes.

### 4.2 The centerline node table and horizontal displacement

The node spacing $\Delta s$ and superelevation reference baselength $b$ are finite and positive, and the curvature, superelevation and vertical profiles share one station domain. Their starts coincide; their ends may differ by final-place rounding caused by different segment-accumulation orders. The implementation compares ends after division by $\max\{1,\lvert s_{\min}\rvert,\lvert e_1\rvert,\lvert e_2\rvert\}$, with a tolerance that grows with the two profiles' breakpoint counts and machine precision, and then adopts the shortest end as the common end. This canonicalisation absorbs only a floating-point accumulation difference; it does not admit profiles with physically different lengths.

The node table is laid on the ordered union $\{B_j\}$ of curvature and vertical breakpoints (sorted and deduplicated). A breakpoint interval $[B_j,B_{j+1}]$ of length $\ell_j$ is divided into $n_j=\max\{1,\lceil \ell_j/\Delta s\rceil\}$ panels, with nodes at $B_j+\ell_j\,q/n_j$ for $q=0,\dots,n_j-1$, followed by one terminal node. A node interval therefore straddles neither a switch of the horizontal-integration formula nor a switch of the centerline-derivative formula. Each interval queries `LocalPolynomialDegree` at its midpoint to decide whether it has constant curvature and records the curvature there.

Each node stores the heading $\psi_n$ (the exact integral `curvature_.IntegralFromStart`) and the centerline position. The horizontal displacement from a node to any station inside its interval is given by `HorizontalDisplacementFromNode`. Where the curvature is constant it uses the half-angle form of the circular chord:

$$
h=\tfrac12\,\kappa\,L,\qquad
\Delta\mathbf C_{xy}=L\,\frac{\sin h}{h}\begin{bmatrix}\cos(\psi_n+h)\\ \sin(\psi_n+h)\end{bmatrix},\qquad
\left.\frac{\sin h}{h}\right|_{h=0}:=1
$$

Written as a difference of sines divided by the curvature it would first subtract two nearly equal numbers and then multiply by the radius, losing most of the significant digits on a gentle curve with short panels; the half-angle form has no cancellation and degrades naturally to the straight case at $h=0$, so no separate branch is needed (`chord_ratio` takes `1.0` when `half_turn == 0.0`). Where the curvature is not constant, the heading is a polynomial whose sine and cosine have no elementary antiderivative, and a fixed eight-point Gauss–Legendre rule is used instead:

$$
\Delta\mathbf C_{xy}=\frac{L}{2}\sum_{q=1}^{8}w_q\begin{bmatrix}\cos\psi\!\left(m+\tfrac{L}{2}\xi_q\right)\\ \sin\psi\!\left(m+\tfrac{L}{2}\xi_q\right)\end{bmatrix},\qquad m=s_n+\tfrac{L}{2}
$$

The abscissae and weights are constant tables in the source:

```text
kQuadraturePointCount = 8
kQuadratureAbscissae = {-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                         0.1834346424956498,  0.5255324099163290,  0.7966664774136267,  0.9602898564975363}
kQuadratureWeights   = { 0.1012285362903763,  0.2223810344533745,  0.3137066458778873,  0.3626837833783620,
                         0.3626837833783620,  0.3137066458778873,  0.2223810344533745,  0.1012285362903763}
```

A fixed order over panels bounded by $\Delta s$ makes horizontal-integration error depend jointly on curvature variation, quadrature order and panel scale rather than on an adaptive stopping criterion. The code provides no uniform absolute error bound for arbitrary positive $\Delta s$, so $\Delta s$ is part of the numerical approximation rather than merely a storage parameter.

Node positions are a running sum over the nodes, so ordinary floating-point accumulation error grows with the node count and a finer spacing can make the absolute position worse even as the quadrature improves. The constructor uses Neumaier compensation to suppress, rather than mathematically eliminate, this accumulated error, keeping a sum $S$ and a compensation $c$ per horizontal component:

$$
t=S+a,\qquad
c\leftarrow c+\begin{cases}(S-t)+a,&\lvert S\rvert\ge\lvert a\rvert\\ (a-t)+S,&\text{otherwise}\end{cases},\qquad
S\leftarrow t,\qquad x_n=S+c
$$

A node's $z$ component is taken directly as `-grade_.IntegralFromStart` and takes no part in the running sum. The position at a non-node station is the position of the node found by the binary search `NodeIndexAtOrBefore` plus one call of `HorizontalDisplacementFromNode` from that node to the target station, that is, at most the remaining part of one panel is integrated; a station landing on the last node belongs to the interval that ends there.

### 4.3 Track-frame kinematics

`EvaluateTrackFrame` produces the pose and the station derivatives in one pass. It first takes $\psi$, $\kappa$, $g$, $g'$, $u$ and $u'$ (zero or boundary values outside the interval, per Section 3.5), then forms the three axes and the rotation as in Section 3.4. The station derivatives of the axes are

$$
\mathbf t'=\begin{bmatrix}-\kappa\sin\psi\\ \kappa\cos\psi\\ -g'\end{bmatrix},\qquad
n'=\frac{g\,g'}{n},\qquad
\mathbf x_0'=\frac{\mathbf t'}{n}-\mathbf x_0\,\frac{n'}{n},\qquad
\mathbf y_0'=\begin{bmatrix}-\kappa\cos\psi\\ -\kappa\sin\psi\\ 0\end{bmatrix},\qquad
\mathbf z_0'=\mathbf x_0'\times\mathbf y_0+\mathbf x_0\times\mathbf y_0'
$$

corresponding to `tangent_rate`, `slope_norm_rate`, `roll_free_x_rate`, `roll_free_y_rate` and `roll_free_z_rate`. The angular velocity of an orthonormal triad is half the sum of the cross products of its columns with their derivatives, which avoids writing three special cases by hand; the roll about $\mathbf x_0$ is then added:

$$
\boldsymbol\omega_0=\tfrac12\left(\mathbf x_0\times\mathbf x_0'+\mathbf y_0\times\mathbf y_0'+\mathbf z_0\times\mathbf z_0'\right),\qquad
\phi'=\frac{u'/b}{\sqrt{1-(u/b)^{2}}},\qquad
\boldsymbol\omega_{IT}=\boldsymbol\omega_0+\phi'\,\mathbf x_0
$$

The denominator of $\phi'$ is exactly the quantity that the open interval $\lvert u\rvert<b$ keeps away from zero. `TrackFrameKinematics` consists of $R_{IT}$, `CenterlinePositionUnchecked(s)`, $\mathbf t$ and $\boldsymbol\omega_{IT}$, so attitude, translational tangent and rotation rate are obtained at one station from the same geometric state.

### 4.4 Local branch projection

`ProjectPointOntoSeededBranch(point, seed)` starts from the supplied seed and makes at most two Newton corrections. If $s_k$ denotes iterate $k$, then

$$
s_{k+1}=s_k-\frac{f'(s_k)}{f''(s_k)},\qquad k=0,1,
$$

and the scaled stationary-point conditions of Section 3.6 are evaluated first at each of $s_0,s_1,s_2$. The condition $f''>0$ is both the second-order condition for a local minimum and a prerequisite for another Newton correction; using the same formula at a non-convex point would advance toward a distance maximum and is outside the algorithm. The projection uses only $\mathbf C$, $\mathbf C'$ and $\mathbf C''$, all of which follow the tangent continuation outside the definition interval, so the seed and root may lie outside the interval or cross its boundary.

The reason for avoiding a whole-line search is branch identity, not merely cost. One spatial point can have several distance stationary points on a line, for example near a circular curve or below a vertical alignment that first rises and then falls. A global closest point can switch roots, whereas local branch projection treats $s_{\text{seed}}$ as part of the current geometric branch. The line geometry stores no time history; a higher layer advances the seed with the dynamical state.

The algorithm defines no finite window around the seed and does not require the node grid to bracket the root; otherwise a window wall or node placement could change whether the same continuous branch is recognised. The corresponding locality assumption is strong: the current state must lie near a regular root with $f''>0$, and the seed must be in a Newton attraction region reachable in at most two corrections. The size of that region depends on curvature, grade and point-to-centerline distance; the code supplies no uniform radius.

### 4.5 Numerical approximation and non-smooth points

The implementation combines analytic computations with controlled numerical approximations. Scalar-profile values, derivatives, integrals and heading are analytic within each piece. Constant-curvature horizontal displacement uses the half-angle chord to avoid cancellation; varying-curvature panels use fixed eight-point Gauss–Legendre quadrature; node positions use Neumaier compensated summation. A common profile end absorbs only floating-point accumulation differences, while $\lvert u\rvert<b$ is assessed through extrema of the analytic pieces. Local projection uses a scaled residual with an absolute floor, the strict condition $f''>0$, and at most two corrections. Centerline horizontal position and the projection root are therefore numerical approximations; the piece polynomials and their analytic integrals are not.

The non-smooth points are the following. At the two ends of a Hermite blend, value and first derivative are continuous while the second derivative generally jumps; a degenerate $\Delta v=0$ segment or an adjacent segment whose end second derivative happens to match is an exception. At a boundary without a seam, value is continuous, the first derivative is continuous and zero at a constant-to-blend junction, and the second derivative may jump; a junction of equal constants is entirely smooth. At seam-window ends, value, first and second derivatives are continuous while the third may jump. At a definition boundary, $\mathbf C$, $\mathbf C'$ and $R_{IT}$ are continuous, but $\kappa$, $g'$ and $u'$ become zero outside, so $\mathbf C''$ and $\boldsymbol\omega_{IT}$ may jump. Piece lookup selects the right-hand piece at an interior boundary and the last piece at the end, so derivative queries return one-sided values there. The derivative of $\phi=\arcsin(u/b)$ is unbounded as $\lvert u\rvert\to b$, hence the model is restricted to $\lvert u\rvert<b$.

Complexity: one profile evaluation or integral is a binary search over the piece starts plus Horner, that is $O(\log P)$ for $P$ pieces; one centerline position is a binary search over the nodes, $O(\log N)$, plus one closed form or eight heading evaluations; one `EvaluateTrackFrame` is a constant number of profile queries plus one position; one projection is at most three objective-derivative evaluations. Construction is $O(N)$ panel displacements plus $O(P)$ extremum searches.

## 5. Implementation mapping

The table locates only the implementations that carry the central mathematical relations; it is not an interface or configuration reference.

| Theoretical object or algorithm | Main implementation | Source |
|---|---|---|
| Constant and Hermite-cubic scalar profiles, analytic derivatives and integrals | `TrackScalarProfile` | [`track_geometry_segments.cc`](../../../libs/track_geometry/src/track_geometry_segments.cc) |
| Quintic $C^2$ seam | `internal::BuildQuinticHermiteCoefficients` | [`track_profile_quintic.cc`](../../../libs/track_geometry/src/track_profile_quintic.cc) |
| Piece extrema and derivative-polynomial roots | `MaximumAbsoluteValue`, `FindPolynomialRootsInClosedInterval` | [`track_geometry_segments.cc`](../../../libs/track_geometry/src/track_geometry_segments.cc) |
| Heading, centerline nodes and horizontal quadrature | `TrackGeometry`, `HorizontalDisplacementFromNode` | [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| Track attitude and station rotation rate | `EvaluateTrackFrame` | [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| Three-dimensional tangent continuation outside the finite domain | `CenterlinePositionUnchecked` and the profile queries | [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| First and second derivatives of the distance objective and Newton projection | `EvaluateObjectiveDerivatives`, `ProjectPointOntoSeededBranch` | [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| Analytic quantities of the vertical profile | `TrackVerticalProfile` | [`track_vertical_profile.cc`](../../../libs/track_geometry/src/track_vertical_profile.cc) |

## 6. Theoretical assumptions and range of applicability

- Track station $s$ is centerline mileage projected onto the horizontal plane, not three-dimensional arc length. Consequently $\|\mathbf C'(s)\|=\sqrt{1+g^2}$, which equals one only when $g=0$.
- Planar curvature and superelevation consist only of constant pieces, Hermite cubic blends and optional quintic $C^2$ seams. For a curvature transition from $\kappa_0$ to $\kappa_1$, the Hermite law
  $$
  \kappa(s)=\kappa_0+(\kappa_1-\kappa_0)(3x^2-2x^3)
  $$
  is exactly the curvature law of a Bloss transition curve, not a clothoid. The current model has no linear-curvature clothoid, sinusoidal transition or arbitrary sampled alignment; those shapes cannot be represented losslessly by the existing segment types.
- Superelevation uses a centerline-roll model. The quantity $u=b\sin\phi$ is the signed separation along the roll-free vertical axis. The model does not define how the centerline translates when an inner- or outer-rail datum is fixed, and $b$ is not identified with nominal track gauge.
- The horizontal centerline is integrated analytically on constant-curvature panels and by fixed eight-point Gauss–Legendre quadrature on varying-curvature panels. The spacing $\Delta s$ sets the panel scale; the present derivation gives no uniform absolute error bound for arbitrary curvature pieces and arbitrary $\Delta s$.
- Outside the finite definition interval, the model uses the boundary three-dimensional tangent. This is a chosen geometric continuation, not a unique consequence of the curvature, grade or superelevation formulas inside the interval.
- Spatial-point projection is a seeded local-branch problem, not a whole-line closest-point problem. At most two Newton corrections assume that the seed already lies in the local attraction region of a regular minimum with $f''>0$; no uniform analytic radius for that region is available.
- Smoothness follows the piece type: Hermite blend endpoints are generally $C^1$, and quintic seam endpoints are $C^2$. Tangent continuation preserves $\mathbf C$ and $\mathbf C'$ at a definition boundary, while $\mathbf C''$ and the track-frame rotation rate may jump.
