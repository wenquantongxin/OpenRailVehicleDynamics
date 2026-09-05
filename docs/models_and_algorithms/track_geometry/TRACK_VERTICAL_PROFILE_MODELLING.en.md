[中文](TRACK_VERTICAL_PROFILE_MODELLING.md)

# Track vertical profiles and their three-dimensional coupling

This chapter explains how ORVD constructs a vertical profile from constant-grade segments, parabolic vertical curves, and circular vertical curves parameterized by planar-projected station; how explicit quintic seams connect adjacent segments; and how the vertical profile combines with planar curvature and superelevation to form the three-dimensional centerline and track frame. The formulas state the mathematical model and the analytic relations used by its code implementation.

## 1. Scope and notation

The track inertial frame is denoted by `I`: its `+x` axis follows increasing station at the line's starting point, `+y` points right when facing increasing station, and `+z` points downward. Station $s$ is arc length of the centerline's horizontal projection, not arc length of the three-dimensional centerline. See [Conventions and Notation](../CONVENTIONS_AND_NOTATION.en.md) for the complete coordinate conventions and [Track geometry and frames](TRACK_GEOMETRY_AND_FRAMES.en.md) for the general algorithms for planar curvature, centerline integration, and track frames.

| Symbol | Meaning |
|---|---|
| $[s_{\min},s_{\max}]$ | Finite definition interval of the line geometry |
| $\hat s$ | Local planar-projected station measured from zero inside a segment |
| $g(s)$ | Centerline upward grade; positive uphill along increasing station |
| $I_g(s)$ | Grade integral $\int_{s_{\min}}^s g(\sigma)\,d\sigma$ accumulated from the profile start |
| $\theta(s)=\arctan g(s)$ | Uphill inclination angle, on the principal branch $(-\pi/2,\pi/2)$ |
| $\psi(s)$, $\kappa(s)$ | Horizontal heading and planar curvature, with $d\psi/ds=\kappa$ |
| $\mathbf C(s)$ | Three-dimensional centerline position expressed in `I` |
| $\ell$ | Three-dimensional centerline arc length |
| $\kappa_v$ | Signed curvature of the vertical profile in the “planar-projected station–upward elevation” plane |
| $u(s)$, $b$, $\phi(s)$ | Signed superelevation, superelevation reference baselength, and track roll angle |

Track vertical irregularity is a local excitation superposed on the ideal line and is not part of what this chapter calls the vertical profile. A vertical curve specifically means a transition over which grade varies continuously; it is not a synonym for every nonzero-grade segment.

## 2. Planar-projected station and elevation

Because `+z` points downward while positive $g$ denotes an uphill grade, the centerline's elevation component satisfies

$$
\frac{dz}{ds}=-g(s),\qquad z(s)-z(s_{\min})=-I_g(s).
$$

The horizontal projection of the three-dimensional centerline derivative is a unit vector:

$$
\mathbf C'(s)=
\begin{bmatrix}
\cos\psi(s)\\
\sin\psi(s)\\
-g(s)
\end{bmatrix},
\qquad
\left\lVert\mathbf C'(s)\right\rVert=\sqrt{1+g(s)^2}.
$$

Planar-projected station and three-dimensional arc length are therefore related by

$$
\frac{d\ell}{ds}=\sqrt{1+g^2},\qquad \dot\ell=\sqrt{1+g^2}\,\dot s.
$$

When grade is nonzero, $s$, $\ell$, and their time rates are not interchangeable. All vertical-alignment segment lengths and seam widths are measured along $s$.

## 3. The vertical profile as a mathematical object

A vertical profile is a finite end-to-end sequence of segments. Each raw segment analytically supplies $g$, $g'$, $g''$, and $\int g\,ds$ accumulated from that segment's start; the complete profile forms $I_g$ by adding the integrals of preceding pieces to the local integral of the current piece. ORVD currently implements three raw segment types:

| Physical alignment | Parameters | Within-segment grade property |
|---|---|---|
| Constant-grade segment | $L>0$, $g_0$ | $g=g_0$ |
| Parabolic vertical curve (PL2) | $L>0$, $g_1\ne g_2$ | $g$ is linear in $\hat s$ |
| Circular vertical curve (CIR) | $L>0$, $g_1\ne g_2$ | $\sin\theta$ is linear in $\hat s$ and vertical-profile curvature is constant |

A PL2 or CIR with equal endpoint grades describes the same degenerate limit as a constant-grade segment, so the model uses the constant-grade segment as the unique representation. Adjacent boundaries not covered by a seam window have continuous grade values; continuity of higher derivatives depends on the two segment types.

The profile itself is defined only on $[s_{\min},s_{\max}]$. Behaviour outside the interval, described in Section 8, belongs to the assembled line geometry and is not an extrapolation of the final analytic vertical segment.

## 4. Vertical-alignment elements

### 4.1 Constant-grade segment

A segment of length $L$ and grade $g_0$ satisfies, for $0\le\hat s\le L$,

$$
g(\hat s)=g_0,\qquad g'(\hat s)=g''(\hat s)=0,
$$

$$
\int_0^{\hat s}g(u)\,du=g_0\hat s,
\qquad
z(\hat s)-z(0)=-g_0\hat s.
$$

Zero grade is a special case of a constant-grade segment; the segment itself is not a vertical curve.

### 4.2 Parabolic vertical curve (PL2)

A PL2 connects grades $g_1$ and $g_2$, with grade varying linearly in planar-projected station:

$$
g(\hat s)=g_1+\frac{g_2-g_1}{L}\hat s,
\qquad 0\le\hat s\le L.
$$

Let $q=(g_2-g_1)/L$. Then

$$
g'=q,\qquad g''=0,
$$

$$
\int_0^{\hat s}g(u)\,du=g_1\hat s+\frac{q}{2}\hat s^2,
\qquad
z(\hat s)-z(0)=-g_1\hat s-\frac{q}{2}\hat s^2.
$$

“Parabolic” means that elevation is quadratic in planar-projected station. It does not mean that the inclination angle $\theta$ is linear in station, because

$$
\frac{d\theta}{ds}=\frac{q}{1+g^2},
\qquad
\frac{d^2\theta}{ds^2}=-\frac{2gq^2}{(1+g^2)^2}.
$$

A PL2 has smooth elevation and grade within the segment, but its $g'$ generally jumps where it directly meets a constant-grade segment. The explicit seam in Section 5 supplies higher junction smoothness when required.

### 4.3 Circular vertical curve (CIR)

A CIR is an exact circular arc in the “planar-projected station–upward elevation” plane. It is parameterized by $L$, $g_1$, and $g_2$, with

$$
\theta_i=\arctan g_i,
\qquad
\sin\theta_i=\frac{g_i}{\operatorname{hypot}(1,g_i)},
\qquad
\cos\theta_i=\frac{1}{\operatorname{hypot}(1,g_i)}.
$$

The signed inverse radius and radius are

$$
\rho=\frac{1}{R}=\frac{\sin\theta_2-\sin\theta_1}{L},
\qquad
R=\frac{L}{\sin\theta_2-\sin\theta_1}.
$$

Within the segment, define

$$
q(\hat s)=\sin\theta_1+\rho\hat s.
$$

Because finite grade has $\cos\theta>0$, the grade follows from

$$
\sin\theta(\hat s)=q(\hat s),
\qquad
\cos\theta(\hat s)=\sqrt{1-q(\hat s)^2},
\qquad
g(\hat s)=\frac{q(\hat s)}{\sqrt{1-q(\hat s)^2}}
$$

Elevation can be written as

$$
z(\hat s)-z(0)=R\left[\cos\theta(\hat s)-\cos\theta_1\right],
$$

while the code uses the mathematically equivalent grade-integral form that avoids multiplying a large radius by a difference of nearby cosines:

$$
\int_0^{\hat s}g(u)\,du
=\hat s\,
\frac{q(\hat s)+\sin\theta_1}
{\sqrt{1-q(\hat s)^2}+\cos\theta_1},
\qquad
z(\hat s)-z(0)=-\int_0^{\hat s}g(u)\,du.
$$

The within-segment derivatives are

$$
\frac{d\theta}{ds}=\frac{\rho}{\cos\theta},
\qquad
\frac{d^2\theta}{ds^2}=\frac{\rho^2\sin\theta}{\cos^3\theta},
$$

$$
g'=\frac{\rho}{\cos^3\theta},
\qquad
g''=\frac{3\rho^2\sin\theta}{\cos^5\theta}.
$$

The grade of a CIR is not linear, nor is its inclination angle; $\sin\theta$ is the linear quantity. The sign of $\rho$ distinguishes the two directions of vertical bending, so no separate crest-or-sag flag is needed.

### 4.4 Inclination rate and vertical-profile curvature

For any differentiable grade,

$$
\frac{d\theta}{ds}=\frac{g'}{1+g^2}.
$$

Let $\ell_v$ be arc length of the parametric curve $(s,h(s))$, where $dh/ds=g$. Then $d\ell_v/ds=\sqrt{1+g^2}$. This has the same numerical value as $d\ell/ds$ for the three-dimensional centerline, but the two curvatures refer to different geometric objects. Signed vertical-profile curvature is

$$
\kappa_v=\frac{d\theta}{d\ell_v}=\frac{g'}{(1+g^2)^{3/2}}.
$$

For a CIR, substituting $g'=\rho/\cos^3\theta$ and $1+g^2=1/\cos^2\theta$ gives

$$
\kappa_v=\rho=\frac{1}{R},
$$

so a CIR has constant curvature in the vertical-profile plane even though $d\theta/ds$ varies with grade.

## 5. Segment boundaries and quintic seams

### 5.1 Boundaries without seams

Without a seam, adjacent raw segments have equal grade at their common boundary $s_b$. Consequently $z$ and $z'=-g$ are continuous and the centerline tangent is continuous; $g'$ or $g''$ may still jump, so the centerline's second or higher derivatives can have one-sided limits without a shared two-sided value.

No implicit smoothing is added at such a boundary. At an internal piece boundary the profile uses the derivative of the right-hand piece, while at the complete profile endpoint it uses the derivative of the left-hand piece; these returned derivatives are explicitly one-sided quantities.

### 5.2 Quintic grade bridge

An explicit seam window is centered on a raw-segment boundary $s_b$, has full width $w>0$, and has endpoints $s_L=s_b-w/2$ and $s_R=s_b+w/2$. The left endpoint data come from the left raw segment and the right endpoint data from the right raw segment:

$$
\mathbf d=
\begin{bmatrix}
g_L & w g'_L & w^2 g''_L & g_R & w g'_R & w^2 g''_R
\end{bmatrix},
\qquad
\xi=\frac{s-s_L}{w}\in[0,1].
$$

Grade inside the window is the unique quintic Hermite polynomial

$$
g_{\mathrm{seam}}(s)=\sum_{r=0}^{5}d_r H_r(\xi),
$$

$$
\begin{aligned}
H_0(\xi)&=1-10\xi^3+15\xi^4-6\xi^5,\\
H_1(\xi)&=\xi-6\xi^3+8\xi^4-3\xi^5,\\
H_2(\xi)&=\tfrac12\xi^2-\tfrac32\xi^3+\tfrac32\xi^4-\tfrac12\xi^5,\\
H_3(\xi)&=10\xi^3-15\xi^4+6\xi^5,\\
H_4(\xi)&=-4\xi^3+7\xi^4-3\xi^5,\\
H_5(\xi)&=\tfrac12\xi^3-\xi^4+\tfrac12\xi^5.
\end{aligned}
$$

These basis functions make the seam match $(g,g',g'')$ from the two raw formulas at $s_L$ and $s_R$, respectively. The seam completely replaces raw grade inside the window; the raw boundary $s_b$ is only its topological center, not a formula switch of the seam polynomial.

### 5.3 Seam integral and smoothness order

The seam contribution to accumulated grade is analytic as well:

$$
\int_{s_L}^{s}g_{\mathrm{seam}}(\sigma)\,d\sigma
=w\sum_{r=0}^{5}d_r\int_0^{\xi}H_r(\eta)\,d\eta.
$$

Elevation therefore uses the area of the replacement seam rather than continuing to accumulate the covered raw segments. Seam grade is $C^2$ at both window endpoints; since $z'=-g$, the corresponding elevation has continuous derivatives through third order there, while its fourth derivative can generally jump.

A seam window lies at an internal boundary with raw segments on both sides, its half-width does not pass beyond either adjacent raw segment, and the interiors of distinct seam windows do not overlap. These geometric conditions ensure that each station is defined by exactly one raw formula or one seam formula.

## 6. Three-dimensional coupling with planar alignment

### 6.1 Centerline integration

Planar curvature $\kappa(s)$ and vertical grade $g(s)$ share the same planar-projected station. Heading, horizontal position, and vertical position are

$$
\psi(s)=\int_{s_{\min}}^s\kappa(\sigma)\,d\sigma,
$$

$$
\begin{bmatrix}x(s)\\y(s)\end{bmatrix}
=\int_{s_{\min}}^s
\begin{bmatrix}\cos\psi(\sigma)\\\sin\psi(\sigma)\end{bmatrix}d\sigma,
\qquad
z(s)=-I_g(s),
$$

with the implementation taking $\mathbf C(s_{\min})=\mathbf 0$ and $\psi(s_{\min})=0$. The vertical profile is not appended to a two-dimensional curve drawn along a fixed world `x` direction; it and the horizontal centerline with changing heading form one spatial curve through their shared station.

The second station derivative is

$$
\mathbf C''(s)=
\begin{bmatrix}
-\kappa\sin\psi\\
\kappa\cos\psi\\
-g'
\end{bmatrix}.
$$

Vertical raw-segment and seam boundaries therefore form the partition of second-order centerline geometry. A raw boundary covered by a seam window remains part of the line topology, but the actual formula switches occur at the two seam-window endpoints.

### 6.2 Planar, vertical-profile, and spatial curvature

Let $\mathbf T$ be the three-dimensional unit tangent, and use the roll-free axes $\mathbf y_0$ and $\mathbf z_0$ from Section 7. Then

$$
\mathbf T=\frac{\mathbf C'}{\sqrt{1+g^2}},
$$

$$
\frac{d\mathbf T}{d\ell}
=\kappa\cos^2\theta\,\mathbf y_0-\kappa_v\,\mathbf z_0,
$$

$$
\left\lVert\frac{d\mathbf T}{d\ell}\right\rVert
=\sqrt{\kappa^2\cos^4\theta+\kappa_v^2}.
$$

Planar curvature, vertical-profile curvature, and three-dimensional centerline curvature are therefore three distinct quantities. Superelevation rolls axes about the centerline and changes neither the centerline itself nor this spatial curvature.

### 6.3 Analytic vertical integration and horizontal quadrature

Vertical position is always formed from the analytic $I_g$ supplied by `TrackVerticalProfile`. Horizontal position depends only on $\kappa$: constant-curvature intervals use a closed-form circular chord, while varying-curvature intervals use panelized fixed-order Gauss–Legendre quadrature. Centerline nodes lie on the ordered union of planar-curvature breakpoints and vertical raw-segment and seam boundaries, so each node interval does not cross a piece boundary of $\mathbf C''$. Superelevation does not change the centerline, so its boundaries do not independently add centerline-integration nodes.

This separation means that horizontal integration panels do not replace the analytic accuracy of the vertical profile; conversely, closed-form PL2 or CIR elevation does not remove the numerical approximation in varying-curvature horizontal integration.

## 7. Coupling with superelevation and the track frame

### 7.1 Roll-free track frame

Let $n=\sqrt{1+g^2}$ and $\mathbf t=\mathbf C'$. The roll-free track-frame axes are

$$
\mathbf x_0=\frac{\mathbf t}{n},
\qquad
\mathbf y_0=
\begin{bmatrix}
-\sin\psi\\
\cos\psi\\
0
\end{bmatrix},
\qquad
\mathbf z_0=\mathbf x_0\times\mathbf y_0.
$$

$\mathbf y_0$ remains horizontal and points to the right of the line. When grade is nonzero, $\mathbf z_0$ is not the downward axis of the inertial frame.

### 7.2 Superelevation roll

ORVD defines signed superelevation $u$ as the separation of the two endpoints of a superelevation reference baseline along $\mathbf z_0$, and uses

$$
\phi=\arcsin\left(\frac{u}{b}\right),
\qquad |u|<b.
$$

The final track-profile frame `T` is obtained by rolling the roll-free frame about its own longitudinal axis:

$$
R_{IT}=R_{I0}R_x(\phi),
\qquad
R_x(\phi)=
\begin{bmatrix}
1&0&0\\
0&\cos\phi&-\sin\phi\\
0&\sin\phi&\cos\phi
\end{bmatrix}.
$$

Positive superelevation places the right reference point lower. If the baseline endpoints are separated by $b$ along the track-profile lateral axis, their separation along the downward inertial `z` axis is

$$
\Delta z_I=b\sin\phi\,(\mathbf z_0\cdot\mathbf e_z)
=\frac{u}{\sqrt{1+g^2}}.
$$

Thus, at nonzero grade, the input $u$ is not the two points' height difference along inertial `z`.

### 7.3 Track-frame station rotation rate

The track-frame station rotation rate is constructed from the same $\kappa$, $g$, $g'$, $u$, and $u'$ as the pose. Derivatives of the roll-free axes are

$$
\mathbf t'=\mathbf C'',
\qquad
n'=\frac{gg'}{n},
\qquad
\mathbf x_0'=\frac{\mathbf t'}{n}-\mathbf x_0\frac{n'}{n},
$$

$$
\mathbf y_0'=
\begin{bmatrix}
-\kappa\cos\psi\\
-\kappa\sin\psi\\
0
\end{bmatrix},
\qquad
\mathbf z_0'=\mathbf x_0'\times\mathbf y_0+\mathbf x_0\times\mathbf y_0'.
$$

Define

$$
\boldsymbol\omega_0=\frac12\left(
\mathbf x_0\times\mathbf x_0'
+\mathbf y_0\times\mathbf y_0'
+\mathbf z_0\times\mathbf z_0'
\right),
$$

$$
\phi'=\frac{u'/b}{\sqrt{1-(u/b)^2}},
\qquad
\boldsymbol\omega_{IT}=\boldsymbol\omega_0+\phi'\mathbf x_0.
$$

Every prime here denotes differentiation with respect to $s$, so $\boldsymbol\omega_{IT}$ is rotation per unit station and satisfies

$$
\frac{dR_{IT}}{ds}=\operatorname{skew}(\boldsymbol\omega_{IT})R_{IT}.
$$

### 7.4 Theoretical boundary of superelevation datums

The current implementation uses a centerline datum: superelevation rotates the track frame without translating $\mathbf C(s)$ as $u$ changes. The superelevation reference baselength $b$ is the length of an abstract reference baseline, not nominal track gauge or a wheel–rail profile-placement distance.

**Theory only:** If the inner or outer rail is used as a fixed datum, changing superelevation must also translate the centerline in space so that the designated rail reference point remains fixed. Those alternatives require additional definitions of the fixed side, reference point, and displacement direction; the current `TrackGeometry` does not represent that centerline translation.

## 8. Definition interval and irregularity layering

### 8.1 Three-dimensional tangent extension

Planar curvature, the vertical profile, and superelevation share a finite interval $[s_{\min},s_{\max}]$ when assembled. For a finite station outside that interval, let $s_b$ be the nearest boundary. The assembled line geometry uses the three-dimensional tangent extension

$$
\mathbf C(s)=\mathbf C(s_b)+(s-s_b)\mathbf C'(s_b),
\qquad
\mathbf C'(s)=\mathbf C'(s_b),
\qquad
\mathbf C''(s)=\mathbf 0.
$$

Outside the interval, boundary values of $\psi$, $g$, $u$, and $\phi$ are held, while $\kappa=g'=u'=0$. The centerline and track attitude are therefore continuous at a definition boundary, but the centerline second derivative and track-frame rotation rate may jump. This is an explicit extension of the assembled line, not an extrapolation uniquely implied by its final PL2 or CIR.

### 8.2 Ideal vertical alignment and irregularity

The ideal vertical profile determines centerline elevation, grade, and the track frame. Vertical irregularity is a local displacement field superposed at a rail-profile reference and does not rewrite $g(s)$, $I_g(s)$, or the ideal centerline. The displacement rate produced as a vehicle samples irregularity along the line belongs to wheel–rail relative motion; it is not a new constant-grade segment, PL2, or CIR.

## 9. Approximations, non-smoothness, and applicability

- This model always uses planar-projected station as its independent variable. Constant-grade segments, PL2, and CIR obey the distinct analytic relations in Section 4; similar names do not make those formulas interchangeable.
- Grade, derivatives, and integrals of constant-grade, PL2, CIR, and quintic-seam pieces are evaluated analytically within each piece. Numerical approximation occurs in horizontal-centerline integration for varying planar curvature, not in the vertical-segment formulas.
- An unseamed boundary guarantees only continuity of grade; $g'$ and higher derivatives may jump. A quintic seam matches through $g''$ at its window endpoints, while $g'''$ may generally still jump.
- A CIR uses the finite-grade branch $\theta\in(-\pi/2,\pi/2)$ and its exact circular definition requires $L>0$ and $g_1\ne g_2$. A PL2 likewise uses $g_1\ne g_2$ to remain nondegenerate; equal-grade cases use a constant-grade segment.
- Superelevation uses centerline roll and satisfies $|u|<b$; a fixed inner- or outer-rail datum appears only as the theoretical boundary in Section 7.4.
- Three-dimensional tangent extension outside the domain, explicit seam windows, and separation of ideal alignment from irregularity are parts of the model rather than unique consequences automatically implied by local curve formulas.

## 10. Core code mapping

The following table only locates the core implementations that carry the mathematical relations in this chapter.

| Theoretical object | Primary implementation | Source file |
|---|---|---|
| Grade, derivatives, and analytic integrals of constant-grade, PL2, and CIR segments | `TrackVerticalProfile` and the three named vertical segment types | [`track_vertical_profile.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_vertical_profile.h), [`track_vertical_profile.cc`](../../../libs/track_geometry/src/track_vertical_profile.cc) |
| Quintic $(g,g',g'')$ seam | `internal::BuildQuinticHermiteCoefficients` | [`track_profile_quintic.cc`](../../../libs/track_geometry/src/track_profile_quintic.cc) |
| Three-dimensional centerline, tangent extension, and track-frame kinematics | `TrackGeometry`, `EvaluateTrackFrame` | [`track_geometry.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_geometry.h), [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| Value types for pose, centerline derivative, and station rotation rate | `TrackFramePose`, `TrackFrameKinematics` | [`track_frame_pose.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_frame_pose.h) |
