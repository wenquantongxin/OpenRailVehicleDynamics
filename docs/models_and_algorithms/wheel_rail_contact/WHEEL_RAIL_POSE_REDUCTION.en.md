[中文](WHEEL_RAIL_POSE_REDUCTION.md)

# Wheel–Rail Pose Reduction and Irregularity Inputs

This document explains how ORVD reduces the three-dimensional state of a wheelset in the track-profile frame to the four pose scalars used by contact geometry: pair roll, pair yaw, lateral offset, and vertical raise. It also explains how alignment and vertical irregularities enter that reduction. The model is implemented by [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc), [roll_yaw_pitch.cc](../../../libs/wheel_rail_contact/src/roll_yaw_pitch.cc), [track_irregularity_field.cc](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc), and [rail_gauge_datum.cc](../../../libs/wheel_rail_contact/src/rail_gauge_datum.cc).

## 1. Scope

The reduction takes a wheelset placement, a planar station rate, lateral and vertical irregularity displacements and their time rates, and the fixed geometry of one wheel–rail side. Its output describes only relative pose in the current cross-section. It does not contain the full rigid pose of the rail profile, the effective sampling station, or wheel-to-rail material-point velocity; the vehicle force assembly forms those quantities in parallel.

The spectrum and stochastic realization of irregularities are covered in the Chinese-only [Track Irregularity Spectra and Their Spatial Realization](../track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.md). Consumption of the four pose scalars is covered in [Contact Geometry](CONTACT_GEOMETRY.en.md), and the contact frame and creepages in [Creepages and the Contact Frame](CREEPAGE_AND_CONTACT_FRAME.en.md).

## 2. Notation

Coordinates and signs follow [Conventions and Notation](../CONVENTIONS_AND_NOTATION.en.md). The track-profile frame is $T$, with $x$ along increasing track station, $y$ to the right, and $z$ downward.

| Symbol | Meaning | Implementation quantity |
|---|---|---|
| $W$ | Non-spinning wheel-profile frame, with attitude $R_{TW}$ | attitude source of WheelsetPlacement |
| $y_w,z_w$ | Lateral and vertical coordinates of the wheelset body origin in $T$ | lateral_meters, vertical_meters |
| $\phi_w,\psi_w$ | X-Z-Y roll and yaw resolved from $R_{TW}$ | roll_radians, yaw_radians |
| $\dot s$ | Signed planar station rate | track_station_rate_meters_per_second |
| $y_\epsilon,z_\epsilon$ | Lateral and vertical irregularity displacements | displacement fields of TrackIrregularity |
| $\dot y_\epsilon,\dot z_\epsilon$ | Time rates obtained while traversing the irregularity field | rate fields of TrackIrregularity |
| $\phi_r$ | Signed rail-cant roll for the selected side | rail_roll_radians |
| $y_r,z_r$ | Fixed rail-profile origin datum in $T$ | rail_lateral_datum_meters, rail_vertical_datum_meters |
| $\sigma$ | Signed wheel-profile lateral datum along the axle | wheel_lateral_datum_meters |
| $r_0,r$ | Nominal and pitch-corrected rolling radii | nominal_rolling_radius_meters |
| $T_c$ | Rail-cant frame, $R_{TT_c}=R_x(\phi_r)$ | canted rail cross-section |
| $T_\ell$ | Local tangential rail frame | combined alignment, vertical-slope, and cant attitude |
| $\phi_p,\psi_p$ | Pair roll and pair yaw | ContactPoseScalars |
| $\ell_p,h_p$ | Pair-frame lateral offset and upward-positive vertical raise | ContactPoseScalars |
| $s_c,s_e$ | Shared carrier station and side-specific effective profile station | station quantities formed by force assembly |

## 3. Model

### 3.1 The three frames

The reduction deliberately distinguishes three frames:

1. The track-profile frame $T$ carries wheelset placement, the rail-profile datum, and irregularity displacement.
2. The rail-cant frame $T_c$ is the rail cross-section's own frame and expresses the lateral and vertical separation of the wheel and rail profile datums.
3. The local tangential rail frame $T_\ell$ additionally contains the alignment and vertical irregularity slopes and is the reference for pair roll.

These frames cannot be merged. Contact geometry compares two profile curves in the canted rail cross-section, whereas the common normal and angle of attack must be defined relative to the actual local rail tangent.

### 3.2 Protected slope angles and pair yaw

Slope angles are formed with a denominator floor:

$$
\operatorname{SafeAtan2Ratio}(a,b)=
\begin{cases}
0, & |b|<\varepsilon_{\dot s},\\
\operatorname{atan2}(a,b), & |b|\geq\varepsilon_{\dot s}.
\end{cases}
$$

The alignment and vertical irregularity angles are

$$
\psi_\epsilon=\operatorname{SafeAtan2Ratio}(\dot y_\epsilon,\dot s),
\qquad
\theta_\epsilon=-\operatorname{SafeAtan2Ratio}(\dot z_\epsilon,\dot s).
$$

Because $z_T$ points downward, positive $dz/ds$ means that the rail descends with station and therefore gives a negative rotation about $+y_T$. Pair yaw is the angle of attack relative to the rail's actual lateral direction:

$$
\psi_p=\psi_w-\psi_\epsilon.
$$

Using a ratio of rates rather than passing a spatial slope separately concentrates the direction-of-travel sign in $\dot s$.

### 3.3 Local tangential rail frame and pair roll

The local tangential rail attitude is

$$
R_{TT_\ell}=R_z(\psi_\epsilon)R_y(\theta_\epsilon)R_x(\phi_r).
$$

Let its columns 1 and 2 be $\mathbf c_1,\mathbf c_2$. The wheelset axle direction is column 1 of $R_{TW}$:

$$
\mathbf a=
\begin{bmatrix}
-\sin\psi_w\\
\cos\phi_w\cos\psi_w\\
\sin\phi_w\cos\psi_w
\end{bmatrix}.
$$

Pair roll is then

$$
\phi_p=\operatorname{atan2}
\left(\mathbf c_2^\mathsf T\mathbf a,\,
      \mathbf c_1^\mathsf T\mathbf a\right).
$$

This is equivalent to resolving the X-Z-Y roll of the relative attitude $R_{TT_\ell}^{\mathsf T}R_{TW}$, although the implementation forms only the two required columns and dot products.

### 3.4 Pitch correction of rolling radius

Vertical irregularity pitches the cross-section relative to the wheelset's vertical lever arm. Its projected radius is

$$
r=r_0\cos\theta_\epsilon.
$$

An independent model switch controls this correction; without it, $r=r_0$. For a small slope, omitting the projection introduces a spurious separation of approximately $r_0\theta_\epsilon^2/2$.

### 3.5 Wheel and rail profile datums

The wheel-profile datum extends by $\sigma$ along the axle and then by $r$ downward along the wheelset's own vertical direction. With $\rho=\sigma\cos\psi_w$,

$$
y_{wd}=y_w+\rho\cos\phi_w-r\sin\phi_w,
\qquad
z_{wd}=z_w+\rho\sin\phi_w+r\cos\phi_w.
$$

The wheelset angles $\phi_w,\psi_w$ must be used because these lever arms belong to the wheelset rather than to the relative wheel–rail pose. The actual rail-profile datum is

$$
y_{rd}=y_r+y_\epsilon,\qquad z_{rd}=z_r+z_\epsilon.
$$

### 3.6 Separation and pair encoding

First transform the separation in $T$ into the rail-cant frame:

$$
\begin{bmatrix}\ell_c\\v_c\end{bmatrix}
=
\begin{bmatrix}
\cos\phi_r & \sin\phi_r\\
-\sin\phi_r & \cos\phi_r
\end{bmatrix}
\begin{bmatrix}
y_{wd}-y_{rd}\\z_{wd}-z_{rd}
\end{bmatrix}.
$$

Then encode it in the pair frame:

$$
\ell_p=\cos\phi_p\,\ell_c+\sin\phi_p\,v_c,
\qquad
h_p=\sin\phi_p\,\ell_c-\cos\phi_p\,v_c.
$$

The second matrix has determinant $-1$ and squares to the identity, so contact geometry decodes $(\ell_c,v_c)$ with the same expression. The sign of $h_p$ is upward-positive; a more negative $h_p$ therefore represents deeper geometric penetration.

### 3.7 Fixed geometry

The fixed quantities define the geometry of each wheel–rail side and are not state variables. Track gauge $G$ is measured between the two gauge faces at a specified depth below the rail crown. Let $\delta_{\mathrm R}$ and $\delta_{\mathrm L}$ be the gauge-face offsets from the respective right and left rail-profile origins, and let $\phi_c$ be the cant magnitude. Then

$$
y_r^{\mathrm R}=\frac{G}{2}+\delta_{\mathrm R},
\qquad
y_r^{\mathrm L}=-\left(\frac{G}{2}+\delta_{\mathrm L}\right),
\qquad
\phi_r^{\mathrm R}=-\phi_c,
\qquad
\phi_r^{\mathrm L}=+\phi_c.
$$

For each side, the authored rail-profile points are first rolled by the corresponding $\mp\phi_c$, sorted by their rolled lateral coordinate, and connected in that sorted order as a piecewise-linear polyline. The centre-facing intersection of this polyline with the measurement line determines $\delta_{\mathrm R}$ or $\delta_{\mathrm L}$. The gauge definition is therefore independent of the profile interpolant chosen later. It can be reduced to $\delta_{\mathrm R}=\delta_{\mathrm L}=\delta$ only for a mirror-symmetric profile or an equivalent symmetry of the selected intersections. The quantity $z_r$ is the vertical profile reference offset, $\sigma$ changes sign between sides, and $r_0$ has the same physical meaning as the nominal radius used by contact geometry.

### 3.8 Longitudinal origin

There are two mathematical conventions for the longitudinal origin of a rail cross-section. A track-station convention leaves the supplied profile origin unchanged. A profile-coordinate convention translates it along its own first axis so that the local longitudinal coordinate from the wheel body origin to the rail-profile origin equals $d=s_e-s_c$.

Let the original rail-profile origin be $\mathbf o$, its attitude $R_{TR}$, and the wheel body origin $\mathbf w$. Then

$$
\mathbf o'=\mathbf o+
R_{TR}
\begin{bmatrix}
d-\left(R_{TR}^{\mathsf T}(\mathbf o-\mathbf w)\right)_x\\0\\0
\end{bmatrix},
$$

and

$$
\left(R_{TR}^{\mathsf T}(\mathbf o'-\mathbf w)\right)_x=d.
$$

The reference must be the wheel body origin. Substituting a wheel-profile datum that already contains the axle extension would apply the station correction twice under nonzero yaw.

### 3.9 Irregularity field and effective station

Lateral and vertical irregularities are represented by independent natural cubic splines $\eta_y(s)$ and $\eta_z(s)$; their knots and domains need not coincide. Let the closed domain of a channel be $I_q=[s_0^q,s_n^q]$ and the line's own defined interval be $I_T$. For either channel $q\in\{y,z\}$, the field that actually enters this contact assembly is

$$
q_\epsilon(s)=
\begin{cases}
\eta_q(s), & s\in I_q\cap I_T,\\
0, & \text{otherwise},
\end{cases}
\qquad
q_\epsilon'(s)=
\begin{cases}
\eta_q'(s), & s\in I_q\cap I_T,\\
0, & \text{otherwise}.
\end{cases}
$$

The irregularity wrapper first supplies zero displacement and zero slope outside $I_q$, and contact assembly independently zeros the channel outside $I_T$; this differs from the endpoint-value continuation of the underlying natural spline. At a boundary belonging to both closed intervals, it still returns the knot value and the one-sided interior slope. A nonzero value at either activation boundary therefore creates a discontinuity in the assembled field.

The directional correction samples the lateral slope separately at the shared station $s_c$. The side-specific displacements, both slopes, and their time rates are then resampled at the effective station $s_e$, with

$$
\dot y_\epsilon=y_\epsilon'(s_e)\dot s,\qquad
\dot z_\epsilon=z_\epsilon'(s_e)\dot s.
$$

The effective station depends on the shared carrier station, the corrected direction, and planar track curvature. After correcting yaw with the alignment slope at the carrier station,

$$
\psi_e=\psi_w-
\operatorname{SafeAtan2Ratio}\!\left(y_\epsilon'(s_c)\dot s,\dot s\right),
$$

$$
s_e=s_c+
\frac{-\sigma\sin\psi_e}
     {1-\kappa\left(y_w+\sigma\cos\psi_e\right)}.
$$

This is a two-stage choice: first determine direction, then select the side-specific cross-section. The line-domain classifications of $s_c$ and $s_e$ are independent: validity of the former does not imply validity of the latter. If either station lies outside $I_T$, the irregularity quantities for that stage are zero by the definition above even when an irregularity spline is itself defined there.

### 3.10 X-Z-Y attitude and rates

For $R=R_x(\phi)R_z(\psi)R_y(\theta)$, the implementation resolves

$$
\phi=\operatorname{atan2}(R_{21},R_{11}),
$$

$$
\psi=\operatorname{atan2}\!\left(-R_{01},
\sqrt{R_{00}^2+R_{02}^2}\right),
\qquad
\theta=\operatorname{atan2}(R_{02},R_{00}).
$$

The forward angular-velocity relation is

$$
\boldsymbol\omega
=\dot\phi\,\mathbf e_x
+\dot\psi\,R_x(\phi)\mathbf e_z
+\dot\theta\,R_x(\phi)R_z(\psi)\mathbf e_y.
$$

With $\tau=\omega_y\cos\phi+\omega_z\sin\phi$, its inverse is

$$
\dot\psi=-\omega_y\sin\phi+\omega_z\cos\phi,
\qquad
\dot\theta=\frac{\tau}{\cos\psi},
\qquad
\dot\phi=\omega_x+\tau\tan\psi.
$$

## 4. Algorithm structure

BuildContactPoseScalars proceeds as follows:

1. Form $\psi_\epsilon,\theta_\epsilon$ from the irregularity rates and station rate.
2. Resolve $\phi_p$ from the local tangential rail frame and the axle direction, and set $\psi_p=\psi_w-\psi_\epsilon$.
3. Compute the pitch-corrected radius $r$.
4. Place the wheel and rail profile datums and form their separation in $T$.
5. Transform the separation into $T_c$, then use the self-inverse reflection to encode $(\ell_p,h_p)$.

The reduction itself contains a constant number of trigonometric operations, dot products, and two-dimensional linear transformations. Effective-station selection and irregularity-spline evaluation occur upstream. X-Z-Y attitude and rate resolution are also constant-time operations.

## 5. Non-smoothness and theoretical applicability

- When $|\dot s|$ crosses the denominator floor, a slope angle switches between zero and its atan2 value; this switch is discontinuous when the numerator is nonzero.
- atan2 has a $\pm\pi$ branch on the negative real axis. During reverse travel, even the sign bit of a zero rate can select a different branch, so the forward smooth-track limit cannot simply be reused.
- On smooth track during forward travel, $\theta_\epsilon=0$ and $\psi_p=\psi_w$; when $\cos\psi_w>0$, $\phi_p=\phi_w-\phi_r$.
- Each irregularity channel switches between the zero function and its spline at a boundary of the support intersection $I_q\cap I_T$; a nonzero endpoint value or slope creates a jump.
- X-Z-Y resolution is singular at $\cos\psi=0$, and the inverse rate map contains the same singularity.
- The effective-station map requires $1-\kappa(y_w+\sigma\cos\psi_e)\ne0$; as this denominator approaches zero, the local map from transverse offset to centreline station becomes geometrically singular.
- The rail is a geometric construction carried by the track, with no independent inertia or material-point velocity in this model. Irregularities alter rail-profile position and attitude, but this reduction does not generate rail material velocity.
- The same irregularity slope enters pose reduction through a rate ratio and may enter the rail rigid attitude directly as a spatial slope. They agree in forward travel above the speed floor; at standstill and in reverse they are different model continuations.

## 6. Implementation mapping

| Theoretical object | Main implementation |
|---|---|
| Four-scalar pose reduction and pair roll | [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) |
| X-Z-Y attitude resolution and rate map | [roll_yaw_pitch.cc](../../../libs/wheel_rail_contact/src/roll_yaw_pitch.cc) |
| Two-channel irregularity field | [track_irregularity_field.cc](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc) |
| Gauge face and left/right rail datums | [rail_gauge_datum.cc](../../../libs/wheel_rail_contact/src/rail_gauge_datum.cc) |
| Effective station, profile placement, and input construction | [wheel_rail_contact_force_plan.cc](../../../libs/forces/src/wheel_rail_contact_force_plan.cc) |
| Downstream decoding of the four scalars | [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
