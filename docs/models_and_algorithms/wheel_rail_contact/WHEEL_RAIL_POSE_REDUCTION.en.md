[中文](WHEEL_RAIL_POSE_REDUCTION.md)

# Wheel-rail pose reduction and irregularity inputs

This document explains how ORVD reduces the three-dimensional state of the non-spinning wheel-profile carrier in the track frame to the four pose scalars used by contact geometry: pair roll, pair yaw, lateral offset and vertical raise. The carrier is the wheelset on the rigid-wheelset path and the axle bridge on the independently rotating wheel path. The document also explains how alignment and vertical irregularities enter that reduction. The model is implemented by [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc), [roll_yaw_pitch.cc](../../../libs/wheel_rail_contact/src/roll_yaw_pitch.cc), [track_irregularity_field.cc](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc) and [rail_gauge_datum.cc](../../../libs/wheel_rail_contact/src/rail_gauge_datum.cc).

## 1. Scope

The reduction takes a carrier placement, a planar station rate, the lateral and vertical irregularity displacements with their time rates and the fixed geometry of one wheel-rail side. The source input type is named `WheelsetPlacement`; on the independently rotating wheel path it carries the axle bridge's non-spinning profile pose, not the independent wheel body's spin attitude. The output describes only relative pose in the current cross-section. It does not contain the full rigid pose of the rail profile, the effective sampling station or wheel-to-rail material-point velocity; the vehicle force assembly forms those quantities in parallel.

The spectrum and stochastic realization of irregularities are covered in [Track-irregularity spectra and their spatial random realization](../track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.en.md). Consumption of the four pose scalars is covered in [Contact geometry](CONTACT_GEOMETRY.en.md), and the contact frame and creepages in [Creepages and the contact frame](CREEPAGE_AND_CONTACT_FRAME.en.md).

## 2. Notation

Coordinates and signs follow [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md). The track frame is $T$, with $x$ along increasing track station, $y$ to the right and $z$ downward; $\mathbf e_1=\mathbf e_x$, $\mathbf e_2=\mathbf e_y$ and $\mathbf e_3=\mathbf e_z$ are the unit basis vectors along those three axes.

The four pose scalars keep the shared symbols $\varphi,\beta,d_y,d_z^{\uparrow}$ of the wheel-rail pose scalars section of the conventions document; this document coins no letters of its own for them. What it does introduce is the lateral and vertical separation $\ell_c,v_c$ inside the rail-cant frame, the two rail-side frames $T_c$ and $T_\ell$, the wheel-profile lateral datum $\sigma$ along the axle, the two stations $s_c,s_e$ and the planar track curvature $\kappa$. The lateral and vertical translation pair that contact geometry recovers by decoding the same four scalars (written $t_y,t_z$ there) is exactly $(\ell_c,v_c)$. The component $v_c=t_z$ is positive downward along the vertical axis of $T_c$, whereas $d_z^{\uparrow}$ is positive upward; they are negatives of one another only at $\varphi=0$ and are coupled to the lateral component at a general pose.

| Symbol | Meaning | Implementation quantity |
|---|---|---|
| $W$ | Non-spinning wheel-profile frame, with attitude $R_{TW}$ | attitude source of `WheelsetPlacement` |
| $y_w,z_w$ | Lateral and vertical coordinates in $T$ of the non-spinning wheel-profile carrier origin; on the rigid path this is the wheelset origin | `WheelsetPlacement::lateral_meters`, `vertical_meters` |
| $\phi_w,\psi_w$ | X-Z-Y roll and yaw resolved from $R_{TW}$ | `roll_radians`, `yaw_radians` |
| $\dot s$ | Signed planar station rate | `track_station_rate_meters_per_second` |
| $y_\epsilon,z_\epsilon$ | Lateral and vertical irregularity displacements | displacement fields of `TrackIrregularity` |
| $\dot y_\epsilon,\dot z_\epsilon$ | Time rates obtained while traversing the irregularity field | rate fields of `TrackIrregularity` |
| $\phi_r$ | Signed rail-cant roll for the selected side | `rail_roll_radians` |
| $y_r,z_r$ | Fixed rail-profile origin datum in $T$ | `rail_lateral_datum_meters`, `rail_vertical_datum_meters` |
| $\sigma$ | Signed wheel-profile lateral datum along the axle | `wheel_lateral_datum_meters` |
| $r_0,r$ | Nominal and pitch-corrected rolling radii | `nominal_rolling_radius_meters` and its pitch-corrected value |
| $\kappa$ | Planar track curvature, in this document | `curvature_radians_per_meter` |
| $T_c$ | Rail-cant frame, $R_{TT_c}=R_x(\phi_r)$ | canted rail cross-section |
| $T_\ell$ | Local tangential rail frame | combined alignment, vertical-slope and cant attitude |
| $\varphi,\beta$ | Pair roll and pair yaw, the latter being the angle of attack | `ContactPoseScalars::roll_radians`, `ContactPoseScalars::yaw_radians` |
| $d_y,d_z^{\uparrow}$ | Encoded lateral offset and upward-positive vertical raise in the four pose scalars | `ContactPoseScalars::lateral_offset_meters`, `ContactPoseScalars::vertical_raise_meters` |
| $\ell_c,v_c$ | Lateral and vertical separation inside the rail-cant frame | intermediate quantity formed before the encoding |
| $s_c,s_e$ | Shared carrier station and side-specific effective profile station | station quantities formed by force assembly |

## 3. Model

### 3.1 The three frames

The reduction deliberately distinguishes three frames:

1. The track frame $T$ carries placement of the non-spinning wheel-profile carrier, the rail-profile datum and irregularity displacement.
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

Because $z_T$ points downward, positive $dz/ds$ means that the rail descends with station and therefore gives a negative rotation about $+y_T$. Pair yaw is the angle of attack of the non-spinning wheel profile relative to the rail's actual lateral direction:

$$
\beta=\psi_w-\psi_\epsilon.
$$

Using a ratio of rates rather than passing a spatial slope separately concentrates the direction-of-travel sign in $\dot s$.

### 3.3 Local tangential rail frame and pair roll

The local tangential rail attitude is

$$
R_{TT_\ell}=R_z(\psi_\epsilon)R_y(\theta_\epsilon)R_x(\phi_r).
$$

The physical axle lies along the non-spinning wheel-profile frame's own lateral axis, so its direction in $T$ is $\mathbf a=R_{TW}\mathbf e_2$, that is

$$
\mathbf a=
\begin{bmatrix}
-\sin\psi_w\\
\cos\phi_w\cos\psi_w\\
\sin\phi_w\cos\psi_w
\end{bmatrix}.
$$

The wheel spins about that same axis, so $\mathbf a$ does not depend on the X-Z-Y pitch. Projecting $\mathbf a$ onto the lateral and vertical basis directions of the local tangential rail frame, $R_{TT_\ell}\mathbf e_2$ and $R_{TT_\ell}\mathbf e_3$, gives the pair roll

$$
\varphi=\operatorname{atan2}
\left(\left(R_{TT_\ell}\mathbf e_3\right)^{\mathsf T}\mathbf a,\,
      \left(R_{TT_\ell}\mathbf e_2\right)^{\mathsf T}\mathbf a\right).
$$

This is equivalent to resolving the X-Z-Y roll of the relative attitude $R_{TT_\ell}^{\mathsf T}R_{TW}$, although the implementation forms only the two required directions and dot products.

### 3.4 Pitch correction of rolling radius

Vertical irregularity pitches the cross-section relative to the profile carrier's vertical lever arm. Its projected radius is

$$
r=r_0\cos\theta_\epsilon.
$$

When pitch projection is selected, the expression above is used; without that correction, $r=r_0$. For a small slope, omitting the projection introduces a spurious separation of approximately $r_0\theta_\epsilon^2/2$.

### 3.5 Wheel and rail profile datums

The wheel-profile datum extends from the profile-carrier origin by $\sigma$ along the axle and then by $r$ along the carrier's own downward vertical direction. With $\rho=\sigma\cos\psi_w$,

$$
y_{wd}=y_w+\rho\cos\phi_w-r\sin\phi_w,
\qquad
z_{wd}=z_w+\rho\sin\phi_w+r\cos\phi_w.
$$

The carrier angles $\phi_w,\psi_w$ must be used because these lever arms belong to the non-spinning wheel-profile carrier rather than to the relative wheel-rail pose. The actual rail-profile datum is

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

Then encode it by the pose roll as the two offset scalars:

$$
\begin{bmatrix}d_y\\d_z^{\uparrow}\end{bmatrix}
=
\begin{bmatrix}
\cos\varphi & \sin\varphi\\
\sin\varphi & -\cos\varphi
\end{bmatrix}
\begin{bmatrix}\ell_c\\v_c\end{bmatrix}.
$$

The encoding matrix $\begin{bmatrix}\cos\varphi&\sin\varphi\\ \sin\varphi&-\cos\varphi\end{bmatrix}$ has determinant $-1$ and squares to the identity, so contact geometry decodes $(\ell_c,v_c)$ with the same expression. The sign of $d_z^{\uparrow}$ is upward-positive; a more negative $d_z^{\uparrow}$ therefore represents deeper geometric penetration.

### 3.7 Fixed geometry

The fixed quantities define the geometry of each wheel-rail side and are not state variables. Track gauge $G$ is measured between the two gauge faces at a specified depth below the rail crown. Let $\delta_{\mathrm R}$ and $\delta_{\mathrm L}$ be the gauge-face offsets from the respective right and left rail-profile origins, and let $\phi_c$ be the cant magnitude. Then

$$
y_r^{\mathrm R}=\frac{G}{2}+\delta_{\mathrm R},
\qquad
y_r^{\mathrm L}=-\left(\frac{G}{2}+\delta_{\mathrm L}\right),
\qquad
\phi_r^{\mathrm R}=-\phi_c,
\qquad
\phi_r^{\mathrm L}=+\phi_c.
$$

For each side, the authored rail-profile points are first rolled by the corresponding $\mp\phi_c$, sorted by their rolled lateral coordinate and connected in that sorted order as a piecewise-linear polyline. The center-facing intersection of this polyline with the measurement line determines $\delta_{\mathrm R}$ or $\delta_{\mathrm L}$. The gauge definition is therefore independent of the profile interpolant chosen later. It can be reduced to $\delta_{\mathrm R}=\delta_{\mathrm L}=\delta$ only for a mirror-symmetric profile or an equivalent symmetry of the selected intersections. The quantity $z_r$ is the vertical profile reference offset, $\sigma$ changes sign between sides and $r_0$ has the same physical meaning as the nominal radius used by contact geometry.

### 3.8 Longitudinal origin

There are two mathematical conventions for the longitudinal origin of a rail cross-section. A track-station convention leaves the supplied profile origin unchanged. A profile-coordinate convention translates it along the rail-profile frame's own $\mathbf e_1$ direction so that the local longitudinal coordinate from the wheel body origin to the rail-profile origin equals $\Delta s=s_e-s_c$. Both mathematical branches are implemented; the derivation below concerns only the profile-coordinate convention.

The attitude $R_{T\mathrm{rail}}$ is not merely a rail-cant rotation. Let $R_{IT(s)}$ be the attitude from the track frame at station $s$ to the inertial frame, and define the spatial-slope angles at the effective station by

$$
\widehat\psi_\epsilon=\operatorname{atan2}\!\left(y_\epsilon'(s_e),1\right),
\qquad
\widehat\theta_\epsilon=-\operatorname{atan2}\!\left(z_\epsilon'(s_e),1\right).
$$

Taking the track frame at $s_c$ as this document's $T$, the implemented rail-profile placement attitude is a product of five factors:

$$
R_{T\mathrm{rail}}
=R_{IT(s_c)}^{\mathsf T}R_{IT(s_e)}
 R_z(\widehat\psi_\epsilon)
 R_y(\widehat\theta_\epsilon)
 R_x(\phi_r).
$$

The first two factors convert between the track frames at the two stations; the remaining factors add alignment slope, vertical slope and rail cant in that order. The hatted angles come directly from spatial slopes and must not be identified unconditionally with the rate-ratio angles of section 3.2. They coincide during forward travel when the station rate is above its denominator floor.

Let the rail-profile origin placed at the effective station be $\mathbf o_c$, and let the wheel body origin be $\mathbf o_{\mathrm{wheel}}$. The profile-coordinate translation below holds for any orthogonal $R_{T\mathrm{rail}}$:

$$
\mathbf o_c'=\mathbf o_c+
R_{T\mathrm{rail}}
\begin{bmatrix}
\Delta s-\left(R_{T\mathrm{rail}}^{\mathsf T}(\mathbf o_c-\mathbf o_{\mathrm{wheel}})\right)_x\\0\\0
\end{bmatrix},
$$

and

$$
\left(R_{T\mathrm{rail}}^{\mathsf T}(\mathbf o_c'-\mathbf o_{\mathrm{wheel}})\right)_x=\Delta s.
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

The directional correction samples the lateral slope separately at the shared station $s_c$. The side-specific displacements, both slopes and their time rates are then resampled at the effective station $s_e$, with

$$
\dot y_\epsilon=y_\epsilon'(s_e)\dot s,\qquad
\dot z_\epsilon=z_\epsilon'(s_e)\dot s.
$$

The effective station depends on the shared carrier station, the corrected direction $\psi_e$ and the planar track curvature $\kappa$. After correcting yaw with the alignment slope at the carrier station,

$$
\psi_e=\psi_w-
\operatorname{SafeAtan2Ratio}\!\left(y_\epsilon'(s_c)\dot s,\dot s\right),
$$

$$
s_e=s_c+
\frac{-\sigma\sin\psi_e}
     {1-\kappa\left(y_w+\sigma\cos\psi_e\right)}.
$$

This is a two-stage choice: first determine direction, then select the side-specific cross-section. The corrected direction $\psi_e$ and the pair yaw $\beta$ are the same expression sampled at different stations: the former takes the alignment slope at $s_c$ and the latter at $s_e$, so in general the two differ, and the station selection does not read $\beta$ back. The line-domain classifications of $s_c$ and $s_e$ are independent: validity of the former does not imply validity of the latter. If either station lies outside $I_T$, the irregularity quantities for that stage are zero by the definition above even when an irregularity spline is itself defined there.

### 3.10 X-Z-Y attitude and rates

For $R=R_x(\phi)R_z(\psi)R_y(\theta)$, with matrix-entry indices below counted from one in the mathematical convention, the implementation resolves

$$
\phi=\operatorname{atan2}(R_{32},R_{22}),
$$

$$
\psi=\operatorname{atan2}\!\left(-R_{12},
\sqrt{R_{11}^2+R_{13}^2}\right),
\qquad
\theta=\operatorname{atan2}(R_{13},R_{11}).
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

`BuildContactPoseScalars` proceeds as follows:

1. Form $\psi_\epsilon,\theta_\epsilon$ from the irregularity rates and station rate.
2. Resolve $\varphi$ from the local tangential rail frame and the axle direction, and set $\beta=\psi_w-\psi_\epsilon$.
3. Compute the pitch-corrected radius $r$.
4. Place the wheel and rail profile datums and form their separation in $T$.
5. Transform the separation into $T_c$, then use the self-inverse reflection to encode $(d_y,d_z^{\uparrow})$.

The reduction itself contains a constant number of trigonometric operations, dot products and two-dimensional linear transformations. Effective-station selection and irregularity-spline evaluation occur upstream. X-Z-Y attitude and rate resolution are also constant-time operations.

## 5. Non-smoothness and theoretical applicability

- When $|\dot s|$ crosses the denominator floor, a slope angle switches between zero and its atan2 value; this switch is discontinuous when the numerator is nonzero.
- atan2 has a $\pm\pi$ branch on the negative real axis. During reverse travel, even the sign bit of a zero rate can select a different branch, so the forward smooth-track limit cannot simply be reused.
- On smooth track during forward travel, $\theta_\epsilon=0$ and $\beta=\psi_w$; when $\cos\psi_w>0$, $\varphi=\phi_w-\phi_r$.
- Each irregularity channel switches between the zero function and its spline at a boundary of the support intersection $I_q\cap I_T$; a nonzero endpoint value or slope creates a jump.
- X-Z-Y resolution is singular at $\cos\psi=0$, and the inverse rate map contains the same singularity.
- The effective-station map requires $1-\kappa(y_w+\sigma\cos\psi_e)\ne0$; as this denominator approaches zero, the local map from transverse offset to centerline station becomes geometrically singular.
- The rail is a geometric construction carried by the track, with no independent inertia or material-point velocity in this model. Irregularities alter rail-profile position and attitude, but this reduction does not generate rail material velocity.
- The same irregularity slope enters pose reduction through a rate ratio and may enter the rail rigid attitude directly as a spatial slope. They agree in forward travel above the speed floor; at standstill and in reverse they are different model continuations.

## 6. Implementation mapping

| Theoretical object | Main implementation |
|---|---|
| Four-scalar pose reduction and pair roll | `BuildContactPoseScalars`, in [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) |
| Application of the longitudinal-origin convention | `PlaceRailProfileLongitudinalOrigin`, in [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) |
| X-Z-Y attitude resolution and rate map | `ResolveRollYawPitch` and `ResolveRollYawPitchRates`, in [roll_yaw_pitch.cc](../../../libs/wheel_rail_contact/src/roll_yaw_pitch.cc) |
| Two-channel irregularity field | `TrackIrregularityField`, in [track_irregularity_field.cc](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc) |
| Gauge face and left/right rail datums | `ComputeRailGaugeDatum`, in [rail_gauge_datum.cc](../../../libs/wheel_rail_contact/src/rail_gauge_datum.cc) |
| Effective station, profile placement and input construction | `WheelRailContactForcePlan::CalcAppliedForces`, in [wheel_rail_contact_force_plan.cc](../../../libs/forces/src/wheel_rail_contact_force_plan.cc) |
| Downstream decoding of the four scalars | `ContactGeometrySolver::Solve`, in [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
