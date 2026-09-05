[中文](CONVENTIONS_AND_NOTATION.md)

# Conventions and Notation

This document unifies the frames, signs, station variable, pose, state and wrench notation used by the ORVD theory documents. Code links identify where a theoretical quantity is realised in this library; they are not an API or configuration reference.

## 1. Purpose and scope

Other theory documents follow these conventions and declare only the notation they add. A model that uses a different sign or expressed-in frame states that choice before the corresponding equations.

This document covers the theoretical quantities shared by track geometry, wheel-rail contact, multibody dynamics, force elements, system state and time integration. Software architecture, caches, workspaces, exceptions, tests and experimental terminology are outside its scope.

Subscripts identify frames. The last subscript of a vector identifies its expressed-in frame; a rotation $R_{AB}$ transforms components from frame B to frame A. Metres, seconds, radians, newtons and pascals follow SI.

## 2. Frames and signs

### 2.1 Track inertial frame I

The inertial frame I is fixed at the start of the line: `+x` points along increasing station there, `+y` points to the right of an observer facing increasing station, and `+z` points downward; the axes form a right-handed frame. Gravity acts along `+z`. The code definition is in [`track_inertial_frame.h`](../../libs/track_geometry/include/orvd/track_geometry/track_inertial_frame.h).

Let the centreline $\mathbf C(s)$ be parameterised by station $s$, with heading $\psi(s)$ and upward grade $g(s)$:

$$
\mathbf C'(s)=
\begin{bmatrix}
\cos\psi(s)\\
\sin\psi(s)\\
-g(s)
\end{bmatrix},
\qquad
\lVert\mathbf C'(s)\rVert=\sqrt{1+g(s)^2}
$$

The horizontal projection is therefore parameterised by unit arc length; the full three-dimensional derivative is a unit vector only when $g=0$.

### 2.2 Roll-free tangent frame and track frame T

The unit axes of the roll-free tangent frame are

$$
\mathbf x_0=\frac{\mathbf C'(s)}{\sqrt{1+g^2}},\qquad
\mathbf y_0=\begin{bmatrix}-\sin\psi&\cos\psi&0\end{bmatrix}^{\mathsf T},\qquad
\mathbf z_0=\mathbf x_0\times\mathbf y_0
$$

The track frame T is obtained by rotating the roll-free frame through the superelevation angle $\phi$ about its own `+x` axis:

$$
R_{IT}=R_{I0}R_x(\phi),\qquad
\phi=\arcsin\left(\frac{u}{b}\right)
$$

Here $u$ is signed superelevation and $b$ is the superelevation reference base length; the theoretical domain is $|u|<b$. Positive superelevation puts the right reference point lower and, because `+z` is downward, is a positive roll. The implementation is in [`track_geometry.cc`](../../libs/track_geometry/src/track_geometry.cc).

Planar curvature satisfies

$$
\frac{d\psi}{ds}=\kappa(s)
$$

Positive curvature turns right. Upward grade is positive, hence $dz/ds=-g$.

### 2.3 Profile frames and sides

The lateral coordinate of a wheel or rail profile is positive to the right of the track and its vertical coordinate is positive downward. An authored point list becomes a physical-side list through `ResolveForSide`: the right side retains the lateral sign; the left side is mirrored laterally and sorted into ascending order. The code location is [`profile_points.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/profile_points.h).

The non-spinning wheel-profile frame W uses `[circumferential x, lateral y, radial-down z]`. Its origin is the profile datum on the axle, and its orientation excludes wheel spin because the geometry of an axisymmetric profile is unchanged by spin. The letter P is reserved for the wheel-side application point of the contact force. It is defined by the wrench-reduction convention in the assembly model and is not assumed to be an exact material point on the undeformed surface of revolution.

The roll of a rail gauge datum changes sign with side. The lateral datum and gauge-face offset mirror between sides only when the input rail profile is itself mirror-symmetric; this is not an unconditional property of an arbitrary profile.

### 2.4 Planar curvature, grade and superelevation

Curvature $\kappa$, grade $g$ and superelevation $u$ all use planar projected station $s$ as their independent variable. Analytic alignment segments, seams and continuation outside the definition interval must retain this one station definition; $s$ must not be replaced by three-dimensional arc length $\ell$.

ORVD's Hermite cubic blend uses the normalised polynomial

$$
H(\xi)=3\xi^2-2\xi^3,\qquad 0\le\xi\le1
$$

Applied to a curvature transition, it gives a Bloss-type curvature law; it is not a clothoid, whose curvature is linear in station.

### 2.5 Wheel-rail pose scalars

`ContactPoseScalars` describes the intrinsic pose of the wheel profile relative to the rail profile through four scalars: roll $\varphi$, angle of attack $\beta$, lateral offset $d_y$ and vertical raise $d_z^{\uparrow}$. Vertical raise is positive upward, the only displacement sign here that runs opposite to the project's downward-positive `+z` convention. The code definition is in [`wheel_rail_pose.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_pose.h).

These quantities are measured between the wheel and rail profile datums. They are neither the translation between two rigid-body origins nor a complete relative attitude. The angle of attack also contains the lateral direction of motion and is not a single Euler-angle component of a rotation matrix.

### 2.6 Heights and penetration in contact geometry

Contact geometry writes the wheel and rail heights in one cross-sectional coordinate system and defines the overlap function as

$$
g_c(y)=z_w(y)-z_r(y)
$$

$g_c>0$ denotes geometric interpenetration of the undeformed surfaces. `vertical_penetration_meters` is the deepest overlap along the vertical axis of the track frame; `normal_penetration_meters` is the projection of the same depth onto the local rail-surface normal.

Each patch retains two angles: the common-normal angle contributes to construction of the longitudinal scale, whereas `rail_slope_angle_radians` defines the contact frame and the frame in which forces are expressed. They are not interchangeable.

### 2.7 Contact frame C

The contact frame C differs from the track frame T only by a rotation through the contact angle $\alpha$ about the longitudinal axis:

$$
R_{TC}=
\begin{bmatrix}
1&0&0\\
0&\cos\alpha&-\sin\alpha\\
0&\sin\alpha&\cos\alpha
\end{bmatrix}
$$

Its third axis is the contact normal. Longitudinal and lateral creepages and tangential forces are expressed in C; spin creepage has unit $\mathrm{m}^{-1}$. The implementation is in [`contact_creepage.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h).

### 2.8 Track-irregularity frame

Lateral and vertical irregularity displacements are expressed in the track frame, positive rightward and downward, and remain functions of station $s$. A spatial slope multiplied by station rate gives the corresponding sampling rate. That rate describes a vehicle sampling a fixed spatial field and is not automatically the velocity of a rail material point.

## 3. Station, arc length and rates

Station $s$ is arc length of the horizontal projection of the centreline. Three-dimensional arc length $\ell$ satisfies

$$
\frac{d\ell}{ds}=\sqrt{1+g^2},\qquad
\dot\ell=\sqrt{1+g^2}\,\dot s
$$

Planar station rate $\dot s$ and three-dimensional path speed $\dot\ell$ therefore cannot be exchanged on a non-zero grade. Direction angles in pose reduction are based on $\dot s$; the current creepage implementation uses three-dimensional path speed to construct its reference speed. The mappings are in [`wheel_rail_pose.cc`](../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) and [`contact_creepage.cc`](../../libs/wheel_rail_contact/src/contact_creepage.cc).

Let $\boldsymbol\omega_{IT}$ be the station derivative of track-frame orientation, expressed in I. It satisfies

$$
\frac{dR_{IT}}{ds}=\operatorname{skew}(\boldsymbol\omega_{IT})R_{IT}
$$

Projection of a spatial point onto centreline station follows a locally seeded branch rather than searching the entire line for a global closest point. Its objective and Newton update are given in [Track geometry and track frames](track_geometry/TRACK_GEOMETRY_AND_FRAMES.en.md).

## 4. Pose and kinematic notation

### 4.1 Single-letter frame notation

$R_{AB}$ is the rotation of B in A and satisfies

$$
\mathbf v_A=R_{AB}\mathbf v_B,\qquad R_{AC}=R_{AB}R_{BC}
$$

$\mathbf p_{AoBo\_A}$ is the position from the origin of A to the origin of B, expressed in A. $\boldsymbol\omega_{AB\_E}$ is the angular velocity of B relative to A, expressed in E; $\mathbf v_{ABo\_E}$ is the velocity of the origin of B relative to A, expressed in E.

### 4.2 Quaternions and free bodies

A free body uses seven generalised positions and six generalised velocities: position is `[quaternion w,x,y,z; origin position]`, and velocity is `[angular velocity; origin translational velocity]`. Four quaternion components carry three rotational degrees of freedom, so in general $n_q\ne n_v$. The code layout is in [`multibody_coordinate_ranges.h`](../../libs/multibody_model/include/orvd/multibody_model/multibody_coordinate_ranges.h).

### 4.3 Ball-RPY joint

Ball-RPY position uses a Z-Y-X composition:

$$
R_{FM}=R_z(\mathrm{yaw})R_y(\mathrm{pitch})R_x(\mathrm{roll})
$$

Its three generalised velocities are the physical angular velocity $\boldsymbol\omega_{FM\_F}$, not the derivatives of the three Euler angles; a configuration-dependent map relates the two.

### 4.4 X-Z-Y resolution of wheelset attitude

`ResolveRollYawPitch` uses an X-Z-Y sequence. For a rotation matrix $R$,

$$
\begin{aligned}
\mathrm{roll}&=\operatorname{atan2}(R_{21},R_{11}),\\
\mathrm{yaw}&=\operatorname{atan2}\left(-R_{01},\sqrt{R_{00}^2+R_{02}^2}\right),\\
\mathrm{pitch}&=\operatorname{atan2}(R_{02},R_{00})
\end{aligned}
$$

This corresponds to $R=R_x(\mathrm{roll})R_z(\mathrm{yaw})R_y(\mathrm{pitch})$ and differs from the Z-Y-X composition of Ball-RPY.

### 4.5 Position-derivative map

Generalised velocities and position derivatives are related by

$$
\dot q=N(q)v
$$

For a quaternion free body, the reverse map is a left pseudoinverse onto the quaternion tangent space; a component of $\dot q$ parallel to the quaternion does not represent a physical angular velocity.

## 5. State, forces and units

### 5.1 Continuous state `[q; v; z]`

The ORVD continuous state is

$$
x=\begin{bmatrix}q\\v\\z\end{bmatrix},\qquad
\dot x=\begin{bmatrix}N(q)v\\\dot v\\\dot z\end{bmatrix}
$$

$q$ and $v$ are the multibody generalised positions and velocities; $z$ contains internal states of force elements. The current source of $z$ is the internal force of a series spring-viscous damper. Time, numerical projection seeds and held controller quantities are not part of this continuous state.

### 5.2 Wrench reduction point and expressed-in frame

Write a spatial wrench as $\mathcal W_Q^E=(\boldsymbol\tau_Q^E,\mathbf f^E)$: force and moment are expressed in E and the moment is taken about Q. Moving the reduction point from Q to O gives

$$
\boldsymbol\tau_O^E=\boldsymbol\tau_Q^E+\mathbf p_{OQ}^E\times\mathbf f^E
$$

Changing the expressed-in frame and moving the reduction point are distinct operations.

Wheel-rail interaction is represented by a paired wrench. At a common reduction point and in a common expressed-in frame, the forces and moments of its two halves are exact opposites. The wheel-side force application point P and the rail-material reference point R used to evaluate relative velocity are distinct points.

### 5.3 Inverse-dynamics sign

The required generalised force in this library follows

$$
\tau_{\mathrm{required}}
=M(q)\dot v+C(q,v)v-\tau_{\mathrm{gravity}}-\tau_{\mathrm{damping}}
$$

$\tau_{\mathrm{gravity}}$ and $\tau_{\mathrm{damping}}$ denote applied generalised forces and therefore enter with a minus sign in the force required to realise a prescribed acceleration.

### 5.4 Units

| Quantity | Unit |
|---|---|
| length, station, semi-axis, penetration | m |
| time | s |
| velocity | m/s |
| angle | rad |
| angular velocity | rad/s |
| curvature, spin creepage | 1/m |
| force, moment | N, N·m |
| stress, elastic modulus | Pa |
| mass, rotational inertia | kg, kg·m² |

Grade, superelevation ratio, longitudinal and lateral creepages, and Poisson ratio are dimensionless.

## 6. Core Chinese-English terminology

| 中文 | English | Principal code name |
|---|---|---|
| 站位 | track station | `track_station_meters` |
| 三维弧长 | three-dimensional arc length | `arc_rate_meters_per_second` |
| 航向 | heading | `heading_radians` |
| 平面曲率 | planar curvature | `curvature_radians_per_meter` |
| 纵坡 | grade | `centerline_upward_grade` |
| 超高 | superelevation | `superelevation_meters` |
| 轨型系 | track frame | `TrackFramePose` |
| 型面 | profile | `ProfilePoints` |
| 轨距基准 | rail gauge datum | `RailGaugeDatum` |
| 接触斑 | contact patch | `ContactPatch` |
| 竖向穿透 | vertical penetration | `vertical_penetration_meters` |
| 等效穿透 | equivalent penetration | `equivalent_penetration_meters` |
| 公法线角 | common-normal angle | `common_normal_angle_radians` |
| 接触系 | contact frame | `ContactFrame` |
| 纵向蠕滑率 | longitudinal creepage | `longitudinal` |
| 横向蠕滑率 | lateral creepage | `lateral` |
| 自旋蠕滑率 | spin creepage | `spin_per_meter` |
| 法向力 | normal force | `NormalContactResult` |
| 切向力 | tangential force | `TangentialContactResult` |
| 扳手 | wrench | `SpatialWrench` |
| 广义位置 | generalised position | `q` |
| 广义速度 | generalised velocity | `v` |
| 内部力状态 | internal force state | `z` |
| 质量矩阵 | mass matrix | `CalcGeneralizedMassMatrix` |
| 逆动力学 | inverse dynamics | `CalcRequiredGeneralizedForces` |
| 前向动力学 | forward dynamics | `CalcGeneralizedVelocityDerivatives` |
| 数值 Jacobian | numerical Jacobian | `DenseFiniteDifferenceJacobianProvider` |
| 时间积分器 | time integrator | `ContinuousStateAdvancer` |
