[中文](CONVENTIONS_AND_NOTATION.md)

# Conventions and notation

This document unifies the frames, signs, station variable, pose, state and wrench notation used by the ORVD theory documents. Code links identify where a theoretical quantity is realized in this library; they are not an API or configuration reference.

## 1. Purpose and scope

Other theory documents follow these conventions and declare only the notation they add. A model that uses a different sign or expressed-in frame states that choice before the corresponding equations. Section 6 collects the Chinese and English terms shared across the documents.

This document covers the theoretical quantities shared by track geometry, wheel-rail contact, multibody dynamics, force elements, system state and time integration. Software architecture, caches, workspaces, exceptions, tests and experimental terminology are outside its scope.

Letters identify frames. A rotation $R_{AB}$ transforms components from frame B to frame A; a vector's expressed-in frame is stated by its final subscript, a superscript or the text that first introduces it. Meters, seconds, radians, newtons and pascals follow SI.

## 2. Frames and signs

### 2.1 Track inertial frame I

The inertial frame I is fixed at the start of the line: `+x` points along increasing station there, `+y` points to the right of an observer facing increasing station, and `+z` points downward; the axes form a right-handed frame. Gravity acts along `+z`. The code definition is in [`track_inertial_frame.h`](../../libs/track_geometry/include/orvd/track_geometry/track_inertial_frame.h).

Let the centerline $\mathbf C(s)$ be parameterized by station $s$, with heading $\psi(s)$ and upward grade $g(s)$:

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

The horizontal projection is therefore parameterized by unit arc length; the full three-dimensional derivative is a unit vector only when $g=0$.

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

The lateral coordinate of a wheel or rail profile is positive to the right of the track and its vertical coordinate is positive downward. The side sign is $\varsigma$: $+1$ for the right side and $-1$ for the left (`WheelSide`). An authored point list becomes a physical-side list through `ResolveForSide`: both sides are sorted into ascending lateral order, the right side retaining the authored lateral sign and the left side being mirrored laterally first. An authored list may be written in descending order, so the ordering of the right side can change as well. The code location is [`profile_points.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/profile_points.h), in `ProfilePoints::ResolveForSide`.

The non-spinning wheel-profile frame W uses `[circumferential x, lateral y, radial-down z]`. Its origin is the profile datum on the axle, and its orientation excludes wheel spin because the geometry of an axisymmetric profile is unchanged by spin. The letter `P` is reserved for the wheel-side application point of the contact force. It is defined by the wrench-reduction convention in the assembly model and is not assumed to be an exact material point on the undeformed surface of revolution.

The rail-cant magnitude used by pose reduction and profile placement is $\phi_c$, a positive number, and the signed cant roll of one rail of the pair is

$$
\phi_r=-\varsigma\,\phi_c
$$

that is, the right rail leans toward the track center, which with the lateral axis pointing right and the vertical axis pointing down is a negative roll about the longitudinal axis (`RailGaugeDatum::roll_radians`). The lateral datum and gauge-face offset mirror between sides only when the authored rail profile is itself mirror-symmetric about its own lateral zero $y=0$; this is not an unconditional property of an arbitrary profile. The code location is [`rail_gauge_datum.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/rail_gauge_datum.h), in `ComputeRailGaugeDatum`.

### 2.4 Rail-side reference frames and profile placement

The rail-cant frame $T_c$ is the cross-sectional reference used to encode the pose scalars and solve the two-dimensional contact geometry. Relative to the current track frame T, it contains only the side-signed rail-cant roll:

$$
R_{TT_c}=R_x(\phi_r)
$$

The lateral and vertical separation between the wheel and rail profile datums, the height comparison in the two-dimensional contact geometry and the vertical penetration together with its projection onto the normal are all expressed in this cross-sectional reference. The frame $T_c$ is not the rail profile's complete placement in three-dimensional space: the placed profile origin is written $\mathbf o_c$ and its attitude is $R_{T\mathrm{rail}}$. In addition to rail cant, that attitude contains the alignment and vertical-irregularity slope angles and the attitude difference between the track frames at the carrier station and the effective profile station; it therefore cannot be written as $R_{TT_c}$.

The local tangential rail frame $T_\ell$ is a second reference used to extract pose roll. On top of the pure rail-cant attitude it adds two angles formed from the spatial irregularity slopes, the alignment angle $\psi_\epsilon$ and the vertical irregularity angle $\theta_\epsilon$:

$$
R_{TT_\ell}=R_z(\psi_\epsilon)R_y(\theta_\epsilon)R_x(\phi_r)
$$

The sign of $\theta_\epsilon$ is taken so that a rail descending along the station is a negative rotation about $+y$. Within pose reduction, $T_\ell$ is used only to measure the pose roll $\varphi$; the lateral and vertical separation is not measured in it. When both irregularity slopes vanish, $T_\ell$ coincides with $T_c$ at the same station. The construction of the two angles is given in [Wheel-rail pose reduction and irregularity inputs](wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.en.md).

### 2.5 Planar curvature, grade and superelevation

Curvature $\kappa$, grade $g$ and superelevation $u$ all use planar projected station $s$ as their independent variable. Analytic alignment segments, seams and continuation outside the definition interval must retain this one station definition; $s$ must not be replaced by three-dimensional arc length $\ell$.

ORVD's Hermite cubic blend uses the normalized polynomial

$$
H(\xi)=3\xi^2-2\xi^3,\qquad 0\le\xi\le1
$$

Applied to a curvature transition, it gives a Bloss-type curvature law; it is not a clothoid, whose curvature is linear in station.

### 2.6 Wheel-rail pose scalars

`ContactPoseScalars` describes the intrinsic pose of the wheel profile relative to the rail profile through four scalars: roll $\varphi$, angle of attack $\beta$, lateral offset $d_y$ and vertical raise $d_z^{\uparrow}$. Vertical raise is positive upward, the only displacement sign here that runs opposite to the project's downward-positive `+z` convention. The code definition is in [`wheel_rail_pose.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_pose.h), in `ContactPoseScalars`.

These quantities are measured between the wheel and rail profile datums. They are neither the translation between two rigid-body origins nor a complete relative attitude. The lateral and vertical separation between the wheel and rail profile datums is measured in the rail-cant frame $T_c$; the two offsets $d_y$ and $d_z^{\uparrow}$ are obtained from that separation through one further self-inverse reflection involving the roll $\varphi$, so only at $\varphi=0$ does $d_y$ equal the lateral separation in $T_c$ and $d_z^{\uparrow}$ the negated vertical separation in $T_c$; the encoding is given in [Wheel-rail pose reduction and irregularity inputs](wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.en.md). The roll $\varphi$ is measured against the local tangential rail frame $T_\ell$. The angle of attack $\beta$ is the wheelset yaw less the local rail direction supplied by the alignment irregularity; the implementation forms that direction as $\operatorname{atan2}(\dot\eta_y,\dot s)$. It is not a single Euler-angle component of a rotation matrix.

### 2.7 Heights and penetration in contact geometry

Contact geometry writes the wheel and rail heights in one cross-sectional coordinate system, that of the rail-cant frame $T_c$. With $z_r(Y)$ the rail cross-sectional height and $H(Y)$ the single-valued upper envelope of the wheel surface projected onto that cross-section, the vertical overlap function is

$$
g(Y)=H(Y)-z_r(Y)
$$

$g>0$ denotes geometric interpenetration of the undeformed surfaces; contact detection applies a further gap threshold $\epsilon$ as the strict inequality $g>\epsilon$, given in [Contact geometry](wheel_rail_contact/CONTACT_GEOMETRY.en.md). The $H(Y)$ here is that projected envelope and is not the same function as the Hermite blending polynomial $H(\xi)$ of Section 2.5; likewise $g(Y)$ takes the cross-sectional lateral coordinate as its argument and is not the grade $g(s)$ of Section 2.1, which takes station.

Vertical penetration $\delta_v$ (`vertical_penetration_meters`) is the deepest overlap along the vertical axis of the rail-cant frame $T_c$; normal penetration $\delta_n$ (`normal_penetration_meters`) is the projection of the same depth onto the local rail-surface normal; equivalent penetration $\delta_{\mathrm{eq}}$ (`equivalent_penetration_meters`) is the penetration the normal force law actually uses.

Each patch retains two angles: the common-normal angle $\gamma$ (`common_normal_angle_radians`) contributes to construction of the longitudinal scale, whereas the contact-frame angle $\alpha$ (`rail_slope_angle_radians`) defines the contact frame and the frame in which forces are expressed. They are not interchangeable. The rail surface's own slope angle at the patch centroid is written $\alpha_r$ and is internal to contact geometry.

Rolling radii use a lowercase letter: the nominal rolling radius is $r_0$ (`nominal_rolling_radius_meters`) and the local rolling radius at the contact is $r$ (`rolling_radius_meters`). Rotation matrices use an uppercase $R$; the material reference point `R` appears only as a point label or a subscript on its position vector.

### 2.8 Contact frame C

The contact frame C differs from the track frame T only by a rotation through the contact-frame angle $\alpha$ about the longitudinal axis:

$$
R_{TC}=
\begin{bmatrix}
1&0&0\\
0&\cos\alpha&-\sin\alpha\\
0&\sin\alpha&\cos\alpha
\end{bmatrix}
$$

Its third axis is the contact normal. The contact-geometry chain separately writes its own rail-cant magnitude as $c_r$ (`ContactGeometryConfiguration::rail_cant_radians`), so that

$$
\alpha=\alpha_r-\varsigma c_r.
$$

The current implementation deliberately stores $c_r$ separately from the pose chain's $\phi_c$. They are the same kind of geometric angle but must not be treated as numerically identical in a derivation. Contact geometry forms the common-normal angle $\gamma$ after removing pair roll and angle of attack, and the normal law uses it to construct a longitudinal scale; $\gamma$ and $\alpha$ are distinct.

Longitudinal and lateral creepages are written $\xi_x$ and $\xi_y$ (`longitudinal`, `lateral`), and spin creepage is written $\xi_{sp}$ (`spin_per_meter`) with unit $\mathrm{m}^{-1}$; all three, and the tangential forces, are expressed in C. The code definitions of `ContactFrame` and `Creepages` are in [`contact_creepage.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h). The construction of $R_{TC}$ is implemented in [`contact_creepage.cc`](../../libs/wheel_rail_contact/src/contact_creepage.cc), in `MakeContactFrame`.

### 2.9 Track-irregularity frame

Lateral and vertical irregularity displacements are expressed in the track frame, positive rightward and downward, and remain functions of station $s$. A spatial slope multiplied by station rate gives the corresponding sampling rate. That rate describes a vehicle sampling a fixed spatial field and is not automatically the velocity of a rail material point.

## 3. Station, arc length and rates

Station $s$ is arc length of the horizontal projection of the centerline. Three-dimensional arc length $\ell$ satisfies

$$
\frac{d\ell}{ds}=\sqrt{1+g^2},\qquad
\dot\ell=\sqrt{1+g^2}\,\dot s
$$

Planar station rate $\dot s$ and three-dimensional path speed $\dot\ell$ therefore cannot be exchanged on a non-zero grade. Direction angles in pose reduction are based on $\dot s$; the current creepage implementation uses three-dimensional path speed to construct its reference speed. The mappings are in [`wheel_rail_pose.cc`](../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) and [`contact_creepage.cc`](../../libs/wheel_rail_contact/src/contact_creepage.cc).

Let $\boldsymbol\omega_{IT}$ be the station derivative of track-frame orientation, expressed in I. It satisfies

$$
\frac{dR_{IT}}{ds}=\operatorname{skew}(\boldsymbol\omega_{IT})R_{IT}
$$

Projection of a spatial point onto centerline station follows a locally seeded branch rather than searching the entire line for a global closest point. Its objective and Newton update are given in [Line geometry and track frames](track_geometry/TRACK_GEOMETRY_AND_FRAMES.en.md).

## 4. Pose and kinematic notation

### 4.1 Single-letter frame notation

$R_{AB}$ is the rotation of B in A and satisfies

$$
\mathbf v_A=R_{AB}\mathbf v_B,\qquad R_{AC}=R_{AB}R_{BC}
$$

$\mathbf p_{AoBo\_A}$ is the position from the origin of A to the origin of B, expressed in A. $\boldsymbol\omega_{AB\_E}$ is the angular velocity of B relative to A, expressed in E; $\mathbf v_{ABo\_E}$ is the velocity of the origin of B relative to A, expressed in E. The wrench notation of Section 5.2 uses an explicit superscript for the expressed-in frame. An established abbreviation such as $\boldsymbol\omega_{IT}$ in Section 3 instead states its expressed-in frame when first introduced.

Letter subscripts on a rotation matrix name frames, whereas two numeric subscripts name an entry: $R_{ij}$ is the entry in row $i$ and column $j$, with $i,j\in\{1,2,3\}$, so rows and columns are numbered from 1. A column is referred to as $R\mathbf e_j$, where $\mathbf e_j$ is the $j$-th standard basis vector.

### 4.2 Quaternions and free bodies

A free body uses seven generalized positions and six generalized velocities: position is `[quaternion w,x,y,z; origin position]`, and velocity is `[angular velocity; origin translational velocity]`, both expressed in the world frame. Four quaternion components carry three rotational degrees of freedom, so in general $n_q\ne n_v$. The range types are in [`multibody_coordinate_ranges.h`](../../libs/multibody_model/include/orvd/multibody_model/multibody_coordinate_ranges.h). The two component orderings above are stated in [`multibody_model.h`](../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h), at `MultibodyModel::GetFreeBodyPositionRange` and `MultibodyModel::GetFreeBodyVelocityRange`.

### 4.3 Ball-RPY joint

Ball-RPY position uses a Z-Y-X composition:

$$
R_{FM}=R_z(\mathrm{yaw})R_y(\mathrm{pitch})R_x(\mathrm{roll})
$$

Its three generalized velocities are the physical angular velocity $\boldsymbol\omega_{FM\_F}$, not the derivatives of the three Euler angles; a configuration-dependent map relates the two.

### 4.4 X-Z-Y resolution of wheelset attitude

`ResolveRollYawPitch` uses an X-Z-Y sequence. For a rotation matrix $R$,

$$
\begin{aligned}
\mathrm{roll}&=\operatorname{atan2}(R_{32},R_{22}),\\
\mathrm{yaw}&=\operatorname{atan2}\left(-R_{12},\sqrt{R_{11}^2+R_{13}^2}\right),\\
\mathrm{pitch}&=\operatorname{atan2}(R_{13},R_{11})
\end{aligned}
$$

This corresponds to $R=R_x(\mathrm{roll})R_z(\mathrm{yaw})R_y(\mathrm{pitch})$ and differs from the Z-Y-X composition of Ball-RPY.

### 4.5 Position-derivative map

Generalized velocities and position derivatives are related by

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

$q$ and $v$ are the multibody generalized positions and velocities; $z$ contains internal states of force elements. The current source of $z$ is the internal force of a series spring-viscous damper. Time, numerical projection seeds and held controller quantities are not part of this continuous state.

### 5.2 Wrench reduction point and expressed-in frame

Write a spatial wrench as $\mathcal W_Q^E=(\boldsymbol\tau_Q^E,\mathbf f^E)$: force and moment are expressed in E and the moment is taken about Q. Moving the reduction point from Q to O gives

$$
\boldsymbol\tau_O^E=\boldsymbol\tau_Q^E+\mathbf p_{OQ}^E\times\mathbf f^E
$$

Changing the expressed-in frame and moving the reduction point are distinct operations.

Wheel-rail interaction is represented by a paired wrench. At a common reduction point and in a common expressed-in frame, the forces and moments of its two halves are exact opposites. The wheel-side force application point `P` and the rail-material reference point `R` used to evaluate relative velocity are distinct points; their position vectors are written $\mathbf x_P$ and $\mathbf x_R$ so that neither is confused with a rotation matrix $R$.

### 5.3 Inverse-dynamics sign

The required generalized force in this library follows

$$
\tau_{\mathrm{required}}
=M(q)\dot v+C(q,v)v-\tau_{\mathrm{gravity}}-\tau_{\mathrm{damping}}
$$

$\tau_{\mathrm{gravity}}$ and $\tau_{\mathrm{damping}}$ denote applied generalized forces and therefore enter with a minus sign in the force required to realize a prescribed acceleration. The $\tau$ of this section is a generalized force and is not the same quantity as the spatial moment $\boldsymbol\tau$ of Section 5.2. $M(q)$ is the system mass matrix and $C(q,v)$ the system-level Coriolis and centrifugal matrix; a force element's scalar stiffness and damping always take the lowercase $k$ and $c$.

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

Grade, superelevation ratio, longitudinal creepage, lateral creepage and Poisson ratio are dimensionless.

The letter $N$ has two established uses: in the multibody and numerical chain it is the position-derivative map $N(q)$ of Section 4.5, while in the wheel-rail contact chain it is the total normal force $N$, whose elastic and damping parts are $F_e$ and $F_d$ (`NormalContactResult`). State dimension is written $n_x$.

## 6. Core Chinese-English terminology

| 中文 | English | Principal code name |
|---|---|---|
| 站位 | track station | `track_station_meters` |
| 三维弧长 | three-dimensional arc length | — (present in the code only as its rate) |
| 三维路径速率 | three-dimensional path rate | `path_rate_meters_per_second` (named `arc_rate_meters_per_second` at the contact-model entry) |
| 航向 | heading | `heading_radians` |
| 平面曲率 | planar curvature | `curvature_radians_per_meter` |
| 纵坡 | grade | `centerline_upward_grade` |
| 超高 | superelevation | `superelevation_meters` |
| 无侧滚切向系 | roll-free tangent frame | — |
| 轨型系 | track frame | `TrackFramePose` |
| 型面 | profile | `ProfilePoints` |
| 轨距基准 | rail gauge datum | `RailGaugeDatum` |
| 轨底坡 | rail cant | `rail_cant_radians` |
| 轨底坡系 | rail-cant frame | — |
| 局部切向钢轨系 | local tangential rail frame | — |
| 方向不平顺角 | alignment angle | — |
| 高低不平顺角 | vertical irregularity angle | — |
| 冲角 | angle of attack | `ContactPoseScalars::yaw_radians` |
| 接触斑 | contact patch | `ContactPatch` |
| 竖向穿透 | vertical penetration | `vertical_penetration_meters` |
| 法向穿透 | normal penetration | `normal_penetration_meters` |
| 等效穿透 | equivalent penetration | `equivalent_penetration_meters` |
| 公法线角 | common-normal angle | `common_normal_angle_radians` |
| 接触坐标系角 | contact-frame angle | `rail_slope_angle_radians` |
| 接触坐标系 | contact frame | `ContactFrame` |
| 纵向蠕滑率 | longitudinal creepage | `longitudinal` |
| 横向蠕滑率 | lateral creepage | `lateral` |
| 自旋蠕滑率 | spin creepage | `spin_per_meter` |
| 蠕滑系数 | creepage coefficients | `KalkerCoefficients` |
| 法向力 | normal force | `NormalContactResult` |
| 切向力 | tangential force | `TangentialContactResult` |
| 扳手 | wrench | `SpatialWrench` |
| 广义位置 | generalized position | `q` |
| 广义速度 | generalized velocity | `v` |
| 内部力状态 | internal force state | `z` |
| 质量矩阵 | mass matrix | `CalcGeneralizedMassMatrix` |
| 逆动力学 | inverse dynamics | `CalcRequiredGeneralizedForces` |
| 前向动力学 | forward dynamics | `CalcGeneralizedVelocityDerivatives` |
| 数值 Jacobian | numerical Jacobian | `DenseFiniteDifferenceJacobianProvider` |
| 时间积分器 | time integrator | `ContinuousStateAdvancer` |
