[中文](STARTUP_STATE_ASSEMBLY.md)

# Startup state assembly

This chapter explains how the startup state of one vehicle is assembled from a set of physically meaningful initial quantities into the initial value $y_0=[q_0;v_0;z_0]$ of the system's continuous state, and which quantities given alongside the record enter the equations of motion without being state. The published chapters on line geometry, irregularity spectra, wheel-rail contact and force elements are components; [Multibody equations of motion](MULTIBODY_EQUATIONS_OF_MOTION.en.md) (the multibody chapter below) combines the body wrenches those components produce with the multibody layer's own gravity and joint damping into the complete right-hand side $[N(q)v;\dot v;\dot z]$, this chapter constructs the initial value from which that right-hand side is advanced starting at $t_0$, [Contact force plan kinematics](../wheel_rail_contact/CONTACT_FORCE_PLAN_KINEMATICS.en.md) (the contact-plan chapter below) explains how the multibody state becomes the contact input of every wheel-rail interface, and [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md) (the time-integration chapter below) explains how the solver advances the state and differences the complete right-hand side. This chapter is not a solver: it solves for no static equilibrium, no preload and no contact, and only writes quantities that were resolved elsewhere into a state by fixed geometric and kinematic rules. It is organized around three questions: which physically meaningful initial quantities are given (Section 2); how they form $q_0$, $v_0$ and $z_0$ (Sections 3 to 5); and which quantities influence the equations without belonging to the state (Section 6). The types of the startup record are in [`resolved_startup_state.h`](../../../libs/configuration/include/orvd/configuration/resolved_startup_state.h); the assembly is performed by `AssembleResolvedInitialContext` in [`assemble_resolved_initial_context.cc`](../../../libs/configuration/src/assemble_resolved_initial_context.cc).

## 1. Scope and notation

### 1.1 Objects

The assembly takes four inputs: an assembled vehicle, that is, the rigid-body tree of Section 2.1 of the multibody chapter (free bodies, revolute joints, Ball-RPY joints, weld joints) together with its force elements and an optional wheel-rail contact force plan; a line; the station $s_{\mathrm{ref}}$ at which the vehicle's layout reference body is placed; and a startup record, a set of initial quantities already resolved elsewhere. Its output is the continuous state $y_0=[q_0;v_0;z_0]$ at the startup time $t_0=0$, together with the accompanying parameters that are written into the runtime context without being state. The three blocks of the continuous state follow Section 5.1 of [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md) (the conventions below): $q$ consists of the seven generalized positions of each free body and the generalized positions of each coordinate-carrying joint, $v$ of the six generalized velocities of each free body and the joint rates, and $z$ of the internal force of each series spring-viscous-damper element. Every free body and every coordinate-carrying joint owns exactly one segment of $q$ and one of $v$, and every series element owns exactly one component of $z$; the segments do not overlap and together they tile the three vectors, and the assembly fills them segment by segment by name, relying on no declaration order.

Startup records come in two kinds only: a record with common spin generation, which in addition to the per-body and per-joint quantities states one common effective rolling radius and uses it with the vehicle start speed to derive part of the angular velocities and joint rates; and an explicitly stated record, in which every free body's angular velocity and every revolute joint's rate are stated directly and there is no common spin quantity. Correspondingly, wheel-rail carriers come in two kinds only: rigid wheelsets, and axle bodies carrying independently rotating wheels. The former usually go with records of the first kind and the latter with explicit records, but the kind of record is a property of the record itself and is not inferred from the kind of carrier.

### 1.2 Notation

| Symbol | Meaning | Source |
|---|---|---|
| I, $T(s)$, B, F, M | Track inertial frame; track frame at station $s$; body frame of a free body; parent and child frames of a Ball-RPY joint | Conventions §2.1–2.2, §4.3 |
| $R_{IT}(s)$, $\mathbf C(s)$ | Attitude and origin of the track frame, the origin being the centerline point | Conventions §2.1–2.2; line-geometry chapter §3.4 |
| $\boldsymbol\omega_{IT}$ | Rotation rate of the track frame per meter of station, expressed in I | Conventions §3 |
| $s_{\mathrm{ref}}$ | Station of the vehicle's layout reference body, the argument of this run | New here |
| $\Delta s_{\mathrm{mech},B}$, $\Delta s_{\mathrm{res},B}$ | Mechanical station offset (vehicle definition) and resolved station offset (startup record) of free body $B$ | New here |
| $s_B$ | Startup station of free body $B$ | New here |
| $\mathsf q_{AB}$ | Unit quaternion corresponding to $R_{AB}$, components in the order $(w,x,y,z)$ | Conventions §4.2 |
| $y_B$, $z_B$, $\boldsymbol\rho_B$ | Lateral and vertical offsets of the body origin in $T(s_B)$, and $\boldsymbol\rho_B=(0,y_B,z_B)^{\mathsf T}$ | New here |
| $\boldsymbol\omega_{\mathrm{expl},B}$ | Explicitly stated angular velocity of B relative to I, expressed in B | New here |
| $\mathbf c_B$ | Common-spin coefficient vector, dimensionless, expressed in B | New here |
| $V_0$ | Vehicle start speed, the longitudinal component in $T(s_B)$ of the inertial velocity of every body origin | New here |
| $v_{y,B}$, $v_{z,B}$ | Lateral and vertical components in $T(s_B)$ of the inertial velocity of the body origin | New here |
| $r_{\mathrm{eff}}$, $\Omega_0$ | Common effective rolling radius; common spin quantity $\Omega_0=V_0/r_{\mathrm{eff}}$ | New here |
| $\theta_j$, $\dot\theta_j$, $\lambda_j$ | Angle and rate of revolute joint $j$, and the dimensionless multiplier of the rate with respect to $\Omega_0$ | New here |
| $(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})$, $\boldsymbol\omega_{FM\_F}$ | The three angles of a Ball-RPY joint and the angular velocity of the child frame relative to the parent frame, expressed in the parent frame | Conventions §4.3 |
| $F_i$ | Internal force of the $i$-th series element, the axial force on its reference end | Series chapter §1.1 |
| $\mathbf f_0$ | Nominal force of a translational element, expressed in its reference-end frame | Translational chapter §2.3 |
| $z_r$ | Vertical datum of the rail-profile origin in $T$, along $+z_T$ | Pose-reduction chapter §2 |
| $g$, $\mathbf g_I$ | Magnitude of gravitational acceleration and the gravity vector $g\,\mathbf e_z^I$ | Conventions §2.1; multibody chapter §2.4 |
| $[q;v;z]$, $y_0$, $N(q)$ | Continuous state, its initial value and the position-derivative map | Conventions §5.1; time-integration chapter §1.1 |

### 1.3 Relation to the conventions and to the other chapters

Frames, rotation matrices and angular-velocity subscripts follow Section 4.1 of the conventions: $R_{TB}$ maps body-frame components to track-frame components, $\boldsymbol\omega_{IB\_B}$ is the angular velocity of B relative to I expressed in B, and $\mathbf v_{IBo\_T}$ is the velocity of the origin of B relative to I expressed in T. The short names line-geometry chapter, series chapter, translational chapter and pose-reduction chapter refer to [Line geometry and track frames](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.en.md), [Series spring-viscous-damper element](../force_elements/SERIES_SPRING_VISCOUS_DAMPER.en.md), [Three-axis translational spring-damper element](../force_elements/TRANSLATIONAL_SPRING_DAMPER.en.md) and [Wheel-rail pose reduction and irregularity inputs](../wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.en.md) respectively. The following symbols are local to this chapter and are declared here. The sans-serif $\mathsf q$ is a unit quaternion, not the generalized position $q$. $V_0$ is the vehicle start speed, written in upper case to keep it apart from the velocity block $v$ of the state and its initial value $v_0$. $g$ denotes only the magnitude of gravitational acceleration, as in Section 2.4 of the multibody chapter; the grade $g(s)$ of Section 2.1 of the conventions does not appear as a symbol in this chapter and is referred to only through $\lVert\mathbf C'(s)\rVert$. $\Omega_0$ is the scalar common spin quantity, unrelated to the angular-velocity vector $\boldsymbol\omega$ and distinct from the wheel pitch rate $\Omega$ of the contact-plan and creepage chapters. $\mathbf c_B$ is a dimensionless coefficient vector, unrelated to the center-of-mass offset $\mathbf c$ of the multibody chapter and to the force-element damping $c$ of Section 5.3 of the conventions. $\theta_j$ is the angle of a revolute joint, unrelated to the vertical irregularity angle $\theta_\epsilon$ of Section 2.4 of the conventions. $F_i$ follows the force state $F$ of the series chapter and is unrelated to the elastic part $F_e$ of the normal force in the contact chain. $\dot s_B$ appears only in the derivation of Section 4.4; it is a quantity derived from $V_0$, not a state.

## 2. The given initial quantities

This section states what the startup record gives in terms of physical quantities, not in terms of the record's fields. The record carries no absolute mileage: where the vehicle sits on the line is the argument $s_{\mathrm{ref}}$ of this run, and the record states only the vehicle's own attitudes, offsets and motion, so one record can be assembled at any finite station of a line.

**Each free body $B$.** Six groups of quantities, all stated in the local track frame $T(s_B)$ at the body's own station $s_B$ or in the body frame B: the attitude $R_{TB}$, given as the unit quaternion $\mathsf q_{TB}$; the resolved station offset $\Delta s_{\mathrm{res},B}$, that is, how far the resolution moved the body along the line, the mechanical layout itself belonging to the vehicle definition and not being repeated in the record; the lateral offset $y_B$ and vertical offset $z_B$ of the body origin in $T(s_B)$, positive to the right and downward as in Section 2.2 of the conventions; the explicit inertial angular velocity $\boldsymbol\omega_{\mathrm{expl},B}$, the angular velocity of B relative to I expressed in B; the common-spin coefficient vector $\mathbf c_B$, dimensionless and expressed in B, zero in an explicit record; and the lateral component $v_{y,B}$ and vertical component $v_{z,B}$ in $T(s_B)$ of the inertial velocity of the body origin. The longitudinal component is not given per body; see the vehicle-level quantities below.

**Each revolute joint $j$.** The angle $\theta_j$, and one generation rule for the rate: either the rate $\dot\theta_{\mathrm{expl},j}$ is given directly, or a dimensionless multiplier $\lambda_j$ is given and the rate equals $\lambda_j$ times the common spin quantity. The two rules produce the same physical quantity, see Section 5.1.

**Each Ball-RPY joint.** The three angles $(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})$, composed as $R_{FM}=R_z(\mathrm{yaw})R_y(\mathrm{pitch})R_x(\mathrm{roll})$ by Section 4.3 of the conventions, and the angular velocity $\boldsymbol\omega_{FM\_F}$ of the child frame M relative to the parent frame F, expressed in F. A weld joint has no coordinates and no entry in the record.

**Each series spring-viscous-damper element $i$.** The internal force $F_i$: the axial force along the element's acting axis on its reference end, with the sign of Section 1.1 of [Series spring-viscous-damper element](../force_elements/SERIES_SPRING_VISCOUS_DAMPER.en.md).

**Each three-axis translational spring-damper element.** The nominal force $\mathbf f_0$: a constant vector acting on the reference end and expressed in the reference-end frame, with the meaning of Section 2.3 of [Three-axis translational spring-damper element](../force_elements/TRANSLATIONAL_SPRING_DAMPER.en.md).

**Vehicle-level quantities.** The start speed $V_0>0$, the longitudinal component in its local track frame of the inertial velocity of every free-body origin; the running direction, of which there is currently one definition only, starting toward increasing station; for a record with common spin generation, the effective rolling radius $r_{\mathrm{eff}}>0$; the rail-profile vertical datum offset, a signed length along $+z_T$; and the magnitude $g$ of the gravitational acceleration under which the record was resolved.

The table below groups these quantities by destination; the answer to the third question is its last column.

| Initial quantity | Frame in which it is stated | Destination |
|---|---|---|
| $\mathsf q_{TB}$, $\Delta s_{\mathrm{res},B}$, $y_B$, $z_B$ | $T(s_B)$ | Free-body segment of $q_0$ (Section 3) |
| $\boldsymbol\omega_{\mathrm{expl},B}$, $\mathbf c_B$ | B | Free-body segment of $v_0$ (Section 4) |
| $V_0$, $v_{y,B}$, $v_{z,B}$ | $T(s_B)$ | Free-body segment of $v_0$ (Section 4) |
| $\theta_j$; $\dot\theta_{\mathrm{expl},j}$ or $\lambda_j$ | Joint axis | Revolute segments of $q_0$ and $v_0$ (Section 5) |
| $(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})$, $\boldsymbol\omega_{FM\_F}$ | F | Ball-RPY segments of $q_0$ and $v_0$ (Section 5) |
| $F_i$ | Element acting axis | $z_0$ (Section 5) |
| $r_{\mathrm{eff}}$ | — | Enters only the generation $\Omega_0=V_0/r_{\mathrm{eff}}$, not the state (Section 4) |
| $\mathbf f_0$ | Reference-end frame | Context parameter (Section 6) |
| Rail-profile vertical datum offset | The $+z$ axis of $T$ | Contact-geometry constant $z_r$ (Section 6) |
| $g$, running direction | I | Premise and definition, not state (Section 6) |

## 3. Station and attitude

### 3.1 The station as a sum of three terms

The startup station of free body $B$ is a sum of three terms:

$$
s_B=s_{\mathrm{ref}}+\Delta s_{\mathrm{mech},B}+\Delta s_{\mathrm{res},B}.
$$

$s_{\mathrm{ref}}$ is the station at which the vehicle's layout reference body is placed, the argument of this run; $\Delta s_{\mathrm{mech},B}$ is the body's mechanical station offset from the layout reference body, a property of the vehicle definition that does not change wherever on a line the vehicle is placed; $\Delta s_{\mathrm{res},B}$ is the amount by which the resolution moved the body along the line, a property of the startup record. Each term has one source: the mechanical layout keeps a single authority, and the record states only what the resolution changed.

### 3.2 The complete track frame at every station

At station $s$ the line provides the track frame $T(s)$: the attitude $R_{IT}(s)$ and the origin $\mathbf C(s)$, the point on the centerline, defined in Sections 2.1 to 2.2 of the conventions and Section 3.4 of [Line geometry and track frames](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.en.md). $R_{IT}(s)$ contains the full effect of heading, grade and superelevation; for a station outside the line's definition interval the three-dimensional tangent continuation of Section 3.5 of the line-geometry chapter is used. The assembly evaluates the complete $T(s_B)$ at each free body's own station $s_B$ and straightens or levels no line: startups on curves, grades and superelevation use the same formulas.

### 3.3 Attitude and position

The attitude of the body frame B in I is the composition of two rotations, implemented as the product of two unit quaternions:

$$
R_{IB}=R_{IT}(s_B)\,R_{TB},
\qquad
\mathsf q_{IB}=\mathsf q_{IT}(s_B)\,\mathsf q_{TB},
$$

where $\mathsf q_{IT}(s_B)$ is the unit quaternion corresponding to $R_{IT}(s_B)$ and the product is the quaternion product, whose composition order is that of the matrix product. The quaternion norm is multiplicative, and $\mathsf q_{TB}$ and $\mathsf q_{IT}$ are both unit quaternions, so $\mathsf q_{IB}$ is one as well; its four components enter the body's segment of $q_0$ in the order $(w,x,y,z)$, and the position-derivative map of Section 3.1 of the multibody chapter uses it as stored. The position of the body origin in I is the track-frame origin plus the lateral and vertical offsets expressed in $T(s_B)$:

$$
\mathbf p_{IoBo\_I}=\mathbf C(s_B)+R_{IT}(s_B)\,\boldsymbol\rho_B,
\qquad
\boldsymbol\rho_B=\begin{bmatrix}0\\ y_B\\ z_B\end{bmatrix}.
$$

The offset vector has no longitudinal component, so the body origin lies in the cross-section of its own station: $\mathbf p_{IoBo\_I}-\mathbf C(s_B)$ is orthogonal to $\mathbf C'(s_B)$, $s_B$ is a stationary point of the projection objective of Section 3.6 of the line-geometry chapter, and when the second-order condition of that section holds it is the projected station of the body origin. Section 6.5 uses this fact again.

On a straight, level line without superelevation (whose heading is identically zero by Section 2.1 of the conventions and whose centerline starts at the origin of the inertial frame by Section 3.3 of the line-geometry chapter), $R_{IT}=\mathbf 1$ and $\mathbf C(s)=(s-s_{\min},0,0)^{\mathsf T}$ with $s_{\min}$ the start station of the line, and the formulas reduce to $R_{IB}=R_{TB}$ and $\mathbf p_{IoBo\_I}=(s_B-s_{\min},y_B,z_B)^{\mathsf T}$. On a general line the frames $T(s_{B_1})$ and $T(s_{B_2})$ of two free bodies differ, so even with $R_{TB_1}=R_{TB_2}$ the two attitudes in I differ, and the relative attitude $R_{B_1B_2}=R_{TB_1}^{\mathsf T}R_{IT}(s_{B_1})^{\mathsf T}R_{IT}(s_{B_2})R_{TB_2}$ is set by the line between the two stations: at $t_0$ the vehicle bends along the line.

## 4. Forming the velocities

### 4.1 Angular velocity

The angular velocity of a free body is first composed in the body frame and then taken to I:

$$
\boldsymbol\omega_{IB\_B}=\boldsymbol\omega_{\mathrm{expl},B}+\mathbf c_B\,\Omega_0,
\qquad
\Omega_0=\frac{V_0}{r_{\mathrm{eff}}},
\qquad
\boldsymbol\omega_{IB\_I}=R_{IB}\,\boldsymbol\omega_{IB\_B}.
$$

$\Omega_0$ is the common spin quantity and is defined only for a record with common spin generation; under an explicit record there is no $r_{\mathrm{eff}}$, every body's $\mathbf c_B$ is zero and the angular velocity is $\boldsymbol\omega_{\mathrm{expl},B}$ itself. $\mathbf c_B$ is dimensionless: under common spin generation, a body spinning about one axis of its body frame usually takes the unit vector along that axis times the sign fixed by the positive sense of spin, the magnitude of the generated spin component being supplied uniformly by $\Omega_0$; a body that does not take part in the generation takes $\mathbf c_B=\mathbf 0$ and its angular velocity is given entirely by $\boldsymbol\omega_{\mathrm{expl},B}$. Which bodies take part is stated body by body in the record, not decided by the kind of carrier. The three components of $\boldsymbol\omega_{IB\_I}$ enter the first three positions of the body's segment of $v_0$, the angular-velocity part of a free body's generalized velocity in Section 2.1 of the multibody chapter.

### 4.2 Origin velocity

The inertial velocity of the body origin is first written in $T(s_B)$, with the longitudinal component taken from the vehicle's $V_0$ and the lateral and vertical components from the body's own values, and then taken to I:

$$
\mathbf v_{IBo\_T}=\begin{bmatrix}V_0\\ v_{y,B}\\ v_{z,B}\end{bmatrix},
\qquad
\mathbf v_{IBo\_I}=R_{IT}(s_B)\,\mathbf v_{IBo\_T}.
$$

$V_0$ is not copied body by body: it is one quantity of the vehicle, from which the longitudinal component of every free body is generated. $\mathbf v_{IBo\_I}$ enters the last three positions of the body's segment of $v_0$.

### 4.3 A change of basis only, no transport term

The two quantities of Sections 4.1 and 4.2 are both inertial velocities, the angular velocity relative to I and the origin velocity relative to I; the record merely writes them in a basis a person can check, the body frame or the local track frame. Taking them to I is therefore a pure change of basis, and no transport term appears. For contrast: had the record given velocities relative to a track frame moving with the body along the line, the inertial velocity would additionally contain $\dot s\big(\mathbf C'(s)+\boldsymbol\omega_{IT}\times R_{IT}\boldsymbol\rho_B\big)$ and the angular velocity $\dot s\,\boldsymbol\omega_{IT}$; none of these terms occurs in the assembly of this chapter, because the record's numbers are inertial already. One consequence: for a startup on a curve, a yaw rate of the car body that follows the curve must be written into $\boldsymbol\omega_{\mathrm{expl},B}$; the assembly does not add it on the record's behalf.

### 4.4 Start speed and station rate

$V_0$ is the local longitudinal component of an inertial velocity and is in general not equal to the station rate $\dot s_B$. Let the body origin move while staying in the cross-section of its projected station, $\mathbf p=\mathbf C(s)+R_{IT}(s)\boldsymbol\rho$ with the longitudinal component of $\boldsymbol\rho$ identically zero. Differentiating in time and using $\dfrac{dR_{IT}}{ds}=\operatorname{skew}(\boldsymbol\omega_{IT})R_{IT}$ of Section 3 of the conventions,

$$
\dot{\mathbf p}=\big(\mathbf C'(s)+\boldsymbol\omega_{IT}\times R_{IT}\boldsymbol\rho\big)\dot s+R_{IT}\dot{\boldsymbol\rho}.
$$

Taking the component along $\mathbf x_T=\mathbf C'/\lVert\mathbf C'\rVert$, to which $\dot{\boldsymbol\rho}$ contributes nothing since it has no longitudinal component, gives

$$
V_0=\Big(\lVert\mathbf C'(s_B)\rVert+\big(\boldsymbol\omega_{IT\_T}\times\boldsymbol\rho_B\big)_x\Big)\dot s_B,
$$

where $\boldsymbol\omega_{IT\_T}=R_{IT}^{\mathsf T}\boldsymbol\omega_{IT}$ and the subscript $x$ takes the longitudinal component in $T$. $\lVert\mathbf C'\rVert$ equals one only where the grade is zero (Section 2.1 of the conventions); the second term may be nonzero where the line has a rotation rate and the body origin is off the centerline. On a level curve without superelevation and with curvature $\kappa$, for example, $\boldsymbol\omega_{IT\_T}=(0,0,\kappa)^{\mathsf T}$ and hence $V_0=(1-\kappa y_B)\dot s_B$: for one and the same $V_0$, a body on the inside of the curve has the larger station rate. $\dot s_B$ is not a state and does not occur in the assembly; the contact force plan re-derives it from the current state by the same relation at every evaluation, see Section 4 of the contact-plan chapter.

### 4.5 Common spin is an optional generation rule, not a rolling constraint

Common spin generation supplies a nominal spin component satisfying $\Omega_0\,r_{\mathrm{eff}}=V_0$: when $\mathbf c_B$ is a unit vector it is the part of the body's axial angular rate contributed by the generation rule, and the total axial rate also contains the projection of $\boldsymbol\omega_{\mathrm{expl},B}$ onto that axis. This is one algebraic expansion: it constitutes no kinematic constraint and does not guarantee that the complete startup state rolls without sliding at the contact, since $r_{\mathrm{eff}}$ is not the rolling radius at the contact; the equations of motion (Section 5 of the multibody chapter) contain no rolling constraint, spin and translation are independent state from $t_0$ on, and the full dynamics couples them afterwards. $r_{\mathrm{eff}}$ therefore takes part in the generation of velocities only; it is neither the nominal rolling radius $r_0$ of the contact chain nor the local rolling radius $r$ at the contact (Section 2.7 of the conventions), and it enters no contact geometry. An explicit record does not use this rule; if its author wants some rolling relation to hold at the instant of startup, that is arithmetic among the record's explicit values, which the assembly neither requires nor checks.

## 5. Joint coordinates and force-element internal forces

### 5.1 Revolute joints: one angle, two sources of the rate

The generalized position of revolute joint $j$ is the angle $\theta_j$ about the joint axis and its generalized velocity is the rate $\dot\theta_j$ (Section 2.1 of the multibody chapter). The angle is taken from the record directly; the rate is generated by one of two rules:

$$
\dot\theta_j=\dot\theta_{\mathrm{expl},j}
\qquad\text{or}\qquad
\dot\theta_j=\lambda_j\,\Omega_0 .
$$

The two rules give the same physical quantity, the relative angular rate of the child frame with respect to the parent frame about the joint axis; only its source differs. The multiplier form serves rates that are by definition proportional to $\Omega_0$: for example, a part hung on a spinning body through a revolute joint about the spin axis, which is to have no spin in I, must have a relative rate that cancels the body's complete axial rate; when that axial rate comes from the generated component alone and the generation coefficient $\mathbf c_B$ is the unit vector along that axis, this is $-\Omega_0$ times the sign of the axis (for a coefficient of other length it is $-\mathbf c_B\cdot\mathbf e\,\Omega_0$, with $\mathbf e$ the unit axis vector), and writing it as a multiplier avoids a second copy of $V_0/r_{\mathrm{eff}}$ in the record. An explicit record has the first form only; under common spin generation both forms may occur, decided joint by joint by the record and not by the kind of carrier.

### 5.2 Ball-RPY joints and weld joints

The three generalized positions of a Ball-RPY joint are the record's three angles $(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})$, composed as $R_{FM}=R_z(\mathrm{yaw})R_y(\mathrm{pitch})R_x(\mathrm{roll})$ by Section 4.3 of the conventions; its three generalized velocities are the record's $\boldsymbol\omega_{FM\_F}$, the physical angular velocity of the child frame relative to the parent frame, expressed in the parent frame. It is not the time derivative of the three angles: the two are related by the configuration-dependent map of Section 3.2 of the multibody chapter, which is singular at $\cos(\mathrm{pitch})=0$; the assembly does not evaluate that map and writes both groups exactly as the record states them. A weld joint has no generalized coordinates, the pose of its child frame relative to its parent frame is a constant of the model, and the record has no entry for it.

### 5.3 Series internal forces and $z_0$

Each component of the $z$ block belongs to one series element and its initial value is the internal force given by the record:

$$
z_0=\begin{bmatrix}F_1\\ \vdots\\ F_{n_z}\end{bmatrix},
$$

where $F_i$ acts along the element's axis on its reference end with the sign of Section 1.1 of the series chapter. It has to be given as an initial value because, as Section 2.1 of the series chapter shows, this force cannot be determined algebraically from $(q,v)$. $F_i=0$ means the element starts relaxed; $F_i\neq0$ is an internal force at the instant of startup but not a static preload: a series element has no static stiffness, and if its ends are subsequently at relative rest the force decays to zero with the relaxation time $c/k$ (Section 2.3 of the series chapter).

### 5.4 Composition of the initial value

Collecting Sections 3 to 5,

$$
q_0=\Big[\ \{\mathsf q_{IB};\ \mathbf p_{IoBo\_I}\}_{B}\ ;\ \{\theta_j\}_{j}\ ;\ \{(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})\}\ \Big],
\qquad
v_0=\Big[\ \{\boldsymbol\omega_{IB\_I};\ \mathbf v_{IBo\_I}\}_{B}\ ;\ \{\dot\theta_j\}_{j}\ ;\ \{\boldsymbol\omega_{FM\_F}\}\ \Big],
$$

where the braces denote filling each segment by name; the position of a segment within the vector belongs to the model and is not prescribed by this chapter. $y_0=[q_0;v_0;z_0]$ is written together with the time $t_0=0$ into a fresh runtime context and becomes the initial condition $y(t_0)=y_0$ of the initial-value problem of Section 1.1 of the time-integration chapter.

## 6. Accompanying non-state quantities

### 6.1 Nominal forces

The nominal force $\mathbf f_0$ of each three-axis translational element is written into the parameters of the runtime context, not into the state: it is read at every right-hand-side evaluation, never integrated, and the integrator's trial context copies the same set of values. Its mathematical role is given in Section 2.3 of the translational chapter: it is the value of the elastic part at coincident origins and the only term of the law that can move the force-free configuration away from coincident origins. The $\mathbf f_0$ given by the record therefore changes the force-displacement relation of each translational element without changing $y_0$.

### 6.2 Rail-profile vertical datum

The rail-profile vertical datum offset is a signed length along $+z_T$ that becomes, as stated, the rail-side vertical datum $z_r$ of the wheel-rail pose constants of both sides, that is, the fixed part of the actual rail-profile datum $z_{rd}=z_r+z_\epsilon$ of Section 3.5 of [Wheel-rail pose reduction and irregularity inputs](../wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.en.md); it is likewise the fixed part of the vertical coordinate, in the track frame of the effective station, of the rail-profile origin $\mathbf o_c$ placed at that station in Section 3.8 of that chapter. Since $+z_T$ points down, a positive value lowers the rail profile and a negative one raises it. It is a constant of the contact geometry for the whole run: changing it changes the relative wheel-rail pose at $t_0$ and at every later instant, and with it the meaning of the startup state the record describes.

### 6.3 The common gravity premise

The gravity of the multibody model is the constant vector $\mathbf g_I=g\,\mathbf e_z^I$ with $g>0$, its direction fixed by the track inertial frame (Section 2.1 of the conventions, Section 2.4 of the multibody chapter). The series internal forces, nominal forces and rail-profile vertical datum offset of a startup record are one set of quantities resolved under one value of $g$, and this chapter assumes that the $g$ of the model used in the assembly is the same number as the $g$ under which the record was resolved: this is the premise under which the record is meaningful, not part of its content. $g$ is a model constant and does not belong to $y_0$.

### 6.4 Running direction

There is currently one definition of the running direction: the vehicle starts toward increasing station. $V_0$ is a magnitude and the direction is carried by this definition, so the longitudinal component of $\mathbf v_{IBo\_T}$ is positive for every free body, and the $\dot s_B$ of Section 4.4 is positive as well whenever its denominator is positive.

### 6.5 Projection seed

A wheel-rail contact force plan assembled together with the startup state records one initial projection station per carrier, given by the $s_B$ of its station reference body, that is, by the same three-term sum of Section 3.1. It is the initial value of the seed of the local branch projection of Section 3 of the contact-plan chapter and does not belong to $[q;v;z]$; by Section 3.3, $s_B$ is a stationary point of the projection objective of that body's origin at $t_0$ and is the projection station when the second-order condition cited there holds, in which case the seed and the projection agree at the instant of startup.

## 7. Mathematical properties and conditions of applicability

- **Kinematic consistency.** $\mathsf q_{IB}$ is the product of two unit quaternions and hence a unit quaternion, and $R_{IB}$ is a proper rotation; the overall sign of the quaternion carries no physics, $\pm\mathsf q_{IB}$ giving the same $R_{IB}$. Each free-body segment of $v_0$ is an angular velocity and an origin velocity expressed in I, the same definition as the free-body generalized velocity of Section 2.1 of the multibody chapter, so $\dot q_0=N(q_0)v_0$ is defined directly and no projection or normalization is needed at startup.
- **Fitted to the line at $t_0$ only.** Positions and attitudes are built on the complete track frame at each body's own station, whereas the velocities are the inertial values given by the record, and no term ties them to the curvature or rotation rate of the line. After $t_0$ the bodies obey only the equations of motion and the loads written by the force elements and the contact force plan; the line no longer appears as a constraint.
- **Start speed and station rate.** $V_0$ is the same for all free bodies, whereas by the relation of Section 4.4 the $\dot s_B$ of the bodies on a curve or grade in general differ from one another and from $V_0$; $\dot s_B$ is re-derived by the contact force plan at every evaluation.
- **No equilibrium and no preload solve.** The loads at $t_0$ are whatever the constitutive laws give at $(q_0,v_0,z_0)$ with the nominal forces and $z_r$; $\dot v(t_0)=\mathbf 0$ is neither imposed nor checked, and a record inconsistent with equilibrium starts the motion with a transient. A nonzero series internal force decays with its element's relaxation time unless relative motion sustains it.
- **One record under different placements is not one initial-value problem.** The record states the pose of each body in a local track frame. On a straight, level line without superelevation every $s_{\mathrm{ref}}$ gives the same relative poses between bodies; on a general line the relative poses between bodies may change with the placement, the deformations the force elements see at $t_0$ change with them, and $z_0$, the nominal forces and $z_r$ are not adjusted to the placement.
- **Domain.** $V_0>0$; with common spin generation $r_{\mathrm{eff}}>0$, otherwise $\mathbf c_B=\mathbf 0$ and revolute joints have explicit rates only; $\lVert\mathsf q_{TB}\rVert=1$; every $s_B$ finite, possibly on the tangent continuation outside the definition interval; the superelevation domain $\lvert u\rvert<b$ is guaranteed by the line itself (Section 2.2 of the conventions). The three angles of a Ball-RPY joint are themselves unrestricted, but $\cos(\mathrm{pitch})=0$ makes $N(q)$ singular and the integration cannot start there (Section 3.2 of the multibody chapter).
- **Quantities outside $y_0$.** The time $t_0$, the projection seeds and the nominal forces live in the context and not in the continuous state, consistent with Section 5.1 of the conventions; $\Omega_0$, $r_{\mathrm{eff}}$ and $\dot s_B$ are generated or derived quantities that are not retained once the assembly is complete.

## 8. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| Physical quantities of the startup record: free bodies, revolute joints, Ball-RPY joints, series internal forces, nominal forces, common spin generation, vehicle-level quantities | `ResolvedStartupState`, `FreeBodyStartupState`, `RevoluteJointStartupState`, `BallRpyJointStartupState`, `SeriesSpringViscousDamperForceState`, `TranslationalSpringDamperNominalForce`, `CommonWheelSpinGeneration`, in [`resolved_startup_state.h`](../../../libs/configuration/include/orvd/configuration/resolved_startup_state.h) |
| Definition of the running direction | `StartupRunningDirection`, in [`resolved_startup_state.h`](../../../libs/configuration/include/orvd/configuration/resolved_startup_state.h) |
| Mechanical station offset $\Delta s_{\mathrm{mech},B}$ | `VehicleFreeBodyStationOffsetDefinition`, in [`vehicle_definition.h`](../../../libs/configuration/include/orvd/configuration/vehicle_definition.h) |
| Station sum, formation of $\mathsf q_{IB}$ and $\mathbf p_{IoBo\_I}$, of $\boldsymbol\omega_{IB\_I}$ and $\mathbf v_{IBo\_I}$, of the joint coordinates and of $z_0$ | `AssembleResolvedInitialContext`, in [`assemble_resolved_initial_context.cc`](../../../libs/configuration/src/assemble_resolved_initial_context.cc) |
| $R_{IT}(s)$ and $\mathbf C(s)$ at station $s$ | `TrackGeometry::EvaluateTrackFrame`, `TrackFramePose`, in [`track_geometry.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_geometry.h) and [`track_frame_pose.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_frame_pose.h) |
| Component order within a free-body segment | `MultibodyModel::GetFreeBodyPositionRange`, `MultibodyModel::GetFreeBodyVelocityRange`, in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Component of the $z$ block belonging to each series element | `SystemInstance::series_spring_damper_force_state_range`, in [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| Writing $y_0$ and $t_0$ into the runtime context | `SystemInstance::SetTimeAndContinuousState`, in [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| Nominal force $\mathbf f_0$ as a context parameter | `SystemInstance::SetNominalForce`, in [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| Rail-profile vertical datum $z_r$ entering the pose constants | `MakePoseConstants`, in [`wheel_rail_contact_personality_assembly.cc`](../../../libs/configuration/src/wheel_rail_contact_personality_assembly.cc); `WheelRailPoseConstants`, in [`wheel_rail_pose.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_pose.h) |
| Gravity direction and the model's gravity constant | `GravitationalAccelerationInInertial`, in [`track_inertial_frame.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_inertial_frame.h); `MultibodyModel::gravity_vector`, in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Initial projection station of a carrier taken as $s_B$ | `MakeCarrier`, in [`assembled_vehicle_contact_scenario.cc`](../../../libs/configuration/src/assembled_vehicle_contact_scenario.cc); `WheelRailContactCarrierDefinition::initial_projection_station_meters`, in [`wheel_rail_contact_force_plan.h`](../../../libs/forces/include/orvd/forces/wheel_rail_contact_force_plan.h) |
