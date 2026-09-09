[中文](CONTACT_FORCE_PLAN_KINEMATICS.md)

# Contact force plan kinematics

This chapter closes the wheel-rail contact chain and sits between the components and the vehicle's right-hand side. The published chapters on line geometry, irregularity spectra, wheel-rail contact and force elements are the components; [Multibody equations of motion](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.en.md) combines the body wrenches those components produce with the multibody layer's own gravity and joint damping into the complete right-hand side $[N(q)v;\dot v;\dot z]$, [Startup state assembly](../vehicle_dynamics/STARTUP_STATE_ASSEMBLY.en.md) constructs the initial value $y_0$, this chapter explains how the multibody state becomes the contact input of every wheel-rail interface and how the contact result becomes one equivalent wrench acting on the wheel body, and Section 2.3 of [Time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md) explains how the solver differences the complete right-hand side. No contact mechanics is written here: pose reduction, contact geometry, normal force, creepages and tangential force belong to [Wheel-rail pose reduction and irregularity inputs](WHEEL_RAIL_POSE_REDUCTION.en.md), [Single-wheel contact-model assembly and paired wrench](CONTACT_MODEL_ASSEMBLY_AND_WRENCH.en.md) and the chapters upstream of them; this chapter only forms their inputs and consumes their outputs. The projection of the carrier station, the pose and rates in the track frame, the spin-free geometric attitude, the rigid motion of the profile, the sampling of the irregularity, the placement of the rail profile and the combination of the per-patch wrenches are all defined here. The implementation is `WheelRailContactForcePlan` in [`wheel_rail_contact_force_plan.h`](../../../libs/forces/include/orvd/forces/wheel_rail_contact_force_plan.h) and [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc).

## 1. Scope and notation

### 1.1 The object

The object of this chapter is the contact force plan: a force source bound to a finalized multibody model and immutable after construction. At every right-hand-side evaluation it turns the multibody state $(q,v)$ into the contact input of each wheel-rail interface and writes the contact result as one equivalent wrench acting on that interface's wheel body. It contains no contact mechanics of its own and holds no state that evolves in time; what it holds is the line geometry, an optional irregularity field, the contact model and fixed geometry of each side, and the definitions of the carriers and interfaces.

Carriers come in two kinds. A rigid wheelset: the wheel body is the carrier, and the spin is contained in that body's attitude and angular velocity. An axle body with independently rotating wheels: the carrier is the non-spinning axle body, and each wheel is a separate rigid body attached to it by one revolute joint; the wheel's origin velocity and absolute angular velocity are taken from the wheel body itself, while the non-spinning profile attitude and the station history are still taken from the carrier. Both kinds follow one mathematical path and part only at the interface kinematics of Section 5.

The chapter answers four questions: how the station of a carrier on the line is determined and continued in time; how the carrier's pose and velocity are taken into the track frame and freed of spin; how the profile rigid motion, the irregularity input and the rail-profile placement of each interface are formed; and how the wrenches of the contact patches combine into one body wrench. Sections 2 to 7 answer them in order, Section 8 gives the order of computation and Section 9 lists the conditions of applicability.

### 1.2 Notation

| Symbol | Meaning | Source |
|---|---|---|
| $I$, $T$, $T(s)$ | Track inertial frame; track frame at the carrier station $s_c$; track frame at an arbitrary station $s$ | Conventions §2.1–2.2 |
| $B$, $W$ | Body frame of the carrier; non-spinning wheel-profile frame | Conventions §2.3; §2.1 here |
| $R_{BW}$ | Fixed attitude of the wheel-profile axes in the body frame; converts $W$-frame components to $B$-frame components | New here |
| $\mathbf p_{Bo}$, $R_{IB}$, $\mathbf v_{IBo\_I}$, $\boldsymbol\omega_{IB\_I}$ | Position of the carrier's body origin in $I$, carrier attitude, body-origin velocity and angular velocity | Conventions §4.1 |
| $s_c$, $s_e$ | Shared station of the carrier; effective profile station of one side | Pose-reduction chapter §2 |
| $\mathbf C(s)$, $\mathbf C'(s)$, $R_{IT}(s)$, $\boldsymbol\omega_{IT}(s)$ | Centerline position, its station derivative, track-frame attitude, track-frame rotation rate per metre, the last two expressed in $I$ | Conventions §2.1, §3 |
| $\mathbf c'$, $\boldsymbol\omega_T$ | $\mathbf C'(s_c)$ and $\boldsymbol\omega_{IT}(s_c)$ expressed in $T$ | New here |
| $\mathbf r_B$, $\mathbf v_B$, $\boldsymbol\omega_B$ | Position, inertial velocity and inertial angular velocity of the carrier's body origin, all expressed in $T$ | New here |
| $y_B$, $z_B$ | Lateral and vertical components of $\mathbf r_B$, that is, $y_w$ and $z_w$ of the pose-reduction chapter | Pose-reduction chapter §2 |
| $\phi_w$, $\psi_w$, $\chi$ | X-Z-Y roll, yaw and pitch of $R_{TW}$ | Pose-reduction chapter §2, §3.10 |
| $\bar R_{TW}$ | Spin-free geometric attitude $R_x(\phi_w)R_z(\psi_w)$ | New here |
| $\mathbf a$ | Axle direction $\bar R_{TW}\mathbf e_2$, expressed in $T$ | Pose-reduction chapter §3.3 |
| $\dot s_c$, $\dot\ell$ | Signed station rate and path rate of the carrier | Conventions §3 |
| $\boldsymbol\omega_{TB\_T}$ | Angular velocity of the carrier relative to the track frame, expressed in $T$ | Conventions §4.1 |
| $\dot\chi$ | X-Z-Y pitch rate of the carrier | New here |
| $\kappa$ | Planar curvature at station $s_c$ | Conventions §2.2 |
| $\mathbf o_{\mathrm{wheel}}$, $\mathbf v_{\mathrm{wheel}}$, $\boldsymbol\omega_{\mathrm{wheel}}$ | Position, inertial velocity and inertial angular velocity of the wheel-body origin in $T$ | Pose-reduction chapter §3.8 (position) |
| $\dot q_j$, $\mathbf u$ | Generalized velocity of the independent-wheel revolute joint and its positive unit axis | New here |
| $\Omega$ | Wheel pitch-rate scalar handed to the contact model | Creepage chapter §1 |
| $\sigma$ | Signed wheel-profile lateral datum along the axle | Pose-reduction chapter §2 |
| $\mathbf o_W$, $\mathbf v_o$ | Position and velocity of the wheel-profile datum | Single-wheel chapter §3.1 |
| $y_\epsilon$, $z_\epsilon$, $y_\epsilon'$, $z_\epsilon'$, $\dot y_\epsilon$, $\dot z_\epsilon$ | Irregularity displacements, spatial slopes and time rates | Pose-reduction chapter §3.9 |
| $I_T$ | Defined interval of the line | Pose-reduction chapter §3.9 |
| $\psi_e$ | Yaw corrected by the alignment slope at $s_c$ | Pose-reduction chapter §3.9 |
| $y_r$, $z_r$, $\phi_r$ | Lateral and vertical rail-profile datums and signed rail cant | Pose-reduction chapter §2 |
| $\mathbf o_c$, $R_{T\mathrm{rail}}$ | Placement origin and placement attitude of the rail profile | Pose-reduction chapter §3.8 |
| $\widehat\psi_\epsilon$, $\widehat\theta_\epsilon$ | Alignment and vertical spatial-slope angles at the effective station | Pose-reduction chapter §3.8 |
| $\varphi$, $\beta$, $d_y$, $d_z^{\uparrow}$ | The four pose scalars | Conventions §2.6 |
| $\mathbf x_{P,k}$, $\mathbf f_{T,k}$, $\mathbf m_{P,k}$ | Wheel-side application point of patch $k$, and the rail-on-wheel force and moment, expressed in $T$ | Single-wheel chapter §3.2, §4 |
| $\mathcal W_Q^E=(\boldsymbol\tau_Q^E,\mathbf f^E)$ | Wrench reduced about $Q$ and expressed in $E$ | Conventions §5.2 |

### 1.3 Relation to the notation of the other chapters

Frames, signs and rotation notation follow [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md). $T$ is the track frame at $s_c$, as in the pose-reduction chapter; the two interfaces of one carrier share this one $T$. The subscript $w$ of $\phi_w$ and $\psi_w$ is inherited from the pose-reduction chapter, where it denotes the carrier of the non-spinning wheel profile, that is, the carrier of this chapter; quantities of the wheel body always carry the subscript $\mathrm{wheel}$, following $\mathbf o_{\mathrm{wheel}}$ of Section 3.8 of that chapter. The pitch angle is written $\chi$; it is the third X-Z-Y angle, written generically as $\theta$ in Section 3.10 of the pose-reduction chapter, renamed so that it cannot be confused with the vertical-irregularity angle $\theta_\epsilon$. $R_{TW}$ and $\bar R_{TW}$ are deliberately distinguished: the former is computed directly from the multibody state and contains the spin for a rigid wheelset; the latter keeps only roll and yaw and is the spin-free $R_{TW}$ of Section 3.2 of the single-wheel chapter. The source calls the non-spinning wheel-profile frame the profile frame and labels it with the letter P; following Section 2.3 of the conventions this chapter reserves $P$ for the wheel-side application point of the contact force and writes the profile frame as $W$. $\Omega$ is the wheel rotation rate of the creepage chapter, negative in forward rolling; $\dot s_c$ is the $\dot s$ of the pose-reduction chapter. The bold $\mathbf v_B$ and $\mathbf v_{\mathrm{wheel}}$ are velocity vectors, unrelated to the scalar separation $v_c$ of Section 3.6 of the pose-reduction chapter. The $\boldsymbol\omega_T$ of this chapter is the track frame's rotation rate per metre expressed in $T$, in radians per metre; the creepage chapter uses the same letter for the relative angular velocity at the contact, and the two never appear in one chapter. $\dot q_j$ is the generalized velocity of one revolute joint and belongs to the $v$ block of the continuous state of Section 5.1 of the conventions. Section 2.1 of [Multibody equations of motion](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.en.md) writes the axis of a revolute joint as $\mathbf a$; this chapter keeps $\mathbf a$ for the axle direction of Section 3.3 of the pose-reduction chapter and writes the positive axis of the revolute joint as $\mathbf u$.

## 2. Carriers and wheels

### 2.1 The carrier

A carrier is a station-reference rigid body. A carrier definition consists of three things: a rigid body, whose body frame is $B$; an initial projection station; and a constant rotation $R_{BW}$. $R_{BW}$ is the fixed attitude of the axes of the non-spinning wheel-profile frame $W$ in the body frame $B$; by the convention of Section 4.1 of the conventions it converts $W$-frame components to $B$-frame components. The axes of $W$ follow Section 2.3 of the conventions, $y$ along the axle and $z$ radially downward. It is a constant orthogonal matrix that only realigns coordinate axes; when the body frame already follows the axis convention of $W$ it is the identity. It does not contain, and cannot contain, the spin phase of the wheel: the spin is carried in time by the multibody state, a constant matrix cannot remove it, and the removal of spin happens in Section 4.2.

The attitude of the wheel-profile frame in the track frame is composed from the carrier attitude and the constant rotation:

$$
R_{TW}=R_{TB}R_{BW},\qquad R_{TB}=R_{IT}^{\mathsf T}R_{IB}.
$$

For a rigid wheelset $R_{TB}$ turns with the wheel, so $R_{TW}$ contains the spin; for an axle body $R_{TB}$ is the attitude of a non-spinning body and $R_{TW}$ contains only its roll, yaw and small pitch.

### 2.2 The interface

An interface consists of four items: a carrier, a wheel body, one of the two sides, and an optional revolute joint. The interface of a rigid wheelset carries no revolute joint and its wheel body is the carrier body; an interface with an independently rotating wheel must name the revolute joint that attaches the wheel to the carrier, and that joint owns exactly one generalized position and one generalized velocity, see Section 2.1 of [Multibody equations of motion](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.en.md). A carrier may carry one interface on each of its two sides, and the two sides share all the carrier quantities of Sections 3 and 4.

Each side has one contact model and one set of fixed geometry: the rail-profile datums $y_r$ and $z_r$, the signed rail cant $\phi_r$, the wheel-profile lateral datum $\sigma$, the nominal rolling radius $r_0$ and the pitch-correction choice of Section 3.4 of the pose-reduction chapter; the longitudinal-origin convention of the rail profile is shared by both sides. They are fixed once when the plan is constructed and none of them is state. Every interface writes exactly one body wrench per evaluation, so the number of body wrenches the plan writes equals the number of interfaces.

## 3. Projection of the carrier station

### 3.1 The projection problem

Every evaluation first projects the body origin of each carrier onto the line. The spatial point is the body-origin position $\mathbf p_{Bo}$ obtained from the multibody layer's world-frame query; this chapter identifies the multibody layer's world frame with the track inertial frame $I$. The projection is the local-branch problem of Section 3.6 of [Line geometry and track frames](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.en.md): with the objective

$$
f(s)=\tfrac12\left\lVert\mathbf p_{Bo}-\mathbf C(s)\right\rVert^2
$$

it starts from a seed that identifies the current branch, makes at most two Newton corrections as in Section 4.4 of that chapter, and accepts a station that satisfies the scaled stationary-point condition with $f''(s)>0$. The resulting station is written $s_c$ and called the shared station of the carrier: both interfaces of the carrier take it as the station of $T$. What is projected is the body origin, not the wheel-profile datum of either side; the effective station $s_e$ of each side is derived from $s_c$ in Section 6.1.

The stationary-point condition $f'(s_c)=-(\mathbf p_{Bo}-\mathbf C(s_c))\cdot\mathbf C'(s_c)=0$ together with $\mathbf C'=\sqrt{1+g^2}\,\mathbf x_0$ of Section 2.2 of the conventions ($g$ is the upward grade there; the superelevation roll is about $\mathbf x_0$, so the $x$ axis $\mathbf x_T$ of $T$ is $\mathbf x_0$) shows that, at an exact projection, the longitudinal coordinate of the body origin in $T$ is zero, $\mathbf e_1^{\mathsf T}\mathbf r_B=0$. The station rate of Section 4.3 uses this.

### 3.2 The projection seed and its advance

The seed is taken from the accepted-step history and is not part of the continuous state. The initial seed of each carrier is the initial projection station of its definition; the system's runtime context holds one seed per carrier. A right-hand-side evaluation only reads the seed array: it projects from the seed, obtains $s_c$ and passes it downstream, but does not write $s_c$ back into the seed. After every accepted internal step has installed its endpoint state, a projection-only evaluation reprojects all carriers starting from the current seeds and replaces the seeds as a whole by the resulting stations; every trial context copies this accepted set of seeds. When Section 5.1 of the conventions excludes numerical projection seeds from $[q;v;z]$, it means exactly these quantities.

The mathematical consequence of this arrangement is that within one internal step the seeds are fixed, so the right-hand side is a function of $(t,y)$ independent of the order of the trials within the step; across steps the seeds move forward with the accepted endpoints, which continues the local branch while the line-geometry layer itself keeps no time history.

## 4. Carrier attitude and rates

### 4.1 Into the track frame

At $s_c$ the pose of the track frame and its station derivative are evaluated at once: the centerline position $\mathbf C(s_c)$, the attitude $R_{IT}$, the centerline derivative $\mathbf C'(s_c)$ and the rotation rate per metre $\boldsymbol\omega_{IT}(s_c)$, see Sections 3.4 and 4.3 of the line-geometry chapter. Below, $R_{IT}$ without an argument always means $R_{IT}(s_c)$. The carrier's position, velocity and angular velocity are taken into $T$:

$$
\mathbf r_B=R_{IT}^{\mathsf T}\left(\mathbf p_{Bo}-\mathbf C(s_c)\right),\qquad
\mathbf v_B=R_{IT}^{\mathsf T}\mathbf v_{IBo\_I},\qquad
\boldsymbol\omega_B=R_{IT}^{\mathsf T}\boldsymbol\omega_{IB\_I}.
$$

$\mathbf v_{IBo\_I}$ and $\boldsymbol\omega_{IB\_I}$ form the spatial velocity of the body frame relative to the world frame as the multibody layer returns it, with the translational part taken at the body origin, see Section 4.2 of [Multibody equations of motion](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.en.md). The three formulas only change basis: $\mathbf v_B$ and $\boldsymbol\omega_B$ remain an inertial velocity and an inertial angular velocity, not velocities relative to the track frame, and no transport term for the track frame's own motion is subtracted. This agrees with the stationary-rail assumption of Section 2 of the single-wheel chapter: the rail is fixed in $I$, the relative velocity at the contact is the inertial velocity of the wheel material, and what is fed to the contact model must therefore be an inertial velocity. The line quantities are taken into $T$ in the same way:

$$
\mathbf c'=R_{IT}^{\mathsf T}\mathbf C'(s_c),\qquad
\boldsymbol\omega_T=R_{IT}^{\mathsf T}\boldsymbol\omega_{IT}(s_c).
$$

Because $\mathbf C'$ lies along the $x$ axis of $T$, in exact arithmetic $\mathbf c'=(\sqrt{1+g^2},0,0)^{\mathsf T}$; the implementation forms it by the rotation above and does not substitute this closed form.

### 4.2 The spin-free geometric attitude

The X-Z-Y resolution of Section 4.4 of the conventions is applied to $R_{TW}=R_{IT}^{\mathsf T}R_{IB}R_{BW}$, giving

$$
R_{TW}=R_x(\phi_w)\,R_z(\psi_w)\,R_y(\chi),\qquad \psi_w\in\left[-\tfrac{\pi}{2},\tfrac{\pi}{2}\right],
$$

with the explicit expressions of the three angles in Section 3.10 of the pose-reduction chapter. Roll and yaw are kept, the pitch $\chi$ is discarded, and the geometric attitude is rebuilt from the first two angles:

$$
\bar R_{TW}=R_x(\phi_w)\,R_z(\psi_w),\qquad R_{TW}=\bar R_{TW}\,R_y(\chi).
$$

It is this step, and not the constant rotation $R_{BW}$, that removes the spin. For a rigid wheelset $R_{BW}$ is constant while the wheel turns, so the whole spin of $R_{TW}$ lands in $\chi$ and only discarding $\chi$ removes it; for an axle body $\chi$ is the small pitch of a non-spinning body and is discarded as well. The profile attitude handed to contact geometry therefore contains no rotation about the axle for either kind of carrier, which the axisymmetry of the profile permits: turning an axisymmetric profile about its axle does not change its geometric placement, see Section 3.2 of the single-wheel chapter.

$\bar R_{TW}$ has a characterization that does not depend on the resolution steps. Since $R_y(\chi)\mathbf e_2=\mathbf e_2$, the axle direction

$$
\mathbf a=R_{TW}\mathbf e_2=\bar R_{TW}\mathbf e_2=
\begin{bmatrix}-\sin\psi_w\\ \cos\phi_w\cos\psi_w\\ \sin\phi_w\cos\psi_w\end{bmatrix}
$$

is independent of $\chi$ and is exactly the $\mathbf a$ of Section 3.3 of the pose-reduction chapter. For $\cos\psi_w>0$, $\bar R_{TW}$ is the unique rotation of the form $R_x(\cdot)R_z(\cdot)$, with yaw confined to the closed quarter turn, that takes $\mathbf e_2$ to $\mathbf a$: the spin-free geometric attitude is determined by the axle direction alone, whatever phase the body frame has turned to about the axle.

### 4.3 Station rate and path rate

The body-origin position decomposes as $\mathbf p_{Bo}=\mathbf C(s_c)+R_{IT}(s_c)\,\mathbf r_B$, and the projection keeps $\mathbf e_1^{\mathsf T}\mathbf r_B$ at zero. Differentiating in time and using $dR_{IT}/ds=\operatorname{skew}(\boldsymbol\omega_{IT})R_{IT}$ of Section 3 of the conventions,

$$
\mathbf v_{IBo\_I}=\dot s_c\left(\mathbf C'(s_c)+\boldsymbol\omega_{IT}(s_c)\times R_{IT}\mathbf r_B\right)+R_{IT}\dot{\mathbf r}_B.
$$

Taking this into $T$ and keeping the longitudinal component, with $\mathbf e_1^{\mathsf T}\dot{\mathbf r}_B=0$, gives the station rate

$$
\dot s_c=\frac{\mathbf e_1^{\mathsf T}\mathbf v_B}{\mathbf e_1^{\mathsf T}\left(\mathbf c'+\boldsymbol\omega_T\times\mathbf r_B\right)}.
$$

The numerator is the component of the body origin's inertial velocity along the longitudinal axis of the track frame; the first term of the denominator is $\sqrt{1+g^2}$, and the second is the transport term produced by the track frame's rotation per metre, which on a planar curve without grade equals $-\kappa\,(y_B\cos\phi-z_B\sin\phi)$, $\phi$ being the superelevation angle of Section 2.2 of the conventions, that is, minus the curvature times the horizontal lateral offset of the body origin from the centerline, so that with zero superelevation the denominator reduces to the familiar $1-\kappa y_B$. The denominator also has an exact expression: from $\mathbf C''=(\sqrt{1+g^2})'\,\mathbf x_T+\sqrt{1+g^2}\,\boldsymbol\omega_{IT}\times\mathbf x_T$ and $\mathbf e_1^{\mathsf T}\mathbf r_B=0$ one obtains, for the objective of Section 3.6 of the line-geometry chapter, $f''(s_c)=\sqrt{1+g^2}\;\mathbf e_1^{\mathsf T}\left(\mathbf c'+\boldsymbol\omega_T\times\mathbf r_B\right)$, so the station-rate denominator is $f''(s_c)/\lVert\mathbf C'(s_c)\rVert$ and, at an exact projection, has the sign of the regularity condition $f''>0$ of the projection. $\dot s_c$ is signed, and its sign is the direction of travel.

The path rate follows from the station rate as in Section 3 of the conventions:

$$
\dot\ell=\left\lVert\mathbf C'(s_c)\right\rVert\,\dot s_c=\sqrt{1+g^2}\,\dot s_c,
$$

also signed. It is the $\dot\ell$ of the reference speed of Section 3.1 of the creepage chapter; the slope angles of the pose-reduction chapter use only $\dot s_c$, and the two cannot be interchanged on a nonzero grade.

### 4.4 Relative angular velocity and pitch rate

The track frame $T$ moves with $s_c(t)$; its angular velocity in $I$ is $\boldsymbol\omega_{IT}\dot s_c$. The angular velocity of the carrier relative to the track frame, in $T$, is

$$
\boldsymbol\omega_{TB\_T}=\boldsymbol\omega_B-\boldsymbol\omega_T\,\dot s_c.
$$

Since $R_{BW}$ is constant, this is also the angular velocity of $W$ relative to $T$. It is fed, together with the resolved $(\phi_w,\psi_w)$, into the inverse X-Z-Y rate map of Section 3.10 of the pose-reduction chapter, and of the three angle rates only the pitch rate is kept:

$$
\dot\chi=\frac{\omega_y\cos\phi_w+\omega_z\sin\phi_w}{\cos\psi_w},
$$

where $\omega_y$ and $\omega_z$ are components of $\boldsymbol\omega_{TB\_T}$. The roll rate and the yaw rate do not enter anything downstream. $\chi$ is discarded geometrically but its rate is kept: for a rigid wheelset $\dot\chi$ is the rate of the wheel's spin angle, the third X-Z-Y angle, relative to the track frame, and for an axle body it is that body's pitch rate relative to the track frame. Finally the planar curvature at that station, $\kappa=\kappa(s_c)$, is taken for Section 6.1.

## 5. Interface kinematics

### 5.1 Rigid wheelset

The wheel body is the carrier, and the wheel quantities are the carrier quantities:

$$
\mathbf o_{\mathrm{wheel}}=\mathbf r_B,\qquad
\mathbf v_{\mathrm{wheel}}=\mathbf v_B,\qquad
\boldsymbol\omega_{\mathrm{wheel}}=\boldsymbol\omega_B,\qquad
\Omega=\dot\chi.
$$

The two interfaces of one wheelset share these four quantities and share the profile attitude $\bar R_{TW}$ as well.

### 5.2 Axle body with independently rotating wheels

The wheel body is not the carrier. The three basis changes of Section 4.1 are repeated for the wheel body, with the carrier's own $R_{IT}$ and $\mathbf C(s_c)$, giving the position $\mathbf o_{\mathrm{wheel}}$, the inertial velocity $\mathbf v_{\mathrm{wheel}}$ and the inertial angular velocity $\boldsymbol\omega_{\mathrm{wheel}}$ of the wheel origin in $T$. The wheel body's own attitude is not read: its profile attitude is the carrier's $\bar R_{TW}$, its station is the carrier's $s_c$, and it is not projected separately. The pitch-rate scalar is

$$
\Omega=\dot\chi-\dot q_j,
$$

where $\dot q_j$ is the generalized velocity of the revolute joint named by the interface, that is, the one component of $v$ that belongs to that joint.

### 5.3 Origin of the minus sign in the pitch rate

The minus sign above comes from the connection convention, not from the words "independent wheel"; the derivation follows. Let $\mathbf u$ be the positive unit axis of the revolute joint, its sense fixed by the convention that the angular velocity of the wheel relative to the carrier equals $\dot q_j\mathbf u$, so that the effect of the parent-child order is absorbed in the sign of $\mathbf u$; $\mathbf u_W=R_{WB}\mathbf u_B$ is its expression in the wheel-profile frame. The angular velocity of the wheel relative to the track frame exceeds the carrier's by the joint term:

$$
\boldsymbol\omega_{T\,\mathrm{wheel}\_T}=\boldsymbol\omega_{TB\_T}+\dot q_j\,R_{TW}\mathbf u_W.
$$

The implemented formula presupposes the axial condition $\mathbf u_W=-\mathbf e_2$: the positive axis of the revolute joint, expressed in the wheel-profile frame, points opposite to the axle. Then $R_{TW}\mathbf u_W=-R_{TW}\mathbf e_2=-\bar R_{TW}\mathbf e_2=-\mathbf a$, exactly the negative of the third X-Z-Y axis $R_x(\phi_w)R_z(\psi_w)\mathbf e_y$. The forward relation $\boldsymbol\omega=\dot\phi\,\mathbf e_x+\dot\psi\,R_x(\phi)\mathbf e_z+\dot\theta\,R_x(\phi)R_z(\psi)\mathbf e_y$ of Section 3.10 of the pose-reduction chapter shows that an angular velocity of magnitude $\lambda$ lying exactly along the third axis resolves to $(\dot\phi,\dot\psi,\dot\theta)=(0,0,\lambda)$, and that this resolution is unique when $\cos\psi_w\ne0$. The joint term therefore changes only the pitch rate, while the roll and yaw rates equal the carrier's:

$$
\Omega=\dot\chi-\dot q_j.
$$

This is consistent with the wheel and the carrier sharing one $\bar R_{TW}$: the joint only adds a turn about the axle and does not change the axle direction. In general, if the positive axis of the joint is parallel to the axle, $\mathbf u_W=\varepsilon\,\mathbf e_2$ with $\varepsilon=\pm1$, then $\Omega=\dot\chi+\varepsilon\,\dot q_j$ with $\varepsilon=\mathbf u^{\mathsf T}R_x(\phi_w)R_z(\psi_w)\mathbf e_2$ evaluated in $T$; the implementation corresponds to $\varepsilon=-1$. A connection whose positive axis lies along $+\mathbf e_2$ would give a plus sign. A connection whose axis is not parallel to the axle would in general let $\dot q_j$ affect the roll rate or the yaw rate, no longer changing the pitch rate alone, and the wheel would no longer share the carrier's profile attitude; it lies outside this model. The resulting $\Omega$ is the pitch rate obtained by resolving the wheel body's angular velocity relative to the track frame at the carrier's $(\phi_w,\psi_w)$, and it is the $\Omega$ of the creepage chapter: in the right-handed frame with $y_T$ to the right and $z_T$ downward it is negative in forward rolling.

## 6. Profile motion and irregularity input

The quantities below are formed interface by interface, with the fixed geometry of the side the interface belongs to.

### 6.1 Effective yaw and effective station

The profile cross-section is selected in two stages as in Section 3.9 of the pose-reduction chapter. First the spatial slope of the lateral irregularity is sampled at the shared station $s_c$ and corrects the carrier yaw:

$$
\psi_e=\psi_w-\operatorname{SafeAtan2Ratio}\!\left(y_\epsilon'(s_c)\,\dot s_c,\ \dot s_c\right),
$$

then the corrected direction selects the effective station of the side:

$$
s_e=s_c+\frac{-\sigma\sin\psi_e}{1-\kappa\left(y_B+\sigma\cos\psi_e\right)}.
$$

$\operatorname{SafeAtan2Ratio}$ is the ratio angle with a denominator floor of Section 3.2 of the pose-reduction chapter. This is the effective cross-section station map under the corrected direction $\psi_e$: the numerator is the longitudinal reach computed with $\psi_e$ rather than $\psi_w$, and in general differs from the longitudinal offset $-\sigma\sin\psi_w$ of the actual profile datum from the body origin in Section 6.4; the denominator is the planar curvature correction factor and has the same form as the station-rate denominator of Section 4.3 only under planar conditions. The derivation and properties of the two-stage selection are in Section 3.9 of the pose-reduction chapter and are not repeated here. When $s_c$ lies outside $I_T$, or when there is no irregularity field, the slope at $s_c$ is zero and $\psi_e=\psi_w$ in forward motion with $\dot s_c$ above the floor.

### 6.2 Sampling the irregularity

The fields entering this assembly are $y_\epsilon$ and $z_\epsilon$ of Section 3.9 of the pose-reduction chapter: the natural cubic spline inside the intersection of the channel's domain with $I_T$, and zero outside; without an irregularity field all four quantities are identically zero. Apart from the one lateral slope taken at $s_c$ in Section 6.1, the displacements and both slopes of the side are sampled at the effective station: $y_\epsilon(s_e)$, $z_\epsilon(s_e)$, $y_\epsilon'(s_e)$ and $z_\epsilon'(s_e)$. Whether $s_e$ lies in $I_T$ is decided independently of $s_c$. The time rates are formed with the carrier's station rate:

$$
\dot y_\epsilon=y_\epsilon'(s_e)\,\dot s_c,\qquad
\dot z_\epsilon=z_\epsilon'(s_e)\,\dot s_c.
$$

A strict chain rule would give $y_\epsilon'(s_e)\,\dot s_e$; the implementation does not form $\dot s_e$ and uses the carrier's $\dot s_c$. The two rates are auxiliary inputs formed for the downstream construction of the direction angles, paired with the carrier station rate; they are not the complete time derivatives of the values sampled at the effective station. Within the pose reduction this pairing has no further consequence: the two rates enter the pose reduction only through the ratios $\operatorname{SafeAtan2Ratio}(\dot y_\epsilon,\dot s_c)$ and $\operatorname{SafeAtan2Ratio}(\dot z_\epsilon,\dot s_c)$ of Section 3.2 of the pose-reduction chapter, whose denominator is the same $\dot s_c$, so the resulting direction angles depend only on the spatial slopes at $s_e$ and on the sign of $\dot s_c$ and whether it exceeds the floor. Nor are they a velocity of the rail material; the contact model keeps the rail material velocity at zero as in Section 2 of the single-wheel chapter.

### 6.3 Input to the pose reduction

The input of Section 3 of the pose-reduction chapter is assembled here as: the carrier placement $(y_B,z_B,\phi_w,\psi_w)$, that is, the lateral and vertical coordinates of the body origin in $T$ and the two angles of the spin-free attitude; the station rate $\dot s_c$; the irregularity displacements and rates of Section 6.2; and the fixed geometry of the side. With independently rotating wheels the placement is still the carrier's, not the wheel body's, as Section 1 of the pose-reduction chapter states. $\sigma$ and the rolling radius are applied inside the reduction by Section 3.5 of that chapter and are not added to the placement in advance. The reduction returns the four pose scalars $\varphi$, $\beta$, $d_y$ and $d_z^{\uparrow}$, which are the first part of the contact input.

### 6.4 Rigid motion of the wheel profile

The second part of the contact input is the rigid motion of the wheel-profile datum, composed from the wheel-body quantities and the spin-free attitude:

$$
\mathbf o_W=\mathbf o_{\mathrm{wheel}}+\sigma\,\bar R_{TW}\mathbf e_2=\mathbf o_{\mathrm{wheel}}+\sigma\,\mathbf a,\qquad
\mathbf v_o=\mathbf v_{\mathrm{wheel}}+\boldsymbol\omega_{\mathrm{wheel}}\times\left(\mathbf o_W-\mathbf o_{\mathrm{wheel}}\right).
$$

The attitude is $\bar R_{TW}$, the angular velocity is $\boldsymbol\omega_{\mathrm{wheel}}$, the arc rate is the path rate $\dot\ell$ and the pitch-rate scalar is the $\Omega$ of Section 5. The second formula is the transport of velocity between two points of a rigid body, the same formula as Section 2.2 of [Force-element kinematics and spatial wrenches](../force_elements/FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md).

Geometric de-spinning and material de-spinning must be kept apart here. The attitude $\bar R_{TW}$ contains no spin, because placing an axisymmetric profile does not need it; the angular velocity $\boldsymbol\omega_{\mathrm{wheel}}$ contains the spin, because the material velocity $\mathbf v_o+\boldsymbol\omega_{\mathrm{wheel}}\times(\mathbf x_R-\mathbf o_W)$ at the rail material reference point of Section 3.1 of the single-wheel chapter needs it: the lever arm $\mathbf x_R-\mathbf o_W$ has a radial component, and this is how the spin enters the creepages. The transport term $\boldsymbol\omega_{\mathrm{wheel}}\times\sigma\mathbf a$ in the datum velocity, however, contains no spin rate: decomposing $\boldsymbol\omega_{\mathrm{wheel}}$ into a component along the axle and a component perpendicular to it, $\mathbf o_W-\mathbf o_{\mathrm{wheel}}=\sigma\mathbf a$ is parallel to the former and only the latter survives the cross product. The spin of a rigid wheelset is along $\mathbf a$, and under the axial condition of Section 5.3 the joint rotation of an independent wheel is along $\mathbf a$ too, so for both kinds of carrier the transport term is fixed by the yaw, roll and track-following components of the angular velocity alone; $\mathbf v_o$ itself is the inertial velocity $\mathbf v_{\mathrm{wheel}}$ of the wheel origin plus this transport term. The spin rate enters the rim speed of Section 3.1 of the creepage chapter separately, as $\Omega$.

### 6.5 Placement of the rail profile

The third part of the contact input is the rigid placement of the rail profile in $T$. The profile origin is first placed in the track frame at the effective station $s_e$, at the lateral and vertical coordinates datum plus irregularity displacement, and then taken into $T$:

$$
\mathbf o_c=R_{IT}^{\mathsf T}\left(\mathbf C(s_e)+R_{IT}(s_e)\begin{bmatrix}0\\ y_r+y_\epsilon(s_e)\\ z_r+z_\epsilon(s_e)\end{bmatrix}-\mathbf C(s_c)\right).
$$

The placement attitude is the five-factor product $R_{T\mathrm{rail}}=R_{IT}^{\mathsf T}R_{IT}(s_e)\,R_z(\widehat\psi_\epsilon)\,R_y(\widehat\theta_\epsilon)\,R_x(\phi_r)$ of Section 3.8 of the pose-reduction chapter, whose two hatted angles are formed directly from the spatial slopes $y_\epsilon'(s_e)$ and $z_\epsilon'(s_e)$ at $s_e$. The longitudinal-origin convention is applied as in that section too: the profile-coordinate convention translates $\mathbf o_c$ with the wheel-body origin $\mathbf o_{\mathrm{wheel}}$ as reference and $\Delta s=s_e-s_c$ as the target longitudinal coordinate, while the track-station convention leaves $\mathbf o_c$ unchanged; the reference must be the wheel-body origin and not the datum $\mathbf o_W$, which already contains $\sigma$, for the reason given there. The complete derivation of the effective station and of the rail-profile placement is in Sections 3.8 to 3.10 of the pose-reduction chapter, and this chapter only links to it.

The four pose scalars, the rail-profile placement and the rigid motion of the wheel profile together form the contact input listed in Section 2 of the single-wheel chapter, and the contact model returns the paired wrenches of zero, one or several contact patches by the algorithm of Section 6 of that chapter.

## 7. The equivalent body wrench

### 7.1 Per-patch transport and summation

Let an interface obtain $K$ loaded contact patches, $K=0$ included. The rail-on-wheel wrench $(\mathbf m_{P,k},\mathbf f_{T,k})$ of patch $k$ is reduced about the wheel-side application point $\mathbf x_{P,k}$ and expressed in $T$, see Section 5.3 of the single-wheel chapter; the current model emits no direct within-patch spin moment, $\mathbf m_{P,k}=\mathbf 0$, see Section 4 of that chapter, and the formula below is written in the general form. Each patch wrench is first transported to the wheel-body origin as in Section 5.2 of the conventions:

$$
\boldsymbol\tau_k=\mathbf m_{P,k}+\left(\mathbf x_{P,k}-\mathbf o_{\mathrm{wheel}}\right)\times\mathbf f_{T,k},
$$

with the lever arm running from the new reduction point to the old one, as in Section 5.1 of the single-wheel chapter. Then the wrenches are summed:

$$
\mathbf f_T=\sum_{k=1}^{K}\mathbf f_{T,k},\qquad
\boldsymbol\tau_T=\sum_{k=1}^{K}\boldsymbol\tau_k.
$$

The order, transport first and sum second, cannot be reversed: wrenches reduced about different points cannot be added. The reduction point is the origin of the wheel body, not the profile datum $\mathbf o_W$; with independently rotating wheels it is taken on the wheel body rather than on the carrier, and whether the two origins coincide in space is decided by the connection and does not change the definition of the reduction point. For $K=0$ both sums are zero vectors.

### 7.2 Change of basis and output

The summed wrench is still in $T$; following Section 5.2 of the conventions it is rotated into the inertial frame, changing basis without changing point:

$$
\mathbf f^I=R_{IT}\,\mathbf f_T,\qquad
\boldsymbol\tau^I=R_{IT}\,\boldsymbol\tau_T.
$$

The matrix is the $R_{IT}$ at the carrier station $s_c$, the same one that took the quantities into $T$ in Section 4.1. The result is written as one body-wrench entry acting on the wheel body: the application point is the origin $\mathbf 0$ in the body frame, the expressed-in frame is the world frame, and the moment is taken about that point, that is,

$$
\mathcal W^{I}_{\mathrm{wheel}\,o}=\left(\boldsymbol\tau^I,\ \mathbf f^I\right).
$$

How the multibody layer accepts this entry is described in Section 4.3 of [Multibody equations of motion](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.en.md): the entry is already at the body origin, the multibody layer only accumulates it into that body's spatial force, and the projection to generalized forces happens in the forward-dynamics recursion, not in this chapter. Each interface writes exactly one entry; an interface without contact writes a zero wrench rather than nothing, so the length and order of the entry list are fixed at construction and do not change with the contact state.

### 7.3 The rail side

For every patch the contact model returns both the rail-on-wheel and the wheel-on-rail halves, see Section 5.3 of the single-wheel chapter. This chapter consumes only the rail-on-wheel half: the rail is geometry carried by the line and belongs to the fixed world, it has no state and no inertia, and there is no rigid body in the multibody tree that could receive the wheel-on-rail load, in agreement with the stationary-rail assumption of Section 2 of the single-wheel chapter. The entire output of the contact force plan is therefore one wheel-body wrench per interface, and no rail-side load enters the multibody system.

## 8. Computational implementation

One evaluation has two parts. The first is a serial preparation that contains every multibody query:

1. For each carrier: read the body-origin pose and obtain $s_c$ by the local-branch projection from the seed (Section 3).
2. For each carrier: read the body-frame spatial velocity, evaluate the track-frame pose and its station derivative at $s_c$, and form $\mathbf r_B$, $\mathbf v_B$, $\boldsymbol\omega_B$, $(\phi_w,\psi_w)$, $\dot s_c$, $\dot\ell$, $\dot\chi$ and $\kappa$ (Section 4).
3. For each interface: a rigid wheelset reuses the carrier quantities directly; an independent wheel reads the wheel body's pose and spatial velocity, takes them into the carrier's $T$, reads the revolute-joint velocity and forms $\Omega$ (Section 5).

The second part evaluates the interfaces one by one and reads only the quantities prepared in the first: it forms $\psi_e$, $s_e$ and the irregularity samples, assembles the pose-reduction input, rebuilds $\bar R_{TW}$ and forms the wheel-profile rigid motion and the rail-profile placement, obtains the four pose scalars (Section 6), calls the side's contact model, transports the patch wrenches to the wheel origin, sums them, rotates the sum into the inertial frame and writes it out (Section 7). The evaluations of different interfaces exchange no data; each result depends only on the output of the first part and on the fixed geometry of its own interface.

At the endpoint of an accepted internal step there is a further evaluation that performs step 1 only: it reprojects all carriers from the current seeds and replaces the seeds as a whole (Section 3.2), evaluating no velocities, no track-frame attitude and no contact.

## 9. Mathematical properties and conditions of applicability

- **Validity of the local branch.** The station comes from the local branch identified by the seed, not from a whole-line closest point. The premise is that the current body origin lies near a regular root with $f''>0$ and that the seed lies in the attraction domain reachable within two Newton corrections; the size of that domain depends on curvature, grade and the distance of the body origin from the centerline, and Section 4.4 of the line-geometry chapter gives no uniform radius. The seeds move forward with the accepted steps, which keeps the branch continuous; the line-geometry layer stores no history.
- **Functional character of the right-hand side.** With the seeds fixed, the map from $(q,v)$ to the body wrenches is a well-defined function, and the projection result changes no integrated quantity; the seeds are updated only at accepted endpoints, so within one internal step the right-hand side is a single-valued function of $(t,y)$.
- **The two denominators.** The station-rate denominator $\mathbf e_1^{\mathsf T}(\mathbf c'+\boldsymbol\omega_T\times\mathbf r_B)$ equals $f''(s_c)/\lVert\mathbf C'(s_c)\rVert$ at an exact projection and is positive when the projection is regular; the effective-station denominator $1-\kappa(y_B+\sigma\cos\psi_e)$ must be nonzero, and as it approaches zero the local map from lateral offset to centerline station becomes geometrically singular, as Section 5 of the pose-reduction chapter already lists. On a planar curve without grade and without superelevation both have the form $1-\kappa y$, using the body-origin lateral coordinate $y_B$ and the lateral coordinate $y_B+\sigma\cos\psi_e$ of the effective map respectively; the latter is formed with $\psi_e$ rather than $\psi_w$ and is not the lateral coordinate of the actual profile datum of Section 6.4.
- **Singularity of the X-Z-Y resolution.** The attitude resolution is singular at $\cos\psi_w=0$, and the pitch rate has the same $\cos\psi_w$ in its denominator; the resolution confines $\psi_w$ to the closed quarter turn. $\lvert\psi_w\rvert=\pi/2$ means an axle along the line, a boundary of the formulas rather than an operating attitude of a rail vehicle. The uniqueness argument of Section 5.3 also requires $\cos\psi_w\ne0$.
- **Domain of the fields.** The irregularity is zero outside the channel domains and outside the line's defined interval, and the interval classifications of $s_c$ and $s_e$ are independent of each other; where a boundary value is nonzero the field is discontinuous there, see Section 5 of the pose-reduction chapter. The line itself continues as a tangent outside its defined interval, so the projection and the track frame remain defined there; only the irregularity is zero.
- **Geometric versus material de-spinning.** The profile attitude $\bar R_{TW}$ contains no spin, justified by the axisymmetry of the profile; the angular velocity $\boldsymbol\omega_{\mathrm{wheel}}$ contains the spin, justified by the material velocity needing it. The transport term in the datum velocity $\mathbf v_o$ contains no spin rate, and the spin enters the creepage reference speed as the scalar $\Omega$. For a profile that is not axisymmetric, discarding $\chi$ would no longer be valid; this model does not cover that case.
- **Nature of the velocities.** The $\mathbf v_o$ and $\boldsymbol\omega_{\mathrm{wheel}}$ handed to the contact model are inertial quantities expressed in $T$, with no transport term subtracted; the only quantity relative to the track frame is the $\boldsymbol\omega_{TB\_T}$ used to form the pitch rate. This matches a rail fixed in $I$: the inertial velocity of the rail material is zero, so the inertial velocity of the wheel material is the relative velocity.
- **One track frame.** All quantities of a carrier and of its two interfaces are expressed in $T(s_c)$; the track frame at the effective station $s_e$ enters only the rail-profile placement, its origin through $\mathbf C(s_e)$ and $R_{IT}(s_e)$ and its attitude through the first two factors, and the equivalent wrench is rotated back to the inertial frame with $R_{IT}(s_c)$. The track frame at $s_e$ is used to express no velocity.
- **The axial condition.** The independent-wheel pitch-rate formula $\Omega=\dot\chi-\dot q_j$ presupposes that the positive axis of the revolute joint lies along $-\mathbf e_2$ of the wheel-profile frame; along $+\mathbf e_2$ the sign is reversed, and a connection whose axis is not parallel to the axle lies outside this model. This is a consequence of the connection convention, not a general law of independent wheels.
- **Scope of the constant rotation.** $R_{BW}$ only realigns coordinate axes; it removes no spin, contains no time and does not move the body origin. The datum offset $\sigma$ is not part of $R_{BW}$; it is applied in Section 6.4 along the spin-free axle direction $\mathbf a$.

## 10. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| Carrier definition, revolute-joint definition and interface definition | `WheelRailContactCarrierDefinition`, `IndependentWheelRevoluteJointDefinition`, `WheelRailContactInterfaceDefinition`, in [`wheel_rail_contact_force_plan.h`](../../../libs/forces/include/orvd/forces/wheel_rail_contact_force_plan.h) |
| Per-side contact model, fixed geometry and rail longitudinal-origin convention | `WheelRailContactRuntimePersonality`, in [`wheel_rail_contact_runtime_personality.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_contact_runtime_personality.h); `WheelRailPoseConstants`, `RailProfileOriginMode`, in [`wheel_rail_pose.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_pose.h) |
| Pose and spatial velocity of the carrier and wheel bodies in the world frame | `MultibodyModel::CalcPoseInWorld`, `MultibodyModel::CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld`, in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Local-branch projection of the carrier station $s_c$ | `WheelRailContactForcePlan::EvaluateCarrierProjections`, in [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc); `TrackGeometry::ProjectPointOntoSeededBranch`, in [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| Initial seeds, read-only use by the right-hand side and update at accepted endpoints | Constructor of `SystemRuntimeContext`, `SystemInstance::UpdateWheelRailProjectionStationHints`, in [`system_instance.cc`](../../../libs/system_assembly/src/system_instance.cc); `CompiledSystemPlan::CalcStateTimeDerivatives`, in [`compiled_system_plan.cc`](../../../libs/system_assembly/src/compiled_system_plan.cc); `WheelRailContactForcePlan::CalcProjectionStationHints`, in [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc); `AdvanceToImpl`, in [`system_continuous_state_advancer.cc`](../../../libs/integrators/src/system_continuous_state_advancer.cc) |
| Track-frame pose and its station derivative | `TrackGeometry::EvaluateTrackFrame`, `TrackFrameKinematics`, in [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) and [`track_frame_pose.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_frame_pose.h) |
| Change into the track frame, X-Z-Y resolution, station rate, path rate and pitch rate | `WheelRailContactForcePlan::CompleteCarrierKinematics`, in [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc); `ResolveRollYawPitch`, `ResolveRollYawPitchRates`, in [`roll_yaw_pitch.cc`](../../../libs/wheel_rail_contact/src/roll_yaw_pitch.cc) |
| Interface kinematics and the pitch-rate scalar $\Omega$ | `WheelRailContactForcePlan::CompleteInterfaceKinematics`, in [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc) |
| Effective yaw, effective station, irregularity sampling, reconstruction of $\bar R_{TW}$, wheel-profile rigid motion and rail-profile placement | `evaluate_interface` within `CalcAppliedForcesImpl`, in [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc); `SafeAtan2Ratio`, `PlaceRailProfileLongitudinalOrigin`, in [`wheel_rail_pose.cc`](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc); `TrackIrregularityField`, in [`track_irregularity_field.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/track_irregularity_field.h) |
| The three parts of the contact input and the per-patch result | `WheelRailContactInput`, `RailProfileFrame`, `WheelProfileRigidMotion`, `WheelRailContactResult`, in [`wheel_rail_contact_model.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_contact_model.h) |
| Per-patch transport, summation and change of basis | `TransportWrench`, `RotateWrench`, in [`contact_wrench.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_wrench.h) and [`contact_wrench.cc`](../../../libs/wheel_rail_contact/src/contact_wrench.cc) |
| Body-wrench entry acting on the wheel body | `AppliedBodyWrench`, in [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
