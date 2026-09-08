[中文](HALF_ANGLE_MIDPOINT_RPY_BUSHING.md)

# Half-angle midpoint RPY bushing

This chapter states and derives the constitutive law of the six-component linear bushing family. Its three translational components are stated in the half-angle intermediate frame B that lies midway between the attitudes of the two ends, with the relative material velocity of the two bodies at their instantaneous common midpoint as velocity input; its three rotational components are stated in the space-XYZ roll, pitch and yaw angles of the opposite end in the reference end, with the time derivatives of those angles as rate input, and the physical moment is recovered by power conjugacy. The resulting force and moment are applied as a pair of opposite wrenches at the instantaneous world midpoint of the two origins, with no support moment. The relative motion of the two ends, the reduction-point rules for wrenches and the power identities of the three application schemes are given in [force-element kinematics and spatial wrenches](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md), referred to below as the shared chapter; this chapter derives only what is specific to the family — the half-angle frame, the midpoint velocity, the angular coordinates and their rate map — and proves that the family satisfies the organizing principle of Section 4.5 of the shared chapter. The type and its contract are `HalfAngleMidpointRollPitchYawBushing` in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h); the evaluation is in `VehicleForcePlan::CalcAppliedForces` in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc).

## 1. Objects and notation

### 1.1 The element

A bushing connects two frames, end A and end C, rigidly fixed to the rigid bodies $\mathcal A$ and $\mathcal C$. As in Section 1.1 of the shared chapter, the family takes A as the reference end and C as the opposite end when it evaluates the relative motion, but neither set of constitutive constants is stated directly in A: the translational stiffness $\mathbf k_t$ and damping $\mathbf c_t$ are diagonal along the three axes of the half-angle intermediate frame B, and the rotational stiffness $\mathbf k_r$ and damping $\mathbf c_r$ are diagonal in the three angular coordinates $\boldsymbol\eta$. The definition of the family carries three contracts: the translational velocity input is the relative material velocity of the two bodies at their instantaneous common midpoint; the rotational rate input is the time derivative of the angular coordinates, and the physical moment follows from power conjugacy; both wrenches are applied at the instantaneous world midpoint, with no reference-end support moment. The family has no internal state: every load is an algebraic function of the current $(q,v)$ and occupies no part of the $z$ block of Section 5.1 of the conventions.

### 1.2 Notation

| Symbol | Meaning | Source |
|---|---|---|
| A, C, $\mathcal A$, $\mathcal C$, I | The two end frames, the bodies carrying them and the inertial frame | Shared chapter §1.1 |
| B | Half-angle intermediate frame, with $R_{AB}=R_{BC}$ | This chapter §2.2 |
| $\mathbf p_A$, $\mathbf p_C$, $\mathbf d_I$, $\mathbf d_A$ | Positions of the two origins and their relative position $\mathbf p_C-\mathbf p_A$ | Shared chapter §1.2, §2.1 |
| $\mathbf d_B$, $\mathbf d_C$ | The relative position expressed in B and in C | This chapter §2.3 |
| $R_{IA}$, $R_{IC}$, $R_{AC}$ | Attitudes of the two ends and their relative attitude | Shared chapter §2.1 |
| $R_{AB}$, $R_{BA}$, $R_{IB}$ | Attitudes involving the half-angle frame | This chapter §2.2 |
| $q_{AC}=(q_0,\mathbf q)$ | Unit quaternion of $R_{AC}$ with non-negative scalar part, $\mathbf q=(q_1,q_2,q_3)^{\mathsf T}$ | This chapter §2.2 |
| $\theta$, $\mathbf n$ | Rotation angle and unit axis of $R_{AC}$ | This chapter §2.2 |
| $\mathbf v_{Ao}$, $\mathbf v_{Co}$, $\boldsymbol\omega_A$, $\boldsymbol\omega_C$ | Velocities of the two origins and angular velocities of the two bodies, expressed in I | Shared chapter §1.2 |
| $\mathbf u_I$, $\mathbf u_A$ | Velocity of the opposite-end origin relative to the reference end, $\mathbf u_A=\dot{\mathbf d}_A$ | Shared chapter §2.3 |
| $\boldsymbol\omega_{rel,I}$, $\boldsymbol\omega_{rel,A}$ | Angular velocity of C relative to A | Shared chapter §2.4 |
| $\boldsymbol\omega_{AB,A}$ | Angular velocity of B relative to A, expressed in A | This chapter §2.4 |
| $\mathbf x_m$, $\mathbf v^{(\mathcal A)}(\mathbf x)$ | Instantaneous world midpoint; velocity of the material point of a body coinciding with $\mathbf x$ | Shared chapter §2.5, §2.2 |
| $\mathbf u_{m,I}$, $\mathbf u_{m,A}$, $\mathbf u_{m,B}$ | Midpoint relative material velocity expressed in I, A and B | Shared chapter §2.5; the B form is this chapter §2.4 |
| $\boldsymbol\eta=(\eta_1,\eta_2,\eta_3)^{\mathsf T}$ | Space-XYZ roll, pitch and yaw of C in A | This chapter §2.6 |
| $\mathrm c_i$, $\mathrm s_i$ | $\cos\eta_i$, $\sin\eta_i$ | This chapter §2.6 |
| $\sigma_+$, $\sigma_-$ | The two half-sum and half-difference angles of the extraction | This chapter §2.6 |
| $E(\boldsymbol\eta)$, $H(\boldsymbol\eta)$ | $\boldsymbol\omega_{rel,A}=E\dot{\boldsymbol\eta}$ and its inverse $\dot{\boldsymbol\eta}=H\boldsymbol\omega_{rel,A}$ | This chapter §2.7 |
| $\mathbf k_t$, $\mathbf c_t$, $\mathbf k_r$, $\mathbf c_r$ | Diagonal translational and rotational stiffness and damping vectors | This chapter §1.1 |
| $\mathbf f_{C,B}$, $\mathbf f_{C,I}$, $\boldsymbol\tau_{C,A}$, $\boldsymbol\tau_{C,I}$ | Force and moment on end C; the second subscript is the expressed-in frame | Two-subscript form of the shared chapter §1.3 |
| $\boldsymbol\tau_\eta$ | Generalized moment conjugate to $\dot{\boldsymbol\eta}$ | This chapter §2.8 |
| $\mathcal W_m^I[\mathcal A]$, $\mathbf r_m^{\mathcal A}$, $\mathbf x_{\mathcal A}$, $R_{I\mathcal A}$ | Midpoint wrench, body coordinates of the midpoint, body pose | Shared chapter §3.1, §3.4 |
| $\mathcal P$, $\mathcal V_t$, $\mathcal V_r$ | Total power delivered to the two bodies; translational and rotational stored energy | Shared chapter §1.2; the stored-energy subscripts are this chapter §4 |
| $\mathbf a\circ\mathbf b$, $\operatorname{skew}(\mathbf w)$ | Componentwise product; skew-symmetric matrix | Shared chapter §1.2 |

### 1.3 Notational remarks

This chapter keeps every convention of Section 1.3 of the shared chapter: $R_{AB}$ maps components in B to components in A; $\mathbf u$, $\boldsymbol\omega_{rel}$ and $\mathbf u_m$ are always measured relative to A and their last subscript names the expressed-in frame only; a load on end C carries two subscripts, the loaded end first and the expressed-in frame second. The law of this family is naturally stated in terms of the loads on end C, so the text writes $\mathbf f_{C,B}$ and $\boldsymbol\tau_{C,A}$ throughout; the loads on end A are their negatives. Stiffness and damping are lowercase as required by Section 5.3 of the conventions, with subscripts $t$ and $r$ distinguishing translation from rotation.

The following symbols have their stated meaning in this chapter only: $\boldsymbol\eta$ is a vector of attitude coordinates, unrelated to the wheel-profile lateral station $\eta$ and to the irregularities $\eta_y$, $\eta_z$ of the wheel-rail chain; $\theta$ and $\mathbf n$ are the rotation angle and unit axis of $R_{AC}$; the quaternion components $(q_0,q_1,q_2,q_3)$ correspond in order to the $(w,x,y,z)$ of Section 4.2 of the conventions; $\sigma_\pm$ are intermediate quantities of the angle extraction. The cosine and sine abbreviations $\mathrm c_i$, $\mathrm s_i$ are set upright to keep them apart from the italic $c$ that Section 5.3 of the conventions reserves for damping; $E(\boldsymbol\eta)$ is the rate-map matrix of Section 2.7, not the placeholder E that marks the expressed-in frame in the shared chapter's wrench symbol $\mathcal W_Q^E$. A matrix entry $R_{ij}$ without a frame subscript always denotes an entry of $R_{AC}$, numbered from 1 as in Section 4.1 of the conventions; when it points at an Eigen expression, both indices are one less.

## 2. Model and derivation

### 2.1 Where the relative motion comes from

The family consumes the whole relative motion of Section 2 of the shared chapter: $\mathbf d_A=R_{IA}^{\mathsf T}(\mathbf p_C-\mathbf p_A)$ and $R_{AC}=R_{IA}^{\mathsf T}R_{IC}$ (shared chapter §2.1), the relative velocity with its transport term $\mathbf u_A=\dot{\mathbf d}_A$ (shared chapter §2.3), and the relative angular velocity $\boldsymbol\omega_{rel,A}$ with its relation $\dot R_{AC}=\operatorname{skew}(\boldsymbol\omega_{rel,A})R_{AC}$ to the derivative of the relative attitude (shared chapter §2.4). None of these is re-derived here; the half-angle frame, the midpoint velocity and the angular coordinates are built on top of them.

### 2.2 The half-angle intermediate frame B

$R_{AC}$ is a rotation; let $\theta$ be its angle and $\mathbf n$ its unit axis. Every rotation corresponds to exactly two unit quaternions $\pm q$; the family takes the one with non-negative scalar part and uses it to fix the range of $\theta$:

$$
q_{AC}=\left(q_0,\ \mathbf q\right)=\left(\cos\tfrac{\theta}{2},\ \sin\tfrac{\theta}{2}\,\mathbf n\right),
\qquad
q_0=\tfrac12\sqrt{1+\operatorname{tr}R_{AC}}\ \ge0,
\qquad
\theta\in[0,\pi].
$$

The half-angle intermediate frame B is the frame rotated about the same axis through half the angle. Its quaternion follows from $q_{AC}$ by the half-angle formulas:

$$
q_{AB}=\left(\sqrt{\tfrac{1+q_0}{2}},\ \frac{\mathbf q}{2\sqrt{(1+q_0)/2}}\right)
=\left(\cos\tfrac{\theta}{4},\ \sin\tfrac{\theta}{4}\,\mathbf n\right).
$$

The second equality uses $\cos\tfrac\theta4=\sqrt{(1+\cos\tfrac\theta2)/2}$ and $\sin\tfrac\theta4=\sin\tfrac\theta2\big/\left(2\cos\tfrac\theta4\right)$, both with the positive root because $\theta/4\in[0,\pi/4]$, so the scalar part of $q_{AB}$ is again non-negative; $\lVert\mathbf q\rVert^2=1-q_0^2$ shows directly that $q_{AB}$ is a unit quaternion. Converting $q_{AB}$ to a rotation matrix gives $R_{AB}$. Since $R_{AB}$ shares its axis with $R_{AC}$ and halves its angle,

$$
R_{AB}=R_{BC},
\qquad
R_{AC}=R_{AB}R_{BC}=R_{AB}^{2},
\qquad
R_{BA}=R_{AB}^{\mathsf T},
\qquad
R_{IB}=R_{IA}R_{AB}.
$$

B is equidistant in attitude from A and from C: the rotation taking A to B is the same rotation that takes B to C.

What the sign of the scalar part decides is which square root of $R_{AC}$ becomes $R_{AB}$. $-q_{AC}$ represents the same rotation, written as a rotation through $2\pi-\theta$ about $-\mathbf n$; for $0<\theta<\pi$ its half-angle quaternion $\left(\sqrt{(1-q_0)/2},\ -\mathbf q/(2\sqrt{(1-q_0)/2})\right)$ is the rotation through $\pi-\theta/2$ about $-\mathbf n$, the other square root of $R_{AC}$; at $\theta=0$ that expression has a zero denominator, the identity has more than two square roots (every half-turn about any axis is one), and the principal half-angle root taken is the identity itself. Choosing $q_0\ge0$ selects the square root with rotation angle $\theta/2\le\pi/2$ — the branch nearer the identity — and that branch is a continuous function of $R_{AC}$ for $\theta<\pi$. At $\theta=\pi$, $q_0=0$: neither representative has a negative scalar part, the two square roots are equally far from the identity and the construction no longer has a preferred branch. The continuous domain of the principal half-angle branch is therefore the set of relative attitudes with $\theta<\pi$. It is a different set from the invertibility domain $\lvert\eta_2\rvert<\tfrac\pi2$ of the angular coordinates in Section 2.7: $R_x(\pi)$ has zero pitch yet lies on the cut of the half-angle branch.

### 2.3 The relative position in the half-angle frame

The translational deformation measure is the relative position of the two origins expressed in B:

$$
\mathbf d_B=R_{BA}\mathbf d_A=R_{BC}\mathbf d_C,
\qquad
\mathbf d_C=R_{AC}^{\mathsf T}\mathbf d_A.
$$

The second equality follows from $R_{BC}R_{AC}^{\mathsf T}=R_{AB}R_{AB}^{\mathsf T}R_{AB}^{\mathsf T}=R_{BA}$. It shows that $\mathbf d_B$ favors neither end: the relative position as seen from A, turned through half the relative angle, and the relative position as seen from C, turned back through half the relative angle, are one and the same vector. What Section 2.1 of the shared chapter says about $\mathbf d_A$ carries over: $\mathbf d_B$ is a difference of origins, carries no natural length and vanishes when the two origins coincide.

### 2.4 Relative material velocity at the instantaneous midpoint

The instantaneous world midpoint of the two origins is $\mathbf x_m=\mathbf p_A+\tfrac12\mathbf d_I$. It is not a fixed material point of either body; at every instant, body $\mathcal A$ and body $\mathcal C$ each have one material point coinciding with it. By the transport formula of Section 2.2 of the shared chapter, those two material points move with

$$
\mathbf v^{(\mathcal A)}(\mathbf x_m)=\mathbf v_{Ao}+\boldsymbol\omega_A\times\left(\mathbf x_m-\mathbf p_A\right)=\mathbf v_{Ao}+\tfrac12\,\boldsymbol\omega_A\times\mathbf d_I,
$$

$$
\mathbf v^{(\mathcal C)}(\mathbf x_m)=\mathbf v_{Co}+\boldsymbol\omega_C\times\left(\mathbf x_m-\mathbf p_C\right)=\mathbf v_{Co}-\tfrac12\,\boldsymbol\omega_C\times\mathbf d_I,
$$

where $\mathbf x_m-\mathbf p_A=\tfrac12\mathbf d_I$ and $\mathbf x_m-\mathbf p_C=-\tfrac12\mathbf d_I$ were used. The relative material velocity of the two bodies at their common midpoint is the difference of the two:

$$
\mathbf u_{m,I}=\mathbf v^{(\mathcal C)}(\mathbf x_m)-\mathbf v^{(\mathcal A)}(\mathbf x_m)
=\left(\mathbf v_{Co}-\mathbf v_{Ao}\right)-\tfrac12\left(\boldsymbol\omega_A+\boldsymbol\omega_C\right)\times\mathbf d_I.
$$

This expression is symmetric in the two bodies: exchanging A and C only reverses $\mathbf d_I$ and the overall sign. Its relation to the relative velocity with transport term of Section 2.3 of the shared chapter, $\mathbf u_I=\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I$, follows by adding and subtracting $\boldsymbol\omega_A\times\mathbf d_I$ inside the bracket:

$$
\mathbf u_{m,I}
=\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right)
-\tfrac12\left(\boldsymbol\omega_C-\boldsymbol\omega_A\right)\times\mathbf d_I
=\mathbf u_I-\tfrac12\,\boldsymbol\omega_{rel,I}\times\mathbf d_I.
$$

Cross products commute with rotations, so in A and then in B,

$$
\mathbf u_{m,A}=\mathbf u_A-\tfrac12\,\boldsymbol\omega_{rel,A}\times\mathbf d_A,
\qquad
\mathbf u_{m,B}=R_{BA}\,\mathbf u_{m,A}.
$$

This is the midpoint velocity registered in Section 2.5 of the shared chapter, and it is the input of the translational damping of this family. The same quantity has two equivalent forms: starting from the difference of the world velocities of the two origins, the correction is $-\tfrac12(\boldsymbol\omega_A+\boldsymbol\omega_C)\times\mathbf d$; starting from $\mathbf u$, which already contains the transport term, the correction is $-\tfrac12\boldsymbol\omega_{rel}\times\mathbf d$. The two must not be mixed.

$\mathbf u_{m,B}$ is in general not the time derivative of $\mathbf d_B$. Let $\boldsymbol\omega_{AB,A}$ be the angular velocity of B relative to A, so that $\dot R_{AB}=\operatorname{skew}(\boldsymbol\omega_{AB,A})R_{AB}$. Differentiating $\mathbf d_B=R_{AB}^{\mathsf T}\mathbf d_A$ in the same way as $\dot{\mathbf d}_A$ is derived in Section 2.3 of the shared chapter gives

$$
\dot{\mathbf d}_B=R_{BA}\left(\mathbf u_A-\boldsymbol\omega_{AB,A}\times\mathbf d_A\right),
\qquad
\mathbf u_{m,B}-\dot{\mathbf d}_B=R_{BA}\left[\left(\boldsymbol\omega_{AB,A}-\tfrac12\,\boldsymbol\omega_{rel,A}\right)\times\mathbf d_A\right].
$$

The angular velocity of the half-angle frame is in general not half the relative angular velocity: $R_{AB}$ shares its axis with $R_{AC}$, but when the axis $\mathbf n$ changes with time, halving the angle does not halve the angular velocity (the derivative of the rotation vector $\theta\,\mathbf n$ and the angular velocity are related by a map that depends on that vector, and the map differs at $\theta\,\mathbf n$ and at $\tfrac12\theta\,\mathbf n$). Only when the relative axis stays fixed — which includes every planar relative motion — does $\boldsymbol\omega_{AB,A}=\tfrac12\boldsymbol\omega_{rel,A}$ hold, and then $\mathbf u_{m,B}=\dot{\mathbf d}_B$. Under general three-dimensional rotation $\mathbf u_{m,B}$ is the midpoint relative material velocity and not the derivative of a displacement; this distinction sets the conditions of the stored-energy relation in Section 4.

### 2.5 Translational law

The translational law is a diagonal spring-damper law in B, and it gives the force acting on end C:

$$
\mathbf f_{C,B}=-\left(\mathbf k_t\circ\mathbf d_B+\mathbf c_t\circ\mathbf u_{m,B}\right),
\qquad
\mathbf f_{C,I}=R_{IB}\,\mathbf f_{C,B}=R_{IA}R_{AB}\,\mathbf f_{C,B}.
$$

$\mathbf d_B$ points from end A to end C, so for positive $\mathbf k_t$ the term $-\mathbf k_t\circ\mathbf d_B$ pulls end C back toward end A; the force $-\mathbf f_{C,B}$ on end A pulls A toward C, in the same sense as the reference-end force of the [three-axis translational spring-damper element](TRANSLATIONAL_SPRING_DAMPER.en.md). The three stiffnesses and three dampings act along the three axes of B; because B is symmetric with respect to the two ends, these constants belong to neither end's frame.

### 2.6 Extracting the space-XYZ roll, pitch and yaw angles

The rotational deformation measure is the space-XYZ roll-pitch-yaw triple $\boldsymbol\eta=(\eta_1,\eta_2,\eta_3)^{\mathsf T}$ of C in A: a rotation through $\eta_1$ about the first axis of A, then through $\eta_2$ about the second axis of A, then through $\eta_3$ about the third axis of A. All three rotations are about space-fixed axes, which is equivalent to successive body-fixed rotations in the order Z, Y, X, the same Z-Y-X composition as Section 4.3 of the conventions:

$$
R_{AC}=R_z(\eta_3)\,R_y(\eta_2)\,R_x(\eta_1)
=\begin{bmatrix}
\mathrm c_3\mathrm c_2 & \mathrm c_3\mathrm s_2\mathrm s_1-\mathrm s_3\mathrm c_1 & \mathrm c_3\mathrm s_2\mathrm c_1+\mathrm s_3\mathrm s_1\\
\mathrm s_3\mathrm c_2 & \mathrm s_3\mathrm s_2\mathrm s_1+\mathrm c_3\mathrm c_1 & \mathrm s_3\mathrm s_2\mathrm c_1-\mathrm c_3\mathrm s_1\\
-\mathrm s_2 & \mathrm c_2\mathrm s_1 & \mathrm c_2\mathrm c_1
\end{bmatrix},
\qquad
\mathrm c_i=\cos\eta_i,\ \mathrm s_i=\sin\eta_i.
$$

One and the same $R_{AC}$ corresponds to two families of angles: if $(\eta_1,\eta_2,\eta_3)$ generates it, so does $(\eta_1+\pi,\ \pi-\eta_2,\ \eta_3+\pi)$, and each angle may further be shifted by multiples of $2\pi$. The extraction must fix a branch.

The pitch is determined by the entry $R_{31}$ together with the four other entries in its column and its row:

$$
\eta_2=\operatorname{atan2}\!\left(-R_{31},\ \sqrt{\tfrac12\left(R_{11}^2+R_{21}^2+R_{32}^2+R_{33}^2\right)}\right),
\qquad
-R_{31}=\mathrm s_2,
\qquad
\sqrt{\tfrac12\left(R_{11}^2+R_{21}^2+R_{32}^2+R_{33}^2\right)}=\lvert \mathrm c_2\rvert.
$$

The second argument is non-negative, so $\eta_2\in[-\tfrac\pi2,\tfrac\pi2]$: this selects the family of angles with $\cos\eta_2\ge0$.

Roll and yaw are not read as $\operatorname{atan2}$ of matrix entries directly but through half-sum and half-difference angles of the quaternion. With $q_{AC}=(q_0,\mathbf q)$ the quaternion of $R_{AC}$ with non-negative scalar part, expanding $q_{AC}=q_z(\eta_3)\,q_y(\eta_2)\,q_x(\eta_1)$ shows that the sums and differences of its components satisfy

$$
q_1+q_3=\left(\cos\tfrac{\eta_2}{2}-\sin\tfrac{\eta_2}{2}\right)\sin\sigma_+,
\qquad
q_0-q_2=\left(\cos\tfrac{\eta_2}{2}-\sin\tfrac{\eta_2}{2}\right)\cos\sigma_+,
\qquad
\sigma_+=\tfrac{\eta_3+\eta_1}{2},
$$

$$
q_3-q_1=\left(\cos\tfrac{\eta_2}{2}+\sin\tfrac{\eta_2}{2}\right)\sin\sigma_-,
\qquad
q_0+q_2=\left(\cos\tfrac{\eta_2}{2}+\sin\tfrac{\eta_2}{2}\right)\cos\sigma_-,
\qquad
\sigma_-=\tfrac{\eta_3-\eta_1}{2},
$$

where, if the quaternion composed from the angles differs by a sign from the representative with non-negative scalar part, the right-hand sides of both equalities in a row are multiplied by $-1$. For $\lvert\eta_2\rvert<\tfrac\pi2$ both prefactors are positive, hence

$$
\sigma_+=\operatorname{atan2}\left(q_1+q_3,\ q_0-q_2\right),
\qquad
\sigma_-=\operatorname{atan2}\left(q_3-q_1,\ q_0+q_2\right),
$$

$$
\eta_1=\operatorname{wrap}\left(\sigma_+-\sigma_-\right),
\qquad
\eta_3=\operatorname{wrap}\left(\sigma_++\sigma_-\right),
$$

where $\operatorname{wrap}$ shifts an argument strictly greater than $\pi$ or strictly less than $-\pi$ by one multiple of $2\pi$, back into $[-\pi,\pi]$. Each $\operatorname{atan2}$ lies in $[-\pi,\pi]$, their sum and difference lie in $[-2\pi,2\pi]$, and after wrapping $\eta_1$ and $\eta_3$ take their representatives in $[-\pi,\pi]$. Three mathematical choices fix the branch: the quaternion with non-negative scalar part puts $\sigma_\pm$ on their principal values; flipping the sign of the quaternion shifts $\sigma_+$ and $\sigma_-$ by $\pi$ simultaneously, so their sum and difference shift by $0$ or $\pm2\pi$ only, and the wrapped $\eta_1$, $\eta_3$ are independent of the quaternion sign and determined by $R_{AC}$ alone; the pitch selects the family with $\cos\eta_2\ge0$. Together, $\boldsymbol\eta$ is a triple with $\eta_2\in[-\tfrac\pi2,\tfrac\pi2]$ and $\eta_1,\eta_3\in[-\pi,\pi]$ that generates $R_{AC}$; it is unique when $\lvert\eta_1\rvert,\lvert\eta_3\rvert<\pi$ and $\cos\eta_2\ne0$, while at the endpoints $\pm\pi$ the two representatives generate the same $R_{AC}$ (for $\cos\eta_2=0$ see Section 2.7). Within that domain the half-sum extraction returns the same angles as $\operatorname{atan2}(R_{32},R_{33})$ and $\operatorname{atan2}(R_{21},R_{11})$ taken directly.

### 2.7 The angular-rate map

The time derivative of $\boldsymbol\eta$ is not a component vector of $\boldsymbol\omega_{rel,A}$. Differentiating $R_{AC}=R_z(\eta_3)R_y(\eta_2)R_x(\eta_1)$,

$$
\dot R_{AC}=\dot R_z R_y R_x+R_z\dot R_y R_x+R_z R_y\dot R_x,
$$

multiplying on the right by $R_{AC}^{\mathsf T}=R_x^{\mathsf T}R_y^{\mathsf T}R_z^{\mathsf T}$, using the single-axis relations $\dot R_z R_z^{\mathsf T}=\dot\eta_3\operatorname{skew}(\mathbf e_3)$, $\dot R_yR_y^{\mathsf T}=\dot\eta_2\operatorname{skew}(\mathbf e_2)$, $\dot R_xR_x^{\mathsf T}=\dot\eta_1\operatorname{skew}(\mathbf e_1)$ together with the identity $R\operatorname{skew}(\mathbf w)R^{\mathsf T}=\operatorname{skew}(R\mathbf w)$, and reading $\dot R_{AC}R_{AC}^{\mathsf T}=\operatorname{skew}(\boldsymbol\omega_{rel,A})$ from Section 2.4 of the shared chapter gives

$$
\boldsymbol\omega_{rel,A}
=\dot\eta_1\,R_z(\eta_3)R_y(\eta_2)\mathbf e_1+\dot\eta_2\,R_z(\eta_3)\mathbf e_2+\dot\eta_3\,\mathbf e_3
=E(\boldsymbol\eta)\,\dot{\boldsymbol\eta},
\qquad
E=\begin{bmatrix}
\mathrm c_3\mathrm c_2 & -\mathrm s_3 & 0\\
\mathrm s_3\mathrm c_2 & \mathrm c_3 & 0\\
-\mathrm s_2 & 0 & 1
\end{bmatrix},
\qquad
\det E=\cos\eta_2.
$$

The three columns of $E$ are the directions in A of the three instantaneous rotation axes: the roll axis is the first axis of A carried by the two later rotations, the pitch axis is the second axis of A carried by the last rotation, and the yaw axis is the third axis of A itself. Since $\det E=\cos\eta_2$, $E$ is invertible whenever $\cos\eta_2\ne0$, and its inverse is the angular-rate map:

$$
\dot{\boldsymbol\eta}=H(\boldsymbol\eta)\,\boldsymbol\omega_{rel,A},
\qquad
H=E^{-1}=\begin{bmatrix}
\frac{\mathrm c_3}{\mathrm c_2} & \frac{\mathrm s_3}{\mathrm c_2} & 0\\
-\mathrm s_3 & \mathrm c_3 & 0\\
\frac{\mathrm c_3\mathrm s_2}{\mathrm c_2} & \frac{\mathrm s_3\mathrm s_2}{\mathrm c_2} & 1
\end{bmatrix}.
$$

Direct multiplication confirms $HE=I$. Neither $H$ nor $E$ contains $\eta_1$: the angular velocity is expressed in A, which is precisely the space-fixed frame of the first rotation, so the first angle does not move the two later axes within A. $\cos\eta_2=0$ is the singularity of these coordinates: there $R_{31}=\mp1$, the first axis of C is parallel to the third axis of A, the roll and yaw axes coincide, $E$ loses rank and the first and third rows of $H$ are unbounded. The singularity belongs to the space-XYZ coordinates themselves and has nothing to do with $\boldsymbol\omega_{rel,A}$; the invertibility domain of the angular coordinates is $\lvert\eta_2\rvert<\tfrac\pi2$, a set independent of the domain $\theta<\pi$ of the principal half-angle branch of Section 2.2.

### 2.8 Rotational law and physical moment

The rotational law is stated in the angular coordinates:

$$
\boldsymbol\tau_\eta=-\left(\mathbf k_r\circ\boldsymbol\eta+\mathbf c_r\circ\dot{\boldsymbol\eta}\right),
\qquad
\boldsymbol\tau_{C,A}=H^{\mathsf T}\boldsymbol\tau_\eta,
\qquad
\boldsymbol\tau_{C,I}=R_{IA}\,\boldsymbol\tau_{C,A}.
$$

$\boldsymbol\tau_\eta$ is the generalized moment conjugate to $\dot{\boldsymbol\eta}$, not a moment vector in space; what can be applied to a rigid body is the physical moment $\boldsymbol\tau_{C,A}$ conjugate to $\boldsymbol\omega_{rel,A}$. The two are tied by equality of power: requiring, for every $\boldsymbol\omega_{rel,A}$,

$$
\boldsymbol\tau_{C,A}\cdot\boldsymbol\omega_{rel,A}
=\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}
=\boldsymbol\tau_\eta\cdot H\boldsymbol\omega_{rel,A}
=\left(H^{\mathsf T}\boldsymbol\tau_\eta\right)\cdot\boldsymbol\omega_{rel,A},
$$

gives $\boldsymbol\tau_{C,A}=H^{\mathsf T}\boldsymbol\tau_\eta$. Equivalently $\boldsymbol\tau_\eta=E^{\mathsf T}\boldsymbol\tau_{C,A}$: each component of the generalized moment is the projection of the physical moment onto the corresponding instantaneous axis. Written as a combination of column vectors,

$$
\boldsymbol\tau_{C,A}
=\frac{\tau_{\eta,1}}{\mathrm c_2}\begin{bmatrix}\mathrm c_3\\ \mathrm s_3\\ 0\end{bmatrix}
+\tau_{\eta,2}\begin{bmatrix}-\mathrm s_3\\ \mathrm c_3\\ 0\end{bmatrix}
+\frac{\tau_{\eta,3}}{\mathrm c_2}\begin{bmatrix}\mathrm c_3\mathrm s_2\\ \mathrm s_3\mathrm s_2\\ \mathrm c_2\end{bmatrix},
\qquad
\tau_{\eta,i}=\boldsymbol\tau_{C,A}\cdot\left(E\mathbf e_i\right).
$$

The three columns of $H^{\mathsf T}$ are the dual basis of the three columns of $E$: the physical moment is decomposed along those dual directions, not along the three rotation axes. The roll and yaw generalized moments are both amplified by $1/\cos\eta_2$ on their way to the physical moment, which is how the singularity shows on the load side. The rotational moment is expressed in A and taken to I directly; B plays no part in it.

### 2.9 The midpoint wrench pair and the power principle

The family uses the third application scheme of Section 3.4 of the shared chapter: the load $(\boldsymbol\tau_{C,I},\mathbf f_{C,I})$ on end C and its negative on end A are both reduced about, and applied at, the instantaneous world midpoint $\mathbf x_m$,

$$
\mathcal W_{m}^{I}[\mathcal C]=\left(\boldsymbol\tau_{C,I},\ \mathbf f_{C,I}\right),
\qquad
\mathcal W_{m}^{I}[\mathcal A]=\left(-\boldsymbol\tau_{C,I},\ -\mathbf f_{C,I}\right),
$$

$$
\mathbf r_m^{\mathcal A}=R_{I\mathcal A}^{\mathsf T}\left(\mathbf x_m-\mathbf x_{\mathcal A}\right),
\qquad
\mathbf r_m^{\mathcal C}=R_{I\mathcal C}^{\mathsf T}\left(\mathbf x_m-\mathbf x_{\mathcal C}\right).
$$

The two wrenches are reduced about the same point and are exact opposites, so their resultant force and moment vanish; the family adds no support moment of any kind. The total power this pair delivers to the two bodies follows body by body from Section 4.1 of the shared chapter, with the velocity of the application point taken as the material-point velocity at the midpoint from Section 2.4:

$$
\mathcal P
=\mathbf f_{C,I}\cdot\mathbf v^{(\mathcal C)}(\mathbf x_m)+\boldsymbol\tau_{C,I}\cdot\boldsymbol\omega_C
-\mathbf f_{C,I}\cdot\mathbf v^{(\mathcal A)}(\mathbf x_m)-\boldsymbol\tau_{C,I}\cdot\boldsymbol\omega_A
=\mathbf f_{C,I}\cdot\mathbf u_{m,I}+\boldsymbol\tau_{C,I}\cdot\boldsymbol\omega_{rel,I}.
$$

This is the endpoint power of the third row of the table in Section 4.5 of the shared chapter, stated in terms of the loads on the opposite end. To prove that the family satisfies the organizing principle it remains to show that the velocity inputs its law actually uses are exactly the two quantities of that row. Translation: the dot product is invariant under rotation, so $\mathbf f_{C,I}\cdot\mathbf u_{m,I}=\mathbf f_{C,B}\cdot\mathbf u_{m,B}$, and the damping input of Section 2.5 is precisely $\mathbf u_{m,B}$, with the elastic term $-\mathbf k_t\circ\mathbf d_B$ paired with it. Rotation: by the conjugacy of Section 2.8, $\boldsymbol\tau_{C,I}\cdot\boldsymbol\omega_{rel,I}=\boldsymbol\tau_{C,A}\cdot\boldsymbol\omega_{rel,A}=\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}$, and the rate input of the rotational law is precisely $\dot{\boldsymbol\eta}$. Hence

$$
\mathcal P
=\mathbf f_{C,B}\cdot\mathbf u_{m,B}+\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}
=-\mathbf d_B\cdot\left(\mathbf k_t\circ\mathbf u_{m,B}\right)-\mathbf u_{m,B}\cdot\left(\mathbf c_t\circ\mathbf u_{m,B}\right)
-\boldsymbol\eta\cdot\left(\mathbf k_r\circ\dot{\boldsymbol\eta}\right)-\dot{\boldsymbol\eta}\cdot\left(\mathbf c_r\circ\dot{\boldsymbol\eta}\right),
$$

and the power received by the two bodies equals, term by term, the sum of the powers of the six constitutive components. This equality holds in every configuration and for every relative motion; it does not require a stored energy to exist.

The three contracts lock each other in place here. Applying the same force at the two origins with a support moment would change the endpoint power to $\mathbf f_{C,A}\cdot\mathbf u_A$, which is not conjugate to a damping law with $\mathbf u_{m,B}$ as its input; adding a support moment to the midpoint pair would give the already balanced pair a net couple whose power $\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)$ belongs to no constitutive component; applying the generalized moment $\boldsymbol\tau_\eta$ directly as a physical moment would change the rotational power to $\boldsymbol\tau_\eta\cdot\boldsymbol\omega_{rel,A}\ne\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}$. Application at the midpoint, the absence of a support moment and the return to a physical moment through $H^{\mathsf T}$ together make the identity above hold.

## 3. Computational implementation

### 3.1 Taking the relative motion

At every evaluation, each bushing obtains from the relative-motion query of Section 5.1 of the shared chapter, with end A as reference and end C as opposite, the quantities $\mathbf d_A$, $R_{AC}$, $\mathbf u_A$ and $\boldsymbol\omega_{rel,A}$, together with the $\mathbf d_I$, $R_{IA}$, $\mathbf p_A$ and the bodies carrying the two origins that writing the loads requires. The names A and C decide only which frame is the reference of the relative motion at this step; afterwards the translational part moves into B, which is symmetric in the two ends, and the rotational part stays in A. By the contract of the multibody layer the translational component of the relative spatial velocity already contains the transport term, so the midpoint correction subtracts only $\tfrac12\boldsymbol\omega_{rel,A}\times\mathbf d_A$, and not the correction of Section 2.4 that starts from the difference of world velocities.

### 3.2 Evaluating the translational part

As in Section 2.2, the unit quaternion with non-negative scalar part is taken from $R_{AC}$, substituted into the half-angle formulas and converted to $R_{AB}$; $R_{BA}=R_{AB}^{\mathsf T}$. In sequence,

$$
\mathbf d_B=R_{BA}\,\mathbf d_A,
\qquad
\mathbf u_{m,B}=R_{BA}\left(\mathbf u_A-\tfrac12\,\boldsymbol\omega_{rel,A}\times\mathbf d_A\right),
\qquad
\mathbf f_{C,B}=-\left(\mathbf k_t\circ\mathbf d_B+\mathbf c_t\circ\mathbf u_{m,B}\right),
$$

and then $\mathbf f_{C,I}=R_{IA}R_{AB}\,\mathbf f_{C,B}$ takes the force to I. The componentwise products realize the diagonal stiffness and damping; each of the six constants acts along one axis of B only.

### 3.3 Evaluating the rotational part

From the same $R_{AC}$, $\boldsymbol\eta$ is extracted as in Section 2.6 and $H(\boldsymbol\eta)$ is formed as in Section 2.7; one call yields both, since $H$ depends only on the sines and cosines of $\eta_2$ and $\eta_3$. Then

$$
\dot{\boldsymbol\eta}=H\,\boldsymbol\omega_{rel,A},
\qquad
\boldsymbol\tau_\eta=-\left(\mathbf k_r\circ\boldsymbol\eta+\mathbf c_r\circ\dot{\boldsymbol\eta}\right),
\qquad
\boldsymbol\tau_{C,A}=H^{\mathsf T}\boldsymbol\tau_\eta,
\qquad
\boldsymbol\tau_{C,I}=R_{IA}\,\boldsymbol\tau_{C,A}.
$$

The rotational part does not use B.

### 3.4 Writing the wrenches

From $\mathbf x_m=\mathbf p_A+\tfrac12\mathbf d_I$ and the world poses of the two bodies, the body coordinates $\mathbf r_m^{\mathcal A}$ and $\mathbf r_m^{\mathcal C}$ of the midpoint on each body are formed, and two load entries expressed in I are written: $(\mathbf r_m^{\mathcal A},\ -\boldsymbol\tau_{C,I},\ -\mathbf f_{C,I})$ for $\mathcal A$ and $(\mathbf r_m^{\mathcal C},\ \boldsymbol\tau_{C,I},\ \mathbf f_{C,I})$ for $\mathcal C$. Neither application point is a connection-frame origin, and both are converted afresh as the relative motion changes; the multibody layer then moves each moment to its body origin as in Section 3.5 of the shared chapter. The family writes no state derivative.

### 3.5 Branches that change the mathematical result

The following choices change the resulting loads and belong to the definition of the family rather than to numerical detail:

- The quaternion representative with non-negative scalar part is taken, so that $R_{AB}$ is the square root of $R_{AC}$ with half its rotation angle (Section 2.2).
- The pitch is taken with $\operatorname{atan2}$ of a non-negative second argument, so that $\cos\eta_2\ge0$ (Section 2.6).
- Roll and yaw are the difference and the sum of the half-sum angles, wrapped back into $[-\pi,\pi]$ (Section 2.6).
- $H$ is undefined at $\cos\eta_2=0$, the boundary of the invertibility domain $\lvert\eta_2\rvert<\tfrac\pi2$ of the angular coordinates (Section 2.7). The continuous theoretical branch adopted by the family satisfies three open conditions at once: $\theta<\pi$ (principal half-angle branch, Section 2.2), $\lvert\eta_2\rvert<\tfrac\pi2$ (invertible angular coordinates, Section 2.7) and $\lvert\eta_1\rvert,\lvert\eta_3\rvert<\pi$ (no crossing of the wrap boundary, Section 4.4); the three are independent, and the domain is the intersection of the three open sets.

## 4. Mathematical properties and conditions of applicability

### 4.1 Small-displacement limit

Under a joint linearization about the configuration at rest with coincident origins — the displacement $\mathbf d_A$, the relative rotation angle $\theta$, the relative translational velocity $\mathbf u_A$ and the relative angular velocity $\boldsymbol\omega_{rel,A}$ all small — one has $R_{AB}\to I$, $H\to I$ and $\boldsymbol\eta\to\theta\,\mathbf n$, while $\tfrac12\boldsymbol\omega_{rel,A}\times\mathbf d_A$, a product of two small quantities, is of second order, and the family reduces to a linear six-component bushing in A:

$$
\mathbf f_{C,A}\approx-\left(\mathbf k_t\circ\mathbf d_A+\mathbf c_t\circ\mathbf u_A\right),
\qquad
\boldsymbol\tau_{C,A}\approx-\left(\mathbf k_r\circ\boldsymbol\eta+\mathbf c_r\circ\boldsymbol\omega_{rel,A}\right),
\qquad
\boldsymbol\eta\approx\theta\,\mathbf n.
$$

The half-angle frame and the angular-rate map are corrections that appear only under finite rotation; the midpoint-velocity correction is not — it is present already at $R_{AC}=I$, is of first order in the displacement at finite relative angular velocity, and is of higher order only within the joint linearization above.

### 4.2 Domain and symmetry of the half-angle frame

$R_{AB}$ is a continuous function of $R_{AC}$ for $\theta<\pi$; at $\theta=\pi$ neither square root of $R_{AC}$ is preferred. Exchanging the names of the two ends turns the relative attitude into $R_{AC}^{\mathsf T}$, whose quaternion with non-negative scalar part is $(q_0,-\mathbf q)$ and whose half-angle rotation is $R_{AB}^{\mathsf T}=R_{CB}$: it defines the same frame B, and $\mathbf d_B$ and $\mathbf u_{m,B}$ change sign, so $\mathbf f_{C,B}$ changes sign and now acts on the former end A. The translational part is therefore independent of how the ends are named. The rotational part is not: the space-XYZ angles of $R_{AC}^{\mathsf T}$ are in general not $-\boldsymbol\eta$, only to first order, and $\boldsymbol\tau_{C,A}$ is expressed in A; under the joint small perturbation of Section 4.1, exchanging the ends yields rotational loads that differ from second order on, while at finite relative angular velocity the difference of the damping part can generally contain a first-order term in the attitude perturbation.

### 4.3 The midpoint velocity is not the derivative of a displacement

By Section 2.4, $\mathbf u_{m,B}-\dot{\mathbf d}_B=R_{BA}[(\boldsymbol\omega_{AB,A}-\tfrac12\boldsymbol\omega_{rel,A})\times\mathbf d_A]$, which vanishes when the relative axis is fixed and not under general three-dimensional rotation. $\mathbf u_{m,A}$ differs from $\mathbf u_A=\dot{\mathbf d}_A$ by $\tfrac12\boldsymbol\omega_{rel,A}\times\mathbf d_A$; the two agree when the origins coincide or when the relative angular velocity is parallel to the line of centers.

### 4.4 Singularity and wrap boundary of the angular coordinates

At $\cos\eta_2=0$, $E$ loses rank and the operator norm of $H$ is unbounded: near the singularity some finite $\boldsymbol\omega_{rel,A}$ give arbitrarily large $\dot\eta_1$, $\dot\eta_3$ and some finite $\boldsymbol\tau_\eta$ give arbitrarily large $\boldsymbol\tau_{C,A}$, but not every input is amplified — $H\mathbf e_3=\mathbf e_3$ and $H^{\mathsf T}\mathbf e_2=(-\sin\eta_3,\cos\eta_3,0)^{\mathsf T}$ remain finite throughout; at the singularity itself the inverse map is undefined. The rotational law is defined for $\lvert\eta_2\rvert<\tfrac\pi2$, and the amplification grows without bound as the relative pitch approaches a right angle. In addition, $\eta_1$ and $\eta_3$ wrap at $\pm\pi$: crossing that boundary makes the coordinate jump by $2\pi$, and the elastic moment $-\mathbf k_r\circ\boldsymbol\eta$ jumps with it; this is the other boundary of the continuous branch described in Section 3.5.

### 4.5 The rotational stiffness is diagonal only in the angular coordinates

$\mathbf k_r$ and $\mathbf c_r$ act componentwise on $\boldsymbol\eta$ and $\dot{\boldsymbol\eta}$; the physical moment $\boldsymbol\tau_{C,A}=-H^{\mathsf T}(\mathbf k_r\circ\boldsymbol\eta)-H^{\mathsf T}\operatorname{diag}(\mathbf c_r)H\,\boldsymbol\omega_{rel,A}$ is in general not a diagonal law in A: the effective physical damping matrix $H^{\mathsf T}\operatorname{diag}(\mathbf c_r)H$ is symmetric positive semidefinite but configuration dependent, and the effective elastic moment is not a diagonal spring in any physical frame. This is exactly what separates the family from the [roll spring-damper couple](ROLL_SPRING_DAMPER_COUPLE.en.md), which takes a matrix entry and a component of $\boldsymbol\omega_{rel,A}$ as inputs directly, passes through no attitude map and has none of the singularities of this section.

### 4.6 General six-component virtual-power relation

The identity $\mathcal P=\mathbf f_{C,B}\cdot\mathbf u_{m,B}+\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}$ of Section 2.9 holds unconditionally within the domain: $(\mathbf f_{C,B},\boldsymbol\tau_\eta)$ are the generalized forces conjugate to the six rates $(\mathbf u_{m,B},\dot{\boldsymbol\eta})$. When every component of $\mathbf c_t$ and $\mathbf c_r$ is non-negative, the two damping terms $\mathbf u_{m,B}\cdot(\mathbf c_t\circ\mathbf u_{m,B})$ and $\dot{\boldsymbol\eta}\cdot(\mathbf c_r\circ\dot{\boldsymbol\eta})$ are non-negative and the damping part only absorbs power from the two bodies.

### 4.7 Stored-energy relations

Within a continuous branch of the angular coordinates (no crossing of the $\pm\pi$ wrap boundary, $\cos\eta_2\ne0$), the rotational elastic power is an exact derivative, because $\dot{\boldsymbol\eta}$ is there the derivative of $\boldsymbol\eta$:

$$
\mathcal V_r=\tfrac12\,\boldsymbol\eta\cdot\left(\mathbf k_r\circ\boldsymbol\eta\right),
\qquad
\boldsymbol\eta\cdot\left(\mathbf k_r\circ\dot{\boldsymbol\eta}\right)=\dot{\mathcal V}_r.
$$

The translational elastic power in general is not, and its deviation from the candidate stored energy is governed by the difference between $\mathbf u_{m,B}$ and $\dot{\mathbf d}_B$:

$$
\mathcal V_t=\tfrac12\,\mathbf d_B\cdot\left(\mathbf k_t\circ\mathbf d_B\right),
\qquad
\mathbf d_B\cdot\left(\mathbf k_t\circ\mathbf u_{m,B}\right)-\dot{\mathcal V}_t=\mathbf d_B\cdot\left[\mathbf k_t\circ\left(\mathbf u_{m,B}-\dot{\mathbf d}_B\right)\right].
$$

That deviation vanishes under either of two conditions. First, a fixed relative axis (including every planar relative motion), for which $\mathbf u_{m,B}=\dot{\mathbf d}_B$. Second, an isotropic translational stiffness, for which the elastic force lies along the line of centers and is orthogonal to $\tfrac12\boldsymbol\omega_{rel}\times\mathbf d$:

$$
\mathbf k_t=k_t\begin{bmatrix}1\\1\\1\end{bmatrix}
\quad\Rightarrow\quad
\mathbf d_B\cdot\left(\mathbf k_t\circ\mathbf u_{m,B}\right)=k_t\,\mathbf d_A\cdot\mathbf u_A=\frac{d}{dt}\left(\tfrac12 k_t\,\lVert\mathbf d_A\rVert^2\right),
$$

in which case $\mathcal V_t=\tfrac12k_t\lVert\mathbf d_A\rVert^2$ is independent of the choice of B. Under either condition the total power reads

$$
\mathcal P=-\frac{d}{dt}\left(\mathcal V_t+\mathcal V_r\right)-\mathbf u_{m,B}\cdot\left(\mathbf c_t\circ\mathbf u_{m,B}\right)-\dot{\boldsymbol\eta}\cdot\left(\mathbf c_r\circ\dot{\boldsymbol\eta}\right),
$$

and the element delivers to the two bodies no more than the rate at which its stored energy decreases. When neither condition holds the power identity still stands, but the work of the translational elastic part around a closed path of relative motion need not vanish.

### 4.8 Force-free configuration and state

When every translational and rotational stiffness component is positive, the resting configuration with $\mathbf f_{C,B}=\mathbf 0$ and $\boldsymbol\tau_\eta=\mathbf 0$ is unique, namely $\mathbf d_A=\mathbf 0$ and $R_{AC}=I$; the family has no nominal-force term, so the force-free configuration cannot be shifted by a constant. Every load is determined by the current $(q,v)$; there is no internal state and nothing is written to the $z$ block.

## 5. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| The family type, its two ends and the contract of its four stiffness and damping vectors | `HalfAngleMidpointRollPitchYawBushing`, in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| $\mathbf d_A$, $R_{AC}$, $\mathbf u_A$, $\boldsymbol\omega_{rel,A}$ with A as reference and C as opposite end | `CalcRelativeMotion` within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Unit quaternion $q_{AC}$ with non-negative scalar part | `CanonicalQuaternion` in the call chain of `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Half-angle rotation $R_{AB}$ | `CalcHalfAngleRotation`, called by `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Extraction of $\boldsymbol\eta$ and the angular-rate map $H(\boldsymbol\eta)$ | `CalcSpaceXyzRollPitchYawKinematics`, called by `VehicleForcePlan::CalcAppliedForces`; its result `SpaceXyzRollPitchYawKinematics::rates_from_parent_angular_velocity` is $H$, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Evaluation of $\mathbf d_B$, $\mathbf u_{m,B}$, $\mathbf f_{C,B}$, $\dot{\boldsymbol\eta}$, $\boldsymbol\tau_\eta$, $\boldsymbol\tau_{C,A}$ | The bushing branch of `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Body coordinates $\mathbf r_m^{\mathcal A}$, $\mathbf r_m^{\mathcal C}$ of the midpoint and the two midpoint wrenches | `EmitMidpointBushingWrenchPair`, called by `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| World poses of the bodies needed to convert the midpoint | `MultibodyModel::CalcPoseInWorld`, in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Load entries $\mathcal W_m^I[\mathcal A]$, $\mathcal W_m^I[\mathcal C]$ | `AppliedBodyWrench`, in [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
