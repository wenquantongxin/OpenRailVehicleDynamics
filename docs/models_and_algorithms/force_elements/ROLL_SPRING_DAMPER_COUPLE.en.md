[中文](ROLL_SPRING_DAMPER_COUPLE.md)

# Roll spring-damper couple

This chapter defines and derives the roll spring-damper couple. The element responds only to the relative roll of its two ends: it reads one dimensionless roll measure from the relative attitude $R_{AC}$ and one rate component from the relative angular velocity $\boldsymbol\omega_{rel,A}$, combines them linearly into a moment about the first axis of the reference end and applies that moment to the two rigid bodies as a pair of equal and opposite pure couples. The connection frames, the relative motion, the three wrench-application schemes and the power identities are derived in [Force-element kinematics and spatial wrenches](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md), referred to below as the shared chapter; this chapter adopts its notation and results and writes down only what belongs to this family: its definition, derivation, implementation and properties. The type and the constitutive constants of the family are `RollSpringDamperCouple` in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h). The law is evaluated in the roll-family branch of `VehicleForcePlan::CalcAppliedForces` in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc). The pure couple pair is written by `internal::EmitCoupleWrenchPair` in [`body_wrench_pair.h`](../../../libs/forces/src/body_wrench_pair.h).

## 1. Objects and notation

### 1.1 The element and its two ends

The family connects a reference-end frame A and an opposite-end frame C, fixed to the rigid bodies $\mathcal A$ and $\mathcal C$ respectively, with the meaning of Section 1.1 of the shared chapter. The element is characterized by two scalar constants, the roll stiffness $k$ and the roll damping $c$, both stated in A; its axis of action is the first axis of A. It typically represents a device such as an anti-roll bar that produces a reaction moment against relative roll and nothing else.

Of the four quantities in the relative motion of Section 5.1 of the shared chapter, this family consumes two: the relative attitude $R_{AC}$ and the relative angular velocity $\boldsymbol\omega_{rel,A}$. The relative position $\mathbf d_A$ and the relative translational velocity $\mathbf u_A$ enter neither the law nor the loads; the element is therefore insensitive to where the two origins sit on their bodies and sensitive only to the attitudes and angular velocities of the two ends.

### 1.2 Notation

| Symbol | Meaning | Source |
|---|---|---|
| A, C, $\mathcal A$, $\mathcal C$, I | Reference-end and opposite-end frames, the two bodies carrying them, the inertial frame | Shared chapter §1.1 |
| $R_{IA}$, $R_{IC}$, $R_{AC}=R_{IA}^{\mathsf T}R_{IC}$ | Attitudes of the two ends and the relative attitude; entries $R_{ij}$ are numbered from 1 | Conventions §4.1, shared chapter §2.1 |
| $\boldsymbol\omega_A$, $\boldsymbol\omega_C$ | Angular velocities of the two bodies relative to I, expressed in I | Shared chapter §1.2 |
| $\boldsymbol\omega_{rel,A}$, $\boldsymbol\omega_{rel,I}$ | Angular velocity of C relative to A, expressed in A and in I | Shared chapter §2.4 |
| $\omega_{rel,A,1}$, $\omega_{rel,A,2}$ | First and second components of $\boldsymbol\omega_{rel,A}$ | Component notation of shared chapter §4.3 |
| $\mathbf e_1$, $\mathbf e_2$, $\mathbf e_3$ | Standard basis vectors | Conventions §4.1 |
| $\mathbf a_i=R_{IA}\mathbf e_i$ | Direction in I of the $i$-th unit axis of A; a unit axis of C is written out as $R_{IC}\mathbf e_j$ | New here |
| $\sigma$ | Roll measure $(R_{AC})_{32}$, dimensionless | New here |
| $\phi$ | Angle of a pure roll of C relative to A about the first axis of A | New here |
| $\psi$, $\theta$ | Yaw and pitch angles of the Z-Y-X decomposition used for comparison with angular coordinates | New here |
| $k$, $c$ | Roll stiffness and roll damping | Conventions §5.3 |
| $m$, $\mathbf m_A=m\,\mathbf e_1$, $\mathbf m_I$ | Scalar of the pure couple on the reference end, and its vector in A and in I | Shared chapter §1.3 |
| $\mathcal W_Q^E[\mathcal A]$ | Wrench acting on body $\mathcal A$, reduced about Q, expressed in E | Shared chapter §1.3 |
| $\mathbf r_{A_o}^{\mathcal A}$, $\mathbf r_{C_o}^{\mathcal C}$ | Body coordinates of the two origins in their own body frames | Shared chapter §3.1 |
| $\mathcal P$ | Total power the element delivers to the two bodies | Shared chapter §1.2 |
| $\mathcal V$, $\mathcal D$ | Stored energy and dissipated power under pure roll | Shared chapter §1.2 |
| $\operatorname{skew}(\mathbf w)$ | Skew-symmetric matrix | Shared chapter §1.2 |

### 1.3 Symbol declarations

The $\phi$ of this chapter is the angle of a pure roll of C relative to A about the first axis of A and appears only in the pure-roll case and in the comparison with angular coordinates; it is a local reuse of the superelevation angle $\phi$ of Section 2.2 of the conventions and of the pose roll $\varphi$ of Section 2.6, none of which occur in the same document. $\psi$ and $\theta$ appear only in the angular-coordinate comparison of Sections 2.4 and 4 and are unrelated to the heading $\psi$ and the vertical irregularity angle $\theta_\epsilon$ of the conventions. $\sigma$ is a sine-like dimensionless quantity, not an angle. $\mathcal V$ and $\mathcal D$ are the stored-energy and dissipation symbols registered in the shared chapter; this chapter defines them under pure roll only. The stiffness $k$ and damping $c$ are lowercase as required by Section 5.3 of the conventions; $k$ is measured in N·m/rad, the unit of the torsional stiffness it is equivalent to in the small-angle limit, because the $\sigma$ it multiplies is dimensionless, whereas the $\omega_{rel,A,1}$ multiplied by $c$ is a true angular rate.

## 2. Model and derivation

### 2.1 Roll measure and roll rate

The roll measure is one entry of the relative attitude matrix:

$$
\sigma=(R_{AC})_{32}=\mathbf e_3^{\mathsf T}R_{AC}\,\mathbf e_2=\mathbf a_3\cdot R_{IC}\mathbf e_2 .
$$

The subscripts are numbered from 1 as in Section 4.1 of the conventions, so this is the entry in the third row and second column, which is `(2, 1)` in Eigen's zero-based indexing. The second equality follows Section 4.1 of the conventions: $R_{AC}\mathbf e_2$ is the column of components of the second axis of C in A, and $\sigma$ is its third component. The third equality uses $R_{AC}=R_{IA}^{\mathsf T}R_{IC}$ to write the same number as the dot product of two unit axes in I: $\sigma$ is the projection of the second axis of C onto the third axis of A or, equivalently, the sine of the elevation of $R_{IC}\mathbf e_2$ out of the plane spanned by the first and second axes of A. Both vectors are unit vectors, so $|\sigma|\le1$ for every relative attitude.

The roll rate is the component of the relative angular velocity along the first axis of A:

$$
\omega_{rel,A,1}=\mathbf e_1\cdot\boldsymbol\omega_{rel,A}=\mathbf a_1\cdot\left(\boldsymbol\omega_C-\boldsymbol\omega_A\right).
$$

The second equality uses $\boldsymbol\omega_{rel,A}=R_{IA}^{\mathsf T}(\boldsymbol\omega_C-\boldsymbol\omega_A)$ from Section 2.4 of the shared chapter. It is the rate at which C is turning relative to A about the first axis of A at the present instant, a geometric quantity that depends on no angular coordinates.

### 2.2 Restoring and damping moments

The moment on the reference end is the sum of a restoring term and a damping term, directed along the first axis of A:

$$
m=k\,\sigma+c\,\omega_{rel,A,1},
\qquad
\mathbf m_A=m\,\mathbf e_1 .
$$

The moment on the opposite end is $-\mathbf m_A$, following the sign convention of Section 1.3 of the shared chapter. The sign means the following: when C is rolled positively relative to A, $\sigma>0$ and the elastic term gives the reference end a positive moment about $\mathbf a_1$ that turns it toward the attitude of C and the opposite end a negative moment that turns it back toward the attitude of A — it is the restoring term; the damping term opposes the relative angular velocity $\omega_{rel,A,1}$, not the relative angle — with $\sigma>0$ and $\omega_{rel,A,1}<0$ it resists the motion back toward zero — and is the dissipative term. The sign of the total moment is set by the two together. $k\ge0$ and $c\ge0$ form the domain of the law, see Section 4.

### 2.3 Pure roll and the small-angle limit

Let C be rotated relative to A only about the first axis of A, through the angle $\phi$:

$$
R_{AC}=R_x(\phi)=
\begin{bmatrix}
1&0&0\\
0&\cos\phi&-\sin\phi\\
0&\sin\phi&\cos\phi
\end{bmatrix},
\qquad
\sigma=(R_{AC})_{32}=\sin\phi .
$$

Differentiating in time, $\dot R_x(\phi)=\dot\phi\,\operatorname{skew}(\mathbf e_1)\,R_x(\phi)$, and comparison with $\dot R_{AC}=\operatorname{skew}(\boldsymbol\omega_{rel,A})R_{AC}$ of Section 2.4 of the shared chapter gives

$$
\boldsymbol\omega_{rel,A}=\dot\phi\,\mathbf e_1,
\qquad
\omega_{rel,A,1}=\dot\phi .
$$

The moment under pure roll is therefore

$$
m=k\sin\phi+c\,\dot\phi .
$$

In the small-angle limit, $\sin\phi=\phi-\tfrac16\phi^3+O(\phi^5)$, so

$$
m=k\,\phi+c\,\dot\phi+O(\phi^3),
$$

and to first order the element is a linear torsional spring in parallel with a linear torsional damper about the first axis of A. The tangent stiffness of the restoring term with respect to $\phi$ is $k\cos\phi$; it equals $k$ at $\phi=0$ and decreases as $|\phi|$ grows.

### 2.4 Geometric meaning of the two inputs under finite coupled rotation

Under pure roll the two inputs are $(\sin\phi,\dot\phi)$, the sine and the derivative of one angle; a general $R_{AC}$ contains rotation other than roll, and then even that relation is lost and their meanings have to be stated separately.

$\sigma=\mathbf a_3\cdot R_{IC}\mathbf e_2$ depends on the relative attitude only through the second axis of C and the third axis of A: no rotation of C about its own second axis and no rotation of A about its own third axis changes $\sigma$. Its time derivative follows from $\dot R_{AC}$ of Section 2.4 of the shared chapter,

$$
\dot\sigma=\mathbf e_3^{\mathsf T}\operatorname{skew}(\boldsymbol\omega_{rel,A})\,R_{AC}\,\mathbf e_2
=\boldsymbol\omega_{rel,A}\cdot\left(R_{AC}\mathbf e_2\times\mathbf e_3\right)
=(R_{AC})_{22}\,\omega_{rel,A,1}-(R_{AC})_{12}\,\omega_{rel,A,2}.
$$

In general it differs from $\omega_{rel,A,1}$: under pure roll the two differ by the factor $\cos\phi$, and a second component of the relative angular velocity adds a further term. Conversely, $\omega_{rel,A,1}$ is the component of an angular velocity along an axis that moves with $\mathcal A$; it is the time derivative of no configuration coordinate. The pair $(\sigma,\omega_{rel,A,1})$ is thus a pair of model coordinates of this family — one configuration-level scalar and one velocity-level scalar — and not some coordinate $q$ together with its $\dot q$; only on the pure-roll submanifold does it reduce to $(\sin\phi,\dot\phi)$.

To see this concretely, any decomposition into angular coordinates can serve as a comparison. With the Z-Y-X composition of Section 4.3 of the conventions, write $R_{AC}=R_z(\psi)R_y(\theta)R_x(\phi)$; then

$$
\sigma=\sin\phi\cos\theta,
\qquad
\omega_{rel,A,1}=\dot\phi\cos\theta\cos\psi-\dot\theta\sin\psi .
$$

Neither quantity is the roll angle $\phi$ of this decomposition or its derivative $\dot\phi$: for $\theta=\psi=0$ they reduce to $(\sin\phi,\dot\phi)$, the first still being a sine rather than the angle itself and replaceable by $\phi$ only to first order in small angles; a different composition order gives different expressions. The model uses no such set of angles; they occur only in this comparison.

This also explains why the moment passes through no attitude map. If $m$ were read as a generalized force conjugate to some angular coordinate, the physical moment would have to be obtained through the transpose of that coordinate's rate map and would in general have components along the second and third axes of A; the rotational part of the [half-angle midpoint RPY bushing](HALF_ANGLE_MIDPOINT_RPY_BUSHING.en.md) works that way. This family applies no such map: $m\,\mathbf e_1$ is itself a couple about $\mathbf a_1$ in physical space, and $\sigma$ and $\omega_{rel,A,1}$ are its two inputs rather than the value and derivative of an angle.

### 2.5 Rotation into the inertial frame and the pure couple pair

Loads are written in I. Rotating the couple from A into I,

$$
\mathbf m_I=R_{IA}\,\mathbf m_A=m\,\mathbf a_1 .
$$

The couple vector lies along the first axis of A, an axis that moves with body $\mathcal A$ and not with $\mathcal C$. The two ends receive the pure couple pair of Section 3.3 of the shared chapter:

$$
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf m_I,\ \mathbf 0\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(-\mathbf m_I,\ \mathbf 0\right).
$$

Neither load carries a force; a pure couple is independent of the reduction point, and the resultant force and moment of the pair vanish. The support moment of the translational families has no counterpart here, since without a force there is no lever arm. Expressed in C, the couple on the opposite end is $-m\,R_{AC}^{\mathsf T}\mathbf e_1$, that is, $m$ times the negated first row of $R_{AC}$, which in general has components along all three axes of C; only under pure roll, where $R_{IC}\mathbf e_1=\mathbf a_1$, does it lie along the first axis of C.

### 2.6 The power principle

This family uses the second row of the table in Section 4.5 of the shared chapter: a pure couple pair paired with the relative angular velocity. By Section 4.1 of the shared chapter, a wrench without force delivers to a body the dot product of its couple with that body's angular velocity, so the total over the two bodies is

$$
\mathcal P=\mathbf m_I\cdot\boldsymbol\omega_A-\mathbf m_I\cdot\boldsymbol\omega_C
=-m\,\mathbf a_1\cdot\boldsymbol\omega_{rel,I}
=-m\,\mathbf e_1\cdot\boldsymbol\omega_{rel,A}
=-m\,\omega_{rel,A,1}.
$$

The second equality uses $\mathbf m_I=m\,\mathbf a_1$ and $\boldsymbol\omega_{rel,I}=\boldsymbol\omega_C-\boldsymbol\omega_A$; the third uses $\mathbf a_1\cdot\boldsymbol\omega_{rel,I}=\mathbf e_1^{\mathsf T}R_{IA}^{\mathsf T}\boldsymbol\omega_{rel,I}=\mathbf e_1\cdot\boldsymbol\omega_{rel,A}$. The law takes $\omega_{rel,A,1}$ as its rate input and $m$ as its output, so the power it accounts for is exactly $-m\,\omega_{rel,A,1}$: the endpoint power equals the constitutive power, and the family satisfies the organizing principle. The equality does not depend on where the two origins are, because the power of a pure couple involves no application point.

Substituting the law,

$$
\mathcal P=-k\,\sigma\,\omega_{rel,A,1}-c\,\omega_{rel,A,1}^{2}.
$$

The second term is never positive for any motion; it is the damping dissipation. Under pure roll the first term is the exact time derivative of a stored energy:

$$
-k\sin\phi\,\dot\phi=-\frac{d}{dt}\mathcal V,
\qquad
\mathcal V=k\left(1-\cos\phi\right),
\qquad
\mathcal D=c\,\dot\phi^{2},
$$

so that $\mathcal P=-\dot{\mathcal V}-\mathcal D$. That the damping input must be $\omega_{rel,A,1}$ and not $\dot\sigma$ is precisely what the power principle demands: the endpoint power of a couple pair is always $-m\,\omega_{rel,A,1}$, so if the damping term were rewritten as $c\,\dot\sigma$, its endpoint power $-c\,\dot\sigma\,\omega_{rel,A,1}$ would, by the $\dot\sigma$ of Section 2.4, have no definite sign in general, and under coupled rotation the element could do positive work on the system.

## 3. Computational implementation

### 3.1 Reading the relative motion

At every evaluation the element first obtains one relative-motion record as in Section 5.1 of the shared chapter. This family reads $R_{AC}$ and $\boldsymbol\omega_{rel,A}$ from it, together with the $R_{IA}$ and the body-fixed points of the two origins that writing the loads requires; $\mathbf d_A$, $\mathbf u_A$ and $\mathbf d_I$ are not read. $\sigma$ is the entry of $R_{AC}$ at zero-based index `(2, 1)`, and $\omega_{rel,A,1}$ is the first component of $\boldsymbol\omega_{rel,A}$. Both are algebraic in the current $(q,v)$; the element carries no internal state and occupies no part of the $z$ block.

### 3.2 Evaluating the law and rotating into the inertial frame

The moment is evaluated as $m=k\,\sigma+c\,\omega_{rel,A,1}$ and taken to I as $\mathbf m_I=R_{IA}\,(m,0,0)^{\mathsf T}$. This step has no branches: $m$ is one and the same polynomial in the entries of $R_{AC}$ and the components of $\boldsymbol\omega_{rel,A}$, evaluated by the same expression at every relative attitude, including those where $\sigma$ approaches $\pm1$. The saturation of the measure described in Section 4 is a property of $\sigma$ as a matrix entry; the implementation contains no clipping and no change of branch.

### 3.3 Writing the pure couple pair

With the reference-end origin as the positive end and the opposite-end origin as the negative end, two loads expressed in I are written:

$$
\left(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf m_I,\ \mathbf 0\right),
\qquad
\left(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ -\mathbf m_I,\ \mathbf 0\right).
$$

The multibody layer moves each load to the origin of its body as in Section 3.5 of the shared chapter; the force is zero, the lever-arm term vanishes and the moment enters the resultant moment of each body unchanged. The two origins recorded in the load entries are therefore formal application points only: replacing either by any other fixed point of the same body leaves the result unchanged.

## 4. Mathematical properties and conditions of applicability

**Stored energy and dissipation.** Under pure roll, $\mathcal V=k(1-\cos\phi)\ge0$ and $\mathcal D=c\,\dot\phi^2\ge0$, so the element is passive whenever $k\ge0$ and $c\ge0$. With $k=0$ it is a pure roll damper, with $c=0$ a pure roll spring, and with both zero it has no effect on the motion.

**Saturation of the measure and the bound on the restoring moment.** $|\sigma|\le1$ holds for every relative attitude, so the magnitude of the elastic moment never exceeds $k$, whatever the relative rotation. Under pure roll the tangent stiffness $k\cos\phi$ decreases monotonically to zero as $|\phi|$ grows from 0 to $\pi/2$ and is negative for $|\phi|>\pi/2$: the element is a softening spring, and beyond $\pm\pi/2$ the restoring moment decreases as the angle grows. $\mathcal V$ attains its maximum at $\phi=\pi$, where the elastic moment vanishes and the tangent stiffness is $-k$; this is an unstable equilibrium of the elastic part. The map $\phi\mapsto\sin\phi$ is not injective on $(-\pi,\pi]$: $\phi$ and $\pi-\phi$ give the same $\sigma$, and the elastic part cannot distinguish the two configurations. The damping term, whose input is $\omega_{rel,A,1}$, is unaffected by the saturation.

**Range of applicability.** As a restoring element the family has positive tangent stiffness only for $|\phi|<\pi/2$; as a stand-in for a linear torsional spring it holds only for $|\phi|\ll1$, the relative deviation of the restoring moment from the linear law being $1-\sin\phi/\phi=\tfrac16\phi^2+O(\phi^4)$. Situations that require a linear or hardening restoring characteristic at large relative rotation are outside this family.

**The elastic part has no potential on the whole rotation group.** Suppose a function $\mathcal V(R_{AC})$ existed with $-k\,\sigma\,\omega_{rel,A,1}=-\dot{\mathcal V}$ for every $\boldsymbol\omega_{rel,A}$. Its directional derivatives along infinitesimal rotations about the second and third axes of A would vanish; the commutator of infinitesimal rotations about the second and third axes is an infinitesimal rotation about the first, so the directional derivative along the first axis would vanish as well, $\mathcal V$ would be constant, and $k\ne0$ would be contradicted. The argument is local: the elastic moment field is non-integrable on every open neighborhood in which $\sigma\ne0$, and along a general closed path in relative attitude the elastic part can do nonzero net work. The $\mathcal V=k(1-\cos\phi)$ of Section 2.6 is the stored energy restricted to the pure-roll curve; it does not extend to a potential on the whole rotation group. Written out with the Z-Y-X comparison of Section 2.4, the deviation is

$$
-k\,\sigma\,\omega_{rel,A,1}+\frac{d}{dt}\mathcal V
=-k\sin\phi\left[\dot\phi\left(\cos^{2}\theta\cos\psi-1\right)-\dot\theta\cos\theta\sin\psi\right],
$$

whose two bracketed terms vanish identically when $\theta=\psi=0$; for small $\theta$ and $\psi$ the first is of order $(\theta^2+\psi^2)\,\dot\phi$ and the second of order $\psi\,\dot\theta$. When the relative rotation is dominated by roll about $\mathbf a_1$, the deviation of the elastic part from $\mathcal V$ disappears together with the other two angles.

**No angular-coordinate singularity.** $\sigma$ is one entry of $R_{AC}$ and $\omega_{rel,A,1}$ one component of an angular velocity; both are defined and smooth at every relative attitude. The family has none of the singularities or branches that extracting angular coordinates introduces, an instance of the property stated in Section 6 of the shared chapter that a family whose rate input is a component of $\boldsymbol\omega_{rel,A}$ has no such singularity.

**Insensitivity to relative translation.** $\mathbf d_A$ and $\mathbf u_A$ do not enter the law; moving either origin on its body while keeping the attitudes fixed leaves the element and its loads unchanged.

**The choice of reference end is part of the law.** Swapping the ends turns the measure into $(R_{CA})_{32}=(R_{AC})_{23}=\mathbf a_2\cdot R_{IC}\mathbf e_3$ and the couple axis into $R_{IC}\mathbf e_1$. Under pure roll, $(R_{AC})_{23}=-\sin\phi$ and $R_{IC}\mathbf e_1=\mathbf a_1$, so the swapped element produces the same pair of loads; with rotation other than roll present the two differ, and the two elements are not equivalent.

## 5. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| Family type, the two ends and the constants $k$, $c$ | `RollSpringDamperCouple`, in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| One-shot evaluation of the relative motion $R_{AC}$, $\boldsymbol\omega_{rel,A}$, $R_{IA}$ | `CalcRelativeMotion` within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Contract of the relative angular velocity $\boldsymbol\omega_{rel,A}$ | `MultibodyModel::CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame`, in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Evaluation of $\sigma$, $\omega_{rel,A,1}$ and $m$, and formation of $\mathbf m_I$ | The roll-family branch of `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Writing the pure couple pair | `internal::EmitCoupleWrenchPair`, in [`body_wrench_pair.h`](../../../libs/forces/src/body_wrench_pair.h) |
| Load entry and body-fixed point | `AppliedBodyWrench`, `BodyFixedPoint`, in [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
| Reduction of loads to the body origin and entry into forward dynamics | `MultibodyModel::CalcStateTimeDerivatives`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
