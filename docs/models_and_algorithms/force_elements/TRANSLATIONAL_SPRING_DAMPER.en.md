[中文](TRANSLATIONAL_SPRING_DAMPER.md)

# Three-axis translational spring-damper element

This chapter states the constitutive law, paired wrench, power identity, stored energy and dissipation of the three-axis translational spring-damper element, and discusses the attitude dependence that its diagonal law acquires when expressed in space. The family places one parallel pair of a linear spring and a viscous damper on each of the three axes of the reference-end frame, adds a nominal force given per instance, writes the force on the reference end as an affine function of the relative position and relative velocity and applies it to the two bodies as an endpoint force pair with a reference-end support moment. The relative motion of the two ends, the three application schemes and the power identities are derived in [Force-element kinematics and spatial wrenches](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md) (the shared chapter below); this chapter cites them and derives only what belongs to the family. The type is defined by `TranslationalSpringDamper` in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h); the law is evaluated by `VehicleForcePlan::CalcAppliedForces` in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc), and the paired wrench is written by `EmitTranslationalWrenchPair` in the same file.

## 1. Objects and notation

### 1.1 Element, ends and reference end

The element connects the reference-end frame A and the opposite-end frame C, fixed to two distinct rigid bodies $\mathcal A$ and $\mathcal C$, as in Section 1.1 of the shared chapter. All of its constitutive constants — the three-axis stiffness $\mathbf k$, the three-axis damping $\mathbf c$ and the nominal force $\mathbf f_0$ — are stated in A, and the resulting force is the force on the reference end; the opposite end receives its negative. The deformation measure is the relative position $\mathbf d_A$ of Section 2.1 of the shared chapter, and the velocity input is the relative velocity with its transport term, $\mathbf u_A=\dot{\mathbf d}_A$, of Section 2.3 there. The family reads neither the relative attitude $R_{AC}$ nor the relative angular velocity $\boldsymbol\omega_{rel,A}$, and it carries no internal state.

### 1.2 Notation

| Symbol | Meaning | Source |
|---|---|---|
| A, C, $\mathcal A$, $\mathcal C$, I | Reference-end and opposite-end frames, the two bodies carrying them, the inertial frame | Shared chapter, Section 1.1 |
| $\mathbf p_A$, $\mathbf p_C$ | Positions of the origins $A_o$ and $C_o$ in I | Shared chapter, Section 1.2 |
| $\mathbf d_A$, $\mathbf d_I$ | Relative position $\mathbf p_C-\mathbf p_A$, expressed in A and in I | Shared chapter, Section 2.1 |
| $\mathbf u_A$, $\mathbf u_I$ | Velocity of the opposite-end origin relative to the reference end (relative to body $\mathcal A$), $\mathbf u_A=\dot{\mathbf d}_A$ | Shared chapter, Section 2.3 |
| $R_{IA}$, $R_{AC}$ | Attitude of the reference end in I, relative attitude of the opposite end in the reference end | Conventions, Section 4.1 |
| $\mathbf v_{Ao}$, $\mathbf v_{Co}$, $\boldsymbol\omega_A$, $\boldsymbol\omega_C$ | Velocities of the two origins and angular velocities of the two bodies, expressed in I | Shared chapter, Section 1.3 |
| $\mathbf f_A$, $\mathbf f_I$ | Force on the reference end, expressed in A and in I | Shared chapter, Section 1.3 |
| $\mathcal W_Q^E[\mathcal A]$ | Wrench on body $\mathcal A$, reduced about Q, expressed in E | Shared chapter, Section 1.3 |
| $\mathbf r_{A_o}^{\mathcal A}$, $\mathbf r_{C_o}^{\mathcal C}$ | Body coordinates of the two origins in their own body frames | Shared chapter, Section 3.1 |
| $\mathcal P$ | Total power the element delivers to the two bodies | Shared chapter, Section 1.2 |
| $\mathbf a\circ\mathbf b$ | Componentwise product | Shared chapter, Section 1.2 |
| $\operatorname{skew}(\mathbf w)$ | Skew-symmetric matrix | Shared chapter, Section 1.2 |
| $\mathbf e_j$ | The $j$-th standard basis vector | Conventions, Section 4.1 |
| $\mathbf k=(k_1,k_2,k_3)$, $\mathbf c=(c_1,c_2,c_3)$ | Stiffness and damping on the three axes of A | Vector form of Conventions, Section 5.3 |
| $\mathbf f_0$ | Nominal force: a constant vector given per instance, expressed in A, acting on the reference end | New in this chapter |
| $\mathbf f_A^{\mathrm{el}}$, $\mathbf f_A^{\mathrm{d}}$ | Elastic part $\mathbf k\circ\mathbf d_A+\mathbf f_0$ and damping part $\mathbf c\circ\mathbf u_A$ of $\mathbf f_A$ | New in this chapter |
| $\mathbf d_{0,A}$ | Zero-force displacement of the elastic part when every axial stiffness is positive, expressed in A | New in this chapter |
| $\mathcal V$, $\mathcal D$ | Stored-energy function and dissipation rate | Shared chapter, Section 1.2 |
| $\delta\mathbf p_A$, $\delta\boldsymbol\theta_A$, $\delta\mathbf p_C$ | Virtual displacement and virtual rotation of body $\mathcal A$ at $A_o$, virtual displacement of body $\mathcal C$ at $C_o$, expressed in I | New in this chapter |
| $\operatorname{diag}(\mathbf k)$ | Matrix with $\mathbf k$ on its diagonal, $\operatorname{diag}(\mathbf k)\mathbf x=\mathbf k\circ\mathbf x$ | New in this chapter |

### 1.3 Relation to the shared chapter and the conventions

This chapter follows every convention and symbol reservation of Section 1.3 of the shared chapter, and refers to [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md) (the conventions) in the same way: $R_{AB}$ maps components in B to components in A, the bold $\mathbf d$ is the relative position vector, $\mathbf u$ is a relative velocity, power is written $\mathcal P$, an unmarked $\mathbf f$ is the load on the reference end and the last subscript names the expressed-in frame. The symbols new to this chapter are marked in the table above; three of them call for a remark. The subscript 0 of the nominal force $\mathbf f_0$ is not an expressed-in frame, the vector being always expressed in A; the superscripts $\mathrm{el}$ and $\mathrm d$ only separate the elastic and damping parts and leave the meaning of the last subscript unchanged; and the zero-force displacement $\mathbf d_{0,A}$ uses the same qualifier-then-frame double subscript as $\mathbf u_{m,A}$ of the shared chapter. The stiffness and damping of the family are the lowercase vectors $\mathbf k$ and $\mathbf c$ required by Section 5.3 of the conventions; where a matrix form is needed it is written $\operatorname{diag}(\mathbf k)$, $\operatorname{diag}(\mathbf c)$, never uppercase $K$, $C$, which are reserved for system-level matrices.

## 2. Model and derivation

### 2.1 Deformation measure and velocity input

By Sections 2.1 and 2.3 of the shared chapter,

$$
\mathbf d_A=R_{IA}^{\mathsf T}\left(\mathbf p_C-\mathbf p_A\right),
\qquad
\mathbf u_A=R_{IA}^{\mathsf T}\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right)=\dot{\mathbf d}_A.
$$

$\mathbf d_A$ holds the components in A of the difference of the two origins and contains no natural length: $\mathbf d_A=\mathbf 0$ when the origins coincide. $\mathbf u_A$ contains the transport term $-\boldsymbol\omega_A\times\mathbf d_I$; it is the velocity of $C_o$ relative to the body $\mathcal A$, not relative to the point $A_o$, and it is exactly the time derivative of $\mathbf d_A$. Each of the three spring-damper pairs of the family reads one component of $\mathbf d_A$ and one of $\mathbf u_A$; that the deformation measure and the velocity input are derivatives of one another is the premise of the exact stored-energy derivative in Section 2.6.

### 2.2 Parallel constitutive law

Spring and damper in parallel means that both see the same deformation $\mathbf d_A$ and the same velocity $\mathbf u_A$, and that their forces add. The force on the reference end is

$$
\mathbf f_A=\mathbf k\circ\mathbf d_A+\mathbf c\circ\mathbf u_A+\mathbf f_0,
\qquad
f_{A,i}=k_i\,d_{A,i}+c_i\,u_{A,i}+f_{0,i},
\quad i=1,2,3.
$$

The three axes are uncoupled: the force on axis $i$ depends only on the displacement and velocity on axis $i$. This chapter splits the force into an elastic part and a damping part,

$$
\mathbf f_A=\mathbf f_A^{\mathrm{el}}+\mathbf f_A^{\mathrm{d}},
\qquad
\mathbf f_A^{\mathrm{el}}=\mathbf k\circ\mathbf d_A+\mathbf f_0,
\qquad
\mathbf f_A^{\mathrm{d}}=\mathbf c\circ\mathbf u_A.
$$

The law is an affine function of $(\mathbf d_A,\mathbf u_A)$, smooth everywhere, without branches and without any internal quantity to integrate; the force is determined algebraically by the current relative motion. By contrast, in the [series spring-viscous-damper element](SERIES_SPRING_VISCOUS_DAMPER.en.md) the spring and the damper carry one and the same force while their deformations add, which makes the force a state.

**Meaning of the sign.** $\mathbf f_A$ is the force on the reference end, and $f_{A,i}>0$ means by definition that the reference end is loaded along the positive $i$-th axis of A. The three terms are best read separately: the elastic term $k_i\,d_{A,i}$ (with $k_i>0$) is positive when the opposite-end origin has moved away from the reference end along the positive $i$-th axis, so the reference end is pulled along $+\mathbf e_i$ toward the opposite end while the opposite end receives $-k_i d_{A,i}\,\mathbf e_i$ and is pulled back, the element acting like a stretched spring drawing its ends together; the damping term $c_i\,u_{A,i}$ is positive when the opposite end recedes from body $\mathcal A$ along the positive $i$-th axis and opposes the separation; the nominal force $f_{0,i}$ is a constant unrelated to the motion. The sign of their sum is set by all three together and cannot be read off the sign of $d_{A,i}$ alone. The opposite end always receives $-\mathbf f_A$, see Section 2.4.

### 2.3 Mathematical role of the nominal force

The deformation measure $\mathbf d_A$ is a difference of origins, and the kinematic layer provides no natural length or installed configuration (Section 6 of the shared chapter). If the law contained only $\mathbf k\circ\mathbf d_A$ with all three stiffnesses positive, the force-free configuration of the element could only be the one with coincident origins; yet an element often carries a force in that configuration, with the force-free configuration of its spring part lying elsewhere. The nominal force $\mathbf f_0$ is the constant vector that provides for this: it is the value of the elastic part $\mathbf f_A^{\mathrm{el}}$ at $\mathbf d_A=\mathbf 0$, that is, the force on the reference end when the two origins coincide and are at relative rest, and it is the only term of the law that can move the force-free configuration away from coincident origins.

On an axis with positive stiffness it is equivalent to a zero-force displacement:

$$
d_{0,A,i}=-\frac{f_{0,i}}{k_i},
\qquad
k_i\,d_{A,i}+f_{0,i}=k_i\left(d_{A,i}-d_{0,A,i}\right),
\qquad k_i>0.
$$

When all three stiffnesses are positive, $\mathbf d_{0,A}=-\operatorname{diag}(\mathbf k)^{-1}\mathbf f_0$ is the unique force-free configuration of the elastic part, and $\mathbf f_0$ absorbs the zero-displacement offset that the absence of a natural length would otherwise leave unexpressed. On an axis with zero stiffness, $f_{0,i}$ is a constant force along the $i$-th axis of A with no corresponding zero-force displacement. In either case $\mathbf f_0$ is a per-instance constant: it is not a function of the state, it is not integrated and it is expressed in A and acts on the reference end, so like $\mathbf k$ and $\mathbf c$ it turns with the attitude of the reference end (Section 2.7). The type stores no separate natural length, and the law contains no zero-displacement quantity other than $\mathbf f_0$.

### 2.4 Paired wrench

The family applies its load by the first scheme of Section 3.2 of the shared chapter: equal and opposite forces at the two origins, plus a support moment on the reference end. With $\mathbf f_A$ taken to I,

$$
\mathbf f_I=R_{IA}\mathbf f_A,
\qquad
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(\mathbf 0,\ -\mathbf f_I\right).
$$

The support moment is taken about $A_o$ and applied to body $\mathcal A$; since a rotation preserves cross products, $\mathbf d_I\times\mathbf f_I=R_{IA}\left(\mathbf d_A\times\mathbf f_A\right)$, so in A it is simply $\mathbf d_A\times\mathbf f_A$. The pair has zero resultant force and zero resultant moment about any point (Section 3.2 of the shared chapter). The support moment vanishes if and only if $\mathbf f_A$ is parallel to $\mathbf d_A$; in the general case of unequal axial stiffnesses, a nonzero nominal force or a transverse component of $\mathbf u_A$, $\mathbf f_A$ does not lie along the line joining the two origins and the support moment is nonzero. It is not a balancing patch: its necessity follows from the power identity of Section 2.5, and its origin from the gradient structure of Section 2.6.

### 2.5 Verification of the power principle

Section 4.5 of the shared chapter requires every family to choose its velocity input and its application point as a pair, so that the constitutive power equals the endpoint power exactly. This family takes the first row of the table there: velocity input $\mathbf u_A$, application as a force pair at the two origins with the reference-end support moment. By the single-body power formula of Section 4.1 of the shared chapter, the two wrenches above deliver to the two bodies the total power

$$
\mathcal P
=\mathbf f_I\cdot\mathbf v_{Ao}+\left(\mathbf d_I\times\mathbf f_I\right)\cdot\boldsymbol\omega_A-\mathbf f_I\cdot\mathbf v_{Co}
=-\mathbf f_I\cdot\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right)
=-\mathbf f_A\cdot\mathbf u_A.
$$

The second equality uses the triple-product identity $\boldsymbol\omega\cdot(\mathbf d\times\mathbf f)=\mathbf f\cdot(\boldsymbol\omega\times\mathbf d)$ of Section 4.1 of the shared chapter; the third uses the definition of $\mathbf u_A$ in Section 2.1 and the rotational invariance of the dot product. The right-hand side is the negative inner product of the force the law produces with the velocity the law consumes, which is exactly the constitutive power of the family: the velocity conjugate to the wrench pair is the very velocity the law reads. The family therefore satisfies the organizing principle. Had the law used $R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ without the transport term while keeping the wrenches, or kept the law while dropping the support moment, the two powers would differ by the power of the support moment, $\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)$, see Section 4.2 of the shared chapter.

### 2.6 Stored energy, dissipation and passivity

Substituting the law into $\mathcal P=-\mathbf f_A\cdot\mathbf u_A$,

$$
\mathcal P=-\mathbf u_A\cdot\left(\mathbf k\circ\mathbf d_A\right)-\mathbf f_0\cdot\mathbf u_A-\mathbf u_A\cdot\left(\mathbf c\circ\mathbf u_A\right).
$$

Define the stored-energy function and the dissipation rate

$$
\mathcal V(\mathbf d_A)=\tfrac12\,\mathbf d_A\cdot\left(\mathbf k\circ\mathbf d_A\right)+\mathbf f_0\cdot\mathbf d_A
=\sum_{i=1}^{3}\left(\tfrac12\,k_i\,d_{A,i}^2+f_{0,i}\,d_{A,i}\right),
\qquad
\mathcal D(\mathbf u_A)=\mathbf u_A\cdot\left(\mathbf c\circ\mathbf u_A\right)=\sum_{i=1}^{3}c_i\,u_{A,i}^2.
$$

Because $\mathbf u_A=\dot{\mathbf d}_A$ and $\mathbf k$, $\mathbf f_0$ are constant, $\dot{\mathcal V}=\mathbf u_A\cdot(\mathbf k\circ\mathbf d_A)+\mathbf f_0\cdot\mathbf u_A$, hence

$$
\mathcal P=-\dot{\mathcal V}-\mathcal D,
\qquad
\int_{t_0}^{t_1}\mathcal P\,dt
=\mathcal V\!\left(\mathbf d_A(t_0)\right)-\mathcal V\!\left(\mathbf d_A(t_1)\right)-\int_{t_0}^{t_1}\mathcal D\,dt.
$$

The work the element delivers to the two bodies equals the decrease of its stored energy less the dissipation; the mechanical energy of the two bodies plus $\mathcal V$ changes at the rate $-\mathcal D$. That the power of the elastic part is the exact derivative of $\mathcal V$ rests on $\mathbf u_A$ being precisely the derivative of $\mathbf d_A$, that is, on the pairing of Section 2.5.

**Dissipation inequality and passivity.** The relation above is an energy identity and holds for any parameters. When $k_i\ge0$ and $c_i\ge0$ for $i=1,2,3$, $\mathcal D\ge0$ for every $\mathbf u_A$, so $\mathcal P\le-\dot{\mathcal V}$: the element never delivers more power to the bodies than its stored energy releases, which is the dissipation inequality with $\mathcal V$ as storage function. The law is diagonal and so are both quadratic forms, so componentwise non-negativity and positive semidefiniteness are one and the same condition. Passivity asks for one thing more than the dissipation inequality — a storage function bounded below, without which the work extractable from the element has no upper bound; on the unbounded displacement domain this requires $f_{0,i}=0$ on every axis with $k_i=0$. A negative $k_i$ makes $\mathcal V$ unbounded below along that axis, so the element could deliver unbounded work along it; a negative $c_i$ lets $\mathcal D$ take negative values, so the element would generate energy from relative motion. When all three stiffnesses are positive, completing the square gives

$$
\mathcal V(\mathbf d_A)=\tfrac12\sum_{i=1}^{3}k_i\left(d_{A,i}-d_{0,A,i}\right)^2-\tfrac12\sum_{i=1}^{3}\frac{f_{0,i}^2}{k_i},
$$

so $\mathcal V$ attains its minimum at the zero-force displacement $\mathbf d_{0,A}$, the constant affects no force and the storage function is bounded below. On an axis with $k_i=0$ and $f_{0,i}\neq0$, $\mathcal V$ is linear along that axis and unbounded below: the constant force is still a potential force and the energy identity and dissipation inequality hold as before, but the work extractable from it has no upper bound and the element is not passive on that axis. When $k_i=0\Rightarrow f_{0,i}=0$ holds, adding a constant to $\mathcal V$ gives the non-negative storage function $\tfrac12\sum_{k_i>0}k_i(d_{A,i}-d_{0,A,i})^2$ and the element is passive.

**Gradient structure of the stored energy.** The elastic part of the wrench pair is exactly the negative gradient of $\mathcal V$, which can be verified directly. Regard $\mathcal V$ as a function of the configurations of the two bodies, give body $\mathcal A$ a virtual displacement $\delta\mathbf p_A$ at $A_o$ and a virtual rotation $\delta\boldsymbol\theta_A$ and give body $\mathcal C$ a virtual displacement $\delta\mathbf p_C$ at $C_o$, all expressed in I. From $\delta R_{IA}=\operatorname{skew}(\delta\boldsymbol\theta_A)R_{IA}$,

$$
\delta\mathbf d_A=R_{IA}^{\mathsf T}\left(\delta\mathbf p_C-\delta\mathbf p_A-\delta\boldsymbol\theta_A\times\mathbf d_I\right),
$$

and writing $\mathbf f_I^{\mathrm{el}}=R_{IA}\mathbf f_A^{\mathrm{el}}$ and using the triple-product identity once more,

$$
-\delta\mathcal V=-\mathbf f_A^{\mathrm{el}}\cdot\delta\mathbf d_A
=\mathbf f_I^{\mathrm{el}}\cdot\delta\mathbf p_A+\left(\mathbf d_I\times\mathbf f_I^{\mathrm{el}}\right)\cdot\delta\boldsymbol\theta_A-\mathbf f_I^{\mathrm{el}}\cdot\delta\mathbf p_C.
$$

The coefficients of the three virtual displacements are precisely the elastic parts of the two wrenches of Section 2.4: on $\mathcal A$ at $A_o$ the force $\mathbf f_I^{\mathrm{el}}$ and the support moment $\mathbf d_I\times\mathbf f_I^{\mathrm{el}}$, on $\mathcal C$ at $C_o$ the force $-\mathbf f_I^{\mathrm{el}}$ and no moment. The support moment is the generalized force conjugate to the attitude of body $\mathcal A$; it arises because $\mathcal V$ depends on that attitude through $\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I$, and since $\mathcal V$ does not depend on the attitude of $\mathcal C$, end C carries no moment. The damping part is not a gradient, but it is applied with the same lever-arm structure and its power is $-\mathcal D$.

### 2.7 Attitude dependence of the diagonal law in space

Substituting $\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I$ and $\mathbf u_A=R_{IA}^{\mathsf T}\mathbf u_I$ into the law and taking it to I,

$$
\mathbf f_I=R_{IA}\operatorname{diag}(\mathbf k)R_{IA}^{\mathsf T}\,\mathbf d_I+R_{IA}\operatorname{diag}(\mathbf c)R_{IA}^{\mathsf T}\,\mathbf u_I+R_{IA}\mathbf f_0.
$$

The two similarity-transformed matrices are symmetric, with eigenvalues $k_i$ and $c_i$ and eigenvectors $R_{IA}\mathbf e_i$, the directions of the three axes of A seen in I. The law is diagonal with constant coefficients in A but varies with $R_{IA}$ in I: the principal axes of stiffness and damping are fixed to body $\mathcal A$ and turn with it. Three consequences follow. First, the force changes when $\mathcal A$ rotates with $\mathbf d_I$ held fixed, so the stored energy $\mathcal V$ depends on the attitude of $\mathcal A$ — the origin of the support moment in Section 2.6. Second, the nominal force turns with A as well: $R_{IA}\mathbf f_0$ is a follower force fixed to body $\mathcal A$, not a force of fixed direction in space. Third, the choice of reference end is part of the law: swapping the ends reverses the relative position and expresses it in C, the stiffness matrix becomes $R_{CA}\operatorname{diag}(\mathbf k)R_{CA}^{\mathsf T}$, which is in general no longer diagonal unless $R_{AC}$ maps coordinate axes to coordinate axes (repeated values among the $k_i$ can be an exception), the transport term of the velocity input is formed from $\boldsymbol\omega_C$ instead of $\boldsymbol\omega_A$ and the support moment moves to the other body. Only an isotropic law escapes the matrix part of this dependence, see Section 2.8; the general discussion of swapping the ends is in Section 6 of the shared chapter.

### 2.8 Special cases

**Pure spring** ($\mathbf c=\mathbf 0$). $\mathbf f_A=\mathbf k\circ\mathbf d_A+\mathbf f_0$ and $\mathcal P=-\dot{\mathcal V}$; the element is conservative and its wrench pair consists of the coefficients of $-\delta\mathcal V$.

**Pure damper** ($\mathbf k=\mathbf 0$, $\mathbf f_0=\mathbf 0$). $\mathbf f_A=\mathbf c\circ\mathbf u_A$ and $\mathcal P=-\mathcal D\le0$, with no stored energy. The force is still stated axis by axis in A, still turns with the attitude of $\mathcal A$ and still carries the support moment $\mathbf d_A\times(\mathbf c\circ\mathbf u_A)$; it is not a one-dimensional dashpot along the line of centers responding only to the rate of change of that line's length.

**Single axis** ($\mathbf k=k\,\mathbf e_j$, $\mathbf c=c\,\mathbf e_j$, $\mathbf f_0=f_0\,\mathbf e_j$). Then

$$
\mathbf f_A=\left(k\,d_{A,j}+c\,u_{A,j}+f_0\right)\mathbf e_j,
\qquad
\mathbf d_I\times\mathbf f_I=\left(k\,d_{A,j}+c\,u_{A,j}+f_0\right)R_{IA}\left(\mathbf d_A\times\mathbf e_j\right).
$$

The force lies along the $j$-th axis of A rather than along the line of centers, and the support moment vanishes when $\mathbf d_A$ is parallel to $\mathbf e_j$ or the scalar force is zero, and not in general. The [series spring-viscous-damper element](SERIES_SPRING_VISCOUS_DAMPER.en.md) and the [odd-symmetric saturated piecewise-linear damper](SATURATED_PIECEWISE_LINEAR_DAMPER.en.md) use this load form with their own scalar forces.

**Isotropic** ($\mathbf k=k\begin{bmatrix}1&1&1\end{bmatrix}^{\mathsf T}$, $\mathbf c=c\begin{bmatrix}1&1&1\end{bmatrix}^{\mathsf T}$). Now $R_{IA}\operatorname{diag}(\mathbf k)R_{IA}^{\mathsf T}$ is $k$ times the identity, and

$$
\mathbf f_I=k\,\mathbf d_I+c\,\mathbf u_I+R_{IA}\mathbf f_0.
$$

The spring and damping parts no longer depend on the attitude of $\mathcal A$, while the nominal force still turns with A. Taking in addition $c=0$ and $\mathbf f_0=\mathbf 0$, $\mathbf f_I=k\,\mathbf d_I$ is parallel to $\mathbf d_I$ and the support moment vanishes identically: this is the only case in which the family reduces to a central force, the element becoming a linear two-point spring of zero natural length with $\mathcal V=\tfrac12k\,\lVert\mathbf d_I\rVert^2$. With $c\neq0$, $\mathbf u_I$ in general has a component perpendicular to $\mathbf d_I$ and the force is not central; moreover $\mathbf u_I$ is the velocity relative to body $\mathcal A$, not $\dot{\mathbf d}_I$, so isotropy removes the attitude dependence of the matrices but not the dependence of the transport term on the choice of reference end.

## 3. Computational implementation

### 3.1 Reading the relative motion

At every evaluation, one relative-motion record is first obtained from the two frames as in Section 5.1 of the shared chapter. The family reads $\mathbf d_A$ and $\mathbf u_A$ from it as constitutive inputs: $\mathbf u_A$ comes directly from the multibody layer's query for the spatial velocity of C relative to A expressed in A, whose translational component is by contract the velocity of $C_o$ measured in A, transport term included, so the family does not construct it itself. It further reads what writing the loads requires: $R_{IA}$, $\mathbf d_I$ and the body-fixed points $(\mathcal A,\mathbf r_{A_o}^{\mathcal A})$ and $(\mathcal C,\mathbf r_{C_o}^{\mathcal C})$ of the two origins. The relative attitude and the relative angular velocity are not used by this family.

### 3.2 Evaluating the law

$\mathbf f_A$ is formed from two componentwise products and one addition: $\mathbf k\circ\mathbf d_A$ plus $\mathbf c\circ\mathbf u_A$, plus the $\mathbf f_0$ of this instance. $\mathbf f_0$ is not part of the element type; it is supplied as an evaluation input alongside the state, three components per instance, expressed in A and acting on the reference end, read during evaluation and never integrated. The evaluation has no branch and rewrites no history. The family contributes no component to the $z$ block of the continuous state; that block consists solely of the [series spring-viscous-damper element](SERIES_SPRING_VISCOUS_DAMPER.en.md), see Section 1.1 of [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md).

### 3.3 Writing the wrenches

$\mathbf f_I=R_{IA}\mathbf f_A$ is computed first, and the two load entries are then written along the endpoint-pair path of Section 5.2 of the shared chapter: $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf d_I\times\mathbf f_I,\ \mathbf f_I)$ for $\mathcal A$ and $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ \mathbf 0,\ -\mathbf f_I)$ for $\mathcal C$, both expressed in I. The support moment is the cross product of $\mathbf d_I$ and $\mathbf f_I$ formed in I. The multibody layer then moves each entry to the origin of its body and adds it to forward dynamics, see Sections 3.5 and 5.3 of the shared chapter.

## 4. Mathematical properties and conditions of applicability

- **Algebraic, stateless and smooth.** The force is an affine function of $(\mathbf d_A,\mathbf u_A)$, uniquely determined by the current $(q,v)$, free of history and differentiable everywhere; the family does not change the dimension of the continuous state.
- **Dissipation inequality and passivity.** Every component of $\mathbf k$ and $\mathbf c$ non-negative and finite, and $\mathbf f_0$ finite, are the conditions of applicability of the family. Under them $\mathcal P=-\dot{\mathcal V}-\mathcal D$ with $\mathcal D\ge0$, the dissipation inequality; passivity further requires a storage function bounded below, that is, $f_{0,i}=0$ on every axis with $k_i=0$. When the three stiffnesses are positive $\mathcal V$ is bounded below and there is a unique force-free configuration $\mathbf d_{0,A}$.
- **Zero displacement is a regular configuration.** The law is written on the components of $\mathbf d_A$, not on its length, and contains no unit direction vector; at coincident origins the force $\mathbf c\circ\mathbf u_A+\mathbf f_0$ is well defined and the support moment vanishes there. The two origins may be placed coincident, with $\mathbf f_0$ carrying the force in that configuration.
- **Not a central force in general.** The force is resolved along the axes of A, not along the line joining the two origins, and the support moment is generally nonzero; only the isotropic special case with zero damping and zero nominal force is a central force along that line. Treating the family as a one-dimensional element along the line of centers yields transverse forces and moments other than the expected ones.
- **Principal axes turn with the reference end.** The principal axes of stiffness and damping and the direction of the nominal force are all fixed to body $\mathcal A$. The choice of reference end and the attitude of the reference-end frame are both part of the law; swapping the ends or changing the attitude of A generally yields a different element. Isotropy removes only the attitude dependence of the coefficient matrices: with zero nominal force, an isotropic elastic part is a central force, has zero support moment and is invariant under swapping the ends, whereas an isotropic damping part still takes the transport-carrying $\mathbf u_A$ as input, so after the swap the transport term is formed from $\boldsymbol\omega_C$ and the loads in general differ (Section 2.8).
- **A linear law is not a linear system contribution.** In A, $\partial\mathbf f_A/\partial\mathbf d_A=\operatorname{diag}(\mathbf k)$ and $\partial\mathbf f_A/\partial\mathbf u_A=\operatorname{diag}(\mathbf c)$ are constant matrices; but $\mathbf d_A$ and $\mathbf u_A$ depend on the attitude of $\mathcal A$ through $R_{IA}$, and the support moment $\mathbf d\times\mathbf f$ varies with the configuration, so the family's contribution to the system's tangent stiffness and tangent damping is configuration-dependent and contains geometric-stiffness terms.
- **Scope of the nominal force.** $\mathbf f_0$ is constant and is equivalent to a zero-force displacement only on axes with positive stiffness; on an axis with zero stiffness and $f_{0,i}\neq0$ it is a constant force turning with A, whose potential is unbounded below along that axis.
- **The two ends must lie on different bodies.** See Section 6 of the shared chapter; with both ends on one body $\mathbf u_A=\mathbf 0$, $\mathbf d_A$ is constant and the paired wrench lands on a single body with no net effect.
- **What the family represents.** The family describes a translational spring-damper whose three principal directions are fixed to the reference-end body and whose three components act independently, suited to connections whose principal stiffness axes turn with one body; where a one-dimensional element acting along the line of centers is required, only the isotropic pure-spring special case matches it.

## 5. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| Element type: the two ends and the three-axis stiffness and damping | `TranslationalSpringDamper`, in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| Evaluation entry point and the input contract of $\mathbf f_0$ (three components per instance, expressed in A, acting on the reference end) | `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.h`](../../../libs/forces/include/orvd/forces/vehicle_force_plan.h) |
| One-shot evaluation of the relative motion $\mathbf d_A$, $\mathbf u_A$, $\mathbf d_I$, $R_{IA}$ | `CalcRelativeMotion` within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Evaluation of the law $\mathbf f_A=\mathbf k\circ\mathbf d_A+\mathbf c\circ\mathbf u_A+\mathbf f_0$ | The translational-family branch of `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Writing the endpoint force pair with the reference-end support moment | `EmitTranslationalWrenchPair` within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Per-instance statement of the nominal force $\mathbf f_0$ | `SystemInstance::SetNominalForce`, in [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| Contract of the relative spatial velocity, translational component including the transport term | `MultibodyModel::CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame`, in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Load entry and body-fixed point | `AppliedBodyWrench`, `BodyFixedPoint`, in [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
| Reduction of loads to the body origin and entry into forward dynamics | `MultibodyModel::CalcStateTimeDerivatives`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
