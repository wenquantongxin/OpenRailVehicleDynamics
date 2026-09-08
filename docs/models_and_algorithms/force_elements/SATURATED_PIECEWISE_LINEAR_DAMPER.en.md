[中文](SATURATED_PIECEWISE_LINEAR_DAMPER.md)

# Odd-symmetric saturated piecewise-linear damper

This chapter describes one constitutive family under [Force-element kinematics and spatial wrenches](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md), referred to below as the shared chapter. The family is a memoryless damper acting along one fixed axis of the reference-end frame: a finite sequence of nodes defines a force curve on the non-negative velocity half-axis that starts at the origin, interpolates linearly between nodes and holds a constant value beyond the last node, and that curve is extended to the whole velocity axis as an odd function. The chapter defines the curve and its extension, derives the mathematical consequences of the three requirements on its domain, states the axial relative-velocity input and the paired wrench and proves that the family satisfies the power principle of Section 4.5 of the shared chapter. The element type and its node type are `SaturatedPiecewiseLinearDamper` in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h). The curve evaluation and the writing of the loads are `EvaluateValidatedSaturatedDamperCurve` and `VehicleForcePlan::CalcAppliedForces` in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc).

## 1. Objects and notation

### 1.1 The element, its axis and its two ends

The element connects the reference-end frame A and the opposite-end frame C, fixed to the rigid bodies $\mathcal A$ and $\mathcal C$ respectively, with the meaning of Section 1.1 of the shared chapter. The family acts along one coordinate axis of A only; its index is $j\in\{1,2,3\}$ for the longitudinal, lateral and vertical axis in turn. The axis is a direction fixed in body $\mathcal A$ and in general does not lie along the line joining the two origins. The constitutive law is one-dimensional: its input is a scalar velocity and its output a scalar force, both taken along that axis. The family carries no internal state and reads no displacement or attitude quantity.

### 1.2 Notation

| Symbol | Meaning | Source |
|---|---|---|
| A, C, $\mathcal A$, $\mathcal C$, I | Reference-end and opposite-end frames, the two bodies carrying them and the inertial frame | Shared chapter, Section 1.1 |
| $\mathbf u_A$ | Velocity of the opposite-end origin relative to the reference end (relative to body $\mathcal A$), expressed in A, transport term included, $\mathbf u_A=\dot{\mathbf d}_A$ | Shared chapter, Section 2.3 |
| $\mathbf d_I$, $R_{IA}$ | Relative position of the two origins (in I) and attitude of the reference end in I | Shared chapter, Section 2.1 |
| $\mathbf f_A$, $\mathbf f_I$ | Force on the reference end, expressed in A and in I | Shared chapter, Section 1.3 |
| $\mathcal W_Q^E[\mathcal A]$ | Wrench acting on body $\mathcal A$, reduced about Q, expressed in E | Shared chapter, Section 1.3 |
| $\mathbf r_{A_o}^{\mathcal A}$, $\mathbf r_{C_o}^{\mathcal C}$ | Coordinates of the two origins in their own body frames | Shared chapter, Section 3.1 |
| $\mathcal P$ | Total power the element delivers to the two bodies | Shared chapter, Section 1.2 |
| $\mathbf e_j$ | The $j$-th standard basis vector | Conventions, Section 4.1 |
| $j$ | One-based index of the axis of action in A, $j\in\{1,2,3\}$ | This chapter |
| $u$ | Axial relative velocity, $u=u_{A,j}=\mathbf e_j\cdot\mathbf u_A$ | This chapter |
| $w$ | Axial relative speed, $w=\lvert u\rvert\ge0$ | This chapter |
| $n$ | Number of linear segments; there are $n+1$ nodes | This chapter |
| $(u_i,g_i)$ | Velocity and force value of node $i$, $i=0,\dots,n$ | This chapter |
| $c_i$ | Incremental damping of segment $i$, that is, its slope, $i=1,\dots,n$ | This chapter |
| $g$ | Force curve on the non-negative velocity half-axis, $g:[0,\infty)\to[0,\infty)$ | This chapter |
| $F$ | Scalar constitutive force $F(u)$ after the odd extension, acting on the reference end | This chapter |
| $\operatorname{sgn}$ | Sign function | This chapter |
| $\lambda$ | Interpolation parameter within a segment | This chapter |
| $\Phi$ | Dissipation potential | This chapter |

### 1.3 Relation to the shared chapter and the conventions

All notation for relative motion, wrenches and power follows Section 1 of the shared chapter, including the bracketed loaded body in $\mathcal W_Q^E[\mathcal A]$ and the last-subscript convention for the expressed-in frame of a wrench component; the relative velocity, the reduction-point rule and the power identities already derived there are cited here, not rederived. Rotation matrices and the standard basis vectors $\mathbf e_j$ follow Section 4.1 of [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md), and vector components are numbered from 1.

The following symbols carry a meaning specific to this chapter and are declared here to keep clear of their use elsewhere: $g$ is the force curve on the non-negative velocity half-axis, not the grade $g(s)$ of Section 2.1 of the conventions, whose argument is the station, nor the vertical interpenetration function $g(Y)$ of Section 2.7; the scalar $u$ is always the $j$-th component of $\mathbf u_A$, not the superelevation $u$ of Section 2.2 of the conventions nor the Euclidean displacement $u$ of Section 1.2 of [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md); $F$ is the scalar constitutive force of this family and is unrelated to the normal-force components $F_e$ and $F_d$ of the wheel-rail contact chain; the unsubscripted $n$ is the number of linear segments and is unrelated to the state dimension $n_x$ and its kin; $c_i$ is the incremental damping of segment $i$, lowercase $c$ as required by Section 5.3 of the conventions, with the subscript naming the segment; the scalar $w$ is a speed and is unrelated to the dummy vector in $\operatorname{skew}(\mathbf w)$ of the shared chapter.

## 2. Model and derivation

### 2.1 The curve $g$ on the non-negative velocity half-axis

The curve is given by $n+1$ nodes $(u_i,g_i)$, $i=0,\dots,n$, where $u_i$ is a node velocity and $g_i$ a node force value. The three requirements on the domain are: the node velocities increase strictly; the force values are finite and non-negative; the first node is the origin. Written out,

$$
0=u_0<u_1<\cdots<u_n,
\qquad
g_i\ge0\ (i=0,\dots,n),
\qquad
g_0=0,
\qquad n\ge1.
$$

$n\ge1$ says only that there is at least one segment. The incremental damping of each segment, that is, its slope, is

$$
c_i=\frac{g_i-g_{i-1}}{u_i-u_{i-1}},\qquad i=1,\dots,n.
$$

Strict increase makes every denominator positive and every slope finite, and the segment intervals $[u_{i-1},u_i]$ meet only at their endpoints. The half-axis curve is defined as

$$
g(w)=
\begin{cases}
g_{i-1}+c_i\left(w-u_{i-1}\right),& u_{i-1}\le w\le u_i,\quad i=1,\dots,n,\\
g_n,& w>u_n.
\end{cases}
$$

Holding the constant value $g_n$ beyond the last node is the definition of this family: the curve saturates at $u_n$ and the slope of the final segment is not continued beyond the nodes that define it. Two adjacent segments both take the value $g_i$ at their common node $u_i$, so $g$ is continuous on $[0,\infty)$, and $g_0=0$ gives $g(0)=0$. Inside each segment $g$ is affine, so $g$ is a continuous piecewise-affine function whose left and right derivatives at node $u_i$ ($1\le i\le n$) are $c_i$ and $c_{i+1}$, the right derivative being zero for $i=n$.

### 2.2 Odd extension and the value at the origin

The constitutive force on the whole velocity axis is the odd extension of $g$:

$$
F(u)=\operatorname{sgn}(u)\,g(|u|),
\qquad
F(0)=0,
\qquad
F(-u)=-F(u).
$$

Here $\operatorname{sgn}(u)$ is $+1$ for $u>0$ and $-1$ for $u<0$. The value at the origin does not depend on any convention for $\operatorname{sgn}(0)$: whether it is taken as $+1$, $-1$ or $0$, $g(0)=0$ gives $F(0)=0$. Oddness, $F(-u)=-F(u)$, follows from the definition for $u\ne0$ and from $F(0)=0$ at $u=0$. The curve on the negative half-axis is never stated separately, so the force law of this family is symmetric by construction.

$F$ is continuous at the origin because $g$ is continuous at 0 with $g(0)=0$; this is precisely what the requirement that the first node be the origin achieves. Were $g_0>0$ allowed, the odd extension would jump by $2g_0$ at the origin and the element would become one with a static-friction-type discontinuity, which is not this family. In fact, within the first segment,

$$
F(u)=c_1\,u,\qquad |u|\le u_1,
$$

so $F$ is not merely continuous but differentiable at the origin, with $F'(0)=c_1$; the origin is not a kink. Kinks occur only at $\pm u_i$ ($1\le i\le n$), and only where the slopes on the two sides of the node differ: $c_i\ne c_{i+1}$ at an interior node, and $c_n\ne0$ at the saturation end $u_n$.

### 2.3 Consequences of the three domain requirements

- **Strictly increasing node velocities.** $g$ is single-valued and every slope is finite; the curve has no vertical segment and no repeated node. The segment search of Section 3.2 therefore has a unique result for every $w$.
- **Finite non-negative force values.** Within a segment, $g(w)$ is a convex combination of $g_{i-1}$ and $g_i$, and in the saturated region it equals $g_n$, so $g(w)\ge0$ for all $w\ge0$. Hence $F(u)\,u=|u|\,g(|u|)\ge0$: the constitutive force has the sign of the axial relative velocity or vanishes; the element only dissipates and never does positive work. Equality holds exactly when $u=0$ or $g(|u|)=0$.
- **First node at the origin.** $F(0)=0$: the element exerts no static force and has no jump at the origin (Section 2.2).
- **What is not required.** The node force values need not be monotone. Consequently $c_i$ may be negative, $F$ need not be a monotone function of $u$ and $|F|$ may decrease as $|u|$ grows; the saturation value $g_n$ need not be the largest force value either. The three requirements guarantee dissipation in the sign sense, $F(u)u\ge0$, not positive damping in the incremental sense.
- **Boundedness.** $|F(u)|\le\max_i g_i$ for all $u$, and $|F(u)|=g_n$ for $|u|\ge u_n$.

### 2.4 Axial velocity input and the reference-end force

The family acts along the $j$-th axis of A. Its velocity input is the component along that axis of the $\mathbf u_A$ of Section 2.3 of the shared chapter, and the force on the reference end lies along the same axis:

$$
u=\mathbf e_j\cdot\mathbf u_A=u_{A,j},
\qquad
\mathbf f_A=F(u)\,\mathbf e_j .
$$

$\mathbf u_A=\dot{\mathbf d}_A$ contains the transport term: it is the velocity of the opposite-end origin relative to body $\mathcal A$, not the difference of the two origins' inertial velocities, and this component necessarily agrees with the corresponding component of $R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ when the $j$-th axis lies along the line joining the two origins, while for a general direction the two differ by the axial projection of the transport term, $\mathbf e_j^{\mathsf T}R_{IA}^{\mathsf T}(\boldsymbol\omega_A\times\mathbf d_I)$, and agree only when that projection vanishes (Section 6 of the shared chapter). The sign convention is that of the shared chapter: $\mathbf f_A$ is the force on the reference end. $u>0$ means that the opposite-end origin moves away from body $\mathcal A$ along the positive $j$-th axis of A; then $F(u)\ge0$, the reference end is pulled along the positive $j$-th axis, that is, in the direction in which the opposite end is departing, and the opposite end receives the opposite force: the damper resists the relative motion. The displacement $\mathbf d_A$, the relative attitude $R_{AC}$ and the relative angular velocity $\boldsymbol\omega_{rel,A}$ do not enter the law. Within the first segment, $|u|\le u_1$, the family produces the same force as a [three-axis translational spring-damper element](TRANSLATIONAL_SPRING_DAMPER.en.md) with zero stiffness, damping $c_1\mathbf e_j$ and no nominal force.

### 2.5 Paired wrench

The loads are applied as the endpoint force pair with the reference-end support moment of Section 3.2 of the shared chapter:

$$
\mathbf f_I=R_{IA}\mathbf f_A=F(u)\,R_{IA}\mathbf e_j,
\qquad
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(\mathbf 0,\ -\mathbf f_I\right).
$$

$R_{IA}\mathbf e_j$ is the $j$-th column of $R_{IA}$, the direction of the $j$-th axis of A in I. The support moment $\mathbf d_I\times\mathbf f_I$ acts on body $\mathcal A$ and is reduced about $A_o$; it is nonzero whenever $F(u)\ne0$ and $\mathbf d_I$ is not parallel to the axis of action. The two loads have zero resultant force and zero resultant moment about any point, as shown in Section 3.2 of the shared chapter.

### 2.6 Proof of the power principle

In the table of Section 4.5 of the shared chapter, the row for the endpoint force pair with the support moment has velocity input $\mathbf u_A$ and endpoint power $-\mathbf f_A\cdot\mathbf u_A$. The velocity input of this family, $u=\mathbf e_j\cdot\mathbf u_A$, is exactly the component of that row's quantity along the axis of action, and the application scheme is exactly that row's scheme, so the identity of Section 4.2 of the shared chapter gives

$$
\mathcal P=-\mathbf f_A\cdot\mathbf u_A=-F(u)\,\mathbf e_j\cdot\mathbf u_A=-F(u)\,u=-|u|\,g(|u|)\le0 .
$$

The second equality uses the fact that $\mathbf f_A$ has only a $j$-th component, the third the definition of $u$, the fourth $F(u)u=|u|g(|u|)$ and the inequality the non-negativity of Section 2.3. The left-hand side is the total power the two bodies receive from the element and $-F(u)u$ is the power of the one-dimensional damping law itself; their equality is the power principle. It holds for arbitrary $\mathbf d_I$ and arbitrary $\boldsymbol\omega_A$, because the power of the support moment, $\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)$, supplies exactly the power of the transport term. Keeping $\mathbf u_A$ as the input but dropping the support moment changes the power received by the two bodies to $-\mathbf f_A\cdot R_{IA}^{\mathsf T}\dot{\mathbf d}_I=-F(u)\,u-F(u)\,\mathbf e_j\cdot R_{IA}^{\mathsf T}(\boldsymbol\omega_A\times\mathbf d_I)$, whose extra term equals $-F(u)$ times the axial projection of the transport term, $\mathbf e_j^{\mathsf T}R_{IA}^{\mathsf T}(\boldsymbol\omega_A\times\mathbf d_I)$, necessarily vanishes when the axis of action lies along the line joining the two origins and, for a general direction, vanishes when $F(u)=0$ or when that projection vanishes; conversely, taking the component of $R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ as the input while keeping the support moment likewise leaves the power recorded by the law unequal to the endpoint power. That the power is never positive shows that the family stores no energy: the work it does on the two bodies is non-increasing, with equality only when $u=0$ or $g(|u|)=0$.

## 3. Computational implementation

### 3.1 Reading the relative motion

At every evaluation the element first obtains its relative-motion record as in Section 5.1 of the shared chapter. The family consumes only $\mathbf u_A$ from it, together with the $\mathbf d_I$, $R_{IA}$ and the body-fixed points of the two origins needed to write the loads; $\mathbf d_A$, $R_{AC}$ and $\boldsymbol\omega_{rel,A}$ play no part. The axis of action is named by the three-valued enumeration `ForceElementAxis`, whose longitudinal, lateral and vertical values correspond to the one-based indices $j=1,2,3$; $u$ is the $j$-th component of $\mathbf u_A$, that is, element $j-1$ of the Eigen vector. The family has no internal state and writes nothing into the $z$ block; $u$ is determined entirely by the current $(q,v)$.

### 3.2 Evaluating the curve

Given $u$, the evaluation proceeds in three steps that correspond one to one to the definitions of Sections 2.1 and 2.2.

1. **Sign and speed.** The sign is $-1$ when $u<0$ and $+1$ otherwise; $w=|u|$. $u=0$ falls into the $+1$ branch, but since $g(0)=0$ the result is still $F(0)=0$, so this branch does not change the mathematical result.
2. **Saturation branch.** If $w\ge u_n$, return the sign times $g_n$. This implements the constant continuation beyond the last node; $w=u_n$ itself takes this branch, and the resulting $g_n$ equals the value of segment $n$ at its right endpoint.
3. **Interpolation within a segment.** Otherwise, scanning from $i=1$, find the first segment with $w\le u_i$ and compute $\lambda=\frac{w-u_{i-1}}{u_i-u_{i-1}}\in[0,1]$ and $g(w)=(1-\lambda)\,g_{i-1}+\lambda\,g_i$, then return the sign times $g(w)$. The convex-combination form is algebraically identical to the slope form $g_{i-1}+c_i(w-u_{i-1})$ of Section 2.1. Because the node velocities increase strictly, the segment found is unique; when $w$ is exactly an interior node $u_i$, segment $i$ is found with $\lambda=1$ and $g_i$ is returned, which agrees with the value of segment $i+1$ at $\lambda=0$.

At every branch boundary — the origin, each interior node, the saturation end — the two sides give the same value; a branch only selects which segment's formula applies and introduces no jump. This is the continuity of $F$ in implemented form. The evaluation presumes the three requirements of Section 2.1. The scalar law $F(u)$ also has a public entry point that needs no assembled system, `VehicleForcePlan::SaturatedPiecewiseLinearDamperForce`, which returns $F(u)$ directly for a given element and velocity.

### 3.3 Writing the wrenches

From $F(u)$ the vector $\mathbf f_A=F(u)\,\mathbf e_j$ is formed, a three-vector whose only nonzero component is the $j$-th; it is premultiplied by $R_{IA}$ to give $\mathbf f_I$ and handed, together with $\mathbf d_I$ and the body-fixed points of the two origins, to the routine that writes the endpoint force pair, which produces the two load entries $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf d_I\times\mathbf f_I,\ \mathbf f_I)$ and $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ \mathbf 0,\ -\mathbf f_I)$ of the first item of Section 5.2 of the shared chapter. This path is the same one the translational and series families use; the family adds no load term of its own.

## 4. Mathematical properties and conditions of applicability

- **Odd symmetry.** The negative half-axis is never stated separately, so $F(-u)=-F(u)$ holds by construction and the characteristics in extension and compression are identical. The family suits velocity-dependent dampers whose two directions behave alike; a direction-dependent characteristic is outside this family.
- **Pure dissipation, no stored energy.** $\mathcal P=-|u|\,g(|u|)\le0$ for every motion, so the element cannot feed energy into the system. The force depends on one component of $\mathbf u_A$ alone and not on $\mathbf d_A$ or $R_{AC}$, so it vanishes identically at $u=0$: the family produces no static force and cannot represent a preload. It also has no frequency dependence and no hysteresis; it is an algebraic law. A damper with a compliance in series belongs to the [series spring-viscous-damper element](SERIES_SPRING_VISCOUS_DAMPER.en.md).
- **Dissipation potential.** $F$ is the derivative of an even, non-negative, $C^1$, piecewise-quadratic function: $\Phi(u)=\int_0^{u}F(\xi)\,d\xi=\int_0^{|u|}g(w)\,dw\ge0$, $F=\Phi'$. $\Phi$ is convex exactly when $F$ is non-decreasing, that is, when every $c_i\ge0$; the family does not require this, and $\Phi$ is in general non-convex.
- **Non-monotonicity and negative incremental damping.** Inside each segment and in the saturated region the tangent damping is $F'(u)=c_i$ for $u_{i-1}<|u|<u_i$ and $F'(u)=0$ for $|u|>u_n$. When the operating point lies on a segment with $c_i<0$, the linearization about it has negative tangent damping even though the total power remains non-positive; when the operating point lies in the saturated region the tangent damping is zero and the family contributes nothing to the linearized damping; when it lies inside the first segment, $|u_*|<u_1$ (including $u_*=0$), the linearized damping is $c_1$. For linearized analysis the damping contribution of this family is a quantity that depends on the operating point, not a constant.
- **Boundedness and saturation.** $|F|\le\max_i g_i$, and the force does not grow at large velocity. The saturation value $g_n$ is the force value of the last node and is not necessarily the maximum of the curve.
- **Continuous, Lipschitz, piecewise $C^1$.** $F$ is continuous on the whole real line with Lipschitz constant $\max_i|c_i|$; its kinks are those points $\pm u_i$ at which the slopes on the two sides differ, and the origin is not a kink. The right-hand side of the state equation is therefore continuous in $v$ but only piecewise $C^1$, and its Jacobian with respect to $v$ jumps when $u$ crosses a kink.
- **Single axis, body-fixed direction.** The axis of action is a coordinate axis of A and in general does not lie along the line joining the two origins, so $u$ contains the transport term and the support moment is in general nonzero; the family is not a point-to-point damper acting along the line of centers but an axial damper acting along a direction fixed in the reference end. The choice of reference end is part of the law: taking the other end as reference places the axis on that end's body and in general yields a different element (Section 6 of the shared chapter).
- **Degenerate cases.** For $n=1$ the element is a linear viscous damper of coefficient $c_1$ for $|u|\le u_1$ and saturates at $g_1$ beyond; when every $g_i=0$, $F\equiv0$ and the element has no effect on the motion.

## 5. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| Element type, node type and the contract of the three domain requirements | `SaturatedPiecewiseLinearDamper`, `SaturatedPiecewiseLinearDamperPoint`, in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| Type of the axis index $j$ | `ForceElementAxis`, in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| Evaluation of the half-axis curve $g$ and the odd extension $F(u)$ | `EvaluateValidatedSaturatedDamperCurve`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Public entry point of the scalar law $F(u)$ | `VehicleForcePlan::SaturatedPiecewiseLinearDamperForce`, in [`vehicle_force_plan.h`](../../../libs/forces/include/orvd/forces/vehicle_force_plan.h) |
| Selecting the $j$-th component of $\mathbf u_A$, forming $\mathbf f_A$ and taking it to I | The saturated-family branch within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| One-shot evaluation of the relative motion $\mathbf u_A$, $\mathbf d_I$, $R_{IA}$ | `CalcRelativeMotion`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Endpoint force pair with the reference-end support moment | `EmitTranslationalWrenchPair`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Load entry and body-fixed point | `AppliedBodyWrench`, `BodyFixedPoint`, in [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
