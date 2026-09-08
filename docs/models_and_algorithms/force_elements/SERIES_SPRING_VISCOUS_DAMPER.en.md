[中文](SERIES_SPRING_VISCOUS_DAMPER.md)

# Series spring-viscous-damper element

This chapter covers one family of the force-element documents: a linear spring in series with a linear viscous damper, acting along one axis of the reference-end frame. Because the two are in series they share one force while deforming differently, so the force is no longer an algebraic function of the relative motion but a first-order internal state. The chapter derives the Maxwell force equation from the two series conditions, equal force and additive deformation, defines the relaxation time, gives the analytic responses to step and sinusoidal excitation, explains how the force state enters the $z$ block of the system's continuous state and how its derivative is produced in the same evaluation as the two loads and proves that the family satisfies the organizing principle of Section 4.5 of [Force-element kinematics and spatial wrenches](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md). The relative motion of the two ends, the three wrench application schemes and the power identities are all derived in that shared chapter and are cited here rather than repeated. The element type is `SeriesSpringViscousDamper` in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h); the constitutive evaluation and the state derivative are formed in `VehicleForcePlan::CalcAppliedForces` in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc).

## 1. Objects and notation

### 1.1 The element

The element connects the reference-end frame A and the opposite-end frame C, fixed to the rigid bodies $\mathcal A$ and $\mathcal C$ respectively, with the meaning of Section 1.1 of the shared chapter. It consists of a linear spring of stiffness $k$ and a linear viscous damper of damping $c$ joined end to end, and it transmits force only along the $j$-th coordinate axis $\mathbf e_j$ of A, $j\in\{1,2,3\}$; along the other two axes it transmits nothing. $k$, $c$ and the acting axis are all stated in A, so, as for the other families of the shared chapter, the choice of reference end is part of the law.

The force the element transmits is written $F$; it is the only internal state of the family. For $F>0$ the force on the reference end points along $+\mathbf e_j$ and the force on the opposite end along $-\mathbf e_j$; relative motion of the opposite end along $+\mathbf e_j$ drives $F$ upward, that is, the force on the reference end follows the direction of the opposite end's relative motion, consistent with the sign of $\mathbf f_A$ in the [three-axis translational spring-damper element](TRANSLATIONAL_SPRING_DAMPER.en.md).

### 1.2 Notation

| Symbol | Meaning | Source |
|---|---|---|
| A, C, $\mathcal A$, $\mathcal C$, I | Reference end, opposite end, the two bodies and the inertial frame | Shared chapter §1.1 |
| $\mathbf d_A$, $\mathbf d_I$ | Relative position of the two origins, expressed in A and in I | Shared chapter §2.1 |
| $R_{IA}$, $R_{AC}$ | Attitude of the reference end in I, relative attitude of the opposite end in the reference end | Shared chapter §1.2 |
| $\mathbf u_A$ | Velocity of the opposite-end origin relative to the reference end (relative to body $\mathcal A$), expressed in A, $\mathbf u_A=\dot{\mathbf d}_A$ | Shared chapter §2.3 |
| $\boldsymbol\omega_A$, $\boldsymbol\omega_{rel,A}$ | Angular velocity of body $\mathcal A$ (expressed in I), angular velocity of C relative to A (expressed in A) | Shared chapter §1.2 |
| $\mathbf f_A$, $\mathbf f_I$ | Force on the reference end, expressed in A and in I | Shared chapter §1.3 |
| $\mathcal W_Q^E[\mathcal A]$ | Wrench acting on body $\mathcal A$, reduced about point Q, expressed in E | Shared chapter §1.3 |
| $\mathbf r_{A_o}^{\mathcal A}$, $\mathbf r_{C_o}^{\mathcal C}$ | Body coordinates of the two origins in their own bodies | Shared chapter §3.1 |
| $\mathcal P$ | Total power the element delivers to the two bodies | Shared chapter §1.2 |
| $\mathbf e_j$ | The $j$-th standard basis vector of A, that is, the acting axis of the element | Conventions §4.1 |
| $[q;v;z]$, $N(q)$ | System continuous state and position-derivative map | Conventions §5.1, §4.5 |
| $k$, $c$ | Series stiffness and series damping, positive scalars | Conventions §5.3 |
| $u$ | Relative velocity of the two ends along the acting axis, $u=\mathbf e_j^{\mathsf T}\mathbf u_A$ | New here |
| $F$ | Force transmitted by the element, that is, the internal force state | New here |
| $\delta_k$, $\delta_c$ | Elongations of the spring and of the damper, used only in the derivation | New here |
| $\tau_{\mathrm r}$ | Relaxation time $c/k$ | New here |
| $\mathcal V$, $\mathcal D$ | Energy stored in the spring and power dissipated in the damper | Shared chapter §1.2 |
| $\omega$, $\hat u$, $\vartheta$ | Angular frequency and velocity amplitude of a sinusoidal excitation; phase lag of the force behind the velocity | New here |
| $\Delta$, $u_0$, $F_0$ | Size of a relative-position step, size of a relative-velocity step, force before the step | New here |
| $\sigma$ | Dummy time variable of the convolution integral | New here |

### 1.3 Relation to the shared chapter and the conventions

Rotation matrices, wrenches and expressed-in subscripts follow Section 1.3 of the shared chapter exactly; $R_{IA}\mathbf e_j$ is the $j$-th column of $R_{IA}$ as in Section 4.1 of the conventions. The following symbols are local to this chapter and unrelated to symbols of the same shape elsewhere; they are declared here: the scalar $u$ is the $j$-th component of the shared chapter's relative velocity $\mathbf u_A$, a velocity and not the Euclidean displacement $u$ of Section 1.2 of [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md), whose Section 1.1 writes the same quantity as $v_{\mathrm{rel}}$; $F$ follows the force-state symbol of that Section 1.1 and is unrelated to $F_e$ and $F_d$ of the wheel-rail contact chain; $\delta_k$ and $\delta_c$ are the elongations of the element's two internal parts and are unrelated to the penetration $\delta$ of the contact chain; $\tau_{\mathrm r}$ is a scalar time, unrelated to the moment $\boldsymbol\tau$ of the shared chapter and to the generalized force $\tau$ of Section 5.3 of the conventions; $\omega$ is a scalar excitation frequency, unrelated to the angular velocity $\boldsymbol\omega_A$ and its components. Power follows the convention of the shared chapter: the load on the reference end is the positive vector, $\mathcal P$ is the power the element delivers to the two bodies and $-\mathcal P$ is the power the element absorbs.

## 2. Model and derivation

### 2.1 From the series conditions to the Maxwell force equation

A spring and a damper in series satisfy two conditions at once: they transmit the same force $F$, and their elongations add up to the total elongation of the element. With spring elongation $\delta_k$ and damper elongation $\delta_c$,

$$
F=k\,\delta_k,
\qquad
F=c\,\dot\delta_c,
\qquad
\dot\delta_k+\dot\delta_c=u.
$$

The right-hand side of the third equation is the relative velocity of the two ends along the acting axis, $u=\mathbf e_j^{\mathsf T}\mathbf u_A$: the total elongation rate of the element is the component along $\mathbf e_j$ of the velocity of the opposite end relative to the reference end. Section 6 of the shared chapter states that the kinematic layer provides no natural length, so the total elongation itself has no datum; the law of this family uses only its rate, and the datum never appears.

Differentiating the first equation, eliminating $\dot\delta_c=F/c$ with the second and using the third,

$$
\dot F=k\,\dot\delta_k=k\left(u-\dot\delta_c\right)=k\left(u-\frac{F}{c}\right),
$$

which is the Maxwell force equation

$$
\dot F=k\,u-\frac{k}{c}\,F.
$$

Defining the relaxation time

$$
\tau_{\mathrm r}=\frac{c}{k},
$$

the equation reads $\tau_{\mathrm r}\dot F+F=c\,u$. It is a first-order linear ordinary differential equation in the unknown $F$, with $u$ an input determined by the multibody motion. The derivation divided by $c$ and used $k$ to turn $\dot\delta_k$ into $\dot F$; these two steps require $c\ne0$ and $k\ne0$ respectively, and the degenerate cases with either at zero are discussed in Section 4.

The force becomes a state because the series structure adds one degree of freedom the kinematic layer cannot see: the multibody state $(q,v)$ fixes the rate of $\delta_k+\delta_c$ but not the individual values of $\delta_k$ and $\delta_c$, so for the same history of relative motion the force still depends on how much of the elongation the spring currently holds. That degree of freedom can be represented by $\delta_k$ or, equivalently, by the proportional quantity $F=k\,\delta_k$; the family takes $F$ because the load needs it directly. $\delta_c$ never has to be found on its own: its rate $F/c$ is fixed by $F$, and its absolute value, like the total elongation, has no datum.

### 2.2 Deformation measure and velocity input

The family reads no displacement: neither $\mathbf d_A$ nor $R_{AC}$ nor $\boldsymbol\omega_{rel,A}$ enters the law. Its only kinematic input is $u=\mathbf e_j^{\mathsf T}\mathbf u_A$, where $\mathbf u_A$ is the relative velocity with its transport term of Section 2.3 of the shared chapter, that is, $\dot{\mathbf d}_A$. Hence $u=\dot d_{A,j}$ is the time derivative of the $j$-th component of the relative position $\mathbf d_A$, not the projection onto the acting axis of the inertial velocity difference $\dot{\mathbf d}_I$; by Section 6 of the shared chapter the two differ only in the components perpendicular to the line joining the two origins, and they cannot be interchanged when the acting axis is not along that line. $\mathbf d_A$ appears only when the support moment is written out, in Section 2.4.

### 2.3 Analytic responses

The expressions below are symbolic solutions of the constitutive equation, stated as properties. With the force at time $t_0$ equal to $F(t_0)$, variation of constants gives, for any integrable input $u$,

$$
F(t)=F(t_0)\,e^{-(t-t_0)/\tau_{\mathrm r}}+k\int_{t_0}^{t}e^{-(t-\sigma)/\tau_{\mathrm r}}\,u(\sigma)\,d\sigma.
$$

The kernel $k\,e^{-t/\tau_{\mathrm r}}$ is the relaxation function, the force response of the element to a unit step of relative position. It shows that the current force is an exponentially weighted integral of the relative-velocity history, with a weight that forgets on the time scale $\tau_{\mathrm r}$.

**Step of relative position (stress relaxation).** Let the relative position of the two ends along the acting axis jump by $\Delta$ at $t=0$, with force $F_0$ before the jump and $u=0$ afterwards. At the instant of the jump the damper transmits a finite force and therefore has a finite elongation rate, so it cannot elongate at all and the whole jump lands on the spring, $F(0^+)=F_0+k\,\Delta$; thereafter

$$
F(t)=\left(F_0+k\,\Delta\right)e^{-t/\tau_{\mathrm r}},\qquad t>0.
$$

The force decays to zero with time constant $\tau_{\mathrm r}$: the element transmits no force in a configuration at rest and has no static stiffness.

**Step of relative velocity.** Let $u=u_0$ be constant for $t\ge0$ with $F(0)=F_0$. Then

$$
F(t)=c\,u_0+\left(F_0-c\,u_0\right)e^{-t/\tau_{\mathrm r}},\qquad t\ge0.
$$

Starting from $F_0=0$, the initial slope $\dot F(0)=k\,u_0$ is the response of a pure spring and the steady value $c\,u_0$ that of a pure damper; the transition from the former to the latter takes a time measured by $\tau_{\mathrm r}$.

**Sinusoidal steady state.** Let $u(t)=\hat u\sin\omega t$. Once the decaying homogeneous part has vanished,

$$
F(t)=\frac{c\,\hat u}{1+(\omega\tau_{\mathrm r})^2}\left(\sin\omega t-\omega\tau_{\mathrm r}\cos\omega t\right)
=\frac{c\,\hat u}{\sqrt{1+(\omega\tau_{\mathrm r})^2}}\,\sin\left(\omega t-\vartheta\right),
\qquad
\tan\vartheta=\omega\tau_{\mathrm r},
$$

where $0\le\vartheta<\pi/2$ is the phase lag of the force behind the velocity. In complex phasors the ratio of force to velocity is $c/(1+\mathrm i\omega\tau_{\mathrm r})$; dividing instead by the displacement phasor $\hat u/(\mathrm i\omega)$ gives the complex stiffness

$$
k^*(\omega)=\frac{\mathrm i\omega c}{1+\mathrm i\omega\tau_{\mathrm r}}
=k\,\frac{(\omega\tau_{\mathrm r})^2}{1+(\omega\tau_{\mathrm r})^2}
+\mathrm i\,k\,\frac{\omega\tau_{\mathrm r}}{1+(\omega\tau_{\mathrm r})^2}.
$$

The real part is the storage stiffness, which rises monotonically from zero to $k$ with $\omega\tau_{\mathrm r}$; the imaginary part is the loss stiffness, which peaks at $k/2$ when $\omega\tau_{\mathrm r}=1$. For $\omega\tau_{\mathrm r}\ll1$, $k^*\approx\mathrm i\omega c$ and the element behaves as the damper $c$; for $\omega\tau_{\mathrm r}\gg1$, $k^*\to k$ and it behaves as the spring $k$. The mean power dissipated over one cycle is

$$
\bar{\mathcal D}=\frac{c\,\hat u^2}{2\left(1+(\omega\tau_{\mathrm r})^2\right)}.
$$

### 2.4 Loads and the support moment

The law yields the force on the reference end, which in A has only its $j$-th component:

$$
\mathbf f_A=F\,\mathbf e_j,
\qquad
\mathbf f_I=R_{IA}\mathbf f_A=F\,R_{IA}\mathbf e_j.
$$

The loads are applied as the endpoint force pair with the reference-end support moment of Section 3.2 of the shared chapter:

$$
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(\mathbf 0,\ -\mathbf f_I\right).
$$

In A the support moment is $F\,\mathbf d_A\times\mathbf e_j$. It has no component along $\mathbf e_j$, and it vanishes when the line joining the two origins is parallel to the acting axis or when $F=0$; since the acting axis is a coordinate axis of A rather than the direction of that line, the support moment is nonzero in general.

### 2.5 Power principle, stored energy and dissipation

The family uses the first row of the table in Section 4.5 of the shared chapter: the force pair acts at the two origins with the reference-end support moment, and the velocity input is $\mathbf u_A$. Section 4.2 of the shared chapter proves that under this scheme the total power received by the two bodies is $\mathcal P=-\mathbf f_A\cdot\mathbf u_A$. Substituting $\mathbf f_A=F\,\mathbf e_j$,

$$
\mathcal P=-F\,\mathbf e_j^{\mathsf T}\mathbf u_A=-F\,u.
$$

On the other hand, the power the element absorbs according to its law is the product of the force it transmits and its total elongation rate, $F\,u$. The two are exact negatives of each other, so the family satisfies the organizing principle: the velocity $u$ read by the law is precisely the quantity conjugate to the application points of the wrenches. Without the support moment the endpoint power would, by Section 4.2 of the shared chapter, become $-\mathbf f_I\cdot\dot{\mathbf d}_I=-F\,\mathbf e_j^{\mathsf T}R_{IA}^{\mathsf T}\dot{\mathbf d}_I$, which differs from $-F\,u$ by the power of the support moment, $F\,\boldsymbol\omega_A\cdot\left(\mathbf d_I\times R_{IA}\mathbf e_j\right)$, and the work recorded by the law would no longer equal the work the element does on the two bodies. The support moment is therefore the term that makes the two powers equal, not a correction added merely to balance the net moment.

Splitting the absorbed power by the series conditions,

$$
F\,u=F\dot\delta_k+F\dot\delta_c
=k\,\delta_k\dot\delta_k+c\,\dot\delta_c^{\,2}
=\frac{d}{dt}\left(\frac{F^2}{2k}\right)+\frac{F^2}{c}.
$$

Defining the stored energy and the dissipated power

$$
\mathcal V=\frac{F^2}{2k},
\qquad
\mathcal D=\frac{F^2}{c}=c\,\dot\delta_c^{\,2},
$$

gives

$$
-\mathcal P=\dot{\mathcal V}+\mathcal D.
$$

This is verified directly from the force equation: $\dot{\mathcal V}=F\dot F/k=F\,u-F^2/c$. The stored energy is a function of the state $F$ alone and does not involve the multibody configuration, so the power of the elastic part is an exact time derivative; for $c>0$ the dissipation satisfies $\mathcal D\ge0$ at all times. Integrating over any interval, $\int_{t_0}^{t}(-\mathcal P)\,d\sigma\ge\mathcal V(t)-\mathcal V(t_0)\ge-\mathcal V(t_0)$: the net energy the element can deliver to the two bodies never exceeds the energy $F(t_0)^2/(2k)$ stored in the spring at the initial instant, and the element is passive.

## 3. Computational implementation

### 3.1 Consuming the relative motion

At every evaluation the element first obtains one relative-motion record as in Section 5.1 of the shared chapter. The family consumes only the $j$-th component of $\mathbf u_A$ as $u$, together with the $R_{IA}$, $\mathbf d_I$ and body-fixed points of the two origins needed to write the loads; $\mathbf d_A$, $R_{AC}$ and $\boldsymbol\omega_{rel,A}$ are not involved. The acting axis is a discrete attribute of the element whose three values correspond to the first, second and third axis of A.

### 3.2 Constitutive evaluation and the state derivative

The evaluation receives the current force state $F$, taken from the one component of the $z$ block of the system's continuous state that belongs to this element. In the same evaluation the element writes two things: the state derivative

$$
\dot F=k\,u-\frac{k}{c}\,F,
$$

into the corresponding component of $\dot z$, and the algebraic output $\mathbf f_A=F\,\mathbf e_j$, taken to $\mathbf f_I$ and handed to the load writer. The two depend on different things: the scalar force depends on the state $F$ alone and contains no relative velocity, while the state derivative depends on $u$ and $F$. The relative velocity therefore produces no force directly and influences the force only by changing how $F$ evolves; the scalar law has no algebraic feedthrough from velocity. The spatial wrench still depends algebraically on the configuration: $\mathbf f_I=F\,R_{IA}(q)\mathbf e_j$ turns with the attitude, and the support moment $\mathbf d_I(q)\times\mathbf f_I$ varies with the lever arm. Nor is there an algebraic loop between the two computations: the load does not depend on $\dot F$ and $\dot F$ does not depend on the load; both are computed from the same relative-motion record and the same $F$.

### 3.3 Writing the wrenches

$\mathbf f_I=F\,R_{IA}\mathbf e_j$ and $\mathbf d_I$ are handed to the first path of Section 5.2 of the shared chapter, which writes the two load entries $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf d_I\times\mathbf f_I,\ \mathbf f_I)$ and $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ \mathbf 0,\ -\mathbf f_I)$. This is the same writing path as that of the [three-axis translational spring-damper element](TRANSLATIONAL_SPRING_DAMPER.en.md); the only difference is that $\mathbf f_A$ comes from the state and has a single nonzero component.

### 3.4 Coupling to the system state and the integrator

In the continuous state $[q;v;z]$ of Section 5.1 of the conventions, the $z$ block is the sequence of force states of all series elements, each element occupying exactly one component; a system without series elements has an empty $z$ block. The full state derivative is assembled as

$$
\frac{d}{dt}\begin{bmatrix}q\\v\\z\end{bmatrix}
=\begin{bmatrix}N(q)\,v\\ \dot v(q,v,z)\\ \dot z(q,v,z)\end{bmatrix}
$$

in three steps: the force-element evaluation produces all loads and $\dot z$ at once; the multibody layer moves the loads to the body origins as in Section 3.5 of the shared chapter and solves forward dynamics for $\dot v$; and the system-assembly layer concatenates $[N(q)v;\dot v]$ with $\dot z$. The dependence of $\dot v$ on $z$ comes from $\mathbf f_A=F\,\mathbf e_j$; that of $\dot z$ on $v$ comes from $u$; that of $\dot z$ on $z$ itself is diagonal, with diagonal entry $-1/\tau_{\mathrm r}$ for each element. The integrator advances $F$ as a state component on the same footing as $q$ and $v$, so the initial-value problem needs $F(t_0)$: it cannot be determined algebraically from $(q,v)$ and is part of the initial state. The place of the $z$ block in the individual integration methods is described in Section 1.1 of [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md).

## 4. Mathematical properties and conditions of applicability

- **Parameter domain and existence of the state.** The family is defined for $k>0$ and $c>0$, where the relaxation time $\tau_{\mathrm r}=c/k$ is finite and positive. With $k=0$ the spring transmits no force, $F=k\,\delta_k\equiv0$; with $c=0$ the damper transmits no force at any finite elongation rate, $F=c\,\dot\delta_c\equiv0$. In both cases the transmitted force is pinned to zero algebraically and there is no internal degree of freedom to integrate. The force equation itself fails as well: for $k=0$ it degenerates to $\dot F=0$, decoupled from the motion, whose solution is an arbitrary constant rather than the zero the series conditions demand; for $c=0$ the quotient $k/c$ is undefined, corresponding to instantaneous relaxation with zero relaxation time. An element with either constant at zero is therefore an algebraic element, does not belong to this family and occupies no component of the $z$ block.
- **The two infinite limits are algebraic elements too.** As $k\to\infty$ with $c$ fixed, $\tau_{\mathrm r}\to0$ and, after an initial relaxation layer of duration measured by $\tau_{\mathrm r}$, $F\to c\,u$: a pure viscous damper along one axis, whose force is an algebraic function of $v$. As $c\to\infty$ with $k$ fixed, $\tau_{\mathrm r}\to\infty$ and $\dot F=k\,\dot d_{A,j}$, which integrates to $F(t)=F(t_0)+k\,[d_{A,j}(t)-d_{A,j}(t_0)]$: a pure spring along one axis with the constant force $F(t_0)-k\,d_{A,j}(t_0)$, whose force is an algebraic function of $q$. The two limits are the damper and the spring of the three-axis translational family along one axis; this family is exactly the case in between, $0<\tau_{\mathrm r}<\infty$.
- **Stability and passivity.** The eigenvalue of the state equation is $-1/\tau_{\mathrm r}<0$ and the homogeneous solution decays monotonically; $\mathcal V\ge0$ and $\mathcal D\ge0$ make the element passive. $k<0$ or $c<0$ would make the eigenvalue positive or reverse the sign of the dissipation, so that the element does work actively; such values lie outside the domain of the family.
- **No static stiffness.** When the relative position is held fixed the force decays to zero with $\tau_{\mathrm r}$, so the element carries no static load, and any direction that needs static support must be provided by other elements or constraints. The family is suited to viscous dampers with a compliance in series: it tends to the stiffness $k$ for $\omega\tau_{\mathrm r}\gg1$ and to the damping $c$ for $\omega\tau_{\mathrm r}\ll1$.
- **The force is continuous across velocity jumps.** $F$ is an integral of $u$: it is continuous when $u$ is integrable and continuously differentiable when $u$ is continuous. When the relative velocity jumps, the force of an algebraic damper jumps with it, whereas the force of this family only changes slope.
- **Stiffness of the state equation.** $-1/\tau_{\mathrm r}$ is the diagonal entry $\partial\dot F/\partial F$ of the system Jacobian and the eigenvalue of the isolated force equation, but need not be an eigenvalue of the full coupled system: $F$ acts on $\dot v$ through the load and $v$ acts on $\dot F$ through $u$, so the coupled eigenvalues are in general governed by the equations of motion as well (two elements of equal parameters reading the same $u$, whose force difference obeys $\dot D=-D/\tau_{\mathrm r}$ decoupled from the motion, are an example in which the coupled system does retain this eigenvalue). When $\tau_{\mathrm r}$ is much shorter than the time scales of the multibody motion, the separation of time scales brings a group of fast eigenvalues close to $-1/\tau_{\mathrm r}$ and the system is stiff; the treatment by implicit methods is described in Section 2 of [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md).
- **Single axis, fixed to the reference end.** The acting axis $\mathbf e_j$ turns with body $\mathcal A$, not with the line joining the two origins; the element does not sense relative motion perpendicular to that axis and transmits no force in those directions. Swapping the ends makes the acting axis the $j$-th axis of C, which yields a different element unless $R_{AC}\mathbf e_j=\mathbf e_j$ holds at all times. The support moment $F\,\mathbf d_A\times\mathbf e_j$ grows with the offset of the two origins perpendicular to the acting axis.
- **Linearity and superposition.** The equation is linear and time-invariant in $(u,F)$, the convolution solution of Section 2.3 holds for any integrable input and the responses to several inputs superpose.
- **Physical meaning of the initial value.** Starting from equilibrium ($u\equiv0$ with the force already relaxed), $F(t_0)=0$; starting from the steady state of uniform relative motion, $F(t_0)=c\,u_0$. Any other initial value passes through a transient of duration measured by $\tau_{\mathrm r}$.

## 5. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| Element type: the two ends, the acting axis, $k$ and $c$ | `SeriesSpringViscousDamper`, `ForceElementAxis`, in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| Relative-motion record, from whose $\mathbf u_A$ the $j$-th component is taken as $u$ | `CalcRelativeMotion` within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| State derivative $\dot F=k\,u-(k/c)F$ and algebraic output $\mathbf f_A=F\,\mathbf e_j$ | The series-family branch of `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Endpoint force pair with the reference-end support moment | `EmitTranslationalWrenchPair` within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Evaluation contract: $F$ as input, $\dot F$ as output, one state component per element | `VehicleForcePlan::CalcAppliedForces`, `VehicleForcePlan::series_spring_damper_force_state_count`, in [`vehicle_force_plan.h`](../../../libs/forces/include/orvd/forces/vehicle_force_plan.h) |
| Range of the $z$ block within $[q;v;z]$ | `SystemInstance::series_spring_damper_force_state_range`, in [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| Concatenation of loads and $\dot z$ into the full state derivative | `CompiledSystemPlan::CalcStateTimeDerivatives`, in [`compiled_system_plan.cc`](../../../libs/system_assembly/src/compiled_system_plan.cc) |
| Load entry | `AppliedBodyWrench`, in [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
