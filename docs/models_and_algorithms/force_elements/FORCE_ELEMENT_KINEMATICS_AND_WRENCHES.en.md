[中文](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)

# Force-element kinematics and spatial wrenches

This chapter is the shared foundation of the force-element documents. It defines and derives the relative position, relative attitude, relative translational velocity and relative angular velocity between the two connection frames of an element, states the three ways an element applies spatial wrenches to its two rigid bodies together with the rules for moving their reduction points and proves that under each of the three the total power received by the two bodies equals the corresponding constitutive power. The five constitutive families — the [three-axis translational spring-damper element](TRANSLATIONAL_SPRING_DAMPER.en.md), the [roll spring-damper couple](ROLL_SPRING_DAMPER_COUPLE.en.md), the [series spring-viscous-damper element](SERIES_SPRING_VISCOUS_DAMPER.en.md), the [odd-symmetric saturated piecewise-linear damper](SATURATED_PIECEWISE_LINEAR_DAMPER.en.md) and the [half-angle midpoint RPY bushing](HALF_ANGLE_MIDPOINT_RPY_BUSHING.en.md) — state their deformation measures, velocity inputs and loads in the notation of this chapter, and each proves that it satisfies the organizing principle of Section 4.5. The relative motion is evaluated once by `CalcRelativeMotion`, called from `VehicleForcePlan::CalcAppliedForces` in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc). The type of a load entry is defined in [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h).

## 1. Objects and notation

### 1.1 Two ends, two bodies and three frames

A force element connects two frames: the reference-end frame A and the opposite-end frame C. Each is rigidly fixed to one of two distinct rigid bodies of the multibody model, written $\mathcal A$ and $\mathcal C$ in this chapter. The origin $A_o$ of A is a fixed material point of $\mathcal A$, and the origin $C_o$ of C is a fixed material point of $\mathcal C$; the attitude of a connection frame generally differs from that of its body frame, and its origin need not coincide with the body-frame origin. The inertial frame I is the one of [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md), Section 2.1.

Within this directory the letter C denotes the opposite-end frame of a force element. It is a local reuse of the letter that the wheel-rail contact chain assigns to the contact frame (Section 2.8 of the conventions); the two chains never appear in the same document. The letter B is reserved for the half-angle intermediate frame introduced by the [half-angle midpoint RPY bushing](HALF_ANGLE_MIDPOINT_RPY_BUSHING.en.md); this chapter does not construct it.

The relative-motion record of this chapter is expressed uniformly in the reference-end frame A; on that basis each family chooses its own constitutive coordinates — the translational and roll families state their constants directly in A, the scalar laws of the series and saturated families are stated along one axis of A and read only the velocity component on that axis, and the bushing family places its translational constants in the half-angle intermediate frame B and its rotational constants on a set of angular coordinates. The inertial frame I appears only in derivations and when loads are written out.

### 1.2 Notation

| Symbol | Meaning |
|---|---|
| A, C | Reference-end and opposite-end connection frames |
| $\mathcal A$, $\mathcal C$ | The two rigid bodies carrying A and C |
| I | Inertial frame |
| $\mathbf p_A$, $\mathbf p_C$ | Positions of the origins $A_o$ and $C_o$ in I |
| $\mathbf d_I$, $\mathbf d_A$ | Relative position $\mathbf p_C-\mathbf p_A$, expressed in I and in A |
| $R_{IA}$, $R_{IC}$, $R_{AC}$ | Attitudes of A and C in I, and the relative attitude of C in A |
| $\mathbf v_{Ao}$, $\mathbf v_{Co}$ | Velocities of the origins $A_o$ and $C_o$ relative to I, expressed in I |
| $\boldsymbol\omega_A$, $\boldsymbol\omega_C$ | Angular velocities of the bodies $\mathcal A$ and $\mathcal C$ relative to I, expressed in I |
| $\mathbf u_A$, $\mathbf u_I$ | Velocity of the opposite-end origin relative to the reference end, expressed in A and in I |
| $\boldsymbol\omega_{rel,A}$, $\boldsymbol\omega_{rel,I}$ | Angular velocity of C relative to A, expressed in A and in I |
| $\mathbf x_m$ | Instantaneous world midpoint of the two origins |
| $\mathbf u_{m,I}$, $\mathbf u_{m,A}$ | Relative velocity of the two bodies' material points at $\mathbf x_m$, expressed in I and in A |
| $\mathbf v^{(\mathcal A)}(\mathbf x)$ | Velocity of the material point of body $\mathcal A$ that coincides with the spatial point $\mathbf x$ |
| $\mathbf f$, $\boldsymbol\tau$, $\mathbf m$ | Force, moment and pure couple acting on the reference end; the expressed-in frame is the last subscript |
| $\mathcal W_Q^E[\mathcal A]$ | Spatial wrench acting on body $\mathcal A$, reduced about point Q, expressed in E |
| $\mathbf x_{\mathcal A}$, $R_{I\mathcal A}$ | Position and attitude in I of the body-frame origin of $\mathcal A$ |
| $\mathbf r_Q^{\mathcal A}$ | Coordinates of point Q in the body frame of $\mathcal A$ |
| $\mathcal P$ | Total power the element delivers to the two bodies |
| $\mathcal V$, $\mathcal D$ | Stored-energy function and dissipation rate of an element; each family chapter defines them in its own variables, with subscripts where several parts must be told apart |
| $\mathbf a\circ\mathbf b$ | Componentwise product of two vectors |
| $\operatorname{skew}(\mathbf w)$ | Skew-symmetric matrix with $\operatorname{skew}(\mathbf w)\mathbf x=\mathbf w\times\mathbf x$ |

### 1.3 Relation to the shared conventions

This chapter follows the rotation-matrix rule of Section 4.1 of the conventions: $R_{AB}$ maps components in B to components in A and satisfies $R_{AC}=R_{AB}R_{BC}$, and entries $R_{ij}$ are numbered from 1. The long-subscript forms of the conventions correspond to the abbreviations used here as $\mathbf v_{Ao}=\mathbf v_{IAo\_I}$, $\boldsymbol\omega_A=\boldsymbol\omega_{IA\_I}$, $\mathbf u_A=\mathbf v_{ACo\_A}$ and $\boldsymbol\omega_{rel,A}=\boldsymbol\omega_{AC\_A}$. Only these three frames occur in the shared kinematics, the bushing family adding the half-angle intermediate frame B; $\mathbf u$ and $\boldsymbol\omega_{rel}$ are always measured relative to A, and their subscript names the expressed-in frame alone.

Wrenches follow $\mathcal W_Q^E$ of Section 5.2 of the conventions and its reduction-point rule $\boldsymbol\tau_O^E=\boldsymbol\tau_Q^E+\mathbf p_{OQ}^E\times\mathbf f^E$, with two extensions declared here. To name the loaded body, the wrench symbol carries it in brackets: $\mathcal W_Q^E[\mathcal A]$ is the wrench acting on body $\mathcal A$. When the components of a wrench are written out, the expressed-in frame of each component vector is given as its last subscript, as in $\mathbf f_I$, following Section 4.1 of the conventions rather than repeating the superscript.

The following symbols carry a meaning specific to this directory and are distinguished from their use elsewhere: the bold $\mathbf d$ is the relative position vector of the two origins, not the scalar penetration that the wheel-rail contact chain has released; $\mathbf u$ is a relative velocity and is unrelated to the Euclidean displacement $u$ of Section 1.2 of [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md); power is written $\mathcal P$ to keep clear of the wheel-side application point `P`. Element-level stiffness and damping are lowercase $k$ and $c$, or their vector forms $\mathbf k$ and $\mathbf c$, as required by Section 5.3 of the conventions.

An unmarked $\mathbf f$, $\boldsymbol\tau$ or $\mathbf m$ always denotes the load on the reference end. The forces on the two ends are negatives of each other, so are the pure couples, and so are the two wrenches once reduced about one common point; the moments written about each end's own application point are not — in the endpoint force pair the reference end additionally carries the support moment $\mathbf d_I\times\mathbf f_I$ while the opposite end carries none (Section 3.2). When a family needs to state its law in terms of the load on the opposite end, the loaded end is written as a first subscript and the expressed-in frame as a second one: $\mathbf f_{C,B}$ is the force on end C expressed in B.

## 2. Relative motion

### 2.1 Relative position and relative attitude

The relative position of the two origins, in I and in A, is

$$
\mathbf d_I=\mathbf p_C-\mathbf p_A,
\qquad
\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I.
$$

The relative attitude of the opposite end in the reference end is

$$
R_{AC}=R_{IA}^{\mathsf T}R_{IC}.
$$

$\mathbf d_A$ is the common source of the translational deformation measures: it is the difference of the two origins and carries no natural length or zero-displacement vector, and $\mathbf d_A=\mathbf 0$ exactly when the two origins coincide. The translational family takes it directly as its deformation measure, the bushing family uses its representation in B, and the scalar laws of the series and saturated families read no displacement at all, using it only in the support moment $\mathbf d_I\times\mathbf f_I$. $R_{AC}$ is the sole source of every rotational deformation measure; whether a family reads a matrix entry or some angular coordinate from it is the family's own definition.

### 2.2 Transporting velocity between points of a rigid body

Let body $\mathcal A$ have angular velocity $\boldsymbol\omega_A$, and let its material point $A_o$ have velocity $\mathbf v_{Ao}$. Any material point of the body, and the material point of its rigid extension that coincides with an arbitrary spatial point $\mathbf x$, has velocity

$$
\mathbf v^{(\mathcal A)}(\mathbf x)=\mathbf v_{Ao}+\boldsymbol\omega_A\times\left(\mathbf x-\mathbf p_A\right).
$$

Likewise for $\mathcal C$, $\mathbf v^{(\mathcal C)}(\mathbf x)=\mathbf v_{Co}+\boldsymbol\omega_C\times\left(\mathbf x-\mathbf p_C\right)$. This transport formula is the basic tool of the chapter: Section 2.3 uses it to interpret the transport term, Section 2.5 to construct the midpoint velocities and Section 4 to compute the power of a wrench.

### 2.3 Relative velocity with its transport term

The attitude matrix evolves as $\dot R_{IA}=\operatorname{skew}(\boldsymbol\omega_A)R_{IA}$, hence $\dot R_{IA}^{\mathsf T}=-R_{IA}^{\mathsf T}\operatorname{skew}(\boldsymbol\omega_A)$. Differentiating $\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I$ gives

$$
\dot{\mathbf d}_A
=R_{IA}^{\mathsf T}\dot{\mathbf d}_I-R_{IA}^{\mathsf T}\left(\boldsymbol\omega_A\times\mathbf d_I\right)
=R_{IA}^{\mathsf T}\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right).
$$

This chapter defines that quantity as the velocity of the opposite-end origin relative to the reference end:

$$
\mathbf u_I=\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I,
\qquad
\mathbf u_A=R_{IA}^{\mathsf T}\mathbf u_I=\dot{\mathbf d}_A.
$$

$-\boldsymbol\omega_A\times\mathbf d_I$ is the transport term. By Section 2.2, $\mathbf v_{Ao}+\boldsymbol\omega_A\times\mathbf d_I=\mathbf v^{(\mathcal A)}(\mathbf p_C)$ is the velocity of the material point of body $\mathcal A$ that momentarily coincides with $C_o$, so $\mathbf u_I$ is the velocity of $C_o$ relative to the body $\mathcal A$, not relative to the point $A_o$. This is exactly the contract of the multibody layer's relative spatial-velocity query: the velocity of the reference frame is first shifted to the measured origin and the difference is taken afterwards. $R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ and $\mathbf u_A$ differ only in the components perpendicular to $\mathbf d$, because $\mathbf d_I\cdot(\boldsymbol\omega_A\times\mathbf d_I)=0$; along the line joining the two origins they agree.

### 2.4 Relative angular velocity and the derivative of the relative attitude

The relative angular velocity is the difference of the two bodies' angular velocities:

$$
\boldsymbol\omega_{rel,I}=\boldsymbol\omega_C-\boldsymbol\omega_A,
\qquad
\boldsymbol\omega_{rel,A}=R_{IA}^{\mathsf T}\boldsymbol\omega_{rel,I}.
$$

It is tied to the derivative of the relative attitude by

$$
\dot R_{AC}=\operatorname{skew}(\boldsymbol\omega_{rel,A})\,R_{AC}
$$

The derivation: $\dot R_{AC}=\dot R_{IA}^{\mathsf T}R_{IC}+R_{IA}^{\mathsf T}\dot R_{IC}=R_{IA}^{\mathsf T}\operatorname{skew}(\boldsymbol\omega_C-\boldsymbol\omega_A)R_{IC}$, and the identity $R^{\mathsf T}\operatorname{skew}(\mathbf w)R=\operatorname{skew}(R^{\mathsf T}\mathbf w)$ together with $R_{IC}=R_{IA}R_{AC}$ gives the result. The time derivative of any rotational coordinate defined from $R_{AC}$ starts from this equation; the derivative of an angular coordinate is in general not a component of $\boldsymbol\omega_{rel,A}$, and the map between the two is supplied by the family that adopts that coordinate.

### 2.5 Relative material velocity at the instantaneous midpoint

The instantaneous world midpoint of the two origins is

$$
\mathbf x_m=\mathbf p_A+\tfrac12\mathbf d_I=\tfrac12\left(\mathbf p_A+\mathbf p_C\right).
$$

It is not a fixed material point of either body; at each instant, each body has one material point coinciding with it. By the transport formula of Section 2.2, those two material points move with $\mathbf v^{(\mathcal A)}(\mathbf x_m)=\mathbf v_{Ao}+\tfrac12\boldsymbol\omega_A\times\mathbf d_I$ and $\mathbf v^{(\mathcal C)}(\mathbf x_m)=\mathbf v_{Co}-\tfrac12\boldsymbol\omega_C\times\mathbf d_I$, and their difference is

$$
\mathbf u_{m,I}
=\mathbf v^{(\mathcal C)}(\mathbf x_m)-\mathbf v^{(\mathcal A)}(\mathbf x_m)
=\mathbf v_{Co}-\mathbf v_{Ao}-\tfrac12\left(\boldsymbol\omega_A+\boldsymbol\omega_C\right)\times\mathbf d_I.
$$

Subtracting the $\mathbf u_I$ of Section 2.3 and expressing the result in A,

$$
\mathbf u_{m,A}=\mathbf u_A-\tfrac12\,\boldsymbol\omega_{rel,A}\times\mathbf d_A.
$$

$\mathbf u_{m,A}$ is the relative material velocity of the two bodies at their common midpoint. In general it is not the time derivative of any displacement; it coincides with $\dot{\mathbf d}_A$ only when $\boldsymbol\omega_{rel,A}\times\mathbf d_A=\mathbf 0$.

## 3. Applying wrenches and moving their reduction points

### 3.1 Load entries and body-fixed points

Every load an element hands to the multibody layer is a spatial wrench on one rigid body: it names the body, the body coordinates of a body-fixed point Q, the expressed-in frame E and the moment about Q together with the force, that is, $\mathcal W_Q^E[\mathcal A]$. All three application schemes of this directory are expressed in I, and every element writes exactly two entries, one for $\mathcal A$ and one for $\mathcal C$. The application point Q is always delivered as a body-fixed point; a point $\mathbf x$ of world space has, on body $\mathcal A$, the body coordinates

$$
\mathbf r_Q^{\mathcal A}=R_{I\mathcal A}^{\mathsf T}\left(\mathbf x-\mathbf x_{\mathcal A}\right).
$$

The body coordinates of the two origins $A_o$ and $C_o$ are constants; the midpoint of Section 3.4 is converted anew at every evaluation.

### 3.2 Endpoint force pair with the reference-end support moment

Let $\mathbf f_I$ be the force on the reference end. The first scheme applies equal and opposite forces at the two origins and adds a support moment at the reference end:

$$
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(\mathbf 0,\ -\mathbf f_I\right).
$$

The support moment $\mathbf d_I\times\mathbf f_I$ is taken about $A_o$ and applied to body $\mathcal A$. The resultant force of the pair is zero, and its resultant moment about an arbitrary point O is $(\mathbf p_A-\mathbf x_O)\times\mathbf f_I+\mathbf d_I\times\mathbf f_I-(\mathbf p_C-\mathbf x_O)\times\mathbf f_I=\mathbf 0$, so the paired wrench exerts no net force and no net moment on the system. Without the support moment the two endpoint forces leave a net couple $-\mathbf d_I\times\mathbf f_I$ whenever $\mathbf f_I$ is not parallel to $\mathbf d_I$; balance, however, is only a consequence of the support moment, and its necessity follows from the power identity of Section 4.2.

A family supplies $\mathbf f_A$ in A; it is taken to I as $\mathbf f_I=R_{IA}\mathbf f_A$ before it is written.

### 3.3 Pure couple pair

Let $\mathbf m_I$ be the pure couple on the reference end. The second scheme applies equal and opposite pure couples at the two origins, with no force:

$$
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf m_I,\ \mathbf 0\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(-\mathbf m_I,\ \mathbf 0\right).
$$

A pure couple does not depend on the reduction point, so moving the point leaves it unchanged; the load entries still record the two origins as formal application points. Resultant force and resultant moment are both zero. A family supplies $\mathbf m_A$ in A, taken to I as $\mathbf m_I=R_{IA}\mathbf m_A$.

### 3.4 Wrench pair applied at the instantaneous midpoint

Let $\mathbf f_I$ and $\boldsymbol\tau_I$ be the force and moment on the reference end. The third scheme applies both wrenches at the instantaneous world midpoint $\mathbf x_m$ of Section 2.5 and adds no support moment:

$$
\mathcal W_{m}^{I}[\mathcal A]=\left(\boldsymbol\tau_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{m}^{I}[\mathcal C]=\left(-\boldsymbol\tau_I,\ -\mathbf f_I\right).
$$

The two wrenches are reduced about the same point and expressed in the same frame, and they are exact opposites, so resultant force and moment vanish. Converted to the two bodies, $\mathbf x_m$ has body coordinates $\mathbf r_m^{\mathcal A}=R_{I\mathcal A}^{\mathsf T}(\mathbf x_m-\mathbf x_{\mathcal A})$ and $\mathbf r_m^{\mathcal C}=R_{I\mathcal C}^{\mathsf T}(\mathbf x_m-\mathbf x_{\mathcal C})$; neither point is a connection-frame origin, and both change with the relative motion. A family that uses this scheme and states its law in terms of the load on the opposite end, $\mathbf f_{C,I}=-\mathbf f_I$ and $\boldsymbol\tau_{C,I}=-\boldsymbol\tau_I$, merely swaps the labels of the two wrenches above; the application point is unchanged.

### 3.5 Reduction to the body origin

The multibody layer moves each load from its application point Q to the body-frame origin $O_{\mathcal A}$ of the loaded body. By Section 5.2 of the conventions, $\mathbf p_{OQ}$ points from the new reduction point to the old one, so for a point Q of body $\mathcal A$,

$$
\boldsymbol\tau_{O_{\mathcal A}}=\boldsymbol\tau_Q+\left(R_{I\mathcal A}\mathbf r_Q^{\mathcal A}\right)\times\mathbf f,
$$

while the force itself is unchanged. The three schemes differ precisely at this step: the support moment of the endpoint pair is added to the lever-arm term $(R_{I\mathcal A}\mathbf r_{A_o}^{\mathcal A})\times\mathbf f_I$; a pure couple produces no lever-arm term; and the lever arms of the midpoint pair run from the two body origins to one and the same spatial point. Moving the reduction point and changing the expressed-in frame are distinct operations: every reduction-point move in this chapter is carried out in I, and the expressed-in frame changes only between A and I, and, in the bushing family, between B and A.

## 4. Power identities and the organizing principle

### 4.1 Power of a wrench on one rigid body

A wrench $(\boldsymbol\tau_Q,\mathbf f)$ acting on body $\mathcal A$ and reduced about its material point Q delivers to that body the power

$$
\mathcal P=\mathbf f\cdot\mathbf v^{(\mathcal A)}(\mathbf x_Q)+\boldsymbol\tau_Q\cdot\boldsymbol\omega_A.
$$

It does not depend on the reduction point: replacing Q by O, the two terms produced by $\mathbf v^{(\mathcal A)}(\mathbf x_Q)=\mathbf v^{(\mathcal A)}(\mathbf x_O)+\boldsymbol\omega_A\times\mathbf p_{OQ}$ and $\boldsymbol\tau_O=\boldsymbol\tau_Q+\mathbf p_{OQ}\times\mathbf f$, namely $\mathbf f\cdot(\boldsymbol\omega_A\times\mathbf p_{OQ})$ and $(\mathbf p_{OQ}\times\mathbf f)\cdot\boldsymbol\omega_A$, are equal and cancel. Nor does it depend on the expressed-in frame, because dot products are invariant under rotation; the powers below are therefore derived in I and stated in A. The derivations repeatedly use the scalar triple-product identity

$$
\boldsymbol\omega\cdot\left(\mathbf d\times\mathbf f\right)=\mathbf f\cdot\left(\boldsymbol\omega\times\mathbf d\right).
$$

### 4.2 Endpoint force pair

For the pair of loads of Section 3.2, the total power delivered to the two bodies is

$$
\mathcal P
=\mathbf f_I\cdot\mathbf v_{Ao}+\boldsymbol\omega_A\cdot\left(\mathbf d_I\times\mathbf f_I\right)-\mathbf f_I\cdot\mathbf v_{Co}
=-\mathbf f_I\cdot\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right)
=-\mathbf f_A\cdot\mathbf u_A.
$$

The second equality uses the triple-product identity, the third the definition of Section 2.3 and the rotational invariance of the dot product. The endpoint power is thus exactly the negative inner product of the reference-end force with $\dot{\mathbf d}_A$. Without the support moment the total power becomes $-\mathbf f_I\cdot(\mathbf v_{Co}-\mathbf v_{Ao})=-\mathbf f_I\cdot\dot{\mathbf d}_I$, which is not conjugate to a law stated in A with $\mathbf u_A$ as its velocity input; the two differ by the power of the support moment, $\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)$. The support moment is therefore the term that makes the endpoint power equal the constitutive power; balancing the net moment is a by-product.

This identity establishes only the pairing of powers: for a law with $\mathbf u_A=\dot{\mathbf d}_A$ as velocity input, its power $-\mathbf f_A\cdot\dot{\mathbf d}_A$ is exactly the power received by the two bodies. Whether the elastic part is the exact time derivative of a stored energy is a separate matter: it requires $\mathbf f_{\mathrm{el}}(\mathbf d_A)$ to be the gradient of a scalar function, that is, to satisfy the integrability condition $\partial f_{\mathrm{el},i}/\partial d_{A,j}=\partial f_{\mathrm{el},j}/\partial d_{A,i}$; a law such as $\mathbf f_{\mathrm{el}}=k\,(d_{A,2},0,0)^{\mathsf T}$ has the same power pairing and no potential. The three families that use this scheme argue their stored energy and dissipation in their own chapters, by a potential gradient or by an energy identity for the internal variable.

### 4.3 Pure couple pair

For the pair of pure couples of Section 3.3,

$$
\mathcal P
=\mathbf m_I\cdot\boldsymbol\omega_A-\mathbf m_I\cdot\boldsymbol\omega_C
=-\mathbf m_I\cdot\boldsymbol\omega_{rel,I}
=-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}.
$$

The power of a pure couple does not involve an application point; the physical couple is conjugate to the relative angular velocity. Through the rate map $\dot{\boldsymbol\eta}=H\boldsymbol\omega_{rel,A}$ of a set of angular coordinates $\boldsymbol\eta$, the same power can equally be written as the inner product of the generalized moment on the opposite end with the generalized rates, $\mathcal P=\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}$, where $\boldsymbol\tau_\eta$ is the generalized moment acting on end C and the physical couple on the reference end is $\mathbf m_A=-H^{\mathsf T}\boldsymbol\tau_\eta$ — the same pairing in different variables, which is how the rotational part of the bushing family is stated. If a family's couple acts only along the first axis of A, $\mathbf m_A=m\,\mathbf e_1$, then $\mathcal P=-m\,\omega_{rel,A,1}$: the conjugate rate is the first component of $\boldsymbol\omega_{rel,A}$, and that is the component the roll family takes as its damping input.

### 4.4 Midpoint wrench pair

For the pair of wrenches of Section 3.4, with the material-point velocities at $\mathbf x_m$ taken from Section 2.5,

$$
\mathcal P
=\mathbf f_I\cdot\mathbf v^{(\mathcal A)}(\mathbf x_m)+\boldsymbol\tau_I\cdot\boldsymbol\omega_A
-\mathbf f_I\cdot\mathbf v^{(\mathcal C)}(\mathbf x_m)-\boldsymbol\tau_I\cdot\boldsymbol\omega_C
=-\mathbf f_I\cdot\mathbf u_{m,I}-\boldsymbol\tau_I\cdot\boldsymbol\omega_{rel,I}.
$$

In A this reads $\mathcal P=-\mathbf f_A\cdot\mathbf u_{m,A}-\boldsymbol\tau_A\cdot\boldsymbol\omega_{rel,A}$, and in terms of the load on the opposite end $\mathcal P=\mathbf f_{C,A}\cdot\mathbf u_{m,A}+\boldsymbol\tau_{C,A}\cdot\boldsymbol\omega_{rel,A}$. The velocity conjugate to the force is the midpoint material relative velocity $\mathbf u_m$, not $\mathbf u_A$. Applying the same force at the two origins without a support moment changes the power to $-\mathbf f_I\cdot\dot{\mathbf d}_I$; applying it at the two origins with the support moment changes it to $-\mathbf f_A\cdot\mathbf u_A$. The differences between any two of the three are inner products of $\mathbf f$ with vectors of the form $\tfrac12\boldsymbol\omega_{rel}\times\mathbf d$ and $\boldsymbol\omega_A\times\mathbf d$; they vanish when the force is along the line joining the two origins and not in general.

### 4.5 Organizing principle: velocity input and application point are chosen as a pair

The three identities share one structure: the endpoint power is the negative inner product of the load on the reference end with some velocity of the opposite end relative to the reference end, and that velocity is fixed by the complete assignment of the wrenches, that is, by where the force acts together with which moment accompanies it — $\mathbf u_A$ when the forces act at the two origins with a support moment, $\boldsymbol\omega_{rel,A}$ for a pure couple, and $\mathbf u_{m,A}$ together with $\boldsymbol\omega_{rel,A}$ when both wrenches act at the instantaneous midpoint. This directory therefore adopts one organizing principle: **every family chooses the velocity input of its constitutive law and the application point of its wrenches as a pair, so that the constitutive power equals exactly the endpoint power received by the two bodies.** Moving a force to another point while adding the corresponding moment by the reduction rule of Section 3.5 leaves the power unchanged; a family that changed the application point alone, without the compensating moment and without changing its velocity input, or the reverse, would in general break the power identity (special cases such as moving the force along its own line of action excepted): the work the element does on the two bodies would no longer be the work its law accounts for, and any bookkeeping of stored energy and dissipation would lose its meaning. The three pairings are:

| Application scheme | Velocity input | Endpoint power |
|---|---|---|
| Force pair at the two origins with the reference-end support moment | $\mathbf u_A=\dot{\mathbf d}_A$ | $-\mathbf f_A\cdot\mathbf u_A$ |
| Pure couple pair at the two ends | $\boldsymbol\omega_{rel,A}$ | $-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}$ |
| Wrench pair at the instantaneous midpoint | $\mathbf u_{m,A}$ and $\boldsymbol\omega_{rel,A}$ | $-\mathbf f_A\cdot\mathbf u_{m,A}-\boldsymbol\tau_A\cdot\boldsymbol\omega_{rel,A}$ |

Each family chapter cites this section, states which row it uses and proves that its velocity input is indeed the quantity in that row. This chapter guarantees only the equality of powers; whether the elastic part is the exact derivative of some stored-energy function depends on two further things — whether the family's deformation measure and velocity input are derivatives of one another, and whether the elastic law is integrable in that deformation measure (Section 4.2); a family with an internal variable must also account for the energy stored in it. Each family argues these for itself.

## 5. Computational implementation

### 5.1 Evaluating the relative motion once

At every evaluation, each element first obtains one relative-motion record from its two frames: $\mathbf d_A$, $R_{AC}$, $\mathbf u_A$ and $\boldsymbol\omega_{rel,A}$, together with the $\mathbf d_I$, $R_{IA}$, $\mathbf p_A$ and the body-fixed points of the two origins that writing the loads requires. The multibody layer supplies three kinds of query: the pose of a frame in I gives $(R_{IA},\mathbf p_A)$ and $(R_{IC},\mathbf p_C)$; the spatial velocity of C relative to A expressed in A gives $\boldsymbol\omega_{rel,A}$ and $\mathbf u_A$ directly, its translational component being by contract the velocity of $C_o$ measured in A, transport term included; and the resolution of a frame origin to a body-fixed point gives $(\mathcal A,\mathbf r_{A_o}^{\mathcal A})$ and $(\mathcal C,\mathbf r_{C_o}^{\mathcal C})$. $\mathbf d_I$, $\mathbf d_A$ and $R_{AC}$ follow from the two poses by subtraction and multiplication as in Section 2.1. This step is identical for the five families; they differ only in how they consume the record.

### 5.2 Writing the three wrench pairs

Every element writes exactly two load entries, both expressed in I.

- Endpoint pair with support moment: the caller supplies $\mathbf f_I=R_{IA}\mathbf f_A$ and $\mathbf d_I$; the entries written are $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf d_I\times\mathbf f_I,\ \mathbf f_I)$ and $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ \mathbf 0,\ -\mathbf f_I)$. The translational, series and saturated families use this path; for the latter two, $\mathbf f_A$ has a single nonzero component along the stated axis.
- Pure couple pair: the caller supplies $\mathbf m_I=R_{IA}\mathbf m_A$; the entries written are $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf m_I,\ \mathbf 0)$ and $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ -\mathbf m_I,\ \mathbf 0)$. The roll family uses this path.
- Midpoint pair: $\mathbf r_m^{\mathcal A}$ and $\mathbf r_m^{\mathcal C}$ are formed from $\mathbf x_m=\mathbf p_A+\tfrac12\mathbf d_I$ and the world poses of the two bodies; with the load on the opposite end, $\mathbf f_{C,I}$ and $\boldsymbol\tau_{C,I}$, as input, the entries written are $(\mathcal A,\ \mathbf r_m^{\mathcal A},\ -\boldsymbol\tau_{C,I},\ -\mathbf f_{C,I})$ and $(\mathcal C,\ \mathbf r_m^{\mathcal C},\ \boldsymbol\tau_{C,I},\ \mathbf f_{C,I})$. The bushing family uses this path.

### 5.3 Consumption by the multibody layer and assembly of the state derivative

The multibody layer reads the entries one by one, moves each moment about Q to the origin of its body as in Section 3.5 and adds the result to the applied forces of forward dynamics, which yields $\dot v$. A family that carries internal state writes its state derivative in the same evaluation: the force state of the series family forms the $z$ block of Section 5.1 of the conventions, its derivative and the two load entries are computed from the same relative-motion record, and the system-assembly layer concatenates $[N(q)v;\dot v]$ with $\dot z$ into the full state derivative. The place of the $z$ block in the integrators is described in Section 1.1 of [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md). The kinematic layer itself stores no history: $\mathbf d_A$, $R_{AC}$, $\mathbf u_A$ and $\boldsymbol\omega_{rel,A}$ are determined entirely by the current $(q,v)$.

## 6. Mathematical properties and conditions of applicability

- **No natural length.** The translational deformation measure is the difference of origins $\mathbf d_A$; the kinematic layer provides no natural length, zero-displacement vector or installed configuration. The force-free configuration of a family is expressed by its own law, for instance through a constant force term; the two origins coincide if and only if $\mathbf d_A=\mathbf 0$.
- **The two ends must lie on different bodies.** With both ends on one body, $\mathbf d_A$ and $R_{AC}$ are constant while $\mathbf u_A=\mathbf 0$ and $\boldsymbol\omega_{rel,A}=\mathbf 0$, and each of the three paired wrenches, landing on a single body, has zero resultant force and moment: such an element has no effect on the motion. Nor may an end be the inertial frame itself, since there is no rigid body to receive the wrench.
- **The choice of reference end is part of the law.** This item concerns the translational laws stated in A and applied as an endpoint force pair; it does not cover the bushing family, which applies its loads at the midpoint and whose translational part is invariant under renaming the ends, see Section 4.2 of the [half-angle midpoint RPY bushing](HALF_ANGLE_MIDPOINT_RPY_BUSHING.en.md). For the former, the relative motion is expressed in A, the constitutive constants are stated in A and the support moment falls on end A. Swapping the ends reverses the relative position vector, expresses it in C instead and transposes $R_{AC}$; a stiffness or damping that is diagonal in A is in general no longer diagonal in C, and an isotropic coefficient matrix is unaffected by this change of representation; but the transport term $-\boldsymbol\omega_A\times\mathbf d_I$ contains the angular velocity of the reference body alone and becomes $-\boldsymbol\omega_C\times\mathbf d_I'$ after the swap, so a damping law with $\mathbf u_A$ as input changes under the swap even when isotropic, and among these laws only a central elastic force along the line of centers is swap-invariant (Section 2.8 of the [three-axis translational spring-damper element](TRANSLATIONAL_SPRING_DAMPER.en.md)). A reference-end frame of different attitude on the same body likewise yields a different element.
- **The transport term affects only the components perpendicular to the line of centers.** $\mathbf u_A$ and $R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ agree along $\mathbf d$ and differ transversely by the representation of $\boldsymbol\omega_A\times\mathbf d$ in A. A one-dimensional element acting only along that line does not distinguish the two; the three-axis diagonal laws and the single-axis laws of this directory all take $\mathbf u_A$ as their velocity input, and the two in general differ when the axis is not along the line of centers (they still agree when $\boldsymbol\omega_A=\mathbf 0$ or $\boldsymbol\omega_A\parallel\mathbf d$).
- **Relative angular velocity and angular coordinates are not interchangeable.** $\boldsymbol\omega_{rel,A}$ is a geometric quantity independent of any choice of angular coordinates; the derivative of any angular coordinate extracted from $R_{AC}$ is related to $\boldsymbol\omega_{rel,A}$ by a configuration-dependent map with singularities of its own. A family whose rate input is a component of $\boldsymbol\omega_{rel,A}$ has no such singularity.
- **The midpoint velocity is not the derivative of a displacement.** $\mathbf u_{m,A}$ differs from $\dot{\mathbf d}_A$ by $\tfrac12\boldsymbol\omega_{rel,A}\times\mathbf d_A$. For a family with $\mathbf u_m$ as velocity input and some representation of $\mathbf d$ as deformation measure, the elastic power is in general not the exact derivative of a stored energy; the power identity still holds, but energy bookkeeping requires additional conditions.
- **Power is independent of the expressed-in frame.** Every power expression of Section 4 takes the same value in I, in A or in any intermediate frame, so each family may state its power in whichever frame is most convenient.

## 7. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| The two ends and the five constitutive family types | `ForceElementEnd`, `VehicleForceElementCollection`, in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| One-shot evaluation of the relative motion $\mathbf d_A$, $R_{AC}$, $\mathbf u_A$, $\boldsymbol\omega_{rel,A}$ | `CalcRelativeMotion` within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Contract of the relative spatial velocity, whose translational component is the velocity of $C_o$ measured in A | `MultibodyModel::CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame`, in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| World pose of a frame and resolution of its origin to a body-fixed point | `MultibodyModel::CalcPoseInWorld`, `MultibodyModel::CalcFrameOriginAsBodyFixedPoint`, in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Load entry and body-fixed point | `AppliedBodyWrench`, `BodyFixedPoint`, in [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
| Endpoint force pair with the reference-end support moment | `EmitTranslationalWrenchPair` within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Pure couple pair | `internal::EmitCoupleWrenchPair`, in [`body_wrench_pair.h`](../../../libs/forces/src/body_wrench_pair.h) |
| Wrench pair at the instantaneous midpoint | `EmitMidpointBushingWrenchPair` within `VehicleForcePlan::CalcAppliedForces`, in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| Reduction of loads to the body origin and entry into forward dynamics | `MultibodyModel::CalcStateTimeDerivatives`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| Concatenation of loads and $\dot z$ into the full state derivative | `CompiledSystemPlan::CalcStateTimeDerivatives`, in [`compiled_system_plan.cc`](../../../libs/system_assembly/src/compiled_system_plan.cc) |
