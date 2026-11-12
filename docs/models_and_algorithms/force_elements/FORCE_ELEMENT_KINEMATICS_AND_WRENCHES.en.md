[中文](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)

# Force-element kinematics and spatial wrenches

This chapter derives the relative motion of two attachment frames, paired loads and their power relations. Each force element defines its deformation measures, velocity inputs and constitutive law on this basis.

## 1. Objects and notation

### 1.1 Two ends, two bodies and three frames

The reference frame A and opposite frame C are fixed to bodies $\mathcal A$ and $\mathcal C$, with attachment points $A_o$ and $C_o$ as their origins. An attachment frame may have its own position and attitude relative to its body frame. I is inertial; $R_{IA}$ maps components from A to I.

A constitutive law may act along reference-frame axes or along the instantaneous line between the attachments. Each element states its choice.

### 1.2 Notation

| Symbol | Meaning |
|---|---|
| $\mathbf p_A,\mathbf p_C$ | Attachment positions in I |
| $\mathbf d_I,\mathbf d_A$ | Relative position expressed in I and A |
| $R_{IA},R_{IC},R_{AC}$ | End attitudes and relative attitude |
| $\mathbf v_{Ao},\mathbf v_{Co}$ | Attachment velocities relative to I, expressed in I |
| $\boldsymbol\omega_A,\boldsymbol\omega_C$ | Body angular velocities relative to I, expressed in I |
| $\mathbf u_A,\mathbf u_I$ | Opposite-origin velocity relative to the reference frame, expressed in A and I |
| $\boldsymbol\omega_{rel,A},\boldsymbol\omega_{rel,I}$ | Physical angular velocity of C relative to A |
| $l,\mathbf e_I,\mathbf e_A$ | Attachment separation and unit direction from A to C |
| $\mathbf f,\mathbf m$ | Force and pure couple on the reference end; final subscript denotes expression frame |
| $\mathcal W_Q^E[\mathcal A]$ | Wrench on body $\mathcal A$, reduced at Q and expressed in E |
| $\mathbf x_{\mathcal A},R_{I\mathcal A},\mathbf r_Q^{\mathcal A}$ | Body origin, attitude and body coordinates of Q |
| $\mathbf v^{(\mathcal A)}(\mathbf x)$ | Material velocity of body $\mathcal A$ at spatial point $\mathbf x$ |
| $\mathcal P,\mathcal V,\mathcal D$ | Total power delivered to the bodies, element storage and dissipation rate |

### 1.3 Frames and signs

Rotations, wrenches and SI units follow [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md). Define $\operatorname{skew}(\mathbf w)\mathbf x=\mathbf w\times\mathbf x$; $\mathbf a\circ\mathbf b$ denotes componentwise multiplication. Scalar element stiffness and damping are $k,c$; matrix and vector coefficients are defined in the respective chapters.

The long-form equivalents are $\mathbf v_{Ao}=\mathbf v_{IAo\_I}$, $\boldsymbol\omega_A=\boldsymbol\omega_{IA\_I}$, $\mathbf u_A=\mathbf v_{ACo\_A}$ and $\boldsymbol\omega_{rel,A}=\boldsymbol\omega_{AC\_A}$. C denotes the opposite attachment frame here. Wrench entries are ordered as moment and force; at a common reduction point the paired wrenches are opposites.

## 2. Relative motion

### 2.1 Relative position and attitude

$$
\mathbf d_I=\mathbf p_C-\mathbf p_A,\qquad
\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I,\qquad
R_{AC}=R_{IA}^{\mathsf T}R_{IC}.
$$

Translational elasticity and the reference-frame bushing use $\mathbf d_A$ as their deformation measure; point-to-point elements also use its magnitude and direction. The roll couple takes its rotational measure from $R_{AC}$.

### 2.2 Transport of material-point velocity

The material velocity of body $\mathcal A$ at spatial point $\mathbf x$ is

$$
\mathbf v^{(\mathcal A)}(\mathbf x)
=\mathbf v_{Ao}+\boldsymbol\omega_A\times(\mathbf x-\mathbf p_A).
$$

This relation also applies to a rigid extension of the body. The corresponding positions and angular velocity give the relation for $\mathcal C$.

### 2.3 Relative velocity with its transport term

Using $\dot R_{IA}=\operatorname{skew}(\boldsymbol\omega_A)R_{IA}$, differentiate the relative position:

$$
\dot{\mathbf d}_A
=R_{IA}^{\mathsf T}
(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I).
$$

Define

$$
\mathbf u_I=\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I,\qquad
\mathbf u_A=R_{IA}^{\mathsf T}\mathbf u_I=\dot{\mathbf d}_A.
$$

The transport term moves the reference-frame velocity to $C_o$. Thus $\mathbf u_I=\mathbf v_{Co}-\mathbf v^{(\mathcal A)}(\mathbf p_C)$. It differs from the inertial velocity difference by $\boldsymbol\omega_A\times\mathbf d_I$, perpendicular to the line of centers.

### 2.4 Relative angular velocity and attitude derivative

$$
\boldsymbol\omega_{rel,I}=\boldsymbol\omega_C-\boldsymbol\omega_A,\qquad
\boldsymbol\omega_{rel,A}=R_{IA}^{\mathsf T}\boldsymbol\omega_{rel,I}.
$$

Differentiating $R_{AC}=R_{IA}^{\mathsf T}R_{IC}$ and using rotation invariance of the cross product gives

$$
\dot R_{AC}
=R_{IA}^{\mathsf T}\operatorname{skew}(\boldsymbol\omega_C-\boldsymbol\omega_A)R_{IC}
=\operatorname{skew}(\boldsymbol\omega_{rel,A})R_{AC}.
$$

Physical relative angular velocity and the derivatives of chosen angular coordinates are related by a configuration-dependent rate map.

### 2.5 Separation and its rate

For $l=\|\mathbf d_I\|>0$,

$$
\mathbf e_I=\frac{\mathbf d_I}{l},\qquad
\mathbf e_A=\frac{\mathbf d_A}{l},\qquad
\dot l=\mathbf e_I\cdot(\mathbf v_{Co}-\mathbf v_{Ao})
=\mathbf e_A\cdot\mathbf u_A.
$$

The last equality follows from $\mathbf d_I\cdot(\boldsymbol\omega_A\times\mathbf d_I)=0$. The length rate can therefore be evaluated from either the inertial attachment velocities or the reference-frame relative velocity.

## 3. Wrench application and reduction

### 3.1 Load entries and body-fixed points

Each wrench specifies its receiving body, application point, expression frame, moment about that point and force. The body coordinates of spatial point $\mathbf x$ on $\mathcal A$ are

$$
\mathbf r_Q^{\mathcal A}
=R_{I\mathcal A}^{\mathsf T}(\mathbf x-\mathbf x_{\mathcal A}).
$$

All loads below act at the attachments, whose body coordinates are constant, and are expressed in I.

### 3.2 Endpoint force pair with a reference-end support moment

The force on the reference end is $\mathbf f_I=R_{IA}\mathbf f_A$. The paired wrenches are

$$
\mathcal W_{A_o}^{I}[\mathcal A]
=(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I),\qquad
\mathcal W_{C_o}^{I}[\mathcal C]
=(\mathbf 0,\ -\mathbf f_I).
$$

The resultant force is zero. The resultant moment about any point O is

$$
(\mathbf p_A-\mathbf x_O)\times\mathbf f_I
+\mathbf d_I\times\mathbf f_I
-(\mathbf p_C-\mathbf x_O)\times\mathbf f_I=\mathbf 0.
$$

The reference-end support moment also makes the load power-conjugate to the reference-frame displacement derivative, as shown in Section 4.2.

### 3.3 Pure couple pair

$$
\mathcal W_{A_o}^{I}[\mathcal A]=(\mathbf m_I,\ \mathbf 0),\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=(-\mathbf m_I,\ \mathbf 0),\qquad
\mathbf m_I=R_{IA}\mathbf m_A.
$$

A pure couple is independent of the reduction point; resultant force and moment are zero. This pair can be added to the loads of Section 3.2.

### 3.4 Axial force pair along the line of centers

Positive tensile force $F>0$ pulls the attachments together:

$$
\mathbf f_I=F\mathbf e_I,\qquad
\mathcal W_{A_o}^{I}[\mathcal A]=(\mathbf 0,\ F\mathbf e_I),\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=(\mathbf 0,\ -F\mathbf e_I).
$$

Since $\mathbf d_I\times\mathbf e_I=\mathbf 0$, the resultant moment of the two forces about any point is zero.

### 3.5 Reduction to the body origin

Moving the moment from Q to the body origin $O_{\mathcal A}$ gives

$$
\boldsymbol\tau_{O_{\mathcal A}}
=\boldsymbol\tau_Q+(R_{I\mathcal A}\mathbf r_Q^{\mathcal A})\times\mathbf f.
$$

The force is unchanged. The moment from its lever arm and the explicit endpoint moment both enter the multibody equations; a pure couple retains its value.

## 4. Power identities and organizing principle

### 4.1 Power of a wrench on one body

$$
\mathcal P_{\mathcal A}
=\mathbf f\cdot\mathbf v^{(\mathcal A)}(\mathbf x_Q)
+\boldsymbol\tau_Q\cdot\boldsymbol\omega_A.
$$

Velocity transport, moment reduction and the identity

$$
\boldsymbol\omega\cdot(\mathbf d\times\mathbf f)
=\mathbf f\cdot(\boldsymbol\omega\times\mathbf d)
$$

show that power is unchanged by moving the reduction point. Rotations preserve dot products, so power is also independent of the expression frame.

### 4.2 Endpoint force pair

The loads of Section 3.2 give

$$
\mathcal P
=\mathbf f_I\cdot\mathbf v_{Ao}
+\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)
-\mathbf f_I\cdot\mathbf v_{Co}
=-\mathbf f_A\cdot\mathbf u_A.
$$

Relative displacement $\mathbf d_A$ and force $\mathbf f_A$ are power-conjugate. For an elastic force given by a storage gradient $\mathbf f_{\mathrm{el}}=\partial\mathcal V/\partial\mathbf d_A$, the power delivered to the bodies is $-\dot{\mathcal V}$; integrability of a smooth elastic law requires $\partial f_{\mathrm{el},i}/\partial d_{A,j}=\partial f_{\mathrm{el},j}/\partial d_{A,i}$.

### 4.3 Pure couple pair

$$
\mathcal P
=\mathbf m_I\cdot(\boldsymbol\omega_A-\boldsymbol\omega_C)
=-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}.
$$

A physical couple is power-conjugate to relative angular velocity. Its angular-coordinate representation follows the rate map and conjugate-moment relation in Section 3.2 of [Multibody equations of motion](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.en.md).

### 4.4 Axial force pair

Sections 3.4 and 2.5 give

$$
\mathcal P
=F\mathbf e_I\cdot(\mathbf v_{Ao}-\mathbf v_{Co})
=-F\dot l.
$$

Tensile force is power-conjugate to the rate of attachment separation.

### 4.5 Pairing velocity inputs with load application

Each family pairs its constitutive velocity inputs with load application points and support moments so that total endpoint power equals constitutive power.

| Application | Conjugate velocity | Total body power |
|---|---|---|
| Endpoint forces and reference-end support moment | $\mathbf u_A=\dot{\mathbf d}_A$ | $-\mathbf f_A\cdot\mathbf u_A$ |
| Pure couple pair | $\boldsymbol\omega_{rel,A}$ | $-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}$ |
| Axial force pair | $\dot l$ | $-F\dot l$ |

Loads can be superposed, and their powers add. An element with storage $\mathcal V$ and dissipation rate $\mathcal D\ge0$ satisfies $\mathcal P+\dot{\mathcal V}+\mathcal D=0$. Internal storage is included for elements with internal states.

## 5. Evaluation

### 5.1 Relative-motion evaluation

The two poses give $\mathbf d_I,\mathbf d_A,R_{AC}$; the relative spatial velocity gives $\mathbf u_A,\boldsymbol\omega_{rel,A}$; resolving the attachments gives their bodies and body-fixed coordinates. The translational component of the spatial velocity includes the transport term of Section 2.3. These quantities are determined by the current multibody pose and velocity.

### 5.2 Paired loads

- **Endpoint force pair and support moment**: the translational, fixed-axis series and saturated-damper families use Section 3.2.
- **Pure couple**: the roll element uses Section 3.3.
- **Axial force pair**: point-to-point axial elements acting along the attachment line use Section 3.4.

The reference-frame bushing combines the first two forms. Each element supplies one combined wrench to each body.

### 5.3 State-derivative assembly

Both series families produce loads and internal-force derivatives in the same constitutive evaluation. The system state is $[q;v;z]$, with $z$ collecting the series force states in N, and its full derivative is $[N(q)v;\dot v;\dot z]$.

## 6. Mathematical properties and domain

- **Deformation datum.** Relative motion is measured from the attachment positions and attitudes. The unloaded state follows from the constitutive law and initial internal force; a translational element's nominal force enters as a constitutive constant.
- **Endpoints.** Attachments belong to distinct load-bearing bodies. A point-to-point direction requires $l>0$; reference-frame forms remain defined at coincident points.
- **Reference frame.** Anisotropic coefficient directions move with the reference frame. Exchanging the ends changes the expression frame, transport term and support-moment allocation, so the reference-end choice is part of the constitutive definition.
- **Axial motion.** The transport term projects to zero along the line of centers. Exchanging the ends of a point-to-point element preserves $l,\dot l$, reverses the direction and preserves the physical loads.
- **Rotation.** Physical angular velocity follows from body motion and introduces no angular-coordinate singularity when used directly as a rate input; angular-coordinate rate maps have their own coordinate domains.

## 7. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| Endpoints and element types | `ForceElementEnd`, `VehicleForceElementCollection`, in [vehicle_force_elements.h](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| Relative motion and paired loads | `CalcRelativeMotion`, `EmitTranslationalWrenchPair`, `VehicleForcePlan::CalcAppliedForces`, in [vehicle_force_plan.cc](../../../libs/forces/src/vehicle_force_plan.cc) |
| Pure couple | `internal::EmitCoupleWrenchPair`, in [body_wrench_pair.h](../../../libs/forces/src/body_wrench_pair.h) |
| Poses, relative spatial velocity and attachment points | Corresponding queries of `MultibodyModel`, in [multibody_model.h](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Wrench representation | `AppliedBodyWrench`, in [multibody_applied_forces.h](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
| Full state derivative | `CompiledSystemPlan::CalcStateTimeDerivatives`, in [compiled_system_plan.cc](../../../libs/system_assembly/src/compiled_system_plan.cc) |
