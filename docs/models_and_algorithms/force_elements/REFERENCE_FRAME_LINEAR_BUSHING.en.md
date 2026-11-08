[中文](REFERENCE_FRAME_LINEAR_BUSHING.md)

# Reference-frame linear bushing

The bushing connects attachment frames on two bodies and defines three-axis translational elasticity, translational damping and rotational damping in the reference frame. Relative motion, wrench and power notation follows [Force-element kinematics and spatial wrenches](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md).

## 1. Definition

The origins of reference frame A and opposite frame C are the attachment points. Let

$$
\mathbf d_A=R_{IA}^{\mathsf T}(\mathbf p_C-\mathbf p_A),\qquad
\mathbf u_A=\dot{\mathbf d}_A
=R_{IA}^{\mathsf T}
(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I),
$$

$$
\boldsymbol\omega_{rel,A}
=R_{IA}^{\mathsf T}(\boldsymbol\omega_C-\boldsymbol\omega_A).
$$

The coefficients are constant diagonal matrices in the reference frame:

$$
K_t=\operatorname{diag}(k_1,k_2,k_3),\qquad
C_t=\operatorname{diag}(c_1,c_2,c_3),\qquad
C_r=\operatorname{diag}(c_{r,1},c_{r,2},c_{r,3}).
$$

Units are N/m for $k_i$, N·s/m for $c_i$ and N·m·s/rad for $c_{r,i}$; all coefficients are finite and non-negative. Subscripts $t,r$ denote translation and rotation; $C_t,C_r$ are element damping matrices.

The reference-end constitutive force and damping couple are

$$
\mathbf f_A=K_t\mathbf d_A+C_t\mathbf u_A,\qquad
\mathbf m_A=C_r\boldsymbol\omega_{rel,A}.
$$

Translational elasticity has its zero at coincident attachment points. The rotational term uses physical relative angular velocity directly.

## 2. Loads and power

### 2.1 Reference-frame velocity

As the reference frame rotates with its body,

$$
\frac{\mathrm d}{\mathrm dt}
\left(R_{IA}^{\mathsf T}\mathbf d_I\right)
=R_{IA}^{\mathsf T}
\left(\dot{\mathbf d}_I-\boldsymbol\omega_A\times\mathbf d_I\right)
=\mathbf u_A.
$$

Translational damping is paired with this displacement derivative. Rotational damping acts on the physical angular velocity of C relative to A, with its three coefficients defined along the axes of A.

### 2.2 Wrenches at the attachment points

Write $\mathbf f_I=R_{IA}\mathbf f_A$ and $\mathbf m_I=R_{IA}\mathbf m_A$. Apply

$$
\mathcal W_{A_o}^{I}[\mathcal A]
=(\mathbf d_I\times\mathbf f_I+\mathbf m_I,\ \mathbf f_I),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]
=(-\mathbf m_I,\ -\mathbf f_I).
$$

The reference end carries the support moment $\mathbf d_I\times\mathbf f_I$. About any point O, the resultant moment is

$$
(\mathbf p_A-\mathbf x_O)\times\mathbf f_I
+\mathbf d_I\times\mathbf f_I+\mathbf m_I
-(\mathbf p_C-\mathbf x_O)\times\mathbf f_I-\mathbf m_I=\mathbf 0.
$$

The resultant force is also zero.

### 2.3 Storage and dissipation

The total power delivered to the bodies is

$$
\begin{aligned}
\mathcal P
&=\mathbf f_I\cdot(\mathbf v_{Ao}-\mathbf v_{Co})
+\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I+\mathbf m_I)
-\boldsymbol\omega_C\cdot\mathbf m_I\\
&=-\mathbf f_A\cdot\mathbf u_A
-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}.
\end{aligned}
$$

Since $K_t$ is constant and symmetric in the reference frame,

$$
\mathcal V=\tfrac12\mathbf d_A^{\mathsf T}K_t\mathbf d_A,\qquad
\dot{\mathcal V}=\mathbf u_A^{\mathsf T}K_t\mathbf d_A.
$$

The damping dissipation rate is

$$
\mathcal D
=\mathbf u_A^{\mathsf T}C_t\mathbf u_A
+\boldsymbol\omega_{rel,A}^{\mathsf T}C_r\boldsymbol\omega_{rel,A}\ge0,
\qquad
\mathcal P+\dot{\mathcal V}+\mathcal D=0.
$$

The absorbed power therefore consists of the change in elastic storage and damping dissipation.

## 3. Properties and domain

- Coefficient axes move with the reference frame; anisotropy is defined in A. The reference-end choice also determines the transport term and support-moment allocation.
- At coincident attachment points, translational elastic force and support moment vanish; translational and rotational damping still follow from relative velocities.
- Semidefinite coefficients allow free motion in some directions; positive-definite damping gives positive dissipation for the corresponding nonzero relative velocity.
- Force and couple are algebraic functions of current relative motion. Translational storage is carried by the multibody position and velocity state.
- Applying the same spatial rigid transformation to both ends preserves the reference-frame constitutive quantities and rotates the inertial loads accordingly.

## 4. Source mapping

| Theoretical object | Implementation |
|---|---|
| Endpoints and three sets of diagonal coefficients | `ReferenceFrameLinearBushing`, in [vehicle_force_elements.h](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| Relative motion, support moment and damping couple | `CalcRelativeMotion`, `EmitTranslationalWrenchPair`, `VehicleForcePlan::CalcAppliedForces`, in [vehicle_force_plan.cc](../../../libs/forces/src/vehicle_force_plan.cc) |
