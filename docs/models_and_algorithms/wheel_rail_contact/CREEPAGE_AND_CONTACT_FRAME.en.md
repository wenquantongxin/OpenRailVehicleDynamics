[中文](CREEPAGE_AND_CONTACT_FRAME.md)

# Creepages and the contact frame

This chapter explains how ORVD expresses relative motion at a contact in the contact frame and forms longitudinal creepage, lateral creepage, spin creepage, and normal approach speed. The theoretical definitions and their implementation reside in [`contact_creepage.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h) and [`contact_creepage.cc`](../../../libs/wheel_rail_contact/src/contact_creepage.cc), respectively.

## 1. Scope and notation

Contact geometry supplies the patch position, local rolling radius, and rail-surface slope angle; this chapter defines the kinematic quantities built from them. See [normal contact force](NORMAL_CONTACT_FORCE.en.md) for the normal law, [Kalker linear creepage coefficients](KALKER_COEFFICIENTS.en.md) and [FASTSIM tangential contact](TANGENTIAL_CONTACT_FASTSIM.en.md) for the tangential law, and [contact-model assembly and paired wrench](CONTACT_MODEL_ASSEMBLY_AND_WRENCH.en.md) for the final assembly.

Unless stated otherwise, vectors are expressed in the track-profile frame `T`. The notation is:

| Symbol | Meaning |
|---|---|
| $R_{TC}$ | Rotation from contact frame `C` into track-profile frame `T` |
| $\alpha$ | Contact-frame angle: rail-surface slope at the patch centroid plus the side-signed laying cant |
| $\mathbf n_T$ | Contact normal expressed in `T` |
| $\mathbf v_T$, $\boldsymbol\omega_T$ | Translational and angular velocity of the wheel relative to the rail at contact |
| $\dot\ell$ | Path rate at which the contact advances along the line |
| $\Omega$ | Wheel rotation rate about the axle; negative in forward rolling under the repository convention |
| $r$ | Undeformed local rolling radius at contact |
| $V_0$, $V$ | Creepage reference speed before and after flooring |
| $\xi_x$, $\xi_y$, $\varphi$ | Longitudinal, lateral, and spin creepage |
| $v_n$ | Normal approach speed; positive in approach |

## 2. Contact frame and relative motion

### 2.1 Contact frame from the rail-surface slope

`MakeContactFrame` constructs a pure roll about the `+x` axis of the track-profile frame:

$$
R_{TC}(\alpha)=
\begin{bmatrix}
1&0&0\\
0&\cos\alpha&-\sin\alpha\\
0&\sin\alpha&\cos\alpha
\end{bmatrix},\qquad
\mathbf n_T=R_{TC}\mathbf e_z=
\begin{bmatrix}0\\-\sin\alpha\\\cos\alpha\end{bmatrix}.
$$

The longitudinal axes of the contact and track-profile frames therefore coincide exactly. The implementation uses `ContactPatch::rail_slope_angle_radians`, not `common_normal_angle_radians`: the former describes the orientation of the rail reference surface, whereas the latter describes the geometric common normal of the wheel and rail profiles. They are not interchangeable.

Relative translational and angular velocities are expressed in the contact frame with the transpose of the rotation:

$$
\mathbf v_C=R_{TC}^{\mathsf T}\mathbf v_T,
\qquad
\boldsymbol\omega_C=R_{TC}^{\mathsf T}\boldsymbol\omega_T.
$$

In particular, $v_{C,x}=v_{T,x}$; the lateral, normal, and normal-spin components are mixed by $\alpha$.

### 2.2 Motion at the rail material reference point

The assembly layer forms the relative motion at the rail material reference point `R`. Let $\mathbf o_W$ and $\mathbf v_o$ be the position and velocity of the datum of wheel-profile frame W, $\boldsymbol\omega$ the wheel-body angular velocity, and $\mathbf x_R$ the position of `R` in the track-profile frame. Then

$$
\mathbf v_T=\mathbf v_o+\boldsymbol\omega\times(\mathbf x_R-\mathbf o_W),
\qquad
\boldsymbol\omega_T=\boldsymbol\omega.
$$

The rail is stationary geometry carried by the line rather than a state-bearing rigid body in the multibody system, so its material velocity is zero. Point `R` is used to form relative material velocity; it differs from the force application point `P`, as explained in [contact-model assembly and paired wrench](CONTACT_MODEL_ASSEMBLY_AND_WRENCH.en.md).

## 3. Creepages

### 3.1 Reference speed

The rim speed and the unfloored reference speed are

$$
v_{\mathrm{rim}}=-\Omega r,
\qquad
V_0=\frac{1}{2}\left(\dot\ell+v_{\mathrm{rim}}\right).
$$

Pure rolling has $\dot\ell=v_{\mathrm{rim}}$. Their mean is the velocity scale used to normalize relative slip into creepage. To define the division near standstill, the implementation applies a signed floor $V_{\min}>0$:

$$
V=
\begin{cases}
V_0,&|V_0|\ge V_{\min},\\
\operatorname{copysign}(V_{\min},V_0),&0<|V_0|<V_{\min},\\
+V_{\min},&V_0=0.
\end{cases}
$$

This is an explicit low-speed modelling convention, not a smooth regularization. $V$ changes sign at $V_0=0$ and is continuous but not differentiable at $|V_0|=V_{\min}$.

Any claim about a sign reversal must state its conditions. If a numerator retains the same nonzero value on both sides of $V_0=0$, its quotient reverses sign when $V$ does. If the numerator also passes through zero, changes sign, or changes magnitude, the reference speed alone does not determine the sign of the corresponding creepage. The three creepages therefore do not possess an unconditional simultaneous-sign-reversal invariant during an actual vehicle reversal.

### 3.2 Definitions of the three creepages

The implementation uses

$$
\xi_x=\frac{v_{C,x}}{V},
\qquad
\xi_y=\frac{v_{C,y}}{V},
\qquad
\varphi=\frac{\omega_{C,z}}{V}.
$$

$\xi_x$ and $\xi_y$ are dimensionless; $\varphi$ has dimension $\mathrm{m}^{-1}$. Because the contact frame is a pure roll, the numerator of longitudinal creepage is independent of $\alpha$. Spin contains the projection of wheel angular velocity onto the contact normal:

$$
\omega_{C,z}=-\sin\alpha\,\omega_{T,y}+\cos\alpha\,\omega_{T,z}.
$$

### 3.3 Normal approach speed

Normal approach speed is the normal velocity component in the contact frame:

$$
v_n=\mathbf n_T\cdot\mathbf v_T
=-\sin\alpha\,v_{T,y}+\cos\alpha\,v_{T,z}
=v_{C,z}.
$$

The vertical axis of the track-profile frame points downward and $\mathbf n_T$ points into the rail, so a wheel approaching the rail along the normal has $v_n>0$. The normal contact law uses this sign convention for its damping term.

### 3.4 Rolling-radius convention

The radius $r$ in the reference speed is the undeformed local wheel radius:

$$
r=r_0+h_w(y_w),
$$

where $r_0$ is the nominal rolling radius and $h_w(y_w)$ is wheel-profile height at the contact station. This implementation does not subtract elastic penetration from $r$. The force application point may use $r-\delta_{\mathrm{eq}}/2$, but that is a point-placement convention and does not alter the creepage reference speed.

## 4. Numerical structure and applicability

The calculation order is: construct $R_{TC}$ and $\mathbf n_T$, rotate relative motion into the contact frame, form $V$, and then calculate the three creepages. Normal approach speed is projected with the same contact frame. The normal law, creepages, and final force vector therefore use exactly the same $\alpha$.

For bounded numerators, the reference-speed floor keeps the ratios bounded near exact standstill but retains two non-smooth features: the sign jump at $V_0=0$ and the derivative corner at $|V_0|=V_{\min}$. The model is intended for rolling regimes with a well-defined travel direction. A study of sustained low-speed reversal should treat the floor as part of the model and assess its effect on forces and numerical Jacobians.

## 5. Source mapping

| Theoretical object | Primary implementation |
|---|---|
| Contact frame and normal | `ContactFrame`, `MakeContactFrame`; see [`contact_creepage.cc`](../../../libs/wheel_rail_contact/src/contact_creepage.cc) |
| Relative motion and creepages | `ContactRelativeMotion`, `Creepages`, `ComputeCreepages`; see [`contact_creepage.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h) |
| Normal approach speed | `ComputeNormalApproachSpeed`; see [`contact_creepage.cc`](../../../libs/wheel_rail_contact/src/contact_creepage.cc) |
| Rigid-body velocity at point `R` | `WheelRailContactModel::Evaluate`; see [`wheel_rail_contact_model.cc`](../../../libs/wheel_rail_contact/src/wheel_rail_contact_model.cc) |
| Geometric origin of $\alpha$ and local radius | `ContactPatch`; see [`contact_geometry.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_geometry.h) |
