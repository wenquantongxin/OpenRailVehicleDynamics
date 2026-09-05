[中文](CONTACT_MODEL_ASSEMBLY_AND_WRENCH.md)

# Single-wheel contact-model assembly and paired wrench

This chapter explains how `WheelRailContactModel` assembles contact geometry, normal force, creepages, Kalker coefficients, and FASTSIM tangential force into the spatial wrench of one wheel-rail contact patch. It focuses on what passes between the physical stages, why rail material reference point `R` differs from wheel-side force application point `P`, and how forces and moments change reduction point and basis.

## 1. Five-stage physical chain

Contact between one wheel and one rail consists of five theoretical stages:

1. **Contact geometry**: wheel and rail profiles plus relative pose produce zero or more patches, together with each patch's position, width, area, penetration, local radius, and surface directions.
2. **Normal contact**: patch geometry and normal approach speed produce normal force $N$, equivalent penetration $\delta_{\mathrm{eq}}$, and contact-ellipse semi-axes $a$ and $b$.
3. **Creepages**: relative translation, relative rotation, and rolling reference speed in the contact frame produce $\xi_x$, $\xi_y$, and $\varphi$.
4. **Kalker coefficients**: semi-axis ratio $a/b$ and material Poisson's ratio produce $C_{11}$, $C_{22}$, and $C_{23}$, from which the tangential local flexibilities follow.
5. **Tangential contact**: $N$, $a$, $b$, friction coefficient, creepages, and local flexibilities produce $F_x$ and $F_y$ in the contact frame.

The assembly layer then transforms $(F_x,F_y,N)$ into a force in the track-profile frame, places it at wheel-side application point `P`, and forms a paired wrench. The internal theory of each stage is described in [contact geometry](CONTACT_GEOMETRY.en.md), [normal contact force](NORMAL_CONTACT_FORCE.en.md), [creepages and the contact frame](CREEPAGE_AND_CONTACT_FRAME.en.md), [Kalker linear creepage coefficients](KALKER_COEFFICIENTS.en.md), and [FASTSIM tangential contact](TANGENTIAL_CONTACT_FASTSIM.en.md).

## 2. Input kinematics and the stationary-rail assumption

Contact geometry consumes only the relative-pose scalars of the wheel and rail profiles. Mechanical assembly additionally needs the rigid placement of the rail profile in track-profile frame `T` and the position, orientation, velocity, and wheel-body angular velocity of the wheel-profile datum. Path rate and wheel rotation rate separately enter the creepage reference speed.

In this model, the rail is a geometric constraint carried by the line, not a state- and inertia-bearing rigid body in the vehicle multibody tree. Rail material velocity is therefore zero. Relative velocity at contact is supplied entirely by wheel rigid-body motion evaluated at the designated rail material point. This assumption does not state that rail dynamics are absent in general; it states that they are not degrees of freedom of the current contact element.

## 3. Two distinct points: R and P

### 3.1 Rail material reference point R

The geometry stage supplies the coordinates $\mathbf r_R$ of `R` in the rail profile's own frame. The rail-profile origin $\mathbf o_{\mathrm{rail}}$ and orientation $R_{T\mathrm{rail}}$ place it in the track-profile frame:

$$
\mathbf x_R=\mathbf o_{\mathrm{rail}}+R_{T\mathrm{rail}}\mathbf r_R.
$$

Let $\mathbf o_W$ and $\mathbf v_o$ be the position and velocity of the datum of wheel-profile frame W, and let $\boldsymbol\omega$ be the actual wheel-body angular velocity. Wheel material velocity at `R` and the relative angular velocity are

$$
\Delta\mathbf v
=\mathbf v_o+\boldsymbol\omega\times(\mathbf x_R-\mathbf o_W),
\qquad
\Delta\boldsymbol\omega=\boldsymbol\omega.
$$

The purpose of `R` is to compare wheel and rail material velocities at a common spatial point. It is neither the force application point on the wheel nor the pressure centroid.

### 3.2 Wheel-side force application point P

Let $x_w$, $y_w$, and $r$ be the patch's longitudinal coordinate, lateral station, and undeformed local radius on the wheel profile. The implementation defines the force reduction point by

$$
\mathbf r_P=
\begin{bmatrix}
x_w\\y_w\\r-\frac{1}{2}\delta_{\mathrm{eq}}
\end{bmatrix},
\qquad
\mathbf x_P=\mathbf o_W+R_{TW}\mathbf r_P,
$$

where $R_{TW}$ is the wheel-profile orientation with wheel spin omitted. Spin does not alter the geometric placement of an axisymmetric profile, but it remains part of $\boldsymbol\omega$ and affects material velocity.

Point `P` is first of all a model convention for wrench reduction. At the same $(x_w,y_w)$, the downward radial coordinate on the lower half of the undeformed surface of revolution would be

$$
z_{\mathrm{rev}}=\sqrt{\max(0,r^2-x_w^2)},
$$

whereas the expression above uses $r-\delta_{\mathrm{eq}}/2$. Consequently, for $x_w\ne0$, `P` is generally not an exact material point on the surface of revolution. It is the application point obtained by extruding the local profile longitudinally and then shifting it inward by half the equivalent penetration along the wheel-profile $z$ axis. Its undeformed part coincides with the lowest radial position only when $x_w=0$. This distinction sets the lever arm used by later wrench transport but does not alter the $x_w$ found by contact geometry.

`R` and `P` arise from different constructions: the former serves relative velocity, while the latter serves force application. Identifying them changes the lever arm and moment when the contact force is transported to the wheel-body origin.

## 4. Force in the contact frame

Contact frame `C` is constructed from the patch's `rail_slope_angle_radians`, with rotation $R_{TC}$ into the track-profile frame. The same contact frame is used for normal approach speed, creepages, and final force transformation, preventing the three parts from using different surface directions.

The normal $+z_C$ points into the rail, so the normal component of the rail-on-wheel force is $-N$. The forces in the contact and track-profile frames are

$$
\mathbf f_C=
\begin{bmatrix}F_x\\F_y\\-N\end{bmatrix},
\qquad
\mathbf f_T=R_{TC}\mathbf f_C.
$$

The current model does not output a direct spin moment about the patch normal, so at application point `P`,

$$
\mathbf m_P=\mathbf 0.
$$

Contact moments observed by the vehicle at other reference points arise from the lever arm of $\mathbf f_T$, not from adding a within-patch spin moment here.

## 5. Spatial-wrench algebra

### 5.1 Changing the reduction point

Write a spatial wrench as $\mathcal W=(\mathbf m,\mathbf f)$. If it is initially reduced about point $\mathbf a$ and is moved to point $\mathbf b$ without changing basis, then

$$
\mathbf f_b=\mathbf f_a,
\qquad
\mathbf m_b=\mathbf m_a+(\mathbf a-\mathbf b)\times\mathbf f_a.
$$

This is `TransportWrench`. The vector in the cross product runs from the new reduction point to the old one.

### 5.2 Changing the expression basis

If rotation $R$ rewrites components from the old basis into the new basis without changing the reduction point, then

$$
\mathbf f'=R\mathbf f,
\qquad
\mathbf m'=R\mathbf m.
$$

This is `RotateWrench`. Changing point and changing basis are different operations. When rotation also changes point coordinates, their order cannot be interchanged without specifying the geometric relation explicitly.

### 5.3 Paired wrench

`MakePairedContactWrench` constructs both halves of the interaction from one wheel-side force and moment, expressed in the same basis and reduced about the same point `P`:

$$
\mathcal W_{\mathrm{rail\ on\ wheel}}=(\mathbf 0,\mathbf f_T),
\qquad
\mathcal W_{\mathrm{wheel\ on\ rail}}=(\mathbf 0,-\mathbf f_T).
$$

Storing the pair together ensures that action and reaction are formed from the same numbers. If the two halves are later transported to different reference points, their added moments must be computed from their respective lever arms; negating at one point does not imply componentwise cancellation of moments about every other point.

## 6. Assembly algorithm

For one supplied wheel-rail state, the physical calculation in `WheelRailContactModel::Evaluate` can be summarized as

```text
patches = contact_geometry(relative_pose)
for each geometric patch:
    x_R = place_rail_reference_point(patch, rail_profile_frame)
    relative_motion = wheel_rigid_motion_at(x_R) - stationary_rail_motion
    frame = contact_frame(patch.rail_slope_angle)

    v_n = dot(frame.normal, relative_motion.velocity)
    normal = normal_contact(patch.geometry, v_n)
    if normal load is not positive: continue

    creepages = contact_creepages(relative_motion, frame, patch.local_radius)
    mu = friction_law(creepages)
    (F_x, F_y) = tangential_contact(normal, creepages, mu, a / b)

    f_C = (F_x, F_y, -N)
    f_T = frame.rotation * f_C
    x_P = place_wheel_side_application_point(patch, normal.equivalent_penetration)
    emit paired_wrench(point = x_P, force = f_T, direct_moment = 0)
```

Tangential force is not evaluated when normal load is non-positive, because no normal constraint remains to provide Coulomb friction capacity. With multiple patches, each patch independently follows the chain above; the vehicle force plan then transports their wrenches to the required wheel or wheelset reference point and sums them.

## 7. Model scope and approximations

The assembly inherits the applicability limits and non-smooth features of its stages: patch appearance, disappearance, and merging; unilateral normal contact; the low-speed creepage reference convention; piecewise-linear Kalker coefficients and their asymptotic junctions; and FASTSIM adhesion-slip transitions and adaptive strips. Assembly does not remove these features.

The current model also makes four explicit choices: rail material is stationary within the contact element; patches have no direct elastic coupling to one another; direct within-patch spin moment is zero; and `P` uses the profile-extrusion application point above rather than the exact surface-of-revolution material point at nonzero $x_w$. Introducing any of these quantities into vehicle dynamics requires extending the corresponding physical stage; they cannot be recovered uniquely from the existing paired force wrench.

## 8. Source mapping

| Theoretical object | Primary implementation |
|---|---|
| Five-stage assembly and construction of `R` and `P` | `WheelRailContactModel::Evaluate`; see [`wheel_rail_contact_model.cc`](../../../libs/wheel_rail_contact/src/wheel_rail_contact_model.cc) |
| Model input and per-patch result | `WheelRailContactInput`, `WheelRailContactPatchResult`; see [`wheel_rail_contact_model.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_contact_model.h) |
| Contact frame and relative motion | `ContactFrame`, `ContactRelativeMotion`; see [`contact_creepage.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h) |
| Wrench point transport and basis rotation | `TransportWrench`, `RotateWrench`; see [`contact_wrench.cc`](../../../libs/wheel_rail_contact/src/contact_wrench.cc) |
| Paired wrench | `PairedContactWrench`, `MakePairedContactWrench`; see [`contact_wrench.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_wrench.h) |
