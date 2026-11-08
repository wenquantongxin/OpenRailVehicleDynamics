[中文](POINT_TO_POINT_SERIES_SPRING_BILINEAR_DAMPER.md)

# Point-to-point series spring–bilinear damper

A linear spring and an odd bilinear damper in series transmit a common axial force along their attachment line. This force is an internal state evolving with the two bodies. Endpoints and expression frames follow [Force-element kinematics and spatial wrenches](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md).

## 1. Definition

### 1.1 Geometry and parameters

$$
\mathbf d_I=\mathbf p_C-\mathbf p_A,\qquad
l=\|\mathbf d_I\|>0,\qquad
\mathbf e_I=\mathbf d_I/l,\qquad
\dot l=\mathbf e_I\cdot(\mathbf v_{Co}-\mathbf v_{Ao})
=\mathbf e_A\cdot\mathbf u_A.
$$

The direction $\mathbf e_I$ points from reference end A to opposite end C; $F>0$ denotes tension. Spring stiffness is $k$, inner and outer damping slopes are $c,c_2$, and the positive knee force is $f_b$. All four parameters are finite and strictly positive, with units N/m, N·s/m, N·s/m and N respectively. The knee velocity is $v_b=f_b/c$.

### 1.2 Damping law and its inverse

For damper extension rate $v_d$,

$$
D(v_d)=
\begin{cases}
c\,v_d,& |v_d|\le v_b,\\
\operatorname{sgn}(v_d)\left[f_b+c_2(|v_d|-v_b)\right],& |v_d|>v_b.
\end{cases}
$$

The branches meet continuously, and the outer branch extends with slope $c_2$. $D$ is a strictly increasing odd function with a unique inverse on the real line:

$$
D^{-1}(F)=
\begin{cases}
F/c,& |F|\le f_b,\\
\operatorname{sgn}(F)\left[f_b/c+(|F|-f_b)/c_2\right],& |F|>f_b.
\end{cases}
$$

## 2. Series conditions and internal state

### 2.1 Internal-force equation

Let $\delta_s$ and $\delta_d$ be spring and damper extensions. The series conditions are

$$
F=k\delta_s=D(\dot\delta_d),\qquad
\dot\delta_s+\dot\delta_d=\dot l.
$$

Eliminating damper velocity with $\dot\delta_d=D^{-1}(F)$ gives

$$
\dot F=k\left(\dot l-D^{-1}(F)\right).
$$

The initial force $F(t_0)$ and the initial body poses and velocities determine the evolution. With spring extension as the state, $\delta_s=F/k$ and

$$
\dot\delta_s=\dot l-D^{-1}(k\delta_s).
$$

The two state representations are related by constant scaling $k$. This family uses force $F$ in N, with one component of the system's $z$ block per element.

### 2.2 Attachment loads

$$
\mathcal W_{A_o}^{I}[\mathcal A]=(\mathbf 0,\ F\mathbf e_I),\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=(\mathbf 0,\ -F\mathbf e_I).
$$

The forces are collinear, giving zero resultant force and moment about any point. The transport term projects to zero in the length rate, so reference-frame relative velocity and inertial attachment-velocity difference give the same $\dot l$.

### 2.3 Storage and dissipation

Power delivered to the bodies, spring storage and damping dissipation rate are

$$
\mathcal P=-F\dot l,\qquad
\mathcal V=\frac{F^2}{2k},\qquad
\mathcal D=F D^{-1}(F)\ge0.
$$

The internal-force equation gives

$$
\dot{\mathcal V}=\frac{F\dot F}{k}
=F\dot l-F D^{-1}(F),\qquad
\mathcal P+\dot{\mathcal V}+\mathcal D=0.
$$

$\mathcal D\ge0$ because $D^{-1}(F)$ has the same sign as $F$. Tension and compression have the same dissipative properties.

## 3. Response properties

### 3.1 Relaxation at fixed length

Set $\dot l=0$. Within $|F|\le f_b$,

$$
F(t)=F(t_0)e^{-k(t-t_0)/c}.
$$

For an initial force in the outer branch, write $r=|F|>f_b$. Then

$$
\dot r=-k\left[\frac{f_b}{c}+\frac{r-f_b}{c_2}\right].
$$

With $r_*=f_b(1-c_2/c)$, the solution within that branch is

$$
r(t)=r_*+[r(t_0)-r_*]e^{-k(t-t_0)/c_2}.
$$

This expression applies until $r=f_b$, after which the inner exponential decay continues. The quantity $r_*$ is an algebraic constant of the outer-branch equation; since $r_*<f_b$, force magnitude decreases throughout the outer branch and ultimately tends to zero. The local relaxation times are $c/k$ and $c_2/k$.

### 3.2 Constant length rate and linear special case

For constant $\dot l=u_0$, the steady force satisfies

$$
F_\infty=D(u_0).
$$

With $c=c_2$, the global damping law is $D(v_d)=c\,v_d$ and the force equation becomes $\dot F=k\dot l-(k/c)F$. It has the same scalar form as the [fixed-axis series spring–viscous damper](SERIES_SPRING_VISCOUS_DAMPER.en.md). The present element follows the attachment line, while the fixed-axis element uses the displacement derivative along a specified reference-frame axis and its associated support moment.

### 3.3 Continuity and domain

Both $D$ and $D^{-1}$ are continuous at the knees with finite one-sided derivatives; these derivatives change when $c\ne c_2$. The force equation is continuous and Lipschitz in $F$. With continuous line speed and separated attachments, an initial value determines a unique local solution; continuous poses and velocities give continuous force and force derivative. The line direction is undefined at $l=0$.

Exchanging the ends preserves $l,\dot l,F$ and reverses $\mathbf e_I$, preserving the loads on the original bodies.

## 4. Source mapping

| Theoretical object | Implementation |
|---|---|
| Endpoints and four parameters | `PointToPointSeriesSpringBilinearDamper`, in [vehicle_force_elements.h](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| Inverse law, length rate, loads and force derivative | `InverseBilinearDamping`, `VehicleForcePlan::CalcAppliedForces`, in [vehicle_force_plan.cc](../../../libs/forces/src/vehicle_force_plan.cc) |
| Initial force | `PointToPointSeriesSpringBilinearDamperForceState`, in [resolved_startup_state.h](../../../libs/configuration/include/orvd/configuration/resolved_startup_state.h) |
| Force component in the continuous state | `SystemInstance::series_spring_damper_force_state_range`, in [system_instance.h](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
