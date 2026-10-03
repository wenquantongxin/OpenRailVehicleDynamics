[中文](TIME_INTEGRATION_METHODS.md)

# BDF, Radau5, Newmark and Zhai time-integration methods

This chapter explains how BDF, the three-stage fifth-order Radau IIA method, the Newmark average-acceleration method and Zhai's simple explicit method advance continuous equations of motion to discrete states, and discusses their error, stability and compatibility with ORVD's state structure. ORVD implements five method options: CVODE BDF with maximum order 2 or 5, Radau5, the Newmark average-acceleration method with bounded step recovery, and Zhai's simple explicit method. All five are selected explicitly through one public system-integration configuration. BDF and Radau5 act directly on the first-order state; Newmark and Zhai act on the coordinate second-order form of section 1.3. Parts of the classical theory in sections 4.1 and 5.1 that ORVD does not adopt are marked **theory only**.

## 1. Equation forms and common notation

### 1.1 First-order state equation

This chapter follows [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md) with a single substitution: the continuous state written there as $x$ is written here as $y$, following the usual notation for a first-order initial-value problem, and its dimension is written $n_x$, in the same family as that chapter's $n_q$ and $n_v$. The capital letter $N$ is reserved here for the position-derivative map $N(q)$.

BDF and Radau5 act directly on the first-order initial-value problem

$$
\dot y=f(t,y),
\qquad
y(t_0)=y_0,
\qquad
y\in\mathbb R^{n_x}.
$$

ORVD writes its continuous state as

$$
y=
\begin{bmatrix}
q\\v\\z
\end{bmatrix},
\qquad
\dot y=
\begin{bmatrix}
N(q)v\\a(t,q,v,z)\\g(t,q,v,z)
\end{bmatrix}.
$$

Here $q$ is generalized position, $v$ is generalized velocity and $z$ contains the first-order internal variables of force elements such as series spring-viscous-damper (Maxwell-type) elements. When a free body's orientation is represented by a quaternion, $q$ and $v$ have different dimensions and the configuration kinematics are $\dot q=N(q)v$; the complete state therefore cannot be reduced to $\dot q=v$.

Different parts of this right-hand side live in different modules. The multibody map $[q;v]\mapsto[N(q)v;\dot v]$ is expressed by `MultibodyModel::CalcStateTimeDerivatives` in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h). The complete $[q;v;z]$ derivative is assembled by `CompiledSystemPlan::CalcStateTimeDerivatives` in [`compiled_system_plan.cc`](../../../libs/system_assembly/src/compiled_system_plan.cc); `SystemRhsBridge::CalcTimeDerivatives` in [`system_rhs_bridge.cc`](../../../libs/integrators/src/system_rhs_bridge.cc) installs the integrator's $(t,y)$ in the trial context and delegates to that assembly. The series spring-viscous-damper element and its force-state equation are defined by `SeriesSpringViscousDamper` in [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h), and the derivative of that force state is formed by `VehicleForcePlan::CalcAppliedForces` in [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc):

$$
\dot F=k\,v_{\mathrm{rel}}-\frac{k}{c}F.
$$

A spring in series with a viscous damper makes the force itself a state, because the two share one force while their deflections differ. Here $k$ and $c$ are the scalar series stiffness and series damping of that element, and $v_{\mathrm{rel}}$ is the relative velocity of its two ends along the acting axis. The time constant is $c/k$. Lower-case $k$ and $c$ denote element-level scalars throughout this chapter, while capital $M$, $C$ and $K$ denote system-level matrices only.

### 1.2 Second-order mechanical equation

Classical Newmark and Zhai methods are usually formulated from the second-order mechanical equation

$$
M(u)a+C(u,v)v+f_{\mathrm{int}}(u,v,z)=p(t),
\qquad
\dot u=v.
$$

Here $u$ is Euclidean displacement, used as in sections 4.1 and 5.1 and distinct from the generalized position $q$ of section 1.1; $a$ is acceleration and $p$ is the external load. $M$ is the system mass matrix. The product $C(u,v)v$ collects velocity-dependent inertial terms such as Coriolis and centrifugal effects together with viscous damping. $f_{\mathrm{int}}$ holds the remaining internal forces, including elastic restoring forces and element forces that depend on the internal variables $z$.

This form is natural for Euclidean coordinates in which displacement, velocity and acceleration have equal dimensions. There are two routes to a multibody configuration with quaternions and Ball-RPY coordinates. One forms the configuration increment in the tangent space and returns to the configuration manifold through a retraction. The other uses the second-order equation of the configuration coordinates themselves, integrates the stored coordinates and projects the endpoint back onto the constraint set. ORVD follows the second route, described in section 1.3; the first is **theory only** in this chapter. On either route the first-order internal variables $z$ need their own discretization.

### 1.3 Coordinate second-order form

In ORVD, Newmark and Zhai act on the second-order equation of the configuration coordinates. Define the coordinate velocity and the coordinate acceleration

$$
s=\dot q=N(q)v,
\qquad
b=\ddot q=N(q)\dot v+\dot N(q,v)\,v,
$$

where $\dot N(q,v)$ is the time derivative of $N(q)$ along the motion. For the quaternion block of a free body, the corresponding four components of $s$ are the time derivatives of the stored quaternion and differ in dimension from the physical angular velocity; for single-axis joints, Ball-RPY and translational coordinates, $s$ is simply the time derivative of the coordinate.

With the coordinate state $(q,s,z)$ in place of the physical state $(q,v,z)$, the complete right-hand side becomes

$$
\dot q=s,
\qquad
\dot s=B(t,q,s,z),
\qquad
\dot z=G(t,q,s,z),
$$

$$
B(t,q,s,z)=N(q)\,a\left(t,q,N^{+}(q)s,z\right)+\dot N\left(q,N^{+}(q)s\right)N^{+}(q)s,
\qquad
G(t,q,s,z)=g\left(t,q,N^{+}(q)s,z\right).
$$

Here $a$ and $g$ are the $\dot v$ and $\dot z$ parts of the right-hand side in section 1.1, and $N^{+}(q)$ is the left pseudo-inverse of the rate map; see section 3.1 of [Multibody equations of motion](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.en.md). On an exact solution $s$ lies in the range of $N(q)$ and $N^{+}$ recovers the physical velocity exactly. Newton trials and explicit predictions produce values of $s$ outside that range, for example a quaternion block with a radial component along $q$; $N^{+}$ then projects $s$ onto the range before recovering the velocity. This extension permits trials with non-tangent coordinate velocities within the valid configuration domain and agrees with the original equation on exact solutions; it does not enlarge the domains of the rate maps, contact geometry or other physical evaluations.

$B$ is evaluated in this order: recover the physical velocity through $N^{+}$, evaluate the complete right-hand side of section 1.1, take $\dot v$ and $\dot z$, and let the multibody model map $\dot v$ to the coordinate second derivative. The mass matrix appears only inside forward dynamics, the articulated-body algorithm of section 6 of the vehicle chapter; the integration formulas themselves never form or solve with a mass matrix.

The price of the coordinate form is that the discrete equations no longer preserve the stored quaternion norm. For each quaternion block, the exact flow satisfies $q^{\mathsf T}s=0$ and keeps the stored norm constant, whereas a discrete update generally lets it drift. ORVD projects quaternions back to their reference norm at every accepted endpoint, as described in section 4.4. Zero-norm quaternions and the singular pitch domain of Ball-RPY are rejected by geometry and rate-map domain checks; the integrator does not repair them.

### 1.4 Error scaling

State components have different physical dimensions. An adaptive method can combine a relative tolerance and componentwise absolute tolerances into the weights

$$
w_i=\frac{1}{\operatorname{rtol}|y_i|+\operatorname{atol}_i},
\qquad
\lVert e\rVert_{\mathrm{WRMS}}
=\sqrt{\frac{1}{n_x}\sum_{i=1}^{n_x}(w_i e_i)^2}.
$$

A local error estimate in such a weighted norm determines step-size adaptation. Different methods use different error estimators and controllers, so equal `rtol` and `atol` values do not imply equal global errors.

## 2. BDF: implicit linear multistep methods

[CVODE's mathematical description](https://sundials.readthedocs.io/en/latest/cvode/Mathematics_link.html) gives its variable-step, variable-order BDF formulation. A BDF method of order $k$ approximates the derivative at the new endpoint using the unknown current state and several historical states. At a fixed uniform step size it can be written as

$$
\sum_{j=0}^{k}\alpha_j y_{n+1-j}
=h f(t_{n+1},y_{n+1}).
$$

### 2.1 Fixed uniform-step formulas

The uniform-step BDF1–BDF5 formulas are

$$
\begin{aligned}
\text{BDF1:}\quad
&y_{n+1}-y_n=h f_{n+1},\\
\text{BDF2:}\quad
&\frac{3}{2}y_{n+1}-2y_n+\frac{1}{2}y_{n-1}=h f_{n+1},\\
\text{BDF3:}\quad
&\frac{11}{6}y_{n+1}-3y_n+\frac{3}{2}y_{n-1}-\frac{1}{3}y_{n-2}=h f_{n+1},\\
\text{BDF4:}\quad
&\frac{25}{12}y_{n+1}-4y_n+3y_{n-1}-\frac{4}{3}y_{n-2}+\frac{1}{4}y_{n-3}=h f_{n+1},\\
\text{BDF5:}\quad
&\frac{137}{60}y_{n+1}-5y_n+5y_{n-1}-\frac{10}{3}y_{n-2}+\frac{5}{4}y_{n-3}-\frac{1}{5}y_{n-4}=h f_{n+1}.
\end{aligned}
$$

These coefficients apply only to a fixed uniform step size. Variable-step BDF must rebuild its difference coefficients from the recent step-size history. A maximum order of two or five only limits the available orders; it does not mean that every step uses that order.

### 2.2 Implicit endpoint and step-size control

After collecting the historical terms, the new endpoint can be represented by the nonlinear residual

$$
\mathcal F_{\mathrm{BDF}}(y_{n+1})
=y_{n+1}-\gamma_{\mathrm{BDF}}f(t_{n+1},y_{n+1})
-y_{n+1}^{\mathrm{hist}}=0,
$$

where the history vector $y_{n+1}^{\mathrm{hist}}$ is determined by accepted states and $\gamma_{\mathrm{BDF}}$ by the current step size and BDF coefficients. Newton or modified Newton iteration solves

$$
\left(I_{n_x}-\gamma_{\mathrm{BDF}}J\right)\delta
=-\mathcal F_{\mathrm{BDF}},
\qquad
J=\frac{\partial f}{\partial y},
\qquad
y^{(m+1)}=y^{(m)}+\delta.
$$

An adaptive BDF step performs historical extrapolation, nonlinear solution, local-error estimation and selection of the next step size and order. The method history advances only after the new endpoint is accepted. When the state equation or an externally held quantity changes, the old history no longer represents the same initial-value problem and must be reconstructed from the current endpoint.

### 2.3 Forming the finite-difference Jacobian

The iteration matrix $I_{n_x}-\gamma_{\mathrm{BDF}}J$ of section 2.2 requires $J=\partial f/\partial y$, and ORVD forms it by finite differences. The differenced object is the complete right-hand side $f(t,\cdot)$ of section 1.1, that is, the whole map $[q;v;z]\mapsto[N(q)v;\dot v;\dot z]$: multibody forward dynamics, force elements and wheel-rail contact all lie inside the differenced function. The differences are taken at the trial point $(t,y)$ at which the solver requests a linearization; $t$ and every input that is not part of the continuous state (for example the station seeds of the wheel-rail projections and the nominal forces of force elements) are held fixed throughout the batch of column evaluations, only one stored component of the continuous state is changed at a time, and each column is a one-sided difference quotient

$$
J_{:,j}\approx\frac{f(t,\,y+\Delta_j\mathbf e_j)-f(t,\,y)}{\Delta_j},
\qquad j=1,\dots,n_x,
$$

where $\mathbf e_j$ is the $j$-th coordinate unit vector and $f(t,y)$ is the baseline derivative the solver has already evaluated at that point, so forming $J$ once costs $n_x$ additional right-hand-side evaluations. The perturbation is applied to stored values: each of the four components of every quaternion in $q$ is likewise treated as an ordinary scalar and incremented by $\Delta_j$, with no projection onto the unit sphere, and the right-hand side is evaluated at the perturbed stored values (the position-derivative map likewise uses the quaternion as stored, see section 3.1 of [Multibody equations of motion](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.en.md)). The resulting column is therefore the partial derivative, with respect to that stored component, of $f$ as the implementation defines it, including how the right-hand side extends to non-unit quaternions; it is not the tangential derivative on the unit-quaternion manifold. For the components of $v$ and $z$ and for the position coordinates that are not quaternions, the two readings coincide.

The increment $\Delta_j$ follows the scale rule of CVODE's built-in dense difference quotient (see the [CVODE mathematical description](https://sundials.readthedocs.io/en/latest/cvode/Mathematics_link.html) cited above):

$$
\Delta_j=\max\left(\sqrt U\,\lvert y_j\rvert,\ \frac{\Delta_{\min}}{w_j}\right),
\qquad
\Delta_{\min}=
\begin{cases}
\mu\,\lvert h\rvert\,U\,n_x\,\lVert f(t,y)\rVert_{\mathrm{WRMS}}, & \lVert f(t,y)\rVert_{\mathrm{WRMS}}\neq0,\\
1, & \lVert f(t,y)\rVert_{\mathrm{WRMS}}=0.
\end{cases}
$$

Here $U$ is CVODE's unit roundoff, `DBL_EPSILON` in double precision; $w_j$ is the error weight the solver holds for the current step and keeps fixed throughout this batch of differences, formed as in section 1.4 from `rtol` and `atol` at the step's reference state $y^{(w)}$, which need not be the trial point at which the differences are taken, and $\lVert\cdot\rVert_{\mathrm{WRMS}}$ is the weighted norm of the same section; $h$ is the current internal step size; and $\mu=1000$ is the dimensionless constant that defines the increment floor $\Delta_{\min}$. The first term makes the increment proportional to the magnitude of the component itself, the usual compromise of a one-sided difference between truncation and roundoff error. The second term keeps the increment from vanishing as $y_j$ approaches zero: $\Delta_{\min}$ measures, through $\lvert h\rvert\,\lVert f\rVert_{\mathrm{WRMS}}$, the weighted size of the state change over one step, and $1/w_j=\operatorname{rtol}\lvert y^{(w)}_j\rvert+\operatorname{atol}_j$ converts it to the units of component $j$. The denominator of the quotient is the nominal increment $\Delta_j$, not the increment actually realized after floating-point addition; this differs from the Radau5 core of section 3.5.

In the implementation the differencing is organized in one of two ways, both of which define the same finite-difference Jacobian by the same increment rule and the same denominator. The first is the built-in difference quotient of CVODE's dense linear solver. The second is ORVD's own column provider: `DenseFiniteDifferenceJacobianProvider` in [`dense_finite_difference_jacobian_provider.h`](../../../libs/integrators/src/dense_finite_difference_jacobian_provider.h) is responsible only for the perturbed derivatives $f(t,y+\Delta_j\mathbf e_j)$ of the columns, and its system implementation, `CalcPerturbedDerivatives` in [`system_continuous_state_backend.cc`](../../../libs/integrators/src/system_continuous_state_backend.cc), copies $y$ for each column in a separate trial context, adds $\Delta_j$ and calls the complete right-hand side of section 1.1; the increments $\Delta_j$, the baseline derivative $f(t,y)$ and the quotient itself are held by `EvaluateDenseJacobian` in [`cvode_continuous_state_advancer.cc`](../../../libs/integrators/src/cvode_continuous_state_advancer.cc) according to the formulas above, and the provider contains no differencing formula.

The finite-difference Jacobian does not change the BDF nonlinear residual equation itself: $\mathcal F_{\mathrm{BDF}}$ is always evaluated with the exact right-hand side; but an error in $J$ affects the convergence behavior of the modified Newton iteration, so that the endpoint obtained under finite precision and a finite stopping test is not guaranteed to be identical, and it also governs how long one $J$ can be reused. Once formed, $J$ enters the iteration matrix of section 2.2 and can be reused across the iterations of one step and across neighboring steps; when $\gamma_{\mathrm{BDF}}$ changes, only $I_{n_x}-\gamma_{\mathrm{BDF}}J$ has to be re-formed and re-factored. When the differences are recomputed is decided by the solver's refresh policy, which is a solver setting.

### 2.4 Accuracy and stability

- For a sufficiently smooth problem, order-$k$ BDF has local truncation error $O(h^{k+1})$ and global error $O(h^k)$.
- BDF1 and BDF2 are A-stable. BDF3–BDF5 no longer cover the entire left half-plane, although they remain useful for stiff problems.
- BDF is a multistep method and requires startup history. Contact transitions or corners in force laws can reduce the observed high-order convergence.
- Dense output is obtained from a history polynomial over the most recent internal step and is not defined beyond that step.

### 2.5 Implementation in ORVD

`CvodeContinuousStateAdvancer` in [`cvode_continuous_state_advancer.cc`](../../../libs/integrators/src/cvode_continuous_state_advancer.cc) constructs the CVODE backend with `CV_BDF` and fixes the maximum order to either 2 or 5. Maximum orders 2 and 5 correspond to two method options of the public configuration. The map from the complete $[q;v;z]$ state to the system right-hand side is `SystemRhsBridge::CalcTimeDerivatives`, cited in section 1.1. Dense output is supplied by CVODE's history polynomial over the most recent internal step.

## 3. Radau5: three-stage fifth-order Radau IIA

Radau5 here means the three-stage fifth-order Radau IIA collocation method used by [Hairer's RADAU5](https://www.unige.ch/~hairer/software.html). Define

$$
c_1=\frac{4-\sqrt6}{10},
\qquad
c_2=\frac{4+\sqrt6}{10},
\qquad
c_3=1.
$$

### 3.1 Butcher tableau and stage equations

The Butcher tableau is

$$
\begin{array}{c|ccc}
c_1 & \frac{88-7\sqrt6}{360}
    & \frac{296-169\sqrt6}{1800}
    & \frac{-2+3\sqrt6}{225}\\
c_2 & \frac{296+169\sqrt6}{1800}
    & \frac{88+7\sqrt6}{360}
    & \frac{-2-3\sqrt6}{225}\\
1   & \frac{16-\sqrt6}{36}
    & \frac{16+\sqrt6}{36}
    & \frac{1}{9}\\
\hline
    & \frac{16-\sqrt6}{36}
    & \frac{16+\sqrt6}{36}
    & \frac{1}{9}
\end{array}.
$$

The three stages simultaneously satisfy

$$
Y_i=y_n+h\sum_{j=1}^{3}a_{ij}f(t_n+c_jh,Y_j),
\qquad i=1,2,3.
$$

The weights equal the last row of $A$, so the method is stiffly accurate:

$$
y_{n+1}=y_n+h\sum_{i=1}^{3}b_i f(t_n+c_i h,Y_i)=Y_3.
$$

### 3.2 Coupled Newton solution

Because $A$ is not lower triangular, the three stages form a coupled nonlinear system. Stack the three stage residuals as $\mathcal F_{\mathrm{stage}}$. The unreduced simplified-Newton linearization is

$$
\left(I_3\otimes I_{n_x}-hA\otimes J\right)\Delta
=-\mathcal F_{\mathrm{stage}}.
$$

It is unnecessary to factor this full $3n_x\times3n_x$ system. The one real eigenvalue and one complex-conjugate pair of $A^{-1}$ transform it into one real $n_x\times n_x$ system and one complex $n_x\times n_x$ system. The right-hand side is still evaluated at each stage state, while the Jacobian and linear factorizations can be reused across several Newton iterations or neighboring steps.

### 3.3 Error control and dense output

The principal Radau5 formula has order five and stage order three. A complete solver must also define stage initial guesses, a local-error estimator, step-size control and collocation-polynomial dense output. Together these algorithms define an actual Radau5 advance; the Butcher tableau alone does not. The high-order result assumes a sufficiently smooth solution and cannot be presumed at contact-state transitions.

### 3.4 Stability

For the linear test equation $y'=\lambda y$, let $\zeta=h\lambda$. The stability function is

$$
\mathcal R(\zeta)=
\frac{1+\frac{2}{5}\zeta+\frac{1}{20}\zeta^2}
{1-\frac{3}{5}\zeta+\frac{3}{20}\zeta^2-\frac{1}{60}\zeta^3}.
$$

The method is A-stable, and $\mathcal R(\zeta)\to0$ as $|\zeta|\to\infty$ in the left half-plane, so it is also L-stable. L-stability damps strongly decaying stiff modes in the large-step limit but does not remove order reduction caused by nonsmooth points.

### 3.5 Implementation in ORVD

`radau5::Core::AdvanceOneAcceptedStepToward` in [`radau5_core.cc`](../../../external/radau5/src/radau5_core.cc) implements only the ordinary-differential form $y'=f(t,y)$: the ODE mass matrix of the classical RADAU5 interface is the identity, the source carries no such term, and a general mass-matrix form is not supported. This is unrelated to the mechanical mass matrix $M(u)$ of section 1.2. The core uses a dense Jacobian, three-stage fifth-order Radau IIA, adaptive error control and collocation dense output over the latest successful step, and it performs simplified Newton iteration through one real and one complex linear system. `Radau5ContinuousStateAdvancer` in [`radau5_continuous_state_advancer.cc`](../../../libs/integrators/src/radau5_continuous_state_advancer.cc) connects that core to the same complete first-order state right-hand side used by BDF. Radau5 is likewise one method option of the public configuration.

The Radau5 core forms the dense Jacobian required by section 3.2 itself and does not borrow the column provider of section 2.3: `ComputeJacobian` of `radau5::Core` takes one-sided difference quotients of the same complete right-hand side, column by column, at the current accepted endpoint $(t_n,y_n)$; the perturbation is again applied to stored components, and $t_n$ and the inputs that are not part of the continuous state are held fixed. The nominal increment of column $j$ is

$$
\Delta_j=\sqrt{U_R\max\left(10^{-5},\lvert y_j\rvert\right)},
\qquad
U_R=10^{-16},
$$

where $U_R$ is the rounding-unit constant of this core; its meaning differs from CVODE's $U$ of section 2.3, and the two must not be substituted for each other. The constant $10^{-5}$ bounds the magnitude under the square root from below, so that the increment does not vanish with small $\lvert y_j\rvert$. Unlike section 2.3, the denominator is not the nominal increment but the actually representable one: `SelectRepresentablePerturbation` in the same file first forms the perturbed value $\hat y_j=y_j+\Delta_j$ in floating-point arithmetic, replacing it by the representable neighbor of $y_j$ if the sum leaves the stored value unchanged, and then sets $\tilde\Delta_j=\hat y_j-y_j$, so that

$$
J_{:,j}\approx\frac{f(t_n,\hat y)-f(t_n,y_n)}{\tilde\Delta_j},
$$

where $\hat y$ differs from $y_n$ only in component $j$. The denominator thus agrees with the perturbation actually applied, avoiding a mismatch between the nominal denominator and the actual perturbation; the rounding error of the right-hand-side evaluations themselves remains in the quotient. The resulting $J$ enters the real and complex systems of section 3.2 and is reused across Newton iterations and neighboring steps as described there.

## 4. Newmark: the average-acceleration method

### 4.1 Reference theory: the Newmark family

This section states the classical theory as the source of sections 4.2 to 4.6. ORVD implements only the average-acceleration member $\beta=1/4$, $\gamma=1/2$; the general parameter family, the linear solution through an effective stiffness and the explicit branch $\beta=0$ are **theory only**.

The [Newmark method](https://doi.org/10.1061/JMCEA3.0000098) parameterizes the displacement and velocity updates with endpoint accelerations. Given $u_n$, $v_n$, $a_n$ and two dimensionless parameters $\beta$ and $\gamma$, its basic formulas are

$$
u_{n+1}=u_n+h v_n+h^2\left[\left(\frac12-\beta\right)a_n+\beta a_{n+1}\right],
$$

$$
v_{n+1}=v_n+h\left[(1-\gamma)a_n+\gamma a_{n+1}\right].
$$

Implicit Newmark also requires the new endpoint to satisfy dynamic equilibrium,

$$
\mathbf r^{\mathrm{eq}}_{n+1}
=p_{n+1}-M a_{n+1}-C v_{n+1}
-f_{\mathrm{int}}(u_{n+1},v_{n+1},z_{n+1})=0,
$$

and the initial acceleration follows from initial equilibrium, $M a_0=p_0-Cv_0-f_{\mathrm{int}}(u_0,v_0,z_0)$.

For constant $M,C,K$, $f_{\mathrm{int}}=Ku$ and $\beta>0$, define $\kappa_0=1/(\beta h^2)$ and $\kappa_1=\gamma/(\beta h)$. The effective stiffness with $u_{n+1}$ as unknown is

$$
K_{\mathrm{eff}}=K+\kappa_1C+\kappa_0M.
$$

A nonlinear system iterates on the endpoint equilibrium residual; when $M$, $C$ or the loads depend on the state, a consistent tangent must include their derivatives with respect to $u_{n+1}$. The case $\beta=0$ is the explicit branch and cannot use an effective-stiffness formula containing $1/\beta$.

Parameters, accuracy and stability:

- With $\gamma=1/2$ the standard Newmark family is second order; $\gamma>1/2$ introduces algorithmic dissipation and usually reduces the order to one.
- For linear undamped systems, $2\beta\ge\gamma\ge1/2$ is the usual condition for unconditional stability.
- $\beta=1/4,\gamma=1/2$ is the average-acceleration method; it is unconditionally stable for linear undamped oscillators and has no algorithmic dissipation.
- $\beta=1/6,\gamma=1/2$ is the linear-acceleration method, whose stability is limited by the step size.

### 4.2 The coordinate form in ORVD

Substituting the coordinate system of section 1.3 into the updates with $\beta=1/4$, $\gamma=1/2$, and discretizing $z$ with the trapezoidal rule, gives

$$
q_1=q_0+h s_0+\frac{h^2}{4}(b_0+b_1),
\qquad
s_1=s_0+\frac h2(b_0+b_1),
\qquad
z_1=z_0+\frac h2(g_0+g_1).
$$

Subscript 0 denotes the accepted endpoint, where $b_0$ and $g_0$ are the evaluated $B$ and $G$; $b_1$ and $z_1$ are the unknowns. Substituting $s_1$ into $q_1$ gives

$$
q_1=q_0+\frac h2\,(s_0+s_1),
$$

Thus, when the endpoint equations are solved exactly and before projection, the scheme is equivalent to the trapezoidal rule applied to the first-order system $(q,s,z)$; this algebraic equivalence is not restricted to linear systems. The internal and mechanical variables share one discretization rule. The effects of finite solution residuals and endpoint projection on the published state must be considered separately from the accuracy and stability properties of the underlying scheme; see sections 4.4 and 4.6.

The endpoint equation is the residual in the unknown $x=(b_1,z_1)$,

$$
\mathcal R(x)=
\begin{bmatrix}
b_1-B(t_1,q_1,s_1,z_1)\\
z_1-z_0-\frac h2\left(g_0+G(t_1,q_1,s_1,z_1)\right)
\end{bmatrix}
=0,
$$

with $q_1$ and $s_1$ given by $b_1$ through the formulas above, and $t_1$ the endpoint time of section 6.1. Unlike the effective-stiffness form, the dynamics are not split into the linear structure $M$, $C$, $K$; Newton iteration acts on the complete residual, and all nonlinearity of contact, force elements and active loads stays inside $B$ and $G$.

### 4.3 Scaled Newton iteration

The iteration starts from the predictor $b_1^{(0)}=b_0$, $z_1^{(0)}=z_0+h\,g_0$. Each residual component has a scale $\rho_i$ and each unknown a reference magnitude $r_j$, and the residual is measured in the scaled maximum norm

$$
\lVert\mathcal R\rVert_{\rho}=\max_i\frac{\lvert\mathcal R_i\rvert}{\rho_i}.
$$

If the predictor already satisfies $\lVert\mathcal R\rVert_{\rho}\le1$, it is accepted without iteration. This branch is part of the scheme: an accepted endpoint satisfies the endpoint equation only in the sense of the scales.

Otherwise each iteration forms a fresh forward-difference $\partial\mathcal R/\partial x$ at the current iterate. The nominal increment of column $j$ is

$$
\Delta_j=\sqrt{U}\,\max\left(\lvert x_j\rvert,\,r_j\right),
$$

where $U$ is the double-precision constant `DBL_EPSILON`. As in the Radau5 core of section 3.5, the divisor is the increment actually realized by the floating-point addition; if the addition leaves the stored value unchanged, the adjacent representable value is used. The scaled linear system is

$$
\hat J_{ij}=\frac{r_j}{\rho_i}\,\frac{\partial\mathcal R_i}{\partial x_j},
\qquad
\hat J\,\hat\delta=-\left(\frac{\mathcal R_i}{\rho_i}\right)_i,
\qquad
\delta_j=r_j\,\hat\delta_j,
$$

solved by fully pivoted LU, after which $x\leftarrow x+\delta$. The correction is measured by its effect on the endpoint position and coordinate velocity:

$$
\lVert\delta\rVert_{\mathrm{corr}}=
\max\left(
\max_i\frac{h^2\lvert\delta b_i\rvert}{4\,\pi_i},\;
\max_i\frac{h\,\lvert\delta b_i\rvert}{2\,\sigma_i},\;
\max_k\frac{\lvert\delta z_k\rvert}{\zeta_k}
\right),
$$

where $\pi_i$, $\sigma_i$ and $\zeta_k$ are the position-correction, coordinate-velocity-correction and internal-variable-correction scales. An iterate is accepted when $\lVert\mathcal R\rVert_{\rho}\le1$ and $\lVert\delta\rVert_{\mathrm{corr}}\le1$.

The scales are given per coordinate family: translation, angle, quaternion and force each carry one set of scalars. The four quaternion components of a free body take the quaternion family and its three translational components the translation family; revolute and Ball-RPY coordinates take the angle family and prismatic coordinates the translation family; internal variables take the force family. The quaternion-family scalars are multiplied by the stored quaternion norm of the body at the latest successful initialization, so that the criteria match the stored dimension; initialization here includes synchronization. The reference magnitude $r_j$ is the acceleration reference for coordinates and the force reference for internal variables. These scales only determine the scaling of the Newton system, the stopping of the iteration and the difference perturbations; they are not global tolerances on the physical response.

When the iteration limit is reached without acceptance, the attempt is rejected and handled by section 4.5. A singular difference matrix, non-finite values and failures of the right-hand side or projection evaluations are not recoverable.

### 4.4 Endpoint projection

After acceptance, each free-body quaternion block receives a paired projection of position and coordinate velocity:

$$
\hat q=\frac{q_1}{\lVert q_1\rVert}\,\lVert q_{\mathrm{ref}}\rVert,
\qquad
\hat s=N(\hat q)\,N^{+}(q_1)\,s_1.
$$

The position is rescaled to the reference norm. The coordinate velocity is first recovered as a physical angular velocity on the unprojected configuration and then mapped back on the projected configuration, so the physical angular velocity is unchanged and $\hat s$ is orthogonal to $\hat q$. Other coordinate blocks are not projected. $q_{\mathrm{ref}}$ is the stored quaternion at the latest successful initialization.

When the projection actually changes the candidate, $B$ and $G$ are evaluated once more at the projected endpoint; otherwise the endpoint derivatives from the last residual evaluation of the converged iteration are kept. The history $b_1$, $g_1$ always contains the actual $B$, $G$ at the accepted endpoint. Without a projection change, the difference between the acceleration unknown and $B$ is bounded by the acceleration residual scales. With a change, that difference also includes the change in the derivative caused by projection, and history uses the newly evaluated values. $G$ itself is not a Newton unknown.

### 4.5 Bounded step recovery

Let the nominal step $H$ also be the maximum step. An attempt is rejected only when the core's own Newton iteration reaches its limit, with these rules:

- A rejected attempt of step $h$ is retried from the same accepted endpoint with $h/2$; the accepted state, endpoint derivatives, projection reference and scales are unchanged.
- A retry step may not fall below $H/1024$; if the next retry would, the advance ends with a nonlinear convergence failure.
- After every two consecutive accepted and published substeps, the planned step doubles, up to $H$.
- After a step change, the time grid is rebuilt from the current accepted endpoint; see section 6.1.
- A short step imposed by a genuine stop boundary is first attempted with the actual remaining time and may be below the floor; its success counts toward consecutive successes but does not set the planned step to that short step.
- A successful synchronization restores the planned step to $H$.

Each retry solves the same endpoint equations of section 4.2 with the actual step size and applies the same endpoint projection after convergence; recovery does not change the discretization formulas. The properties in section 4.6 remain subject to their stated conditions. The mechanism responds only to nonlinear solution failure; it estimates no local truncation error and does not adjust steps on such an estimate. It is not error adaptivity and gives no global accuracy guarantee.

### 4.6 Mathematical properties and conditions of applicability

- The trapezoidal rule is second order, A-stable and not L-stable. For the linear undamped oscillator $\ddot u=-\omega^2u$, $\omega>0$, the spectral radius of the amplification matrix equals 1 for every $h\omega$: the scheme is unconditionally stable without algorithmic dissipation, with relative phase error $O((h\omega)^2)$ as $h\omega\to0$. Such high-frequency oscillations receive no additional algorithmic damping; this does not exclude the effect of physical damping.
- For a stiff first-order decay $\dot z=-\lambda z$, the one-step amplification factor is $(1-h\lambda/2)/(1+h\lambda/2)$. When $h\lambda\gg1$ it tends to $-1$, so stiff Maxwell modes persist with alternating sign instead of being suppressed.
- Second-order accuracy requires a sufficiently smooth solution and corresponding control of nonlinear solution error; the amplification factors and stability analysis above concern linear equations and unprojected coordinate updates. For a vehicle they characterize the underlying scheme. Endpoint projection, contact switching and force-law kinks, finite Newton scales including zero-iteration acceptance, and the variable steps introduced by recovery must all be included when assessing the actual error.

### 4.7 Comparison with the classical form

| Item | Classical Newmark | Adopted in ORVD |
|---|---|---|
| Variables | Euclidean displacement, velocity, acceleration $u,v,a$ | Stored coordinates $q$, coordinate velocity $s$, coordinate acceleration $b$ |
| Parameters | General $\beta,\gamma$ | Fixed $\beta=1/4$, $\gamma=1/2$ |
| Acceleration | From $Ma=p-Cv-f_{\mathrm{int}}$ | $B$ from forward dynamics and the coordinate second-order map |
| Endpoint equation | Equilibrium residual; effective stiffness in the linear case | Complete residual $\mathcal R(b_1,z_1)$ with a fresh difference Jacobian at every iteration |
| Internal variables | Usually absent | $z$ discretized by the trapezoidal rule and solved together with the endpoint equation |
| Quaternions | Must be specified separately | Stored components integrated, paired projection at the endpoint |
| Step size | Updates use the current $h$ without specifying recovery | Nominal step with bounded step recovery |

## 5. Zhai's simple explicit two-step method

### 5.1 Reference theory

This section states the classical theory as the source of sections 5.2 and 5.3. ORVD adopts only two parameter sets, $\varphi=\psi=1/2$ for regular steps and $\varphi=\psi=0$ for startup steps; other parameter values are **theory only**.

[Zhai's simple explicit method](https://doi.org/10.1002/(SICI)1097-0207(19961230)39:24%3C4199::AID-NME39%3E3.0.CO;2-Y) uses the accelerations of the current and previous endpoints and introduces two dimensionless parameters $\varphi$ and $\psi$:

$$
u_{n+1}=u_n+h v_n+\left(\frac12+\psi\right)h^2a_n-\psi h^2a_{n-1},
$$

$$
v_{n+1}=v_n+(1+\varphi)h a_n-\varphi h a_{n-1}.
$$

The new acceleration then follows explicitly from the equation of motion:

$$
a_{n+1}=M^{-1}\left[p_{n+1}-C v_{n+1}-f_{\mathrm{int}}(u_{n+1},v_{n+1})\right].
$$

"Explicit" means that $u_{n+1}$ and $v_{n+1}$ are fixed by history before $a_{n+1}$ is computed. The original method asks for a diagonal mass matrix so that accelerations are obtained degree of freedom by degree of freedom without solving coupled equations; this is a requirement of the computational organization, not a physical assumption.

A two-step method has no $a_{-1}$ at the start. The usual self-starting step takes $\varphi=\psi=0$:

$$
u_1=u_0+h v_0+\frac12h^2a_0,
\qquad
v_1=v_0+h a_0,
$$

where $a_0$ follows from initial dynamic equilibrium. Regular steps commonly take $\varphi=\psi=1/2$. Only after a new endpoint enters the method history do $a_n$ and $a_{n-1}$ roll forward; when the step size or the equation changes, the uniform-step two-step coefficients cannot be reused.

Accuracy and stability:

- The common form $\varphi=\psi=1/2$ is second order. For an undamped linear oscillator the spectral radius of the amplification matrix equals 1 for $h\omega<2$, with no numerical dissipation but with phase error, and the scheme is unstable for $h\omega>2$. This is the result the original paper gives for the undamped single-degree-of-freedom problem.
- For the constant linear system $M\ddot u+Ku=0$, symmetric positive-definite $M$ and $K$ permit a real modal transformation into independent undamped scalar oscillators, giving $h\omega_{\max}<2$ mode by mode. These are sufficient conditions for extending the bound to multiple degrees of freedom. Gyroscopic terms, nonconservative forces or internal-variable coupling prevent direct use of this decoupling argument.
- Damping imposes further limits. For an isolated scalar decay $\dot v=-\mu v$, $\mu>0$, the velocity update is the second-order Adams–Bashforth method, with decay stability interval $0<h\mu<1$; strongly damped time scales can also limit the step.
- For linear stability analysis of a general coupled system, construct the augmented amplification matrix from the actual two-step recurrence and check its spectral radius and root conditions on the unit circle. The position and velocity rows have different discretization coefficients, so inserting the eigenvalues of the first-order physical Jacobian into a single scalar stability region is insufficient. Wheel–rail contact stiffness, suspension damping and first-order internal variables can all affect this criterion.
- The original formulas are a fixed uniform-step method; variable steps, shortened final steps and dense output need their own definitions.

### 5.2 The coordinate form in ORVD

For the coordinate system of section 1.3, regular steps take $\varphi=\psi=1/2$ and the internal variables use the second-order Adams–Bashforth method:

$$
q_1=q_0+h s_0+h^2\left(b_0-\frac12 b_{-1}\right),
\qquad
s_1=s_0+h\left(\frac32 b_0-\frac12 b_{-1}\right),
\qquad
z_1=z_0+h\left(\frac32 g_0-\frac12 g_{-1}\right).
$$

The first step, and any step whose actual size differs from the previous one, uses the self-starting form

$$
q_1=q_0+h s_0+\frac{h^2}{2}\,b_0,
\qquad
s_1=s_0+h\,b_0,
\qquad
z_1=z_0+h\,g_0,
$$

that is, $\varphi=\psi=0$ and explicit Euler for the internal variables. After forming $q_1$ and $s_1$, the endpoint projection of section 4.4 is applied first, and $B$ and $G$ are then evaluated once at the projected endpoint to become the history of the next step. Each successful step evaluates the complete right-hand side once, with no Newton iteration or difference Jacobian construction.

Because forward dynamics supplies the acceleration, the scheme stays explicit without a diagonal mass matrix; forward dynamics still performs linear algebra internally. Inside every right-hand-side evaluation, the contact computations of the individual wheel–rail interfaces are mutually independent and can proceed concurrently; this is a structural property of the right-hand side shared by all five method options. A Zhai step also includes the state recurrence, coordinate conversion, endpoint projection and accepted-state commit.

### 5.3 Mathematical properties and conditions of applicability

- For smooth problems within the stable step-size range, a regular step has local state error $O(h^3)$ and global order two. A startup step has local state error $O(h^2)$ in velocity and internal variables; second-order accuracy can be retained if the number of startups in a fixed time interval stays bounded under step refinement. A fixed control-event schedule meets this counting condition but still requires smoothness within each segment, stable error propagation and consistent event handling. If every step restarts because its size changes, velocity and internal variables continually use Euler updates and the order generally falls to one.
- The bound $h\omega_{\max}<2$ of section 5.1 applies to decoupled undamped oscillators. An isolated Maxwell internal-variable decay $\dot z=-(k/c)z$ advanced by AB2 requires $0<h<c/k$ for decay stability. Once mechanics and internal variables are coupled, the actual restriction must be analyzed through the augmented amplification matrix of the complete recurrence, rather than by checking isolated subproblems or the spectrum of the first-order physical Jacobian alone.
- The absence of algorithmic dissipation applies only to normal uniform-step recurrences for undamped linear mechanical oscillators; it does not extend to general internal-variable coupling, startup or updates after projection.
- Endpoint projection, contact switching and force-law kinks likewise make the actual error depart from these asymptotic statements.

### 5.4 Comparison with the classical form

| Item | Classical Zhai | Adopted in ORVD |
|---|---|---|
| Variables | Euclidean displacement, velocity, acceleration $u,v,a$ | Stored coordinates $q$, coordinate velocity $s$, coordinate acceleration $b$ |
| Parameters | General $\varphi,\psi$ | $1/2$ on regular steps, $0$ on startup steps |
| Acceleration | $M^{-1}(\cdots)$, preferably with diagonal $M$ | Forward dynamics and the coordinate second-order map |
| Internal variables | Usually absent | $z$ by AB2, Euler on startup steps |
| Quaternions | Must be specified separately | Paired projection at the endpoint before evaluation |
| Startup | Self-starting first step | On the first step, after a step-size change or after synchronization |

## 6. Common parts of the coordinate-form methods

### 6.1 Time grid and stop steps

Both methods build endpoint times from a grid origin $t_{\mathrm a}$ and an integer step index $k$ as $t_k=t_{\mathrm a}+k\,h$, computed with one fused multiply-add rather than by accumulating floating-point steps. When a grid endpoint differs from an external stop time only by floating-point rounding, the endpoint takes the stop time exactly; the formula coefficients still use $h$, and the right-hand side is evaluated at that endpoint time. When a stop time falls between two grid endpoints, the final step takes the actual remaining time. After a step change or on reaching a stop boundary, the grid restarts from the current accepted endpoint.

### 6.2 Sampling interpolation

Sample times do not become integration stops. Samples inside an interval are linear interpolations, component by component, of the accepted physical states at its two ends. A quaternion block first normalizes both ends, flips one of them if necessary so that both lie in the same hemisphere, interpolates linearly, normalizes and multiplies by the reference norm. The two ends of the interval are copied exactly. Interpolation evaluates no right-hand side and changes neither the method history nor the work counts.

### 6.3 Initialization and synchronization

Initialization does not repair an invalid state: the coordinate velocity of a quaternion block must be orthogonal to the stored quaternion. Every successful initialization, including the synchronization after a change of external inputs, evaluates $B$ and $G$ at the given state, takes the given quaternions as the projection reference and re-expands the quaternion scales with their norms. Newmark restores the planned step to $H$; Zhai clears its two-step history, so the next step uses the self-starting form.

## 7. Method comparison and source map

| Method | Basic equation | Main order | Implicitness | Stability | History structure |
|---|---|---:|---|---|---|
| CVODE BDF | First-order $\dot y=f(t,y)$ | 1–2 or 1–5 | Implicit | BDF1–2 are A-stable | Multistep history |
| Radau5 | First-order $\dot y=f(t,y)$ | 5 | Three-stage fully implicit | A-stable and L-stable | One-step stages and linearization history |
| Newmark average acceleration | Coordinate second-order $(q,s,z)$ | 2 | Implicit, full Newton | Underlying trapezoidal scheme is A-stable; no algorithmic dissipation for undamped oscillators | One-step endpoint derivatives |
| Zhai | Coordinate second-order $(q,s,z)$ | 2 | Explicit | Conditionally stable | Two-step acceleration history |

The orders and stability properties in the table refer to the underlying schemes under the conditions stated in their respective sections. BDF and Radau5 consume ORVD's complete first-order right-hand side directly; Newmark and Zhai consume the same right-hand side through the coordinate bridge of section 1.3. All five method options are selected explicitly through the public system-integration configuration; the library has no implicit default method.

| Theoretical object | Main implementation |
|---|---|
| Conversion between coordinate and physical states, evaluation of $B$ and $G$, endpoint projection, sampling interpolation | `MakeCoordinateState`, `CopyPhysicalState`, `Evaluate`, `ProjectEndpoint` and `CopyLinearlyInterpolatedPhysicalState` of `SystemCoordinateProblem` in [`system_coordinate_problem.cc`](../../../libs/integrators/src/system_coordinate_problem.cc) |
| Rate map, left pseudo-inverse and coordinate second derivative | `MapGeneralizedVelocitiesToPositionDerivatives`, `MapGeneralizedPositionDerivativesToVelocities` and `MapGeneralizedVelocityDerivativesToPositionSecondDerivatives` of `MultibodyModel` in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Accepted state, candidate state, projection reference and endpoint commit | `CoordinateCoreState` in [`coordinate_core_state.h`](../../../libs/integrators/src/coordinate_core_state.h) |
| Newmark residual, predictor, difference Jacobian, scaled Newton and correction norm | `Residual`, `Solve`, `CorrectionNorm` and `Advance` in the `NewmarkCore` implementation, [`newmark_core.cc`](../../../libs/integrators/src/newmark_core.cc) |
| Expansion of the four scale families to coordinates | `NewmarkCoordinateLayout::Expand` in [`newmark_coordinate_layout.cc`](../../../libs/integrators/src/newmark_coordinate_layout.cc) |
| Bounded step recovery | `NewmarkRecoveryStepPolicy` in [`coordinate_step_policy.h`](../../../libs/integrators/src/coordinate_step_policy.h) |
| Zhai regular and self-starting steps | `AdvanceOneStep` in the `ZhaiCore` implementation, [`zhai_core.cc`](../../../libs/integrators/src/zhai_core.cc) |
| Time grid, stop steps, publication, sampling and synchronization | `ChooseStep`, `Advance`, `CopyDenseState` and `Reinitialize` of `BasicCoordinateAdvancerImplementation` in [`basic_coordinate_advancer.h`](../../../libs/integrators/src/basic_coordinate_advancer.h); the endpoint-time criteria in [`coordinate_step_time.h`](../../../libs/integrators/src/coordinate_step_time.h) |
| Public method configuration | `NewmarkConfiguration` and `ZhaiConfiguration` in [`mechanical_integration_configuration.h`](../../../libs/integrators/include/orvd/integrators/mechanical_integration_configuration.h); `SystemIntegrationConfiguration` in [`system_integration_configuration.h`](../../../libs/integrators/include/orvd/integrators/system_integration_configuration.h) |
