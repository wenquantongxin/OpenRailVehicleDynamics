[中文](TIME_INTEGRATION_METHODS.md)

# BDF, Radau5, Newmark and Zhai time-integration methods

This chapter explains how BDF, the three-stage fifth-order Radau IIA method, Newmark methods and Zhai's simple explicit method advance continuous equations of motion to discrete states. It also discusses their error, stability and compatibility with ORVD's state structure. The ORVD production system currently uses CVODE BDF with maximum order 2, while the source tree also contains CVODE BDF with maximum order 5 and a Radau5 implementation. Newmark and Zhai have not been implemented and are marked **theory only** here.

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

Here $u$ is Euclidean displacement, used as in sections 4 and 5 and distinct from the generalized position $q$ of section 1.1; $a$ is acceleration and $p$ is the external load. $M$ is the system mass matrix. The product $C(u,v)v$ collects velocity-dependent inertial terms such as Coriolis and centrifugal effects together with viscous damping. $f_{\mathrm{int}}$ holds the remaining internal forces, including elastic restoring forces and element forces that depend on the internal variables $z$.

This form is natural for Euclidean coordinates in which displacement, velocity and acceleration have equal dimensions. A general multibody system must instead update its configuration through a tangent-space increment and a retraction, while first-order internal variables $z$ require their own discrete equations. Without these extensions, directly applying a first-order integration formula to $[q;v;z]$ produces a different first-order state method rather than the classical Newmark or Zhai method.

### 1.3 Error scaling

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

Here $U$ is CVODE's unit roundoff, `DBL_EPSILON` in double precision; $w_j$ is the error weight the solver holds for the current step and keeps fixed throughout this batch of differences, formed as in section 1.3 from `rtol` and `atol` at the step's reference state $y^{(w)}$, which need not be the trial point at which the differences are taken, and $\lVert\cdot\rVert_{\mathrm{WRMS}}$ is the weighted norm of the same section; $h$ is the current internal step size; and $\mu=1000$ is the dimensionless constant that defines the increment floor $\Delta_{\min}$. The first term makes the increment proportional to the magnitude of the component itself, the usual compromise of a one-sided difference between truncation and roundoff error. The second term keeps the increment from vanishing as $y_j$ approaches zero: $\Delta_{\min}$ measures, through $\lvert h\rvert\,\lVert f\rVert_{\mathrm{WRMS}}$, the weighted size of the state change over one step, and $1/w_j=\operatorname{rtol}\lvert y^{(w)}_j\rvert+\operatorname{atol}_j$ converts it to the units of component $j$. The denominator of the quotient is the nominal increment $\Delta_j$, not the increment actually realized after floating-point addition; this differs from the Radau5 core of section 3.5.

In the implementation the differencing is organized in one of two ways, both of which define the same finite-difference Jacobian by the same increment rule and the same denominator. The first is the built-in difference quotient of CVODE's dense linear solver. The second is ORVD's own column provider: `DenseFiniteDifferenceJacobianProvider` in [`dense_finite_difference_jacobian_provider.h`](../../../libs/integrators/src/dense_finite_difference_jacobian_provider.h) is responsible only for the perturbed derivatives $f(t,y+\Delta_j\mathbf e_j)$ of the columns, and its system implementation, `CalcPerturbedDerivatives` in [`system_continuous_state_backend.cc`](../../../libs/integrators/src/system_continuous_state_backend.cc), copies $y$ for each column in a separate trial context, adds $\Delta_j$ and calls the complete right-hand side of section 1.1; the increments $\Delta_j$, the baseline derivative $f(t,y)$ and the quotient itself are held by `EvaluateDenseJacobian` in [`cvode_continuous_state_advancer.cc`](../../../libs/integrators/src/cvode_continuous_state_advancer.cc) according to the formulas above, and the provider contains no differencing formula.

The finite-difference Jacobian does not change the BDF nonlinear residual equation itself: $\mathcal F_{\mathrm{BDF}}$ is always evaluated with the exact right-hand side; but an error in $J$ affects the convergence behavior of the modified Newton iteration, so that the endpoint obtained under finite precision and a finite stopping test is not guaranteed to be identical, and it also governs how long one $J$ can be reused. Once formed, $J$ enters the iteration matrix of section 2.2 and can be reused across the iterations of one step and across neighboring steps; when $\gamma_{\mathrm{BDF}}$ changes, only $I_{n_x}-\gamma_{\mathrm{BDF}}J$ has to be re-formed and re-factored. When the differences are recomputed is decided by the solver's refresh policy, which is a solver setting.

### 2.4 Accuracy and stability

- For a sufficiently smooth problem, order-$k$ BDF has local truncation error $O(h^{k+1})$ and global error $O(h^k)$.
- BDF1 and BDF2 are A-stable. BDF3–BDF5 no longer cover the entire left half-plane, although they remain useful for stiff problems.
- BDF is a multistep method and requires startup history. Contact transitions or corners in force laws can reduce the observed high-order convergence.
- Dense output is obtained from a history polynomial over the most recent internal step and is not defined beyond that step.

### 2.5 Implementation in ORVD

`CvodeContinuousStateAdvancer` in [`cvode_continuous_state_advancer.cc`](../../../libs/integrators/src/cvode_continuous_state_advancer.cc) constructs the CVODE backend with `CV_BDF` and fixes the maximum order to either 2 or 5. The production system currently uses the maximum-order-two form, while the source tree also retains the maximum-order-five form. The map from the complete $[q;v;z]$ state to the system right-hand side is `SystemRhsBridge::CalcTimeDerivatives`, cited in section 1.1. Dense output is supplied by CVODE's history polynomial over the most recent internal step.

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

`radau5::Core::AdvanceOneAcceptedStepToward` in [`radau5_core.cc`](../../../external/radau5/src/radau5_core.cc) implements only the ordinary-differential form $y'=f(t,y)$: the ODE mass matrix of the classical RADAU5 interface is the identity, the source carries no such term, and a general mass-matrix form is not supported. This is unrelated to the mechanical mass matrix $M(u)$ of section 1.2. The core uses a dense Jacobian, three-stage fifth-order Radau IIA, adaptive error control and collocation dense output over the latest successful step, and it performs simplified Newton iteration through one real and one complex linear system. `Radau5ContinuousStateAdvancer` in [`radau5_continuous_state_advancer.cc`](../../../libs/integrators/src/radau5_continuous_state_advancer.cc) connects that core to the same complete first-order state right-hand side used by BDF. Radau5 is implemented in the source tree, while the production system currently remains on CVODE BDF with maximum order 2.

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

## 4. Newmark: a family of one-step methods for second-order mechanical systems (theory only)

The [Newmark method](https://doi.org/10.1061/JMCEA3.0000098) parameterizes displacement and velocity updates by endpoint acceleration. Given $u_n$, $v_n$, $a_n$ and the two dimensionless parameters $\beta$ and $\gamma$, its basic formulas are

$$
u_{n+1}=u_n+h v_n+h^2\left[\left(\frac12-\beta\right)a_n+\beta a_{n+1}\right],
$$

$$
v_{n+1}=v_n+h\left[(1-\gamma)a_n+\gamma a_{n+1}\right].
$$

### 4.1 Implicit equilibrium

Implicit Newmark also requires equilibrium at the new endpoint:

$$
\mathbf r^{\mathrm{eq}}_{n+1}
=p_{n+1}-M a_{n+1}-C v_{n+1}
-f_{\mathrm{int}}(u_{n+1},v_{n+1},z_{n+1})=0.
$$

The initial acceleration follows from the initial dynamic equilibrium, for example

$$
M a_0=p_0-Cv_0-f_{\mathrm{int}}(u_0,v_0,z_0).
$$

### 4.2 Effective stiffness for a linear system

For constant $M,C,K$, $f_{\mathrm{int}}=Ku$ and $\beta>0$, define

$$
\kappa_0=\frac{1}{\beta h^2},
\qquad
\kappa_1=\frac{\gamma}{\beta h}.
$$

With $u_{n+1}$ as the unknown, the effective stiffness is

$$
K_{\mathrm{eff}}=K+\kappa_1C+\kappa_0M.
$$

A nonlinear system iterates on the endpoint equilibrium residual. If $M$, $C$ or the load depends on state, a consistent tangent also contains the derivatives of those terms with respect to $u_{n+1}$. The case $\beta=0$ belongs to the explicit Newmark branch and cannot use an effective-stiffness expression containing $1/\beta$.

### 4.3 Parameters, accuracy and stability

- The standard Newmark family is second order when $\gamma=1/2$; $\gamma>1/2$ introduces algorithmic dissipation and is generally first order.
- For a linear undamped system, $2\beta\ge\gamma\ge1/2$ is a commonly used unconditional-stability condition.
- $\beta=1/4,\gamma=1/2$ gives the average-acceleration method, which is unconditionally stable for a linear system and has no algorithmic high-frequency dissipation.
- $\beta=1/6,\gamma=1/2$ gives the linear-acceleration method, whose stability is step-size limited.

### 4.4 Relation to ORVD's state

The classical formulas assume equal-dimensional $u$, $v$ and $a$ with $\dot u=v$. Applying Newmark to a complete ORVD vehicle would require forming configuration increments from tangent-space velocity and acceleration, retracting them to a $q$ that contains quaternion and Ball-RPY coordinates, then defining a discrete equation for $z$ coupled to endpoint equilibrium. The effective-stiffness formula for a Euclidean linear structure therefore cannot directly serve as the discrete equation for the complete ORVD multibody model.

## 5. Zhai's simple explicit two-step method (theory only)

[Zhai's simple explicit method](https://doi.org/10.1002/(SICI)1097-0207(19961230)39:24%3C4199::AID-NME39%3E3.0.CO;2-Y) uses the current and previous endpoint accelerations and introduces two dimensionless parameters $\phi$ and $\psi$:

$$
u_{n+1}=u_n+h v_n+\left(\frac12+\psi\right)h^2a_n-\psi h^2a_{n-1},
$$

$$
v_{n+1}=v_n+(1+\phi)h a_n-\phi h a_{n-1}.
$$

The new acceleration is then obtained explicitly from the equation of motion:

$$
a_{n+1}=M^{-1}\left[p_{n+1}-C v_{n+1}-f_{\mathrm{int}}(u_{n+1},v_{n+1})\right].
$$

Here “explicit” means that $u_{n+1}$ and $v_{n+1}$ are already determined from history before $a_{n+1}$ is evaluated. Retaining the original method's computational form without solving simultaneous algebraic equations additionally requires a diagonal mass matrix $M$.

### 5.1 Startup and history

The two-step method has no $a_{-1}$ at startup. A common self-starting choice takes $\phi=\psi=0$:

$$
u_1=u_0+h v_0+\frac12h^2a_0,
\qquad
v_1=v_0+h a_0,
$$

where $a_0$ follows from initial dynamic equilibrium. Normal steps commonly use $\phi=\psi=1/2$ and calculate $a_{n+1}$ after forming $u_{n+1}$ and $v_{n+1}$. The pair $a_n$ and $a_{n-1}$ advances only when the new endpoint becomes part of the method history. If the step size or state equation changes, the uniform-step two-step coefficients cannot continue unchanged.

### 5.2 Accuracy and stability

- The common choice $\phi=\psi=1/2$ is second order and has no numerical dissipation in linear undamped analysis, although it has phase error.
- For an undamped linear oscillator, this parameter choice is stable when $h\omega<2$, equivalently $h<T_{\min}/\pi$.
- The highest resolved frequency limits the explicit step size. Wheel-rail contact stiffness, suspension stiffness and first-order internal variables can all contribute high-frequency time scales.
- The original formula is a fixed-uniform-step method. Variable steps, a shortened terminal step and dense output each require separately defined mathematical formulas.

### 5.3 Relation to ORVD's state

For a complete ORVD vehicle, the Zhai method would likewise need to construct a configuration increment in the tangent space and retract it to the new $q$, while specifying a discrete method for Maxwell and other internal variables $z$. Applying AB2 directly to the complete $[q;v;z]$ state produces an ordinary first-order multistep method; it is not Zhai's simple explicit method.

## 6. Comparison of the methods

| Method | Governing form | Principal order | Implicitness | Stability point | History structure | ORVD implementation status |
|---|---|---:|---|---|---|---|
| CVODE BDF | Complete first-order $\dot y=f(t,y)$ | 1–5 | Implicit | BDF1–2 are A-stable | Multistep history | Production maximum order is 2; a maximum-order-five implementation also exists in the source tree |
| Radau5 | Complete first-order $\dot y=f(t,y)$ | 5 | Three-stage fully implicit | A-stable and L-stable | One-step stages and linearization history | Implemented in the source tree; not currently used by the production system |
| Newmark | Second-order mechanical equilibrium | Usually 2 | Common forms are implicit | Depends on $\beta,\gamma$ | One-step endpoint quantities | **Theory only** |
| Zhai simple explicit method | Second-order mechanical acceleration | 2 | Explicit | Limited by the highest frequency | Two-step acceleration history | **Theory only** |

BDF and Radau5 can consume ORVD's complete first-order right-hand side directly. The original Newmark and Zhai formulas exploit second-order mechanical structure. Before either is applied to a vehicle model containing manifold configurations and first-order internal variables, its extended discrete equations must be defined explicitly; otherwise the method's name no longer describes the algorithm actually being used.
