[中文](TIME_INTEGRATION_METHODS.md)

# BDF, Radau5, Newmark, and Zhai time-integration methods

This chapter explains how BDF, the three-stage fifth-order Radau IIA method, Newmark methods, and Zhai's simple explicit method advance continuous equations of motion to discrete states. It also discusses their error, stability, and compatibility with ORVD's state structure. ORVD currently uses CVODE BDF2 as its default backend and also retains source-tree BDF5 and Radau5 implementations; Newmark and Zhai have not been implemented and are marked **theory only** here.

## 1. Equation forms and common notation

### 1.1 First-order state equation

BDF and Radau5 act directly on the first-order initial-value problem

$$
\dot y=f(t,y),
\qquad
y(t_0)=y_0,
\qquad
y\in\mathbb R^N.
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

Here $q$ is generalized position, $v$ is generalized velocity, and $z$ contains first-order internal variables of force elements such as Maxwell spring-damper series elements. When a free body's orientation is represented by a quaternion, $q$ and $v$ have different dimensions and the configuration kinematics are $\dot q=N(q)v$; the complete state therefore cannot be reduced to $\dot q=v$. The state-to-derivative mapping is defined in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h). For example, a Maxwell force state satisfies

$$
\dot F=K v_{\mathrm{rel}}-\frac{K}{C}F.
$$

### 1.2 Second-order mechanical equation

Classical Newmark and Zhai methods are usually formulated from the second-order mechanical equation

$$
M(q)a+C(q,v)v+f_{\mathrm{int}}(q,v,z)=p(t),
\qquad
\dot q=v.
$$

This form is natural for Euclidean coordinates in which displacement, velocity, and acceleration have equal dimensions. A general multibody system must instead update its configuration through a tangent-space increment and a retraction, while first-order internal variables $z$ require their own discrete equations. Without these extensions, directly applying a first-order integration formula to $[q;v;z]$ produces a different first-order state method rather than the classical Newmark or Zhai method.

### 1.3 Error scaling

State components have different physical dimensions. An adaptive method can combine a relative tolerance and componentwise absolute tolerances into the weights

$$
w_i=\frac{1}{\operatorname{rtol}|y_i|+\operatorname{atol}_i},
\qquad
\lVert e\rVert_{\mathrm{WRMS}}
=\sqrt{\frac{1}{N}\sum_{i=1}^{N}(w_i e_i)^2}.
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
R(y_{n+1})=y_{n+1}-\gamma f(t_{n+1},y_{n+1})-a_n=0,
$$

where $a_n$ is determined by accepted history and $\gamma$ by the current step size and BDF coefficients. Newton or modified Newton iteration solves

$$
\left(I-\gamma J\right)\delta=-R,
\qquad
J=\frac{\partial f}{\partial y},
\qquad
y^{(m+1)}=y^{(m)}+\delta.
$$

An adaptive BDF step performs historical extrapolation, nonlinear solution, local-error estimation, and selection of the next step size and order. The method history advances only after the new endpoint is accepted. When the state equation or an externally held quantity changes, the old history no longer represents the same initial-value problem and must be reconstructed from the current endpoint.

### 2.3 Accuracy and stability

- For a sufficiently smooth problem, order-$k$ BDF has local truncation error $O(h^{k+1})$ and global error $O(h^k)$.
- BDF1 and BDF2 are A-stable. BDF3–BDF5 no longer cover the entire left half-plane, although they remain useful for stiff problems.
- BDF is a multistep method and requires startup history. Contact transitions or corners in force laws can reduce the observed high-order convergence.
- Dense output is obtained from a history polynomial over the most recent internal step and is not defined beyond that step.

### 2.4 Implementation in ORVD

[`cvode_continuous_state_advancer.cc`](../../../libs/integrators/src/cvode_continuous_state_advancer.cc) constructs the CVODE backend with `CV_BDF` and fixes the maximum order to either 2 or 5. The public system advancer selects BDF2, while BDF5 is a named source-tree implementation. [`system_rhs_bridge.cc`](../../../libs/integrators/src/system_rhs_bridge.cc) maps the complete $[q;v;z]$ state to the multibody-dynamics right-hand side, and [`continuous_state_advancer.h`](../../../libs/integrators/include/orvd/integrators/continuous_state_advancer.h) carries the common mathematical results for an accepted internal step and dense output over the latest step.

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

Because $A$ is not lower triangular, the three stages form a coupled nonlinear system. The unreduced simplified-Newton linearization is

$$
\left(I_3\otimes I_N-hA\otimes J\right)\Delta=-R.
$$

It is unnecessary to factor this full $3N\times3N$ system. The one real eigenvalue and one complex-conjugate pair of $A^{-1}$ transform it into one real $N\times N$ system and one complex $N\times N$ system. The right-hand side is still evaluated at each stage state, while the Jacobian and linear factorizations can be reused across several Newton iterations or neighboring steps.

### 3.3 Error control and dense output

The principal Radau5 formula has order five and stage order three. A complete solver must also define stage initial guesses, a local-error estimator, step-size control, and collocation-polynomial dense output. Together these algorithms define an actual Radau5 advance; the Butcher tableau alone does not. The high-order result assumes a sufficiently smooth solution and cannot be presumed at contact-state transitions.

### 3.4 Stability

For the linear test equation $y'=\lambda y$, let $\zeta=h\lambda$. The stability function is

$$
\mathcal R(\zeta)=
\frac{1+\frac{2}{5}\zeta+\frac{1}{20}\zeta^2}
{1-\frac{3}{5}\zeta+\frac{3}{20}\zeta^2-\frac{1}{60}\zeta^3}.
$$

The method is A-stable, and $\mathcal R(\zeta)\to0$ as $|\zeta|\to\infty$ in the left half-plane, so it is also L-stable. L-stability damps strongly decaying stiff modes in the large-step limit but does not remove order reduction caused by nonsmooth points.

### 3.5 Implementation in ORVD

[`radau5_core.cc`](../../../external/radau5/src/radau5_core.cc) implements forward first-order ordinary differential equations with $M=I$, a dense Jacobian, three-stage fifth-order Radau IIA, adaptive error control, and collocation dense output over the latest successful step. It performs simplified Newton iteration through one real and one complex linear system. [`radau5_continuous_state_advancer.cc`](../../../libs/integrators/src/radau5_continuous_state_advancer.cc) connects that core to the same complete first-order state right-hand side used by BDF. Radau5 therefore has a source-tree implementation, but it is not the public default backend.

## 4. Newmark: a family of one-step methods for second-order mechanical systems (theory only)

The [Newmark method](https://doi.org/10.1061/JMCEA3.0000098) parameterizes displacement and velocity updates by endpoint acceleration. Given $u_n$, $v_n$, and $a_n$, its basic formulas are

$$
u_{n+1}=u_n+h v_n+h^2\left[\left(\frac12-\beta\right)a_n+\beta a_{n+1}\right],
$$

$$
v_{n+1}=v_n+h\left[(1-\gamma)a_n+\gamma a_{n+1}\right].
$$

### 4.1 Implicit equilibrium

Implicit Newmark also requires equilibrium at the new endpoint:

$$
R_{n+1}=p_{n+1}-M a_{n+1}-C v_{n+1}-f_{\mathrm{int}}(u_{n+1},v_{n+1},z_{n+1})=0.
$$

The initial acceleration follows from the initial dynamic equilibrium, for example

$$
M a_0=p_0-Cv_0-f_{\mathrm{int}}(u_0,v_0,z_0).
$$

### 4.2 Effective stiffness for a linear system

For constant $M,C,K$, $f_{\mathrm{int}}=Ku$, and $\beta>0$, define

$$
\kappa_0=\frac{1}{\beta h^2},
\qquad
\kappa_1=\frac{\gamma}{\beta h},
$$

With $u_{n+1}$ as the unknown, the effective stiffness is

$$
K_{\mathrm{eff}}=K+\kappa_1C+\kappa_0M.
$$

A nonlinear system iterates on the endpoint equilibrium residual. If $M$, $C$, or the load depends on state, a consistent tangent also contains the derivatives of those terms with respect to $u_{n+1}$. The case $\beta=0$ belongs to the explicit Newmark branch and cannot use an effective-stiffness expression containing $1/\beta$.

### 4.3 Parameters, accuracy, and stability

- The standard Newmark family is second order when $\gamma=1/2$; $\gamma>1/2$ introduces algorithmic dissipation and is generally first order.
- For a linear undamped system, $2\beta\ge\gamma\ge1/2$ is a commonly used unconditional-stability condition.
- $\beta=1/4,\gamma=1/2$ gives the average-acceleration method, which is unconditionally stable for a linear system and has no algorithmic high-frequency dissipation.
- $\beta=1/6,\gamma=1/2$ gives the linear-acceleration method, whose stability is step-size limited.

### 4.4 Relation to ORVD's state

The classical formulas assume equal-dimensional $u$, $v$, and $a$ with $\dot u=v$. Applying Newmark to a complete ORVD vehicle would require forming configuration increments from tangent-space velocity and acceleration, retracting them to a $q$ that contains quaternion and Ball-RPY coordinates, and defining a discrete equation for $z$ coupled to endpoint equilibrium. The effective-stiffness formula for a Euclidean linear structure therefore cannot directly serve as the discrete equation for the complete ORVD multibody model.

## 5. Zhai's simple explicit two-step method (theory only)

Zhai's simple explicit method, introduced in 1996, uses the current and previous endpoint accelerations:

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
- The highest resolved frequency limits the explicit step size. Wheel-rail contact stiffness, suspension stiffness, and first-order internal variables can all contribute high-frequency time scales.
- The original formula is a fixed-uniform-step method. Variable steps, a shortened terminal step, and dense output each require separately defined mathematical formulas.

### 5.3 Relation to ORVD's state

For a complete ORVD vehicle, the Zhai method would likewise need to construct a configuration increment in the tangent space and retract it to the new $q$, while specifying a discrete method for Maxwell and other internal variables $z$. Applying AB2 directly to the complete $[q;v;z]$ state produces an ordinary first-order multistep method and should not be called Zhai's simple explicit method.

## 6. Comparison of the methods

| Method | Governing form | Principal order | Implicitness | Stability point | History structure | ORVD implementation status |
|---|---|---:|---|---|---|---|
| CVODE BDF | Complete first-order $\dot y=f(t,y)$ | 1–5 | Implicit | BDF1–2 are A-stable | Multistep history | BDF2 default; source-tree BDF5 implementation |
| Radau5 | Complete first-order $\dot y=f(t,y)$ | 5 | Three-stage fully implicit | A-stable and L-stable | One-step stages and linearization history | Source-tree implementation, not default |
| Newmark | Second-order mechanical equilibrium | Usually 2 | Common forms are implicit | Depends on $\beta,\gamma$ | One-step endpoint quantities | **Theory only** |
| Zhai simple explicit method | Second-order mechanical acceleration | 2 | Explicit | Limited by the highest frequency | Two-step acceleration history | **Theory only** |

BDF and Radau5 can consume ORVD's complete first-order right-hand side directly. The original Newmark and Zhai formulas exploit second-order mechanical structure. Before either is applied to a vehicle model containing manifold configurations and first-order internal variables, its extended discrete equations must be defined explicitly; otherwise the method's name no longer describes the algorithm actually being used.
